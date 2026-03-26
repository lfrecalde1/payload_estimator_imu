#include "payload_estimator/nodelet.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <eigen3/Eigen/Geometry>
#include <type_traits>

namespace payload_estimator_nodelet {

PayloadEstimatorNodelet::PayloadEstimatorNodelet(
    const rclcpp::NodeOptions &options)
    : Node("payload_estimator_nodelet", options) {

  mass_ = 1.05;
  gravity_ = 9.81;
  inertia_ = Eigen::Matrix3d::Zero();
  inertia_(0, 0) = 0.00345398;
  inertia_(1, 1) = 0.00179687;
  inertia_(2, 2) = 0.0017967;

  declareAndReadParam("mass", mass_, "%.6f");
  declareAndReadParam("gravity", gravity_, "%.6f");
  declareAndReadParam("ixx", inertia_(0, 0), "%.6f");
  declareAndReadParam("iyy", inertia_(1, 1), "%.6f");
  declareAndReadParam("izz", inertia_(2, 2), "%.6f");
  declareAndReadParam("frame_id", frame_id_, "%s");

  RCLCPP_INFO(this->get_logger(),
              "[payload_estimator] inertia diag: [%.6f, %.6f, %.6f]",
              inertia_(0, 0), inertia_(1, 1), inertia_(2, 2));

  const auto qos = rclcpp::SensorDataQoS();

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/quadrotor/odom", qos,
      std::bind(&PayloadEstimatorNodelet::odomCallback, this,
                std::placeholders::_1));
  sub_payload_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/quadrotor/payload/odom", qos,
      std::bind(&PayloadEstimatorNodelet::payloadOdomCallback, this,
                std::placeholders::_1));

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/quadrotor/imu", qos,
      std::bind(&PayloadEstimatorNodelet::imuCallback, this,
                std::placeholders::_1));

  sub_trpy_ = this->create_subscription<quadrotor_msgs::msg::TRPYCommand>(
      "/quadrotor/trpy_cmd", qos,
      std::bind(&PayloadEstimatorNodelet::trpyCallback, this,
                std::placeholders::_1));

  pub_force_inertial_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("force_inertial",
                                                               10);
  pub_thrust_inertial_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "thrust_inertial", 10);
  pub_cable_direction_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "cable_direction", 10);
  pub_cable_direction_geom_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "cable_direction_geom", 10);

  RCLCPP_INFO(this->get_logger(),
              "[payload_estimator] Subscribed to topics: 'odom', 'imu', "
              "'trpy_cmd'");
}

template <typename T>
void PayloadEstimatorNodelet::declareAndReadParam(const std::string &name,
                                                  T &value, const char *fmt) {
  this->declare_parameter<T>(name, value);
  if (!this->get_parameter(name, value)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[payload_estimator] Failed to read parameter: %s",
                 name.c_str());
    return;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    RCLCPP_INFO(this->get_logger(), "[payload_estimator] %s: %s", name.c_str(),
                value.c_str());
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        (std::string("[payload_estimator] ") + name + ": " + fmt).c_str(),
        value);
  }
}

void PayloadEstimatorNodelet::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_odom_ = msg;
  }
  tryPublishEstimates();
}

void PayloadEstimatorNodelet::payloadOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_payload_odom_ = msg;
  }
  tryPublishEstimates();
}

void PayloadEstimatorNodelet::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_imu_ = msg;
  }
  tryPublishEstimates();
}

void PayloadEstimatorNodelet::trpyCallback(
    const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_trpy_ = msg;
  }
  tryPublishEstimates();
}

void PayloadEstimatorNodelet::tryPublishEstimates() {
  nav_msgs::msg::Odometry::SharedPtr odom;
  nav_msgs::msg::Odometry::SharedPtr payload_odom;
  sensor_msgs::msg::Imu::SharedPtr imu;
  quadrotor_msgs::msg::TRPYCommand::SharedPtr trpy;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom = last_odom_;
    payload_odom = last_payload_odom_;
    imu = last_imu_;
    trpy = last_trpy_;
  }

  if (!odom || !imu || !trpy) {
    return;
  }

  Eigen::Vector4d q_wxyz;
  q_wxyz << odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z;
  const Eigen::Matrix3d R = quatToRot(q_wxyz);

  Eigen::Vector3d a_body;
  a_body << imu->linear_acceleration.x, imu->linear_acceleration.y,
      imu->linear_acceleration.z;
  const Eigen::Vector3d a_inertial = R * a_body;
  const Eigen::Vector3d gravity_inertial(0.0, 0.0, gravity_);
  const Eigen::Vector3d a_inertial_no_g = a_inertial - gravity_inertial;
  const Eigen::Vector3d force_inertial = mass_ * a_inertial_no_g;

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  const Eigen::Vector3d thrust_inertial = trpy->thrust * (R * e3);

  std_msgs::msg::Float64MultiArray force_msg;
  force_msg.data = {force_inertial.x(), force_inertial.y(), force_inertial.z()};
  pub_force_inertial_->publish(force_msg);

  std_msgs::msg::Float64MultiArray thrust_msg;
  thrust_msg.data = {thrust_inertial.x(), thrust_inertial.y(),
                     thrust_inertial.z()};
  pub_thrust_inertial_->publish(thrust_msg);

  const Eigen::Vector3d force_diff = force_inertial - thrust_inertial;
  const double diff_norm = force_diff.norm();
  Eigen::Vector3d cable_direction = Eigen::Vector3d::Zero();
  if (diff_norm > 1e-9) {
    cable_direction = force_diff / diff_norm;
  }

  std_msgs::msg::Float64MultiArray cable_msg;
  cable_msg.data = {cable_direction.x(), cable_direction.y(),
                    cable_direction.z()};
  pub_cable_direction_->publish(cable_msg);

  if (payload_odom) {
    Eigen::Vector3d p_q, p_payload;
    p_q << odom->pose.pose.position.x, odom->pose.pose.position.y,
        odom->pose.pose.position.z;
    p_payload << payload_odom->pose.pose.position.x,
        payload_odom->pose.pose.position.y, payload_odom->pose.pose.position.z;

    const Eigen::Vector3d delta = p_payload - p_q;
    const double delta_norm = delta.norm();
    Eigen::Vector3d cable_direction_geom = Eigen::Vector3d::Zero();
    if (delta_norm > 1e-9) {
      cable_direction_geom = delta / 0.76;
    }

    std_msgs::msg::Float64MultiArray cable_geom_msg;
    cable_geom_msg.data = {cable_direction_geom.x(), cable_direction_geom.y(),
                           cable_direction_geom.z()};
    pub_cable_direction_geom_->publish(cable_geom_msg);
  }
}

Eigen::Matrix3d
PayloadEstimatorNodelet::quatToRot(const Eigen::Vector4d &q_wxyz) {
  Eigen::Quaterniond eq(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3));
  return eq.toRotationMatrix();
}

} // namespace payload_estimator_nodelet

RCLCPP_COMPONENTS_REGISTER_NODE(
    payload_estimator_nodelet::PayloadEstimatorNodelet)
