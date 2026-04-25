#include "payload_estimator_imu/nodelet.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>

namespace payload_estimator_imu_nodelet {

PayloadEstimatorNodelet::PayloadEstimatorNodelet(
    const rclcpp::NodeOptions &options)
    : Node("payload_estimator_imu_nodelet", options) {
  mass_ = 1.24;
  payload_mass_ = 0.20;
  gravity_ = 9.81;
  cable_length_ = 0.85;
  inertia_.setZero();
  inertia_(0, 0) = 0.00360915;
  inertia_(1, 1) = 0.00188875;
  inertia_(2, 2) = 0.00188864;

  declareAndReadParam("mass", mass_, "%.6f");
  declareAndReadParam("payload_mass", payload_mass_, "%.6f");

  declareAndReadParam("gravity", gravity_, "%.6f");
  declareAndReadParam("cable_length", cable_length_, "%.6f");

  declareAndReadParam("tau_min", tau_min_, "%.6f");
  declareAndReadParam("force_min", force_min_, "%.6f");

  declareAndReadParam("tension_timeout", tension_timeout_, "%.6f");

  declareAndReadParam("odom_timeout", odom_timeout_, "%.6f");

  declareAndReadParam("max_prediction_dt", max_prediction_dt_, "%.6f");

  declareAndReadParam("reset_dt", reset_dt_, "%.6f");
  declareAndReadParam("publish_rate", publish_rate_, "%.6f");

  declareAndReadParam("use_force_update", use_force_update_, "%d");
  declareAndReadParam("use_tension_update", use_tension_update_, "%d");

  declareAndReadParam("q_n", q_n_, "%.8f");
  declareAndReadParam("q_q", q_q_, "%.8f");
  declareAndReadParam("q_b_tau", q_b_tau_, "%.8f");
  declareAndReadParam("q_b_force", q_b_force_, "%.8f");

  declareAndReadParam("r_force_perp", r_force_perp_, "%.6f");
  declareAndReadParam("r_force_parallel", r_force_parallel_, "%.6f");

  declareAndReadParam("r_tension", r_tension_, "%.6f");

  declareAndReadParam("p0_n", p0_n_, "%.6f");
  declareAndReadParam("p0_q", p0_q_, "%.6f");
  declareAndReadParam("p0_b_tau", p0_b_tau_, "%.6f");
  declareAndReadParam("p0_b_force", p0_b_force_, "%.6f");

  declareAndReadParam("ixx", inertia_(0, 0), "%.6f");
  declareAndReadParam("iyy", inertia_(1, 1), "%.6f");
  declareAndReadParam("izz", inertia_(2, 2), "%.6f");

  declareAndReadParam("frame_id", frame_id_, "%s");

  const auto qos = rclcpp::SensorDataQoS();

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/eagle11/odom", qos,
      std::bind(&PayloadEstimatorNodelet::odomCallback, this,
                std::placeholders::_1));

  sub_payload_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/eagle11/payload/odom", qos,
      std::bind(&PayloadEstimatorNodelet::payloadOdomCallback, this,
                std::placeholders::_1));

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/eagle11/imu", qos,
      std::bind(&PayloadEstimatorNodelet::imuCallback, this,
                std::placeholders::_1));

  sub_trpy_ = this->create_subscription<quadrotor_msgs::msg::TRPYCommand>(
      "/eagle11/trpy_cmd", qos,
      std::bind(&PayloadEstimatorNodelet::trpyCallback, this,
                std::placeholders::_1));

  sub_tension_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "/quadrotor/rope0/tension", qos,
      std::bind(&PayloadEstimatorNodelet::tensionCallback, this,
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

  pub_cable_direction_ekf_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "cable_direction_ekf", 10);

  pub_cable_direction_geom_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "cable_direction_geom", 10);

  pub_payload_est_point_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "payload_estimated_point", 10);

  pub_payload_est_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "payload_estimated_odom", 10);

  const double period_s = 1.0 / std::max(1.0, publish_rate_);
  publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&PayloadEstimatorNodelet::publishTimerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "[payload_estimator_imu] IMU-driven reduced EKF enabled at %.2f "
              "Hz publish rate.",
              publish_rate_);
}

// --------------------------------------------------------------------------
// ROS parameter helper
// --------------------------------------------------------------------------
template <typename T>
void PayloadEstimatorNodelet::declareAndReadParam(const std::string &name,
                                                  T &value, const char *fmt) {
  this->declare_parameter<T>(name, value);
  if (!this->get_parameter(name, value)) {
    RCLCPP_ERROR(this->get_logger(),
                 "[payload_estimator_imu] failed to read parameter: %s",
                 name.c_str());
    return;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    RCLCPP_INFO(this->get_logger(), "[payload_estimator_imu] %s: %s",
                name.c_str(), value.c_str());
  } else if constexpr (std::is_same_v<T, bool>) {
    RCLCPP_INFO(this->get_logger(), "[payload_estimator_imu] %s: %s",
                name.c_str(), value ? "true" : "false");
  } else {
    RCLCPP_INFO(
        this->get_logger(),
        (std::string("[payload_estimator_imu] ") + name + ": " + fmt).c_str(),
        value);
  }
}

// --------------------------------------------------------------------------
// Callbacks
// --------------------------------------------------------------------------
void PayloadEstimatorNodelet::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_odom_ = msg;
}

void PayloadEstimatorNodelet::payloadOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_payload_odom_ = msg;
}

void PayloadEstimatorNodelet::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  nav_msgs::msg::Odometry::SharedPtr odom;
  nav_msgs::msg::Odometry::SharedPtr payload_odom;
  quadrotor_msgs::msg::TRPYCommand::SharedPtr trpy;
  sensor_msgs::msg::FluidPressure::SharedPtr tension;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_imu_ = msg;
    odom = last_odom_;
    payload_odom = last_payload_odom_;
    trpy = last_trpy_;
    tension = last_tension_;
  }

  processImu(*msg, odom, trpy, tension, payload_odom);
}

void PayloadEstimatorNodelet::trpyCallback(
    const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_trpy_ = msg;
}

void PayloadEstimatorNodelet::tensionCallback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_tension_ = msg;
}

// --------------------------------------------------------------------------
// High-rate IMU-driven propagation and correction
// --------------------------------------------------------------------------
void PayloadEstimatorNodelet::processImu(
    const sensor_msgs::msg::Imu &imu_msg,
    const nav_msgs::msg::Odometry::SharedPtr &odom,
    const quadrotor_msgs::msg::TRPYCommand::SharedPtr &trpy,
    const sensor_msgs::msg::FluidPressure::SharedPtr &tension,
    const nav_msgs::msg::Odometry::SharedPtr &payload_odom) {

  if (!odom || !trpy) {
    return;
  }

  const double t_imu = stampToSec(imu_msg.header.stamp);
  if (t_imu <= 0.0) {
    return;
  }

  const double t_odom = stampToSec(odom->header.stamp);
  if (t_odom <= 0.0 || std::fabs(t_imu - t_odom) > odom_timeout_) {
    return;
  }

  const Eigen::Matrix3d R = odomRotation(*odom);
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  const Eigen::Vector3d p_q = odomPositionWorld(*odom);

  Eigen::Vector3d f_b;
  f_b << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z;
  const Eigen::Vector3d A_world = R * f_b; // A = a_Q + g e3

  const double thrust_newton = thrustToNewton(*trpy);
  const Eigen::Vector3d thrust_inertial = thrust_newton * (R * e3);
  const Eigen::Vector3d force_inertial = mass_ * A_world;

  const Eigen::Vector3d force_residual = force_inertial - thrust_inertial;
  const double force_norm = force_residual.norm();

  Eigen::Vector3d raw_cable_direction = Eigen::Vector3d::Zero();
  if (force_norm > 1e-9) {
    raw_cable_direction = force_residual / force_norm;
  }

  bool have_tension = false;
  double tension_newton = 0.0;
  double tension_var = r_tension_ * r_tension_;
  double tension_stamp = -1.0;
  if (tension) {
    tension_stamp = stampToSec(tension->header.stamp);
    tension_newton = tensionToNewton(*tension);
    tension_var = tensionVarianceNewton2(*tension);
    have_tension = tension_stamp > 0.0 &&
                   std::fabs(t_imu - tension_stamp) <= tension_timeout_ &&
                   std::isfinite(tension_newton);
  }

  std::lock_guard<std::mutex> lock(filter_mutex_);

  // Cache debug values for the timer thread.
  last_A_world_ = A_world;
  last_force_inertial_ = force_inertial;
  last_thrust_inertial_ = thrust_inertial;
  last_force_residual_ = force_residual;
  last_raw_cable_direction_ = raw_cable_direction;
  last_force_norm_ = force_norm;
  last_tension_newton_ = tension_newton;
  last_have_tension_ = have_tension;

  if (!filter_initialized_ || last_filter_time_ < 0.0 ||
      std::fabs(t_imu - last_filter_time_) > reset_dt_) {
    Eigen::Vector3d n0(0.0, 0.0, -1.0);
    if (have_tension && tension_newton > tau_min_ && force_norm > force_min_) {
      n0 = raw_cable_direction;
    }
    initializeFilter(n0, A_world, force_residual, have_tension, tension_newton,
                     t_imu);
  }

  const double dt = t_imu - last_filter_time_;
  if (dt > 0.0) {
    const int n_steps =
        std::max(1, static_cast<int>(std::ceil(dt / max_prediction_dt_)));
    const double h = dt / static_cast<double>(n_steps);
    for (int i = 0; i < n_steps; ++i) {
      predictFilter(h, A_world);
    }
    last_filter_time_ = t_imu;
  }

  const bool valid_force = force_norm > force_min_;
  const bool taut_by_tension = have_tension && tension_newton > tau_min_;

  if (use_force_update_ && valid_force && taut_by_tension &&
      t_imu > last_force_update_time_ + 1e-9) {
    if (updateForceResidual(force_residual, A_world, tension_newton)) {
      last_force_update_time_ = t_imu;
      RCLCPP_INFO(this->get_logger(),
                  "[payload_estimator_imu] Using Force Error");
    }
  }

  if (use_tension_update_ && taut_by_tension &&
      tension_stamp > last_tension_update_time_ + 1e-9) {
    if (updateTension(tension_newton, tension_var, A_world)) {
      last_tension_update_time_ = tension_stamp;
      RCLCPP_INFO(this->get_logger(),
                  "[payload_estimator_imu] Using Tension Error");
    }
  }

  normalizeFilterState();
  symmetrizeCovariance();
}

// --------------------------------------------------------------------------
// Fixed-rate publisher
// --------------------------------------------------------------------------
void PayloadEstimatorNodelet::publishTimerCallback() {
  nav_msgs::msg::Odometry::SharedPtr odom;
  nav_msgs::msg::Odometry::SharedPtr payload_odom;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom = last_odom_;
    payload_odom = last_payload_odom_;
  }

  if (!odom) {
    return;
  }

  Eigen::Matrix<double, 10, 1> x;
  Eigen::Matrix<double, 10, 10> P;
  bool initialized = false;
  double filter_time = -1.0;
  Eigen::Vector3d A_world, force_inertial, thrust_inertial, force_residual;
  Eigen::Vector3d raw_cable_direction;
  double force_norm = 0.0;
  double tension_newton = 0.0;
  bool have_tension = false;

  {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    x = x_;
    P = P_;
    initialized = filter_initialized_;
    filter_time = last_filter_time_;
    A_world = last_A_world_;
    force_inertial = last_force_inertial_;
    thrust_inertial = last_thrust_inertial_;
    force_residual = last_force_residual_;
    raw_cable_direction = last_raw_cable_direction_;
    force_norm = last_force_norm_;
    tension_newton = last_tension_newton_;
    have_tension = last_have_tension_;
  }

  if (!initialized) {
    return;
  }

  const Eigen::Matrix3d R = odomRotation(*odom);
  const Eigen::Vector3d p_q = odomPositionWorld(*odom);
  const Eigen::Vector3d v_q = odomVelocityWorld(*odom, R);

  Eigen::Vector3d n_hat = x.segment<3>(0);
  if (n_hat.norm() > 1e-9) {
    n_hat.normalize();
  } else {
    n_hat = Eigen::Vector3d(0.0, 0.0, -1.0);
  }

  Eigen::Vector3d q_hat = x.segment<3>(3);
  q_hat = (Eigen::Matrix3d::Identity() - n_hat * n_hat.transpose()) * q_hat;

  const double b_tau_hat = x(6);
  const Eigen::Vector3d b_force_hat = x.segment<3>(7);
  const double tau_model =
      payload_mass_ *
      (cable_length_ * q_hat.squaredNorm() - n_hat.dot(A_world));
  const double tau_sensor_pred = tau_model + b_tau_hat;

  const Eigen::Vector3d p_payload_est = p_q + cable_length_ * n_hat;
  const Eigen::Vector3d v_payload_est = v_q + cable_length_ * q_hat;

  publishFloatVector(
      pub_force_inertial_,
      {force_inertial.x(), force_inertial.y(), force_inertial.z()});

  publishFloatVector(
      pub_thrust_inertial_,
      {thrust_inertial.x(), thrust_inertial.y(), thrust_inertial.z()});

  publishFloatVector(pub_cable_direction_,
                     {raw_cable_direction.x(), raw_cable_direction.y(),
                      raw_cable_direction.z(), raw_cable_direction.norm(),
                      force_norm});

  publishFloatVector(pub_cable_direction_ekf_,
                     {n_hat.x(), n_hat.y(), n_hat.z(), n_hat.norm(), q_hat.x(),
                      q_hat.y(), q_hat.z(), q_hat.norm()});

  builtin_interfaces::msg::Time stamp = odom->header.stamp;
  if (filter_time > 0.0) {
    rclcpp::Time t(static_cast<int64_t>(filter_time * 1e9));
    stamp = t;
  }

  geometry_msgs::msg::PointStamped payload_point_msg;
  payload_point_msg.header.stamp = stamp;
  payload_point_msg.header.frame_id = frame_id_;
  payload_point_msg.point.x = p_payload_est.x();
  payload_point_msg.point.y = p_payload_est.y();
  payload_point_msg.point.z = p_payload_est.z();
  pub_payload_est_point_->publish(payload_point_msg);

  nav_msgs::msg::Odometry payload_odom_msg;
  payload_odom_msg.header.stamp = stamp;
  payload_odom_msg.header.frame_id = frame_id_;
  payload_odom_msg.child_frame_id = "payload_estimated";
  payload_odom_msg.pose.pose.position.x = p_payload_est.x();
  payload_odom_msg.pose.pose.position.y = p_payload_est.y();
  payload_odom_msg.pose.pose.position.z = p_payload_est.z();
  payload_odom_msg.pose.pose.orientation.w = 1.0;
  payload_odom_msg.pose.pose.orientation.x = 0.0;
  payload_odom_msg.pose.pose.orientation.y = 0.0;
  payload_odom_msg.pose.pose.orientation.z = 0.0;
  payload_odom_msg.twist.twist.linear.x = v_payload_est.x();
  payload_odom_msg.twist.twist.linear.y = v_payload_est.y();
  payload_odom_msg.twist.twist.linear.z = v_payload_est.z();
  pub_payload_est_odom_->publish(payload_odom_msg);

  if (payload_odom) {
    const Eigen::Vector3d p_payload = odomPositionWorld(*payload_odom);
    const Eigen::Vector3d delta = p_payload - p_q;
    const double delta_norm = delta.norm();
    Eigen::Vector3d cable_direction_geom = Eigen::Vector3d::Zero();
    if (delta_norm > 1e-9) {
      cable_direction_geom = delta / delta_norm;
    }
    publishFloatVector(pub_cable_direction_geom_,
                       {cable_direction_geom.x(), cable_direction_geom.y(),
                        cable_direction_geom.z(), cable_direction_geom.norm()});
  }
}

// --------------------------------------------------------------------------
// EKF core
// --------------------------------------------------------------------------
void PayloadEstimatorNodelet::initializeFilter(
    const Eigen::Vector3d &n0_in, const Eigen::Vector3d &A_world,
    const Eigen::Vector3d &force_residual, bool have_tension,
    double tension_newton, double stamp_sec) {
  Eigen::Vector3d n0 = n0_in;
  if (!finiteVec(n0) || n0.norm() < 1e-6) {
    n0 = Eigen::Vector3d(0.0, 0.0, -1.0);
  }
  n0.normalize();

  x_.setZero();
  x_.segment<3>(0) = n0;
  x_.segment<3>(3).setZero();

  const double tau_model0 = payload_mass_ * (-n0.dot(A_world));
  x_(6) = (have_tension && std::isfinite(tension_newton))
              ? (tension_newton - tau_model0)
              : 0.0;

  if (have_tension && std::isfinite(tension_newton) &&
      tension_newton > tau_min_) {
    x_.segment<3>(7) = force_residual - tension_newton * n0;
  } else {
    x_.segment<3>(7).setZero();
  }

  P_.setZero();
  P_.block<3, 3>(0, 0) = (p0_n_ * p0_n_) * Eigen::Matrix3d::Identity();
  P_.block<3, 3>(3, 3) = (p0_q_ * p0_q_) * Eigen::Matrix3d::Identity();
  P_(6, 6) = p0_b_tau_ * p0_b_tau_;
  P_.block<3, 3>(7, 7) =
      (p0_b_force_ * p0_b_force_) * Eigen::Matrix3d::Identity();

  filter_initialized_ = true;
  last_filter_time_ = stamp_sec;
  last_force_update_time_ = -1.0;
  last_tension_update_time_ = -1.0;
  last_payload_odom_update_time_ = -1.0;

  normalizeFilterState();
  symmetrizeCovariance();
}

void PayloadEstimatorNodelet::predictFilter(double dt,
                                            const Eigen::Vector3d &A_world) {
  if (!filter_initialized_ || dt <= 0.0 || !finiteVec(A_world)) {
    return;
  }

  Eigen::Vector3d n = stateN();
  Eigen::Vector3d q = stateQ();

  auto f = [&](const Eigen::Vector3d &ni, const Eigen::Vector3d &qi,
               Eigen::Vector3d &ndot, Eigen::Vector3d &qdot) {
    Eigen::Vector3d nn = ni;
    if (nn.norm() > 1e-9) {
      nn.normalize();
    }
    Eigen::Vector3d qq =
        (Eigen::Matrix3d::Identity() - nn * nn.transpose()) * qi;
    ndot = qq;
    qdot = (1.0 / cable_length_) *
               ((nn * nn.transpose() - Eigen::Matrix3d::Identity()) * A_world) -
           qq.squaredNorm() * nn;
  };

  Eigen::Vector3d k1n, k1q, k2n, k2q, k3n, k3q, k4n, k4q;
  f(n, q, k1n, k1q);
  f(n + 0.5 * dt * k1n, q + 0.5 * dt * k1q, k2n, k2q);
  f(n + 0.5 * dt * k2n, q + 0.5 * dt * k2q, k3n, k3q);
  f(n + dt * k3n, q + dt * k3q, k4n, k4q);

  x_.segment<3>(0) = n + (dt / 6.0) * (k1n + 2.0 * k2n + 2.0 * k3n + k4n);
  x_.segment<3>(3) = q + (dt / 6.0) * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
  normalizeFilterState();

  n = stateN();
  q = stateQ();
  const double alpha = n.dot(A_world);
  const double beta = q.squaredNorm();

  Eigen::Matrix<double, 10, 10> F = Eigen::Matrix<double, 10, 10>::Zero();
  F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
  F.block<3, 3>(3, 0) =
      (1.0 / cable_length_) *
          (n * A_world.transpose() + alpha * Eigen::Matrix3d::Identity()) -
      beta * Eigen::Matrix3d::Identity();
  F.block<3, 3>(3, 3) = -2.0 * n * q.transpose();

  const Eigen::Matrix<double, 10, 10> Phi =
      Eigen::Matrix<double, 10, 10>::Identity() + F * dt;

  Eigen::Matrix<double, 10, 10> Q = Eigen::Matrix<double, 10, 10>::Zero();
  Q.block<3, 3>(0, 0) = q_n_ * dt * Eigen::Matrix3d::Identity();
  Q.block<3, 3>(3, 3) = q_q_ * dt * Eigen::Matrix3d::Identity();
  Q(6, 6) = q_b_tau_ * dt;
  Q.block<3, 3>(7, 7) = q_b_force_ * dt * Eigen::Matrix3d::Identity();

  P_ = Phi * P_ * Phi.transpose() + Q;
  symmetrizeCovariance();
}

bool PayloadEstimatorNodelet::updateForceResidual(
    const Eigen::Vector3d &z_force, const Eigen::Vector3d &A_world,
    double tension_newton_for_cov) {
  if (!filter_initialized_ || !finiteVec(z_force) || !finiteVec(A_world)) {
    return false;
  }

  const Eigen::Vector3d n = stateN();
  const Eigen::Vector3d q = stateQ();
  const Eigen::Vector3d b_f = stateBForce();

  const double tau =
      payload_mass_ * (cable_length_ * q.squaredNorm() - n.dot(A_world));
  const Eigen::Vector3d h = tau * n + b_f;
  const Eigen::Vector3d r = z_force - h;

  Eigen::Matrix<double, 3, 10> H = Eigen::Matrix<double, 3, 10>::Zero();
  H.block<3, 3>(0, 0) = tau * Eigen::Matrix3d::Identity() -
                        payload_mass_ * n * A_world.transpose();
  H.block<3, 3>(0, 3) = 2.0 * payload_mass_ * cable_length_ * n * q.transpose();
  H.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();

  const Eigen::Matrix3d Pn = n * n.transpose();
  const Eigen::Matrix3d Pt = Eigen::Matrix3d::Identity() - Pn;

  double scale = 1.0;

  Eigen::Matrix3d Rm = (r_force_perp_ * r_force_perp_ * scale) * Pt +
                       (r_force_parallel_ * r_force_parallel_) * Pn;
  Rm += 1e-9 * Eigen::Matrix3d::Identity();

  const Eigen::Matrix3d S = H * P_ * H.transpose() + Rm;

  const Eigen::Matrix<double, 10, 3> K =
      P_ * H.transpose() * S.ldlt().solve(Eigen::Matrix3d::Identity());
  const Eigen::Matrix<double, 10, 1> dx = K * r;

  x_ += dx;
  const Eigen::Matrix<double, 10, 10> I =
      Eigen::Matrix<double, 10, 10>::Identity();
  P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * Rm * K.transpose();

  normalizeFilterState();
  symmetrizeCovariance();
  return true;
}

bool PayloadEstimatorNodelet::updateTension(double z_tension, double z_variance,
                                            const Eigen::Vector3d &A_world) {
  if (!filter_initialized_ || !std::isfinite(z_tension) ||
      !finiteVec(A_world)) {
    return false;
  }

  const Eigen::Vector3d n = stateN();
  const Eigen::Vector3d q = stateQ();
  const double b_tau = stateBTau();

  const double tau =
      payload_mass_ * (cable_length_ * q.squaredNorm() - n.dot(A_world));
  const double h = tau + b_tau;
  const double r = z_tension - h;

  Eigen::Matrix<double, 1, 10> H = Eigen::Matrix<double, 1, 10>::Zero();
  H.block<1, 3>(0, 0) = -payload_mass_ * A_world.transpose();
  H.block<1, 3>(0, 3) = 2.0 * payload_mass_ * cable_length_ * q.transpose();
  H(0, 6) = 1.0;

  const double Rm = std::max(1e-9, z_variance);
  const double S = (H * P_ * H.transpose())(0, 0) + Rm;

  const Eigen::Matrix<double, 10, 1> K = P_ * H.transpose() / S;
  const Eigen::Matrix<double, 10, 1> dx = K * r;

  x_ += dx;
  const Eigen::Matrix<double, 10, 10> I =
      Eigen::Matrix<double, 10, 10>::Identity();
  P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * Rm * K.transpose();

  normalizeFilterState();
  symmetrizeCovariance();
  return true;
}

void PayloadEstimatorNodelet::normalizeFilterState() {
  Eigen::Vector3d n = x_.segment<3>(0);
  if (!finiteVec(n) || n.norm() < 1e-9) {
    n = Eigen::Vector3d(0.0, 0.0, -1.0);
  } else {
    n.normalize();
  }
  x_.segment<3>(0) = n;

  Eigen::Vector3d q = x_.segment<3>(3);
  if (!finiteVec(q)) {
    q.setZero();
  }
  q = (Eigen::Matrix3d::Identity() - n * n.transpose()) * q;
  x_.segment<3>(3) = q;

  if (!std::isfinite(x_(6))) {
    x_(6) = 0.0;
  }
  if (!finiteVec(x_.segment<3>(7))) {
    x_.segment<3>(7).setZero();
  }
}

void PayloadEstimatorNodelet::symmetrizeCovariance() {
  P_ = 0.5 * (P_ + P_.transpose());
  for (int i = 0; i < 10; ++i) {
    if (!std::isfinite(P_(i, i)) || P_(i, i) < 1e-12) {
      P_(i, i) = 1e-12;
    }
  }
}

// --------------------------------------------------------------------------
// Helpers
// --------------------------------------------------------------------------
Eigen::Vector3d PayloadEstimatorNodelet::stateN() const {
  return x_.segment<3>(0);
}

Eigen::Vector3d PayloadEstimatorNodelet::stateQ() const {
  return x_.segment<3>(3);
}

double PayloadEstimatorNodelet::stateBTau() const { return x_(6); }

Eigen::Vector3d PayloadEstimatorNodelet::stateBForce() const {
  return x_.segment<3>(7);
}

Eigen::Matrix3d
PayloadEstimatorNodelet::quatToRot(const Eigen::Vector4d &q_wxyz) {
  Eigen::Quaterniond q(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3));
  q.normalize();
  return q.toRotationMatrix();
}

Eigen::Matrix3d PayloadEstimatorNodelet::skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

double PayloadEstimatorNodelet::stampToSec(
    const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         1e-9 * static_cast<double>(stamp.nanosec);
}

bool PayloadEstimatorNodelet::finiteVec(const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

Eigen::Matrix3d PayloadEstimatorNodelet::odomRotation(
    const nav_msgs::msg::Odometry &odom) const {
  Eigen::Vector4d q_wxyz;
  q_wxyz << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z;
  return quatToRot(q_wxyz);
}

Eigen::Vector3d PayloadEstimatorNodelet::odomPositionWorld(
    const nav_msgs::msg::Odometry &odom) const {
  return Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                         odom.pose.pose.position.z);
}

Eigen::Vector3d
PayloadEstimatorNodelet::odomVelocityWorld(const nav_msgs::msg::Odometry &odom,
                                           const Eigen::Matrix3d &R) const {
  Eigen::Vector3d v(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z);
  return v;
}

double PayloadEstimatorNodelet::tensionToNewton(
    const sensor_msgs::msg::FluidPressure &msg) const {
  return msg.fluid_pressure;
}

double PayloadEstimatorNodelet::tensionVarianceNewton2(
    const sensor_msgs::msg::FluidPressure &msg) const {
  return std::max(1e-9, r_tension_ * r_tension_);
}

double PayloadEstimatorNodelet::thrustToNewton(
    const quadrotor_msgs::msg::TRPYCommand &msg) const {
  return msg.thrust;
}

void PayloadEstimatorNodelet::publishFloatVector(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub,
    const std::initializer_list<double> &values) const {
  if (!pub) {
    return;
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(values.begin(), values.end());
  pub->publish(msg);
}

} // namespace payload_estimator_imu_nodelet

RCLCPP_COMPONENTS_REGISTER_NODE(
    payload_estimator_imu_nodelet::PayloadEstimatorNodelet)
