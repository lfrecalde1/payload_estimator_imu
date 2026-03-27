#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <eigen3/Eigen/Dense>

#include <mutex>
#include <string>

namespace payload_estimator_nodelet {

class PayloadEstimatorNodelet : public rclcpp::Node {
public:
  explicit PayloadEstimatorNodelet(const rclcpp::NodeOptions &options);

private:
  template <typename T>
  void declareAndReadParam(const std::string &name, T &value, const char *fmt);

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void trpyCallback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);
  void tryPublishEstimates();
  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d & q_wxyz);

  // Parameters
  double mass_ = 1.05;
  double gravity_ = 9.81;
  Eigen::Matrix3d inertia_ = Eigen::Matrix3d::Zero();
  std::string frame_id_ = "world";

  // Latest measurements
  std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_payload_odom_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  quadrotor_msgs::msg::TRPYCommand::SharedPtr last_trpy_;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_payload_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_force_inertial_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_thrust_inertial_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cable_direction_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cable_direction_geom_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cable_direction_geom_lc_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_payload_est_point_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_payload_est_odom_;
};

} // namespace payload_estimator_nodelet
