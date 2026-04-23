#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <string>

namespace payload_sync_equivariant_observer_verified_nodelet {

class PayloadSyncEquivariantObserverVerifiedNodelet : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PayloadSyncEquivariantObserverVerifiedNodelet(
      const rclcpp::NodeOptions &options);

private:
  template <typename T>
  void declareAndReadParam(const std::string &name, T &value, const char *fmt);

  // ROS callbacks.
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void payloadOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void trpyCallback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);
  void tensionCallback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void publishTimerCallback();

  // High-rate observer step.
  void processImu(const sensor_msgs::msg::Imu &imu_msg,
                  const nav_msgs::msg::Odometry::SharedPtr &odom,
                  const quadrotor_msgs::msg::TRPYCommand::SharedPtr &trpy,
                  const sensor_msgs::msg::FluidPressure::SharedPtr &tension,
                  const nav_msgs::msg::Odometry::SharedPtr &payload_odom);

  // Observer core. Hold observer_mutex_ when calling these.
  void initializeObserverLocked(const Eigen::Vector3d &b0_body,
                                const Eigen::Vector3d &a_b,
                                const Eigen::Vector3d &zf_b, bool have_tension,
                                double tension_newton, double stamp_sec);
  void propagateObserverLocked(double dt, const Eigen::Vector3d &omega_b,
                               const Eigen::Vector3d &a_b,
                               const Eigen::Vector3d &zf_b, bool have_direction,
                               bool have_tension, double tension_newton);
  void normalizeObserverStateLocked();
  void cacheDebugValuesLocked(const Eigen::Matrix3d &R_wb,
                              const Eigen::Vector3d &a_b, double thrust_newton,
                              const Eigen::Vector3d &zf_b,
                              const Eigen::Vector3d &raw_dir_body,
                              double force_norm_body, bool have_tension,
                              double tension_newton);

  // Helper accessors. Hold observer_mutex_.
  Eigen::Vector3d bBarHatLocked() const;
  Eigen::Vector3d bBodyHatLocked() const;
  Eigen::Vector3d nWorldHatLocked(const Eigen::Matrix3d &R_wb) const;
  Eigen::Vector3d qWorldHatLocked(const Eigen::Matrix3d &R_wb) const;

  // Math helpers.
  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d &q_wxyz);
  static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
  static Eigen::Matrix3d expSO3(const Eigen::Vector3d &w);
  static Eigen::Matrix3d projectToSO3(const Eigen::Matrix3d &R);
  static Eigen::Matrix3d liftFromDirection(const Eigen::Vector3d &b);
  static double stampToSec(const builtin_interfaces::msg::Time &stamp);
  static bool finiteVec(const Eigen::Vector3d &v);

  Eigen::Matrix3d odomRotation(const nav_msgs::msg::Odometry &odom) const;
  Eigen::Vector3d odomPositionWorld(const nav_msgs::msg::Odometry &odom) const;
  Eigen::Vector3d odomVelocityWorld(const nav_msgs::msg::Odometry &odom,
                                    const Eigen::Matrix3d &R_wb) const;

  double tensionToNewton(const sensor_msgs::msg::FluidPressure &msg) const;
  double
  tensionVarianceNewton2(const sensor_msgs::msg::FluidPressure &msg) const;
  double thrustToNewton(const quadrotor_msgs::msg::TRPYCommand &msg) const;

  void publishFloatVector(
      const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub,
      const std::initializer_list<double> &values) const;

  // Parameters.
  double mass_{1.24};
  double payload_mass_{0.20};
  double gravity_{9.81};
  double cable_length_{0.76};

  double thrust_scale_{1.0};
  double thrust_offset_{0.0};
  double tension_scale_{1.0};
  double tension_offset_{0.0};

  double tau_min_{0.10};
  double force_min_{0.20};
  double tension_timeout_{0.05};
  double odom_timeout_{0.10};
  double max_prediction_dt_{0.005};
  double reset_dt_{0.25};
  double publish_rate_{100.0};

  double drag_x_{0.0};
  double drag_y_{0.0};
  double drag_z_{0.0};

  bool odom_twist_in_body_{false};
  bool use_direction_update_{true};
  bool use_tension_update_{true};

  // Observer gains.
  double k_b_{8.0};
  double k_nu_force_{12.0};
  double gamma_tau_{2.0};
  double gamma_f_{3.0};

  // Safety / saturation.
  double max_bias_force_{20.0};
  double max_bias_tension_{20.0};
  double max_nu_norm_{30.0};
  double init_force_bias_scale_{1.0};

  std::string frame_id_{"world"};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Zero()};

  // Cached measurements.
  mutable std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_payload_odom_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  quadrotor_msgs::msg::TRPYCommand::SharedPtr last_trpy_;
  sensor_msgs::msg::FluidPressure::SharedPtr last_tension_;

  // Observer state.
  mutable std::mutex observer_mutex_;
  Eigen::Matrix3d Z_sync_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d Y_dir_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d nu_bar_hat_{Eigen::Vector3d::Zero()};
  double b_tau_hat_{0.0};
  Eigen::Vector3d b_f_bar_hat_{Eigen::Vector3d::Zero()};

  bool observer_initialized_{false};
  double last_observer_time_{-1.0};

  // Fixed-rate publishing cache.
  Eigen::Vector3d last_force_inertial_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_thrust_inertial_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_force_residual_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_force_residual_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_raw_direction_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_raw_direction_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_a_body_{Eigen::Vector3d::Zero()};
  double last_force_norm_body_{0.0};
  double last_tension_newton_{0.0};
  bool last_have_tension_{false};

  // ROS interfaces.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_payload_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr sub_tension_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_force_inertial_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_thrust_inertial_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_cable_direction_raw_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_cable_direction_observer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_cable_direction_geom_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_observer_debug_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pub_payload_est_point_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_payload_est_odom_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace payload_sync_equivariant_observer_verified_nodelet
