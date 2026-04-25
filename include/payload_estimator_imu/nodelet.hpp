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

namespace payload_estimator_imu_nodelet {

class PayloadEstimatorNodelet : public rclcpp::Node {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit PayloadEstimatorNodelet(const rclcpp::NodeOptions &options);

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

  // High-rate IMU-driven estimator.
  void processImu(const sensor_msgs::msg::Imu &imu_msg,
                  const nav_msgs::msg::Odometry::SharedPtr &odom,
                  const quadrotor_msgs::msg::TRPYCommand::SharedPtr &trpy,
                  const sensor_msgs::msg::FluidPressure::SharedPtr &tension,
                  const nav_msgs::msg::Odometry::SharedPtr &payload_odom);

  // EKF core. Call only while holding filter_mutex_.
  void initializeFilter(const Eigen::Vector3d &n0,
                        const Eigen::Vector3d &A_world,
                        const Eigen::Vector3d &force_residual,
                        bool have_tension, double tension_newton,
                        double stamp_sec);

  void predictFilter(double dt, const Eigen::Vector3d &A_world);

  bool updateForceResidual(const Eigen::Vector3d &z_force,
                           const Eigen::Vector3d &A_world,
                           double tension_newton_for_cov);
  bool updateTension(double z_tension, double z_variance,
                     const Eigen::Vector3d &A_world);

  void normalizeFilterState();
  void symmetrizeCovariance();

  // Small helpers.
  Eigen::Vector3d stateN() const;
  Eigen::Vector3d stateQ() const;
  double stateBTau() const;
  Eigen::Vector3d stateBForce() const;

  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d &q_wxyz);
  static Eigen::Matrix3d skew(const Eigen::Vector3d &v);
  static double stampToSec(const builtin_interfaces::msg::Time &stamp);
  static bool finiteVec(const Eigen::Vector3d &v);

  Eigen::Matrix3d odomRotation(const nav_msgs::msg::Odometry &odom) const;
  Eigen::Vector3d odomPositionWorld(const nav_msgs::msg::Odometry &odom) const;
  Eigen::Vector3d odomVelocityWorld(const nav_msgs::msg::Odometry &odom,
                                    const Eigen::Matrix3d &R) const;

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
  double cable_length_{0.85};

  double tau_min_{0.10};
  double force_min_{0.20};

  double tension_timeout_{0.05};

  double odom_timeout_{0.10};

  double payload_odom_timeout_{0.10};

  double max_prediction_dt_{0.005};

  double reset_dt_{0.25};

  double x_opt_0_{1.918101009211921e-06};
  double x_opt_1_{-5.632268117969789e-07};
  double x_opt_2_{3.648328641889230e-06};
  double x_opt_3_{2.235703364573816e-06};

  double publish_rate_{100.0};

  bool use_force_update_{true};
  bool use_tension_update_{false};

  // Discrete process/measurement tuning.
  double q_n_{1e-5};

  double q_q_{5e-4};

  double q_b_tau_{1e-6};

  double q_b_force_{1e-4};

  double r_force_perp_{0.10};
  double r_force_parallel_{2.00};
  double r_tension_{0.10};

  // Initial covariance.
  double p0_n_{0.10};

  double p0_q_{1.00};

  double p0_b_tau_{0.50};

  double p0_b_force_{2.00};

  std::string frame_id_{"world"};
  Eigen::Matrix3d inertia_{Eigen::Matrix3d::Zero()};

  // Cached measurements.
  mutable std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_payload_odom_;
  sensor_msgs::msg::Imu::SharedPtr last_imu_;
  quadrotor_msgs::msg::TRPYCommand::SharedPtr last_trpy_;
  sensor_msgs::msg::FluidPressure::SharedPtr last_tension_;

  // Filter state and debug cache.
  mutable std::mutex filter_mutex_;
  Eigen::Matrix<double, 10, 1> x_{Eigen::Matrix<double, 10, 1>::Zero()};
  Eigen::Matrix<double, 10, 10> P_{Eigen::Matrix<double, 10, 10>::Identity()};

  bool filter_initialized_{false};
  double last_filter_time_{-1.0};
  double last_force_update_time_{-1.0};
  double last_tension_update_time_{-1.0};
  double last_payload_odom_update_time_{-1.0};

  Eigen::Vector3d last_A_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_force_inertial_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_thrust_inertial_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_force_residual_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_raw_cable_direction_{Eigen::Vector3d::Zero()};

  double last_force_norm_{0.0};
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
      pub_cable_direction_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_cable_direction_ekf_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_cable_direction_geom_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pub_payload_est_point_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_payload_est_odom_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

} // namespace payload_estimator_imu_nodelet
