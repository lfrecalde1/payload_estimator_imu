
#include "payload_estimator_imu/payload_equivariant_sync_observer_nodelet.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>

namespace payload_equivariant_sync_observer_nodelet {

PayloadEquivariantSyncObserverNodelet::PayloadEquivariantSyncObserverNodelet(
    const rclcpp::NodeOptions &options)
    : Node("payload_equivariant_sync_observer_nodelet", options) {
  mass_ = 1.24;
  payload_mass_ = 0.20;
  gravity_ = 9.81;
  cable_length_ = 0.76;
  inertia_.setZero();
  inertia_(0, 0) = 0.00360915;
  inertia_(1, 1) = 0.00188875;
  inertia_(2, 2) = 0.00188864;

  declareAndReadParam("mass", mass_, "%.6f");
  declareAndReadParam("payload_mass", payload_mass_, "%.6f");
  declareAndReadParam("gravity", gravity_, "%.6f");
  declareAndReadParam("cable_length", cable_length_, "%.6f");

  declareAndReadParam("thrust_scale", thrust_scale_, "%.6f");
  declareAndReadParam("thrust_offset", thrust_offset_, "%.6f");
  declareAndReadParam("tension_scale", tension_scale_, "%.6f");
  declareAndReadParam("tension_offset", tension_offset_, "%.6f");

  declareAndReadParam("tau_min", tau_min_, "%.6f");
  declareAndReadParam("force_min", force_min_, "%.6f");
  declareAndReadParam("tension_timeout", tension_timeout_, "%.6f");
  declareAndReadParam("odom_timeout", odom_timeout_, "%.6f");
  declareAndReadParam("max_prediction_dt", max_prediction_dt_, "%.6f");
  declareAndReadParam("reset_dt", reset_dt_, "%.6f");
  declareAndReadParam("publish_rate", publish_rate_, "%.6f");

  declareAndReadParam("drag_x", drag_x_, "%.6f");
  declareAndReadParam("drag_y", drag_y_, "%.6f");
  declareAndReadParam("drag_z", drag_z_, "%.6f");

  declareAndReadParam("odom_twist_in_body", odom_twist_in_body_, "%d");
  declareAndReadParam("use_direction_update", use_direction_update_, "%d");
  declareAndReadParam("use_tension_update", use_tension_update_, "%d");

  declareAndReadParam("k_b", k_b_, "%.6f");
  declareAndReadParam("k_nu_force", k_nu_force_, "%.6f");
  declareAndReadParam("gamma_tau", gamma_tau_, "%.6f");
  declareAndReadParam("gamma_f", gamma_f_, "%.6f");

  declareAndReadParam("max_bias_force", max_bias_force_, "%.6f");
  declareAndReadParam("max_bias_tension", max_bias_tension_, "%.6f");
  declareAndReadParam("max_nu_norm", max_nu_norm_, "%.6f");
  declareAndReadParam("init_bias_force_scale", init_bias_force_scale_, "%.6f");

  declareAndReadParam("ixx", inertia_(0, 0), "%.6f");
  declareAndReadParam("iyy", inertia_(1, 1), "%.6f");
  declareAndReadParam("izz", inertia_(2, 2), "%.6f");
  declareAndReadParam("frame_id", frame_id_, "%s");

  const auto qos = rclcpp::SensorDataQoS();

  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/quadrotor/odom", qos,
      std::bind(&PayloadEquivariantSyncObserverNodelet::odomCallback, this,
                std::placeholders::_1));
  sub_payload_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/quadrotor/payload/odom", qos,
      std::bind(&PayloadEquivariantSyncObserverNodelet::payloadOdomCallback,
                this, std::placeholders::_1));
  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/quadrotor/imu", qos,
      std::bind(&PayloadEquivariantSyncObserverNodelet::imuCallback, this,
                std::placeholders::_1));
  sub_trpy_ = this->create_subscription<quadrotor_msgs::msg::TRPYCommand>(
      "/quadrotor/trpy_cmd", qos,
      std::bind(&PayloadEquivariantSyncObserverNodelet::trpyCallback, this,
                std::placeholders::_1));
  sub_tension_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "/quadrotor/rope0/tension", qos,
      std::bind(&PayloadEquivariantSyncObserverNodelet::tensionCallback, this,
                std::placeholders::_1));

  pub_force_inertial_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/force_inertial", 10);
  pub_thrust_inertial_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/thrust_inertial", 10);
  pub_cable_direction_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/cable_direction_raw", 10);
  pub_cable_direction_observer_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/cable_direction_observer", 10);
  pub_cable_direction_geom_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/cable_direction_geom", 10);
  pub_observer_debug_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "sync_observer/debug", 10);
  pub_payload_est_point_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>(
          "sync_observer/payload_estimated_point", 10);
  pub_payload_est_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "sync_observer/payload_estimated_odom", 10);

  const double period_s = 1.0 / std::max(1.0, publish_rate_);
  publish_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&PayloadEquivariantSyncObserverNodelet::publishTimerCallback,
                this));

  RCLCPP_INFO(this->get_logger(),
              "[payload_equivariant_sync_observer] IMU-driven synchronous/"
              "equivariant observer enabled. Publish rate: %.2f Hz",
              publish_rate_);
}

template <typename T>
void PayloadEquivariantSyncObserverNodelet::declareAndReadParam(
    const std::string &name, T &value, const char *fmt) {
  this->declare_parameter<T>(name, value);
  if (!this->get_parameter(name, value)) {
    RCLCPP_ERROR(
        this->get_logger(),
        "[payload_equivariant_sync_observer] failed to read parameter: %s",
        name.c_str());
    return;
  }

  if constexpr (std::is_same_v<T, std::string>) {
    RCLCPP_INFO(this->get_logger(),
                "[payload_equivariant_sync_observer] %s: %s", name.c_str(),
                value.c_str());
  } else if constexpr (std::is_same_v<T, bool>) {
    RCLCPP_INFO(this->get_logger(),
                "[payload_equivariant_sync_observer] %s: %s", name.c_str(),
                value ? "true" : "false");
  } else {
    RCLCPP_INFO(this->get_logger(),
                (std::string("[payload_equivariant_sync_observer] ") + name +
                 ": " + fmt)
                    .c_str(),
                value);
  }
}

void PayloadEquivariantSyncObserverNodelet::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_odom_ = msg;
}

void PayloadEquivariantSyncObserverNodelet::payloadOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_payload_odom_ = msg;
}

void PayloadEquivariantSyncObserverNodelet::imuCallback(
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

void PayloadEquivariantSyncObserverNodelet::trpyCallback(
    const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_trpy_ = msg;
}

void PayloadEquivariantSyncObserverNodelet::tensionCallback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  last_tension_ = msg;
}

void PayloadEquivariantSyncObserverNodelet::processImu(
    const sensor_msgs::msg::Imu &imu_msg,
    const nav_msgs::msg::Odometry::SharedPtr &odom,
    const quadrotor_msgs::msg::TRPYCommand::SharedPtr &trpy,
    const sensor_msgs::msg::FluidPressure::SharedPtr &tension,
    const nav_msgs::msg::Odometry::SharedPtr &payload_odom) {

  (void)payload_odom;

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

  const Eigen::Matrix3d R_wb = odomRotation(*odom);
  const Eigen::Vector3d v_q_w = odomVelocityWorld(*odom, R_wb);
  const Eigen::Vector3d v_q_b = R_wb.transpose() * v_q_w;

  Eigen::Vector3d a_b;
  a_b << imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
      imu_msg.linear_acceleration.z;

  Eigen::Vector3d omega_b;
  omega_b << imu_msg.angular_velocity.x, imu_msg.angular_velocity.y,
      imu_msg.angular_velocity.z;

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  const double thrust_newton = thrustToNewton(*trpy);
  const Eigen::Vector3d drag_b(drag_x_ * v_q_b.x(), drag_y_ * v_q_b.y(),
                               drag_z_ * v_q_b.z());
  const Eigen::Vector3d zf_b = mass_ * a_b - thrust_newton * e3 - drag_b;

  const double force_norm_body = zf_b.norm();
  Eigen::Vector3d raw_dir_body = Eigen::Vector3d::Zero();
  if (force_norm_body > 1e-9) {
    raw_dir_body = zf_b / force_norm_body;
  }

  bool have_tension = false;
  double tension_newton = 0.0;
  if (tension) {
    const double t_tau = stampToSec(tension->header.stamp);
    tension_newton = tensionToNewton(*tension);
    have_tension = t_tau > 0.0 &&
                   std::fabs(t_imu - t_tau) <= tension_timeout_ &&
                   std::isfinite(tension_newton);
  }

  {
    std::lock_guard<std::mutex> lock(observer_mutex_);

    cacheDebugValues(R_wb, a_b, thrust_newton, zf_b, raw_dir_body,
                     force_norm_body, have_tension, tension_newton);

    if (!observer_initialized_ || last_observer_time_ < 0.0 ||
        std::fabs(t_imu - last_observer_time_) > reset_dt_) {
      Eigen::Vector3d b0_body(0.0, 0.0, -1.0);
      if (have_tension && tension_newton > tau_min_ &&
          force_norm_body > force_min_) {
        b0_body = raw_dir_body;
      }
      initializeObserver(b0_body, a_b, zf_b, have_tension, tension_newton,
                         t_imu);
    }

    double dt = t_imu - last_observer_time_;
    if (dt > 0.0) {
      const int n_steps =
          std::max(1, static_cast<int>(std::ceil(dt / max_prediction_dt_)));
      const double h = dt / static_cast<double>(n_steps);

      for (int i = 0; i < n_steps; ++i) {
        const bool have_dir = use_direction_update_ && have_tension &&
                              tension_newton > tau_min_ &&
                              force_norm_body > force_min_;
        const bool have_tau =
            use_tension_update_ && have_tension && tension_newton > tau_min_;
        propagateObserver(h, omega_b, a_b, zf_b, have_dir, have_tau,
                          tension_newton);
      }
      last_observer_time_ = t_imu;
    }

    normalizeObserverState();
  }
}

void PayloadEquivariantSyncObserverNodelet::initializeObserver(
    const Eigen::Vector3d &b0_body_in, const Eigen::Vector3d &a_b,
    const Eigen::Vector3d &zf_b, bool have_tension, double tension_newton,
    double stamp_sec) {
  Eigen::Vector3d b0_body = b0_body_in;
  if (!finiteVec(b0_body) || b0_body.norm() < 1e-9) {
    b0_body = Eigen::Vector3d(0.0, 0.0, -1.0);
  } else {
    b0_body.normalize();
  }

  Z_.setIdentity();
  Y_ = rotationBToE3(b0_body);
  nu_bar_hat_.setZero();

  const Eigen::Vector3d b_bar0 =
      Y_.transpose() * Eigen::Vector3d(0.0, 0.0, 1.0);
  const double tau_model0 =
      payload_mass_ *
      (cable_length_ * nu_bar_hat_.squaredNorm() - b_bar0.dot(a_b));
  b_tau_hat_ = (have_tension && std::isfinite(tension_newton))
                   ? std::clamp(tension_newton - tau_model0, -max_bias_tension_,
                                max_bias_tension_)
                   : 0.0;

  if (have_tension && std::isfinite(tension_newton) &&
      tension_newton > tau_min_) {
    b_f_bar_hat_ = init_bias_force_scale_ * (zf_b - tension_newton * b_bar0);
    b_f_bar_hat_ =
        b_f_bar_hat_.cwiseMax(-max_bias_force_ * Eigen::Vector3d::Ones())
            .cwiseMin(max_bias_force_ * Eigen::Vector3d::Ones());
  } else {
    b_f_bar_hat_.setZero();
  }

  observer_initialized_ = true;
  last_observer_time_ = stamp_sec;

  normalizeObserverState();

  RCLCPP_INFO(this->get_logger(),
              "[payload_equivariant_sync_observer] Observer initialized at "
              "%.6f with b_body=[%.3f %.3f %.3f], b_tau=%.3f",
              stamp_sec, b0_body.x(), b0_body.y(), b0_body.z(), b_tau_hat_);
}

void PayloadEquivariantSyncObserverNodelet::propagateObserver(
    double dt, const Eigen::Vector3d &omega_b, const Eigen::Vector3d &a_b,
    const Eigen::Vector3d &zf_b, bool have_direction, bool have_tension,
    double tension_newton) {
  if (!observer_initialized_ || dt <= 0.0 || !finiteVec(omega_b) ||
      !finiteVec(a_b) || !finiteVec(zf_b)) {
    return;
  }

  // Synchronize the known body-rate drift.
  Z_ = expSO3(-omega_b * dt) * Z_;
  Z_ = projectToSO3(Z_);

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  const Eigen::Vector3d a_bar = Z_.transpose() * a_b;
  const Eigen::Vector3d zf_bar = Z_.transpose() * zf_b;

  Eigen::Vector3d b_bar_hat = estimatedBBarLocked();

  Eigen::Vector3d y_s = b_bar_hat;
  double k_dir = 0.0;
  if (have_direction) {
    const Eigen::Vector3d dir_vec = zf_bar - b_f_bar_hat_;
    const double dir_norm = dir_vec.norm();
    if (dir_norm > force_min_) {
      y_s = dir_vec / dir_norm;
      k_dir = k_b_;
    }
  }

  // Group update on SO(3): exact left-right split for constant inputs.
  const Eigen::Vector3d right_rot = nu_bar_hat_.cross(y_s);
  const Eigen::Vector3d Ynu = Y_ * nu_bar_hat_;
  const Eigen::Vector3d Yy = Y_ * y_s;
  const Eigen::Vector3d left_rot =
      Ynu.cross(Yy) - Ynu.cross(e3) + k_dir * Yy.cross(e3);
  Y_ = expSO3(left_rot * dt) * Y_ * expSO3(right_rot * dt);
  Y_ = projectToSO3(Y_);

  b_bar_hat = estimatedBBarLocked();

  const double tau_hat =
      payload_mass_ *
      (cable_length_ * nu_bar_hat_.squaredNorm() - b_bar_hat.dot(a_bar));

  double r_tau = 0.0;
  if (have_tension && std::isfinite(tension_newton)) {
    r_tau = tension_newton - (tau_hat + b_tau_hat_);
  }

  Eigen::Vector3d r_f = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_f_perp = Eigen::Vector3d::Zero();
  if (have_direction) {
    r_f = zf_bar - (tau_hat * b_bar_hat + b_f_bar_hat_);
    r_f_perp =
        (Eigen::Matrix3d::Identity() - b_bar_hat * b_bar_hat.transpose()) * r_f;
  }

  Eigen::Vector3d nu_dot =
      (1.0 / cable_length_) *
          ((b_bar_hat * b_bar_hat.transpose() - Eigen::Matrix3d::Identity()) *
           a_bar) -
      nu_bar_hat_.squaredNorm() * b_bar_hat;

  if (have_direction) {
    nu_dot += k_nu_force_ * r_f_perp;
  }
  if (have_tension) {
    nu_dot +=
        2.0 * gamma_tau_ * payload_mass_ * cable_length_ * nu_bar_hat_ * r_tau;
  }

  nu_bar_hat_ += dt * nu_dot;
  nu_bar_hat_ =
      (Eigen::Matrix3d::Identity() - b_bar_hat * b_bar_hat.transpose()) *
      nu_bar_hat_;

  if (have_tension) {
    b_tau_hat_ += dt * gamma_tau_ * r_tau;
    b_tau_hat_ = std::clamp(b_tau_hat_, -max_bias_tension_, max_bias_tension_);
  }

  if (have_direction) {
    b_f_bar_hat_ += dt * gamma_f_ * r_f;
    b_f_bar_hat_ =
        b_f_bar_hat_.cwiseMax(-max_bias_force_ * Eigen::Vector3d::Ones())
            .cwiseMin(max_bias_force_ * Eigen::Vector3d::Ones());
  }

  // Optional: if tension is unavailable or below threshold, slowly relax force
  // bias to avoid frozen bias when the cable is slack.
  if (!have_tension || tension_newton <= tau_min_) {
    b_f_bar_hat_ *= std::max(0.0, 1.0 - 0.1 * dt);
  }

  normalizeObserverState();
}

void PayloadEquivariantSyncObserverNodelet::normalizeObserverState() {
  Z_ = projectToSO3(Z_);
  Y_ = projectToSO3(Y_);

  Eigen::Vector3d b_bar_hat = estimatedBBarLocked();
  if (!finiteVec(b_bar_hat) || b_bar_hat.norm() < 1e-9) {
    Y_.setIdentity();
    b_bar_hat = estimatedBBarLocked();
  }

  if (!finiteVec(nu_bar_hat_)) {
    nu_bar_hat_.setZero();
  }
  nu_bar_hat_ =
      (Eigen::Matrix3d::Identity() - b_bar_hat * b_bar_hat.transpose()) *
      nu_bar_hat_;

  const double nu_norm = nu_bar_hat_.norm();
  if (nu_norm > max_nu_norm_ && nu_norm > 1e-9) {
    nu_bar_hat_ *= (max_nu_norm_ / nu_norm);
  }

  if (!std::isfinite(b_tau_hat_)) {
    b_tau_hat_ = 0.0;
  }
  b_tau_hat_ = std::clamp(b_tau_hat_, -max_bias_tension_, max_bias_tension_);

  if (!finiteVec(b_f_bar_hat_)) {
    b_f_bar_hat_.setZero();
  }
  b_f_bar_hat_ =
      b_f_bar_hat_.cwiseMax(-max_bias_force_ * Eigen::Vector3d::Ones())
          .cwiseMin(max_bias_force_ * Eigen::Vector3d::Ones());
}

void PayloadEquivariantSyncObserverNodelet::cacheDebugValues(
    const Eigen::Matrix3d &R_world_from_body, const Eigen::Vector3d &a_b,
    double thrust_newton, const Eigen::Vector3d &zf_b,
    const Eigen::Vector3d &raw_dir_body, double force_norm_body,
    bool have_tension, double tension_newton) {
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  last_force_inertial_ = R_world_from_body * (mass_ * a_b);
  last_thrust_inertial_ = thrust_newton * (R_world_from_body * e3);
  last_force_residual_body_ = zf_b;
  last_force_residual_world_ = R_world_from_body * zf_b;
  last_raw_direction_body_ = raw_dir_body;
  last_raw_direction_world_ = R_world_from_body * raw_dir_body;
  last_force_norm_body_ = force_norm_body;
  last_tension_newton_ = tension_newton;
  last_have_tension_ = have_tension;
}

void PayloadEquivariantSyncObserverNodelet::publishTimerCallback() {
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

  Eigen::Matrix3d Z, Y;
  Eigen::Vector3d nu_bar_hat;
  double b_tau_hat = 0.0;
  Eigen::Vector3d b_f_bar_hat;
  bool initialized = false;
  double t_obs = -1.0;
  Eigen::Vector3d force_inertial, thrust_inertial, zf_world, raw_dir_world;
  double force_norm_body = 0.0;
  double tension_newton = 0.0;
  bool have_tension = false;

  {
    std::lock_guard<std::mutex> lock(observer_mutex_);
    Z = Z_;
    Y = Y_;
    nu_bar_hat = nu_bar_hat_;
    b_tau_hat = b_tau_hat_;
    b_f_bar_hat = b_f_bar_hat_;
    initialized = observer_initialized_;
    t_obs = last_observer_time_;
    force_inertial = last_force_inertial_;
    thrust_inertial = last_thrust_inertial_;
    zf_world = last_force_residual_world_;
    raw_dir_world = last_raw_direction_world_;
    force_norm_body = last_force_norm_body_;
    tension_newton = last_tension_newton_;
    have_tension = last_have_tension_;
  }

  if (!initialized) {
    return;
  }

  const Eigen::Matrix3d R_wb = odomRotation(*odom);
  const Eigen::Vector3d p_q = odomPositionWorld(*odom);
  const Eigen::Vector3d v_q_w = odomVelocityWorld(*odom, R_wb);

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  Eigen::Vector3d b_bar_hat = Y.transpose() * e3;
  if (b_bar_hat.norm() > 1e-9) {
    b_bar_hat.normalize();
  } else {
    b_bar_hat = Eigen::Vector3d(0.0, 0.0, -1.0);
  }

  const Eigen::Vector3d b_body_hat = Z * b_bar_hat;
  const Eigen::Vector3d n_world_hat = R_wb * b_body_hat;
  const Eigen::Vector3d q_world_hat = R_wb * (Z * nu_bar_hat);

  const Eigen::Vector3d p_payload_hat = p_q + cable_length_ * n_world_hat;
  const Eigen::Vector3d v_payload_hat = v_q_w + cable_length_ * q_world_hat;

  publishFloatVector(
      pub_force_inertial_,
      {force_inertial.x(), force_inertial.y(), force_inertial.z()});
  publishFloatVector(
      pub_thrust_inertial_,
      {thrust_inertial.x(), thrust_inertial.y(), thrust_inertial.z()});
  publishFloatVector(pub_cable_direction_,
                     {raw_dir_world.x(), raw_dir_world.y(), raw_dir_world.z(),
                      force_norm_body});

  publishFloatVector(pub_cable_direction_observer_,
                     {n_world_hat.x(), n_world_hat.y(), n_world_hat.z(),
                      b_body_hat.x(), b_body_hat.y(), b_body_hat.z(),
                      q_world_hat.x(), q_world_hat.y(), q_world_hat.z(),
                      nu_bar_hat.x(), nu_bar_hat.y(), nu_bar_hat.z()});

  // Use synchronized residual relation for debug in a numerically explicit way.
  const Eigen::Vector3d zf_body = R_wb.transpose() * zf_world;
  const Eigen::Vector3d zf_bar = Z.transpose() * zf_body;
  const Eigen::Vector3d A_world = force_inertial / std::max(1e-9, mass_);
  const Eigen::Vector3d a_b = R_wb.transpose() * A_world;
  const Eigen::Vector3d a_bar = Z.transpose() * a_b;
  const double tau_hat_dbg =
      payload_mass_ *
      (cable_length_ * nu_bar_hat.squaredNorm() - b_bar_hat.dot(a_bar));
  const Eigen::Vector3d rf_dbg =
      zf_bar - (tau_hat_dbg * b_bar_hat + b_f_bar_hat);

  publishFloatVector(pub_observer_debug_,
                     {tau_hat_dbg, tau_hat_dbg + b_tau_hat, tension_newton,
                      have_tension ? 1.0 : 0.0, force_norm_body, b_tau_hat,
                      b_f_bar_hat.x(), b_f_bar_hat.y(), b_f_bar_hat.z(),
                      rf_dbg.x(), rf_dbg.y(), rf_dbg.z()});

  builtin_interfaces::msg::Time stamp = odom->header.stamp;
  if (t_obs > 0.0) {
    rclcpp::Time tt(static_cast<int64_t>(t_obs * 1e9));
    stamp = tt;
  }

  geometry_msgs::msg::PointStamped payload_point_msg;
  payload_point_msg.header.stamp = stamp;
  payload_point_msg.header.frame_id = frame_id_;
  payload_point_msg.point.x = p_payload_hat.x();
  payload_point_msg.point.y = p_payload_hat.y();
  payload_point_msg.point.z = p_payload_hat.z();
  pub_payload_est_point_->publish(payload_point_msg);

  nav_msgs::msg::Odometry payload_odom_msg;
  payload_odom_msg.header.stamp = stamp;
  payload_odom_msg.header.frame_id = frame_id_;
  payload_odom_msg.child_frame_id = "payload_sync_equivariant";
  payload_odom_msg.pose.pose.position.x = p_payload_hat.x();
  payload_odom_msg.pose.pose.position.y = p_payload_hat.y();
  payload_odom_msg.pose.pose.position.z = p_payload_hat.z();
  payload_odom_msg.pose.pose.orientation.w = 1.0;
  payload_odom_msg.pose.pose.orientation.x = 0.0;
  payload_odom_msg.pose.pose.orientation.y = 0.0;
  payload_odom_msg.pose.pose.orientation.z = 0.0;
  payload_odom_msg.twist.twist.linear.x = v_payload_hat.x();
  payload_odom_msg.twist.twist.linear.y = v_payload_hat.y();
  payload_odom_msg.twist.twist.linear.z = v_payload_hat.z();
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

Eigen::Vector3d
PayloadEquivariantSyncObserverNodelet::estimatedBBarLocked() const {
  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  Eigen::Vector3d b_bar = Y_.transpose() * e3;
  if (b_bar.norm() > 1e-9) {
    b_bar.normalize();
  }
  return b_bar;
}

Eigen::Vector3d
PayloadEquivariantSyncObserverNodelet::estimatedBBodyLocked() const {
  return Z_ * estimatedBBarLocked();
}

Eigen::Vector3d PayloadEquivariantSyncObserverNodelet::estimatedNWorldLocked(
    const Eigen::Matrix3d &R) const {
  return R * estimatedBBodyLocked();
}

Eigen::Vector3d PayloadEquivariantSyncObserverNodelet::estimatedQWorldLocked(
    const Eigen::Matrix3d &R) const {
  return R * (Z_ * nu_bar_hat_);
}

Eigen::Matrix3d PayloadEquivariantSyncObserverNodelet::quatToRot(
    const Eigen::Vector4d &q_wxyz) {
  Eigen::Quaterniond q(q_wxyz(0), q_wxyz(1), q_wxyz(2), q_wxyz(3));
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q.toRotationMatrix();
}

Eigen::Matrix3d
PayloadEquivariantSyncObserverNodelet::skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Matrix3d
PayloadEquivariantSyncObserverNodelet::expSO3(const Eigen::Vector3d &w) {
  const double theta = w.norm();
  const Eigen::Matrix3d W = skew(w);
  if (theta < 1e-9) {
    return Eigen::Matrix3d::Identity() + W;
  }
  const double a = std::sin(theta) / theta;
  const double b = (1.0 - std::cos(theta)) / (theta * theta);
  return Eigen::Matrix3d::Identity() + a * W + b * W * W;
}

Eigen::Matrix3d
PayloadEquivariantSyncObserverNodelet::projectToSO3(const Eigen::Matrix3d &R) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();
  Eigen::Matrix3d Rproj = U * V.transpose();
  if (Rproj.determinant() < 0.0) {
    U.col(2) *= -1.0;
    Rproj = U * V.transpose();
  }
  return Rproj;
}

double PayloadEquivariantSyncObserverNodelet::stampToSec(
    const builtin_interfaces::msg::Time &stamp) {
  return static_cast<double>(stamp.sec) +
         1e-9 * static_cast<double>(stamp.nanosec);
}

bool PayloadEquivariantSyncObserverNodelet::finiteVec(
    const Eigen::Vector3d &v) {
  return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

Eigen::Matrix3d
PayloadEquivariantSyncObserverNodelet::rotationBToE3(const Eigen::Vector3d &b) {
  Eigen::Vector3d bb = b;
  if (bb.norm() < 1e-9) {
    return Eigen::Matrix3d::Identity();
  }
  bb.normalize();
  Eigen::Quaterniond q =
      Eigen::Quaterniond::FromTwoVectors(bb, Eigen::Vector3d(0.0, 0.0, 1.0));
  q.normalize();
  return q.toRotationMatrix();
}

Eigen::Matrix3d PayloadEquivariantSyncObserverNodelet::odomRotation(
    const nav_msgs::msg::Odometry &odom) const {
  Eigen::Vector4d q_wxyz;
  q_wxyz << odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z;
  return quatToRot(q_wxyz);
}

Eigen::Vector3d PayloadEquivariantSyncObserverNodelet::odomPositionWorld(
    const nav_msgs::msg::Odometry &odom) const {
  return Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
                         odom.pose.pose.position.z);
}

Eigen::Vector3d PayloadEquivariantSyncObserverNodelet::odomVelocityWorld(
    const nav_msgs::msg::Odometry &odom, const Eigen::Matrix3d &R) const {
  Eigen::Vector3d v(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                    odom.twist.twist.linear.z);
  if (odom_twist_in_body_) {
    v = R * v;
  }
  return v;
}

double PayloadEquivariantSyncObserverNodelet::tensionToNewton(
    const sensor_msgs::msg::FluidPressure &msg) const {
  return tension_scale_ * msg.fluid_pressure + tension_offset_;
}

double PayloadEquivariantSyncObserverNodelet::tensionVarianceNewton2(
    const sensor_msgs::msg::FluidPressure &msg) const {
  if (std::isfinite(msg.variance) && msg.variance > 0.0) {
    return std::max(1e-9, tension_scale_ * tension_scale_ * msg.variance);
  }
  return std::max(1e-9, 1.0);
}

double PayloadEquivariantSyncObserverNodelet::thrustToNewton(
    const quadrotor_msgs::msg::TRPYCommand &msg) const {
  return thrust_scale_ * msg.thrust + thrust_offset_;
}

void PayloadEquivariantSyncObserverNodelet::publishFloatVector(
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr &pub,
    const std::initializer_list<double> &values) const {
  if (!pub) {
    return;
  }
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(values.begin(), values.end());
  pub->publish(msg);
}

} // namespace payload_equivariant_sync_observer_nodelet

RCLCPP_COMPONENTS_REGISTER_NODE(payload_equivariant_sync_observer_nodelet::PayloadEquivariantSyncObserverNodelet)
