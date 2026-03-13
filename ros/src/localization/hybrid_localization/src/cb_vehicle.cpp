#include "hybrid_localization/hybrid_localization_node.hpp"

#include <cmath>

namespace hybrid_localization
{

// NOTE: Vehicle 콜백은 속도/조향 기반의 제약 업데이트를 담당한다.
void HybridLocalizationNode::velocity_callback(
  const autoware_vehicle_msgs::msg::VelocityReport::SharedPtr t_msg)
{
  const bool activated = (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, last_velocity_stamp_,
      "Velocity"))
  {
    return;
  }

  latest_velocity_ = t_msg;
  last_velocity_stamp_ = current_stamp;
  velocity_count_++;

  if (!activated) {
    return;
  }

  const auto & constraints = node_params_.vehicle_constraints;
  if (!(constraints.enable_speed_update || constraints.enable_nhc ||
    constraints.enable_zupt))
  {
    return;
  }

  if (!std::isfinite(constraints.speed_var) || !(constraints.speed_var > 0.0) ||
    !std::isfinite(constraints.nhc_var) || !(constraints.nhc_var > 0.0) ||
    !std::isfinite(constraints.zupt_var) || !(constraints.zupt_var > 0.0))
  {
    return;
  }

  double v_raw = static_cast<double>(t_msg->longitudinal_velocity);
  v_raw *= constraints.speed_factor;  // Apply speed factor for calibration/tuning
  const double v_abs = std::abs(v_raw);
  const double v_meas =
    constraints.speed_use_abs ? v_abs : v_raw;

  bool eskf_initialized = eskf_.initialized();
  Eigen::Quaterniond q_map_from_base = Eigen::Quaterniond::Identity();
  if (eskf_initialized) {
    q_map_from_base = eskf_.q_map_from_base();
  }
  const Eigen::Matrix3d R = q_map_from_base.toRotationMatrix();

  // ZYX 순서 (yaw, pitch, roll)
  const double yaw   = std::atan2(R(1,0), R(0,0));
  const double pitch = std::asin(-R(2,0));
  const double roll  = std::atan2(R(2,1), R(2,2));

  double v_meas_pitch_calibrated = v_meas * std::cos(pitch);


  // 정지 판단
  // - 1차: OBD 속도 기반 정지 후보
  // - 2차: GNSS 속도가 1km/h 이상이면 저속 크립
  // ZUPT는 "OBD 정지 후보" + "GNSS 속도도 충분히 저속"일 때만 적용
  static constexpr double kPvtZuptSpeedGateMps = 1.0 / 3.6;
  bool is_stationary =
    constraints.enable_zupt && std::isfinite(v_abs) &&
    (v_abs < constraints.zupt_speed_threshold_mps);
  if (is_stationary && latest_gnss_vel_ && (last_gnss_vel_stamp_.seconds() > 0.0)) {
    const double dt_pvt_sec =
      std::abs((current_stamp - last_gnss_vel_stamp_).seconds());
    if (std::isfinite(dt_pvt_sec) && (dt_pvt_sec < 0.5) &&
      std::isfinite(latest_gnss_vel_->twist.linear.x) &&
      std::isfinite(latest_gnss_vel_->twist.linear.y))
    {
      const double ve = static_cast<double>(latest_gnss_vel_->twist.linear.x);
      const double vn = static_cast<double>(latest_gnss_vel_->twist.linear.y);
      const double pvt_speed_mps = std::sqrt(
        vn * vn + ve * ve);
      if (std::isfinite(pvt_speed_mps) && (pvt_speed_mps >= kPvtZuptSpeedGateMps)) {
        is_stationary = false;
      }
    }
  }

  std::scoped_lock<std::mutex> lock(state_mutex_);
  if (!eskf_.initialized()) {
    return;
  }
  const auto tol = rclcpp::Duration::from_seconds(
    std::max(0.0, node_params_.time_alignment_tolerance_sec));
  if (state_stamp_.seconds() > 0.0 && (current_stamp + tol) < state_stamp_) {
    return;
  }

  bool any_applied = false;

  // 속도 업데이트 또는 ZUPT
  if (is_stationary) {
    const auto dbg =
      eskf_.update_body_velocity_component(0, 0.0, constraints.zupt_var);
    any_applied = any_applied || dbg.applied;
  } else if (constraints.enable_speed_update && std::isfinite(v_meas_pitch_calibrated)) {
    const double v_gate = std::abs(v_meas_pitch_calibrated);
    if (constraints.min_speed_mps_for_speed_update > 0.0 &&
      (!std::isfinite(v_gate) ||
      v_gate < constraints.min_speed_mps_for_speed_update))
    {
    } else {
      const auto dbg = eskf_.update_body_velocity_component(
        0, v_meas_pitch_calibrated, constraints.speed_var);
      any_applied = any_applied || dbg.applied;
    }
  }

  // NHC (측면/수직 속도 ≈ 0)
  if (constraints.enable_nhc) {
    const auto dbg_y =
      eskf_.update_body_velocity_component(1, 0.0, constraints.nhc_var);
    const auto dbg_z =
      eskf_.update_body_velocity_component(2, 0.0, constraints.nhc_var);
    any_applied = any_applied || dbg_y.applied || dbg_z.applied;
  }

  if (any_applied) {
    if (current_stamp > state_stamp_) {
      state_stamp_ = current_stamp;
    }
  }
}

void HybridLocalizationNode::steering_callback(
  const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr t_msg)
{
  const bool activated = (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time current_stamp(t_msg->stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, last_steering_stamp_,
      "Steering"))
  {
    return;
  }

  latest_steering_ = t_msg;
  last_steering_stamp_ = current_stamp;
  steering_count_++;

  if (!activated) {
    return;
  }

  const auto & constraints = node_params_.vehicle_constraints;
  if (!constraints.enable_yaw_rate_update) {
    return;
  }

  // yaw rate 업데이트는 속도/각속도 정보가 필요
  if (!latest_velocity_) {
    return;
  }
  if (!have_last_omega_base_) {
    return;
  }
  if (!std::isfinite(node_params_.vehicle.wheelbase) ||
    !(node_params_.vehicle.wheelbase > 0.0) ||
    !std::isfinite(constraints.yaw_rate_var) ||
    !(constraints.yaw_rate_var > 0.0))
  {
    return;
  }

  const double v =
    static_cast<double>(latest_velocity_->longitudinal_velocity);
  if (!std::isfinite(v) || (std::abs(v) < constraints.yaw_rate_min_speed_mps)) {
    return;
  }

  const double delta = static_cast<double>(t_msg->steering_tire_angle);
  if (!std::isfinite(delta)) {
    return;
  }

  // 기준 yaw rate 계산 (v*tan(delta)/L)
  const double yaw_rate_ref =
    v * std::tan(delta) / node_params_.vehicle.wheelbase;
  if (!std::isfinite(yaw_rate_ref)) {
    return;
  }

  const double omega_meas_z = static_cast<double>(last_omega_base_.z);
  if (!std::isfinite(omega_meas_z)) {
    return;
  }

  std::scoped_lock<std::mutex> lock(state_mutex_);
  if (!eskf_.initialized()) {
    return;
  }
  const auto tol = rclcpp::Duration::from_seconds(
    std::max(0.0, node_params_.time_alignment_tolerance_sec));
  if (state_stamp_.seconds() > 0.0 && (current_stamp + tol) < state_stamp_) {
    return;
  }

  const auto dbg = eskf_.update_yaw_rate_from_steer(
    omega_meas_z, yaw_rate_ref, constraints.yaw_rate_var);
  if (dbg.applied) {
    if (current_stamp > state_stamp_) {
      state_stamp_ = current_stamp;
    }
  }
}

} // namespace hybrid_localization
