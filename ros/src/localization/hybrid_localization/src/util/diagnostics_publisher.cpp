#include "hybrid_localization/util/diagnostics_publisher.hpp"

#include <string>

namespace hybrid_localization
{

void EskfDiagnosticsPublisher::add_key_value(
  diagnostic_msgs::msg::DiagnosticStatus & status,
  const std::string & key,
  const std::string & value)
{
  diagnostic_msgs::msg::KeyValue kv;
  kv.key = key;
  kv.value = value;
  status.values.push_back(kv);
}

diagnostic_msgs::msg::DiagnosticArray EskfDiagnosticsPublisher::build(
  const rclcpp::Time & stamp,
  const EskfDiagnosticsInput & input) const
{
  diagnostic_msgs::msg::DiagnosticArray diag_array;
  diag_array.header.stamp = stamp;

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "hybrid_localization";
  // diag_level: 0=OK, 1=WARN, 2=ERROR, 3=STALE (상태 머신이 결정, 기본=OK)
  status.level = input.diag_level;
  status.message = "Phase 7: ESKF running";

  add_key_value(
    status, "is_activated",
    input.is_activated ? "true" : "false");
  if (!input.is_activated) {
    if (status.level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.message = "process is not activated";
  }

  add_key_value(status, "imu_count", std::to_string(input.imu_count));
  add_key_value(status, "gnss_count", std::to_string(input.gnss_count));
  add_key_value(status, "gnss_vel_count", std::to_string(input.gnss_vel_count));
  add_key_value(
    status, "velocity_count",
    std::to_string(input.velocity_count));
  add_key_value(
    status, "steering_count",
    std::to_string(input.steering_count));

  add_key_value(
    status, "eskf_initialized",
    input.eskf_initialized ? "true" : "false");

  add_key_value(
    status, "gnss_pos_update_applied",
    input.gnss_pos_update.applied ? "true" : "false");
  if (!input.gnss_pos_update.reason.empty()) {
    add_key_value(status, "gnss_pos_update_reason", input.gnss_pos_update.reason);
  }
  if (input.gnss_pos_update.applied) {
    add_key_value(
      status, "gnss_pos_nis",
      std::to_string(input.gnss_pos_update.nis));
    add_key_value(
      status, "gnss_pos_residual_norm",
      std::to_string(input.gnss_pos_update.residual.norm()));
  }

  add_key_value(
    status, "gnss_vel_update_applied",
    input.gnss_vel_update.applied ? "true" : "false");
  if (!input.gnss_vel_update.reason.empty()) {
    add_key_value(status, "gnss_vel_update_reason", input.gnss_vel_update.reason);
  }
  if (input.gnss_vel_update.applied) {
    add_key_value(
      status, "gnss_vel_nis",
      std::to_string(input.gnss_vel_update.nis));
    add_key_value(
      status, "gnss_vel_residual_norm",
      std::to_string(input.gnss_vel_update.residual.norm()));
  }

  add_key_value(
    status, "heading_yaw_update_applied",
    input.heading_yaw_update.applied ? "true" : "false");
  if (!input.heading_yaw_update.reason.empty()) {
    add_key_value(
      status, "heading_yaw_update_reason",
      input.heading_yaw_update.reason);
  }
  if (input.heading_yaw_update.applied) {
    add_key_value(
      status, "heading_yaw_nis",
      std::to_string(input.heading_yaw_update.nis));
    add_key_value(
      status, "heading_yaw_residual_rad",
      std::to_string(input.heading_yaw_update.residual_rad));
  }

  if (input.has_gnss_status) {
    add_key_value(status, "gnss_status", std::to_string(input.gnss_status));
  }

  if (input.has_gnss_pos_R) {
    add_key_value(status, "gnss_pos_R_xx", std::to_string(input.gnss_pos_R_xx));
    add_key_value(status, "gnss_pos_R_yy", std::to_string(input.gnss_pos_R_yy));
    add_key_value(status, "gnss_pos_R_zz", std::to_string(input.gnss_pos_R_zz));
    add_key_value(
      status, "gnss_pos_status_inflate",
      std::to_string(input.gnss_pos_status_inflate));
    add_key_value(
      status, "gnss_pos_nis_inflate",
      std::to_string(input.gnss_pos_nis_inflate));
  }

  if (input.has_gnss_vel_R) {
    add_key_value(status, "gnss_vel_R_xx", std::to_string(input.gnss_vel_R_xx));
    add_key_value(status, "gnss_vel_R_yy", std::to_string(input.gnss_vel_R_yy));
    add_key_value(status, "gnss_vel_R_zz", std::to_string(input.gnss_vel_R_zz));
    add_key_value(
      status, "gnss_vel_status_inflate",
      std::to_string(input.gnss_vel_status_inflate));
    add_key_value(
      status, "gnss_vel_nis_inflate",
      std::to_string(input.gnss_vel_nis_inflate));
  }

  if (input.has_heading_yaw_R) {
    add_key_value(
      status, "heading_yaw_var",
      std::to_string(input.heading_yaw_var));
    add_key_value(
      status, "heading_yaw_nis_inflate",
      std::to_string(input.heading_yaw_nis_inflate));
    add_key_value(
      status, "heading_yaw_var_eff",
      std::to_string(input.heading_yaw_var_eff));
    add_key_value(
      status, "heading_yaw_var_applied",
      std::to_string(input.heading_yaw_var_applied));
    add_key_value(
      status, "heading_status_inflate",
      std::to_string(input.heading_status_inflate));
    add_key_value(
      status, "heading_recover_inflate",
      std::to_string(input.heading_recover_inflate));
    add_key_value(
      status, "heading_yaw_var_source",
      input.heading_yaw_var_source);
  }

  if (input.has_vehicle_R) {
    add_key_value(
      status, "vehicle_speed_var",
      std::to_string(input.vehicle_speed_var));
    add_key_value(status, "vehicle_nhc_var", std::to_string(input.nhc_var));
    add_key_value(status, "vehicle_zupt_var", std::to_string(input.zupt_var));
    add_key_value(
      status, "vehicle_yaw_rate_var",
      std::to_string(input.yaw_rate_var));
  }

  if (input.has_imu_Q) {
    add_key_value(
      status, "imu_gyro_noise_std",
      std::to_string(input.imu_gyro_noise_std));
    add_key_value(
      status, "imu_accel_noise_std",
      std::to_string(input.imu_accel_noise_std));
    add_key_value(
      status, "imu_gyro_bias_noise_std",
      std::to_string(input.imu_gyro_bias_noise_std));
    add_key_value(
      status, "imu_accel_bias_noise_std",
      std::to_string(input.imu_accel_bias_noise_std));
  }

  if (input.has_P_stats) {
    add_key_value(status, "P_trace", std::to_string(input.P_trace));
    add_key_value(status, "P_max_diag", std::to_string(input.P_max_diag));
    add_key_value(status, "P_min_diag", std::to_string(input.P_min_diag));
    add_key_value(status, "P_min_eig", std::to_string(input.P_min_eig));
    add_key_value(status, "P_pos_max_diag", std::to_string(input.P_pos_max_diag));
    add_key_value(status, "P_vel_max_diag", std::to_string(input.P_vel_max_diag));
    add_key_value(status, "P_att_max_diag", std::to_string(input.P_att_max_diag));
    add_key_value(status, "P_bg_max_diag", std::to_string(input.P_bg_max_diag));
    add_key_value(status, "P_ba_max_diag", std::to_string(input.P_ba_max_diag));
  }

  if (input.has_imu_dt_stats) {
    const double dt_mean = input.imu_dt_stats.mean();
    add_key_value(
      status, "imu_dt_min",
      std::to_string(input.imu_dt_stats.dt_min * 1000.0) +
      " ms");
    add_key_value(
      status, "imu_dt_mean",
      std::to_string(dt_mean * 1000.0) + " ms");
    add_key_value(
      status, "imu_dt_max",
      std::to_string(input.imu_dt_stats.dt_max * 1000.0) +
      " ms");
  }

  if (input.has_gnss_delay) {
    add_key_value(
      status, "gnss_delay",
      std::to_string(input.gnss_delay_sec * 1000.0) + " ms");
  }
  if (input.has_gnss_vel_delay) {
    add_key_value(
      status, "gnss_vel_delay",
      std::to_string(input.gnss_vel_delay_sec * 1000.0) + " ms");
  }
  if (input.has_velocity_delay) {
    add_key_value(
      status, "velocity_delay",
      std::to_string(input.velocity_delay_sec * 1000.0) + " ms");
  }
  if (input.has_steering_delay) {
    add_key_value(
      status, "steering_delay",
      std::to_string(input.steering_delay_sec * 1000.0) + " ms");
  }


  // FGO Stage 5 진단
  if (input.has_fgo_stats) {
    add_key_value(
      status, "fgo_keyframe_count",
      std::to_string(input.fgo_keyframe_count));
    add_key_value(
      status, "fgo_correction_count",
      std::to_string(input.fgo_correction_count));
  }

  diag_array.status.push_back(status);
  return diag_array;
}

} // namespace hybrid_localization
