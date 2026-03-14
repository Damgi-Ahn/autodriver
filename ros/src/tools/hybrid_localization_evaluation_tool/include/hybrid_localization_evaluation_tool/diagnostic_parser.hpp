#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

namespace autodriver::tools {

struct DiagSample {
  rclcpp::Time stamp;

  // Raw key-value map (for inspector tab)
  std::unordered_map<std::string, std::string> raw_kv;

  // --- Core state ---
  bool is_activated = false;
  bool eskf_initialized = false;
  uint8_t diag_level = 0;  // 0=OK 1=WARN 2=ERROR 3=STALE

  // --- Sensor counts ---
  std::optional<size_t> imu_count;
  std::optional<size_t> gnss_count;
  std::optional<size_t> gnss_vel_count;
  std::optional<size_t> velocity_count;
  std::optional<size_t> steering_count;

  // --- GNSS position update ---
  bool has_gnss_pos_update_applied = false;
  bool gnss_pos_update_applied = false;
  std::string gnss_pos_update_reason;
  std::optional<double> gnss_pos_nis;
  std::optional<double> gnss_pos_residual_norm;

  // --- GNSS status & R matrix ---
  std::optional<int> gnss_status;
  std::optional<double> gnss_pos_R_xx;
  std::optional<double> gnss_pos_R_yy;
  std::optional<double> gnss_pos_R_zz;
  std::optional<double> gnss_pos_status_inflate;
  std::optional<double> gnss_pos_nis_inflate;

  // --- GNSS velocity update ---
  bool has_gnss_vel_update_applied = false;
  bool gnss_vel_update_applied = false;
  std::string gnss_vel_update_reason;
  std::optional<double> gnss_vel_nis;
  std::optional<double> gnss_vel_residual_norm;

  // --- GNSS velocity R matrix ---
  std::optional<double> gnss_vel_R_xx;
  std::optional<double> gnss_vel_R_yy;
  std::optional<double> gnss_vel_R_zz;
  std::optional<double> gnss_vel_status_inflate;
  std::optional<double> gnss_vel_nis_inflate;

  // --- Heading yaw update ---
  bool has_heading_yaw_update_applied = false;
  bool heading_yaw_update_applied = false;
  std::string heading_yaw_update_reason;
  std::optional<double> heading_yaw_nis;
  std::optional<double> heading_yaw_residual_rad;

  // --- Heading yaw variance / inflate ---
  std::optional<double> heading_yaw_var;
  std::optional<double> heading_yaw_var_eff;
  std::optional<double> heading_yaw_var_applied;
  std::optional<double> heading_yaw_nis_inflate;
  std::optional<double> heading_status_inflate;
  std::optional<double> heading_recover_inflate;
  std::string heading_yaw_var_source;

  // --- Vehicle measurement noise (R) ---
  std::optional<double> vehicle_speed_var;
  std::optional<double> vehicle_nhc_var;
  std::optional<double> vehicle_zupt_var;
  std::optional<double> vehicle_yaw_rate_var;

  // --- IMU process noise (Q) ---
  std::optional<double> imu_gyro_noise_std;
  std::optional<double> imu_accel_noise_std;
  std::optional<double> imu_gyro_bias_noise_std;
  std::optional<double> imu_accel_bias_noise_std;

  // --- Covariance (P) health ---
  std::optional<double> P_trace;
  std::optional<double> P_max_diag;
  std::optional<double> P_min_diag;
  std::optional<double> P_min_eig;
  std::optional<double> P_pos_max_diag;
  std::optional<double> P_vel_max_diag;
  std::optional<double> P_att_max_diag;
  std::optional<double> P_bg_max_diag;
  std::optional<double> P_ba_max_diag;

  // --- IMU dt stats [ms] ---
  std::optional<double> imu_dt_min_ms;
  std::optional<double> imu_dt_mean_ms;
  std::optional<double> imu_dt_max_ms;

  // --- Input delays [ms] ---
  std::optional<double> gnss_delay;       // already in ms from publisher
  std::optional<double> gnss_vel_delay;
  std::optional<double> velocity_delay;
  std::optional<double> steering_delay;

  // --- FGO stats ---
  std::optional<size_t> fgo_keyframe_count;
  std::optional<size_t> fgo_correction_count;
};

class DiagnosticParser {
 public:
  bool Parse(const diagnostic_msgs::msg::DiagnosticArray& msg,
             DiagSample* out_sample,
             std::string* error) const;

 private:
  static bool ParseBool(const std::string& value, bool* out);
  static bool ParseDouble(const std::string& value, double* out);
  // Parses "123.456 ms" → 123.456 (extracts leading numeric part)
  static bool ParseDoubleWithUnit(const std::string& value, double* out);
  static bool ParseInt(const std::string& value, int* out);
  static bool ParseSizeT(const std::string& value, size_t* out);
  static std::unordered_map<std::string, std::string> BuildMap(
      const diagnostic_msgs::msg::DiagnosticStatus& status);
};

}  // namespace autodriver::tools
