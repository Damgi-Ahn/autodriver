#ifndef HYBRID_LOCALIZATION__UTIL__DIAGNOSTICS_PUBLISHER_HPP_
#define HYBRID_LOCALIZATION__UTIL__DIAGNOSTICS_PUBLISHER_HPP_

#include <cstddef>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/time.hpp>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/util/time_processing.hpp"

namespace hybrid_localization
{

struct EskfDiagnosticsInput
{
  size_t imu_count{0};
  size_t gnss_count{0};
  size_t gnss_vel_count{0};
  size_t velocity_count{0};
  size_t steering_count{0};

  bool is_activated{true};
  bool eskf_initialized{false};

  EskfGnssPosUpdateDebug gnss_pos_update{};
  EskfGnssVelUpdateDebug gnss_vel_update{};
  EskfYawUpdateDebug heading_yaw_update{};

  // Latest GNSS status (NavSatFix.status.status)
  bool has_gnss_status{false};
  int gnss_status{0};

  // Measurement noise (R) quantitative info
  bool has_gnss_pos_R{false};
  double gnss_pos_R_xx{0.0};
  double gnss_pos_R_yy{0.0};
  double gnss_pos_R_zz{0.0};
  double gnss_pos_status_inflate{1.0};
  double gnss_pos_nis_inflate{1.0};

  bool has_gnss_vel_R{false};
  double gnss_vel_R_xx{0.0};
  double gnss_vel_R_yy{0.0};
  double gnss_vel_R_zz{0.0};
  double gnss_vel_status_inflate{1.0};
  double gnss_vel_nis_inflate{1.0};

  bool has_heading_yaw_R{false};
  double heading_yaw_var{0.0};
  double heading_yaw_nis_inflate{1.0};
  double heading_yaw_var_eff{0.0};
  double heading_yaw_var_applied{0.0};
  double heading_status_inflate{1.0};
  double heading_recover_inflate{1.0};
  std::string heading_yaw_var_source{};

  bool has_vehicle_R{false};
  double vehicle_speed_var{0.0};
  double nhc_var{0.0};
  double zupt_var{0.0};
  double yaw_rate_var{0.0};

  // IMU enters as process noise (Q), not measurement noise (R)
  bool has_imu_Q{false};
  double imu_gyro_noise_std{0.0};
  double imu_accel_noise_std{0.0};
  double imu_gyro_bias_noise_std{0.0};
  double imu_accel_bias_noise_std{0.0};

  // Covariance (P) diagnostics (15x15 error-state covariance)
  bool has_P_stats{false};
  double P_trace{0.0};
  double P_max_diag{0.0};
  double P_min_diag{0.0};
  double P_min_eig{0.0};
  double P_pos_max_diag{0.0};
  double P_vel_max_diag{0.0};
  double P_att_max_diag{0.0};
  double P_bg_max_diag{0.0};
  double P_ba_max_diag{0.0};

  bool has_imu_dt_stats{false};
  ImuDtStats imu_dt_stats{};

  bool has_gnss_delay{false};
  double gnss_delay_sec{0.0};
  bool has_gnss_vel_delay{false};
  double gnss_vel_delay_sec{0.0};
  bool has_velocity_delay{false};
  double velocity_delay_sec{0.0};
  bool has_steering_delay{false};
  double steering_delay_sec{0.0};

  // FGO Stage 5 진단
  bool has_fgo_stats{false};
  size_t fgo_keyframe_count{0};   // 총 키프레임 수
  size_t fgo_correction_count{0}; // 총 FGO 보정 횟수

  // 상위 시스템 fault 보고용 수준 (diagnostic_msgs::msg::DiagnosticStatus level)
  // 0=OK, 1=WARN, 2=ERROR, 3=STALE
  uint8_t diag_level{0};
};

class EskfDiagnosticsPublisher
{
public:
  diagnostic_msgs::msg::DiagnosticArray build(
    const rclcpp::Time & stamp,
    const EskfDiagnosticsInput & input) const;

private:
  static void add_key_value(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const std::string & value);
};

} // namespace hybrid_localization

#endif // HYBRID_LOCALIZATION__UTIL__DIAGNOSTICS_PUBLISHER_HPP_
