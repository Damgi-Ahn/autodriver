#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace autodriver::tools {

namespace {

bool ToLowerEquals(const std::string& value, const std::string& expected)
{
  if (value.size() != expected.size()) return false;
  for (size_t i = 0; i < value.size(); ++i) {
    if (static_cast<char>(std::tolower(value[i])) != expected[i]) return false;
  }
  return true;
}

// Macro-like helper to reduce repetitive optional-double parsing
#define PARSE_OPT_DOUBLE(kv_map, key, field) \
  do { \
    auto _it = (kv_map).find(key); \
    if (_it != (kv_map).end()) { \
      double _v = 0.0; \
      if (DiagnosticParser::ParseDouble(_it->second, &_v)) (field) = _v; \
    } \
  } while (0)

#define PARSE_OPT_DOUBLE_UNIT(kv_map, key, field) \
  do { \
    auto _it = (kv_map).find(key); \
    if (_it != (kv_map).end()) { \
      double _v = 0.0; \
      if (DiagnosticParser::ParseDoubleWithUnit(_it->second, &_v)) (field) = _v; \
    } \
  } while (0)

#define PARSE_OPT_SIZET(kv_map, key, field) \
  do { \
    auto _it = (kv_map).find(key); \
    if (_it != (kv_map).end()) { \
      size_t _v = 0; \
      if (DiagnosticParser::ParseSizeT(_it->second, &_v)) (field) = _v; \
    } \
  } while (0)

}  // namespace

bool DiagnosticParser::Parse(
    const diagnostic_msgs::msg::DiagnosticArray& msg,
    DiagSample* out_sample,
    std::string* error) const
{
  if (!out_sample) return false;

  const diagnostic_msgs::msg::DiagnosticStatus* target = nullptr;
  for (const auto& status : msg.status) {
    if (status.name == "hybrid_localization") {
      target = &status;
      break;
    }
  }

  if (!target) {
    if (error) *error = "hybrid_localization status not found";
    return false;
  }

  DiagSample sample;
  sample.stamp = rclcpp::Time(msg.header.stamp);
  sample.diag_level = target->level;

  const auto kv = BuildMap(*target);
  sample.raw_kv = kv;

  // ---- Core state --------------------------------------------------------
  {
    auto it = kv.find("is_activated");
    if (it != kv.end()) ParseBool(it->second, &sample.is_activated);
    it = kv.find("eskf_initialized");
    if (it != kv.end()) ParseBool(it->second, &sample.eskf_initialized);
  }

  // ---- Sensor counts -----------------------------------------------------
  PARSE_OPT_SIZET(kv, "imu_count",      sample.imu_count);
  PARSE_OPT_SIZET(kv, "gnss_count",     sample.gnss_count);
  PARSE_OPT_SIZET(kv, "gnss_vel_count", sample.gnss_vel_count);
  PARSE_OPT_SIZET(kv, "velocity_count", sample.velocity_count);
  PARSE_OPT_SIZET(kv, "steering_count", sample.steering_count);

  // ---- GNSS position update ----------------------------------------------
  {
    auto it = kv.find("gnss_pos_update_applied");
    if (it != kv.end()) {
      sample.has_gnss_pos_update_applied = true;
      ParseBool(it->second, &sample.gnss_pos_update_applied);
    }
    it = kv.find("gnss_pos_update_reason");
    if (it != kv.end()) sample.gnss_pos_update_reason = it->second;
  }
  PARSE_OPT_DOUBLE(kv, "gnss_pos_nis",           sample.gnss_pos_nis);
  PARSE_OPT_DOUBLE(kv, "gnss_pos_residual_norm",  sample.gnss_pos_residual_norm);

  // GNSS status & R
  {
    auto it = kv.find("gnss_status");
    if (it != kv.end()) {
      int v = 0;
      if (ParseInt(it->second, &v)) sample.gnss_status = v;
    }
  }
  PARSE_OPT_DOUBLE(kv, "gnss_pos_R_xx",          sample.gnss_pos_R_xx);
  PARSE_OPT_DOUBLE(kv, "gnss_pos_R_yy",          sample.gnss_pos_R_yy);
  PARSE_OPT_DOUBLE(kv, "gnss_pos_R_zz",          sample.gnss_pos_R_zz);
  PARSE_OPT_DOUBLE(kv, "gnss_pos_status_inflate", sample.gnss_pos_status_inflate);
  PARSE_OPT_DOUBLE(kv, "gnss_pos_nis_inflate",    sample.gnss_pos_nis_inflate);

  // ---- GNSS velocity update ----------------------------------------------
  {
    auto it = kv.find("gnss_vel_update_applied");
    if (it != kv.end()) {
      sample.has_gnss_vel_update_applied = true;
      ParseBool(it->second, &sample.gnss_vel_update_applied);
    }
    it = kv.find("gnss_vel_update_reason");
    if (it != kv.end()) sample.gnss_vel_update_reason = it->second;
  }
  PARSE_OPT_DOUBLE(kv, "gnss_vel_nis",            sample.gnss_vel_nis);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_residual_norm",   sample.gnss_vel_residual_norm);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_R_xx",            sample.gnss_vel_R_xx);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_R_yy",            sample.gnss_vel_R_yy);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_R_zz",            sample.gnss_vel_R_zz);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_status_inflate",  sample.gnss_vel_status_inflate);
  PARSE_OPT_DOUBLE(kv, "gnss_vel_nis_inflate",     sample.gnss_vel_nis_inflate);

  // ---- Heading yaw update ------------------------------------------------
  {
    auto it = kv.find("heading_yaw_update_applied");
    if (it != kv.end()) {
      sample.has_heading_yaw_update_applied = true;
      ParseBool(it->second, &sample.heading_yaw_update_applied);
    }
    it = kv.find("heading_yaw_update_reason");
    if (it != kv.end()) sample.heading_yaw_update_reason = it->second;
  }
  PARSE_OPT_DOUBLE(kv, "heading_yaw_nis",          sample.heading_yaw_nis);
  PARSE_OPT_DOUBLE(kv, "heading_yaw_residual_rad",  sample.heading_yaw_residual_rad);
  PARSE_OPT_DOUBLE(kv, "heading_yaw_var",           sample.heading_yaw_var);
  PARSE_OPT_DOUBLE(kv, "heading_yaw_var_eff",       sample.heading_yaw_var_eff);
  PARSE_OPT_DOUBLE(kv, "heading_yaw_var_applied",   sample.heading_yaw_var_applied);
  PARSE_OPT_DOUBLE(kv, "heading_yaw_nis_inflate",   sample.heading_yaw_nis_inflate);
  PARSE_OPT_DOUBLE(kv, "heading_status_inflate",    sample.heading_status_inflate);
  PARSE_OPT_DOUBLE(kv, "heading_recover_inflate",   sample.heading_recover_inflate);
  {
    auto it = kv.find("heading_yaw_var_source");
    if (it != kv.end()) sample.heading_yaw_var_source = it->second;
  }

  // ---- Vehicle measurement noise (R) -------------------------------------
  PARSE_OPT_DOUBLE(kv, "vehicle_speed_var",    sample.vehicle_speed_var);
  PARSE_OPT_DOUBLE(kv, "vehicle_nhc_var",      sample.vehicle_nhc_var);
  PARSE_OPT_DOUBLE(kv, "vehicle_zupt_var",     sample.vehicle_zupt_var);
  PARSE_OPT_DOUBLE(kv, "vehicle_yaw_rate_var", sample.vehicle_yaw_rate_var);

  // ---- IMU process noise (Q) ---------------------------------------------
  PARSE_OPT_DOUBLE(kv, "imu_gyro_noise_std",        sample.imu_gyro_noise_std);
  PARSE_OPT_DOUBLE(kv, "imu_accel_noise_std",       sample.imu_accel_noise_std);
  PARSE_OPT_DOUBLE(kv, "imu_gyro_bias_noise_std",   sample.imu_gyro_bias_noise_std);
  PARSE_OPT_DOUBLE(kv, "imu_accel_bias_noise_std",  sample.imu_accel_bias_noise_std);

  // ---- Covariance (P) health ---------------------------------------------
  PARSE_OPT_DOUBLE(kv, "P_trace",        sample.P_trace);
  PARSE_OPT_DOUBLE(kv, "P_max_diag",     sample.P_max_diag);
  PARSE_OPT_DOUBLE(kv, "P_min_diag",     sample.P_min_diag);
  PARSE_OPT_DOUBLE(kv, "P_min_eig",      sample.P_min_eig);
  PARSE_OPT_DOUBLE(kv, "P_pos_max_diag", sample.P_pos_max_diag);
  PARSE_OPT_DOUBLE(kv, "P_vel_max_diag", sample.P_vel_max_diag);
  PARSE_OPT_DOUBLE(kv, "P_att_max_diag", sample.P_att_max_diag);
  PARSE_OPT_DOUBLE(kv, "P_bg_max_diag",  sample.P_bg_max_diag);
  PARSE_OPT_DOUBLE(kv, "P_ba_max_diag",  sample.P_ba_max_diag);

  // ---- IMU dt stats (published as "X.XX ms") -----------------------------
  PARSE_OPT_DOUBLE_UNIT(kv, "imu_dt_min",  sample.imu_dt_min_ms);
  PARSE_OPT_DOUBLE_UNIT(kv, "imu_dt_mean", sample.imu_dt_mean_ms);
  PARSE_OPT_DOUBLE_UNIT(kv, "imu_dt_max",  sample.imu_dt_max_ms);

  // ---- Input delays (published as "X.XX ms") -----------------------------
  PARSE_OPT_DOUBLE_UNIT(kv, "gnss_delay",      sample.gnss_delay);
  PARSE_OPT_DOUBLE_UNIT(kv, "gnss_vel_delay",  sample.gnss_vel_delay);
  PARSE_OPT_DOUBLE_UNIT(kv, "velocity_delay",  sample.velocity_delay);
  PARSE_OPT_DOUBLE_UNIT(kv, "steering_delay",  sample.steering_delay);

  // ---- FGO stats ---------------------------------------------------------
  PARSE_OPT_SIZET(kv, "fgo_keyframe_count",   sample.fgo_keyframe_count);
  PARSE_OPT_SIZET(kv, "fgo_correction_count", sample.fgo_correction_count);

  *out_sample = sample;
  return true;
}

bool DiagnosticParser::ParseBool(const std::string& value, bool* out)
{
  if (!out) return false;
  if (ToLowerEquals(value, "true") || value == "1") {
    *out = true;
    return true;
  }
  if (ToLowerEquals(value, "false") || value == "0") {
    *out = false;
    return true;
  }
  return false;
}

bool DiagnosticParser::ParseDouble(const std::string& value, double* out)
{
  if (!out) return false;
  try {
    size_t idx = 0;
    const double parsed = std::stod(value, &idx);
    if (idx == value.size()) {
      *out = parsed;
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool DiagnosticParser::ParseDoubleWithUnit(const std::string& value, double* out)
{
  if (!out || value.empty()) return false;
  try {
    size_t idx = 0;
    const double parsed = std::stod(value, &idx);
    // Accept as long as numeric part was parsed (trailing " ms" etc. are ignored)
    if (idx > 0) {
      *out = parsed;
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool DiagnosticParser::ParseInt(const std::string& value, int* out)
{
  if (!out) return false;
  try {
    size_t idx = 0;
    const int parsed = std::stoi(value, &idx);
    if (idx == value.size()) {
      *out = parsed;
      return true;
    }
  } catch (...) {
  }
  return false;
}

bool DiagnosticParser::ParseSizeT(const std::string& value, size_t* out)
{
  if (!out) return false;
  try {
    size_t idx = 0;
    const unsigned long parsed = std::stoul(value, &idx);
    if (idx == value.size()) {
      *out = static_cast<size_t>(parsed);
      return true;
    }
  } catch (...) {
  }
  return false;
}

std::unordered_map<std::string, std::string> DiagnosticParser::BuildMap(
    const diagnostic_msgs::msg::DiagnosticStatus& status)
{
  std::unordered_map<std::string, std::string> kv;
  kv.reserve(status.values.size());
  for (const auto& item : status.values) {
    kv[item.key] = item.value;
  }
  return kv;
}

}  // namespace autodriver::tools
