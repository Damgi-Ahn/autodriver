#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <cctype>

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

  const auto kv = BuildMap(*target);
  sample.raw_kv = kv;

  auto it = kv.find("is_activated");
  if (it != kv.end()) ParseBool(it->second, &sample.is_activated);
  it = kv.find("eskf_initialized");
  if (it != kv.end()) ParseBool(it->second, &sample.eskf_initialized);

  it = kv.find("gnss_pos_update_applied");
  if (it != kv.end()) {
    sample.has_gnss_pos_update_applied = true;
    ParseBool(it->second, &sample.gnss_pos_update_applied);
  }
  it = kv.find("gnss_pos_update_reason");
  if (it != kv.end()) sample.gnss_pos_update_reason = it->second;
  it = kv.find("gnss_pos_nis");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.gnss_pos_nis = value;
  }

  it = kv.find("gnss_vel_update_applied");
  if (it != kv.end()) {
    sample.has_gnss_vel_update_applied = true;
    ParseBool(it->second, &sample.gnss_vel_update_applied);
  }
  it = kv.find("gnss_vel_update_reason");
  if (it != kv.end()) sample.gnss_vel_update_reason = it->second;
  it = kv.find("gnss_vel_nis");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.gnss_vel_nis = value;
  }

  it = kv.find("heading_yaw_update_applied");
  if (it != kv.end()) {
    sample.has_heading_yaw_update_applied = true;
    ParseBool(it->second, &sample.heading_yaw_update_applied);
  }
  it = kv.find("heading_yaw_update_reason");
  if (it != kv.end()) sample.heading_yaw_update_reason = it->second;
  it = kv.find("heading_yaw_nis");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.heading_yaw_nis = value;
  }

  it = kv.find("gnss_delay");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.gnss_delay = value;
  }
  it = kv.find("gnss_vel_delay");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.gnss_vel_delay = value;
  }
  it = kv.find("velocity_delay");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.velocity_delay = value;
  }
  it = kv.find("steering_delay");
  if (it != kv.end()) {
    double value = 0.0;
    if (ParseDouble(it->second, &value)) sample.steering_delay = value;
  }

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
