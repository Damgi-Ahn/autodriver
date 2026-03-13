#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <string>
#include <unordered_map>

namespace autodriver::tools {

struct DiagSample {
  rclcpp::Time stamp;

  bool is_activated = false;
  bool eskf_initialized = false;

  bool has_gnss_pos_update_applied = false;
  bool gnss_pos_update_applied = false;
  std::string gnss_pos_update_reason;
  std::optional<double> gnss_pos_nis;

  bool has_gnss_vel_update_applied = false;
  bool gnss_vel_update_applied = false;
  std::string gnss_vel_update_reason;
  std::optional<double> gnss_vel_nis;

  bool has_heading_yaw_update_applied = false;
  bool heading_yaw_update_applied = false;
  std::string heading_yaw_update_reason;
  std::optional<double> heading_yaw_nis;

  std::optional<double> gnss_delay;
  std::optional<double> gnss_vel_delay;
  std::optional<double> velocity_delay;
  std::optional<double> steering_delay;
};

class DiagnosticParser {
 public:
  bool Parse(const diagnostic_msgs::msg::DiagnosticArray& msg,
             DiagSample* out_sample,
             std::string* error) const;

 private:
  static bool ParseBool(const std::string& value, bool* out);
  static bool ParseDouble(const std::string& value, double* out);
  static std::unordered_map<std::string, std::string> BuildMap(
      const diagnostic_msgs::msg::DiagnosticStatus& status);
};

}  // namespace autodriver::tools
