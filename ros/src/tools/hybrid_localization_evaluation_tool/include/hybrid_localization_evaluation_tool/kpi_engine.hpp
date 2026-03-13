#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <deque>
#include <map>
#include <string>
#include <vector>

namespace autodriver::tools {

struct StatSummary {
  size_t count = 0;
  double mean = 0.0;
  double p95 = 0.0;
  double max = 0.0;
};

struct UpdateRateSummary {
  size_t total = 0;
  size_t applied = 0;
  double ratio = 0.0;
  std::map<std::string, size_t> reason_hist;
};

struct OutputAvailability {
  size_t output_count = 0;
  double expected_count = 0.0;
  double ratio = 0.0;
  bool has_output = false;
  double last_age_sec = 0.0;
};

struct KpiSnapshot {
  rclcpp::Time stamp;
  double window_sec = 0.0;

  UpdateRateSummary gnss_pos_update;
  UpdateRateSummary gnss_vel_update;
  UpdateRateSummary heading_yaw_update;

  OutputAvailability output_availability;
  double diag_rate_hz = 0.0;
  double output_rate_hz = 0.0;

  StatSummary gnss_pos_nis;
  StatSummary gnss_vel_nis;
  StatSummary heading_yaw_nis;

  StatSummary gnss_delay;
  StatSummary gnss_vel_delay;
  StatSummary velocity_delay;
  StatSummary steering_delay;
};

class KpiEngine {
 public:
  KpiEngine(double window_sec, double expected_output_rate_hz);

  void AddSample(const DiagSample& sample);
  void AddOutputStamp(const rclcpp::Time& stamp);
  KpiSnapshot ComputeSnapshot(const rclcpp::Time& now);

  double window_sec() const { return window_sec_; }

 private:
  void Prune(const rclcpp::Time& now);
  static StatSummary ComputeStats(const std::vector<double>& values);

  double window_sec_ = 10.0;
  double expected_output_rate_hz_ = 0.0;
  std::deque<DiagSample> samples_;
  std::deque<rclcpp::Time> output_stamps_;
};

}  // namespace autodriver::tools
