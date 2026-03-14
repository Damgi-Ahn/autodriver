#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <deque>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace autodriver::tools {

struct StatSummary {
  size_t count = 0;
  double mean  = 0.0;
  double p95   = 0.0;
  double max   = 0.0;
};

struct UpdateRateSummary {
  size_t total   = 0;
  size_t applied = 0;
  double ratio   = 0.0;
  std::map<std::string, size_t> reason_hist;
};

struct OutputAvailability {
  size_t output_count    = 0;
  double expected_count  = 0.0;
  double ratio           = 0.0;
  bool   has_output      = false;
  double last_age_sec    = 0.0;
};

// ---------------------------------------------------------------------------
// KpiSnapshot — one aggregation snapshot (window_sec)
// ---------------------------------------------------------------------------
struct KpiSnapshot {
  rclcpp::Time stamp;
  double window_sec = 0.0;

  // --- Fusion quality ---
  UpdateRateSummary gnss_pos_update;
  UpdateRateSummary gnss_vel_update;
  UpdateRateSummary heading_yaw_update;

  StatSummary gnss_pos_nis;
  StatSummary gnss_vel_nis;
  StatSummary heading_yaw_nis;

  StatSummary gnss_pos_residual_norm;
  StatSummary gnss_vel_residual_norm;
  StatSummary heading_yaw_residual_rad;

  // --- Covariance & noise (latest values from window tail) ---
  std::optional<double> P_trace_latest;
  std::optional<double> P_min_eig_latest;
  std::optional<double> P_max_diag_latest;
  std::optional<double> P_pos_max_diag_latest;
  std::optional<double> P_vel_max_diag_latest;
  std::optional<double> P_att_max_diag_latest;

  // Time series for charts (last N samples within window)
  std::vector<double> P_trace_series;
  std::vector<double> P_min_eig_series;

  // --- Sensor timing ---
  StatSummary gnss_delay;
  StatSummary gnss_vel_delay;
  StatSummary velocity_delay;
  StatSummary steering_delay;

  std::optional<double> imu_dt_min_ms_latest;
  std::optional<double> imu_dt_mean_ms_latest;
  std::optional<double> imu_dt_max_ms_latest;

  // --- Output availability ---
  OutputAvailability output_availability;
  double diag_rate_hz   = 0.0;
  double output_rate_hz = 0.0;

  // --- FGO stats (latest in window) ---
  std::optional<size_t> fgo_keyframe_count_latest;
  std::optional<size_t> fgo_correction_count_latest;

  // --- NIS violation rate (fraction of samples that exceeded gate) ---
  double gnss_pos_nis_violation_rate  = 0.0;
  double gnss_vel_nis_violation_rate  = 0.0;
  double heading_nis_violation_rate   = 0.0;
};

// ---------------------------------------------------------------------------
// KpiEngine
// ---------------------------------------------------------------------------
class KpiEngine {
 public:
  KpiEngine(double window_sec, double expected_output_rate_hz);

  void AddSample(const DiagSample& sample);
  void AddOutputStamp(const rclcpp::Time& stamp);
  KpiSnapshot ComputeSnapshot(const rclcpp::Time& now);

  double window_sec() const { return window_sec_; }

  // NIS gates (used for violation rate computation)
  void set_nis_gates(double pos, double vel, double heading) {
    nis_gate_pos_     = pos;
    nis_gate_vel_     = vel;
    nis_gate_heading_ = heading;
  }

 private:
  void Prune(const rclcpp::Time& now);
  static StatSummary ComputeStats(const std::vector<double>& values);

  double window_sec_              = 10.0;
  double expected_output_rate_hz_ = 0.0;
  double nis_gate_pos_            = 11.34;
  double nis_gate_vel_            = 11.34;
  double nis_gate_heading_        = 6.63;

  std::deque<DiagSample>    samples_;
  std::deque<rclcpp::Time>  output_stamps_;
};

}  // namespace autodriver::tools
