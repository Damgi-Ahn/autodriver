#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace autodriver::tools {

KpiEngine::KpiEngine(double window_sec, double expected_output_rate_hz)
    : window_sec_(window_sec),
      expected_output_rate_hz_(expected_output_rate_hz)
{
  if (window_sec_ <= 0.0) window_sec_ = 10.0;
  if (expected_output_rate_hz_ < 0.0) expected_output_rate_hz_ = 0.0;
}

void KpiEngine::AddSample(const DiagSample& sample)
{
  samples_.push_back(sample);
}

void KpiEngine::AddOutputStamp(const rclcpp::Time& stamp)
{
  output_stamps_.push_back(stamp);
}

void KpiEngine::Prune(const rclcpp::Time& now)
{
  const auto window_ns = rclcpp::Duration::from_seconds(window_sec_);
  while (!samples_.empty() && (now - samples_.front().stamp) > window_ns) {
    samples_.pop_front();
  }
  while (!output_stamps_.empty() && (now - output_stamps_.front()) > window_ns) {
    output_stamps_.pop_front();
  }
}

KpiSnapshot KpiEngine::ComputeSnapshot(const rclcpp::Time& now)
{
  Prune(now);

  KpiSnapshot snapshot;
  snapshot.stamp      = now;
  snapshot.window_sec = window_sec_;

  std::vector<double> pos_nis;
  std::vector<double> vel_nis;
  std::vector<double> heading_nis;
  std::vector<double> pos_residual;
  std::vector<double> vel_residual;
  std::vector<double> heading_residual_rad;
  std::vector<double> gnss_delay;
  std::vector<double> gnss_vel_delay;
  std::vector<double> velocity_delay;
  std::vector<double> steering_delay;

  size_t pos_nis_violations     = 0;
  size_t vel_nis_violations     = 0;
  size_t heading_nis_violations = 0;
  size_t pos_nis_total          = 0;
  size_t vel_nis_total          = 0;
  size_t heading_nis_total      = 0;

  for (const auto& s : samples_) {
    // ---- Update rate aggregation ----------------------------------------
    if (s.has_gnss_pos_update_applied) {
      snapshot.gnss_pos_update.total += 1;
      if (s.gnss_pos_update_applied) snapshot.gnss_pos_update.applied += 1;
      if (!s.gnss_pos_update_reason.empty())
        snapshot.gnss_pos_update.reason_hist[s.gnss_pos_update_reason] += 1;
    }
    if (s.has_gnss_vel_update_applied) {
      snapshot.gnss_vel_update.total += 1;
      if (s.gnss_vel_update_applied) snapshot.gnss_vel_update.applied += 1;
      if (!s.gnss_vel_update_reason.empty())
        snapshot.gnss_vel_update.reason_hist[s.gnss_vel_update_reason] += 1;
    }
    if (s.has_heading_yaw_update_applied) {
      snapshot.heading_yaw_update.total += 1;
      if (s.heading_yaw_update_applied) snapshot.heading_yaw_update.applied += 1;
      if (!s.heading_yaw_update_reason.empty())
        snapshot.heading_yaw_update.reason_hist[s.heading_yaw_update_reason] += 1;
    }

    // ---- NIS & residuals -------------------------------------------------
    if (s.gnss_pos_nis.has_value() && std::isfinite(*s.gnss_pos_nis)) {
      pos_nis.push_back(*s.gnss_pos_nis);
      pos_nis_total++;
      if (*s.gnss_pos_nis > nis_gate_pos_) pos_nis_violations++;
    }
    if (s.gnss_vel_nis.has_value() && std::isfinite(*s.gnss_vel_nis)) {
      vel_nis.push_back(*s.gnss_vel_nis);
      vel_nis_total++;
      if (*s.gnss_vel_nis > nis_gate_vel_) vel_nis_violations++;
    }
    if (s.heading_yaw_nis.has_value() && std::isfinite(*s.heading_yaw_nis)) {
      heading_nis.push_back(*s.heading_yaw_nis);
      heading_nis_total++;
      if (*s.heading_yaw_nis > nis_gate_heading_) heading_nis_violations++;
    }
    if (s.gnss_pos_residual_norm.has_value()) pos_residual.push_back(*s.gnss_pos_residual_norm);
    if (s.gnss_vel_residual_norm.has_value()) vel_residual.push_back(*s.gnss_vel_residual_norm);
    if (s.heading_yaw_residual_rad.has_value()) heading_residual_rad.push_back(*s.heading_yaw_residual_rad);

    // ---- Delays ----------------------------------------------------------
    if (s.gnss_delay.has_value())     gnss_delay.push_back(*s.gnss_delay);
    if (s.gnss_vel_delay.has_value()) gnss_vel_delay.push_back(*s.gnss_vel_delay);
    if (s.velocity_delay.has_value()) velocity_delay.push_back(*s.velocity_delay);
    if (s.steering_delay.has_value()) steering_delay.push_back(*s.steering_delay);

    // ---- P stats (build time series, track latest) -----------------------
    if (s.P_trace.has_value() && std::isfinite(*s.P_trace)) {
      snapshot.P_trace_series.push_back(*s.P_trace);
      snapshot.P_trace_latest = *s.P_trace;
    }
    if (s.P_min_eig.has_value() && std::isfinite(*s.P_min_eig)) {
      snapshot.P_min_eig_series.push_back(*s.P_min_eig);
      snapshot.P_min_eig_latest = *s.P_min_eig;
    }
    if (s.P_max_diag.has_value())     snapshot.P_max_diag_latest     = *s.P_max_diag;
    if (s.P_pos_max_diag.has_value()) snapshot.P_pos_max_diag_latest = *s.P_pos_max_diag;
    if (s.P_vel_max_diag.has_value()) snapshot.P_vel_max_diag_latest = *s.P_vel_max_diag;
    if (s.P_att_max_diag.has_value()) snapshot.P_att_max_diag_latest = *s.P_att_max_diag;

    // ---- IMU dt (latest) -------------------------------------------------
    if (s.imu_dt_min_ms.has_value())  snapshot.imu_dt_min_ms_latest  = *s.imu_dt_min_ms;
    if (s.imu_dt_mean_ms.has_value()) snapshot.imu_dt_mean_ms_latest = *s.imu_dt_mean_ms;
    if (s.imu_dt_max_ms.has_value())  snapshot.imu_dt_max_ms_latest  = *s.imu_dt_max_ms;

    // ---- FGO stats (latest) ----------------------------------------------
    if (s.fgo_keyframe_count.has_value())   snapshot.fgo_keyframe_count_latest   = *s.fgo_keyframe_count;
    if (s.fgo_correction_count.has_value()) snapshot.fgo_correction_count_latest = *s.fgo_correction_count;
  }

  // ---- Compute update ratios ---------------------------------------------
  auto compute_ratio = [](UpdateRateSummary& u) {
    if (u.total > 0) u.ratio = static_cast<double>(u.applied) / static_cast<double>(u.total);
  };
  compute_ratio(snapshot.gnss_pos_update);
  compute_ratio(snapshot.gnss_vel_update);
  compute_ratio(snapshot.heading_yaw_update);

  // ---- NIS stats & violation rates ---------------------------------------
  snapshot.gnss_pos_nis           = ComputeStats(pos_nis);
  snapshot.gnss_vel_nis           = ComputeStats(vel_nis);
  snapshot.heading_yaw_nis        = ComputeStats(heading_nis);
  snapshot.gnss_pos_residual_norm = ComputeStats(pos_residual);
  snapshot.gnss_vel_residual_norm = ComputeStats(vel_residual);
  snapshot.heading_yaw_residual_rad = ComputeStats(heading_residual_rad);

  if (pos_nis_total > 0)
    snapshot.gnss_pos_nis_violation_rate = static_cast<double>(pos_nis_violations) / static_cast<double>(pos_nis_total);
  if (vel_nis_total > 0)
    snapshot.gnss_vel_nis_violation_rate = static_cast<double>(vel_nis_violations) / static_cast<double>(vel_nis_total);
  if (heading_nis_total > 0)
    snapshot.heading_nis_violation_rate = static_cast<double>(heading_nis_violations) / static_cast<double>(heading_nis_total);

  // ---- Delay stats -------------------------------------------------------
  snapshot.gnss_delay     = ComputeStats(gnss_delay);
  snapshot.gnss_vel_delay = ComputeStats(gnss_vel_delay);
  snapshot.velocity_delay = ComputeStats(velocity_delay);
  snapshot.steering_delay = ComputeStats(steering_delay);

  // ---- Output availability -----------------------------------------------
  snapshot.output_availability.output_count   = output_stamps_.size();
  snapshot.output_availability.expected_count = window_sec_ * expected_output_rate_hz_;
  if (snapshot.output_availability.expected_count > 0.0) {
    snapshot.output_availability.ratio = std::min(
        1.0,
        static_cast<double>(snapshot.output_availability.output_count) /
            snapshot.output_availability.expected_count);
  }
  if (!output_stamps_.empty()) {
    snapshot.output_availability.has_output = true;
    snapshot.output_availability.last_age_sec = (now - output_stamps_.back()).seconds();
  }
  if (window_sec_ > 0.0) {
    snapshot.diag_rate_hz   = static_cast<double>(samples_.size()) / window_sec_;
    snapshot.output_rate_hz = static_cast<double>(output_stamps_.size()) / window_sec_;
  }

  return snapshot;
}

StatSummary KpiEngine::ComputeStats(const std::vector<double>& values)
{
  StatSummary stat;
  if (values.empty()) return stat;

  stat.count = values.size();
  stat.max   = *std::max_element(values.begin(), values.end());
  stat.mean  = std::accumulate(values.begin(), values.end(), 0.0) /
               static_cast<double>(values.size());

  std::vector<double> copy = values;
  const double rank = std::ceil(0.95 * static_cast<double>(copy.size()));
  const size_t idx  = (rank <= 1.0) ? 0 : static_cast<size_t>(rank - 1.0);
  std::nth_element(copy.begin(), copy.begin() + idx, copy.end());
  stat.p95 = copy[idx];

  return stat;
}

}  // namespace autodriver::tools
