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
  snapshot.stamp = now;
  snapshot.window_sec = window_sec_;

  std::vector<double> pos_nis;
  std::vector<double> vel_nis;
  std::vector<double> heading_nis;
  std::vector<double> gnss_delay;
  std::vector<double> gnss_vel_delay;
  std::vector<double> velocity_delay;
  std::vector<double> steering_delay;

  for (const auto& sample : samples_) {
    if (sample.has_gnss_pos_update_applied) {
      snapshot.gnss_pos_update.total += 1;
      if (sample.gnss_pos_update_applied) snapshot.gnss_pos_update.applied += 1;
      if (!sample.gnss_pos_update_reason.empty()) {
        snapshot.gnss_pos_update.reason_hist[sample.gnss_pos_update_reason] += 1;
      }
    }

    if (sample.has_gnss_vel_update_applied) {
      snapshot.gnss_vel_update.total += 1;
      if (sample.gnss_vel_update_applied) snapshot.gnss_vel_update.applied += 1;
      if (!sample.gnss_vel_update_reason.empty()) {
        snapshot.gnss_vel_update.reason_hist[sample.gnss_vel_update_reason] += 1;
      }
    }

    if (sample.has_heading_yaw_update_applied) {
      snapshot.heading_yaw_update.total += 1;
      if (sample.heading_yaw_update_applied) snapshot.heading_yaw_update.applied += 1;
      if (!sample.heading_yaw_update_reason.empty()) {
        snapshot.heading_yaw_update.reason_hist[sample.heading_yaw_update_reason] += 1;
      }
    }

    if (sample.gnss_pos_nis.has_value()) pos_nis.push_back(sample.gnss_pos_nis.value());
    if (sample.gnss_vel_nis.has_value()) vel_nis.push_back(sample.gnss_vel_nis.value());
    if (sample.heading_yaw_nis.has_value()) heading_nis.push_back(sample.heading_yaw_nis.value());

    if (sample.gnss_delay.has_value()) gnss_delay.push_back(sample.gnss_delay.value());
    if (sample.gnss_vel_delay.has_value()) gnss_vel_delay.push_back(sample.gnss_vel_delay.value());
    if (sample.velocity_delay.has_value()) velocity_delay.push_back(sample.velocity_delay.value());
    if (sample.steering_delay.has_value()) steering_delay.push_back(sample.steering_delay.value());
  }

  if (snapshot.gnss_pos_update.total > 0) {
    snapshot.gnss_pos_update.ratio =
        static_cast<double>(snapshot.gnss_pos_update.applied) /
        static_cast<double>(snapshot.gnss_pos_update.total);
  }
  if (snapshot.gnss_vel_update.total > 0) {
    snapshot.gnss_vel_update.ratio =
        static_cast<double>(snapshot.gnss_vel_update.applied) /
        static_cast<double>(snapshot.gnss_vel_update.total);
  }
  if (snapshot.heading_yaw_update.total > 0) {
    snapshot.heading_yaw_update.ratio =
        static_cast<double>(snapshot.heading_yaw_update.applied) /
        static_cast<double>(snapshot.heading_yaw_update.total);
  }

  snapshot.gnss_pos_nis = ComputeStats(pos_nis);
  snapshot.gnss_vel_nis = ComputeStats(vel_nis);
  snapshot.heading_yaw_nis = ComputeStats(heading_nis);
  snapshot.gnss_delay = ComputeStats(gnss_delay);
  snapshot.gnss_vel_delay = ComputeStats(gnss_vel_delay);
  snapshot.velocity_delay = ComputeStats(velocity_delay);
  snapshot.steering_delay = ComputeStats(steering_delay);

  snapshot.output_availability.output_count = output_stamps_.size();
  snapshot.output_availability.expected_count = window_sec_ * expected_output_rate_hz_;
  if (snapshot.output_availability.expected_count > 0.0) {
    snapshot.output_availability.ratio = std::min(
        1.0,
        static_cast<double>(snapshot.output_availability.output_count) /
            snapshot.output_availability.expected_count);
  }
  if (!output_stamps_.empty()) {
    snapshot.output_availability.has_output = true;
    snapshot.output_availability.last_age_sec =
        (now - output_stamps_.back()).seconds();
  }
  if (window_sec_ > 0.0) {
    snapshot.diag_rate_hz =
        static_cast<double>(samples_.size()) / window_sec_;
    snapshot.output_rate_hz =
        static_cast<double>(output_stamps_.size()) / window_sec_;
  }

  return snapshot;
}

StatSummary KpiEngine::ComputeStats(const std::vector<double>& values)
{
  StatSummary stat;
  if (values.empty()) return stat;

  stat.count = values.size();
  stat.max = *std::max_element(values.begin(), values.end());
  stat.mean = std::accumulate(values.begin(), values.end(), 0.0) /
              static_cast<double>(values.size());

  std::vector<double> copy = values;
  const double rank = std::ceil(0.95 * static_cast<double>(copy.size()));
  const size_t idx = (rank <= 1.0) ? 0 : static_cast<size_t>(rank - 1.0);
  std::nth_element(copy.begin(), copy.begin() + idx, copy.end());
  stat.p95 = copy[idx];

  return stat;
}

}  // namespace autodriver::tools
