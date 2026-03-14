#include "hybrid_localization_evaluation_tool/innovation_analyzer.hpp"

#include <cmath>
#include <numeric>

namespace autodriver::tools {

namespace {

void PushRingBuffer(std::deque<double>& buf, double val, size_t max_size)
{
  buf.push_back(val);
  if (buf.size() > max_size) buf.pop_front();
}

}  // namespace

void InnovationAnalyzer::AddSample(const DiagSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (sample.gnss_pos_residual_norm.has_value())
    PushRingBuffer(pos_residuals_,     *sample.gnss_pos_residual_norm, kWindowSize);
  if (sample.gnss_vel_residual_norm.has_value())
    PushRingBuffer(vel_residuals_,     *sample.gnss_vel_residual_norm, kWindowSize);
  if (sample.heading_yaw_residual_rad.has_value())
    PushRingBuffer(heading_residuals_, std::abs(*sample.heading_yaw_residual_rad), kWindowSize);
}

AcfValues InnovationAnalyzer::ComputeChannel(const std::deque<double>& data)
{
  AcfValues result;
  const int N = static_cast<int>(data.size());
  if (N < kMaxLag + 2) return result;

  // Mean
  double mean = 0.0;
  for (double v : data) mean += v;
  mean /= N;

  // Variance (lag-0 covariance)
  double var = 0.0;
  for (double v : data) var += (v - mean) * (v - mean);
  if (var < 1e-12) return result;

  // ACF for lag 1..max_lag
  result.significance = 1.96 / std::sqrt(static_cast<double>(N));
  for (int lag = 1; lag <= kMaxLag && lag < N; ++lag) {
    double cov = 0.0;
    for (int i = 0; i < N - lag; ++i) {
      cov += (data[i] - mean) * (data[i + lag] - mean);
    }
    const double acf_val = cov / var;
    result.lags.push_back(static_cast<double>(lag));
    result.acf.push_back(acf_val);
    if (std::abs(acf_val) > result.significance) result.model_mismatch = true;
  }
  return result;
}

AcfSnapshot InnovationAnalyzer::Compute() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  AcfSnapshot snap;
  snap.gnss_pos = ComputeChannel(pos_residuals_);
  snap.gnss_vel = ComputeChannel(vel_residuals_);
  snap.heading  = ComputeChannel(heading_residuals_);
  snap.valid    = !snap.gnss_pos.acf.empty()
               || !snap.gnss_vel.acf.empty()
               || !snap.heading.acf.empty();
  return snap;
}

}  // namespace autodriver::tools
