#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <deque>
#include <mutex>
#include <vector>

namespace autodriver::tools {

// ---------------------------------------------------------------------------
// Per-channel ACF result
// ---------------------------------------------------------------------------
struct AcfValues {
  std::vector<double> lags;       // lag indices [1, 2, ..., max_lag]
  std::vector<double> acf;        // ACF(lag) normalized to [-1, 1]
  double significance = 0.0;      // ±1.96 / sqrt(N) significance threshold
  bool model_mismatch = false;    // true if any |ACF(lag)| > significance
};

struct AcfSnapshot {
  bool      valid    = false;
  AcfValues gnss_pos;
  AcfValues gnss_vel;
  AcfValues heading;
};

// ---------------------------------------------------------------------------
// InnovationAnalyzer
//   Maintains sliding window of measurement residuals and computes ACF.
//   ACF(k) = Σ(x_i - μ)(x_{i+k} - μ) / Σ(x_i - μ)²
//   If high lag correlation is detected → filter model mismatch.
// ---------------------------------------------------------------------------
class InnovationAnalyzer {
 public:
  static constexpr size_t kWindowSize = 100;  // 10 s @ 10 Hz
  static constexpr int    kMaxLag     = 20;

  void AddSample(const DiagSample& sample);

  // Compute ACF for all channels (thread-safe snapshot)
  AcfSnapshot Compute() const;

 private:
  static AcfValues ComputeChannel(const std::deque<double>& data);

  mutable std::mutex mutex_;
  std::deque<double> pos_residuals_;
  std::deque<double> vel_residuals_;
  std::deque<double> heading_residuals_;
};

}  // namespace autodriver::tools
