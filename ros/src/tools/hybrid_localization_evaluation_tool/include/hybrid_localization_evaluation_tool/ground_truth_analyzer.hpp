#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <deque>
#include <mutex>
#include <vector>

namespace autodriver::tools {

// ---------------------------------------------------------------------------
// Absolute Trajectory Error statistics
// ---------------------------------------------------------------------------
struct AteStats {
  size_t count  = 0;
  double mean_m = 0.0;  // mean ATE [m]
  double rmse_m = 0.0;  // RMSE ATE [m]
  double max_m  = 0.0;  // max ATE [m]
};

// ---------------------------------------------------------------------------
// Per-sample ground truth error record
// ---------------------------------------------------------------------------
struct GtErrorSample {
  rclcpp::Time stamp;
  double eskf_ate_m    = 0.0;
  double fgo_ate_m     = 0.0;
  double yaw_error_deg = 0.0;  // GT yaw minus ESKF yaw [deg]
};

// ---------------------------------------------------------------------------
// GroundTruthAnalyzer
//   - Receives ground truth (GNSS RTK) and estimated poses
//   - Computes ATE (ESKF vs GT, FGO vs GT)
//   - Maintains a ring buffer of per-sample errors and running ATE stats
// ---------------------------------------------------------------------------
class GroundTruthAnalyzer {
 public:
  static constexpr size_t kRingSize = 6000;  // 10 min @ 10 Hz

  // Called from ROS thread on each new estimated pose
  void UpdateEskfPose(double x, double y, double yaw_rad);
  void UpdateFgoPose(double x, double y, double yaw_rad);

  // Called from ROS thread on each new ground truth sample
  void AddGroundTruth(const rclcpp::Time& stamp, double x, double y, double yaw_rad);

  // Called from Qt thread (returns snapshot copies)
  AteStats                   GetEskfAte()       const;
  AteStats                   GetFgoAte()        const;
  std::vector<GtErrorSample> GetErrorHistory()  const;
  bool                       has_gt()           const;

  void Clear();

 private:
  mutable std::mutex mutex_;

  // Latest estimated poses (updated at source rate)
  bool   has_eskf_   = false;
  double eskf_x_     = 0.0;
  double eskf_y_     = 0.0;
  double eskf_yaw_   = 0.0;

  bool   has_fgo_    = false;
  double fgo_x_      = 0.0;
  double fgo_y_      = 0.0;
  double fgo_yaw_    = 0.0;

  bool has_gt_ = false;

  // Ring buffer of per-sample errors
  std::deque<GtErrorSample> errors_;

  // Running accumulators for ATE stats
  size_t eskf_count_   = 0;
  double eskf_sum_     = 0.0;
  double eskf_sum_sq_  = 0.0;
  double eskf_max_     = 0.0;

  size_t fgo_count_    = 0;
  double fgo_sum_      = 0.0;
  double fgo_sum_sq_   = 0.0;
  double fgo_max_      = 0.0;
};

}  // namespace autodriver::tools
