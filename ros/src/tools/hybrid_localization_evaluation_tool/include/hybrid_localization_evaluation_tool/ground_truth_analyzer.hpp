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
  double mean_m = 0.0;
  double rmse_m = 0.0;
  double max_m  = 0.0;
};

// ---------------------------------------------------------------------------
// Relative Pose Error statistics (SE2, 2D)
// ---------------------------------------------------------------------------
struct RpeStats {
  size_t count        = 0;
  double trans_mean_m = 0.0;
  double trans_rmse_m = 0.0;
  double trans_max_m  = 0.0;
  double rot_mean_deg = 0.0;
  double rot_rmse_deg = 0.0;
  double rot_max_deg  = 0.0;
  double delta_sec    = 1.0;  // Δ used to compute this
};

// ---------------------------------------------------------------------------
// Per-sample GT error record
// ---------------------------------------------------------------------------
struct GtErrorSample {
  rclcpp::Time stamp;
  double eskf_ate_m    = 0.0;
  double fgo_ate_m     = 0.0;
  double yaw_error_deg = 0.0;
};

// ---------------------------------------------------------------------------
// Trajectory point with ATE colour value (for heatmap)
// ---------------------------------------------------------------------------
struct ColoredTrajectoryPoint {
  double x     = 0.0;
  double y     = 0.0;
  double ate_m = 0.0;
};

// ---------------------------------------------------------------------------
// GroundTruthAnalyzer
//   • ATE: per-sample and running stats for ESKF and FGO
//   • RPE: SE2 relative-pose error at configurable delta (1 s, 5 s, 10 s)
//   • Heatmap: ESKF trajectory coloured by ATE magnitude
// ---------------------------------------------------------------------------
class GroundTruthAnalyzer {
 public:
  static constexpr size_t kRingSize     = 6000;  // 10 min @ 10 Hz
  static constexpr size_t kPoseBufSize  = 1200;  // 2  min @ 10 Hz

  // ROS thread: update estimated poses (call from OnEskfOdom / OnFgoPose)
  void UpdateEskfPose(const rclcpp::Time& stamp, double x, double y, double yaw_rad);
  void UpdateFgoPose (const rclcpp::Time& stamp, double x, double y, double yaw_rad);

  // ROS thread: new ground-truth sample
  void AddGroundTruth(const rclcpp::Time& stamp, double x, double y, double yaw_rad);

  // Qt thread: ATE stats
  AteStats GetEskfAte() const;
  AteStats GetFgoAte()  const;

  // Qt thread: RPE at given delta — lazy O(N²) over pose ring buffer
  RpeStats ComputeEskfRpe(double delta_sec) const;
  RpeStats ComputeFgoRpe (double delta_sec) const;

  // Qt thread: per-sample error history (for ATE chart)
  std::vector<GtErrorSample>          GetErrorHistory()     const;

  // Qt thread: ESKF trajectory with ATE colour (for heatmap)
  std::vector<ColoredTrajectoryPoint> GetEskfColoredTraj()  const;

  bool has_gt() const;
  void Clear();

 private:
  // ---- Internal timestamped pose record -----------------------------------
  struct StampedPose {
    rclcpp::Time stamp;
    double x = 0.0, y = 0.0, yaw_rad = 0.0;
  };

  // SE2 relative motion (in local frame of p0)
  struct RelMotion { double dx, dy, dyaw; };
  static RelMotion ComputeRelMotion(const StampedPose& p0, const StampedPose& p1);

  // Find nearest pose in buf to stamp t; returns nullptr if no match within max_diff_sec
  static const StampedPose* FindNearest(const std::deque<StampedPose>& buf,
                                        const rclcpp::Time& t,
                                        double max_diff_sec = 0.5);

  // Find pose in buf whose stamp is closest to (anchor.stamp + delta_sec)
  static const StampedPose* FindAtDelta(const std::deque<StampedPose>& buf,
                                        const rclcpp::Time& anchor,
                                        double delta_sec,
                                        double tolerance_sec = 0.5);

  RpeStats ComputeRpe(const std::deque<StampedPose>& est_buf, double delta_sec) const;

  mutable std::mutex mutex_;

  // Latest pose (for ATE — no timestamp needed)
  bool   has_eskf_  = false;
  double eskf_x_    = 0.0, eskf_y_    = 0.0, eskf_yaw_    = 0.0;
  bool   has_fgo_   = false;
  double fgo_x_     = 0.0, fgo_y_     = 0.0, fgo_yaw_     = 0.0;
  bool   has_gt_    = false;

  // Timestamped pose ring buffers (for RPE)
  std::deque<StampedPose> gt_pose_buf_;
  std::deque<StampedPose> eskf_pose_buf_;
  std::deque<StampedPose> fgo_pose_buf_;

  // ATE error history
  std::deque<GtErrorSample>          errors_;

  // Coloured trajectory
  std::deque<ColoredTrajectoryPoint> eskf_colored_;

  // Running ATE accumulators
  size_t eskf_count_ = 0;
  double eskf_sum_ = 0.0, eskf_sum_sq_ = 0.0, eskf_max_ = 0.0;
  size_t fgo_count_ = 0;
  double fgo_sum_  = 0.0, fgo_sum_sq_  = 0.0, fgo_max_  = 0.0;
};

}  // namespace autodriver::tools
