#include "hybrid_localization_evaluation_tool/ground_truth_analyzer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autodriver::tools {

namespace {

constexpr double kDeg = 180.0 / M_PI;

double WrapPi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

template<typename T>
void PushRing(std::deque<T>& d, T val, size_t max_size)
{
  d.push_back(std::move(val));
  while (d.size() > max_size) d.pop_front();
}

}  // namespace

// ---- Pose update methods (ROS thread) ----------------------------------------

void GroundTruthAnalyzer::UpdateEskfPose(const rclcpp::Time& stamp,
                                          double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_eskf_ = true;
  eskf_x_ = x; eskf_y_ = y; eskf_yaw_ = yaw_rad;
  PushRing(eskf_pose_buf_, StampedPose{stamp, x, y, yaw_rad}, kPoseBufSize);
}

void GroundTruthAnalyzer::UpdateFgoPose(const rclcpp::Time& stamp,
                                         double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_fgo_ = true;
  fgo_x_ = x; fgo_y_ = y; fgo_yaw_ = yaw_rad;
  PushRing(fgo_pose_buf_, StampedPose{stamp, x, y, yaw_rad}, kPoseBufSize);
}

void GroundTruthAnalyzer::AddGroundTruth(const rclcpp::Time& stamp,
                                          double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_gt_ = true;
  PushRing(gt_pose_buf_, StampedPose{stamp, x, y, yaw_rad}, kPoseBufSize);

  GtErrorSample sample;
  sample.stamp = stamp;

  // ESKF ATE
  if (has_eskf_) {
    const double dx = x - eskf_x_, dy = y - eskf_y_;
    sample.eskf_ate_m    = std::sqrt(dx * dx + dy * dy);
    sample.yaw_error_deg = WrapPi(yaw_rad - eskf_yaw_) * kDeg;

    eskf_sum_    += sample.eskf_ate_m;
    eskf_sum_sq_ += sample.eskf_ate_m * sample.eskf_ate_m;
    eskf_max_     = std::max(eskf_max_, sample.eskf_ate_m);
    ++eskf_count_;

    // Coloured trajectory point
    PushRing(eskf_colored_,
             ColoredTrajectoryPoint{eskf_x_, eskf_y_, sample.eskf_ate_m},
             kRingSize);
  }

  // FGO ATE
  if (has_fgo_) {
    const double dx = x - fgo_x_, dy = y - fgo_y_;
    sample.fgo_ate_m = std::sqrt(dx * dx + dy * dy);
    fgo_sum_    += sample.fgo_ate_m;
    fgo_sum_sq_ += sample.fgo_ate_m * sample.fgo_ate_m;
    fgo_max_     = std::max(fgo_max_, sample.fgo_ate_m);
    ++fgo_count_;
  }

  PushRing(errors_, sample, kRingSize);
}

// ---- Qt thread query methods -------------------------------------------------

AteStats GroundTruthAnalyzer::GetEskfAte() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (eskf_count_ == 0) return {};
  const double n = static_cast<double>(eskf_count_);
  return {eskf_count_, eskf_sum_ / n, std::sqrt(eskf_sum_sq_ / n), eskf_max_};
}

AteStats GroundTruthAnalyzer::GetFgoAte() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fgo_count_ == 0) return {};
  const double n = static_cast<double>(fgo_count_);
  return {fgo_count_, fgo_sum_ / n, std::sqrt(fgo_sum_sq_ / n), fgo_max_};
}

std::vector<GtErrorSample> GroundTruthAnalyzer::GetErrorHistory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {errors_.begin(), errors_.end()};
}

std::vector<ColoredTrajectoryPoint> GroundTruthAnalyzer::GetEskfColoredTraj() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {eskf_colored_.begin(), eskf_colored_.end()};
}

bool GroundTruthAnalyzer::has_gt() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_gt_;
}

void GroundTruthAnalyzer::Clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  errors_.clear(); eskf_colored_.clear();
  gt_pose_buf_.clear(); eskf_pose_buf_.clear(); fgo_pose_buf_.clear();
  has_gt_ = false; has_eskf_ = false; has_fgo_ = false;
  eskf_count_ = 0; eskf_sum_ = 0.0; eskf_sum_sq_ = 0.0; eskf_max_ = 0.0;
  fgo_count_  = 0; fgo_sum_  = 0.0; fgo_sum_sq_  = 0.0; fgo_max_  = 0.0;
}

// ---- RPE helpers -------------------------------------------------------------

GroundTruthAnalyzer::RelMotion GroundTruthAnalyzer::ComputeRelMotion(
    const StampedPose& p0, const StampedPose& p1)
{
  const double c  = std::cos(p0.yaw_rad), s = std::sin(p0.yaw_rad);
  const double dxg = p1.x - p0.x, dyg = p1.y - p0.y;
  return {c * dxg + s * dyg, -s * dxg + c * dyg, WrapPi(p1.yaw_rad - p0.yaw_rad)};
}

const GroundTruthAnalyzer::StampedPose* GroundTruthAnalyzer::FindNearest(
    const std::deque<StampedPose>& buf, const rclcpp::Time& t, double max_diff_sec)
{
  const StampedPose* best = nullptr;
  double best_diff = std::numeric_limits<double>::max();
  for (const auto& p : buf) {
    const double diff = std::abs((p.stamp - t).seconds());
    if (diff < best_diff) { best_diff = diff; best = &p; }
  }
  return (best_diff <= max_diff_sec) ? best : nullptr;
}

const GroundTruthAnalyzer::StampedPose* GroundTruthAnalyzer::FindAtDelta(
    const std::deque<StampedPose>& buf, const rclcpp::Time& anchor,
    double delta_sec, double tolerance_sec)
{
  const rclcpp::Time target = anchor + rclcpp::Duration::from_seconds(delta_sec);
  return FindNearest(buf, target, tolerance_sec);
}

RpeStats GroundTruthAnalyzer::ComputeRpe(const std::deque<StampedPose>& est_buf,
                                          double delta_sec) const
{
  RpeStats stats;
  stats.delta_sec = delta_sec;
  if (gt_pose_buf_.size() < 2 || est_buf.size() < 2) return stats;

  double t_sum  = 0.0, t_sq = 0.0, t_max = 0.0;
  double r_sum  = 0.0, r_sq = 0.0, r_max = 0.0;

  for (size_t i = 0; i < gt_pose_buf_.size(); ++i) {
    const auto& gt0 = gt_pose_buf_[i];

    // Find GT at t + delta
    const StampedPose* gt1_ptr = FindAtDelta(gt_pose_buf_, gt0.stamp, delta_sec);
    if (!gt1_ptr) continue;
    const auto& gt1 = *gt1_ptr;

    // Find ESKF nearest to gt0 and gt1
    const StampedPose* e0_ptr = FindNearest(est_buf, gt0.stamp);
    const StampedPose* e1_ptr = FindNearest(est_buf, gt1.stamp);
    if (!e0_ptr || !e1_ptr) continue;

    // Relative motions
    const auto rel_gt  = ComputeRelMotion(gt0, gt1);
    const auto rel_est = ComputeRelMotion(*e0_ptr, *e1_ptr);

    const double te = std::sqrt((rel_gt.dx - rel_est.dx) * (rel_gt.dx - rel_est.dx)
                              + (rel_gt.dy - rel_est.dy) * (rel_gt.dy - rel_est.dy));
    const double re = std::abs(WrapPi(rel_gt.dyaw - rel_est.dyaw)) * kDeg;

    t_sum += te; t_sq += te * te; t_max = std::max(t_max, te);
    r_sum += re; r_sq += re * re; r_max = std::max(r_max, re);
    ++stats.count;
  }

  if (stats.count > 0) {
    const double n = static_cast<double>(stats.count);
    stats.trans_mean_m = t_sum / n;
    stats.trans_rmse_m = std::sqrt(t_sq / n);
    stats.trans_max_m  = t_max;
    stats.rot_mean_deg = r_sum / n;
    stats.rot_rmse_deg = std::sqrt(r_sq / n);
    stats.rot_max_deg  = r_max;
  }
  return stats;
}

RpeStats GroundTruthAnalyzer::ComputeEskfRpe(double delta_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ComputeRpe(eskf_pose_buf_, delta_sec);
}

RpeStats GroundTruthAnalyzer::ComputeFgoRpe(double delta_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return ComputeRpe(fgo_pose_buf_, delta_sec);
}

}  // namespace autodriver::tools
