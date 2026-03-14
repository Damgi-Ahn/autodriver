#include "hybrid_localization_evaluation_tool/ground_truth_analyzer.hpp"

#include <cmath>

namespace autodriver::tools {

namespace {
constexpr double kDeg = 180.0 / M_PI;

double WrapPi(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

void GroundTruthAnalyzer::UpdateEskfPose(double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  eskf_x_ = x; eskf_y_ = y; eskf_yaw_ = yaw_rad;
  has_eskf_ = true;
}

void GroundTruthAnalyzer::UpdateFgoPose(double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fgo_x_ = x; fgo_y_ = y; fgo_yaw_ = yaw_rad;
  has_fgo_ = true;
}

void GroundTruthAnalyzer::AddGroundTruth(const rclcpp::Time& stamp,
                                          double x, double y, double yaw_rad)
{
  std::lock_guard<std::mutex> lock(mutex_);
  has_gt_ = true;

  GtErrorSample sample;
  sample.stamp = stamp;

  // ESKF ATE
  if (has_eskf_) {
    const double dx = x - eskf_x_;
    const double dy = y - eskf_y_;
    sample.eskf_ate_m    = std::sqrt(dx * dx + dy * dy);
    sample.yaw_error_deg = WrapPi(yaw_rad - eskf_yaw_) * kDeg;

    eskf_sum_    += sample.eskf_ate_m;
    eskf_sum_sq_ += sample.eskf_ate_m * sample.eskf_ate_m;
    eskf_max_     = std::max(eskf_max_, sample.eskf_ate_m);
    ++eskf_count_;
  }

  // FGO ATE
  if (has_fgo_) {
    const double dx = x - fgo_x_;
    const double dy = y - fgo_y_;
    sample.fgo_ate_m = std::sqrt(dx * dx + dy * dy);

    fgo_sum_    += sample.fgo_ate_m;
    fgo_sum_sq_ += sample.fgo_ate_m * sample.fgo_ate_m;
    fgo_max_     = std::max(fgo_max_, sample.fgo_ate_m);
    ++fgo_count_;
  }

  errors_.push_back(sample);
  while (errors_.size() > kRingSize) errors_.pop_front();
}

AteStats GroundTruthAnalyzer::GetEskfAte() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (eskf_count_ == 0) return {};
  AteStats s;
  s.count  = eskf_count_;
  s.mean_m = eskf_sum_ / static_cast<double>(eskf_count_);
  s.rmse_m = std::sqrt(eskf_sum_sq_ / static_cast<double>(eskf_count_));
  s.max_m  = eskf_max_;
  return s;
}

AteStats GroundTruthAnalyzer::GetFgoAte() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (fgo_count_ == 0) return {};
  AteStats s;
  s.count  = fgo_count_;
  s.mean_m = fgo_sum_ / static_cast<double>(fgo_count_);
  s.rmse_m = std::sqrt(fgo_sum_sq_ / static_cast<double>(fgo_count_));
  s.max_m  = fgo_max_;
  return s;
}

std::vector<GtErrorSample> GroundTruthAnalyzer::GetErrorHistory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return std::vector<GtErrorSample>(errors_.begin(), errors_.end());
}

bool GroundTruthAnalyzer::has_gt() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return has_gt_;
}

void GroundTruthAnalyzer::Clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  errors_.clear();
  has_gt_ = false; has_eskf_ = false; has_fgo_ = false;
  eskf_count_ = 0; eskf_sum_ = 0.0; eskf_sum_sq_ = 0.0; eskf_max_ = 0.0;
  fgo_count_  = 0; fgo_sum_  = 0.0; fgo_sum_sq_  = 0.0; fgo_max_  = 0.0;
}

}  // namespace autodriver::tools
