#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

namespace autodriver::tools {

void RosQtBridge::UpdateSample(const DiagSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_sample_ = sample;
  has_sample_ = true;
}

void RosQtBridge::UpdateKpi(const KpiSnapshot& snapshot)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_kpi_ = snapshot;
  has_kpi_ = true;
}

bool RosQtBridge::GetLatestSample(DiagSample* out_sample) const
{
  if (!out_sample) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_sample_) return false;
  *out_sample = latest_sample_;
  return true;
}

bool RosQtBridge::GetLatestKpi(KpiSnapshot* out_snapshot) const
{
  if (!out_snapshot) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_kpi_) return false;
  *out_snapshot = latest_kpi_;
  return true;
}

}  // namespace autodriver::tools
