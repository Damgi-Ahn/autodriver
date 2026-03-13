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

void RosQtBridge::UpdateGnssPose(const PoseSnapshot& pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_gnss_pose_ = pose;
}

void RosQtBridge::UpdateEskfPose(const PoseSnapshot& pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_eskf_pose_ = pose;
}

void RosQtBridge::UpdateFgoPose(const PoseSnapshot& pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_fgo_pose_ = pose;
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

bool RosQtBridge::GetLatestGnssPose(PoseSnapshot* out_pose) const
{
  if (!out_pose) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out_pose = latest_gnss_pose_;
  return out_pose->has_pose;
}

bool RosQtBridge::GetLatestEskfPose(PoseSnapshot* out_pose) const
{
  if (!out_pose) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out_pose = latest_eskf_pose_;
  return out_pose->has_pose;
}

bool RosQtBridge::GetLatestFgoPose(PoseSnapshot* out_pose) const
{
  if (!out_pose) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out_pose = latest_fgo_pose_;
  return out_pose->has_pose;
}

}  // namespace autodriver::tools
