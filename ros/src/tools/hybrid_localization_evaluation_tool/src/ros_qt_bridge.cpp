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

void RosQtBridge::UpdateLocalizationState(const std::string& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  localization_state_ = state;
}

void RosQtBridge::AddAlerts(const std::vector<AlertEvent>& events)
{
  if (events.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& e : events) event_log_.push_back(e);
}

void RosQtBridge::UpdateTrajectories(const SessionStore& store)
{
  // SessionStore is already thread-safe; get copies, then lock bridge
  auto gnss = store.GetGnssTrajectory();
  auto eskf = store.GetEskfTrajectory();
  auto fgo  = store.GetFgoTrajectory();
  auto hist = store.GetSampleHistory();

  std::lock_guard<std::mutex> lock(mutex_);
  gnss_traj_      = std::move(gnss);
  eskf_traj_      = std::move(eskf);
  fgo_traj_       = std::move(fgo);
  sample_history_ = std::move(hist);
}

void RosQtBridge::UpdateHealthState(HealthState state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  health_state_ = state;
}

void RosQtBridge::UpdateGtData(const GroundTruthAnalyzer& analyzer)
{
  // Acquire all GT data outside bridge lock (each getter is independently thread-safe)
  const bool     has_gt      = analyzer.has_gt();
  const AteStats eskf_ate    = analyzer.GetEskfAte();
  const AteStats fgo_ate     = analyzer.GetFgoAte();
  const RpeStats eskf_rpe_1s = analyzer.ComputeEskfRpe(1.0);
  const RpeStats eskf_rpe_5s = analyzer.ComputeEskfRpe(5.0);
  const RpeStats fgo_rpe_1s  = analyzer.ComputeFgoRpe(1.0);
  const RpeStats fgo_rpe_5s  = analyzer.ComputeFgoRpe(5.0);
  auto history               = analyzer.GetErrorHistory();
  auto colored               = analyzer.GetEskfColoredTraj();

  std::lock_guard<std::mutex> lock(mutex_);
  has_gt_       = has_gt;
  eskf_ate_     = eskf_ate;
  fgo_ate_      = fgo_ate;
  eskf_rpe_1s_  = eskf_rpe_1s;
  eskf_rpe_5s_  = eskf_rpe_5s;
  fgo_rpe_1s_   = fgo_rpe_1s;
  fgo_rpe_5s_   = fgo_rpe_5s;
  gt_errors_    = std::move(history);
  eskf_colored_ = std::move(colored);
}

void RosQtBridge::UpdateAcfData(const AcfSnapshot& acf)
{
  std::lock_guard<std::mutex> lock(mutex_);
  acf_ = acf;
}

BridgeData RosQtBridge::Snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  BridgeData d;
  d.has_sample         = has_sample_;
  d.sample             = latest_sample_;
  d.has_kpi            = has_kpi_;
  d.kpi                = latest_kpi_;
  d.gnss_pose          = latest_gnss_pose_;
  d.eskf_pose          = latest_eskf_pose_;
  d.fgo_pose           = latest_fgo_pose_;
  d.localization_state = localization_state_;
  d.health_state       = health_state_;
  d.has_gt             = has_gt_;
  d.eskf_ate           = eskf_ate_;
  d.fgo_ate            = fgo_ate_;
  d.eskf_rpe_1s        = eskf_rpe_1s_;
  d.eskf_rpe_5s        = eskf_rpe_5s_;
  d.fgo_rpe_1s         = fgo_rpe_1s_;
  d.fgo_rpe_5s         = fgo_rpe_5s_;
  d.acf                = acf_;
  d.gt_error_history   = gt_errors_;
  d.eskf_colored_traj  = eskf_colored_;
  d.event_log          = event_log_;
  d.gnss_traj          = gnss_traj_;
  d.eskf_traj          = eskf_traj_;
  d.fgo_traj           = fgo_traj_;
  d.sample_history     = sample_history_;
  return d;
}

bool RosQtBridge::GetLatestSample(DiagSample* out) const
{
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_sample_) return false;
  *out = latest_sample_;
  return true;
}

bool RosQtBridge::GetLatestKpi(KpiSnapshot* out) const
{
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_kpi_) return false;
  *out = latest_kpi_;
  return true;
}

bool RosQtBridge::GetLatestGnssPose(PoseSnapshot* out) const
{
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out = latest_gnss_pose_;
  return out->has_pose;
}

bool RosQtBridge::GetLatestEskfPose(PoseSnapshot* out) const
{
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out = latest_eskf_pose_;
  return out->has_pose;
}

bool RosQtBridge::GetLatestFgoPose(PoseSnapshot* out) const
{
  if (!out) return false;
  std::lock_guard<std::mutex> lock(mutex_);
  *out = latest_fgo_pose_;
  return out->has_pose;
}

}  // namespace autodriver::tools
