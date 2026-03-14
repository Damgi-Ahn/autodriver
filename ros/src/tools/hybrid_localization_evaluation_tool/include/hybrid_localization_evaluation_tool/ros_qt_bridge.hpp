#pragma once

#include "hybrid_localization_evaluation_tool/alert_engine.hpp"
#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/ground_truth_analyzer.hpp"
#include "hybrid_localization_evaluation_tool/health_state_engine.hpp"
#include "hybrid_localization_evaluation_tool/innovation_analyzer.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/session_store.hpp"

#include <mutex>
#include <string>
#include <vector>

namespace autodriver::tools {

struct PoseSnapshot {
  bool has_pose  = false;
  rclcpp::Time stamp;
  double x       = 0.0;
  double y       = 0.0;
  double z       = 0.0;
  double yaw_rad = 0.0;
};

// ---------------------------------------------------------------------------
// BridgeData — full snapshot consumed by UI each refresh cycle
// ---------------------------------------------------------------------------
struct BridgeData {
  bool       has_sample = false;
  DiagSample sample;

  bool        has_kpi = false;
  KpiSnapshot kpi;

  PoseSnapshot gnss_pose;
  PoseSnapshot eskf_pose;
  PoseSnapshot fgo_pose;

  std::string localization_state;

  // Health state machine output
  HealthState health_state = HealthState::OK;

  // Ground truth error analysis
  bool                       has_gt          = false;
  AteStats                   eskf_ate;
  AteStats                   fgo_ate;
  std::vector<GtErrorSample> gt_error_history;

  // RPE (Relative Pose Error)
  RpeStats eskf_rpe_1s;
  RpeStats eskf_rpe_5s;
  RpeStats fgo_rpe_1s;
  RpeStats fgo_rpe_5s;

  // Innovation ACF analysis
  AcfSnapshot acf;

  // Trajectory heatmap (ESKF coloured by ATE)
  std::vector<ColoredTrajectoryPoint> eskf_colored_traj;

  std::vector<AlertEvent>      event_log;
  std::vector<TrajectoryPoint> gnss_traj;
  std::vector<TrajectoryPoint> eskf_traj;
  std::vector<TrajectoryPoint> fgo_traj;
  std::vector<DiagSample>      sample_history;
};

// ---------------------------------------------------------------------------
// RosQtBridge — thread-safe ROS→Qt data handoff
// ---------------------------------------------------------------------------
class RosQtBridge {
 public:
  void UpdateSample(const DiagSample& sample);
  void UpdateKpi(const KpiSnapshot& snapshot);
  void UpdateGnssPose(const PoseSnapshot& pose);
  void UpdateEskfPose(const PoseSnapshot& pose);
  void UpdateFgoPose(const PoseSnapshot& pose);
  void UpdateLocalizationState(const std::string& state);
  void AddAlerts(const std::vector<AlertEvent>& events);
  void UpdateTrajectories(const SessionStore& store);
  void UpdateHealthState(HealthState state);
  void UpdateGtData(const GroundTruthAnalyzer& analyzer);
  void UpdateAcfData(const AcfSnapshot& acf);

  // Qt thread: atomically copy everything into BridgeData
  BridgeData Snapshot() const;

  // Compatibility helpers
  bool GetLatestSample(DiagSample* out) const;
  bool GetLatestKpi(KpiSnapshot* out) const;
  bool GetLatestGnssPose(PoseSnapshot* out) const;
  bool GetLatestEskfPose(PoseSnapshot* out) const;
  bool GetLatestFgoPose(PoseSnapshot* out) const;

 private:
  mutable std::mutex mutex_;

  bool        has_sample_ = false;
  bool        has_kpi_    = false;
  DiagSample  latest_sample_;
  KpiSnapshot latest_kpi_;
  PoseSnapshot latest_gnss_pose_;
  PoseSnapshot latest_eskf_pose_;
  PoseSnapshot latest_fgo_pose_;
  std::string  localization_state_;
  HealthState  health_state_ = HealthState::OK;
  bool         has_gt_       = false;
  AteStats     eskf_ate_;
  AteStats     fgo_ate_;
  RpeStats     eskf_rpe_1s_,  eskf_rpe_5s_;
  RpeStats     fgo_rpe_1s_,   fgo_rpe_5s_;
  AcfSnapshot  acf_;
  std::vector<GtErrorSample>          gt_errors_;
  std::vector<ColoredTrajectoryPoint> eskf_colored_;
  std::vector<AlertEvent>      event_log_;
  std::vector<TrajectoryPoint> gnss_traj_;
  std::vector<TrajectoryPoint> eskf_traj_;
  std::vector<TrajectoryPoint> fgo_traj_;
  std::vector<DiagSample>      sample_history_;
};

}  // namespace autodriver::tools
