#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <mutex>

namespace autodriver::tools {

struct PoseSnapshot {
  bool has_pose = false;
  rclcpp::Time stamp;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw_rad = 0.0;
};

class RosQtBridge {
 public:
  void UpdateSample(const DiagSample& sample);
  void UpdateKpi(const KpiSnapshot& snapshot);
  void UpdateGnssPose(const PoseSnapshot& pose);
  void UpdateEskfPose(const PoseSnapshot& pose);
  void UpdateFgoPose(const PoseSnapshot& pose);

  bool GetLatestSample(DiagSample* out_sample) const;
  bool GetLatestKpi(KpiSnapshot* out_snapshot) const;
  bool GetLatestGnssPose(PoseSnapshot* out_pose) const;
  bool GetLatestEskfPose(PoseSnapshot* out_pose) const;
  bool GetLatestFgoPose(PoseSnapshot* out_pose) const;

 private:
  mutable std::mutex mutex_;
  bool has_sample_ = false;
  bool has_kpi_ = false;
  DiagSample latest_sample_;
  KpiSnapshot latest_kpi_;
  PoseSnapshot latest_gnss_pose_;
  PoseSnapshot latest_eskf_pose_;
  PoseSnapshot latest_fgo_pose_;
};

}  // namespace autodriver::tools
