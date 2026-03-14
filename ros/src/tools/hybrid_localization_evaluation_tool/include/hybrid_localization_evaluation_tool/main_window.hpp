#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"
#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"
#include "hybrid_localization_evaluation_tool/ui/covariance_noise_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/events_inspector_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/export_session_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/fusion_quality_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/overview_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/pose_trajectory_tab.hpp"
#include "hybrid_localization_evaluation_tool/ui/sensor_timing_tab.hpp"

#include <QLabel>
#include <QMainWindow>
#include <QElapsedTimer>
#include <QTabWidget>
#include <QTimer>

#include <functional>
#include <memory>

namespace autodriver::tools {

class EvaluationMainWindow : public QMainWindow {
 public:
  explicit EvaluationMainWindow(const std::shared_ptr<RosQtBridge>& bridge,
                                StorageExporter* exporter = nullptr,
                                QWidget* parent = nullptr);

  // Propagate NIS gate values to the fusion quality tab
  void SetNisGates(double pos, double vel, double heading);

  // Callbacks forwarded to export tab
  void SetOnFlushSessionSummary(std::function<void()> cb);
  void SetOnClearSession(std::function<void()> cb);

 private:
  void UpdateUi();

  std::shared_ptr<RosQtBridge> bridge_;

  QTabWidget* tabs_ = nullptr;

  OverviewTab*         tab_overview_    = nullptr;
  FusionQualityTab*    tab_fusion_      = nullptr;
  CovarianceNoiseTab*  tab_covariance_  = nullptr;
  SensorTimingTab*     tab_timing_      = nullptr;
  PoseTrajectoryTab*   tab_trajectory_  = nullptr;
  EventsInspectorTab*  tab_events_      = nullptr;
  ExportSessionTab*    tab_export_      = nullptr;

  // Status bar widgets
  QLabel* hz_label_  = nullptr;
  QLabel* cpu_label_ = nullptr;

  QTimer*       timer_ = nullptr;
  QElapsedTimer cpu_timer_;
  double        last_cpu_seconds_ = 0.0;
  int           cpu_cores_        = 1;
};

}  // namespace autodriver::tools
