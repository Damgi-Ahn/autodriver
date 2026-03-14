#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QCheckBox>
#include <QLabel>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

#include <memory>

namespace autodriver::tools {

class PoseTrajectoryTab : public QWidget {
  Q_OBJECT
 public:
  explicit PoseTrajectoryTab(const std::shared_ptr<RosQtBridge>& bridge,
                             QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  // Pose cards
  QLabel* gnss_pos_label_   = nullptr;
  QLabel* gnss_yaw_label_   = nullptr;
  QLabel* eskf_pos_label_   = nullptr;
  QLabel* eskf_yaw_label_   = nullptr;
  QLabel* fgo_pos_label_    = nullptr;
  QLabel* fgo_yaw_label_    = nullptr;

  // Layer toggles
  QCheckBox* gnss_toggle_  = nullptr;
  QCheckBox* eskf_toggle_  = nullptr;
  QCheckBox* fgo_toggle_   = nullptr;

  // Trajectory series
  QScatterSeries* gnss_series_ = nullptr;
  QLineSeries*    eskf_series_ = nullptr;
  QLineSeries*    fgo_series_  = nullptr;

  QChartView* traj_view_ = nullptr;
};

}  // namespace autodriver::tools
