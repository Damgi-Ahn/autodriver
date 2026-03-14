#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <memory>

namespace autodriver::tools {

class CovarianceNoiseTab : public QWidget {
  Q_OBJECT
 public:
  explicit CovarianceNoiseTab(const std::shared_ptr<RosQtBridge>& bridge,
                              QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  // P stats cards
  QLabel* P_trace_label_    = nullptr;
  QLabel* P_min_eig_label_  = nullptr;
  QLabel* P_max_diag_label_ = nullptr;
  QLabel* P_pos_label_      = nullptr;
  QLabel* P_vel_label_      = nullptr;
  QLabel* P_att_label_      = nullptr;

  // R/Q cards
  QLabel* gnss_pos_R_label_  = nullptr;
  QLabel* gnss_vel_R_label_  = nullptr;
  QLabel* heading_var_label_ = nullptr;
  QLabel* vehicle_var_label_ = nullptr;
  QLabel* imu_Q_label_       = nullptr;

  // Inflation cards
  QLabel* pos_inflate_label_     = nullptr;
  QLabel* vel_inflate_label_     = nullptr;
  QLabel* heading_inflate_label_ = nullptr;

  // Charts
  QLineSeries* P_trace_series_   = nullptr;
  QLineSeries* P_min_eig_series_ = nullptr;

  double t_start_ = -1.0;
};

}  // namespace autodriver::tools
