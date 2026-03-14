#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <memory>

namespace autodriver::tools {

class SensorTimingTab : public QWidget {
  Q_OBJECT
 public:
  explicit SensorTimingTab(const std::shared_ptr<RosQtBridge>& bridge,
                           QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  // IMU dt
  QLabel* imu_dt_min_label_  = nullptr;
  QLabel* imu_dt_mean_label_ = nullptr;
  QLabel* imu_dt_max_label_  = nullptr;
  QLabel* imu_jitter_label_  = nullptr;

  // Last message age
  QLabel* gnss_age_label_     = nullptr;
  QLabel* gnss_vel_age_label_ = nullptr;
  QLabel* vel_age_label_      = nullptr;
  QLabel* steer_age_label_    = nullptr;

  // Delay charts
  QLineSeries* delay_gnss_series_     = nullptr;
  QLineSeries* delay_gnss_vel_series_ = nullptr;
  QLineSeries* delay_vel_series_      = nullptr;
  QLineSeries* delay_steer_series_    = nullptr;

  double t_start_ = -1.0;
};

}  // namespace autodriver::tools
