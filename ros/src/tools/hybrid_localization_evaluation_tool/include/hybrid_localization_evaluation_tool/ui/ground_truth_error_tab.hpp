#pragma once

#include "hybrid_localization_evaluation_tool/ground_truth_analyzer.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QWidget>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <memory>

namespace autodriver::tools {

class GroundTruthErrorTab : public QWidget {
  Q_OBJECT
 public:
  explicit GroundTruthErrorTab(const std::shared_ptr<RosQtBridge>& bridge,
                               QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  // ESKF ATE stats
  QLabel* eskf_mean_label_ = nullptr;
  QLabel* eskf_rmse_label_ = nullptr;
  QLabel* eskf_max_label_  = nullptr;

  // FGO ATE stats
  QLabel* fgo_mean_label_  = nullptr;
  QLabel* fgo_rmse_label_  = nullptr;
  QLabel* fgo_max_label_   = nullptr;

  // Yaw error
  QLabel* yaw_error_label_ = nullptr;

  // GT status
  QLabel* gt_status_label_ = nullptr;
  QLabel* sample_count_label_ = nullptr;

  // ATE time series
  QLineSeries* eskf_ate_series_ = nullptr;
  QLineSeries* fgo_ate_series_  = nullptr;
  QLineSeries* yaw_err_series_  = nullptr;

  QChartView* ate_chart_view_  = nullptr;
  QChartView* yaw_chart_view_  = nullptr;

  double t_start_ = -1.0;
};

}  // namespace autodriver::tools
