#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <memory>

namespace autodriver::tools {

class OverviewTab : public QWidget {
  Q_OBJECT
 public:
  explicit OverviewTab(const std::shared_ptr<RosQtBridge>& bridge,
                       QWidget* parent = nullptr);

  void Refresh(const BridgeData& data);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  // Status banner
  QLabel* banner_label_         = nullptr;
  QLabel* state_label_          = nullptr;
  QLabel* activation_label_     = nullptr;
  QLabel* eskf_init_label_      = nullptr;

  // KPI row
  QLabel* output_rate_label_    = nullptr;
  QLabel* availability_label_   = nullptr;
  QLabel* diag_rate_label_      = nullptr;
  QLabel* last_output_age_label_ = nullptr;

  // Red flag alert cards
  QLabel* nis_flag_label_       = nullptr;
  QLabel* delay_flag_label_     = nullptr;
  QLabel* cov_flag_label_       = nullptr;

  // Mini trend charts
  QLineSeries* nis_pos_mini_    = nullptr;
  QLineSeries* delay_gnss_mini_ = nullptr;
  QLineSeries* P_trace_mini_    = nullptr;

  double t_start_ = -1.0;
};

}  // namespace autodriver::tools
