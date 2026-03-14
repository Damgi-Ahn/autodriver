#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QWidget>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>

#include <memory>

namespace autodriver::tools {

class FusionQualityTab : public QWidget {
  Q_OBJECT
 public:
  explicit FusionQualityTab(const std::shared_ptr<RosQtBridge>& bridge,
                            QWidget* parent = nullptr);

  void Refresh(const BridgeData& data);
  void SetNisGates(double pos, double vel, double heading);

 private:
  std::shared_ptr<RosQtBridge> bridge_;

  double nis_gate_pos_     = 11.34;
  double nis_gate_vel_     = 11.34;
  double nis_gate_heading_ = 6.63;

  // Update rate labels
  QLabel* pos_ratio_label_     = nullptr;
  QLabel* vel_ratio_label_     = nullptr;
  QLabel* heading_ratio_label_ = nullptr;
  QLabel* pos_reason_label_    = nullptr;
  QLabel* vel_reason_label_    = nullptr;
  QLabel* heading_reason_label_ = nullptr;

  // NIS χ² consistency score (fraction within gate)
  QLabel* nis_pos_consistency_label_     = nullptr;
  QLabel* nis_vel_consistency_label_     = nullptr;
  QLabel* nis_heading_consistency_label_ = nullptr;

  // NIS series (value + gate line)
  QLineSeries* nis_pos_series_     = nullptr;
  QLineSeries* nis_vel_series_     = nullptr;
  QLineSeries* nis_heading_series_ = nullptr;
  QLineSeries* gate_pos_series_    = nullptr;
  QLineSeries* gate_vel_series_    = nullptr;
  QLineSeries* gate_heading_series_ = nullptr;

  // Residual norm series
  QLineSeries* res_pos_series_     = nullptr;
  QLineSeries* res_vel_series_     = nullptr;
  QLineSeries* res_heading_series_ = nullptr;

  // ACF series and significance lines
  QLineSeries* acf_pos_series_     = nullptr;
  QLineSeries* acf_vel_series_     = nullptr;
  QLineSeries* acf_heading_series_ = nullptr;
  QLineSeries* acf_sig_pos_hi_     = nullptr;
  QLineSeries* acf_sig_pos_lo_     = nullptr;
  QLineSeries* acf_sig_vel_hi_     = nullptr;
  QLineSeries* acf_sig_vel_lo_     = nullptr;
  QLineSeries* acf_sig_hdg_hi_     = nullptr;
  QLineSeries* acf_sig_hdg_lo_     = nullptr;

  // Mismatch warning labels
  QLabel* acf_pos_warn_     = nullptr;
  QLabel* acf_vel_warn_     = nullptr;
  QLabel* acf_heading_warn_ = nullptr;

  double t_start_ = -1.0;
};

}  // namespace autodriver::tools
