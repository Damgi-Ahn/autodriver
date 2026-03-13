#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QMainWindow>
#include <QElapsedTimer>
#include <QTimer>
#include <QPlainTextEdit>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>

#include <memory>

namespace autodriver::tools {

class EvaluationMainWindow : public QMainWindow {
 public:
  explicit EvaluationMainWindow(const std::shared_ptr<RosQtBridge>& bridge,
                                QWidget* parent = nullptr);

 private:
  void UpdateUi();
  static QString FormatStat(const StatSummary& stat, const QString& unit);
  static QString FormatUpdateRate(const UpdateRateSummary& summary);
  static QString FormatReasonHist(const UpdateRateSummary& summary);

  std::shared_ptr<RosQtBridge> bridge_;

  QLabel* status_label_ = nullptr;
  QLabel* kpi_label_ = nullptr;
  QLabel* nis_label_ = nullptr;
  QLabel* delay_label_ = nullptr;
  QLabel* reason_label_ = nullptr;
  QLabel* timestamp_label_ = nullptr;
  QLabel* nis_pos_card_ = nullptr;
  QLabel* nis_vel_card_ = nullptr;
  QLabel* nis_heading_card_ = nullptr;
  QLabel* hz_label_ = nullptr;
  QLabel* cpu_label_ = nullptr;
  QPlainTextEdit* diag_dump_ = nullptr;
  QLabel* gnss_pose_card_ = nullptr;
  QLabel* eskf_pose_card_ = nullptr;
  QLabel* fgo_pose_card_ = nullptr;

  QLineSeries* nis_pos_series_ = nullptr;
  QLineSeries* nis_vel_series_ = nullptr;
  QLineSeries* nis_heading_series_ = nullptr;
  QLineSeries* delay_gnss_series_ = nullptr;
  QLineSeries* delay_gnss_vel_series_ = nullptr;
  QLineSeries* delay_vehicle_series_ = nullptr;
  QLineSeries* delay_steer_series_ = nullptr;
  QChartView* nis_pos_chart_view_ = nullptr;
  QChartView* nis_vel_chart_view_ = nullptr;
  QChartView* nis_heading_chart_view_ = nullptr;
  QChartView* delay_gnss_chart_view_ = nullptr;
  QChartView* delay_gnss_vel_chart_view_ = nullptr;
  QChartView* delay_vehicle_chart_view_ = nullptr;
  QChartView* delay_steer_chart_view_ = nullptr;

  QTimer* timer_ = nullptr;
  QElapsedTimer cpu_timer_;
  double last_cpu_seconds_ = 0.0;
  int cpu_cores_ = 1;
};

}  // namespace autodriver::tools
