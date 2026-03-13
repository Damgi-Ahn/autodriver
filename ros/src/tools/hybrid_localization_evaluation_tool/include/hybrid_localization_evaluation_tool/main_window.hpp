#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QMainWindow>
#include <QTimer>

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

  QTimer* timer_ = nullptr;
};

}  // namespace autodriver::tools
