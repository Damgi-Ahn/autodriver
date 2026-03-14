#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"
#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"

#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <functional>
#include <memory>

namespace autodriver::tools {

class ExportSessionTab : public QWidget {
  Q_OBJECT
 public:
  explicit ExportSessionTab(const std::shared_ptr<RosQtBridge>& bridge,
                            StorageExporter* exporter,
                            QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

  // Callback invoked when the user requests a session summary flush
  void SetOnFlushSessionSummary(std::function<void()> cb) { on_flush_ = std::move(cb); }
  // Callback invoked when the user requests a session clear
  void SetOnClearSession(std::function<void()> cb) { on_clear_ = std::move(cb); }

 private:
  std::shared_ptr<RosQtBridge> bridge_;
  StorageExporter*             exporter_ = nullptr;

  std::function<void()> on_flush_;
  std::function<void()> on_clear_;

  // Info labels
  QLabel* output_dir_label_  = nullptr;
  QLabel* raw_path_label_    = nullptr;
  QLabel* kpi_path_label_    = nullptr;
  QLabel* events_path_label_ = nullptr;
  QLabel* session_path_label_= nullptr;

  // Stats
  QLabel* sample_count_label_ = nullptr;
  QLabel* alert_count_label_  = nullptr;
  QLabel* session_time_label_ = nullptr;

  QPushButton* flush_btn_  = nullptr;
  QPushButton* clear_btn_  = nullptr;
  QPushButton* status_lbl_ = nullptr;

  // Cached sample/alert counts
  size_t sample_count_ = 0;
  size_t alert_count_  = 0;
};

}  // namespace autodriver::tools
