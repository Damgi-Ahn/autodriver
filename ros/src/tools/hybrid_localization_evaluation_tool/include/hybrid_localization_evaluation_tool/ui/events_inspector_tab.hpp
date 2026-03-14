#pragma once

#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"

#include <QLabel>
#include <QLineEdit>
#include <QTableWidget>
#include <QWidget>

#include <memory>

namespace autodriver::tools {

class EventsInspectorTab : public QWidget {
  Q_OBJECT
 public:
  explicit EventsInspectorTab(const std::shared_ptr<RosQtBridge>& bridge,
                              QWidget* parent = nullptr);
  void Refresh(const BridgeData& data);

 private:
  void ApplyDiagFilter(const QString& text);

  std::shared_ptr<RosQtBridge> bridge_;

  // Event log table
  QTableWidget* event_table_  = nullptr;
  QLabel*       event_count_  = nullptr;

  // Diagnostics raw kv table
  QLineEdit*    diag_filter_  = nullptr;
  QTableWidget* diag_table_   = nullptr;

  // Cached raw kv for filtering
  std::vector<std::pair<QString, QString>> diag_kv_cache_;
};

}  // namespace autodriver::tools
