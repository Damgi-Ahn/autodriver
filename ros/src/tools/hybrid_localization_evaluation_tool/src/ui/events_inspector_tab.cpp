#include "hybrid_localization_evaluation_tool/ui/events_inspector_tab.hpp"

#include <QComboBox>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QSplitter>
#include <QVBoxLayout>

#include <algorithm>

namespace autodriver::tools {

namespace {

QString SevColor(AlertSeverity s)
{
  switch (s) {
    case AlertSeverity::WARN:  return "#f39c12";
    case AlertSeverity::ERROR: return "#e74c3c";
    default:                   return "#2ecc71";
  }
}

}  // namespace

EventsInspectorTab::EventsInspectorTab(const std::shared_ptr<RosQtBridge>& bridge,
                                       QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  auto* root = new QVBoxLayout(this);

  auto* splitter = new QSplitter(Qt::Vertical);
  splitter->setHandleWidth(4);

  // ---- Event log table ---------------------------------------------------
  auto* ev_widget = new QWidget();
  auto* ev_layout = new QVBoxLayout(ev_widget);
  ev_layout->setContentsMargins(0, 0, 0, 0);

  auto* ev_header = new QHBoxLayout();
  auto* ev_title  = new QLabel("Alert Event Log");
  ev_title->setStyleSheet("color:#a3b1c6; font-weight:bold; font-size:12px;");
  event_count_ = new QLabel("0 events");
  event_count_->setStyleSheet("color:#8a93a3; font-size:11px;");

  // Severity filter
  sev_filter_ = new QComboBox();
  sev_filter_->addItem("All Severities");
  sev_filter_->addItem("INFO");
  sev_filter_->addItem("WARN");
  sev_filter_->addItem("ERROR");
  sev_filter_->setStyleSheet(
      "QComboBox{background:#141a22; color:#ecf0f1; border:1px solid #2c3e50;"
      " border-radius:4px; padding:3px 6px;}"
      "QComboBox::drop-down{border:none;}"
      "QComboBox QAbstractItemView{background:#141a22; color:#ecf0f1;}");

  // Alert type filter
  type_filter_ = new QComboBox();
  type_filter_->addItem("All Types");
  type_filter_->addItem("NIS_GATE_VIOLATION");
  type_filter_->addItem("SENSOR_DELAY");
  type_filter_->addItem("COVARIANCE");
  type_filter_->addItem("SENSOR_DROPOUT");
  type_filter_->addItem("STATE_CHANGE");
  type_filter_->addItem("OUTPUT_AVAILABILITY");
  type_filter_->setStyleSheet(sev_filter_->styleSheet());
  type_filter_->setMinimumWidth(180);

  ev_header->addWidget(ev_title);
  ev_header->addStretch();
  ev_header->addWidget(sev_filter_);
  ev_header->addWidget(type_filter_);
  ev_header->addWidget(event_count_);

  event_table_ = new QTableWidget(0, 5);
  event_table_->setHorizontalHeaderLabels({"stamp (s)", "severity", "type", "message", "value"});
  event_table_->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Stretch);
  event_table_->horizontalHeader()->setStyleSheet("color:#8a93a3;");
  event_table_->setStyleSheet(
      "QTableWidget{background:#0e1116; color:#d6d9dd; gridline-color:#222831; border:none;}"
      "QTableWidget::item{padding:3px;}"
      "QHeaderView::section{background:#141a22; color:#8a93a3; border:1px solid #222831; padding:4px;}");
  event_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  event_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  event_table_->setSortingEnabled(true);
  event_table_->verticalHeader()->setVisible(false);
  event_table_->setAlternatingRowColors(true);
  event_table_->setMinimumHeight(150);

  ev_layout->addLayout(ev_header);
  ev_layout->addWidget(event_table_);
  splitter->addWidget(ev_widget);

  // ---- Diagnostics raw kv table ------------------------------------------
  auto* diag_widget = new QWidget();
  auto* diag_layout = new QVBoxLayout(diag_widget);
  diag_layout->setContentsMargins(0, 0, 0, 0);

  auto* diag_header = new QHBoxLayout();
  auto* diag_title  = new QLabel("Raw Diagnostics");
  diag_title->setStyleSheet("color:#a3b1c6; font-weight:bold; font-size:12px;");
  diag_filter_ = new QLineEdit();
  diag_filter_->setPlaceholderText("filter key...");
  diag_filter_->setMaximumWidth(200);
  diag_filter_->setStyleSheet(
      "QLineEdit{background:#141a22; color:#ecf0f1; border:1px solid #2c3e50;"
      " border-radius:4px; padding:3px 6px;}");
  diag_header->addWidget(diag_title);
  diag_header->addStretch();
  diag_header->addWidget(diag_filter_);

  diag_table_ = new QTableWidget(0, 2);
  diag_table_->setHorizontalHeaderLabels({"key", "value"});
  diag_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  diag_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  diag_table_->setStyleSheet(
      "QTableWidget{background:#0e1116; color:#d6d9dd; gridline-color:#222831; border:none;}"
      "QTableWidget::item{padding:3px; font-family:'JetBrains Mono'; font-size:11px;}"
      "QHeaderView::section{background:#141a22; color:#8a93a3; border:1px solid #222831; padding:4px;}");
  diag_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  diag_table_->verticalHeader()->setVisible(false);
  diag_table_->setAlternatingRowColors(true);
  diag_table_->setMinimumHeight(150);

  diag_layout->addLayout(diag_header);
  diag_layout->addWidget(diag_table_);
  splitter->addWidget(diag_widget);

  root->addWidget(splitter);
  setLayout(root);

  connect(diag_filter_, &QLineEdit::textChanged, this, &EventsInspectorTab::ApplyDiagFilter);
  connect(sev_filter_,  QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { ApplyEventFilter(); });
  connect(type_filter_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { ApplyEventFilter(); });
}

void EventsInspectorTab::Refresh(const BridgeData& d)
{
  // ---- Event log ---------------------------------------------------------
  event_cache_ = d.event_log;
  ApplyEventFilter();

  // ---- Raw diagnostics ---------------------------------------------------
  if (d.has_sample) {
    diag_kv_cache_.clear();
    diag_kv_cache_.reserve(d.sample.raw_kv.size());
    for (const auto& [k, v] : d.sample.raw_kv) {
      diag_kv_cache_.emplace_back(QString::fromStdString(k), QString::fromStdString(v));
    }
    std::sort(diag_kv_cache_.begin(), diag_kv_cache_.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    ApplyDiagFilter(diag_filter_->text());
  }
}

void EventsInspectorTab::ApplyEventFilter()
{
  const QString sev_filter  = sev_filter_->currentIndex() == 0
                                  ? QString{}
                                  : sev_filter_->currentText();
  const QString type_filter = type_filter_->currentIndex() == 0
                                  ? QString{}
                                  : type_filter_->currentText();

  event_table_->setUpdatesEnabled(false);
  event_table_->setSortingEnabled(false);
  event_table_->setRowCount(0);

  int shown = 0;
  for (const auto& ev : event_cache_) {
    const QString sev_str  = QString::fromUtf8(AlertSeverityStr(ev.severity));
    const QString type_str = QString::fromUtf8(AlertTypeStr(ev.type));

    if (!sev_filter.isEmpty()  && sev_str  != sev_filter)  continue;
    if (!type_filter.isEmpty() && !type_str.contains(type_filter, Qt::CaseInsensitive)) continue;

    const int row = event_table_->rowCount();
    event_table_->insertRow(row);
    const QString color = SevColor(ev.severity);

    auto* stamp_item = new QTableWidgetItem(QString::number(ev.stamp.seconds(), 'f', 3));
    auto* sev_item   = new QTableWidgetItem(sev_str);
    sev_item->setForeground(QBrush(QColor(color)));
    auto* type_item  = new QTableWidgetItem(type_str);
    auto* msg_item   = new QTableWidgetItem(QString::fromStdString(ev.message));
    auto* val_item   = new QTableWidgetItem(QString::number(ev.value, 'g', 4));
    for (auto* item : {stamp_item, sev_item, type_item, msg_item, val_item}) {
      item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    }
    event_table_->setItem(row, 0, stamp_item);
    event_table_->setItem(row, 1, sev_item);
    event_table_->setItem(row, 2, type_item);
    event_table_->setItem(row, 3, msg_item);
    event_table_->setItem(row, 4, val_item);
    ++shown;
  }

  event_table_->setSortingEnabled(true);
  event_table_->setUpdatesEnabled(true);
  event_count_->setText(QString("%1 / %2 events").arg(shown).arg(event_cache_.size()));
}

void EventsInspectorTab::ApplyDiagFilter(const QString& text)
{
  const QString lower = text.toLower();

  diag_table_->setUpdatesEnabled(false);
  diag_table_->setSortingEnabled(false);
  diag_table_->setRowCount(0);

  for (const auto& [k, v] : diag_kv_cache_) {
    if (!lower.isEmpty() && !k.toLower().contains(lower) && !v.toLower().contains(lower)) {
      continue;
    }
    const int row = diag_table_->rowCount();
    diag_table_->insertRow(row);
    auto* k_item = new QTableWidgetItem(k);
    auto* v_item = new QTableWidgetItem(v);
    k_item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    v_item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
    diag_table_->setItem(row, 0, k_item);
    diag_table_->setItem(row, 1, v_item);
  }

  diag_table_->setSortingEnabled(true);
  diag_table_->setUpdatesEnabled(true);
}

}  // namespace autodriver::tools
