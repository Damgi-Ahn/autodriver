#include "hybrid_localization_evaluation_tool/ui/export_session_tab.hpp"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGridLayout>

namespace autodriver::tools {

namespace {

QLabel* InfoRow(QGridLayout* g, int row, const QString& title, QLabel*& value)
{
  auto* t = new QLabel(title);
  t->setStyleSheet("color:#8a93a3;");
  value = new QLabel("—");
  value->setStyleSheet("color:#ecf0f1; font-family:'JetBrains Mono'; font-size:11px;");
  value->setWordWrap(true);
  value->setTextInteractionFlags(Qt::TextSelectableByMouse);
  g->addWidget(t,     row, 0);
  g->addWidget(value, row, 1);
  return t;
}

QPushButton* StyledButton(const QString& text, const QString& color)
{
  auto* btn = new QPushButton(text);
  btn->setStyleSheet(QString(
      "QPushButton{background:%1; color:#ecf0f1; border:none; border-radius:6px;"
      " padding:8px 20px; font-weight:bold;}"
      "QPushButton:hover{background:%2;}"
      "QPushButton:pressed{background:%3;}").arg(color).arg(color + "cc").arg(color + "88"));
  return btn;
}

}  // namespace

ExportSessionTab::ExportSessionTab(const std::shared_ptr<RosQtBridge>& bridge,
                                   StorageExporter* exporter,
                                   QWidget* parent)
    : QWidget(parent), bridge_(bridge), exporter_(exporter)
{
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(16, 16, 16, 16);
  root->setSpacing(16);

  // ---- File paths box -------------------------------------------------------
  auto* paths_box = new QGroupBox("CSV Output Files");
  paths_box->setStyleSheet(
      "QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}"
      "QGroupBox::title{subcontrol-origin:margin;left:8px;}");
  auto* paths_grid = new QGridLayout(paths_box);
  paths_grid->setColumnStretch(1, 1);
  InfoRow(paths_grid, 0, "Output dir",    output_dir_label_);
  InfoRow(paths_grid, 1, "Raw CSV",       raw_path_label_);
  InfoRow(paths_grid, 2, "KPI CSV",       kpi_path_label_);
  InfoRow(paths_grid, 3, "Events CSV",    events_path_label_);
  InfoRow(paths_grid, 4, "Session CSV",   session_path_label_);
  root->addWidget(paths_box);

  // ---- Session stats box ----------------------------------------------------
  auto* stats_box = new QGroupBox("Session Statistics");
  stats_box->setStyleSheet(
      "QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}"
      "QGroupBox::title{subcontrol-origin:margin;left:8px;}");
  auto* stats_grid = new QGridLayout(stats_box);
  stats_grid->setColumnStretch(1, 1);
  InfoRow(stats_grid, 0, "Samples recorded",  sample_count_label_);
  InfoRow(stats_grid, 1, "Alert events",       alert_count_label_);
  InfoRow(stats_grid, 2, "Session duration",   session_time_label_);
  root->addWidget(stats_box);

  // ---- Actions box ----------------------------------------------------------
  auto* action_box = new QGroupBox("Actions");
  action_box->setStyleSheet(
      "QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}"
      "QGroupBox::title{subcontrol-origin:margin;left:8px;}");
  auto* action_layout = new QHBoxLayout(action_box);

  flush_btn_ = StyledButton("Write Session Summary", "#2980b9");
  clear_btn_ = StyledButton("Clear Session",         "#c0392b");
  status_lbl_ = new QPushButton("Idle");
  status_lbl_->setFlat(true);
  status_lbl_->setStyleSheet("color:#8a93a3; font-size:11px;");
  status_lbl_->setEnabled(false);

  action_layout->addWidget(flush_btn_);
  action_layout->addWidget(clear_btn_);
  action_layout->addStretch();
  action_layout->addWidget(status_lbl_);
  root->addWidget(action_box);

  root->addStretch();
  setLayout(root);

  // ---- Connections ----------------------------------------------------------
  connect(flush_btn_, &QPushButton::clicked, this, [this]() {
    if (on_flush_) {
      on_flush_();
      status_lbl_->setText("Session summary written.");
    }
  });
  connect(clear_btn_, &QPushButton::clicked, this, [this]() {
    if (on_clear_) {
      on_clear_();
      status_lbl_->setText("Session cleared.");
      sample_count_ = 0;
      alert_count_  = 0;
    }
  });

  // Populate file path labels immediately if exporter is already enabled
  if (exporter_ && exporter_->enabled()) {
    raw_path_label_->setText(QString::fromStdString(exporter_->raw_path()));
    kpi_path_label_->setText(QString::fromStdString(exporter_->kpi_path()));
    events_path_label_->setText(QString::fromStdString(exporter_->events_path()));
    session_path_label_->setText(QString::fromStdString(exporter_->session_path()));
  }
}

void ExportSessionTab::Refresh(const BridgeData& d)
{
  // Update file paths if exporter is enabled
  if (exporter_ && exporter_->enabled()) {
    const auto r = QString::fromStdString(exporter_->raw_path());
    if (raw_path_label_->text() != r) {
      raw_path_label_->setText(r);
      kpi_path_label_->setText(QString::fromStdString(exporter_->kpi_path()));
      events_path_label_->setText(QString::fromStdString(exporter_->events_path()));
      session_path_label_->setText(QString::fromStdString(exporter_->session_path()));

      // Extract output dir
      const auto idx = r.lastIndexOf('/');
      output_dir_label_->setText(idx >= 0 ? r.left(idx) : r);
    }
  }

  // Update stats
  sample_count_ = d.sample_history.size();
  alert_count_  = d.event_log.size();
  sample_count_label_->setText(QString::number(sample_count_));
  alert_count_label_->setText(QString::number(alert_count_));

  if (d.has_sample && d.sample_history.size() >= 2) {
    const double dur = d.sample_history.back().stamp.seconds()
                     - d.sample_history.front().stamp.seconds();
    session_time_label_->setText(QString("%1 s").arg(dur, 0, 'f', 1));
  }
}

}  // namespace autodriver::tools
