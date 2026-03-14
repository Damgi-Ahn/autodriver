#include "hybrid_localization_evaluation_tool/ui/overview_tab.hpp"
#include "hybrid_localization_evaluation_tool/health_state_engine.hpp"

#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QScrollArea>
#include <QVBoxLayout>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>

#include <cmath>
#include <sstream>

namespace autodriver::tools {

namespace {

// Status level → color CSS
const char* LevelColor(uint8_t level)
{
  switch (level) {
    case 0:  return "#27ae60";  // OK  - green
    case 1:  return "#f39c12";  // WARN - orange
    case 2:  return "#e74c3c";  // ERROR - red
    default: return "#7f8c8d";  // STALE - grey
  }
}
const char* LevelText(uint8_t level)
{
  switch (level) { case 0: return "OK"; case 1: return "WARN"; case 2: return "ERROR"; default: return "STALE"; }
}

QLabel* MakeCardLabel(const QString& title)
{
  auto* lbl = new QLabel(title);
  lbl->setAlignment(Qt::AlignCenter);
  lbl->setWordWrap(true);
  lbl->setStyleSheet("background:#1a2030; border:1px solid #2c3e50;"
                     " border-radius:4px; padding:8px; color:#ecf0f1;");
  return lbl;
}

QChartView* MakeMiniChart(const QString& title, QLineSeries* series, const QColor& color)
{
  series->setColor(color);
  auto* chart = new QChart();
  chart->addSeries(series);
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 9));
  chart->legend()->setVisible(false);
  chart->setMargins(QMargins(4, 4, 4, 4));

  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3")); ax->setTitleText("t");
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3"));
  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);
  series->attachAxis(ax); series->attachAxis(ay);

  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(120);
  return view;
}

}  // namespace

OverviewTab::OverviewTab(const std::shared_ptr<RosQtBridge>& bridge, QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  auto* root = new QVBoxLayout(this);
  root->setSpacing(8);

  // ---- Status banner -------------------------------------------------------
  banner_label_ = new QLabel("● WAITING");
  banner_label_->setAlignment(Qt::AlignCenter);
  banner_label_->setFont(QFont("Noto Sans", 16, QFont::Bold));
  banner_label_->setStyleSheet("background:#10151c; padding:8px; border-radius:4px; color:#ecf0f1;");
  banner_label_->setFixedHeight(48);
  root->addWidget(banner_label_);

  // ---- Health state row ----------------------------------------------------
  health_state_label_ = new QLabel("● HEALTH: OK");
  health_state_label_->setAlignment(Qt::AlignCenter);
  health_state_label_->setFont(QFont("Noto Sans", 13, QFont::Bold));
  health_state_label_->setStyleSheet("background:#1a3020; padding:6px; border-radius:4px;"
                                     " color:#27ae60; border:1px solid #27ae60;");
  health_state_label_->setFixedHeight(40);
  root->addWidget(health_state_label_);

  // ---- Sub-status row ------------------------------------------------------
  auto* sub_row = new QHBoxLayout();
  state_label_      = MakeCardLabel("State: —");
  activation_label_ = MakeCardLabel("Activation: —");
  eskf_init_label_  = MakeCardLabel("ESKF Init: —");
  sub_row->addWidget(state_label_);
  sub_row->addWidget(activation_label_);
  sub_row->addWidget(eskf_init_label_);
  root->addLayout(sub_row);

  // ---- KPI row -------------------------------------------------------------
  auto* kpi_box = new QGroupBox("KPIs");
  kpi_box->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:8px;}"
                         "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* kpi_layout = new QHBoxLayout(kpi_box);
  output_rate_label_     = MakeCardLabel("Output Rate\n—");
  availability_label_    = MakeCardLabel("Availability\n—");
  diag_rate_label_       = MakeCardLabel("Diag Rate\n—");
  last_output_age_label_ = MakeCardLabel("Last Output Age\n—");
  kpi_layout->addWidget(output_rate_label_);
  kpi_layout->addWidget(availability_label_);
  kpi_layout->addWidget(diag_rate_label_);
  kpi_layout->addWidget(last_output_age_label_);
  root->addWidget(kpi_box);

  // ---- Alert flag cards ----------------------------------------------------
  auto* flag_box = new QGroupBox("Red Flags");
  flag_box->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:8px;}"
                          "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* flag_layout = new QHBoxLayout(flag_box);
  nis_flag_label_   = MakeCardLabel("NIS\n—");
  delay_flag_label_ = MakeCardLabel("Delay\n—");
  cov_flag_label_   = MakeCardLabel("Covariance\n—");
  flag_layout->addWidget(nis_flag_label_);
  flag_layout->addWidget(delay_flag_label_);
  flag_layout->addWidget(cov_flag_label_);
  root->addWidget(flag_box);

  // ---- Mini trend charts ---------------------------------------------------
  nis_pos_mini_    = new QLineSeries();
  delay_gnss_mini_ = new QLineSeries();
  P_trace_mini_    = new QLineSeries();

  auto* charts_layout = new QHBoxLayout();
  charts_layout->addWidget(MakeMiniChart("NIS pos",       nis_pos_mini_,    QColor("#3498db")));
  charts_layout->addWidget(MakeMiniChart("GNSS delay(ms)",delay_gnss_mini_, QColor("#e67e22")));
  charts_layout->addWidget(MakeMiniChart("P trace",       P_trace_mini_,    QColor("#9b59b6")));
  root->addLayout(charts_layout);

  root->addStretch();
  setLayout(root);
}

void OverviewTab::Refresh(const BridgeData& d)
{
  // ---- Health state machine output -----------------------------------------
  {
    const char* color = HealthStateColor(d.health_state);
    const char* text  = HealthStateStr(d.health_state);
    health_state_label_->setText(QString("● HEALTH: %1").arg(text));
    health_state_label_->setStyleSheet(
        QString("background:%1; padding:6px; border-radius:4px;"
                " color:#ffffff; border:1px solid %1; font-size:13px; font-weight:bold;")
            .arg(color));
  }

  // ---- Banner & state ------------------------------------------------------
  if (d.has_sample) {
    const uint8_t lvl = d.sample.diag_level;
    const QString color  = LevelColor(lvl);
    const QString text   = QString("● %1").arg(LevelText(lvl));
    banner_label_->setText(text);
    banner_label_->setStyleSheet(
        QString("background:%1; padding:8px; border-radius:4px; color:#ffffff; font-size:16px; font-weight:bold;").arg(color));

    activation_label_->setText(QString("Activation\n%1").arg(d.sample.is_activated ? "✓ ACTIVE" : "✗ INACTIVE"));
    eskf_init_label_->setText(QString("ESKF Init\n%1").arg(d.sample.eskf_initialized ? "✓ YES" : "✗ NO"));
  }
  if (!d.localization_state.empty()) {
    state_label_->setText(QString("State\n%1").arg(QString::fromStdString(d.localization_state)));
  }

  // ---- KPIs ----------------------------------------------------------------
  if (d.has_kpi) {
    const auto& k = d.kpi;
    output_rate_label_->setText(
        QString("Output Rate\n%1 Hz").arg(k.output_rate_hz, 0, 'f', 1));
    availability_label_->setText(
        QString("Availability\n%1 %").arg(k.output_availability.ratio * 100.0, 0, 'f', 1));
    diag_rate_label_->setText(
        QString("Diag Rate\n%1 Hz").arg(k.diag_rate_hz, 0, 'f', 1));
    if (k.output_availability.has_output) {
      last_output_age_label_->setText(
          QString("Last Output Age\n%1 ms").arg(k.output_availability.last_age_sec * 1000.0, 0, 'f', 1));
    }

    // NIS flag
    const double max_nis_viol = std::max({k.gnss_pos_nis_violation_rate,
                                          k.gnss_vel_nis_violation_rate,
                                          k.heading_nis_violation_rate});
    if (max_nis_viol > 0.2) {
      nis_flag_label_->setStyleSheet("background:#7d241e; border:1px solid #e74c3c;"
                                     " border-radius:4px; padding:8px; color:#e74c3c;");
      nis_flag_label_->setText(QString("NIS Gate\n%1% violations").arg(max_nis_viol * 100.0, 0, 'f', 0));
    } else {
      nis_flag_label_->setStyleSheet("background:#1a2030; border:1px solid #27ae60;"
                                     " border-radius:4px; padding:8px; color:#27ae60;");
      nis_flag_label_->setText("NIS Gate\nOK");
    }

    // Delay flag
    const double max_delay = std::max({k.gnss_delay.max, k.gnss_vel_delay.max,
                                        k.velocity_delay.max, k.steering_delay.max});
    if (max_delay > 500.0) {
      delay_flag_label_->setStyleSheet("background:#7d241e; border:1px solid #e74c3c;"
                                       " border-radius:4px; padding:8px; color:#e74c3c;");
      delay_flag_label_->setText(QString("Delay\n%1 ms MAX").arg(max_delay, 0, 'f', 0));
    } else if (max_delay > 200.0) {
      delay_flag_label_->setStyleSheet("background:#5d4a1e; border:1px solid #f39c12;"
                                       " border-radius:4px; padding:8px; color:#f39c12;");
      delay_flag_label_->setText(QString("Delay\n%1 ms MAX").arg(max_delay, 0, 'f', 0));
    } else {
      delay_flag_label_->setStyleSheet("background:#1a2030; border:1px solid #27ae60;"
                                       " border-radius:4px; padding:8px; color:#27ae60;");
      delay_flag_label_->setText("Delay\nOK");
    }

    // Covariance flag
    bool cov_warn = false;
    if (k.P_trace_latest.has_value() && *k.P_trace_latest > 100.0) cov_warn = true;
    if (k.P_min_eig_latest.has_value() && *k.P_min_eig_latest < 0.0) cov_warn = true;
    cov_flag_label_->setStyleSheet(cov_warn ?
        "background:#7d241e; border:1px solid #e74c3c; border-radius:4px; padding:8px; color:#e74c3c;" :
        "background:#1a2030; border:1px solid #27ae60; border-radius:4px; padding:8px; color:#27ae60;");
    cov_flag_label_->setText(cov_warn ? "Covariance\n⚠ DRIFT" : "Covariance\nOK");
  }

  // ---- Mini charts ---------------------------------------------------------
  const auto& hist = d.sample_history;
  if (!hist.empty()) {
    if (t_start_ < 0.0) t_start_ = hist.front().stamp.seconds();

    nis_pos_mini_->clear();
    delay_gnss_mini_->clear();
    P_trace_mini_->clear();

    for (const auto& s : hist) {
      const double t = s.stamp.seconds() - t_start_;
      if (s.gnss_pos_nis.has_value())  nis_pos_mini_->append(t, *s.gnss_pos_nis);
      if (s.gnss_delay.has_value())    delay_gnss_mini_->append(t, *s.gnss_delay);
      if (s.P_trace.has_value())       P_trace_mini_->append(t, *s.P_trace);
    }
  }
}

}  // namespace autodriver::tools
