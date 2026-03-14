#include "hybrid_localization_evaluation_tool/ui/fusion_quality_tab.hpp"
#include "hybrid_localization_evaluation_tool/innovation_analyzer.hpp"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSplitter>
#include <QVBoxLayout>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

#include <algorithm>
#include <sstream>

namespace autodriver::tools {

namespace {

QChartView* BuildAcfChart(const QString& title,
                           QLineSeries* acf_series,
                           QLineSeries* sig_hi,
                           QLineSeries* sig_lo)
{
  acf_series->setName("ACF");
  acf_series->setColor(QColor("#3498db"));

  const QPen sig_pen(QColor("#e74c3c"), 1, Qt::DashLine);
  sig_hi->setPen(sig_pen); sig_hi->setName("+sig");
  sig_lo->setPen(sig_pen); sig_lo->setName("-sig");

  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 9));
  chart->setMargins(QMargins(4, 4, 4, 4));
  chart->legend()->setVisible(false);
  chart->addSeries(acf_series);
  chart->addSeries(sig_hi);
  chart->addSeries(sig_lo);

  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3")); ax->setTitleText("lag");
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3")); ay->setRange(-1.0, 1.0);
  chart->addAxis(ax, Qt::AlignBottom); chart->addAxis(ay, Qt::AlignLeft);
  for (auto* s : {acf_series, sig_hi, sig_lo}) {
    s->attachAxis(ax); s->attachAxis(ay);
  }

  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(120);
  return view;
}

QChartView* BuildNisChart(const QString& title,
                          QLineSeries* nis_series,
                          QLineSeries* gate_series)
{
  gate_series->setColor(QColor("#e74c3c"));
  gate_series->setName("gate");
  QPen gate_pen(QColor("#e74c3c"));
  gate_pen.setStyle(Qt::DashLine);
  gate_series->setPen(gate_pen);

  nis_series->setName("NIS");

  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 10, QFont::DemiBold));
  chart->legend()->setVisible(true);
  chart->legend()->setLabelColor(QColor("#a3b1c6"));
  chart->addSeries(nis_series);
  chart->addSeries(gate_series);

  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3")); ax->setTitleText("t (s)");
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3")); ay->setTitleText("NIS");
  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);
  nis_series->attachAxis(ax);  nis_series->attachAxis(ay);
  gate_series->attachAxis(ax); gate_series->attachAxis(ay);

  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(160);
  return view;
}

QChartView* BuildResidualChart(const QString& title, QLineSeries* series)
{
  series->setName("residual");
  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 10, QFont::DemiBold));
  chart->legend()->setVisible(false);
  chart->addSeries(series);

  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3"));
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3"));
  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);
  series->attachAxis(ax); series->attachAxis(ay);

  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(120);
  return view;
}

QString FormatRatioLabel(const UpdateRateSummary& u)
{
  if (u.total == 0) return "—";
  return QString("%1 / %2  (%3%)")
      .arg(u.applied).arg(u.total)
      .arg(u.ratio * 100.0, 0, 'f', 1);
}

QString FormatReasonHist(const UpdateRateSummary& u)
{
  if (u.reason_hist.empty()) return "—";
  QStringList parts;
  for (const auto& [reason, count] : u.reason_hist) {
    parts << QString("%1: %2").arg(QString::fromStdString(reason)).arg(count);
  }
  return parts.join("\n");
}

}  // namespace

FusionQualityTab::FusionQualityTab(const std::shared_ptr<RosQtBridge>& bridge,
                                   QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  // ACF series
  acf_pos_series_     = new QLineSeries(); acf_vel_series_     = new QLineSeries();
  acf_heading_series_ = new QLineSeries();
  acf_sig_pos_hi_     = new QLineSeries(); acf_sig_pos_lo_     = new QLineSeries();
  acf_sig_vel_hi_     = new QLineSeries(); acf_sig_vel_lo_     = new QLineSeries();
  acf_sig_hdg_hi_     = new QLineSeries(); acf_sig_hdg_lo_     = new QLineSeries();

  nis_pos_series_      = new QLineSeries(); nis_pos_series_->setColor(QColor("#3498db"));
  nis_vel_series_      = new QLineSeries(); nis_vel_series_->setColor(QColor("#2ecc71"));
  nis_heading_series_  = new QLineSeries(); nis_heading_series_->setColor(QColor("#e67e22"));
  gate_pos_series_     = new QLineSeries();
  gate_vel_series_     = new QLineSeries();
  gate_heading_series_ = new QLineSeries();
  res_pos_series_      = new QLineSeries(); res_pos_series_->setColor(QColor("#3498db"));
  res_vel_series_      = new QLineSeries(); res_vel_series_->setColor(QColor("#2ecc71"));
  res_heading_series_  = new QLineSeries(); res_heading_series_->setColor(QColor("#e67e22"));

  auto* root   = new QHBoxLayout(this);
  auto* left   = new QVBoxLayout();
  auto* right  = new QVBoxLayout();

  // ---- Left: update ratios & reason histograms ----------------------------
  auto* ratio_box = new QGroupBox("Update Ratios");
  ratio_box->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:8px;}"
                           "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* ratio_layout = new QVBoxLayout(ratio_box);

  const auto add_ratio_row = [&](const QString& title, QLabel*& ratio_lbl, QLabel*& reason_lbl) {
    auto* grp = new QGroupBox(title);
    grp->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:4px;}");
    auto* g = new QVBoxLayout(grp);
    ratio_lbl  = new QLabel("—"); ratio_lbl->setStyleSheet("color:#ecf0f1;");
    reason_lbl = new QLabel("—"); reason_lbl->setStyleSheet("color:#8a93a3; font-size:10px;");
    reason_lbl->setWordWrap(true);
    g->addWidget(ratio_lbl);
    g->addWidget(reason_lbl);
    ratio_layout->addWidget(grp);
  };
  add_ratio_row("GNSS Pos", pos_ratio_label_, pos_reason_label_);
  add_ratio_row("GNSS Vel", vel_ratio_label_, vel_reason_label_);
  add_ratio_row("Heading",  heading_ratio_label_, heading_reason_label_);
  left->addWidget(ratio_box);

  // ---- NIS χ² consistency scores ------------------------------------------
  auto* cons_box = new QGroupBox("NIS χ² Consistency");
  cons_box->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:8px;}"
                          "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* cons_layout = new QVBoxLayout(cons_box);
  auto* cons_note = new QLabel("fraction of samples within χ² gate");
  cons_note->setStyleSheet("color:#8a93a3; font-size:10px;");
  cons_layout->addWidget(cons_note);
  nis_pos_consistency_label_     = new QLabel("GNSS Pos: —");
  nis_vel_consistency_label_     = new QLabel("GNSS Vel: —");
  nis_heading_consistency_label_ = new QLabel("Heading:  —");
  for (auto* lbl : {nis_pos_consistency_label_, nis_vel_consistency_label_,
                    nis_heading_consistency_label_}) {
    lbl->setStyleSheet("color:#ecf0f1; font-size:12px;");
    cons_layout->addWidget(lbl);
  }
  left->addWidget(cons_box);
  left->addStretch();

  // ---- Right: NIS charts + residual charts --------------------------------
  right->addWidget(BuildNisChart("GNSS Pos NIS",     nis_pos_series_,     gate_pos_series_));
  right->addWidget(BuildNisChart("GNSS Vel NIS",     nis_vel_series_,     gate_vel_series_));
  right->addWidget(BuildNisChart("Heading Yaw NIS",  nis_heading_series_, gate_heading_series_));

  auto* res_box = new QGroupBox("Residual Norms");
  res_box->setStyleSheet("QGroupBox{color:#a3b1c6; border:1px solid #2c3e50; margin-top:8px;}");
  auto* res_layout = new QHBoxLayout(res_box);
  res_layout->addWidget(BuildResidualChart("Pos residual", res_pos_series_));
  res_layout->addWidget(BuildResidualChart("Vel residual", res_vel_series_));
  res_layout->addWidget(BuildResidualChart("Heading residual (rad)", res_heading_series_));
  right->addWidget(res_box);

  // ---- ACF section ---------------------------------------------------------
  auto* acf_box = new QGroupBox("Innovation ACF  (model mismatch detector)");
  acf_box->setStyleSheet("QGroupBox{color:#f39c12; border:1px solid #2c3e50; margin-top:8px;}"
                         "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* acf_layout = new QVBoxLayout(acf_box);

  auto* acf_warn_row = new QHBoxLayout();
  acf_pos_warn_     = new QLabel("GNSS Pos ACF: OK");
  acf_vel_warn_     = new QLabel("GNSS Vel ACF: OK");
  acf_heading_warn_ = new QLabel("Heading ACF: OK");
  for (auto* lbl : {acf_pos_warn_, acf_vel_warn_, acf_heading_warn_}) {
    lbl->setStyleSheet("color:#27ae60; font-size:11px;");
    acf_warn_row->addWidget(lbl);
  }
  acf_layout->addLayout(acf_warn_row);

  auto* acf_charts_row = new QHBoxLayout();
  acf_charts_row->addWidget(BuildAcfChart("GNSS Pos ACF",
      acf_pos_series_, acf_sig_pos_hi_, acf_sig_pos_lo_));
  acf_charts_row->addWidget(BuildAcfChart("GNSS Vel ACF",
      acf_vel_series_, acf_sig_vel_hi_, acf_sig_vel_lo_));
  acf_charts_row->addWidget(BuildAcfChart("Heading ACF",
      acf_heading_series_, acf_sig_hdg_hi_, acf_sig_hdg_lo_));
  acf_layout->addLayout(acf_charts_row);
  right->addWidget(acf_box);

  root->addLayout(left, 1);
  root->addLayout(right, 3);
  setLayout(root);
}

void FusionQualityTab::SetNisGates(double pos, double vel, double heading)
{
  nis_gate_pos_     = pos;
  nis_gate_vel_     = vel;
  nis_gate_heading_ = heading;
}

void FusionQualityTab::Refresh(const BridgeData& d)
{
  // Update ratio labels
  if (d.has_kpi) {
    pos_ratio_label_->setText(FormatRatioLabel(d.kpi.gnss_pos_update));
    vel_ratio_label_->setText(FormatRatioLabel(d.kpi.gnss_vel_update));
    heading_ratio_label_->setText(FormatRatioLabel(d.kpi.heading_yaw_update));
    pos_reason_label_->setText(FormatReasonHist(d.kpi.gnss_pos_update));
    vel_reason_label_->setText(FormatReasonHist(d.kpi.gnss_vel_update));
    heading_reason_label_->setText(FormatReasonHist(d.kpi.heading_yaw_update));

    // NIS χ² consistency = 1 - violation_rate
    const auto fmt_cons = [](double violation_rate) {
      const double pct = (1.0 - violation_rate) * 100.0;
      const QString color = pct >= 90.0 ? "#27ae60" : (pct >= 70.0 ? "#f39c12" : "#e74c3c");
      return QString("<span style='color:%1'>%2%</span>").arg(color).arg(pct, 0, 'f', 1);
    };
    nis_pos_consistency_label_->setText(
        QString("GNSS Pos: %1").arg(fmt_cons(d.kpi.gnss_pos_nis_violation_rate)));
    nis_vel_consistency_label_->setText(
        QString("GNSS Vel: %1").arg(fmt_cons(d.kpi.gnss_vel_nis_violation_rate)));
    nis_heading_consistency_label_->setText(
        QString("Heading:  %1").arg(fmt_cons(d.kpi.heading_nis_violation_rate)));
    for (auto* lbl : {nis_pos_consistency_label_, nis_vel_consistency_label_,
                      nis_heading_consistency_label_}) {
      lbl->setTextFormat(Qt::RichText);
    }
  }

  // NIS time series
  const auto& hist = d.sample_history;
  if (hist.empty()) return;
  if (t_start_ < 0.0) t_start_ = hist.front().stamp.seconds();

  nis_pos_series_->clear();
  nis_vel_series_->clear();
  nis_heading_series_->clear();
  gate_pos_series_->clear();
  gate_vel_series_->clear();
  gate_heading_series_->clear();
  res_pos_series_->clear();
  res_vel_series_->clear();
  res_heading_series_->clear();

  const double t0 = hist.front().stamp.seconds() - t_start_;
  const double t1 = hist.back().stamp.seconds()  - t_start_;
  gate_pos_series_->append(t0, nis_gate_pos_);
  gate_pos_series_->append(t1, nis_gate_pos_);
  gate_vel_series_->append(t0, nis_gate_vel_);
  gate_vel_series_->append(t1, nis_gate_vel_);
  gate_heading_series_->append(t0, nis_gate_heading_);
  gate_heading_series_->append(t1, nis_gate_heading_);

  for (const auto& s : hist) {
    const double t = s.stamp.seconds() - t_start_;
    if (s.gnss_pos_nis.has_value())          nis_pos_series_->append(t,     *s.gnss_pos_nis);
    if (s.gnss_vel_nis.has_value())          nis_vel_series_->append(t,     *s.gnss_vel_nis);
    if (s.heading_yaw_nis.has_value())       nis_heading_series_->append(t, *s.heading_yaw_nis);
    if (s.gnss_pos_residual_norm.has_value())res_pos_series_->append(t,     *s.gnss_pos_residual_norm);
    if (s.gnss_vel_residual_norm.has_value())res_vel_series_->append(t,     *s.gnss_vel_residual_norm);
    if (s.heading_yaw_residual_rad.has_value()) res_heading_series_->append(t, std::abs(*s.heading_yaw_residual_rad));
  }

  // ---- ACF update ----------------------------------------------------------
  if (!d.acf.valid) return;

  const auto refresh_acf = [](const AcfValues& av,
                               QLineSeries* acf_s,
                               QLineSeries* sig_hi,
                               QLineSeries* sig_lo,
                               QLabel* warn_lbl,
                               const QString& ch_name)
  {
    acf_s->clear(); sig_hi->clear(); sig_lo->clear();
    if (av.acf.empty()) return;
    const double sig = av.significance;
    const double max_lag = av.lags.back();
    sig_hi->append(0.0, sig); sig_hi->append(max_lag + 0.5, sig);
    sig_lo->append(0.0, -sig); sig_lo->append(max_lag + 0.5, -sig);
    for (size_t i = 0; i < av.acf.size(); ++i) {
      acf_s->append(av.lags[i], av.acf[i]);
    }
    if (av.model_mismatch) {
      warn_lbl->setText(QString("%1 ACF: ⚠ MODEL MISMATCH").arg(ch_name));
      warn_lbl->setStyleSheet("color:#e74c3c; font-size:11px; font-weight:bold;");
    } else {
      warn_lbl->setText(QString("%1 ACF: OK").arg(ch_name));
      warn_lbl->setStyleSheet("color:#27ae60; font-size:11px;");
    }
  };

  refresh_acf(d.acf.gnss_pos, acf_pos_series_, acf_sig_pos_hi_, acf_sig_pos_lo_,
              acf_pos_warn_,     "GNSS Pos");
  refresh_acf(d.acf.gnss_vel, acf_vel_series_, acf_sig_vel_hi_, acf_sig_vel_lo_,
              acf_vel_warn_,     "GNSS Vel");
  refresh_acf(d.acf.heading,  acf_heading_series_, acf_sig_hdg_hi_, acf_sig_hdg_lo_,
              acf_heading_warn_, "Heading");
}

}  // namespace autodriver::tools
