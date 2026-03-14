#include "hybrid_localization_evaluation_tool/ui/covariance_noise_tab.hpp"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

namespace autodriver::tools {

namespace {

QLabel* InfoCard(const QString& title, QLabel*& body)
{
  (void)title;
  body = new QLabel("—");
  body->setStyleSheet("color:#ecf0f1; background:#1a2030; padding:6px;"
                      " border:1px solid #2c3e50; border-radius:4px;");
  body->setWordWrap(true);
  return body;
}

QChartView* BuildChart(const QString& title, QLineSeries* series, const QColor& color)
{
  series->setColor(color);
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
  view->setMinimumHeight(160);
  return view;
}

QString Fmt(const std::optional<double>& v, int prec = 4)
{
  if (!v.has_value()) return "—";
  return QString::number(*v, 'g', prec);
}

}  // namespace

CovarianceNoiseTab::CovarianceNoiseTab(const std::shared_ptr<RosQtBridge>& bridge,
                                       QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  P_trace_series_   = new QLineSeries();
  P_min_eig_series_ = new QLineSeries();

  auto* root  = new QHBoxLayout(this);
  auto* left  = new QVBoxLayout();
  auto* right = new QVBoxLayout();

  // ---- P stats -------------------------------------------------------------
  auto* p_box = new QGroupBox("Covariance (P) Stats");
  p_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}"
                       "QGroupBox::title{subcontrol-origin:margin;left:8px;}");
  auto* p_grid = new QGridLayout(p_box);
  p_grid->addWidget(new QLabel("P trace"),    0, 0); InfoCard("P trace",    P_trace_label_);   p_grid->addWidget(P_trace_label_,   0, 1);
  p_grid->addWidget(new QLabel("P min_eig"),  1, 0); InfoCard("P min_eig",  P_min_eig_label_); p_grid->addWidget(P_min_eig_label_,  1, 1);
  p_grid->addWidget(new QLabel("P max_diag"), 2, 0); InfoCard("P max_diag", P_max_diag_label_);p_grid->addWidget(P_max_diag_label_, 2, 1);
  p_grid->addWidget(new QLabel("P pos"),      3, 0); InfoCard("P pos",      P_pos_label_);     p_grid->addWidget(P_pos_label_,     3, 1);
  p_grid->addWidget(new QLabel("P vel"),      4, 0); InfoCard("P vel",      P_vel_label_);     p_grid->addWidget(P_vel_label_,     4, 1);
  p_grid->addWidget(new QLabel("P att"),      5, 0); InfoCard("P att",      P_att_label_);     p_grid->addWidget(P_att_label_,     5, 1);
  const char* lbl_style = "color:#8a93a3;";
  for (int r = 0; r < p_grid->rowCount(); ++r)
    if (auto* lbl = qobject_cast<QLabel*>(p_grid->itemAtPosition(r, 0)->widget()))
      lbl->setStyleSheet(lbl_style);
  left->addWidget(p_box);

  // ---- R/Q applied ---------------------------------------------------------
  auto* rq_box = new QGroupBox("R / Q Applied");
  rq_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}");
  auto* rq_layout = new QVBoxLayout(rq_box);
  auto add_rq = [&](const QString& title, QLabel*& lbl) {
    auto* row = new QHBoxLayout();
    auto* t = new QLabel(title); t->setStyleSheet("color:#8a93a3; min-width:120px;");
    lbl = new QLabel("—"); lbl->setStyleSheet("color:#ecf0f1;");
    row->addWidget(t); row->addWidget(lbl); rq_layout->addLayout(row);
  };
  add_rq("GNSS Pos R [xx,yy,zz]", gnss_pos_R_label_);
  add_rq("GNSS Vel R [xx,yy,zz]", gnss_vel_R_label_);
  add_rq("Heading var / eff / applied", heading_var_label_);
  add_rq("Vehicle [spd,nhc,zupt,yaw]", vehicle_var_label_);
  add_rq("IMU Q [gyro,accel,bg,ba]",   imu_Q_label_);
  left->addWidget(rq_box);

  // ---- Inflation factors ---------------------------------------------------
  auto* inf_box = new QGroupBox("Inflation Factors");
  inf_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}");
  auto* inf_layout = new QVBoxLayout(inf_box);
  auto add_inf = [&](const QString& title, QLabel*& lbl) {
    auto* row = new QHBoxLayout();
    auto* t = new QLabel(title); t->setStyleSheet("color:#8a93a3; min-width:120px;");
    lbl = new QLabel("—"); lbl->setStyleSheet("color:#ecf0f1;");
    row->addWidget(t); row->addWidget(lbl); inf_layout->addLayout(row);
  };
  add_inf("GNSS Pos inflate", pos_inflate_label_);
  add_inf("GNSS Vel inflate", vel_inflate_label_);
  add_inf("Heading inflate",  heading_inflate_label_);
  left->addWidget(inf_box);
  left->addStretch();

  // ---- Charts --------------------------------------------------------------
  right->addWidget(BuildChart("P trace over time",    P_trace_series_,   QColor("#9b59b6")));
  right->addWidget(BuildChart("P min_eig over time",  P_min_eig_series_, QColor("#1abc9c")));
  right->addStretch();

  root->addLayout(left, 1);
  root->addLayout(right, 2);
  setLayout(root);
}

void CovarianceNoiseTab::Refresh(const BridgeData& d)
{
  if (d.has_sample) {
    const auto& s = d.sample;
    P_trace_label_->setText(Fmt(s.P_trace));
    P_min_eig_label_->setText(Fmt(s.P_min_eig));
    P_max_diag_label_->setText(Fmt(s.P_max_diag));
    P_pos_label_->setText(Fmt(s.P_pos_max_diag));
    P_vel_label_->setText(Fmt(s.P_vel_max_diag));
    P_att_label_->setText(Fmt(s.P_att_max_diag));

    gnss_pos_R_label_->setText(
        QString("[%1, %2, %3]").arg(Fmt(s.gnss_pos_R_xx)).arg(Fmt(s.gnss_pos_R_yy)).arg(Fmt(s.gnss_pos_R_zz)));
    gnss_vel_R_label_->setText(
        QString("[%1, %2, %3]").arg(Fmt(s.gnss_vel_R_xx)).arg(Fmt(s.gnss_vel_R_yy)).arg(Fmt(s.gnss_vel_R_zz)));
    heading_var_label_->setText(
        QString("%1 / %2 / %3").arg(Fmt(s.heading_yaw_var)).arg(Fmt(s.heading_yaw_var_eff)).arg(Fmt(s.heading_yaw_var_applied)));
    vehicle_var_label_->setText(
        QString("[%1, %2, %3, %4]").arg(Fmt(s.vehicle_speed_var)).arg(Fmt(s.vehicle_nhc_var))
        .arg(Fmt(s.vehicle_zupt_var)).arg(Fmt(s.vehicle_yaw_rate_var)));
    imu_Q_label_->setText(
        QString("[%1, %2, %3, %4]").arg(Fmt(s.imu_gyro_noise_std)).arg(Fmt(s.imu_accel_noise_std))
        .arg(Fmt(s.imu_gyro_bias_noise_std)).arg(Fmt(s.imu_accel_bias_noise_std)));

    pos_inflate_label_->setText(
        QString("status×%1  nis×%2").arg(Fmt(s.gnss_pos_status_inflate)).arg(Fmt(s.gnss_pos_nis_inflate)));
    vel_inflate_label_->setText(
        QString("status×%1  nis×%2").arg(Fmt(s.gnss_vel_status_inflate)).arg(Fmt(s.gnss_vel_nis_inflate)));
    heading_inflate_label_->setText(
        QString("status×%1  recover×%2  nis×%3")
        .arg(Fmt(s.heading_status_inflate)).arg(Fmt(s.heading_recover_inflate)).arg(Fmt(s.heading_yaw_nis_inflate)));
  }

  const auto& hist = d.sample_history;
  if (hist.empty()) return;
  if (t_start_ < 0.0) t_start_ = hist.front().stamp.seconds();
  P_trace_series_->clear();
  P_min_eig_series_->clear();
  for (const auto& s : hist) {
    const double t = s.stamp.seconds() - t_start_;
    if (s.P_trace.has_value())   P_trace_series_->append(t,   *s.P_trace);
    if (s.P_min_eig.has_value()) P_min_eig_series_->append(t, *s.P_min_eig);
  }
}

}  // namespace autodriver::tools
