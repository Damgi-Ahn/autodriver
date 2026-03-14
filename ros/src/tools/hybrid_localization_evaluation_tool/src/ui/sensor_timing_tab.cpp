#include "hybrid_localization_evaluation_tool/ui/sensor_timing_tab.hpp"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

namespace autodriver::tools {

namespace {

QChartView* BuildDelayChart(const QString& title, QLineSeries* series, const QColor& col)
{
  series->setColor(col);
  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 10, QFont::DemiBold));
  chart->legend()->setVisible(false);
  chart->addSeries(series);
  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3")); ax->setTitleText("t (s)");
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3")); ay->setTitleText("ms");
  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);
  series->attachAxis(ax); series->attachAxis(ay);
  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  view->setMinimumHeight(130);
  return view;
}

QString FmtMs(const std::optional<double>& v)
{
  if (!v) return "—";
  return QString::number(*v, 'f', 2) + " ms";
}

QString DelayBadge(const StatSummary& s)
{
  if (s.count == 0) return "—";
  return QString("mean %1  p95 %2  max %3")
      .arg(s.mean, 0, 'f', 1).arg(s.p95, 0, 'f', 1).arg(s.max, 0, 'f', 1);
}

}  // namespace

SensorTimingTab::SensorTimingTab(const std::shared_ptr<RosQtBridge>& bridge, QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  delay_gnss_series_     = new QLineSeries();
  delay_gnss_vel_series_ = new QLineSeries();
  delay_vel_series_      = new QLineSeries();
  delay_steer_series_    = new QLineSeries();

  auto* root  = new QHBoxLayout(this);
  auto* left  = new QVBoxLayout();
  auto* right = new QVBoxLayout();

  // ---- IMU dt stats -------------------------------------------------------
  auto* imu_box = new QGroupBox("IMU dt Stats");
  imu_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}");
  auto* imu_grid = new QGridLayout(imu_box);
  const auto add_row = [&](QGridLayout* g, int row, const QString& title, QLabel*& lbl) {
    g->addWidget(new QLabel(title), row, 0);
    lbl = new QLabel("—"); lbl->setStyleSheet("color:#ecf0f1;");
    g->addWidget(lbl, row, 1);
  };
  add_row(imu_grid, 0, "dt min",    imu_dt_min_label_);
  add_row(imu_grid, 1, "dt mean",   imu_dt_mean_label_);
  add_row(imu_grid, 2, "dt max",    imu_dt_max_label_);
  add_row(imu_grid, 3, "jitter (max-min)", imu_jitter_label_);
  for (int r = 0; r < 4; ++r)
    if (auto* l = qobject_cast<QLabel*>(imu_grid->itemAtPosition(r, 0)->widget()))
      l->setStyleSheet("color:#8a93a3;");
  left->addWidget(imu_box);

  // ---- Last message age badges --------------------------------------------
  auto* age_box = new QGroupBox("Last Message Age (from KPI window)");
  age_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}");
  auto* age_grid = new QGridLayout(age_box);
  add_row(age_grid, 0, "GNSS delay",     gnss_age_label_);
  add_row(age_grid, 1, "GNSS vel delay", gnss_vel_age_label_);
  add_row(age_grid, 2, "Velocity delay", vel_age_label_);
  add_row(age_grid, 3, "Steering delay", steer_age_label_);
  for (int r = 0; r < 4; ++r)
    if (auto* l = qobject_cast<QLabel*>(age_grid->itemAtPosition(r, 0)->widget()))
      l->setStyleSheet("color:#8a93a3;");
  left->addWidget(age_box);
  left->addStretch();

  // ---- Delay charts -------------------------------------------------------
  right->addWidget(BuildDelayChart("GNSS delay",     delay_gnss_series_,     QColor("#3498db")));
  right->addWidget(BuildDelayChart("GNSS vel delay", delay_gnss_vel_series_, QColor("#2ecc71")));
  right->addWidget(BuildDelayChart("Vehicle delay",  delay_vel_series_,      QColor("#e67e22")));
  right->addWidget(BuildDelayChart("Steering delay", delay_steer_series_,    QColor("#9b59b6")));
  right->addStretch();

  root->addLayout(left, 1);
  root->addLayout(right, 2);
  setLayout(root);
}

void SensorTimingTab::Refresh(const BridgeData& d)
{
  if (d.has_sample) {
    const auto& s = d.sample;
    imu_dt_min_label_->setText(FmtMs(s.imu_dt_min_ms));
    imu_dt_mean_label_->setText(FmtMs(s.imu_dt_mean_ms));
    imu_dt_max_label_->setText(FmtMs(s.imu_dt_max_ms));
    if (s.imu_dt_min_ms && s.imu_dt_max_ms) {
      const double jitter = *s.imu_dt_max_ms - *s.imu_dt_min_ms;
      const QString jstyle = jitter > 5.0 ? "color:#e74c3c;" : (jitter > 2.0 ? "color:#f39c12;" : "color:#27ae60;");
      imu_jitter_label_->setStyleSheet(jstyle);
      imu_jitter_label_->setText(QString::number(jitter, 'f', 2) + " ms");
    }
  }
  if (d.has_kpi) {
    gnss_age_label_->setText(DelayBadge(d.kpi.gnss_delay));
    gnss_vel_age_label_->setText(DelayBadge(d.kpi.gnss_vel_delay));
    vel_age_label_->setText(DelayBadge(d.kpi.velocity_delay));
    steer_age_label_->setText(DelayBadge(d.kpi.steering_delay));
  }

  const auto& hist = d.sample_history;
  if (hist.empty()) return;
  if (t_start_ < 0.0) t_start_ = hist.front().stamp.seconds();
  delay_gnss_series_->clear();
  delay_gnss_vel_series_->clear();
  delay_vel_series_->clear();
  delay_steer_series_->clear();
  for (const auto& s : hist) {
    const double t = s.stamp.seconds() - t_start_;
    if (s.gnss_delay)     delay_gnss_series_->append(t,     *s.gnss_delay);
    if (s.gnss_vel_delay) delay_gnss_vel_series_->append(t, *s.gnss_vel_delay);
    if (s.velocity_delay) delay_vel_series_->append(t,      *s.velocity_delay);
    if (s.steering_delay) delay_steer_series_->append(t,    *s.steering_delay);
  }
}

}  // namespace autodriver::tools
