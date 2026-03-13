#include "hybrid_localization_evaluation_tool/main_window.hpp"

#include <QApplication>
#include <QFont>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QStringList>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>

namespace autodriver::tools {

namespace {

QGroupBox* BuildGroup(const QString& title, QWidget* content)
{
  auto* group = new QGroupBox(title);
  auto* layout = new QVBoxLayout(group);
  layout->addWidget(content);
  group->setLayout(layout);
  return group;
}

QFrame* BuildDivider()
{
  auto* line = new QFrame();
  line->setFrameShape(QFrame::HLine);
  line->setFrameShadow(QFrame::Sunken);
  line->setFixedHeight(1);
  return line;
}

QLabel* BuildMetricLabel(const QString& text);

QFrame* BuildCard(const QString& title, QLabel** out_body_label)
{
  auto* frame = new QFrame();
  frame->setObjectName("Card");
  frame->setFrameShape(QFrame::StyledPanel);
  frame->setFrameShadow(QFrame::Raised);

  auto* layout = new QVBoxLayout(frame);
  auto* title_label = new QLabel(title);
  title_label->setObjectName("CardTitle");
  auto* body_label = BuildMetricLabel("n/a");
  body_label->setObjectName("CardBody");
  layout->addWidget(title_label);
  layout->addWidget(body_label);
  frame->setLayout(layout);
  if (out_body_label) *out_body_label = body_label;
  return frame;
}

QFrame* BuildChartPlaceholder(const QString& title)
{
  auto* frame = new QFrame();
  frame->setObjectName("Chart");
  frame->setFrameShape(QFrame::StyledPanel);
  frame->setFrameShadow(QFrame::Sunken);

  auto* layout = new QVBoxLayout(frame);
  auto* title_label = new QLabel(title);
  title_label->setObjectName("ChartTitle");
  auto* hint = new QLabel("Plot placeholder");
  hint->setObjectName("ChartHint");
  hint->setAlignment(Qt::AlignCenter);
  layout->addWidget(title_label);
  layout->addStretch(1);
  layout->addWidget(hint);
  layout->addStretch(2);
  frame->setLayout(layout);
  frame->setMinimumHeight(180);
  return frame;
}

QLabel* BuildMetricLabel(const QString& text)
{
  auto* label = new QLabel(text);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setWordWrap(true);
  return label;
}

QString FormatRatio(double ratio)
{
  return QString::number(ratio * 100.0, 'f', 1) + "%";
}

QString FormatCardStat(const StatSummary& stat)
{
  if (stat.count == 0) return "n/a";
  return QString("mean %1 | p95 %2")
      .arg(stat.mean, 0, 'f', 2)
      .arg(stat.p95, 0, 'f', 2);
}

double ReadProcessCpuSeconds()
{
  struct rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return 0.0;
  const double user = static_cast<double>(usage.ru_utime.tv_sec) +
                      static_cast<double>(usage.ru_utime.tv_usec) / 1e6;
  const double sys = static_cast<double>(usage.ru_stime.tv_sec) +
                     static_cast<double>(usage.ru_stime.tv_usec) / 1e6;
  return user + sys;
}

QString JoinReasons(const std::map<std::string, size_t>& reasons)
{
  QStringList parts;
  for (const auto& [reason, count] : reasons) {
    parts << QString::fromStdString(reason) + ":" + QString::number(count);
  }
  return parts.join(", ");
}

}  // namespace

EvaluationMainWindow::EvaluationMainWindow(const std::shared_ptr<RosQtBridge>& bridge,
                                           QWidget* parent)
    : QMainWindow(parent), bridge_(bridge)
{
  setObjectName("MainWindow");

  auto* app = qobject_cast<QApplication*>(QApplication::instance());
  if (app) {
    QFont base_font("Noto Sans");
    base_font.setPointSize(11);
    app->setFont(base_font);
  }

  setStyleSheet(
      "QWidget {"
      "  background-color: #0e1116;"
      "  color: #d6d9dd;"
      "}"
      "QGroupBox {"
      "  border: 1px solid #222831;"
      "  border-radius: 8px;"
      "  margin-top: 12px;"
      "  font-weight: 600;"
      "  padding: 8px;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  subcontrol-position: top left;"
      "  padding: 0 8px;"
      "  color: #9fb0c4;"
      "}"
      "QLabel#StatusPrimary {"
      "  font-size: 16px;"
      "  font-weight: 700;"
      "  color: #e6e9ee;"
      "}"
      "QLabel#Timestamp {"
      "  color: #8a93a3;"
      "  font-size: 12px;"
      "}"
      "QLabel#MetricLabel {"
      "  font-family: \"JetBrains Mono\";"
      "  font-size: 12px;"
      "  color: #c9d1d9;"
      "}"
      "QFrame#Card {"
      "  background-color: #141a22;"
      "  border: 1px solid #232b36;"
      "  border-radius: 10px;"
      "}"
      "QLabel#CardTitle {"
      "  color: #9fb0c4;"
      "  font-size: 11px;"
      "  letter-spacing: 0.6px;"
      "  text-transform: uppercase;"
      "}"
      "QLabel#CardBody {"
      "  font-family: \"JetBrains Mono\";"
      "  font-size: 14px;"
      "  font-weight: 600;"
      "  color: #e6e9ee;"
      "}"
      "QFrame#Chart {"
      "  background-color: #10151c;"
      "  border: 1px dashed #2d3643;"
      "  border-radius: 12px;"
      "}"
      "QLabel#ChartTitle {"
      "  color: #a3b1c6;"
      "  font-size: 12px;"
      "  font-weight: 600;"
      "}"
      "QLabel#ChartHint {"
      "  color: #556171;"
      "  font-size: 11px;"
      "}"
      "QFrame {"
      "  background-color: #1f2630;"
      "}"
  );

  auto* central = new QWidget(this);
  auto* layout = new QGridLayout(central);
  layout->setContentsMargins(18, 18, 18, 18);
  layout->setHorizontalSpacing(16);
  layout->setVerticalSpacing(16);

  status_label_ = new QLabel("Waiting for diagnostics...");
  status_label_->setObjectName("StatusPrimary");
  timestamp_label_ = new QLabel("-");
  timestamp_label_->setObjectName("Timestamp");

  kpi_label_ = BuildMetricLabel("-");
  kpi_label_->setObjectName("MetricLabel");
  nis_label_ = BuildMetricLabel("-");
  nis_label_->setObjectName("MetricLabel");
  delay_label_ = BuildMetricLabel("-");
  delay_label_->setObjectName("MetricLabel");
  reason_label_ = BuildMetricLabel("-");
  reason_label_->setObjectName("MetricLabel");

  status_label_->setWordWrap(true);

  auto* status_content = new QWidget();
  auto* status_layout = new QVBoxLayout(status_content);
  status_layout->addWidget(status_label_);
  status_layout->addWidget(timestamp_label_);
  status_layout->addWidget(BuildDivider());
  status_layout->addWidget(BuildMetricLabel("Output availability KPI is shown in Update Rates."));
  status_content->setLayout(status_layout);

  auto* left_panel = new QWidget();
  auto* left_layout = new QVBoxLayout(left_panel);
  left_layout->setSpacing(12);
  left_layout->addWidget(BuildGroup("Current Localization", status_content));
  left_layout->addWidget(BuildGroup("Update Rates", kpi_label_));
  left_layout->addWidget(BuildGroup("Reasons", reason_label_));
  left_layout->addStretch(1);

  auto* center_panel = new QWidget();
  auto* center_layout = new QVBoxLayout(center_panel);
  center_layout->setSpacing(12);
  auto* stat_row = new QHBoxLayout();
  stat_row->setSpacing(12);
  stat_row->addWidget(BuildCard("GNSS POS NIS", &nis_pos_card_));
  stat_row->addWidget(BuildCard("GNSS VEL NIS", &nis_vel_card_));
  stat_row->addWidget(BuildCard("HEADING NIS", &nis_heading_card_));
  center_layout->addLayout(stat_row);
  center_layout->addWidget(BuildChartPlaceholder("NIS Trend"));
  center_layout->addWidget(BuildChartPlaceholder("Delay Trend"));
  center_layout->addWidget(BuildGroup("Delay Summary", delay_label_));

  auto* bottom_bar = new QFrame();
  bottom_bar->setObjectName("Card");
  auto* bottom_layout = new QHBoxLayout(bottom_bar);
  bottom_layout->setContentsMargins(12, 8, 12, 8);
  hz_label_ = BuildMetricLabel("Diagnostics: n/a Hz | Output: n/a Hz");
  hz_label_->setObjectName("MetricLabel");
  cpu_label_ = BuildMetricLabel("Eval node CPU: n/a %");
  cpu_label_->setObjectName("MetricLabel");
  bottom_layout->addWidget(hz_label_);
  bottom_layout->addStretch(1);
  bottom_layout->addWidget(cpu_label_);

  layout->addWidget(left_panel, 0, 0);
  layout->addWidget(center_panel, 0, 1);
  layout->addWidget(bottom_bar, 1, 0, 1, 2);
  layout->setRowStretch(0, 1);
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 2);

  setCentralWidget(central);
  setWindowTitle("Hybrid Localization Evaluation Tool");
  resize(1080, 720);

  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { UpdateUi(); });
  timer_->start(200);

  cpu_timer_.start();
  last_cpu_seconds_ = ReadProcessCpuSeconds();
  cpu_cores_ = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  if (cpu_cores_ <= 0) cpu_cores_ = 1;
}

QString EvaluationMainWindow::FormatStat(const StatSummary& stat, const QString& unit)
{
  if (stat.count == 0) return "n/a";
  return QString("mean %1%4, p95 %2%4, max %3%4 (n=%5)")
      .arg(stat.mean, 0, 'f', 3)
      .arg(stat.p95, 0, 'f', 3)
      .arg(stat.max, 0, 'f', 3)
      .arg(unit)
      .arg(static_cast<int>(stat.count));
}

QString EvaluationMainWindow::FormatUpdateRate(const UpdateRateSummary& summary)
{
  if (summary.total == 0) return "n/a";
  return QString("%1 (%2/%3)")
      .arg(FormatRatio(summary.ratio))
      .arg(static_cast<int>(summary.applied))
      .arg(static_cast<int>(summary.total));
}

QString EvaluationMainWindow::FormatReasonHist(const UpdateRateSummary& summary)
{
  if (summary.reason_hist.empty()) return "n/a";
  return JoinReasons(summary.reason_hist);
}

void EvaluationMainWindow::UpdateUi()
{
  if (!bridge_) return;

  DiagSample sample;
  if (bridge_->GetLatestSample(&sample)) {
    status_label_->setText(QString("activated: %1 | eskf: %2")
                               .arg(sample.is_activated ? "true" : "false")
                               .arg(sample.eskf_initialized ? "true" : "false"));
    timestamp_label_->setText(QString("stamp: %1 s")
                                  .arg(sample.stamp.seconds(), 0, 'f', 3));
  }

  KpiSnapshot kpi;
  if (bridge_->GetLatestKpi(&kpi)) {
    const QString output_ratio =
        (kpi.output_availability.expected_count > 0.0)
            ? QString::number(kpi.output_availability.ratio * 100.0, 'f', 1) + "%"
            : "n/a";
    const QString output_age =
        kpi.output_availability.has_output
            ? QString::number(kpi.output_availability.last_age_sec, 'f', 2) + " s"
            : "n/a";

    kpi_label_->setText(QString("pos %1 | vel %2 | heading %3\noutput %4 (age %5)")
                            .arg(FormatUpdateRate(kpi.gnss_pos_update))
                            .arg(FormatUpdateRate(kpi.gnss_vel_update))
                            .arg(FormatUpdateRate(kpi.heading_yaw_update))
                            .arg(output_ratio)
                            .arg(output_age));

    reason_label_->setText(QString("pos: %1\nvel: %2\nheading: %3")
                               .arg(FormatReasonHist(kpi.gnss_pos_update))
                               .arg(FormatReasonHist(kpi.gnss_vel_update))
                               .arg(FormatReasonHist(kpi.heading_yaw_update)));

    nis_label_->setText(QString("pos: %1\nvel: %2\nheading: %3")
                            .arg(FormatStat(kpi.gnss_pos_nis, ""))
                            .arg(FormatStat(kpi.gnss_vel_nis, ""))
                            .arg(FormatStat(kpi.heading_yaw_nis, "")));

    delay_label_->setText(QString("gnss: %1\nvel: %2\nvehicle vel: %3\nsteer: %4")
                              .arg(FormatStat(kpi.gnss_delay, " s"))
                              .arg(FormatStat(kpi.gnss_vel_delay, " s"))
                              .arg(FormatStat(kpi.velocity_delay, " s"))
                              .arg(FormatStat(kpi.steering_delay, " s")));

    if (nis_pos_card_) nis_pos_card_->setText(FormatCardStat(kpi.gnss_pos_nis));
    if (nis_vel_card_) nis_vel_card_->setText(FormatCardStat(kpi.gnss_vel_nis));
    if (nis_heading_card_) {
      nis_heading_card_->setText(FormatCardStat(kpi.heading_yaw_nis));
    }
    if (hz_label_) {
      hz_label_->setText(QString("Diagnostics: %1 Hz | Output: %2 Hz")
                             .arg(kpi.diag_rate_hz, 0, 'f', 1)
                             .arg(kpi.output_rate_hz, 0, 'f', 1));
    }
  }

  if (cpu_label_) {
    const double wall_sec = cpu_timer_.elapsed() / 1000.0;
    const double cpu_now = ReadProcessCpuSeconds();
    const double cpu_delta = cpu_now - last_cpu_seconds_;
    const double wall_delta = wall_sec;
    if (wall_delta > 0.0) {
      const double pct = (cpu_delta / (wall_delta * cpu_cores_)) * 100.0;
      cpu_label_->setText(QString("Eval node CPU: %1 %")
                              .arg(std::max(0.0, pct), 0, 'f', 1));
    }
    cpu_timer_.restart();
    last_cpu_seconds_ = cpu_now;
  }
}

}  // namespace autodriver::tools
