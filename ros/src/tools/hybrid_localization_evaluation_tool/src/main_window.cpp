#include "hybrid_localization_evaluation_tool/main_window.hpp"

#include <QApplication>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QStatusBar>
#include <QVBoxLayout>

#include <sys/resource.h>
#include <unistd.h>

#include <algorithm>
#include <functional>

namespace autodriver::tools {

namespace {

double ReadProcessCpuSeconds()
{
  struct rusage usage {};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return 0.0;
  return static_cast<double>(usage.ru_utime.tv_sec)
       + static_cast<double>(usage.ru_utime.tv_usec) / 1e6
       + static_cast<double>(usage.ru_stime.tv_sec)
       + static_cast<double>(usage.ru_stime.tv_usec) / 1e6;
}

}  // namespace

EvaluationMainWindow::EvaluationMainWindow(const std::shared_ptr<RosQtBridge>& bridge,
                                           StorageExporter* exporter,
                                           QWidget* parent)
    : QMainWindow(parent), bridge_(bridge)
{
  // ---- Global font & palette -----------------------------------------------
  if (auto* app = qobject_cast<QApplication*>(QApplication::instance())) {
    QFont f("Noto Sans");
    f.setPointSize(11);
    app->setFont(f);
  }

  setStyleSheet(
      "QMainWindow, QWidget {"
      "  background-color: #0e1116;"
      "  color: #d6d9dd;"
      "}"
      "QTabWidget::pane {"
      "  border: 1px solid #222831;"
      "  border-radius: 4px;"
      "  background-color: #0e1116;"
      "}"
      "QTabBar::tab {"
      "  background: #141a22;"
      "  color: #8a93a3;"
      "  padding: 7px 16px;"
      "  border: 1px solid #222831;"
      "  border-bottom: none;"
      "  border-top-left-radius: 5px;"
      "  border-top-right-radius: 5px;"
      "  min-width: 90px;"
      "}"
      "QTabBar::tab:selected {"
      "  background: #1f2936;"
      "  color: #e6e9ee;"
      "  font-weight: 600;"
      "  border-bottom: 2px solid #3498db;"
      "}"
      "QTabBar::tab:hover:!selected {"
      "  background: #1a2230;"
      "  color: #c9d1d9;"
      "}"
      "QGroupBox {"
      "  border: 1px solid #222831;"
      "  border-radius: 6px;"
      "  margin-top: 10px;"
      "  font-weight: 600;"
      "  padding: 6px;"
      "}"
      "QGroupBox::title {"
      "  subcontrol-origin: margin;"
      "  subcontrol-position: top left;"
      "  padding: 0 6px;"
      "  color: #9fb0c4;"
      "}"
      "QLabel {"
      "  color: #d6d9dd;"
      "}"
      "QStatusBar {"
      "  background-color: #0a0e14;"
      "  color: #8a93a3;"
      "  font-size: 11px;"
      "}"
      "QSplitter::handle {"
      "  background: #222831;"
      "}"
  );

  // ---- Tabs ----------------------------------------------------------------
  tabs_ = new QTabWidget(this);
  tabs_->setDocumentMode(true);

  tab_overview_   = new OverviewTab(bridge_);
  tab_fusion_     = new FusionQualityTab(bridge_);
  tab_covariance_ = new CovarianceNoiseTab(bridge_);
  tab_timing_     = new SensorTimingTab(bridge_);
  tab_trajectory_ = new PoseTrajectoryTab(bridge_);
  tab_events_     = new EventsInspectorTab(bridge_);
  tab_export_     = new ExportSessionTab(bridge_, exporter);

  tabs_->addTab(tab_overview_,   "Overview");
  tabs_->addTab(tab_fusion_,     "Fusion Quality");
  tabs_->addTab(tab_covariance_, "Covariance");
  tabs_->addTab(tab_timing_,     "Sensor Timing");
  tabs_->addTab(tab_trajectory_, "Trajectory");
  tabs_->addTab(tab_events_,     "Events");
  tabs_->addTab(tab_export_,     "Export");

  setCentralWidget(tabs_);

  // ---- Status bar ----------------------------------------------------------
  hz_label_ = new QLabel("Diagnostics: — Hz | Output: — Hz");
  cpu_label_ = new QLabel("CPU: — %");
  statusBar()->addPermanentWidget(hz_label_);
  statusBar()->addPermanentWidget(cpu_label_);

  // ---- Window properties ---------------------------------------------------
  setWindowTitle("Hybrid Localization Evaluation Tool");
  resize(1280, 800);

  // ---- Refresh timer -------------------------------------------------------
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { UpdateUi(); });
  timer_->start(200);

  cpu_timer_.start();
  last_cpu_seconds_ = ReadProcessCpuSeconds();
  cpu_cores_ = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  if (cpu_cores_ <= 0) cpu_cores_ = 1;
}

void EvaluationMainWindow::SetNisGates(double pos, double vel, double heading)
{
  if (tab_fusion_) tab_fusion_->SetNisGates(pos, vel, heading);
}

void EvaluationMainWindow::SetOnFlushSessionSummary(std::function<void()> cb)
{
  if (tab_export_) tab_export_->SetOnFlushSessionSummary(std::move(cb));
}

void EvaluationMainWindow::SetOnClearSession(std::function<void()> cb)
{
  if (tab_export_) tab_export_->SetOnClearSession(std::move(cb));
}

void EvaluationMainWindow::UpdateUi()
{
  if (!bridge_) return;

  const BridgeData d = bridge_->Snapshot();
  const int active_idx = tabs_->currentIndex();

  // Always refresh Overview (tab 0) to keep banner/KPI live
  if (tab_overview_) tab_overview_->Refresh(d);

  // Refresh only the active tab for the heavier updates (charts etc.)
  switch (active_idx) {
    case 1: if (tab_fusion_)     tab_fusion_->Refresh(d);     break;
    case 2: if (tab_covariance_) tab_covariance_->Refresh(d); break;
    case 3: if (tab_timing_)     tab_timing_->Refresh(d);     break;
    case 4: if (tab_trajectory_) tab_trajectory_->Refresh(d); break;
    case 5: if (tab_events_)     tab_events_->Refresh(d);     break;
    case 6: if (tab_export_)     tab_export_->Refresh(d);     break;
    default: break;
  }

  // ---- Status bar ----------------------------------------------------------
  if (d.has_kpi) {
    hz_label_->setText(QString("Diagnostics: %1 Hz | Output: %2 Hz")
                           .arg(d.kpi.diag_rate_hz,   0, 'f', 1)
                           .arg(d.kpi.output_rate_hz, 0, 'f', 1));
  }

  const double wall_sec   = cpu_timer_.elapsed() / 1000.0;
  const double cpu_now    = ReadProcessCpuSeconds();
  const double cpu_delta  = cpu_now - last_cpu_seconds_;
  if (wall_sec > 0.0) {
    const double pct = (cpu_delta / (wall_sec * cpu_cores_)) * 100.0;
    cpu_label_->setText(QString("CPU: %1 %").arg(std::max(0.0, pct), 0, 'f', 1));
  }
  cpu_timer_.restart();
  last_cpu_seconds_ = cpu_now;
}

}  // namespace autodriver::tools
