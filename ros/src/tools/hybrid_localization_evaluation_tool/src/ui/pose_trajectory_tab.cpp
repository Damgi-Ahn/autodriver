#include "hybrid_localization_evaluation_tool/ui/pose_trajectory_tab.hpp"

#include <QCheckBox>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPainter>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

#include <cmath>

namespace autodriver::tools {

namespace {

constexpr double kDeg = 180.0 / M_PI;

QLabel* MakeCard(const QString& title, QLabel*& body, QWidget* parent = nullptr)
{
  (void)parent;
  auto* lbl = new QLabel(title);
  lbl->setStyleSheet("color:#8a93a3; font-size:10px;");
  body = new QLabel("—");
  body->setStyleSheet("color:#ecf0f1; font-family:'JetBrains Mono'; font-size:11px;");
  body->setWordWrap(true);
  return lbl;
}

QString FmtPose(const PoseSnapshot& p)
{
  if (!p.has_pose) return "—";
  return QString("x %1\ny %2\nz %3")
      .arg(p.x, 0, 'f', 3)
      .arg(p.y, 0, 'f', 3)
      .arg(p.z, 0, 'f', 3);
}

QString FmtYaw(const PoseSnapshot& p)
{
  if (!p.has_pose) return "—";
  return QString("%1 °").arg(p.yaw_rad * kDeg, 0, 'f', 2);
}

}  // namespace

PoseTrajectoryTab::PoseTrajectoryTab(const std::shared_ptr<RosQtBridge>& bridge,
                                     QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  // --- Trajectory series ---------------------------------------------------
  gnss_series_ = new QScatterSeries();
  gnss_series_->setName("GNSS");
  gnss_series_->setColor(QColor("#3498db"));
  gnss_series_->setMarkerSize(4.0);
  gnss_series_->setBorderColor(Qt::transparent);

  eskf_series_ = new QLineSeries();
  eskf_series_->setName("ESKF");
  eskf_series_->setColor(QColor("#2ecc71"));
  eskf_series_->setPen(QPen(QColor("#2ecc71"), 2));

  fgo_series_ = new QLineSeries();
  fgo_series_->setName("FGO");
  fgo_series_->setColor(QColor("#e67e22"));
  fgo_series_->setPen(QPen(QColor("#e67e22"), 2));

  auto* chart = new QChart();
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitle("2D Trajectory");
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 10, QFont::DemiBold));
  chart->legend()->setVisible(true);
  chart->legend()->setLabelColor(QColor("#a3b1c6"));
  chart->addSeries(gnss_series_);
  chart->addSeries(eskf_series_);
  chart->addSeries(fgo_series_);

  auto* ax = new QValueAxis(); ax->setLabelsColor(QColor("#8a93a3")); ax->setTitleText("E (m)");
  auto* ay = new QValueAxis(); ay->setLabelsColor(QColor("#8a93a3")); ay->setTitleText("N (m)");
  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);
  gnss_series_->attachAxis(ax); gnss_series_->attachAxis(ay);
  eskf_series_->attachAxis(ax); eskf_series_->attachAxis(ay);
  fgo_series_->attachAxis(ax);  fgo_series_->attachAxis(ay);

  traj_view_ = new QChartView(chart);
  traj_view_->setRenderHint(QPainter::Antialiasing);
  traj_view_->setMinimumSize(400, 350);

  // --- Layout --------------------------------------------------------------
  auto* root  = new QHBoxLayout(this);
  auto* left  = new QVBoxLayout();
  auto* right = new QVBoxLayout();

  // Pose cards
  auto make_pose_box = [&](const QString& title, QLabel*& pos_lbl, QLabel*& yaw_lbl) {
    auto* box = new QGroupBox(title);
    box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}"
                       "QGroupBox::title{subcontrol-origin:margin;left:8px;}");
    auto* g = new QGridLayout(box);

    auto* pos_title = MakeCard("position (m)", pos_lbl);
    auto* yaw_title = MakeCard("yaw", yaw_lbl);
    g->addWidget(pos_title, 0, 0);
    g->addWidget(pos_lbl,   0, 1);
    g->addWidget(yaw_title, 1, 0);
    g->addWidget(yaw_lbl,   1, 1);
    return box;
  };

  left->addWidget(make_pose_box("GNSS Pose",  gnss_pos_label_, gnss_yaw_label_));
  left->addWidget(make_pose_box("ESKF Pose",  eskf_pos_label_, eskf_yaw_label_));
  left->addWidget(make_pose_box("FGO Pose",   fgo_pos_label_,  fgo_yaw_label_));

  // Layer toggles
  auto* toggle_box = new QGroupBox("Trajectory Layers");
  toggle_box->setStyleSheet("QGroupBox{color:#a3b1c6;border:1px solid #2c3e50;margin-top:8px;}");
  auto* tg_layout = new QHBoxLayout(toggle_box);
  gnss_toggle_ = new QCheckBox("GNSS");  gnss_toggle_->setChecked(true);
  eskf_toggle_ = new QCheckBox("ESKF");  eskf_toggle_->setChecked(true);
  fgo_toggle_  = new QCheckBox("FGO");   fgo_toggle_->setChecked(true);
  gnss_toggle_->setStyleSheet("color:#3498db;");
  eskf_toggle_->setStyleSheet("color:#2ecc71;");
  fgo_toggle_->setStyleSheet("color:#e67e22;");
  tg_layout->addWidget(gnss_toggle_);
  tg_layout->addWidget(eskf_toggle_);
  tg_layout->addWidget(fgo_toggle_);
  left->addWidget(toggle_box);
  left->addStretch();

  right->addWidget(traj_view_, 1);

  root->addLayout(left, 1);
  root->addLayout(right, 3);
  setLayout(root);

  // Connect toggles to series visibility
  connect(gnss_toggle_, &QCheckBox::toggled, this, [this](bool v) {
    gnss_series_->setVisible(v);
  });
  connect(eskf_toggle_, &QCheckBox::toggled, this, [this](bool v) {
    eskf_series_->setVisible(v);
  });
  connect(fgo_toggle_, &QCheckBox::toggled, this, [this](bool v) {
    fgo_series_->setVisible(v);
  });
}

void PoseTrajectoryTab::Refresh(const BridgeData& d)
{
  // Update pose cards
  gnss_pos_label_->setText(FmtPose(d.gnss_pose));
  gnss_yaw_label_->setText(FmtYaw(d.gnss_pose));
  eskf_pos_label_->setText(FmtPose(d.eskf_pose));
  eskf_yaw_label_->setText(FmtYaw(d.eskf_pose));
  fgo_pos_label_->setText(FmtPose(d.fgo_pose));
  fgo_yaw_label_->setText(FmtYaw(d.fgo_pose));

  // Update trajectories
  gnss_series_->clear();
  eskf_series_->clear();
  fgo_series_->clear();

  // Compute a common origin offset for local coordinate display
  // Use first ESKF point as origin if available, else GNSS
  double ox = 0.0, oy = 0.0;
  if (!d.eskf_traj.empty()) {
    ox = d.eskf_traj.front().x;
    oy = d.eskf_traj.front().y;
  } else if (!d.gnss_traj.empty()) {
    ox = d.gnss_traj.front().x;
    oy = d.gnss_traj.front().y;
  }

  for (const auto& pt : d.gnss_traj) {
    gnss_series_->append(pt.x - ox, pt.y - oy);
  }
  for (const auto& pt : d.eskf_traj) {
    eskf_series_->append(pt.x - ox, pt.y - oy);
  }
  for (const auto& pt : d.fgo_traj) {
    fgo_series_->append(pt.x - ox, pt.y - oy);
  }
}

}  // namespace autodriver::tools
