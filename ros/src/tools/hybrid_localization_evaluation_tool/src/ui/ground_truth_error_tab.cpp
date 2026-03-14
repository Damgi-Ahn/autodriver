#include "hybrid_localization_evaluation_tool/ui/ground_truth_error_tab.hpp"

#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QSplitter>
#include <QVBoxLayout>

#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

namespace autodriver::tools {

namespace {

QLabel* MakeStatLabel(const QString& text)
{
  auto* lbl = new QLabel(text);
  lbl->setAlignment(Qt::AlignCenter);
  lbl->setWordWrap(true);
  lbl->setStyleSheet("background:#1a2030; border:1px solid #2c3e50;"
                     " border-radius:4px; padding:8px; color:#ecf0f1; font-size:12px;");
  lbl->setMinimumHeight(48);
  return lbl;
}

QChartView* MakeLineChart(const QString& title,
                           std::initializer_list<std::pair<QLineSeries*, QColor>> series_list)
{
  auto* chart = new QChart();
  chart->setTitle(title);
  chart->setBackgroundBrush(QColor("#10151c"));
  chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  chart->setTitleFont(QFont("Noto Sans", 9));
  chart->setMargins(QMargins(4, 4, 4, 4));
  chart->legend()->setLabelColor(QColor("#a3b1c6"));

  auto* ax = new QValueAxis();
  ax->setLabelsColor(QColor("#8a93a3"));
  ax->setTitleText("t (s)");
  ax->setTitleBrush(QBrush(QColor("#8a93a3")));
  auto* ay = new QValueAxis();
  ay->setLabelsColor(QColor("#8a93a3"));
  ay->setTitleText("error (m)");
  ay->setTitleBrush(QBrush(QColor("#8a93a3")));

  chart->addAxis(ax, Qt::AlignBottom);
  chart->addAxis(ay, Qt::AlignLeft);

  for (auto& [s, c] : series_list) {
    s->setColor(c);
    chart->addSeries(s);
    s->attachAxis(ax);
    s->attachAxis(ay);
  }

  auto* view = new QChartView(chart);
  view->setRenderHint(QPainter::Antialiasing);
  return view;
}

}  // namespace

GroundTruthErrorTab::GroundTruthErrorTab(const std::shared_ptr<RosQtBridge>& bridge,
                                         QWidget* parent)
    : QWidget(parent), bridge_(bridge)
{
  auto* root = new QHBoxLayout(this);
  root->setSpacing(8);

  // ---- Left panel: stats ---------------------------------------------------
  auto* left = new QWidget();
  auto* left_layout = new QVBoxLayout(left);
  left->setMaximumWidth(260);
  left->setMinimumWidth(220);

  gt_status_label_ = new QLabel("Ground Truth: WAITING");
  gt_status_label_->setAlignment(Qt::AlignCenter);
  gt_status_label_->setStyleSheet("background:#1a2030; border-radius:4px; padding:8px;"
                                  " color:#8a93a3; font-weight:bold;");
  left_layout->addWidget(gt_status_label_);

  sample_count_label_ = new QLabel("Samples: 0");
  sample_count_label_->setAlignment(Qt::AlignCenter);
  sample_count_label_->setStyleSheet("color:#8a93a3; font-size:11px;");
  left_layout->addWidget(sample_count_label_);

  // ESKF ATE box
  auto* eskf_box = new QGroupBox("ESKF ATE vs GT");
  eskf_box->setStyleSheet("QGroupBox{color:#3498db; border:1px solid #2c3e50; margin-top:8px;}"
                          "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* eskf_layout = new QVBoxLayout(eskf_box);
  eskf_mean_label_ = MakeStatLabel("Mean: —");
  eskf_rmse_label_ = MakeStatLabel("RMSE: —");
  eskf_max_label_  = MakeStatLabel("Max:  —");
  eskf_layout->addWidget(eskf_mean_label_);
  eskf_layout->addWidget(eskf_rmse_label_);
  eskf_layout->addWidget(eskf_max_label_);
  left_layout->addWidget(eskf_box);

  // FGO ATE box
  auto* fgo_box = new QGroupBox("FGO ATE vs GT");
  fgo_box->setStyleSheet("QGroupBox{color:#e67e22; border:1px solid #2c3e50; margin-top:8px;}"
                         "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* fgo_layout = new QVBoxLayout(fgo_box);
  fgo_mean_label_ = MakeStatLabel("Mean: —");
  fgo_rmse_label_ = MakeStatLabel("RMSE: —");
  fgo_max_label_  = MakeStatLabel("Max:  —");
  fgo_layout->addWidget(fgo_mean_label_);
  fgo_layout->addWidget(fgo_rmse_label_);
  fgo_layout->addWidget(fgo_max_label_);
  left_layout->addWidget(fgo_box);

  // Yaw error
  auto* yaw_box = new QGroupBox("Yaw Error (ESKF)");
  yaw_box->setStyleSheet("QGroupBox{color:#9b59b6; border:1px solid #2c3e50; margin-top:8px;}"
                         "QGroupBox::title{subcontrol-origin:margin; left:8px;}");
  auto* yaw_layout = new QVBoxLayout(yaw_box);
  yaw_error_label_ = MakeStatLabel("Latest: —");
  yaw_layout->addWidget(yaw_error_label_);
  left_layout->addWidget(yaw_box);

  left_layout->addStretch();
  root->addWidget(left);

  // ---- Right panel: charts -------------------------------------------------
  auto* right_splitter = new QSplitter(Qt::Vertical);
  right_splitter->setHandleWidth(4);

  eskf_ate_series_ = new QLineSeries();
  eskf_ate_series_->setName("ESKF");
  fgo_ate_series_  = new QLineSeries();
  fgo_ate_series_->setName("FGO");

  ate_chart_view_ = MakeLineChart("ATE over Time",
      {{eskf_ate_series_, QColor("#3498db")},
       {fgo_ate_series_,  QColor("#e67e22")}});
  right_splitter->addWidget(ate_chart_view_);

  yaw_err_series_  = new QLineSeries();
  yaw_err_series_->setName("Yaw err (deg)");
  yaw_err_series_->setColor(QColor("#9b59b6"));

  auto* yaw_chart = new QChart();
  yaw_chart->setTitle("Yaw Error (ESKF vs GT)");
  yaw_chart->setBackgroundBrush(QColor("#10151c"));
  yaw_chart->setTitleBrush(QBrush(QColor("#a3b1c6")));
  yaw_chart->setTitleFont(QFont("Noto Sans", 9));
  yaw_chart->setMargins(QMargins(4, 4, 4, 4));
  yaw_chart->legend()->setLabelColor(QColor("#a3b1c6"));
  auto* yaw_ax = new QValueAxis(); yaw_ax->setLabelsColor(QColor("#8a93a3")); yaw_ax->setTitleText("t (s)");
  auto* yaw_ay = new QValueAxis(); yaw_ay->setLabelsColor(QColor("#8a93a3")); yaw_ay->setTitleText("deg");
  yaw_chart->addAxis(yaw_ax, Qt::AlignBottom); yaw_chart->addAxis(yaw_ay, Qt::AlignLeft);
  yaw_chart->addSeries(yaw_err_series_);
  yaw_err_series_->attachAxis(yaw_ax); yaw_err_series_->attachAxis(yaw_ay);
  yaw_chart_view_ = new QChartView(yaw_chart);
  yaw_chart_view_->setRenderHint(QPainter::Antialiasing);
  right_splitter->addWidget(yaw_chart_view_);

  root->addWidget(right_splitter, 1);
  setLayout(root);
}

void GroundTruthErrorTab::Refresh(const BridgeData& d)
{
  if (!d.has_gt) {
    gt_status_label_->setText("Ground Truth: NO DATA\n(Waiting for /localization/kinematic_state_gnss)");
    gt_status_label_->setStyleSheet("background:#1a2030; border-radius:4px; padding:8px;"
                                    " color:#8a93a3; font-weight:bold;");
    return;
  }

  gt_status_label_->setText("Ground Truth: ACTIVE");
  gt_status_label_->setStyleSheet("background:#1a3020; border-radius:4px; padding:8px;"
                                  " color:#27ae60; font-weight:bold;");

  const auto& ea = d.eskf_ate;
  const auto& fa = d.fgo_ate;

  sample_count_label_->setText(QString("Samples: %1").arg(ea.count));

  if (ea.count > 0) {
    eskf_mean_label_->setText(QString("Mean: %1 m").arg(ea.mean_m, 0, 'f', 3));
    eskf_rmse_label_->setText(QString("RMSE: %1 m").arg(ea.rmse_m, 0, 'f', 3));
    eskf_max_label_->setText( QString("Max:  %1 m").arg(ea.max_m,  0, 'f', 3));
  }
  if (fa.count > 0) {
    fgo_mean_label_->setText(QString("Mean: %1 m").arg(fa.mean_m, 0, 'f', 3));
    fgo_rmse_label_->setText(QString("RMSE: %1 m").arg(fa.rmse_m, 0, 'f', 3));
    fgo_max_label_->setText( QString("Max:  %1 m").arg(fa.max_m,  0, 'f', 3));
  }

  // ---- Charts --------------------------------------------------------------
  const auto& hist = d.gt_error_history;
  if (hist.empty()) return;

  if (t_start_ < 0.0) t_start_ = hist.front().stamp.seconds();

  eskf_ate_series_->clear();
  fgo_ate_series_->clear();
  yaw_err_series_->clear();

  for (const auto& s : hist) {
    const double t = s.stamp.seconds() - t_start_;
    eskf_ate_series_->append(t, s.eskf_ate_m);
    fgo_ate_series_->append(t, s.fgo_ate_m);
    yaw_err_series_->append(t, s.yaw_error_deg);
  }

  // Update yaw error latest
  if (!hist.empty()) {
    yaw_error_label_->setText(
        QString("Latest: %1°").arg(hist.back().yaw_error_deg, 0, 'f', 2));
  }
}

}  // namespace autodriver::tools
