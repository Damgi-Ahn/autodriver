#include "hybrid_localization_evaluation_tool/evaluation_node.hpp"

#include <chrono>
#include <cmath>
#include <utility>

namespace autodriver::tools {

namespace {

double YawFromQuaternion(double x, double y, double z, double w)
{
  const double siny = 2.0 * (w * z + x * y);
  const double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

PoseSnapshot BuildPoseSnapshot(const rclcpp::Time& stamp,
                               const geometry_msgs::msg::Point& p,
                               const geometry_msgs::msg::Quaternion& q)
{
  PoseSnapshot pose;
  pose.has_pose  = true;
  pose.stamp     = stamp;
  pose.x         = p.x;
  pose.y         = p.y;
  pose.z         = p.z;
  pose.yaw_rad   = YawFromQuaternion(q.x, q.y, q.z, q.w);
  return pose;
}

}  // namespace

EvaluationNode::EvaluationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hybrid_localization_evaluation_tool", options),
      kpi_engine_(declare_parameter("window_sec", 10.0),
                  declare_parameter("expected_output_rate_hz", 50.0))
{
  // --- Alert thresholds from parameters ---
  AlertThresholds thr;
  thr.nis_gate_gnss_pos  = declare_parameter("nis_gate_gnss_pos",  11.34);
  thr.nis_gate_gnss_vel  = declare_parameter("nis_gate_gnss_vel",  11.34);
  thr.nis_gate_heading   = declare_parameter("nis_gate_heading",    6.63);
  thr.delay_warn_ms      = declare_parameter("alert_delay_warn_ms",  200.0);
  thr.delay_error_ms     = declare_parameter("alert_delay_error_ms", 500.0);
  thr.P_trace_warn       = declare_parameter("alert_P_trace_warn",  100.0);
  thr.P_trace_error      = declare_parameter("alert_P_trace_error", 1000.0);
  thr.imu_dt_jitter_warn_ms  = declare_parameter("alert_imu_dt_jitter_warn_ms",  2.0);
  thr.imu_dt_jitter_error_ms = declare_parameter("alert_imu_dt_jitter_error_ms", 5.0);
  thr.output_avail_warn  = declare_parameter("alert_output_avail_warn",  0.90);
  thr.output_avail_error = declare_parameter("alert_output_avail_error", 0.70);
  alert_engine_.set_thresholds(thr);

  // Propagate NIS gates to KpiEngine for violation rate calculation
  kpi_engine_.set_nis_gates(thr.nis_gate_gnss_pos, thr.nis_gate_gnss_vel, thr.nis_gate_heading);

  // --- CSV export ---
  const std::string csv_dir = declare_parameter("csv_output_dir", std::string{});
  if (!csv_dir.empty()) {
    if (exporter_.Enable(csv_dir)) {
      RCLCPP_INFO(get_logger(), "CSV export enabled: %s",  csv_dir.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to enable CSV export at %s", csv_dir.c_str());
    }
  }

  session_store_.SetSessionStart(now());

  // --- Subscriptions ---
  diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10},
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        OnDiagnostics(std::move(msg));
      });

  // /localization/kinematic_state → output availability counter
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnOutputOdom(std::move(msg)); });

  gnss_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/sensing/gnss/pose_with_covariance", rclcpp::QoS{10},
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        OnGnssPose(std::move(msg));
      });

  // /localization/kinematic_state also used as ESKF pose card
  eskf_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnEskfOdom(std::move(msg)); });

  fgo_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/localization/pose_twist_fusion_filter/pose", rclcpp::QoS{10},
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { OnFgoPose(std::move(msg)); });

  state_sub_ = create_subscription<std_msgs::msg::String>(
      "/localization/state", rclcpp::QoS{10},
      [this](std_msgs::msg::String::SharedPtr msg) {
        OnLocalizationState(std::move(msg));
      });

  keyframe_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/localization/fgo/keyframe_path", rclcpp::QoS{10},
      [this](nav_msgs::msg::Path::SharedPtr msg) { OnKeyframePath(std::move(msg)); });

  // /localization/kinematic_state_gnss → ground truth (RTK GNSS accuracy)
  gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state_gnss", rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnGroundTruth(std::move(msg)); });

  // KPI timer: 1 Hz aggregation
  kpi_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() { OnKpiTimer(); });
}

void EvaluationNode::SetBridge(const std::shared_ptr<RosQtBridge>& bridge)
{
  bridge_ = bridge;
}

void EvaluationNode::OnDiagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
  DiagSample sample;
  std::string error;
  if (!parser_.Parse(*msg, &sample, &error)) {
    return;
  }
  if (sample.stamp.nanoseconds() == 0) {
    sample.stamp = now();
  }

  kpi_engine_.AddSample(sample);
  session_store_.AddSample(sample);

  // Alert evaluation
  const auto alerts = alert_engine_.Evaluate(sample);
  if (!alerts.empty()) {
    session_store_.AddAlerts(alerts);
    if (bridge_) bridge_->AddAlerts(alerts);
    if (exporter_.enabled()) exporter_.WriteEvents(alerts);
  }

  if (bridge_) bridge_->UpdateSample(sample);
  if (exporter_.enabled()) exporter_.WriteRawSample(sample);
}

void EvaluationNode::OnOutputOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  if (stamp.nanoseconds() == 0) stamp = now();
  kpi_engine_.AddOutputStamp(stamp);
}

void EvaluationNode::OnGnssPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  const auto pose = BuildPoseSnapshot(stamp, msg->pose.pose.position, msg->pose.pose.orientation);
  session_store_.AddGnssPoint(pose.x, pose.y);
  if (bridge_) bridge_->UpdateGnssPose(pose);
}

void EvaluationNode::OnEskfOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  const auto pose = BuildPoseSnapshot(stamp, msg->pose.pose.position, msg->pose.pose.orientation);
  session_store_.AddEskfPoint(pose.x, pose.y);
  gt_analyzer_.UpdateEskfPose(pose.x, pose.y, pose.yaw_rad);
  if (bridge_) bridge_->UpdateEskfPose(pose);
}

void EvaluationNode::OnFgoPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  const auto pose = BuildPoseSnapshot(stamp, msg->pose.position, msg->pose.orientation);
  session_store_.AddFgoPoint(pose.x, pose.y);
  gt_analyzer_.UpdateFgoPose(pose.x, pose.y, pose.yaw_rad);
  if (bridge_) bridge_->UpdateFgoPose(pose);
}

void EvaluationNode::OnGroundTruth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  const auto pose = BuildPoseSnapshot(stamp, msg->pose.pose.position, msg->pose.pose.orientation);
  gt_analyzer_.AddGroundTruth(stamp, pose.x, pose.y, pose.yaw_rad);
}

void EvaluationNode::OnLocalizationState(const std_msgs::msg::String::SharedPtr msg)
{
  if (bridge_) bridge_->UpdateLocalizationState(msg->data);
}

void EvaluationNode::OnKeyframePath(const nav_msgs::msg::Path::SharedPtr /*msg*/)
{
  // Keyframe path is rendered directly from the trajectory store;
  // FGO pose updates already populate fgo_traj_ via OnFgoPose.
  // This subscription is a hook for future keyframe marker rendering.
}

void EvaluationNode::OnKpiTimer()
{
  const auto snapshot = kpi_engine_.ComputeSnapshot(now());

  // Output availability alert
  const auto avail_alerts = alert_engine_.EvaluateOutputAvailability(
      now(), snapshot.output_availability.ratio);
  if (!avail_alerts.empty()) {
    session_store_.AddAlerts(avail_alerts);
    if (bridge_) bridge_->AddAlerts(avail_alerts);
    if (exporter_.enabled()) exporter_.WriteEvents(avail_alerts);
  }

  // Update health state machine
  const bool eskf_init = [this]() {
    DiagSample s;
    return bridge_ && bridge_->GetLatestSample(&s) && s.eskf_initialized;
  }();
  const HealthState health = health_engine_.Update(snapshot, eskf_init);

  if (bridge_) {
    bridge_->UpdateKpi(snapshot);
    bridge_->UpdateTrajectories(session_store_);
    bridge_->UpdateHealthState(health);
    bridge_->UpdateGtData(gt_analyzer_);
  }
  if (exporter_.enabled()) exporter_.WriteKpiSnapshot(snapshot);
}

}  // namespace autodriver::tools
