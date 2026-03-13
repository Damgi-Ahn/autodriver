#include "hybrid_localization_evaluation_tool/evaluation_node.hpp"

#include <chrono>
#include <utility>

namespace autodriver::tools {

EvaluationNode::EvaluationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("hybrid_localization_evaluation_tool", options),
      kpi_engine_(declare_parameter("window_sec", 10.0),
                  declare_parameter("expected_output_rate_hz", 50.0))
{
  const std::string csv_dir = declare_parameter("csv_output_dir", std::string{});
  if (!csv_dir.empty()) {
    if (exporter_.Enable(csv_dir)) {
      RCLCPP_INFO(get_logger(), "CSV export enabled: %s , %s",
                  exporter_.raw_path().c_str(), exporter_.kpi_path().c_str());
    } else {
      RCLCPP_WARN(get_logger(), "Failed to enable CSV export at %s", csv_dir.c_str());
    }
  }

  diag_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10},
      [this](diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg) {
        OnDiagnostics(std::move(msg));
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization/kinematic_state", rclcpp::QoS{10},
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { OnOutputOdom(std::move(msg)); });

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

  if (bridge_) bridge_->UpdateSample(sample);
  if (exporter_.enabled()) exporter_.WriteRawSample(sample);
}

void EvaluationNode::OnOutputOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  rclcpp::Time stamp = rclcpp::Time(msg->header.stamp);
  if (stamp.nanoseconds() == 0) {
    stamp = now();
  }
  kpi_engine_.AddOutputStamp(stamp);
}

void EvaluationNode::OnKpiTimer()
{
  const auto snapshot = kpi_engine_.ComputeSnapshot(now());
  if (bridge_) bridge_->UpdateKpi(snapshot);
  if (exporter_.enabled()) exporter_.WriteKpiSnapshot(snapshot);
}

}  // namespace autodriver::tools
