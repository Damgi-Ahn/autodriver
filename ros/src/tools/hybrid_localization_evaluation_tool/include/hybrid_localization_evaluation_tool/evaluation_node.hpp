#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"
#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace autodriver::tools {

class EvaluationNode : public rclcpp::Node {
 public:
  explicit EvaluationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void SetBridge(const std::shared_ptr<RosQtBridge>& bridge);

 private:
  void OnDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void OnOutputOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnKpiTimer();

  DiagnosticParser parser_;
  std::shared_ptr<RosQtBridge> bridge_;
  KpiEngine kpi_engine_;
  StorageExporter exporter_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr kpi_timer_;
};

}  // namespace autodriver::tools
