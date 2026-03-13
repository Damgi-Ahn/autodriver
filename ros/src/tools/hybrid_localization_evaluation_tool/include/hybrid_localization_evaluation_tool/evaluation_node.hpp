#pragma once

#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"
#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
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
  void OnGnssPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void OnEskfOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnFgoPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnKpiTimer();

  DiagnosticParser parser_;
  std::shared_ptr<RosQtBridge> bridge_;
  KpiEngine kpi_engine_;
  StorageExporter exporter_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr eskf_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fgo_pose_sub_;
  rclcpp::TimerBase::SharedPtr kpi_timer_;
};

}  // namespace autodriver::tools
