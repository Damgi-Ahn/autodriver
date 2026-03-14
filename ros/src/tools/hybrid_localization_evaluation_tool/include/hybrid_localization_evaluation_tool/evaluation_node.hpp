#pragma once

#include "hybrid_localization_evaluation_tool/alert_engine.hpp"
#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"
#include "hybrid_localization_evaluation_tool/ground_truth_analyzer.hpp"
#include "hybrid_localization_evaluation_tool/health_state_engine.hpp"
#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"
#include "hybrid_localization_evaluation_tool/ros_qt_bridge.hpp"
#include "hybrid_localization_evaluation_tool/session_store.hpp"
#include "hybrid_localization_evaluation_tool/storage_exporter.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <string>

namespace autodriver::tools {

class EvaluationNode : public rclcpp::Node {
 public:
  explicit EvaluationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void SetBridge(const std::shared_ptr<RosQtBridge>& bridge);

  StorageExporter* GetExporter() { return &exporter_; }
  double nis_gate_pos()     const { return alert_engine_.thresholds().nis_gate_gnss_pos; }
  double nis_gate_vel()     const { return alert_engine_.thresholds().nis_gate_gnss_vel; }
  double nis_gate_heading() const { return alert_engine_.thresholds().nis_gate_heading; }

 private:
  // --- ROS callbacks ---
  void OnDiagnostics(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
  void OnOutputOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnGnssPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void OnEskfOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnFgoPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void OnGroundTruth(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnLocalizationState(const std_msgs::msg::String::SharedPtr msg);
  void OnKeyframePath(const nav_msgs::msg::Path::SharedPtr msg);

  // --- Timer ---
  void OnKpiTimer();

  // --- Members ---
  DiagnosticParser             parser_;
  std::shared_ptr<RosQtBridge> bridge_;
  KpiEngine                    kpi_engine_;
  AlertEngine                  alert_engine_;
  SessionStore                 session_store_;
  StorageExporter              exporter_;
  HealthStateEngine            health_engine_;
  GroundTruthAnalyzer          gt_analyzer_;

  // --- Subscriptions ---
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr        diag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       eskf_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr               fgo_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr                       gt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                         state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr                           keyframe_path_sub_;

  rclcpp::TimerBase::SharedPtr kpi_timer_;
};

}  // namespace autodriver::tools
