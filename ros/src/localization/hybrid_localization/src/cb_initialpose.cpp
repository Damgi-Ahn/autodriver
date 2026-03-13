#include "hybrid_localization/hybrid_localization_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>

namespace hybrid_localization
{

namespace
{

inline bool is_valid_quat(const geometry_msgs::msg::Quaternion & q)
{
  const double n2 =
    q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::isfinite(n2) && (n2 > 1e-12);
}

} // namespace

void HybridLocalizationNode::initialpose_callback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr t_msg)
{
  if (!t_msg) {
    return;
  }

  // In AUTO mode, external initialpose is still useful for dev (RViz reset).
  // In EXTERNAL mode, it is the primary initialization path.
  const bool reset_always = node_params_.init.reset_on_external_pose;

  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    if (!reset_always && eskf_.initialized()) {
      return;
    }
  }

  const std::string target_frame = node_params_.io.map_frame;
  geometry_msgs::msg::PoseWithCovarianceStamped msg = *t_msg;

  // Transform to map frame if required and TF is available.
  const std::string src_frame = msg.header.frame_id;
  if (!src_frame.empty() && (src_frame != target_frame) && tf_buffer_) {
    try {
      // RViz sometimes publishes with a zero stamp. Use the latest available TF in that case.
      const bool zero_stamp = (msg.header.stamp.sec == 0) && (msg.header.stamp.nanosec == 0);
      const auto tf =
        zero_stamp ? tf_buffer_->lookupTransform(target_frame, src_frame, tf2::TimePointZero) :
        tf_buffer_->lookupTransform(
        target_frame, src_frame, msg.header.stamp,
        rclcpp::Duration::from_seconds(0.2));
      tf2::doTransform(msg, msg, tf);
      msg.header.frame_id = target_frame;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "initialpose: TF transform failed (target=%s, src=%s): %s",
        target_frame.c_str(), src_frame.c_str(), ex.what());
      return;
    }
  } else if (!src_frame.empty() && (src_frame != target_frame)) {
    RCLCPP_WARN(
      this->get_logger(),
      "initialpose: frame_id mismatch (target=%s, src=%s) and TF buffer unavailable",
      target_frame.c_str(), src_frame.c_str());
    return;
  }

  if (!is_valid_quat(msg.pose.pose.orientation)) {
    RCLCPP_WARN(this->get_logger(), "initialpose: invalid quaternion, ignoring");
    return;
  }

  const auto & p = msg.pose.pose.position;
  const auto & q = msg.pose.pose.orientation;
  const Eigen::Vector3d p_map(p.x, p.y, p.z);
  const Eigen::Quaterniond q_map_from_base(q.w, q.x, q.y, q.z);

  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    eskf_.initialize(p_map, q_map_from_base);
    state_stamp_ = rclcpp::Time(msg.header.stamp);
    eskf_init_stamp_ =
      (state_stamp_.seconds() > 0.0) ? state_stamp_ : this->now();

    // Clear any internal "pending init" state so auto-init won't immediately override.
    pending_init_position_ = false;
    pending_init_p_map_.setZero();
    pending_init_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    // Reset last update debug snapshots on hard re-init.
    last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
    last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
    last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
  }

  RCLCPP_INFO(
    this->get_logger(),
    "ESKF initialized from external initialpose (frame=%s, p=[%.3f, %.3f, %.3f])",
    msg.header.frame_id.c_str(), p_map.x(), p_map.y(), p_map.z());
}

void HybridLocalizationNode::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if (!req || !res) {
    return;
  }

  if (req->data) {
    reset_imu_dt_stats();
    is_activated_ = true;
    res->success = true;
    res->message = "activated";
  } else {
    is_activated_ = false;
    res->success = true;
    res->message = "deactivated";
  }
}

} // namespace hybrid_localization
