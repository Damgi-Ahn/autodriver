#include "hybrid_localization/hybrid_localization_node.hpp"

#include <algorithm>
#include <cctype>
#include <limits>

namespace hybrid_localization
{

// NOTE: 노드 파라미터 로딩의 단일 진입점. 실제 declare_parameter 로직은
// parameters.cpp에 모아두고 여기서는 적용/반영만 수행한다.
void HybridLocalizationNode::load_parameters()
{
  node_params_.load(*this);
  node_params_.heading_arbitrator.min_status_for_gphdt =
    node_params_.gnss.min_status_for_yaw_update;
  time_processor_ = TimeProcessor(node_params_.time);
  imu_preprocessor_.set_params(node_params_.imu);
  eskf_.set_params(node_params_.eskf);
  {
    std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
    heading_arbitrator_.set_params(node_params_.heading_arbitrator);
    heading_arbitrator_.reset();
  }
  last_yaw_meas_var_rad2_ = node_params_.heading.yaw_var;
  last_heading_status_inflate_dbg_ = 1.0;
  last_heading_recover_inflate_dbg_ = 1.0;
  last_heading_yaw_var_pre_nis_dbg_ = last_yaw_meas_var_rad2_;
  last_heading_yaw_var_applied_dbg_ = std::numeric_limits<double>::quiet_NaN();
  last_heading_yaw_var_source_dbg_ = "normal";

  // Init/activation mode parsing
  std::string mode = node_params_.init.mode;
  std::transform(
    mode.begin(), mode.end(), mode.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  use_external_initialpose_ = (mode == "external_initialpose");
  if (!(mode == "auto" || mode == "external_initialpose")) {
    RCLCPP_WARN(
      this->get_logger(),
      "Unknown init.mode='%s'. Falling back to 'auto'.",
      node_params_.init.mode.c_str());
    use_external_initialpose_ = false;
  }

  // If trigger is required, start deactivated until pose_initializer (or user) activates.
  is_activated_ = !node_params_.init.require_trigger;
}

} // namespace hybrid_localization
