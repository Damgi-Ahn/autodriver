#include "hybrid_localization/hybrid_localization_node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
namespace hybrid_localization
{

namespace
{

// /sensing/gnss/heading convention:
// - reference: ENU +x (East) = 0 deg
// - direction: CW+ (non-standard)
// Convert to standard ENU yaw (CCW+ from East).
inline double heading_deg_to_enu_yaw_rad(const double heading_deg)
{
  constexpr double DEG_TO_RAD = M_PI / 180.0;
  return -(heading_deg * DEG_TO_RAD);
}

inline double sanitize_inflate(const double inflate)
{
  if (!std::isfinite(inflate) || !(inflate > 0.0)) {
    return 1.0;
  }
  return inflate;
}

inline double position_status_inflate(
  const GnssParams & gnss,
  const int status)
{
  if (status == 0) {
    return sanitize_inflate(gnss.pos_inflate_status_fix);
  }
  if (status == 1) {
    return sanitize_inflate(gnss.pos_inflate_status_sbas);
  }
  return 1.0;
}

inline double velocity_status_inflate(
  const GnssParams & gnss,
  const int status)
{
  if (status == 0) {
    return sanitize_inflate(gnss.vel_inflate_status_fix);
  }
  if (status == 1) {
    return sanitize_inflate(gnss.vel_inflate_status_sbas);
  }
  return 1.0;
}

inline double heading_status_inflate(
  const GnssParams & gnss,
  const int status)
{
  if (status < 0) {
    constexpr double kNegStatusInflate = 64.0;
    return kNegStatusInflate;
  }
  if (status == 0) {
    return sanitize_inflate(gnss.vel_inflate_status_fix);
  }
  if (status == 1) {
    return sanitize_inflate(gnss.vel_inflate_status_sbas);
  }
  return 1.0;
}

inline double recover_peak_from_transition(
  const double prev_status_inflate,
  const double curr_status_inflate,
  const double fallback_peak)
{
  const double prev = sanitize_inflate(prev_status_inflate);
  const double curr = sanitize_inflate(curr_status_inflate);
  if (curr > 0.0) {
    const double ratio = prev / curr;
    if (std::isfinite(ratio) && ratio > 1.0) {
      return ratio;
    }
  }
  const double fallback = sanitize_inflate(fallback_peak);
  return (fallback > 1.0) ? fallback : 1.0;
}

double heading_event_peak_inflate(const std::string & reason)
{
  if (reason.empty()) {
    return 1.0;
  }
  if (reason.find("disabled") != std::string::npos ||
    reason.find("not_initialized") != std::string::npos ||
    reason.find("external_initialpose_mode") != std::string::npos ||
    reason.find("time_alignment") != std::string::npos ||
    reason.find("not_activated") != std::string::npos)
  {
    return 1.0;
  }
  if (reason.find("gnss_status_neg1") != std::string::npos ||
    reason.find("heading_non_finite") != std::string::npos ||
    reason.find("non_finite") != std::string::npos)
  {
    return 64.0;
  }
  if (reason.find("heading_timeout") != std::string::npos ||
    reason.find("gnss_status_skip") != std::string::npos ||
    reason.find("recover_holdoff") != std::string::npos)
  {
    return 16.0;
  }
  if (reason.find("rate_gate_skip") != std::string::npos) {
    return 8.0;
  }
  return 4.0;
}

} // namespace

void HybridLocalizationNode::update_gnss_recover_state(
  const int curr_status,
  const rclcpp::Time & stamp)
{
  const auto & recover = node_params_.gnss.recover;

  std::scoped_lock<std::mutex> lock(gnss_recover_mutex_);
  const int prev = prev_gnss_status_;
  prev_gnss_status_ = curr_status;

  if (!recover.enable) {
    gnss_recover_active_ = false;
    gnss_recover_holdoff_until_ = stamp;
    gnss_recover_pos_last_stamp_ = stamp;
    gnss_recover_vel_last_stamp_ = stamp;
    gnss_recover_pos_inflate_ = 1.0;
    gnss_recover_vel_inflate_ = 1.0;
    return;
  }

  if (prev == std::numeric_limits<int>::min()) {
    gnss_recover_pos_last_stamp_ = stamp;
    gnss_recover_vel_last_stamp_ = stamp;
    return;
  }

  // Trigger on any status upgrade to >=1:
  // -1->1/2, 0->1/2, 1->2
  if (!(curr_status > prev && curr_status >= 1)) {
    return;
  }

  const double holdoff_sec = std::max(0.0, recover.holdoff_sec);
  const double ramp_sec = std::max(0.0, recover.ramp_sec);
  gnss_recover_active_ = (holdoff_sec > 0.0) || (ramp_sec > 0.0);
  gnss_recover_holdoff_until_ =
    stamp + rclcpp::Duration::from_seconds(holdoff_sec);
  gnss_recover_pos_last_stamp_ = stamp;
  gnss_recover_vel_last_stamp_ = stamp;

  const double pos_peak = recover_peak_from_transition(
    position_status_inflate(node_params_.gnss, prev),
    position_status_inflate(node_params_.gnss, curr_status),
    recover.pos_max_inflate);
  const double vel_peak = recover_peak_from_transition(
    velocity_status_inflate(node_params_.gnss, prev),
    velocity_status_inflate(node_params_.gnss, curr_status),
    recover.vel_max_inflate);
  gnss_recover_pos_inflate_ = std::max(gnss_recover_pos_inflate_, pos_peak);
  gnss_recover_vel_inflate_ = std::max(gnss_recover_vel_inflate_, vel_peak);
}

bool HybridLocalizationNode::compute_gnss_recover_inflate(
  const GnssRecoverChannel channel,
  const rclcpp::Time & stamp,
  bool & skip_update,
  double & out_inflate)
{
  skip_update = false;
  out_inflate = 1.0;

  const auto & recover = node_params_.gnss.recover;
  if (!recover.enable) {
    return false;
  }

  std::scoped_lock<std::mutex> lock(gnss_recover_mutex_);
  if (!gnss_recover_active_) {
    return false;
  }

  double & inflate_state = (channel == GnssRecoverChannel::kPosition) ?
    gnss_recover_pos_inflate_ :
    gnss_recover_vel_inflate_;
  rclcpp::Time & last_stamp = (channel == GnssRecoverChannel::kPosition) ?
    gnss_recover_pos_last_stamp_ :
    gnss_recover_vel_last_stamp_;
  inflate_state = sanitize_inflate(inflate_state);
  if (inflate_state < 1.0) {
    inflate_state = 1.0;
  }

  // Holdoff window: skip updates entirely.
  if (stamp < gnss_recover_holdoff_until_) {
    skip_update = true;
    out_inflate = inflate_state;
    return true;
  }

  double dt_sec = 0.0;
  if (last_stamp.nanoseconds() > 0 && stamp > last_stamp) {
    dt_sec = (stamp - last_stamp).seconds();
  }
  last_stamp = stamp;

  // Asymmetric recover:
  // - higher input is applied immediately by state update
  // - lower input decays naturally with exponential tail
  const double ramp_sec = std::max(0.0, recover.ramp_sec);
  if (ramp_sec > 0.0 && dt_sec > 0.0 && inflate_state > 1.0) {
    constexpr double kDecayAtRampSec = 5.0;  // exp(-5) ~= 0.0067
    const double tau_sec = ramp_sec / kDecayAtRampSec;
    if (tau_sec > 0.0) {
      inflate_state = 1.0 + (inflate_state - 1.0) * std::exp(-dt_sec / tau_sec);
    }
  } else if (!(ramp_sec > 0.0)) {
    inflate_state = 1.0;
  }

  out_inflate = inflate_state;
  if (!std::isfinite(out_inflate) || !(out_inflate >= 1.0)) {
    inflate_state = 1.0;
    out_inflate = 1.0;
  }

  if (stamp >= gnss_recover_holdoff_until_) {
    constexpr double kDeactivateEps = 1e-3;
    if (gnss_recover_pos_inflate_ <= (1.0 + kDeactivateEps) &&
      gnss_recover_vel_inflate_ <= (1.0 + kDeactivateEps))
    {
      gnss_recover_active_ = false;
      gnss_recover_pos_inflate_ = 1.0;
      gnss_recover_vel_inflate_ = 1.0;
    }
  }
  return true;
}

double HybridLocalizationNode::apply_gnss_inflate_envelope(
  const GnssRecoverChannel channel,
  const rclcpp::Time & stamp,
  const double target_inflate)
{
  const double target = std::max(1.0, sanitize_inflate(target_inflate));

  std::scoped_lock<std::mutex> lock(gnss_recover_mutex_);
  double & inflate_state = (channel == GnssRecoverChannel::kPosition) ?
    gnss_env_pos_inflate_ :
    gnss_env_vel_inflate_;
  rclcpp::Time & last_stamp = (channel == GnssRecoverChannel::kPosition) ?
    gnss_env_pos_last_stamp_ :
    gnss_env_vel_last_stamp_;

  inflate_state = std::max(1.0, sanitize_inflate(inflate_state));

  double dt_sec = 0.0;
  if (last_stamp.nanoseconds() > 0 && stamp > last_stamp) {
    dt_sec = (stamp - last_stamp).seconds();
  }
  last_stamp = stamp;

  if (target > inflate_state) {
    inflate_state = target;
    return inflate_state;
  }

  const double ramp_sec = std::max(0.0, node_params_.gnss.recover.ramp_sec);
  if (!(ramp_sec > 0.0)) {
    inflate_state = target;
    return inflate_state;
  }
  if (!(dt_sec > 0.0)) {
    return inflate_state;
  }

  constexpr double kDecayAtRampSec = 5.0;  // exp(-5) ~= 0.0067
  const double tau_sec = ramp_sec / kDecayAtRampSec;
  if (!(tau_sec > 0.0)) {
    inflate_state = target;
    return inflate_state;
  }

  inflate_state =
    target + (inflate_state - target) * std::exp(-dt_sec / tau_sec);
  if (!std::isfinite(inflate_state) || !(inflate_state >= 1.0)) {
    inflate_state = 1.0;
  }
  return inflate_state;
}

void HybridLocalizationNode::update_heading_recover_state(
  const int curr_status,
  const rclcpp::Time & stamp)
{
  std::scoped_lock<std::mutex> lock(heading_recover_mutex_);
  const int prev = prev_heading_status_;
  prev_heading_status_ = curr_status;

  const double curr_status_inflate =
    heading_status_inflate(node_params_.gnss, curr_status);
  heading_status_inflate_ = std::max(1.0, sanitize_inflate(curr_status_inflate));

  if (prev == std::numeric_limits<int>::min()) {
    heading_recover_last_stamp_ = stamp;
    return;
  }

  const double prev_status_inflate =
    heading_status_inflate(node_params_.gnss, prev);

  const int min_status =
    std::max(0, std::min(2, node_params_.gnss.min_status_for_yaw_update));
  const bool prev_good = (prev >= min_status);
  const bool now_good = (curr_status >= min_status);
  if (prev_good || !now_good || !(curr_status > prev)) {
    return;
  }

  // Keep high uncertainty while heading source is in post-recovery holdoff.
  const double holdoff_sec =
    std::max(0.0, node_params_.heading_arbitrator.gphdt_recover_holdoff_sec);
  heading_recover_holdoff_until_ =
    stamp + rclcpp::Duration::from_seconds(holdoff_sec);
  heading_recover_last_stamp_ = stamp;

  const double peak = recover_peak_from_transition(
    prev_status_inflate,
    curr_status_inflate,
    std::max(4.0, prev_status_inflate));
  heading_recover_inflate_ = std::max(heading_recover_inflate_, peak);
}

void HybridLocalizationNode::raise_heading_recover_inflate(
  const std::string & reason,
  const rclcpp::Time & stamp)
{
  const double peak = heading_event_peak_inflate(reason);
  if (!(peak > 1.0)) {
    return;
  }
  std::scoped_lock<std::mutex> lock(heading_recover_mutex_);
  heading_recover_inflate_ = std::max(heading_recover_inflate_, peak);
  if (heading_recover_last_stamp_.nanoseconds() <= 0) {
    heading_recover_last_stamp_ = stamp;
  }
}

double HybridLocalizationNode::compute_heading_measurement_inflate(
  const rclcpp::Time & stamp,
  double * status_inflate,
  double * recover_inflate)
{
  std::scoped_lock<std::mutex> lock(heading_recover_mutex_);
  heading_status_inflate_ = std::max(1.0, sanitize_inflate(heading_status_inflate_));
  heading_recover_inflate_ = std::max(1.0, sanitize_inflate(heading_recover_inflate_));

  double dt_sec = 0.0;
  if (heading_recover_last_stamp_.nanoseconds() > 0 && stamp > heading_recover_last_stamp_) {
    dt_sec = (stamp - heading_recover_last_stamp_).seconds();
  }
  heading_recover_last_stamp_ = stamp;

  if (stamp >= heading_recover_holdoff_until_) {
    const double ramp_sec = std::max(0.0, node_params_.gnss.recover.ramp_sec);
    if (ramp_sec > 0.0 && dt_sec > 0.0 && heading_recover_inflate_ > 1.0) {
      constexpr double kDecayAtRampSec = 5.0;  // exp(-5) ~= 0.0067
      const double tau_sec = ramp_sec / kDecayAtRampSec;
      if (tau_sec > 0.0) {
        heading_recover_inflate_ =
          1.0 + (heading_recover_inflate_ - 1.0) * std::exp(-dt_sec / tau_sec);
      }
    } else if (!(ramp_sec > 0.0)) {
      heading_recover_inflate_ = 1.0;
    }
  }

  if (!std::isfinite(heading_recover_inflate_) || !(heading_recover_inflate_ >= 1.0)) {
    heading_recover_inflate_ = 1.0;
  }

  if (status_inflate != nullptr) {
    *status_inflate = heading_status_inflate_;
  }
  if (recover_inflate != nullptr) {
    *recover_inflate = heading_recover_inflate_;
  }
  return heading_status_inflate_ * heading_recover_inflate_;
}

void HybridLocalizationNode::gnss_callback(
  const sensor_msgs::msg::NavSatFix::SharedPtr t_msg)
{
  const bool activated = (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(current_stamp, now, last_gnss_stamp_, "GNSS")) {
    return;
  }
  // GNSS 외란 보정을 위한 base↔gnss extrinsic 확보
  if (!tf_cache_.extrinsics().gnss_valid && !t_msg->header.frame_id.empty()) {
    tf_cache_.cache_gnss_from_frame_id(t_msg->header.frame_id, this->get_logger());
  }
  const auto & io = node_params_.io;
  const auto & gnss_params = node_params_.gnss;
  const bool is_first_gnss = (gnss_count_ == 0);
  latest_gnss_ = t_msg;
  last_gnss_stamp_ = current_stamp;
  gnss_count_++;
  const int gnss_status = static_cast<int>(t_msg->status.status);
  update_gnss_recover_state(gnss_status, current_stamp);
  update_heading_recover_state(gnss_status, current_stamp);
  {
    std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
    heading_arbitrator_.update_gnss_status(gnss_status, current_stamp.seconds());
  }
  // GNSS status 기반 하드 게이팅
  if (gnss_status < gnss_params.min_status_for_pos_update) {
    last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
    last_gnss_pos_update_dbg_.reason = "gnss_status_skip";
    last_gnss_pos_recover_inflate_dbg_ = 1.0;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 2000, "GNSS: status=%d < min_status_for_pos_update=%d. Skipping init/update.", gnss_status,
      gnss_params.min_status_for_pos_update);
    return;
  }

  // NOTE: Even when deactivated, keep publishing GNSS debug outputs.
  // Filter state init/update is gated later by `activated`.
  // 위경도 → map 좌표 투영 (맵 정보가 준비된 경우)
  if (map_projector_.valid()) {
    double yaw_for_gnss_lever_arm = std::numeric_limits<double>::quiet_NaN();
    bool eskf_initialized = false;
    {
      std::scoped_lock<std::mutex> lock(state_mutex_);
      if (eskf_.initialized()) {
        eskf_initialized = true;
        const Eigen::Matrix3d R = eskf_.q_map_from_base().toRotationMatrix();
        yaw_for_gnss_lever_arm = std::atan2(R(1, 0), R(0, 0));
      }
    }
    if (!eskf_initialized) {
      std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
      const auto yaw_sel = heading_arbitrator_.select(current_stamp.seconds());
      if (yaw_sel.source != GnssHeadingSource::kNone &&
        std::isfinite(yaw_sel.yaw_rad))
      {
        yaw_for_gnss_lever_arm = yaw_sel.yaw_rad;
      }
    }
    if (!std::isfinite(yaw_for_gnss_lever_arm)) {
      yaw_for_gnss_lever_arm = 0.0;
    }
    GnssPreprocessResult gnss_result;
    const auto status = gnss_preprocessor_.preprocess(
      *t_msg, map_projector_, tf_cache_.extrinsics(),
      yaw_for_gnss_lever_arm, gnss_result);
    if (status == GnssPreprocessStatus::kNoFix) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "GNSS: status=%d (no fix). Skipping projection/debug TF.",
        static_cast<int>(t_msg->status.status));
    } else if (status == GnssPreprocessStatus::kInvalidLatLon) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "GNSS: invalid lat/lon (lat=%.8f, lon=%.8f). " "Skipping projection/debug TF.", t_msg->latitude,
        t_msg->longitude);
    } else if (status == GnssPreprocessStatus::kProjectionFailed) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "GNSS: projection failed. Skipping debug TF. Error: %s",
        gnss_result.projection_error.c_str());
    } else if (status == GnssPreprocessStatus::kNonFiniteProjection) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "GNSS: projection produced non-finite values (x=%.3f, y=%.3f, " "z=%.3f). Skipping debug TF.", gnss_result.antenna_position_map.x, gnss_result.antenna_position_map.y,
        gnss_result.antenna_position_map.z);
    } else if (status == GnssPreprocessStatus::kOk) {
      if (gnss_result.altitude_sanitized) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(), 2000, "GNSS: invalid altitude (alt=%.3f). Using map_origin altitude for " "projection.",
          t_msg->altitude);
      }

      if (is_first_gnss) {
        RCLCPP_INFO(
          this->get_logger(), "First GNSS projection: antenna=(%.3f, %.3f, %.3f) " "base=(%.3f, %.3f, %.3f)", gnss_result.antenna_position_map.x, gnss_result.antenna_position_map.y, gnss_result.antenna_position_map.z, gnss_result.base_position_map.x, gnss_result.base_position_map.y,
          gnss_result.base_position_map.z);
      }

      {
        nav_msgs::msg::Odometry gnss_odom;
        gnss_odom.header.stamp = t_msg->header.stamp;
        gnss_odom.header.frame_id = io.map_frame;
        gnss_odom.child_frame_id = io.map_frame;

        gnss_odom.pose.pose.position.x = gnss_result.base_position_map.x;
        gnss_odom.pose.pose.position.y = gnss_result.base_position_map.y;
        gnss_odom.pose.pose.position.z = gnss_result.base_position_map.z;

        // GNSS-only 시각화를 위한 yaw 적용 (가능한 경우)
        if (heading_received_ && std::isfinite(latest_heading_yaw_rad_)) {
          const Eigen::AngleAxisd yaw_aa(latest_heading_yaw_rad_,
            Eigen::Vector3d::UnitZ());
          const Eigen::Quaterniond q(yaw_aa);
          gnss_odom.pose.pose.orientation.x = q.x();
          gnss_odom.pose.pose.orientation.y = q.y();
          gnss_odom.pose.pose.orientation.z = q.z();
          gnss_odom.pose.pose.orientation.w = q.w();
        } else {
          gnss_odom.pose.pose.orientation.w = 1.0;
        }
        if (gnss_params.use_navsatfix_covariance &&
          t_msg->position_covariance_type !=
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
        {
          gnss_odom.pose.covariance[0] = t_msg->position_covariance[0];
          gnss_odom.pose.covariance[7] = t_msg->position_covariance[4];
          gnss_odom.pose.covariance[14] = t_msg->position_covariance[8];
        } else {
          gnss_odom.pose.covariance[0] = gnss_params.pos_var_fallback;
          gnss_odom.pose.covariance[7] = gnss_params.pos_var_fallback;
          gnss_odom.pose.covariance[14] = gnss_params.pos_var_fallback;
        }
        gnss_odom_pub_->publish(gnss_odom);
      }

      // Optional: publish GNSS-derived pose for pose_initializer (Case B).
      if (gnss_params.publish_pose_with_covariance && gnss_pose_cov_pub_) {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
        pose_cov.header.stamp = t_msg->header.stamp;
        pose_cov.header.frame_id = io.map_frame;

        const auto & p = gnss_params.pose_with_covariance_use_base_position ?
          gnss_result.base_position_map :
          gnss_result.antenna_position_map;
        pose_cov.pose.pose.position.x = p.x;
        pose_cov.pose.pose.position.y = p.y;
        pose_cov.pose.pose.position.z = p.z;

        // If heading is available, add yaw for better initial guess. Otherwise, identity.
        if (heading_received_ && std::isfinite(latest_heading_yaw_rad_)) {
          const Eigen::AngleAxisd yaw_aa(latest_heading_yaw_rad_,
            Eigen::Vector3d::UnitZ());
          const Eigen::Quaterniond q(yaw_aa);
          pose_cov.pose.pose.orientation.x = q.x();
          pose_cov.pose.pose.orientation.y = q.y();
          pose_cov.pose.pose.orientation.z = q.z();
          pose_cov.pose.pose.orientation.w = q.w();
        } else {
          pose_cov.pose.pose.orientation.w = 1.0;
        }

        // Covariance: use NavSatFix diag if available; other components left as 0.
        // pose_initializer typically overwrites covariance with its own parameters anyway.
        pose_cov.pose.covariance.fill(0.0);
        const double var_fallback = gnss_params.pos_var_fallback;
        if (gnss_params.use_navsatfix_covariance &&
          t_msg->position_covariance_type !=
          sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
        {
          pose_cov.pose.covariance[0] = t_msg->position_covariance[0];
          pose_cov.pose.covariance[7] = t_msg->position_covariance[4];
          pose_cov.pose.covariance[14] = t_msg->position_covariance[8];
        } else {
          pose_cov.pose.covariance[0] = var_fallback;
          pose_cov.pose.covariance[7] = var_fallback;
          pose_cov.pose.covariance[14] = var_fallback;
        }
        // Set a large yaw variance if we don't have a heading source.
        pose_cov.pose.covariance[35] =
          (heading_received_ && std::isfinite(latest_heading_yaw_rad_)) ?
          node_params_.heading.yaw_var :
          1.0e6;

        gnss_pose_cov_pub_->publish(pose_cov);
      }

      GnssYawMeasurement yaw_sel;
      {
        std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
        yaw_sel = heading_arbitrator_.select(current_stamp.seconds());
      }

      {
        std::scoped_lock<std::mutex> lock(state_mutex_);

        if (!activated) {
          last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
          last_gnss_pos_update_dbg_.reason = "not_activated";
          return;
        }

        const Eigen::Vector3d z_p_map(gnss_result.base_position_map.x,
          gnss_result.base_position_map.y,
          gnss_result.base_position_map.z);

        // 초기화: GNSS 위치를 캐시하고 신뢰 가능한 yaw 확보 후 시작
        if (!eskf_.initialized()) {
          if (use_external_initialpose_) {
            last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
            last_gnss_pos_update_dbg_.reason = "external_initialpose_mode";
            return;
          }

          pending_init_position_ = true;
          pending_init_p_map_ = z_p_map;
          pending_init_stamp_ = current_stamp;

          double yaw_init = std::numeric_limits<double>::quiet_NaN();
          const char * yaw_source = "Unknown";
          if (yaw_sel.source == GnssHeadingSource::kGphdt) {
            yaw_init = yaw_sel.yaw_rad;
            yaw_source = "Heading";
          }

          if (!std::isfinite(yaw_init)) {
            last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
            last_gnss_pos_update_dbg_.reason = "wait_yaw_for_init";
          } else {
            const Eigen::AngleAxisd yaw_aa(yaw_init, Eigen::Vector3d::UnitZ());
            const Eigen::Quaterniond q_init(yaw_aa);
            eskf_.initialize(z_p_map, q_init);
            eskf_init_stamp_ = current_stamp;
            state_stamp_ = std::max(current_stamp, state_stamp_);
            pending_init_position_ = false;
            RCLCPP_INFO(
              this->get_logger(), "ESKF initialized from GNSS+%s (p=[%.3f, %.3f, %.3f], " "yaw=%.3f rad)", yaw_source,
              z_p_map.x(), z_p_map.y(), z_p_map.z(), yaw_init);
          }
        } else if (gnss_params.enable_gnss_pos_update) {
          // GNSS 위치 업데이트 경로
          const auto tol = rclcpp::Duration::from_seconds(
            std::max(0.0, node_params_.time_alignment_tolerance_sec));
          if (state_stamp_.seconds() > 0.0 &&
            (current_stamp + tol) < state_stamp_)
          {
            last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
            last_gnss_pos_update_dbg_.reason = "time_alignment";
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000, "GNSS: stamp is older than ESKF state (gnss=%.6f, state=%.6f). " "Skipping position update (no time alignment, tol=%.3fms).",
              current_stamp.seconds(), state_stamp_.seconds(),
              std::max(0.0, node_params_.time_alignment_tolerance_sec) * 1000.0);
          } else {
            double status_inflate = 1.0;
            if (gnss_status == 0) {
              status_inflate = gnss_params.pos_inflate_status_fix;
            } else if (gnss_status == 1) {
              status_inflate = gnss_params.pos_inflate_status_sbas;
            } else {
              status_inflate = 1.0;
            }
            if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
              status_inflate = 1.0;
            }

            bool recover_skip = false;
            double recover_inflate = 1.0;
            if (gnss_status >= 1) {
              (void)compute_gnss_recover_inflate(
                GnssRecoverChannel::kPosition,
                current_stamp,
                recover_skip, recover_inflate);
            }
            const double target_inflate = status_inflate * recover_inflate;
            const double effective_inflate = apply_gnss_inflate_envelope(
              GnssRecoverChannel::kPosition, current_stamp, target_inflate);
            last_gnss_pos_recover_inflate_dbg_ = effective_inflate;
            if (recover_skip) {
              last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
              last_gnss_pos_update_dbg_.reason = "recover_holdoff";
              return;
            }

            GnssPosUpdateConfig config;
            config.use_navsatfix_covariance =
              gnss_params.use_navsatfix_covariance;
            config.covariance_scale = gnss_params.cov_scale * effective_inflate;
            config.pos_var_fallback = gnss_params.pos_var_fallback;
            config.pos_var_min = gnss_params.pos_var_min;
            config.pos_var_max = gnss_params.pos_var_max;
            config.cov_diag_only = gnss_params.pos_cov_diag_only;

            last_gnss_pos_update_dbg_ =
              gnss_update_handler_.apply_position_update(
              eskf_, z_p_map,
              *t_msg, config);
            if (current_stamp > state_stamp_) {
              state_stamp_ = current_stamp;
            }

            // FGO Stage 3: 위치 측정값 캐시 (키프레임 생성 시 소비됨)
            {
              GnssMeasurement fgo_gnss;
              fgo_gnss.stamp_sec = current_stamp.seconds();
              fgo_gnss.pos_map = z_p_map;
              fgo_gnss.status = gnss_status;

              // NavSatFix 공분산 추출 (행 우선, ENU x,y,z 순서)
              const auto & cov_arr = t_msg->position_covariance;
              if (t_msg->position_covariance_type !=
                sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
              {
                Eigen::Matrix3d pos_cov;
                pos_cov << cov_arr[0], cov_arr[1], cov_arr[2],
                  cov_arr[3], cov_arr[4], cov_arr[5],
                  cov_arr[6], cov_arr[7], cov_arr[8];
                // covariance_scale 적용 (ESKF 와 동일한 스케일링)
                pos_cov *= gnss_params.cov_scale;
                fgo_gnss.pos_cov = pos_cov;
              }

              // 이미 heading이 있으면 추가
              if (heading_received_) {
                fgo_gnss.has_heading = true;
                fgo_gnss.heading_rad = latest_heading_yaw_rad_;
                fgo_gnss.heading_var = node_params_.heading.yaw_var;
              }

              latest_fgo_gnss_ = fgo_gnss;
            }
          }
        }
      }
    }
  }
}

void HybridLocalizationNode::gnss_vel_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr t_msg)
{
  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(current_stamp, now, last_gnss_vel_stamp_, "GNSS_VEL")) {
    return;
  }

  // GNSS 속도(ENU) → 차량 속도 메시지 변환
  // - NMEA navsat driver publishes TwistStamped:
  //   linear.x = ve (east), linear.y = vn (north). vertical velocity is not provided.
  const double ve = static_cast<double>(t_msg->twist.linear.x);
  const double vn = static_cast<double>(t_msg->twist.linear.y);
  static constexpr double kVuAssumed = 0.0;
  const double speed_ms = std::sqrt(vn * vn + ve * ve + kVuAssumed * kVuAssumed);

  auto velocity_msg = autoware_vehicle_msgs::msg::VelocityReport();
  velocity_msg.header = t_msg->header;
  velocity_msg.longitudinal_velocity = static_cast<float>(speed_ms);
  velocity_msg.lateral_velocity = 0.0f;
  velocity_msg.heading_rate = 0.0f;

  gnss_velocity_pub_->publish(velocity_msg);

  // GNSS 속도 업데이트 (vn/ve/vu 사용)
  if (node_params_.gnss.enable_gnss_vel_update) {
    const auto latest_gnss = latest_gnss_;
    if (!latest_gnss) {
      last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
      last_gnss_vel_update_dbg_.reason = "gnss_status_skip";
      last_gnss_vel_recover_inflate_dbg_ = 1.0;
    } else {
      const int gnss_status = static_cast<int>(latest_gnss->status.status);
      if (gnss_status < 0) {
        last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
        last_gnss_vel_update_dbg_.reason = "gnss_status_skip";
        last_gnss_vel_recover_inflate_dbg_ = 1.0;
      } else {
        double status_inflate = 1.0;
        if (gnss_status == 0) {
          status_inflate = node_params_.gnss.vel_inflate_status_fix;
        } else if (gnss_status == 1) {
          status_inflate = node_params_.gnss.vel_inflate_status_sbas;
        } else {
          status_inflate = 1.0;
        }
        if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
          status_inflate = 1.0;
        }

        std::scoped_lock<std::mutex> lock(state_mutex_);

        if (eskf_.initialized()) {
          const double target_inflate = status_inflate;
          bool recover_skip = false;
          double recover_inflate = 1.0;
          if (gnss_status >= 1) {
            (void)compute_gnss_recover_inflate(
              GnssRecoverChannel::kVelocity,
              current_stamp,
              recover_skip, recover_inflate);
          }
          const double effective_inflate = apply_gnss_inflate_envelope(
            GnssRecoverChannel::kVelocity, current_stamp,
            target_inflate * recover_inflate);
          last_gnss_vel_recover_inflate_dbg_ = effective_inflate;
          if (recover_skip) {
            last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
            last_gnss_vel_update_dbg_.reason = "recover_holdoff";
            // Skip velocity update during recovery holdoff window.
          } else {
          const auto tol = rclcpp::Duration::from_seconds(
            std::max(0.0, node_params_.time_alignment_tolerance_sec));
          if (state_stamp_.seconds() > 0.0 &&
            (current_stamp + tol) < state_stamp_)
          {
            last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
            last_gnss_vel_update_dbg_.reason = "time_alignment";
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000, "GNSS_VEL: stamp is older than ESKF state (vel=%.6f, state=%.6f). " "Skipping velocity update (no time alignment, tol=%.3fms).",
              current_stamp.seconds(), state_stamp_.seconds(),
              std::max(0.0, node_params_.time_alignment_tolerance_sec) * 1000.0);
          } else if (!std::isfinite(vn) || !std::isfinite(ve))
          {
            last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
            last_gnss_vel_update_dbg_.reason = "non_finite_measurement";
            RCLCPP_WARN_THROTTLE(
              this->get_logger(),
              *this->get_clock(), 2000, "GNSS_VEL: non-finite velocity (vn/ve), skipping");
          } else {
            Eigen::Vector3d z_v_map(ve, vn, kVuAssumed);

            if (tf_cache_.extrinsics().gnss_valid && have_last_omega_base_) {
              const Eigen::Vector3d omega_base(last_omega_base_.x,
                last_omega_base_.y,
                last_omega_base_.z);
              const auto & t =
                tf_cache_.extrinsics().base_to_gnss.transform.translation;
              const Eigen::Vector3d r_base(t.x, t.y, t.z);
              const Eigen::Matrix3d R =
                eskf_.q_map_from_base().toRotationMatrix();
              z_v_map -= R * (omega_base.cross(r_base));
            }

            GnssVelUpdateConfig config;
            config.covariance_scale =
              node_params_.gnss.vel_cov_scale * effective_inflate;
            config.vel_var_fallback = node_params_.gnss.vel_var_fallback;
            config.vel_var_min = node_params_.gnss.vel_var_min;
            config.vel_var_max = node_params_.gnss.vel_var_max;

            last_gnss_vel_update_dbg_ =
              gnss_update_handler_.apply_velocity_update(
              eskf_, z_v_map,
              config);

            if (current_stamp > state_stamp_) {
              state_stamp_ = current_stamp;
            }

            // FGO Stage 3: 속도 측정값을 캐시된 GNSS 측정값에 추가
            if (latest_fgo_gnss_.has_value()) {
              const double vel_var =
                node_params_.gnss.vel_var_fallback * effective_inflate;
              latest_fgo_gnss_->has_velocity = true;
              latest_fgo_gnss_->vel_map = z_v_map;
              latest_fgo_gnss_->vel_cov =
                Eigen::Matrix3d::Identity() * vel_var;
            }
          }
          }
        }
      }
    }
  }

  latest_gnss_vel_ = t_msg;
  last_gnss_vel_stamp_ = current_stamp;
  gnss_vel_count_++;
}

void HybridLocalizationNode::map_projector_info_callback(
  const tier4_map_msgs::msg::MapProjectorInfo::SharedPtr t_msg)
{
  // map projector 초기화 (GNSS 투영에 필수)
  map_projector_.set_info(*t_msg);

  if (!map_projector_received_) {
    const auto & origin = map_projector_.origin();
    RCLCPP_INFO(
      this->get_logger(), "Map projector info received: type=%s, lat0=%.8f deg (%.8f " "rad), lon0=%.8f deg (%.8f rad), alt0=%.3f m",
      t_msg->projector_type.c_str(), t_msg->map_origin.latitude, origin.lat0_rad, t_msg->map_origin.longitude, origin.lon0_rad,
      origin.alt0_m);
    map_projector_received_ = true;
  }
}

void HybridLocalizationNode::heading_callback(
  const skyautonet_msgs::msg::Gphdt::SharedPtr t_msg)
{
  const bool activated = (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(
      current_stamp, now, last_heading_stamp_,
      "Heading"))
  {
    return;
  }

  // heading(비표준) → ENU yaw 변환
  const double yaw_meas = heading_deg_to_enu_yaw_rad(t_msg->heading);

  const auto & heading_params = node_params_.heading;
  auto source_from_inflate =
    [](const double status_inflate, const double recover_inflate) -> std::string {
      if (recover_inflate > 1.0 + 1e-9) {
        return "decay";
      }
      if (status_inflate > 1.0 + 1e-9) {
        return "inflated_status";
      }
      return "normal";
    };
  auto refresh_heading_var_snapshot =
    [&](const std::string & source) -> double {
      double status_inflate = 1.0;
      double recover_inflate = 1.0;
      const double total_inflate = compute_heading_measurement_inflate(
        current_stamp, &status_inflate, &recover_inflate);
      double yaw_var = heading_params.yaw_var * std::max(1.0, sanitize_inflate(total_inflate));
      if (!std::isfinite(yaw_var) || !(yaw_var > 0.0)) {
        yaw_var = heading_params.yaw_var;
      }
      if (!std::isfinite(yaw_var) || !(yaw_var > 0.0)) {
        yaw_var = 0.01;
      }

      last_yaw_meas_var_rad2_ = yaw_var;
      last_heading_status_inflate_dbg_ = status_inflate;
      last_heading_recover_inflate_dbg_ = recover_inflate;
      last_heading_yaw_var_pre_nis_dbg_ = yaw_var;
      last_heading_yaw_var_source_dbg_ = source;
      return yaw_var;
    };

  bool bypassed_rate_gate = false;
  double status_inflate = 1.0;
  double recover_inflate = 1.0;
  const double total_inflate = compute_heading_measurement_inflate(
    current_stamp, &status_inflate, &recover_inflate);
  double yaw_var_for_update =
    heading_params.yaw_var * std::max(1.0, sanitize_inflate(total_inflate));
  if (!std::isfinite(yaw_var_for_update) || !(yaw_var_for_update > 0.0)) {
    yaw_var_for_update = heading_params.yaw_var;
  }
  if (!std::isfinite(yaw_var_for_update) || !(yaw_var_for_update > 0.0)) {
    yaw_var_for_update = 0.01;
  }
  last_yaw_meas_var_rad2_ = yaw_var_for_update;
  last_heading_status_inflate_dbg_ = status_inflate;
  last_heading_recover_inflate_dbg_ = recover_inflate;
  last_heading_yaw_var_pre_nis_dbg_ = yaw_var_for_update;
  last_heading_yaw_var_source_dbg_ =
    source_from_inflate(status_inflate, recover_inflate);
  last_heading_yaw_var_applied_dbg_ = std::numeric_limits<double>::quiet_NaN();

  bool apply_rate_gate = false;
  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    if (eskf_.initialized()) {
      const double since_init_sec = (current_stamp - eskf_init_stamp_).seconds();
      apply_rate_gate =
        std::isfinite(since_init_sec) &&
        (since_init_sec >= k_heading_rate_gate_init_grace_sec_);
    }
  }

  if (apply_rate_gate && heading_received_ && (last_heading_stamp_.seconds() > 0.0))
  {
    const double dt = (current_stamp - last_heading_stamp_).seconds();
    if (dt > 1e-6 && std::isfinite(dt) &&
      std::isfinite(heading_params.max_rate_radps) &&
      (heading_params.max_rate_radps > 0.0))
    {
      // Rate-gate reference:
      // - Prefer ESKF predicted yaw (more robust than last-good measurement).
      // - Fallback to last-good measurement if ESKF isn't initialized yet.
      double yaw_ref = std::numeric_limits<double>::quiet_NaN();
      {
        std::scoped_lock<std::mutex> lock(state_mutex_);
        if (eskf_.initialized()) {
          const Eigen::Matrix3d R = eskf_.q_map_from_base().toRotationMatrix();
          yaw_ref = std::atan2(R(1, 0), R(0, 0));
        }
      }
      if (!std::isfinite(yaw_ref) && std::isfinite(latest_heading_yaw_rad_)) {
        yaw_ref = latest_heading_yaw_rad_;
      }

      if (std::isfinite(yaw_ref)) {
        const double dyaw =
          std::atan2(
          std::sin(yaw_meas - yaw_ref),
          std::cos(yaw_meas - yaw_ref));
        const double rate = std::abs(dyaw) / dt;
        if (std::isfinite(rate) && rate > heading_params.max_rate_radps) {
        // Track consecutive skips and allow a guarded recovery bypass to avoid
        // getting stuck when the "reference" was previously contaminated.
        const bool have_window_start =
          (heading_rate_gate_skip_window_start_.seconds() > 0.0);
        if (heading_rate_gate_skip_count_ <= 0 || !have_window_start) {
          heading_rate_gate_skip_count_ = 0;
          heading_rate_gate_skip_window_start_ = current_stamp;
        }
        heading_rate_gate_skip_count_++;

        const double skip_elapsed_sec =
          (current_stamp - heading_rate_gate_skip_window_start_).seconds();
        const bool hit_count =
          (heading_params.rate_gate_skip_max_count > 0) &&
          (heading_rate_gate_skip_count_ >= heading_params.rate_gate_skip_max_count);
        const bool hit_time =
          (std::isfinite(heading_params.rate_gate_skip_max_sec)) &&
          (heading_params.rate_gate_skip_max_sec > 0.0) &&
          (std::isfinite(skip_elapsed_sec)) &&
          (skip_elapsed_sec >= heading_params.rate_gate_skip_max_sec);

        bool allow_bypass = hit_count || hit_time;
        if (allow_bypass) {
          // Health check: only bypass when the current sample would be usable
          // under the arbitrator's GNSS status/holdoff/freshness policies.
          const double scale = std::max(1.0, heading_params.rate_gate_bypass_yaw_var_scale);
          const double yaw_var_scaled = yaw_var_for_update * scale;
          const double stamp_sec = current_stamp.seconds();
          bool sample_usable = false;
          {
            std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
            const char * reason = nullptr;
            sample_usable =
              heading_arbitrator_.gphdt_sample_usable(
              yaw_meas, yaw_var_scaled, stamp_sec, stamp_sec, &reason);
          }

          if (sample_usable) {
            bypassed_rate_gate = true;
            yaw_var_for_update = yaw_var_scaled;
            last_yaw_meas_var_rad2_ = yaw_var_for_update;
            last_heading_yaw_var_pre_nis_dbg_ = yaw_var_for_update;
            last_heading_yaw_var_source_dbg_ = "rate_bypass";
            heading_rate_gate_skip_count_ = 0;
            heading_rate_gate_skip_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          }
        }

        if (!bypassed_rate_gate) {
          last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
          last_heading_yaw_update_dbg_.reason = "rate_gate_skip";
          raise_heading_recover_inflate("rate_gate_skip", current_stamp);
          (void)refresh_heading_var_snapshot("inflated_event");
          last_heading_stamp_ = current_stamp;
          return;
        }
      }
      }
    }
  }

  // Passed the rate gate -> clear skip state.
  if (!bypassed_rate_gate) {
    heading_rate_gate_skip_count_ = 0;
    heading_rate_gate_skip_window_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  const double stamp_sec = current_stamp.seconds();
  if (bypassed_rate_gate) {
    last_heading_yaw_var_source_dbg_ = "rate_bypass";
  }
  {
    std::scoped_lock<std::mutex> lock(heading_arbitrator_mutex_);
    heading_arbitrator_.update_gphdt(yaw_meas, yaw_var_for_update, stamp_sec);
    const auto yaw_sel = heading_arbitrator_.select(stamp_sec);
    if (yaw_sel.source != GnssHeadingSource::kGphdt) {
      const char * reason = nullptr;
      heading_arbitrator_.gphdt_usable(stamp_sec, &reason);
      last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
      last_heading_yaw_update_dbg_.reason =
        (reason != nullptr) ? reason : "heading_skip";
      raise_heading_recover_inflate(
        last_heading_yaw_update_dbg_.reason, current_stamp);
      (void)refresh_heading_var_snapshot("inflated_event");
      last_heading_stamp_ = current_stamp;
      return;
    }
  }

  latest_heading_ = t_msg;
  latest_heading_yaw_rad_ = yaw_meas;

  if (!heading_received_) {
    RCLCPP_INFO(
      this->get_logger(), "First heading received: %.2f deg -> ENU yaw %.4f rad (%.2f deg)", t_msg->heading, latest_heading_yaw_rad_,
      latest_heading_yaw_rad_ * 180.0 / M_PI);
    heading_received_ = true;
  }

  if (!activated) {
    last_heading_stamp_ = current_stamp;
    return;
  }

  // heading 기반 yaw 업데이트
  std::scoped_lock<std::mutex> lock(state_mutex_);
  if (!eskf_.initialized()) {
    last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
    last_heading_yaw_update_dbg_.reason =
      use_external_initialpose_ ? "external_initialpose_mode" : "not_initialized";

    if (!use_external_initialpose_ && pending_init_position_ &&
      std::isfinite(yaw_meas))
    {
      const Eigen::AngleAxisd yaw_aa(yaw_meas, Eigen::Vector3d::UnitZ());
      const Eigen::Quaterniond q_init(yaw_aa);
      eskf_.initialize(pending_init_p_map_, q_init);
      eskf_init_stamp_ = current_stamp;
      state_stamp_ = std::max(pending_init_stamp_, current_stamp);
      pending_init_position_ = false;
      RCLCPP_INFO(
        this->get_logger(), "ESKF initialized from cached GNSS+Heading (p=[%.3f, " "%.3f, %.3f], yaw=%.3f rad)",
        pending_init_p_map_.x(), pending_init_p_map_.y(), pending_init_p_map_.z(), yaw_meas);
    }
  } else if (state_stamp_.seconds() > 0.0 &&
    (current_stamp +
    rclcpp::Duration::from_seconds(
      std::max(0.0, node_params_.time_alignment_tolerance_sec))) <
    state_stamp_)
  {
    last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
    last_heading_yaw_update_dbg_.reason = "time_alignment";
  } else if (!heading_params.enable_yaw_update) {
    last_heading_yaw_update_dbg_ = EskfYawUpdateDebug{};
    last_heading_yaw_update_dbg_.reason = "disabled";
  } else {
    last_heading_yaw_update_dbg_ =
      eskf_.update_heading_yaw(yaw_meas, yaw_var_for_update);
    if (last_heading_yaw_update_dbg_.reason.empty()) {
      last_heading_yaw_update_dbg_.reason =
        bypassed_rate_gate ? "rate_gate_bypass_heading" : "heading";
    } else if (last_heading_yaw_update_dbg_.reason == "nis_inflated") {
      if (bypassed_rate_gate) {
        last_heading_yaw_update_dbg_.reason = "rate_gate_bypass_nis_inflated";
      }
    } else {
      last_heading_yaw_update_dbg_.reason =
        (bypassed_rate_gate ? "rate_gate_bypass_heading_" : "heading_") +
        last_heading_yaw_update_dbg_.reason;
    }
    if (last_heading_yaw_update_dbg_.applied) {
      if (std::isfinite(last_heading_yaw_update_dbg_.R) &&
        (last_heading_yaw_update_dbg_.R > 0.0))
      {
        last_heading_yaw_var_applied_dbg_ = last_heading_yaw_update_dbg_.R;
      }
      last_heading_yaw_var_source_dbg_ =
        bypassed_rate_gate ? "applied_rate_bypass" : "applied";
      if (current_stamp > state_stamp_) {
        state_stamp_ = current_stamp;
      }
    } else if (!last_heading_yaw_update_dbg_.reason.empty()) {
      raise_heading_recover_inflate(last_heading_yaw_update_dbg_.reason, current_stamp);
      yaw_var_for_update = refresh_heading_var_snapshot("inflated_event");
      (void)yaw_var_for_update;
    }
  }

  last_heading_stamp_ = current_stamp;
}

} // namespace hybrid_localization
