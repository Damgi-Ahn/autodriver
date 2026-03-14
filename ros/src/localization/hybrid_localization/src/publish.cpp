#include "hybrid_localization/hybrid_localization_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Eigenvalues>

namespace hybrid_localization
{

namespace
{
const char * state_to_string(LocalizationState s)
{
  switch (s) {
    case LocalizationState::UNINITIALIZED: return "UNINITIALIZED";
    case LocalizationState::INITIALIZING:  return "INITIALIZING";
    case LocalizationState::OPERATING:     return "OPERATING";
    case LocalizationState::DEGRADED:      return "DEGRADED";
    case LocalizationState::FAILED:        return "FAILED";
    default:                               return "UNKNOWN";
  }
}
} // namespace

void HybridLocalizationNode::transition_state(LocalizationState new_state)
{
  const LocalizationState prev = localization_state_.exchange(new_state);
  if (prev == new_state) {
    return;
  }
  RCLCPP_WARN(
    this->get_logger(),
    "[LocalizationState] %s → %s",
    state_to_string(prev), state_to_string(new_state));

  if (state_pub_) {
    std_msgs::msg::String msg;
    msg.data = state_to_string(new_state);
    state_pub_->publish(msg);
  }
}

// NOTE: 출력/진단은 고정 주기의 타이머에서 수행하여 시간 일관성을 유지한다.
void HybridLocalizationNode::publish_timer_callback()
{
  const bool activated =
    (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time odom_stamp =
    (state_stamp_.seconds() > 0.0) ? state_stamp_ : this->now();

  bool eskf_initialized = false;
  bool heading_received = false;
  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
  Eigen::Vector3d v_map = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q_map_from_base = Eigen::Quaterniond::Identity();
  EskfCore::P15 P = EskfCore::P15::Zero();
  geometry_msgs::msg::Vector3 omega_base{};
  bool have_omega = false;

  {
    std::scoped_lock<std::mutex> lock(state_mutex_);
    if (eskf_.initialized() && !eskf_.finite()) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 2000, "ESKF state became non-finite. Resetting filter.");
      transition_state(LocalizationState::FAILED);
      eskf_.reset();
      last_gnss_pos_update_dbg_ = EskfGnssPosUpdateDebug{};
      last_gnss_vel_update_dbg_ = EskfGnssVelUpdateDebug{};
    }
    eskf_initialized = eskf_.initialized();
    if (eskf_initialized) {
      p_map = eskf_.p_map();
      v_map = eskf_.v_map();
      q_map_from_base = eskf_.q_map_from_base();
      P = eskf_.P();
    }
    omega_base = last_omega_base_;
    have_omega = have_last_omega_base_;
  }
  heading_received = heading_received_;

  // ---- 상태 머신 평가 (publish timer 에서만 호출) --------------------------
  {
    const rclcpp::Time now = this->now();

    // GNSS 불량 여부: last_gnss_good_stamp_ 가 timeout 이상 갱신 안 된 경우
    bool gnss_degraded = true;
    {
      std::scoped_lock<std::mutex> lock(gnss_recover_mutex_);
      const double good_sec = last_gnss_good_stamp_.seconds();
      if (good_sec > 0.0) {
        gnss_degraded =
          (now - last_gnss_good_stamp_).seconds() >
          node_params_.gnss_degraded_timeout_sec;
      }
    }

    LocalizationState new_state;
    if (!activated) {
      new_state = LocalizationState::UNINITIALIZED;
    } else if (!eskf_initialized) {
      new_state = LocalizationState::INITIALIZING;
    } else if (gnss_degraded) {
      new_state = LocalizationState::DEGRADED;
    } else {
      new_state = LocalizationState::OPERATING;
    }
    transition_state(new_state);

    // 적응형 FGO 윈도우: DEGRADED 시 확장, 그 외 기본값으로 복귀
    if (node_params_.fgo_backend.adaptive_window_enable) {
      const int target_size = gnss_degraded ?
        node_params_.fgo_backend.adaptive_window_max :
        node_params_.fgo_backend.window_size;
      {
        std::scoped_lock<std::mutex> lock(state_mutex_);
        current_fgo_window_size_ = target_size;
      }
    }
  }

  // ---- ImuBuffer 오버플로우 경고 ------------------------------------------
  {
    const size_t overflow = imu_buffer_.take_overflow_count();
    if (overflow > 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "ImuBuffer overflow: %zu samples dropped (max_size=%zu). "
        "Consider increasing fgo.corrector.imu_buffer_max_samples.",
        overflow, imu_buffer_.max_size());
    }
  }

  const bool publish_ready =
    activated && eskf_initialized && heading_received;

  if (activated && !publish_ready) {
    const char * reason = !eskf_initialized ? "waiting_for_eskf_init" :
      (!heading_received ? "waiting_for_heading" : "not_ready");
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Output gated (%s): suppressing /localization/kinematic_state and TF", reason);
  }

  Eigen::Quaterniond q_map_from_base_out = q_map_from_base;
  // 출력용 자세: 평면 주행 모드면 yaw-only로 단순화
  if (eskf_initialized && node_params_.output.flatten_roll_pitch) {
    const Eigen::Matrix3d R = q_map_from_base.toRotationMatrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));
    q_map_from_base_out =
      Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  }

  nav_msgs::msg::Odometry odom_msg =
    eskf_initialized ?
    odom_builder_.build_initialized(
    odom_stamp, p_map, v_map,
    q_map_from_base_out, P, omega_base,
    have_omega) :
    odom_builder_.build_uninitialized(odom_stamp);

  if (eskf_initialized && node_params_.output.flatten_roll_pitch) {
    double var = node_params_.output.roll_pitch_var;
    if (!std::isfinite(var) || !(var > 0.0)) {
      var = 1000.0;
    }
    for (int c = 0; c < 6; ++c) {
      if (c != 3) {
        odom_msg.pose.covariance[3 * 6 + c] = 0.0;
        odom_msg.pose.covariance[c * 6 + 3] = 0.0;
      }
      if (c != 4) {
        odom_msg.pose.covariance[4 * 6 + c] = 0.0;
        odom_msg.pose.covariance[c * 6 + 4] = 0.0;
      }
    }
    odom_msg.pose.covariance[3 * 6 + 3] = var;
    odom_msg.pose.covariance[4 * 6 + 4] = var;
  }

  if (publish_ready && node_params_.io.publish_tf &&
    tf_broadcaster_)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    odom_builder_.build_tf(odom_msg, tf_msg);
    tf_broadcaster_->sendTransform(tf_msg);
  }

  if (publish_ready) {
    odom_pub_->publish(odom_msg);
    if (eskf_initialized && pose_pub_) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = odom_msg.header;
      pose_msg.pose = odom_msg.pose.pose;
      pose_pub_->publish(pose_msg);
    }
  }

  // 진단은 주기 다운샘플링 (기본 10회마다 1회)
  if (++diag_counter_ >= static_cast<size_t>(std::max(1, node_params_.diag_publish_divider))) {
    diag_counter_ = 0;

    EskfDiagnosticsInput diag_input;
    diag_input.imu_count = imu_count_;
    diag_input.gnss_count = gnss_count_;
    diag_input.gnss_vel_count = gnss_vel_count_;
    diag_input.velocity_count = velocity_count_;
    diag_input.steering_count = steering_count_;
    diag_input.is_activated = activated;
    diag_input.eskf_initialized = eskf_initialized;

    diag_input.has_vehicle_R = true;
    diag_input.vehicle_speed_var = node_params_.vehicle_constraints.speed_var;
    diag_input.nhc_var = node_params_.vehicle_constraints.nhc_var;
    diag_input.zupt_var = node_params_.vehicle_constraints.zupt_var;
    diag_input.yaw_rate_var = node_params_.vehicle_constraints.yaw_rate_var;

    diag_input.has_imu_Q = true;
    diag_input.imu_gyro_noise_std = node_params_.eskf.gyro_noise_std;
    diag_input.imu_accel_noise_std = node_params_.eskf.accel_noise_std;
    diag_input.imu_gyro_bias_noise_std =
      node_params_.eskf.gyro_bias_noise_std;
    diag_input.imu_accel_bias_noise_std =
      node_params_.eskf.accel_bias_noise_std;

    const auto latest_gnss = latest_gnss_;
    if (latest_gnss) {
      diag_input.has_gnss_status = true;
      diag_input.gnss_status = static_cast<int>(latest_gnss->status.status);
    }

    // NIS 기반 R 인플레이트 계수 계산
    const auto calc_nis_inflate = [this](const std::string & reason,
        const double nis,
        const double gate) -> double {
        if (reason != "nis_inflated") {
          return 1.0;
        }
        if (!std::isfinite(nis) || !std::isfinite(gate) || !(gate > 0.0)) {
          return 1.0;
        }
        double factor = nis / gate;
        if (!std::isfinite(factor)) {
          return 1.0;
        }
        if (factor < 1.0) {
          factor = 1.0;
        }
        double cap = std::max(1.0, node_params_.eskf.nis_gate_inflate_max);
        if (!std::isfinite(cap)) {
          cap = 1.0;
        }
        if (factor > cap) {
          factor = cap;
        }
        return factor;
      };

    double pos_recover_inflate_for_diag = 1.0;
    double vel_recover_inflate_for_diag = 1.0;
    double heading_status_inflate_for_diag = 1.0;
    double heading_recover_inflate_for_diag = 1.0;
    double heading_var_pre_nis_for_diag = node_params_.heading.yaw_var;
    double heading_var_applied_for_diag = std::numeric_limits<double>::quiet_NaN();
    std::string heading_var_source_for_diag = "normal";

    {
      std::scoped_lock<std::mutex> lock(state_mutex_);
      diag_input.gnss_pos_update = last_gnss_pos_update_dbg_;
      diag_input.gnss_vel_update = last_gnss_vel_update_dbg_;
      diag_input.heading_yaw_update = last_heading_yaw_update_dbg_;
      pos_recover_inflate_for_diag = last_gnss_pos_recover_inflate_dbg_;
      vel_recover_inflate_for_diag = last_gnss_vel_recover_inflate_dbg_;
      heading_status_inflate_for_diag = last_heading_status_inflate_dbg_;
      heading_recover_inflate_for_diag = last_heading_recover_inflate_dbg_;
      heading_var_pre_nis_for_diag = last_heading_yaw_var_pre_nis_dbg_;
      heading_var_applied_for_diag = last_heading_yaw_var_applied_dbg_;
      heading_var_source_for_diag = last_heading_yaw_var_source_dbg_;
      if (!std::isfinite(pos_recover_inflate_for_diag) || !(pos_recover_inflate_for_diag > 0.0)) {
        pos_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(vel_recover_inflate_for_diag) || !(vel_recover_inflate_for_diag > 0.0)) {
        vel_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_status_inflate_for_diag) || !(heading_status_inflate_for_diag > 0.0)) {
        heading_status_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_recover_inflate_for_diag) || !(heading_recover_inflate_for_diag > 0.0)) {
        heading_recover_inflate_for_diag = 1.0;
      }
      if (!std::isfinite(heading_var_pre_nis_for_diag) || !(heading_var_pre_nis_for_diag > 0.0)) {
        heading_var_pre_nis_for_diag = std::isfinite(last_yaw_meas_var_rad2_) ?
          last_yaw_meas_var_rad2_ :
          node_params_.heading.yaw_var;
      }
      if (!std::isfinite(heading_var_pre_nis_for_diag) || !(heading_var_pre_nis_for_diag > 0.0)) {
        heading_var_pre_nis_for_diag = 0.01;
      }
      if (heading_var_source_for_diag.empty()) {
        heading_var_source_for_diag = "normal";
      }

      if (eskf_initialized) {
        const auto P_now = eskf_.P();
        const auto P_sym = 0.5 * (P_now + P_now.transpose());
        const auto diag = P_sym.diagonal();

        diag_input.has_P_stats = true;
        diag_input.P_trace = P_sym.trace();
        diag_input.P_max_diag = diag.maxCoeff();
        diag_input.P_min_diag = diag.minCoeff();

        diag_input.P_pos_max_diag =
          P_sym.block<3, 3>(0, 0).diagonal().maxCoeff();
        diag_input.P_vel_max_diag =
          P_sym.block<3, 3>(3, 3).diagonal().maxCoeff();
        diag_input.P_att_max_diag =
          P_sym.block<3, 3>(6, 6).diagonal().maxCoeff();
        diag_input.P_bg_max_diag =
          P_sym.block<3, 3>(9, 9).diagonal().maxCoeff();
        diag_input.P_ba_max_diag =
          P_sym.block<3, 3>(12, 12).diagonal().maxCoeff();

        Eigen::SelfAdjointEigenSolver<EskfCore::P15> eig(P_sym);
        if (eig.info() == Eigen::Success) {
          diag_input.P_min_eig = eig.eigenvalues().minCoeff();
        } else {
          diag_input.P_min_eig = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }

    if (latest_gnss) {
      {
        double status_inflate = 1.0;
        if (diag_input.gnss_status == 0) {
          status_inflate = node_params_.gnss.pos_inflate_status_fix;
        } else if (diag_input.gnss_status == 1) {
          status_inflate = node_params_.gnss.pos_inflate_status_sbas;
        } else {
          status_inflate = 1.0;
        }
        if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
          status_inflate = 1.0;
        }

        // Use final R_diag from update if applied, otherwise compute for reference
        if (diag_input.gnss_pos_update.applied &&
            diag_input.gnss_pos_update.R_diag.allFinite()) {
          diag_input.has_gnss_pos_R = true;
          diag_input.gnss_pos_R_xx = diag_input.gnss_pos_update.R_diag(0);
          diag_input.gnss_pos_R_yy = diag_input.gnss_pos_update.R_diag(1);
          diag_input.gnss_pos_R_zz = diag_input.gnss_pos_update.R_diag(2);
        } else {
          GnssPosUpdateConfig config;
          config.use_navsatfix_covariance =
            node_params_.gnss.use_navsatfix_covariance;
          config.covariance_scale =
            node_params_.gnss.cov_scale *
            pos_recover_inflate_for_diag;
          config.pos_var_fallback = node_params_.gnss.pos_var_fallback;
          config.pos_var_min = node_params_.gnss.pos_var_min;
          config.pos_var_max = node_params_.gnss.pos_var_max;
          config.cov_diag_only = node_params_.gnss.pos_cov_diag_only;

          const Eigen::Matrix3d Rpos =
            gnss_update_handler_.compute_position_covariance(
            *latest_gnss,
            config);
          diag_input.has_gnss_pos_R = true;
          diag_input.gnss_pos_R_xx = Rpos(0, 0);
          diag_input.gnss_pos_R_yy = Rpos(1, 1);
          diag_input.gnss_pos_R_zz = Rpos(2, 2);
        }
        diag_input.gnss_pos_status_inflate = status_inflate;

        const double gate = node_params_.eskf.nis_gate_gnss_pos_3d;
        diag_input.gnss_pos_nis_inflate =
          calc_nis_inflate(
          diag_input.gnss_pos_update.reason,
          diag_input.gnss_pos_update.nis, gate);
      }

      {
        double status_inflate = 1.0;
        if (diag_input.gnss_status == 0) {
          status_inflate = node_params_.gnss.vel_inflate_status_fix;
        } else if (diag_input.gnss_status == 1) {
          status_inflate = node_params_.gnss.vel_inflate_status_sbas;
        } else {
          status_inflate = 1.0;
        }
        if (!std::isfinite(status_inflate) || !(status_inflate > 0.0)) {
          status_inflate = 1.0;
        }

        // Use final R_diag from update if applied, otherwise compute for reference
        if (diag_input.gnss_vel_update.applied &&
            diag_input.gnss_vel_update.R_diag.allFinite()) {
          diag_input.has_gnss_vel_R = true;
          diag_input.gnss_vel_R_xx = diag_input.gnss_vel_update.R_diag(0);
          diag_input.gnss_vel_R_yy = diag_input.gnss_vel_update.R_diag(1);
          diag_input.gnss_vel_R_zz = diag_input.gnss_vel_update.R_diag(2);
        } else {
          GnssVelUpdateConfig config;
          config.covariance_scale =
            node_params_.gnss.vel_cov_scale *
            vel_recover_inflate_for_diag;
          config.vel_var_fallback = node_params_.gnss.vel_var_fallback;
          config.vel_var_min = node_params_.gnss.vel_var_min;
          config.vel_var_max = node_params_.gnss.vel_var_max;

          const Eigen::Matrix3d Rvel =
            gnss_update_handler_.compute_velocity_covariance(config);
          diag_input.has_gnss_vel_R = true;
          diag_input.gnss_vel_R_xx = Rvel(0, 0);
          diag_input.gnss_vel_R_yy = Rvel(1, 1);
          diag_input.gnss_vel_R_zz = Rvel(2, 2);
        }
        diag_input.gnss_vel_status_inflate = status_inflate;

        const double gate = node_params_.eskf.nis_gate_gnss_vel_3d;
        diag_input.gnss_vel_nis_inflate =
          calc_nis_inflate(
          diag_input.gnss_vel_update.reason,
          diag_input.gnss_vel_update.nis, gate);
      }
    }

    diag_input.has_heading_yaw_R = true;
    // heading_yaw_var: pre-NIS measurement variance that will be fed to ESKF.
    diag_input.heading_yaw_var = heading_var_pre_nis_for_diag;
    diag_input.heading_status_inflate = heading_status_inflate_for_diag;
    diag_input.heading_recover_inflate = heading_recover_inflate_for_diag;
    diag_input.heading_yaw_var_source = heading_var_source_for_diag;
    diag_input.heading_yaw_var_applied = heading_var_applied_for_diag;
    if (diag_input.heading_yaw_update.applied &&
      std::isfinite(diag_input.heading_yaw_update.R) &&
      (diag_input.heading_yaw_update.R > 0.0))
    {
      diag_input.heading_yaw_var_eff = diag_input.heading_yaw_update.R;
      diag_input.heading_yaw_var_applied = diag_input.heading_yaw_update.R;
      if (diag_input.heading_yaw_var > 0.0) {
        diag_input.heading_yaw_nis_inflate =
          diag_input.heading_yaw_var_eff / diag_input.heading_yaw_var;
      } else {
        diag_input.heading_yaw_nis_inflate = 1.0;
      }
    } else {
      diag_input.heading_yaw_nis_inflate = 1.0;
      diag_input.heading_yaw_var_eff = diag_input.heading_yaw_var;
      diag_input.heading_yaw_var_applied = std::numeric_limits<double>::quiet_NaN();
    }

    if (imu_dt_stats_.has_samples()) {
      diag_input.has_imu_dt_stats = true;
      diag_input.imu_dt_stats = imu_dt_stats_;
      reset_imu_dt_stats();
    }

    const rclcpp::Time now = this->now();
    if (latest_gnss_) {
      diag_input.has_gnss_delay = true;
      diag_input.gnss_delay_sec = (now - last_gnss_stamp_).seconds();
    }
    if (latest_gnss_vel_) {
      diag_input.has_gnss_vel_delay = true;
      diag_input.gnss_vel_delay_sec = (now - last_gnss_vel_stamp_).seconds();
    }
    if (latest_velocity_) {
      diag_input.has_velocity_delay = true;
      diag_input.velocity_delay_sec = (now - last_velocity_stamp_).seconds();
    }
    if (latest_steering_) {
      diag_input.has_steering_delay = true;
      diag_input.steering_delay_sec = (now - last_steering_stamp_).seconds();
    }

    diag_input.has_fgo_stats = true;
    diag_input.fgo_keyframe_count = fgo_keyframe_count_;
    diag_input.fgo_correction_count = fgo_correction_count_;

    // 상태 머신 → 진단 레벨 매핑
    switch (localization_state_.load()) {
      case LocalizationState::OPERATING:
        diag_input.diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        break;
      case LocalizationState::DEGRADED:
        diag_input.diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        break;
      case LocalizationState::FAILED:
        diag_input.diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        break;
      case LocalizationState::INITIALIZING:
      case LocalizationState::UNINITIALIZED:
      default:
        diag_input.diag_level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
        break;
    }

    const auto diag_array = diag_builder_.build(now, diag_input);
    diag_pub_->publish(diag_array);
  }
}

} // namespace hybrid_localization
