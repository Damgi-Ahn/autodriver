#include "hybrid_localization/hybrid_localization_node.hpp"

#include <cmath>

#include <gtsam/base/Vector.h>

namespace hybrid_localization
{

// NOTE: IMU 콜백은 시간 검증 → IMU 전처리 → (gyro 기반) ESKF 전파 흐름으로 구성.
void HybridLocalizationNode::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr t_msg)
{
  const bool activated = (!m_node_params.init.require_trigger) || m_is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(current_stamp, now, m_last_imu_stamp, "IMU")) {
    return;
  }

  if (m_last_imu_stamp.seconds() > 0.0) {
    const double dt =
      m_time_processor.compute_clamped_dt(m_last_imu_stamp, current_stamp);
    m_imu_dt_stats.update(dt);
  }

  if (!m_tf_cache.extrinsics().imu_valid && !t_msg->header.frame_id.empty()) {
    m_tf_cache.cache_imu_from_frame_id(
      t_msg->header.frame_id,
      this->get_logger());
  }

  // IMU 캘리브레이션 또는 전처리 경로 선택
  if (m_node_params.init_imu_calibration &&
    m_imu_calibration_manager.in_progress())
  {
    m_imu_calibration_manager.process_sample(
      m_imu_preprocessor, *t_msg, current_stamp,
      m_node_params.calibration_file_path, this->get_logger());
  } else if (m_imu_preprocessor.calibration().valid) {
    if (!m_tf_cache.extrinsics().imu_valid) {
      return;
    }

    double dt = 0.02;
    if (m_last_imu_stamp.seconds() > 0.0) {
      dt = m_time_processor.compute_clamped_dt(m_last_imu_stamp, current_stamp);
    }

    // 전처리 결과(베이스 프레임 기준) 추출
    ImuPreprocessResult imu_result;
    const auto status = m_imu_preprocessor.preprocess(
      *t_msg, m_tf_cache.extrinsics().base_to_imu, dt, imu_result);

    if (status != ImuPreprocessStatus::kOk) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 5000, "IMU preprocessing failed (status: %d)",
        static_cast<int>(status));
      return;
    }

    { // ESKF 전파 (초기화 완료 + activated 상태에서만)
      std::scoped_lock<std::mutex> lock(m_state_mutex);

      m_last_omega_base = imu_result.angular_velocity_base;
      m_have_last_omega_base = true;

      if (activated && m_eskf.initialized()) {
        const Eigen::Vector3d omega(imu_result.angular_velocity_base.x,
          imu_result.angular_velocity_base.y,
          imu_result.angular_velocity_base.z);
        // IMU 가속도는 신뢰하지 않으므로 전파 입력에서는 사용하지 않는다.
        // propagate() 내부에서 accel_meas - b_a를 사용하므로, meas=b_a로 넣어
        // a=0이 되도록 만든다(바이어스 추정값이 바뀌어도 가속도가 끼지 않게).
        const Eigen::Vector3d accel = m_eskf.b_a();
        m_eskf.propagate(omega, accel, dt);
        m_state_stamp = current_stamp;

        // FGO Stage 2+3: raw IMU를 사전적분 버퍼에 누적
        // raw gyro/accel: 전처리(LPF/중력제거) 이전 값을 사용해 정밀도 유지
        const Eigen::Vector3d gyro_raw(t_msg->angular_velocity.x,
          t_msg->angular_velocity.y, t_msg->angular_velocity.z);
        const Eigen::Vector3d accel_raw(t_msg->linear_acceleration.x,
          t_msg->linear_acceleration.y, t_msg->linear_acceleration.z);
        m_imu_preint.push(gyro_raw, accel_raw, dt);

        // GTSAM 사전적분 누적 (FGO CombinedImuFactor 입력)
        if (m_gtsam_preint_) {
          m_gtsam_preint_->integrateMeasurement(
            gtsam::Vector3(gyro_raw.x(), gyro_raw.y(), gyro_raw.z()),
            gtsam::Vector3(accel_raw.x(), accel_raw.y(), accel_raw.z()),
            dt);
        }

        // 키프레임 생성 여부 확인
        NominalState cur_state;
        cur_state.p_map = m_eskf.p_map();
        cur_state.v_map = m_eskf.v_map();
        cur_state.q_map_from_base = m_eskf.q_map_from_base();
        cur_state.b_g = m_eskf.b_g();
        cur_state.b_a = m_eskf.b_a();
        maybe_push_keyframe(current_stamp, cur_state, m_eskf.P());
      }
    }

    sensor_msgs::msg::Imu preprocessed_msg;
    preprocessed_msg.header.stamp = t_msg->header.stamp;
    preprocessed_msg.header.frame_id = m_node_params.io.base_frame;
    preprocessed_msg.angular_velocity = imu_result.angular_velocity_base;
    preprocessed_msg.linear_acceleration = imu_result.linear_acceleration_base;
    preprocessed_msg.orientation = imu_result.orientation_calibrated;
    preprocessed_msg.orientation_covariance = t_msg->orientation_covariance;
    preprocessed_msg.angular_velocity_covariance =
      t_msg->angular_velocity_covariance;
    preprocessed_msg.linear_acceleration_covariance =
      t_msg->linear_acceleration_covariance;
    m_preprocessed_imu_pub->publish(preprocessed_msg);
  }

  m_last_imu_stamp = current_stamp;
  m_imu_count++;
}

} // namespace hybrid_localization
