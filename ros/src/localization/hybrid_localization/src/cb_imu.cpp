#include "hybrid_localization/hybrid_localization_node.hpp"

#include <cmath>

#include <gtsam/base/Vector.h>

namespace hybrid_localization
{

// NOTE: IMU 콜백은 시간 검증 → IMU 전처리 → (gyro 기반) ESKF 전파 흐름으로 구성.
void HybridLocalizationNode::imu_callback(
  const sensor_msgs::msg::Imu::SharedPtr t_msg)
{
  const bool activated = (!node_params_.init.require_trigger) || is_activated_;

  const rclcpp::Time current_stamp(t_msg->header.stamp);
  const rclcpp::Time now = this->now();
  if (!validate_stamp_and_order(current_stamp, now, last_imu_stamp_, "IMU")) {
    return;
  }

  if (last_imu_stamp_.seconds() > 0.0) {
    const double dt =
      time_processor_.compute_clamped_dt(last_imu_stamp_, current_stamp);
    imu_dt_stats_.update(dt);
  }

  if (!tf_cache_.extrinsics().imu_valid && !t_msg->header.frame_id.empty()) {
    tf_cache_.cache_imu_from_frame_id(
      t_msg->header.frame_id,
      this->get_logger());
  }

  // IMU 캘리브레이션 또는 전처리 경로 선택
  if (node_params_.init_imu_calibration &&
    imu_calibration_manager_.in_progress())
  {
    imu_calibration_manager_.process_sample(
      imu_preprocessor_, *t_msg, current_stamp,
      node_params_.calibration_file_path, this->get_logger());
  } else if (imu_preprocessor_.calibration().valid) {
    if (!tf_cache_.extrinsics().imu_valid) {
      return;
    }

    double dt = 0.02;
    if (last_imu_stamp_.seconds() > 0.0) {
      dt = time_processor_.compute_clamped_dt(last_imu_stamp_, current_stamp);
    }

    // 전처리 결과(베이스 프레임 기준) 추출
    ImuPreprocessResult imu_result;
    const auto status = imu_preprocessor_.preprocess(
      *t_msg, tf_cache_.extrinsics().base_to_imu, dt, imu_result);

    if (status != ImuPreprocessStatus::kOk) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(), 5000, "IMU preprocessing failed (status: %d)",
        static_cast<int>(status));
      return;
    }

    { // ESKF 전파 (초기화 완료 + activated 상태에서만)
      std::scoped_lock<std::mutex> lock(state_mutex_);

      last_omega_base_ = imu_result.angular_velocity_base;
      have_last_omega_base_ = true;

      if (activated && eskf_.initialized()) {
        const Eigen::Vector3d omega(imu_result.angular_velocity_base.x,
          imu_result.angular_velocity_base.y,
          imu_result.angular_velocity_base.z);
        // IMU 가속도는 신뢰하지 않으므로 전파 입력에서는 사용하지 않는다.
        // propagate() 내부에서 accel_meas - b_a를 사용하므로, meas=b_a로 넣어
        // a=0이 되도록 만든다(바이어스 추정값이 바뀌어도 가속도가 끼지 않게).
        const Eigen::Vector3d accel = eskf_.b_a();
        eskf_.propagate(omega, accel, dt);
        state_stamp_ = current_stamp;

        // FGO Stage 2+3: raw IMU를 사전적분 버퍼에 누적
        // raw gyro/accel: 전처리(LPF/중력제거) 이전 값을 사용해 정밀도 유지
        const Eigen::Vector3d gyro_raw(t_msg->angular_velocity.x,
          t_msg->angular_velocity.y, t_msg->angular_velocity.z);
        const Eigen::Vector3d accel_raw(t_msg->linear_acceleration.x,
          t_msg->linear_acceleration.y, t_msg->linear_acceleration.z);
        imu_preint_.push(gyro_raw, accel_raw, dt);

        // GTSAM 사전적분 누적 (FGO CombinedImuFactor 입력)
        if (gtsam_preint_) {
          gtsam_preint_->integrateMeasurement(
            gtsam::Vector3(gyro_raw.x(), gyro_raw.y(), gyro_raw.z()),
            gtsam::Vector3(accel_raw.x(), accel_raw.y(), accel_raw.z()),
            dt);
        }

        // FGO Stage 4: EskfCorrector 재적분을 위해 전처리 IMU 버퍼에 저장
        // gyro_radps: propagate() 에 전달된 전처리 각속도 (base 프레임, LPF 완료)
        // accel_mps2: zero-accel 모드의 b_a (재적분 시 eskf.b_a() 사용이므로 참조용)
        ImuMeasurement imu_buf_meas;
        imu_buf_meas.stamp_sec = current_stamp.seconds();
        imu_buf_meas.gyro_radps = omega;
        imu_buf_meas.accel_mps2 = accel;
        imu_buf_meas.dt = dt;
        imu_buffer_.push(imu_buf_meas);

        // 키프레임 생성 여부 확인
        NominalState cur_state;
        cur_state.p_map = eskf_.p_map();
        cur_state.v_map = eskf_.v_map();
        cur_state.q_map_from_base = eskf_.q_map_from_base();
        cur_state.b_g = eskf_.b_g();
        cur_state.b_a = eskf_.b_a();
        maybe_push_keyframe(current_stamp, cur_state, eskf_.P());
      }
    }

    sensor_msgs::msg::Imu preprocessed_msg;
    preprocessed_msg.header.stamp = t_msg->header.stamp;
    preprocessed_msg.header.frame_id = node_params_.io.base_frame;
    preprocessed_msg.angular_velocity = imu_result.angular_velocity_base;
    preprocessed_msg.linear_acceleration = imu_result.linear_acceleration_base;
    preprocessed_msg.orientation = imu_result.orientation_calibrated;
    preprocessed_msg.orientation_covariance = t_msg->orientation_covariance;
    preprocessed_msg.angular_velocity_covariance =
      t_msg->angular_velocity_covariance;
    preprocessed_msg.linear_acceleration_covariance =
      t_msg->linear_acceleration_covariance;
    preprocessed_imu_pub_->publish(preprocessed_msg);
  }

  last_imu_stamp_ = current_stamp;
  imu_count_++;
}

} // namespace hybrid_localization
