#ifndef HYBRID_LOCALIZATION__FGO__IMU_PREINTEGRATION_HPP_
#define HYBRID_LOCALIZATION__FGO__IMU_PREINTEGRATION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// ImuPreintegration
//
// 연속된 두 키프레임 i → j 사이 IMU 측정치를 사전적분(preintegration)하여
// FGO IMU 팩터에 필요한 상대 운동량(ΔR, Δv, Δp)과 공분산, 바이어스
// 야코비안을 계산한다.
//
// 공식 참조:
//   Forster et al., "On-Manifold Preintegration for Real-Time Visual–Inertial
//   Odometry," IEEE Trans. Robotics 2017.
//
// 상태 벡터 순서: [δθ(3), δv(3), δp(3)] (9-DOF)
// ---------------------------------------------------------------------------
class ImuPreintegration
{
public:
  using Mat9 = Eigen::Matrix<double, 9, 9>;
  using Mat93 = Eigen::Matrix<double, 9, 3>;
  using Mat96 = Eigen::Matrix<double, 9, 6>;

  struct Params
  {
    double gyro_noise_std{0.05};       // [rad/s/√Hz]
    double accel_noise_std{1.0};       // [m/s²/√Hz]
    double gyro_bias_rw_std{1.0e-4};   // [rad/s²/√Hz] gyro bias random walk
    double accel_bias_rw_std{1.0e-3};  // [m/s³/√Hz] accel bias random walk
    Eigen::Vector3d gravity_map{0.0, 0.0, -9.80665};  // map 프레임 중력 벡터
  };

  explicit ImuPreintegration(const Params & params = Params{});

  // 선형화 기준점 바이어스를 설정하고 상태 초기화
  void reset(
    const Eigen::Vector3d & lin_b_g,
    const Eigen::Vector3d & lin_b_a);

  // 단일 IMU 샘플 적분 (raw 측정값, 바이어스 보정은 내부에서 수행)
  void push(
    const Eigen::Vector3d & gyro_raw,
    const Eigen::Vector3d & accel_raw,
    double dt);

  // ---- 결과 접근자 -------------------------------------------------------

  // 적분된 상대 회전 ΔR_{ij}
  const Eigen::Quaterniond & delta_R() const {return delta_R_;}

  // 적분된 상대 속도 Δv_{ij}  [m/s, body-i 프레임]
  const Eigen::Vector3d & delta_v() const {return delta_v_;}

  // 적분된 상대 위치 Δp_{ij}  [m, body-i 프레임]
  const Eigen::Vector3d & delta_p() const {return delta_p_;}

  // 총 적분 시간
  double delta_t() const {return delta_t_;}

  // 축적된 IMU 샘플 수
  int num_samples() const {return num_samples_;}

  // 9×9 preintegration 공분산 (순서: [δθ, δv, δp])
  const Mat9 & covariance() const {return cov_;}

  // ΔR/Δv/Δp 의 자이로 바이어스 야코비안 (바이어스 1차 보정용)
  const Mat93 & jacobian_bg() const {return J_bg_;}

  // ΔR/Δv/Δp 의 가속도계 바이어스 야코비안
  const Mat93 & jacobian_ba() const {return J_ba_;}

  // 선형화 기준점 바이어스
  const Eigen::Vector3d & lin_b_g() const {return lin_b_g_;}
  const Eigen::Vector3d & lin_b_a() const {return lin_b_a_;}

  const Params & params() const {return params_;}

  bool valid() const {return num_samples_ > 0;}

  // 바이어스가 δb로 변화했을 때 1차 보정된 적분값
  Eigen::Quaterniond corrected_delta_R(const Eigen::Vector3d & delta_b_g) const;
  Eigen::Vector3d corrected_delta_v(
    const Eigen::Vector3d & delta_b_g,
    const Eigen::Vector3d & delta_b_a) const;
  Eigen::Vector3d corrected_delta_p(
    const Eigen::Vector3d & delta_b_g,
    const Eigen::Vector3d & delta_b_a) const;

private:
  // SO(3) 유틸리티
  static Eigen::Matrix3d skew(const Eigen::Vector3d & v);
  static Eigen::Quaterniond exp_map(const Eigen::Vector3d & phi);  // Exp(φ)
  static Eigen::Vector3d log_map(const Eigen::Quaterniond & q);    // Log(q)
  // Right Jacobian of SO(3): Jr(φ) ≈ I - skew(φ)/2 + ... (for small φ: ≈ I)
  static Eigen::Matrix3d right_jacobian(const Eigen::Vector3d & phi);

  Params params_;

  // 적분 결과
  Eigen::Quaterniond delta_R_{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d delta_v_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d delta_p_{Eigen::Vector3d::Zero()};
  double delta_t_{0.0};
  int num_samples_{0};

  // 선형화 기준점
  Eigen::Vector3d lin_b_g_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d lin_b_a_{Eigen::Vector3d::Zero()};

  // 공분산 (9×9: [δθ, δv, δp])
  Mat9 cov_{Mat9::Zero()};

  // 바이어스 야코비안 (9×3)
  Mat93 J_bg_{Mat93::Zero()};
  Mat93 J_ba_{Mat93::Zero()};
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__IMU_PREINTEGRATION_HPP_
