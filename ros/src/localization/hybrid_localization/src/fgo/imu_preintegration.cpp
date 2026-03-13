#include "hybrid_localization/fgo/imu_preintegration.hpp"

#include <cmath>

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// SO(3) 유틸리티
// ---------------------------------------------------------------------------

Eigen::Matrix3d ImuPreintegration::skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d S;
  S <<  0.0,  -v.z(),  v.y(),
        v.z(),  0.0,  -v.x(),
       -v.y(),  v.x(),  0.0;
  return S;
}

Eigen::Quaterniond ImuPreintegration::exp_map(const Eigen::Vector3d & phi)
{
  // Exp(φ) = (cos(|φ|/2), sin(|φ|/2) * φ/|φ|)
  const double angle = phi.norm();
  if (angle < 1.0e-10) {
    return Eigen::Quaterniond{1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()}.normalized();
  }
  const double half = 0.5 * angle;
  const double s = std::sin(half) / angle;
  return Eigen::Quaterniond{std::cos(half), s * phi.x(), s * phi.y(), s * phi.z()};
}

Eigen::Vector3d ImuPreintegration::log_map(const Eigen::Quaterniond & q)
{
  // Log(q): φ = 2 * atan2(|v|, w) * v/|v|
  const Eigen::Vector3d v = q.vec();
  const double v_norm = v.norm();
  const double w = q.w();
  if (v_norm < 1.0e-10) {
    return 2.0 * v;
  }
  return 2.0 * std::atan2(v_norm, w) / v_norm * v;
}

Eigen::Matrix3d ImuPreintegration::right_jacobian(const Eigen::Vector3d & phi)
{
  // Jr(φ) = I - (1-cos|φ|)/|φ|² * skew(φ) + (|φ|-sin|φ|)/|φ|³ * skew²(φ)
  // 소각 근사: Jr ≈ I for |φ| < 1e-4
  const double angle = phi.norm();
  if (angle < 1.0e-4) {
    return Eigen::Matrix3d::Identity();
  }
  const double angle2 = angle * angle;
  const double angle3 = angle2 * angle;
  const Eigen::Matrix3d S = skew(phi);
  return Eigen::Matrix3d::Identity()
    - (1.0 - std::cos(angle)) / angle2 * S
    + (angle - std::sin(angle)) / angle3 * S * S;
}

// ---------------------------------------------------------------------------
// ImuPreintegration 구현
// ---------------------------------------------------------------------------

ImuPreintegration::ImuPreintegration()
: ImuPreintegration(Params{})
{
}

ImuPreintegration::ImuPreintegration(const Params & params)
: params_(params)
{
  reset(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void ImuPreintegration::reset(
  const Eigen::Vector3d & lin_b_g,
  const Eigen::Vector3d & lin_b_a)
{
  lin_b_g_ = lin_b_g;
  lin_b_a_ = lin_b_a;

  delta_R_ = Eigen::Quaterniond::Identity();
  delta_v_.setZero();
  delta_p_.setZero();
  delta_t_ = 0.0;
  num_samples_ = 0;

  cov_.setZero();
  J_bg_.setZero();
  J_ba_.setZero();
}

void ImuPreintegration::push(
  const Eigen::Vector3d & gyro_raw,
  const Eigen::Vector3d & accel_raw,
  double dt)
{
  if (dt <= 0.0) {
    return;
  }

  // 바이어스 보정
  const Eigen::Vector3d gyro = gyro_raw - lin_b_g_;
  const Eigen::Vector3d accel = accel_raw - lin_b_a_;

  // 현재 ΔR 회전 행렬
  const Eigen::Matrix3d R = delta_R_.toRotationMatrix();

  // ---- 상태 전파 (midpoint integration) -----------------------------------
  // 다음 ΔR (midpoint 각속도 사용, 단순 1차로 충분)
  const Eigen::Vector3d phi = gyro * dt;
  const Eigen::Quaterniond dq = exp_map(phi);
  const Eigen::Quaterniond R_new = (delta_R_ * dq).normalized();

  // Δv, Δp (현재 ΔR 사용)
  const Eigen::Vector3d a_world = R * accel;  // body-i 프레임으로 회전된 가속도
  delta_p_ += delta_v_ * dt + 0.5 * a_world * dt * dt;
  delta_v_ += a_world * dt;
  delta_R_ = R_new;
  delta_t_ += dt;
  ++num_samples_;

  // ---- 공분산 전파 ---------------------------------------------------------
  // F (9×9): [δθ, δv, δp] 오차 상태 전이
  //   dδθ/dt = -skew(gyro) * δθ - Jr * δb_g
  //   dδv/dt = -R * skew(accel) * δθ - R * δb_a
  //   dδp/dt = δv
  Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
  const Eigen::Matrix3d S_phi = skew(phi);        // skew(ω * dt)
  const Eigen::Matrix3d S_acc = skew(accel);       // skew(a_corr)

  // δθ 블록
  F.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() - S_phi;  // ≈ Exp(-ω*dt)
  // δv 블록
  F.block<3, 3>(3, 0) = -R * S_acc * dt;
  F.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  // δp 블록
  F.block<3, 3>(6, 0) = -0.5 * R * S_acc * dt * dt;
  F.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;
  F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

  // Q (9×9): 연속 시간 노이즈 → 이산화
  // gyro noise: σ_g²/dt → 이산 분산 σ_g² * dt (적분 방향)
  // accel noise: σ_a²/dt → 이산 분산 σ_a² * dt
  const double sig_g2 = params_.gyro_noise_std * params_.gyro_noise_std;
  const double sig_a2 = params_.accel_noise_std * params_.accel_noise_std;

  // 노이즈 입력 행렬 B (9×6): [δθ ← gyro, δv ← accel, δp ← accel]
  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  B.block<3, 3>(0, 0) = right_jacobian(phi) * dt;     // δθ ← gyro
  B.block<3, 3>(3, 3) = R * dt;                         // δv ← accel
  B.block<3, 3>(6, 3) = 0.5 * R * dt * dt;             // δp ← accel

  // 이산 노이즈 공분산 Q_d = B * diag(σ²/dt) * B^T
  Eigen::Matrix<double, 6, 6> Q_cont = Eigen::Matrix<double, 6, 6>::Zero();
  Q_cont.block<3, 3>(0, 0) = sig_g2 * Eigen::Matrix3d::Identity();
  Q_cont.block<3, 3>(3, 3) = sig_a2 * Eigen::Matrix3d::Identity();

  cov_ = F * cov_ * F.transpose() + B * (Q_cont / dt) * B.transpose();

  // ---- 바이어스 야코비안 전파 -----------------------------------------------
  // J_bg: ∂[δθ, δv, δp] / ∂b_g 의 선형 전파
  const Eigen::Matrix3d Jr = right_jacobian(phi);

  Eigen::Matrix<double, 9, 3> J_bg_new = Eigen::Matrix<double, 9, 3>::Zero();
  // δθ 야코비안
  J_bg_new.block<3, 3>(0, 0) =
    (Eigen::Matrix3d::Identity() - S_phi) * J_bg_.block<3, 3>(0, 0) - Jr * dt;
  // δv 야코비안
  J_bg_new.block<3, 3>(3, 0) =
    J_bg_.block<3, 3>(3, 0) - R * S_acc * J_bg_.block<3, 3>(0, 0) * dt;
  // δp 야코비안
  J_bg_new.block<3, 3>(6, 0) =
    J_bg_.block<3, 3>(6, 0) + J_bg_.block<3, 3>(3, 0) * dt
    - 0.5 * R * S_acc * J_bg_.block<3, 3>(0, 0) * dt * dt;
  J_bg_ = J_bg_new;

  Eigen::Matrix<double, 9, 3> J_ba_new = Eigen::Matrix<double, 9, 3>::Zero();
  // δv 야코비안
  J_ba_new.block<3, 3>(3, 0) = J_ba_.block<3, 3>(3, 0) - R * dt;
  // δp 야코비안
  J_ba_new.block<3, 3>(6, 0) =
    J_ba_.block<3, 3>(6, 0) + J_ba_.block<3, 3>(3, 0) * dt - 0.5 * R * dt * dt;
  J_ba_ = J_ba_new;
}

// ---------------------------------------------------------------------------
// 바이어스 1차 보정
// ---------------------------------------------------------------------------

Eigen::Quaterniond ImuPreintegration::corrected_delta_R(
  const Eigen::Vector3d & delta_b_g) const
{
  const Eigen::Vector3d dphi = J_bg_.block<3, 3>(0, 0) * delta_b_g;
  return delta_R_ * exp_map(dphi);
}

Eigen::Vector3d ImuPreintegration::corrected_delta_v(
  const Eigen::Vector3d & delta_b_g,
  const Eigen::Vector3d & delta_b_a) const
{
  return delta_v_
    + J_bg_.block<3, 3>(3, 0) * delta_b_g
    + J_ba_.block<3, 3>(3, 0) * delta_b_a;
}

Eigen::Vector3d ImuPreintegration::corrected_delta_p(
  const Eigen::Vector3d & delta_b_g,
  const Eigen::Vector3d & delta_b_a) const
{
  return delta_p_
    + J_bg_.block<3, 3>(6, 0) * delta_b_g
    + J_ba_.block<3, 3>(6, 0) * delta_b_a;
}

}  // namespace hybrid_localization
