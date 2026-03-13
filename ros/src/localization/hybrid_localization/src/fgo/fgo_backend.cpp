#include "hybrid_localization/fgo/fgo_backend.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "hybrid_localization/fgo/factors/gnss_heading_factor.hpp"
#include "hybrid_localization/fgo/factors/gnss_velocity_factor.hpp"
#include "hybrid_localization/fgo/factors/nhc_factor.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// 헬퍼: NominalState ↔ GTSAM 변환
// ---------------------------------------------------------------------------
static gtsam::Pose3 nominal_to_pose3(const NominalState & s)
{
  const gtsam::Rot3 rot(s.q_map_from_base.w(), s.q_map_from_base.x(),
    s.q_map_from_base.y(), s.q_map_from_base.z());
  return gtsam::Pose3(rot, gtsam::Point3(s.p_map.x(), s.p_map.y(), s.p_map.z()));
}

static gtsam::Vector3 nominal_to_vel3(const NominalState & s)
{
  return gtsam::Vector3(s.v_map.x(), s.v_map.y(), s.v_map.z());
}

static gtsam::imuBias::ConstantBias nominal_to_bias(const NominalState & s)
{
  // GTSAM convention: ConstantBias(accelerometer_bias, gyroscope_bias)
  return gtsam::imuBias::ConstantBias(
    gtsam::Vector3(s.b_a.x(), s.b_a.y(), s.b_a.z()),
    gtsam::Vector3(s.b_g.x(), s.b_g.y(), s.b_g.z()));
}

static NominalState gtsam_to_nominal(
  const gtsam::Pose3 & pose,
  const gtsam::Vector3 & vel,
  const gtsam::imuBias::ConstantBias & bias)
{
  NominalState s;
  s.p_map = Eigen::Vector3d(pose.x(), pose.y(), pose.z());
  s.v_map = Eigen::Vector3d(vel.x(), vel.y(), vel.z());
  const gtsam::Quaternion q = pose.rotation().toQuaternion();
  s.q_map_from_base = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
  s.b_a = Eigen::Vector3d(bias.accelerometer().x(), bias.accelerometer().y(),
    bias.accelerometer().z());
  s.b_g = Eigen::Vector3d(bias.gyroscope().x(), bias.gyroscope().y(),
    bias.gyroscope().z());
  return s;
}

// ---------------------------------------------------------------------------
// set_window_size()
// ---------------------------------------------------------------------------
void FgoBackend::set_window_size(int n)
{
  const int lo = 1;
  const int hi = params_.adaptive_window_enable ? params_.adaptive_window_max
                                                : params_.window_size;
  params_.window_size = std::max(lo, std::min(n, hi));
}

// initialize()
// ---------------------------------------------------------------------------
void FgoBackend::initialize(
  const Params & params,
  const ImuPreintegration::Params & imu_params)
{
  params_ = params;
  imu_params_ = imu_params;

  // ISAM2 파라미터 설정
  gtsam::ISAM2Params isam2_params;
  isam2_params.relinearizeThreshold = 0.1;
  isam2_params.relinearizeSkip = 1;
  isam2_params.enablePartialRelinearizationCheck = true;
  isam2_ = gtsam::ISAM2(isam2_params);

  // 노이즈 모델 생성
  prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() <<
    params_.prior_yaw_noise_rad, params_.prior_yaw_noise_rad, params_.prior_yaw_noise_rad,
    params_.prior_pos_noise_m, params_.prior_pos_noise_m, params_.prior_pos_noise_m
    ).finished());

  prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params_.prior_vel_noise_mps);

  prior_bias_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() <<
    params_.prior_bias_accel_noise, params_.prior_bias_accel_noise, params_.prior_bias_accel_noise,
    params_.prior_bias_gyro_noise, params_.prior_bias_gyro_noise, params_.prior_bias_gyro_noise
    ).finished());

  nhc_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector2(params_.nhc_lat_noise_mps, params_.nhc_vert_noise_mps));

  bias_rw_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector6() <<
    params_.bias_accel_rw_noise, params_.bias_accel_rw_noise, params_.bias_accel_rw_noise,
    params_.bias_gyro_rw_noise, params_.bias_gyro_rw_noise, params_.bias_gyro_rw_noise
    ).finished());

  // GTSAM IMU 사전적분 파라미터 (중력: -z 방향, ENU map 프레임)
  gtsam_preint_params_ =
    gtsam::PreintegrationCombinedParams::MakeSharedU(
    std::abs(imu_params_.gravity_map.z()));

  const double sigma_a = imu_params_.accel_noise_std;
  const double sigma_g = imu_params_.gyro_noise_std;
  const double sigma_ba = imu_params_.accel_bias_rw_std;
  const double sigma_bg = imu_params_.gyro_bias_rw_std;

  gtsam_preint_params_->setAccelerometerCovariance(
    sigma_a * sigma_a * gtsam::I_3x3);
  gtsam_preint_params_->setGyroscopeCovariance(
    sigma_g * sigma_g * gtsam::I_3x3);
  gtsam_preint_params_->setIntegrationCovariance(1e-8 * gtsam::I_3x3);
  gtsam_preint_params_->biasAccCovariance =
    sigma_ba * sigma_ba * gtsam::I_3x3;
  gtsam_preint_params_->biasOmegaCovariance =
    sigma_bg * sigma_bg * gtsam::I_3x3;
  gtsam_preint_params_->biasAccOmegaInt = 1e-5 * gtsam::I_6x6;

  initialized_ = true;
  first_keyframe_ = true;
}

// ---------------------------------------------------------------------------
// add_prior_factors()
// ---------------------------------------------------------------------------
void FgoBackend::add_prior_factors(const Keyframe & kf)
{
  const uint64_t idx = kf.index;
  const gtsam::Pose3 init_pose = nominal_to_pose3(kf.state);
  const gtsam::Vector3 init_vel = nominal_to_vel3(kf.state);
  const gtsam::imuBias::ConstantBias init_bias = nominal_to_bias(kf.state);

  new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
    x_sym(idx), init_pose, prior_pose_noise_);
  new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
    v_sym(idx), init_vel, prior_vel_noise_);
  new_factors_.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    b_sym(idx), init_bias, prior_bias_noise_);

  new_values_.insert(x_sym(idx), init_pose);
  new_values_.insert(v_sym(idx), init_vel);
  new_values_.insert(b_sym(idx), init_bias);
}

// ---------------------------------------------------------------------------
// add_imu_factor()
// ---------------------------------------------------------------------------
void FgoBackend::add_imu_factor(const Keyframe & kf)
{
  if (!kf.gtsam_preint) {
    return;  // 첫 번째 키프레임은 IMU 팩터 없음
  }

  const uint64_t j = kf.index;
  const uint64_t i = prev_index_;

  new_factors_.emplace_shared<gtsam::CombinedImuFactor>(
    x_sym(i), v_sym(i), x_sym(j), v_sym(j), b_sym(i), b_sym(j),
    *kf.gtsam_preint);

  // 초기값: ESKF 상태 사용
  new_values_.insert(x_sym(j), nominal_to_pose3(kf.state));
  new_values_.insert(v_sym(j), nominal_to_vel3(kf.state));
  new_values_.insert(b_sym(j), nominal_to_bias(kf.state));

  // 바이어스 랜덤 워크 팩터 (bias_i → bias_j)
  new_factors_.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
    b_sym(i), b_sym(j),
    gtsam::imuBias::ConstantBias{},  // 기대 변화량 = 0
    bias_rw_noise_);
}

// ---------------------------------------------------------------------------
// add_gnss_factors()
// ---------------------------------------------------------------------------
void FgoBackend::add_gnss_factors(
  uint64_t idx,
  const GnssMeasurement & gnss)
{
  // 위치 팩터
  if (gnss.status >= params_.gnss_min_status_for_pos) {
    const gtsam::Point3 gps_pos(gnss.pos_map.x(), gnss.pos_map.y(), gnss.pos_map.z());

    // GTSAM 3x3 공분산 → SharedNoiseModel
    Eigen::Matrix3d cov = gnss.pos_cov;
    // NaN/inf 방어
    if (!cov.allFinite() || cov.diagonal().minCoeff() <= 0.0) {
      cov = Eigen::Matrix3d::Identity() * 0.25;
    }
    const gtsam::SharedNoiseModel gnss_pos_noise =
      gtsam::noiseModel::Gaussian::Covariance(cov);
    new_factors_.emplace_shared<gtsam::GPSFactor>(x_sym(idx), gps_pos, gnss_pos_noise);
  }

  // 속도 팩터
  if (gnss.has_velocity && gnss.status >= params_.gnss_min_status_for_vel) {
    const gtsam::Vector3 vel_meas(gnss.vel_map.x(), gnss.vel_map.y(), gnss.vel_map.z());

    Eigen::Matrix3d vel_cov = gnss.vel_cov;
    if (!vel_cov.allFinite() || vel_cov.diagonal().minCoeff() <= 0.0) {
      vel_cov = Eigen::Matrix3d::Identity() * 0.01;
    }
    const gtsam::SharedNoiseModel gnss_vel_noise =
      gtsam::noiseModel::Gaussian::Covariance(vel_cov);
    new_factors_.emplace_shared<GnssVelocityFactor>(v_sym(idx), vel_meas, gnss_vel_noise);
  }

  // 헤딩 팩터
  if (gnss.has_heading) {
    const double heading_var = (gnss.heading_var > 0.0) ? gnss.heading_var : 0.01;
    const gtsam::SharedNoiseModel heading_noise =
      gtsam::noiseModel::Isotropic::Variance(1, heading_var);
    new_factors_.emplace_shared<GnssHeadingFactor>(x_sym(idx), gnss.heading_rad, heading_noise);
  }
}

// ---------------------------------------------------------------------------
// add_nhc_factor()
// ---------------------------------------------------------------------------
void FgoBackend::add_nhc_factor(uint64_t idx)
{
  new_factors_.emplace_shared<NhcFactor>(x_sym(idx), v_sym(idx), nhc_noise_);
}

// ---------------------------------------------------------------------------
// marginalize_oldest()
// ---------------------------------------------------------------------------
void FgoBackend::marginalize_oldest()
{
  if (window_.empty() ||
    static_cast<int>(window_.size()) <= params_.window_size)
  {
    return;
  }

  const uint64_t old_idx = window_.front();
  window_.pop_front();

  gtsam::FastList<gtsam::Key> keys_to_marginalize{
    x_sym(old_idx), v_sym(old_idx), b_sym(old_idx)};
  isam2_.marginalizeLeaves(keys_to_marginalize);
}

// ---------------------------------------------------------------------------
// extract_result()
// ---------------------------------------------------------------------------
FgoBackend::OptimizationResult FgoBackend::extract_result(uint64_t idx) const
{
  OptimizationResult result;
  try {
    const gtsam::Values estimate = isam2_.calculateEstimate();
    const auto pose = estimate.at<gtsam::Pose3>(x_sym(idx));
    const auto vel = estimate.at<gtsam::Vector3>(v_sym(idx));
    const auto bias = estimate.at<gtsam::imuBias::ConstantBias>(b_sym(idx));

    result.state = gtsam_to_nominal(pose, vel, bias);
    result.keyframe_index = idx;
    result.valid = true;

    // ---- 공분산 추출 (15×15) ------------------------------------------------
    // GTSAM 변수 접선 공간 순서:
    //   Pose3   [6×6]: [rotation_tangent(3), translation_tangent(3)]
    //   Vector3 [3×3]: velocity(3)
    //   Bias    [6×6]: [accel_bias(3), gyro_bias(3)]
    //
    // ESKF P15 순서:
    //   [0:3]   pos, [3:6]  vel, [6:9]  att,
    //   [9:12]  b_g, [12:15] b_a
    //
    // 대각 블록만 추출 (교차 항은 보수적으로 0 유지).
    // 슬라이딩 윈도우 내 활성 변수만 marginalCovariance 가능.
    const auto P_pose = isam2_.marginalCovariance(x_sym(idx));  // 6×6
    const auto P_vel  = isam2_.marginalCovariance(v_sym(idx));  // 3×3
    const auto P_bias = isam2_.marginalCovariance(b_sym(idx));  // 6×6

    Eigen::Matrix<double, 15, 15> P15 = Eigen::Matrix<double, 15, 15>::Zero();
    P15.block<3, 3>(0, 0) = P_pose.block<3, 3>(3, 3);   // pos ← Pose3[3:6, 3:6]
    P15.block<3, 3>(3, 3) = P_vel;                        // vel
    P15.block<3, 3>(6, 6) = P_pose.block<3, 3>(0, 0);   // att ← Pose3[0:3, 0:3]
    P15.block<3, 3>(9, 9) = P_bias.block<3, 3>(3, 3);   // b_g ← Bias[3:6, 3:6]
    P15.block<3, 3>(12, 12) = P_bias.block<3, 3>(0, 0); // b_a ← Bias[0:3, 0:3]

    // 수치 안전: NaN/inf 또는 비양정 대각이면 zero 반환
    if (P15.allFinite() && P15.diagonal().minCoeff() >= 0.0) {
      result.P = P15;
    } else {
      result.P = Eigen::Matrix<double, 15, 15>::Zero();
    }
  } catch (const std::exception & e) {
    result.valid = false;
  }
  return result;
}

// ---------------------------------------------------------------------------
// update() — 메인 엔트리포인트
// ---------------------------------------------------------------------------
FgoBackend::OptimizationResult FgoBackend::update(
  const Keyframe & kf,
  const std::optional<GnssMeasurement> & gnss)
{
  if (!initialized_) {
    return OptimizationResult{};
  }

  const uint64_t idx = kf.index;

  if (first_keyframe_) {
    // 첫 번째 키프레임: prior 팩터만 추가
    add_prior_factors(kf);
    first_keyframe_ = false;
  } else {
    // 이후 키프레임: IMU 팩터 추가 + 초기값 삽입
    add_imu_factor(kf);
  }

  // GNSS 팩터 (있을 경우)
  if (gnss.has_value()) {
    add_gnss_factors(idx, gnss.value());
  }

  // NHC 팩터 (항상 적용)
  add_nhc_factor(idx);

  // ISAM2 업데이트
  const auto update_result1 = isam2_.update(new_factors_, new_values_);
  const auto update_result2 = isam2_.update();  // 추가 리니어화 패스
  new_factors_.resize(0);
  new_values_.clear();
  (void)update_result1;
  (void)update_result2;

  window_.push_back(idx);
  prev_index_ = idx;

  // 슬라이딩 윈도우 유지
  marginalize_oldest();

  // 결과 추출
  latest_result_ = extract_result(idx);
  return latest_result_;
}

}  // namespace hybrid_localization
