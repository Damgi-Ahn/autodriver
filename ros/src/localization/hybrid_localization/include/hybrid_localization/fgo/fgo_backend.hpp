#ifndef HYBRID_LOCALIZATION__FGO__FGO_BACKEND_HPP_
#define HYBRID_LOCALIZATION__FGO__FGO_BACKEND_HPP_

#include <deque>
#include <memory>
#include <optional>

#include <boost/shared_ptr.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "hybrid_localization/fgo/imu_preintegration.hpp"
#include "hybrid_localization/fgo/keyframe_buffer.hpp"
#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// FgoBackend
//
// GTSAM ISAM2 기반 슬라이딩 윈도우 FGO 백엔드.
//
// 변수 per keyframe i:
//   X(i) = gtsam::Pose3     — 위치 + 자세 (map 프레임)
//   V(i) = gtsam::Vector3   — 속도       (map 프레임)
//   B(i) = imuBias::ConstantBias — IMU 바이어스 (gyro, accel)
//
// 팩터:
//   CombinedImuFactor(Xi, Vi, Bi, Xj, Vj, Bj)  — IMU 사전적분
//   GPSFactor(Xj)                               — GNSS 위치
//   GnssVelocityFactor(Vj)                      — GNSS 속도
//   GnssHeadingFactor(Xj)                       — GNSS/GPHDT 헤딩
//   NhcFactor(Xj, Vj)                           — Non-Holonomic 제약
//   PriorFactor<Bias>(Bj)                       — 바이어스 랜덤 워크
// ---------------------------------------------------------------------------
class FgoBackend
{
public:
  // ----- 파라미터 ----------------------------------------------------------
  struct Params
  {
    // 슬라이딩 윈도우 크기 (keyframe 수)
    int window_size{20};

    // Prior 노이즈 (첫 키프레임)
    double prior_pos_noise_m{0.1};          // [m]
    double prior_yaw_noise_rad{0.1};        // [rad]
    double prior_vel_noise_mps{0.1};        // [m/s]
    double prior_bias_gyro_noise{1e-3};     // [rad/s]
    double prior_bias_accel_noise{1e-2};    // [m/s^2]

    // 바이어스 랜덤 워크 (CombinedImuFactor 내 biasAccCovariance 등과 독립)
    double bias_gyro_rw_noise{1e-5};        // [rad/s/√s]
    double bias_accel_rw_noise{1e-4};       // [m/s²/√s]

    // NHC 노이즈
    double nhc_lat_noise_mps{0.05};         // [m/s] lateral
    double nhc_vert_noise_mps{0.1};         // [m/s] vertical

    // GNSS 최소 status (이 이상이어야 FGO 위치/속도 팩터 사용)
    int gnss_min_status_for_pos{0};         // 0=fix
    int gnss_min_status_for_vel{0};

    // 적응형 윈도우 크기 설정
    bool adaptive_window_enable{false};
    int  adaptive_window_max{40};   // GNSS 불량 시 확장되는 최대 윈도우 크기

    // ISAM2 최적화 파라미터
    // `fgo.backend.isam2_relinearize_threshold` [-]
    // - 재선형화 트리거 임계값(변수 delta). ↓: 더 자주 재선형화(정확↑, 비용↑)
    double isam2_relinearize_threshold{0.1};

    // `fgo.backend.isam2_relinearize_skip` [-]
    // - 매 N번째 업데이트마다 재선형화 체크. 1=매 업데이트마다 체크.
    int isam2_relinearize_skip{1};

    // `fgo.backend.isam2_integration_cov` [-]
    // - IMU 수치 적분 잡음 공분산(I_3x3 스케일). ↑: 적분 불확실성 증가
    double isam2_integration_cov{1e-8};

    // `fgo.backend.isam2_bias_acc_omega_int` [-]
    // - 바이어스 적분 잡음 공분산(I_6x6 스케일). ↑: 바이어스 추적 불확실성 증가
    double isam2_bias_acc_omega_int{1e-5};
  };

  // ----- 최적화 결과 -------------------------------------------------------
  struct OptimizationResult
  {
    bool valid{false};
    NominalState state{};       // 최적화된 최신 키프레임 상태
    Eigen::Matrix<double, 15, 15> P{Eigen::Matrix<double, 15, 15>::Zero()};
    uint64_t keyframe_index{0};
    double cost_before{0.0};
    double cost_after{0.0};
  };

  // ----- 생명 주기 --------------------------------------------------------
  FgoBackend() = default;

  // ISAM2 및 노이즈 모델 초기화
  void initialize(
    const Params & params,
    const ImuPreintegration::Params & imu_params);

  bool is_initialized() const {return initialized_;}

  // 런타임 윈도우 크기 변경 (적응형 윈도우 제어용; [1, adaptive_window_max] 범위로 클램프)
  void set_window_size(int n);

  // ----- 키프레임 업데이트 ------------------------------------------------

  // 새 키프레임을 FGO 그래프에 추가하고 최적화 수행.
  // keyframe.gtsam_preint == null 이면 첫 번째 키프레임으로 처리.
  OptimizationResult update(
    const Keyframe & kf,
    const std::optional<GnssMeasurement> & gnss);

  // 최신 최적화 결과 (업데이트 전에는 invalid)
  const OptimizationResult & latest_result() const {return latest_result_;}

  // GTSAM PreintegratedCombinedMeasurements 파라미터 (노드에서 공유 사용)
  boost::shared_ptr<gtsam::PreintegrationCombinedParams> gtsam_preint_params() const
  {
    return gtsam_preint_params_;
  }

private:
  // ----- 내부 상태 --------------------------------------------------------
  bool initialized_{false};
  Params params_{};
  ImuPreintegration::Params imu_params_{};

  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;

  // 슬라이딩 윈도우: 활성 키프레임 인덱스 (oldest → newest)
  std::deque<uint64_t> window_;
  bool first_keyframe_{true};
  uint64_t prev_index_{0};

  OptimizationResult latest_result_{};

  // GTSAM 노이즈 모델 (초기화 시 생성)
  gtsam::SharedNoiseModel prior_pose_noise_;
  gtsam::SharedNoiseModel prior_vel_noise_;
  gtsam::SharedNoiseModel prior_bias_noise_;
  gtsam::SharedNoiseModel nhc_noise_;
  gtsam::SharedNoiseModel bias_rw_noise_;

  boost::shared_ptr<gtsam::PreintegrationCombinedParams> gtsam_preint_params_;

  // ----- 내부 헬퍼 --------------------------------------------------------
  gtsam::Symbol x_sym(uint64_t i) const {return gtsam::Symbol('x', i);}
  gtsam::Symbol v_sym(uint64_t i) const {return gtsam::Symbol('v', i);}
  gtsam::Symbol b_sym(uint64_t i) const {return gtsam::Symbol('b', i);}

  void add_prior_factors(const Keyframe & kf);
  void add_imu_factor(const Keyframe & kf);
  void add_gnss_factors(uint64_t idx, const GnssMeasurement & gnss);
  void add_nhc_factor(uint64_t idx);
  void marginalize_oldest();
  OptimizationResult extract_result(uint64_t idx) const;
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__FGO_BACKEND_HPP_
