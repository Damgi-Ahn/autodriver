#ifndef HYBRID_LOCALIZATION__FGO__ESKF_CORRECTOR_HPP_
#define HYBRID_LOCALIZATION__FGO__ESKF_CORRECTOR_HPP_

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/fgo/fgo_backend.hpp"
#include "hybrid_localization/fgo/imu_buffer.hpp"
#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// EskfCorrector
//
// FGO 최적화 결과를 ESKF 명목 상태에 앵커 보정(anchor correction)하는 모듈.
//
// 보정 절차:
//   1. ESKF 명목 상태를 FGO 최적화 결과(keyframe 시점)로 교체
//   2. keyframe 이후 저장된 IMU 를 사용해 현재 시점까지 재적분
//
// 동기(synchronous) 운용 시:
//   - maybe_push_keyframe() 내에서 즉시 호출
//   - ImuBuffer 에 keyframe 이후 샘플이 0개이므로 재적분 없이 상태만 교체
//   - 이후 도착하는 IMU 는 FGO-corrected 상태에서 propagate 됨
//
// 비동기(async) 운용 시 (미래 Stage 5 확장):
//   - FGO 가 별도 스레드에서 수행
//   - apply() 에서 ImuBuffer 의 reintegration 경로가 활성화됨
// ---------------------------------------------------------------------------
class EskfCorrector
{
public:
  struct Params
  {
    // reintegration 창 최대 길이 [s].
    // ImuBuffer 내에서 이 이상 오래된 샘플은 무시.
    double max_reintegration_window_sec{2.0};

    // FGO 보정 활성화 여부
    bool enabled{true};
  };

  EskfCorrector() = default;
  explicit EskfCorrector(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  // ---------------------------------------------------------------------------
  // apply() — 핵심 보정 메서드
  //
  // @param eskf        : 수정 대상 ESKF (호출자가 mutex 보유)
  // @param result      : FGO 최적화 결과 (keyframe 시점의 상태)
  // @param kf_stamp_sec: 해당 keyframe 의 타임스탬프 [s]
  // @param imu_buffer  : keyframe 이후 저장된 IMU (재적분용)
  // @return            : 실제 재적분된 샘플 수 (0 이면 상태 교체만 수행)
  // ---------------------------------------------------------------------------
  int apply(
    EskfCore & eskf,
    const FgoBackend::OptimizationResult & result,
    double kf_stamp_sec,
    const ImuBuffer & imu_buffer);

  // 마지막 보정 적용 시각 [s] (미적용 시 -1)
  double last_correction_stamp_sec() const {return last_correction_stamp_sec_;}

  // 마지막 보정에서 재적분된 샘플 수
  int last_reintegration_count() const {return last_reintegration_count_;}

private:
  Params params_{};
  double last_correction_stamp_sec_{-1.0};
  int last_reintegration_count_{0};
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__ESKF_CORRECTOR_HPP_
