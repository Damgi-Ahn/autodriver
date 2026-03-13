#ifndef HYBRID_LOCALIZATION__FGO__IMU_BUFFER_HPP_
#define HYBRID_LOCALIZATION__FGO__IMU_BUFFER_HPP_

#include <deque>
#include <mutex>
#include <vector>

#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// ImuBuffer
//
// EskfCorrector 의 재적분(reintegration)을 위한 IMU 원형 버퍼.
//
// 저장 내용: `ImuMeasurement` (stamp, gyro_preprocessed, accel_b_a, dt)
//   - gyro_radps : ESKF.propagate() 에 실제로 전달된 전처리 각속도
//                  (LPF + 좌표변환 완료, ESKF bias는 내부 처리)
//   - accel_mps2 : 해당 시점의 eskf.b_a() (zero-accel 모드)
//   - dt         : 이전 샘플 대비 경과 시간 [s]
//
// EskfCorrector 는 keyframe 이후 구간의 IMU 를 꺼내
// set_nominal_state() + propagate() 재적분에 사용한다.
// ---------------------------------------------------------------------------
class ImuBuffer
{
public:
  // max_samples: 최대 저장 샘플 수 (200Hz × 2s = 400)
  explicit ImuBuffer(size_t max_samples = 400);

  void set_max_size(size_t n);
  size_t max_size() const {return max_size_;}
  size_t size() const;
  bool empty() const;

  // 샘플 추가 (최대치 초과 시 가장 오래된 샘플 제거)
  void push(const ImuMeasurement & meas);

  // stamp_sec 이후의 모든 샘플 복사본 반환
  std::vector<ImuMeasurement> get_since(double stamp_sec) const;

  // 전체 버퍼 비우기
  void clear();

  // 가장 오래된 샘플의 타임스탬프 (없으면 -1)
  double oldest_stamp_sec() const;

  // 가장 최신 샘플의 타임스탬프 (없으면 -1)
  double latest_stamp_sec() const;

private:
  mutable std::mutex mutex_;
  std::deque<ImuMeasurement> buffer_;
  size_t max_size_;
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__IMU_BUFFER_HPP_
