#ifndef HYBRID_LOCALIZATION__FGO__KEYFRAME_BUFFER_HPP_
#define HYBRID_LOCALIZATION__FGO__KEYFRAME_BUFFER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <rclcpp/time.hpp>

#include "hybrid_localization/eskf/eskf_core.hpp"
#include "hybrid_localization/fgo/imu_preintegration.hpp"
#include "hybrid_localization/types.hpp"

namespace hybrid_localization
{

// ---------------------------------------------------------------------------
// Keyframe: FGO 슬라이딩 윈도우의 단일 노드
// ---------------------------------------------------------------------------
struct Keyframe
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

  // ESKF 명목 상태 (키프레임 생성 시점의 스냅샷)
  NominalState state{};

  // 오차 상태 공분산 (15×15)
  EskfCore::P15 P{EskfCore::P15::Zero()};

  // 직전 키프레임에서 이 키프레임까지 IMU 사전적분
  ImuPreintegration preint{};

  // 키프레임 인덱스 (단조 증가)
  uint64_t index{0};
};

// ---------------------------------------------------------------------------
// KeyframeSelectionParams: 키프레임 생성 조건
// ---------------------------------------------------------------------------
struct KeyframeSelectionParams
{
  // 이동 거리 기준 [m]
  double min_dist_m{0.5};

  // 회전 변화 기준 [rad]
  double min_angle_rad{5.0 * M_PI / 180.0};  // 5°

  // 최소 시간 간격 [s] (최대 키프레임 빈도 = 1/min_interval_sec)
  double min_interval_sec{0.1};  // 최대 10 Hz

  // 최대 시간 간격 [s] (이 이상 경과 시 강제 키프레임 생성)
  double max_interval_sec{1.0};
};

// ---------------------------------------------------------------------------
// KeyframeBuffer: 스레드 안전 슬라이딩 윈도우 버퍼
// ---------------------------------------------------------------------------
class KeyframeBuffer
{
public:
  explicit KeyframeBuffer(size_t max_window_size = 20);

  // 최대 윈도우 크기 설정
  void set_max_size(size_t n);
  size_t max_size() const {return max_size_;}
  size_t size() const;
  bool empty() const;

  // 키프레임 추가 (윈도우 초과 시 가장 오래된 것 제거)
  void push(const Keyframe & kf);

  // 가장 오래된 키프레임 꺼내기
  std::optional<Keyframe> pop_oldest();

  // 최신 키프레임 참조 (없으면 nullopt)
  std::optional<Keyframe> latest() const;

  // 전체 윈도우 복사본 반환
  std::vector<Keyframe> get_window() const;

  // 버퍼 비우기
  void clear();

  // ---- 키프레임 선택 정책 --------------------------------------------------

  // 현재 ESKF 상태가 새 키프레임 생성 조건을 만족하는지 확인
  bool should_create_keyframe(
    const rclcpp::Time & stamp,
    const NominalState & state,
    const KeyframeSelectionParams & params) const;

private:
  mutable std::mutex mutex_;
  std::deque<Keyframe> buffer_;
  size_t max_size_;
  uint64_t next_index_{0};

  // 마지막 키프레임 정보 (선택 판단용, 잠금 없이 접근 불가)
  rclcpp::Time last_kf_stamp_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d last_kf_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond last_kf_q_{Eigen::Quaterniond::Identity()};
};

}  // namespace hybrid_localization

#endif  // HYBRID_LOCALIZATION__FGO__KEYFRAME_BUFFER_HPP_
