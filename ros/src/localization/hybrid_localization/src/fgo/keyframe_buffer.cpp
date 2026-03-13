#include "hybrid_localization/fgo/keyframe_buffer.hpp"

#include <cmath>

namespace hybrid_localization
{

KeyframeBuffer::KeyframeBuffer(size_t max_window_size)
: max_size_(max_window_size) {}

void KeyframeBuffer::set_max_size(size_t n)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  max_size_ = n;
}

size_t KeyframeBuffer::size() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return buffer_.size();
}

bool KeyframeBuffer::empty() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return buffer_.empty();
}

void KeyframeBuffer::push(const Keyframe & kf)
{
  std::scoped_lock<std::mutex> lock(mutex_);

  // 인덱스 할당
  Keyframe kf_indexed = kf;
  kf_indexed.index = next_index_++;

  buffer_.push_back(kf_indexed);

  // 최신 키프레임 정보 갱신 (선택 판단용)
  last_kf_stamp_ = kf_indexed.stamp;
  last_kf_pos_ = kf_indexed.state.p_map;
  last_kf_q_ = kf_indexed.state.q_map_from_base;

  // 윈도우 초과 시 가장 오래된 키프레임 제거
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

std::optional<Keyframe> KeyframeBuffer::pop_oldest()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return std::nullopt;
  }
  Keyframe kf = buffer_.front();
  buffer_.pop_front();
  return kf;
}

std::optional<Keyframe> KeyframeBuffer::latest() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return std::nullopt;
  }
  return buffer_.back();
}

std::vector<Keyframe> KeyframeBuffer::get_window() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return std::vector<Keyframe>(buffer_.begin(), buffer_.end());
}

void KeyframeBuffer::clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  buffer_.clear();
}

bool KeyframeBuffer::should_create_keyframe(
  const rclcpp::Time & stamp,
  const NominalState & state,
  const KeyframeSelectionParams & params) const
{
  std::scoped_lock<std::mutex> lock(mutex_);

  // 버퍼가 비어있으면 무조건 생성
  if (buffer_.empty()) {
    return true;
  }

  const double dt = (stamp - last_kf_stamp_).seconds();

  // 최소 간격 미달: 스킵
  if (dt < params.min_interval_sec) {
    return false;
  }

  // 최대 간격 초과: 강제 생성
  if (dt >= params.max_interval_sec) {
    return true;
  }

  // 이동 거리 조건
  const double dist = (state.p_map - last_kf_pos_).norm();
  if (dist >= params.min_dist_m) {
    return true;
  }

  // 회전 변화 조건
  // |Log(q_last^{-1} * q_curr)| ≥ min_angle_rad
  const Eigen::Quaterniond dq = last_kf_q_.inverse() * state.q_map_from_base;
  const double dangle = 2.0 * std::acos(std::abs(std::clamp(dq.w(), -1.0, 1.0)));
  if (dangle >= params.min_angle_rad) {
    return true;
  }

  return false;
}

}  // namespace hybrid_localization
