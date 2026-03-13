#include "hybrid_localization/fgo/imu_buffer.hpp"

namespace hybrid_localization
{

ImuBuffer::ImuBuffer(size_t max_samples)
: max_size_(max_samples)
{}

void ImuBuffer::set_max_size(size_t n)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  max_size_ = n;
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
  }
}

size_t ImuBuffer::size() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return buffer_.size();
}

bool ImuBuffer::empty() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  return buffer_.empty();
}

void ImuBuffer::push(const ImuMeasurement & meas)
{
  std::scoped_lock<std::mutex> lock(mutex_);
  buffer_.push_back(meas);
  while (buffer_.size() > max_size_) {
    buffer_.pop_front();
    overflow_count_.fetch_add(1u, std::memory_order_relaxed);
  }
}

size_t ImuBuffer::take_overflow_count()
{
  return overflow_count_.exchange(0u, std::memory_order_relaxed);
}

std::vector<ImuMeasurement> ImuBuffer::get_since(double stamp_sec) const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  std::vector<ImuMeasurement> result;
  result.reserve(16);
  for (const auto & meas : buffer_) {
    if (meas.stamp_sec > stamp_sec) {
      result.push_back(meas);
    }
  }
  return result;
}

void ImuBuffer::clear()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  buffer_.clear();
}

double ImuBuffer::oldest_stamp_sec() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return -1.0;
  }
  return buffer_.front().stamp_sec;
}

double ImuBuffer::latest_stamp_sec() const
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (buffer_.empty()) {
    return -1.0;
  }
  return buffer_.back().stamp_sec;
}

}  // namespace hybrid_localization
