#include "hybrid_localization_evaluation_tool/session_store.hpp"

namespace autodriver::tools {

SessionStore::SessionStore(size_t ring_size)
    : ring_size_(ring_size > 0 ? ring_size : kDefaultRingSize)
{
}

void SessionStore::AddSample(const DiagSample& sample)
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.push_back(sample);
  if (samples_.size() > ring_size_) samples_.pop_front();
}

void SessionStore::AddAlerts(const std::vector<AlertEvent>& events)
{
  if (events.empty()) return;
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& e : events) event_log_.push_back(e);
}

void SessionStore::AddGnssPoint(double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gnss_traj_.push_back({x, y});
  if (gnss_traj_.size() > ring_size_) gnss_traj_.pop_front();
}

void SessionStore::AddEskfPoint(double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  eskf_traj_.push_back({x, y});
  if (eskf_traj_.size() > ring_size_) eskf_traj_.pop_front();
}

void SessionStore::AddFgoPoint(double x, double y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fgo_traj_.push_back({x, y});
  if (fgo_traj_.size() > ring_size_) fgo_traj_.pop_front();
}

void SessionStore::SetSessionStart(const rclcpp::Time& t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  session_start_ = t;
}

std::vector<DiagSample> SessionStore::GetSampleHistory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {samples_.begin(), samples_.end()};
}

std::vector<AlertEvent> SessionStore::GetEventLog() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return event_log_;
}

std::vector<TrajectoryPoint> SessionStore::GetGnssTrajectory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {gnss_traj_.begin(), gnss_traj_.end()};
}

std::vector<TrajectoryPoint> SessionStore::GetEskfTrajectory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {eskf_traj_.begin(), eskf_traj_.end()};
}

std::vector<TrajectoryPoint> SessionStore::GetFgoTrajectory() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {fgo_traj_.begin(), fgo_traj_.end()};
}

rclcpp::Time SessionStore::session_start() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_start_;
}

void SessionStore::Clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  samples_.clear();
  event_log_.clear();
  gnss_traj_.clear();
  eskf_traj_.clear();
  fgo_traj_.clear();
}

}  // namespace autodriver::tools
