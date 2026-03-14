#pragma once

#include "hybrid_localization_evaluation_tool/alert_engine.hpp"
#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <deque>
#include <mutex>
#include <string>
#include <vector>

namespace autodriver::tools {

// ---------------------------------------------------------------------------
// Trajectory point for 2D pose plot
// ---------------------------------------------------------------------------
struct TrajectoryPoint {
  double x = 0.0;
  double y = 0.0;
};

// ---------------------------------------------------------------------------
// SessionStore
//   - Thread-safe ring buffer for DiagSample history (for charts)
//   - Event log (AlertEvent list)
//   - Trajectory history for GNSS / ESKF / FGO
// ---------------------------------------------------------------------------
class SessionStore {
 public:
  static constexpr size_t kDefaultRingSize = 600;  // 60 s @ 10 Hz

  explicit SessionStore(size_t ring_size = kDefaultRingSize);

  // Called from ROS thread
  void AddSample(const DiagSample& sample);
  void AddAlerts(const std::vector<AlertEvent>& events);
  void AddGnssPoint(double x, double y);
  void AddEskfPoint(double x, double y);
  void AddFgoPoint(double x, double y);
  void SetSessionStart(const rclcpp::Time& t);

  // Called from Qt thread (returns snapshot copy)
  std::vector<DiagSample>      GetSampleHistory() const;
  std::vector<AlertEvent>      GetEventLog() const;
  std::vector<TrajectoryPoint> GetGnssTrajectory() const;
  std::vector<TrajectoryPoint> GetEskfTrajectory() const;
  std::vector<TrajectoryPoint> GetFgoTrajectory() const;

  size_t ring_size() const { return ring_size_; }
  rclcpp::Time session_start() const;

  // Clear all stored data (for session reset)
  void Clear();

 private:
  mutable std::mutex mutex_;
  size_t ring_size_;
  std::deque<DiagSample>      samples_;
  std::vector<AlertEvent>     event_log_;
  std::deque<TrajectoryPoint> gnss_traj_;
  std::deque<TrajectoryPoint> eskf_traj_;
  std::deque<TrajectoryPoint> fgo_traj_;
  rclcpp::Time session_start_;
};

}  // namespace autodriver::tools
