#include "pipeline_profiler/pipeline_profiler.hpp"

#include <time.h>
#include <algorithm>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Stage name table
// ---------------------------------------------------------------------------

const char* PipelineProfiler::stage_name(Stage s)
{
  static constexpr const char* kNames[] = {
    "camera_capture",
    "ipc_send",
    "ipc_recv",
    "scheduler",
    "batch_build",
    "inference",
    "publish"
  };
  const auto idx = static_cast<size_t>(s);
  return (idx < kNumStages) ? kNames[idx] : "unknown";
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

PipelineProfiler::PipelineProfiler(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pipeline_profiler", options)
{
  declare_parameter("publish_rate_hz", 1.0);
  const double rate_hz = get_parameter("publish_rate_hz").as_double();
  const auto   period  = std::chrono::duration<double>(1.0 / rate_hz);

  pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
      "/system/profiler", rclcpp::QoS{10});

  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this] { PublishStats(); });

  RCLCPP_INFO(get_logger(),
              "PipelineProfiler started — %zu inter-stage gaps, publish=%.1f Hz",
              kNumStages - 1, rate_hz);
}

// ---------------------------------------------------------------------------
// Record — called from any pipeline thread (camera, IPC, drain, publish)
// ---------------------------------------------------------------------------

void PipelineProfiler::record(const StageTimestamp& ts)
{
  for (size_t i = 0; i + 1 < kNumStages; ++i) {
    if (ts.ts[i] == 0 || ts.ts[i + 1] == 0) continue;

    const uint64_t gap_us = (ts.ts[i + 1] - ts.ts[i]) / 1000;
    auto& stat = gap_stats_[i];

    stat.count.fetch_add(1, std::memory_order_relaxed);
    stat.sum_us.fetch_add(gap_us, std::memory_order_relaxed);

    // Atomic max via CAS loop — avoids data race on max_us.
    uint64_t prev = stat.max_us.load(std::memory_order_relaxed);
    while (gap_us > prev) {
      if (stat.max_us.compare_exchange_weak(
              prev, gap_us,
              std::memory_order_relaxed,
              std::memory_order_relaxed))
        break;
    }
  }
}

// ---------------------------------------------------------------------------
// now_ns
// ---------------------------------------------------------------------------

uint64_t PipelineProfiler::now_ns()
{
  struct timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec)  * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

// ---------------------------------------------------------------------------
// PublishStats — called from 1 Hz timer
//
// Layout: [avg_us_gap0, max_us_gap0, avg_us_gap1, max_us_gap1, ...]
// One pair per inter-stage gap (kNumStages - 1 pairs total).
// Accumulators are reset after each publish window.
// ---------------------------------------------------------------------------

void PipelineProfiler::PublishStats()
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve((kNumStages - 1) * 2);

  for (size_t i = 0; i + 1 < kNumStages; ++i) {
    auto& stat = gap_stats_[i];

    // Load and atomically reset each field.
    const uint64_t n   = stat.count.exchange(0, std::memory_order_relaxed);
    const uint64_t sum = stat.sum_us.exchange(0, std::memory_order_relaxed);
    const uint64_t max = stat.max_us.exchange(0, std::memory_order_relaxed);

    const float avg = (n > 0)
                      ? static_cast<float>(sum) / static_cast<float>(n)
                      : 0.0f;

    msg.data.push_back(avg);
    msg.data.push_back(static_cast<float>(max));

    if (n > 0) {
      RCLCPP_DEBUG(get_logger(),
                   "  %-16s → %-16s  avg=%.1f µs  max=%.1f µs  n=%lu",
                   stage_name(static_cast<Stage>(i)),
                   stage_name(static_cast<Stage>(i + 1)),
                   avg, static_cast<float>(max), n);
    }
  }

  pub_->publish(msg);
}

}  // namespace autodriver::system
