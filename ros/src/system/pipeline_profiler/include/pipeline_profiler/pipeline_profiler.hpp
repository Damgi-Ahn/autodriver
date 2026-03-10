#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <string>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Stage — pipeline stage IDs in temporal order.
// ---------------------------------------------------------------------------
enum class Stage : uint8_t {
  CAMERA_CAPTURE = 0,  ///< Frame exits GStreamer appsink
  IPC_SEND,            ///< DMABUF fd sent over Unix socket
  IPC_RECV,            ///< DMABUF fd received by inference_manager
  SCHEDULER,           ///< Frame enqueued into model queue
  BATCH_BUILD,         ///< Batch assembled, TRT input tensor filled
  INFERENCE,           ///< enqueueV3 + cudaStreamSynchronize complete
  PUBLISH,             ///< ROS 2 result topic published
  _COUNT
};

static constexpr size_t kNumStages = static_cast<size_t>(Stage::_COUNT);

// ---------------------------------------------------------------------------
// StageTimestamp — one per frame; ts[i] = ns when stage i completed.
// Zero means the stage was not reached.
// ---------------------------------------------------------------------------
struct StageTimestamp {
  uint64_t frame_id{0};
  uint64_t camera_id{0};
  std::array<uint64_t, kNumStages> ts{};
};

// ---------------------------------------------------------------------------
// PipelineProfiler — ROS 2 node
//
// Records inter-stage latency statistics.
// Call record() from any pipeline thread; it is lock-free.
// Publishes /system/profiler (Float32MultiArray) at publish_rate_hz.
//
// Topic layout:
//   data[2*i]   = avg latency gap[i] (µs), window-averaged
//   data[2*i+1] = max latency gap[i] (µs), window-maximum
//   where gap[i] = ts[i+1] - ts[i]  (kNumStages-1 gaps total)
//
// Accumulators reset every publish window (no cumulative drift).
// ---------------------------------------------------------------------------
class PipelineProfiler : public rclcpp::Node {
 public:
  explicit PipelineProfiler(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~PipelineProfiler() override = default;

  /// Record a StageTimestamp. Thread-safe, wait-free.
  void record(const StageTimestamp& ts);

  /// Current monotonic time in nanoseconds.
  static uint64_t now_ns();

  /// Human-readable stage name.
  static const char* stage_name(Stage s);

 private:
  void PublishStats();

  struct StageStat {
    std::atomic<uint64_t> count{0};
    std::atomic<uint64_t> sum_us{0};
    std::atomic<uint64_t> max_us{0};
  };

  std::array<StageStat, kNumStages - 1>                         gap_stats_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

}  // namespace autodriver::system
