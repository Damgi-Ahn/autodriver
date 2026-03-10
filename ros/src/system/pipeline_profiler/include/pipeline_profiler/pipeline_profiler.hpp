#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Pipeline stages — latency is measured between consecutive stages.
// ---------------------------------------------------------------------------
enum class Stage : uint8_t {
    CAMERA_CAPTURE = 0,  ///< Frame exits GStreamer appsink
    IPC_SEND,            ///< DMABUF fd sent over Unix socket
    IPC_RECV,            ///< DMABUF fd received by inference_manager
    SCHEDULER,           ///< Frame enqueued into model queue
    BATCH_BUILD,         ///< Batch assembled, TRT input tensor filled
    INFERENCE,           ///< enqueueV3 + cudaStreamSynchronize complete
    PUBLISH,             ///< ROS2 result topic published
    _COUNT
};

static constexpr size_t kNumStages = static_cast<size_t>(Stage::_COUNT);

// ---------------------------------------------------------------------------
// StageTimestamp — attached to each frame across the pipeline.
// Passed by value (cheap) in FrameMeta extensions or as side-channel.
// ---------------------------------------------------------------------------
struct StageTimestamp {
    uint64_t frame_id{0};
    uint64_t camera_id{0};
    std::array<uint64_t, kNumStages> ts{};  ///< nanoseconds per stage
};

// ---------------------------------------------------------------------------
// PipelineProfiler — ROS2 node
//
// Collects StageTimestamps from all pipeline components via a shared
// lock-free ring buffer, computes per-stage latency statistics, and
// publishes them on /system/profiler at 1 Hz.
// ---------------------------------------------------------------------------
class PipelineProfiler : public rclcpp::Node {
public:
    explicit PipelineProfiler(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~PipelineProfiler() override = default;

    /// Record a completed StageTimestamp. Thread-safe, non-blocking.
    void record(const StageTimestamp& ts);

    /// Convenience: get current monotonic time in nanoseconds.
    static uint64_t now_ns();

    static const char* stage_name(Stage s);

private:
    void publish_stats();

    // Rolling statistics per inter-stage gap
    struct StageStat {
        std::atomic<uint64_t> count{0};
        std::atomic<uint64_t> sum_us{0};   ///< Microseconds
        std::atomic<uint64_t> max_us{0};
    };
    // Gap[i] = ts[i+1] - ts[i]
    std::array<StageStat, kNumStages - 1> gap_stats_;

    rclcpp::TimerBase::SharedPtr                                  timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
};

} // namespace autodriver::system
