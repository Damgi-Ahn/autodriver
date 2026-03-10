#include "pipeline_profiler/pipeline_profiler.hpp"

#include <time.h>
#include <algorithm>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Stage name table
// ---------------------------------------------------------------------------

const char* PipelineProfiler::stage_name(Stage s)
{
    static constexpr const char* names[] = {
        "camera_capture",
        "ipc_send",
        "ipc_recv",
        "scheduler",
        "batch_build",
        "inference",
        "publish"
    };
    const auto idx = static_cast<size_t>(s);
    return (idx < kNumStages) ? names[idx] : "unknown";
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

PipelineProfiler::PipelineProfiler(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pipeline_profiler", options)
{
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/system/profiler", 10);

    timer_ = create_wall_timer(
        std::chrono::seconds(1),
        [this]{ publish_stats(); });
}

// ---------------------------------------------------------------------------
// Record
// ---------------------------------------------------------------------------

void PipelineProfiler::record(const StageTimestamp& ts)
{
    for (size_t i = 0; i + 1 < kNumStages; ++i) {
        if (ts.ts[i] == 0 || ts.ts[i + 1] == 0) continue;

        const uint64_t gap_us = (ts.ts[i + 1] - ts.ts[i]) / 1000;
        auto& stat = gap_stats_[i];

        stat.count.fetch_add(1, std::memory_order_relaxed);
        stat.sum_us.fetch_add(gap_us, std::memory_order_relaxed);

        // Update max (approximate — no atomic CAS for simplicity)
        uint64_t old_max = stat.max_us.load(std::memory_order_relaxed);
        if (gap_us > old_max)
            stat.max_us.store(gap_us, std::memory_order_relaxed);
    }
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

uint64_t PipelineProfiler::now_ns()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

// ---------------------------------------------------------------------------
// Publish
// ---------------------------------------------------------------------------

void PipelineProfiler::publish_stats()
{
    // Layout: [avg_us, max_us] × (kNumStages - 1) inter-stage gaps
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve((kNumStages - 1) * 2);

    for (size_t i = 0; i + 1 < kNumStages; ++i) {
        auto& stat = gap_stats_[i];
        const uint64_t n = stat.count.load();
        const float avg = (n > 0)
            ? static_cast<float>(stat.sum_us.load()) / static_cast<float>(n)
            : 0.0f;
        const float max = static_cast<float>(stat.max_us.load());

        msg.data.push_back(avg);
        msg.data.push_back(max);

        // Reset accumulators for next window
        stat.count.store(0);
        stat.sum_us.store(0);
        stat.max_us.store(0);
    }

    pub_->publish(msg);
}

} // namespace autodriver::system
