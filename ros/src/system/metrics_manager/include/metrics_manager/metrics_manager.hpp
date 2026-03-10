#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <atomic>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace autodriver::system {

// ---------------------------------------------------------------------------
// Per-camera metrics snapshot
// ---------------------------------------------------------------------------
struct CameraMetrics {
    uint32_t camera_id{0};
    float    fps{0.0f};
    uint64_t frames_captured{0};
    uint64_t frames_dropped{0};
};

// ---------------------------------------------------------------------------
// Per-model inference metrics snapshot
// ---------------------------------------------------------------------------
struct ModelMetrics {
    std::string model_name;
    float       avg_latency_ms{0.0f};
    float       avg_batch_size{0.0f};
    uint64_t    batches_run{0};
};

// ---------------------------------------------------------------------------
// GPU / system metrics
// ---------------------------------------------------------------------------
struct GpuMetrics {
    float gpu_utilization_pct{0.0f};  ///< From sysfs GR3D_FREQ
    float gpu_memory_used_mb{0.0f};
    float gpu_memory_total_mb{0.0f};
    float temperature_c{0.0f};        ///< GPU thermal zone
};

// ---------------------------------------------------------------------------
// MetricsManager — ROS2 node
//
// Aggregates camera, model, and GPU metrics, publishing them at 1 Hz
// on /system/metrics as a DiagnosticArray.
// External modules update metrics by calling the thread-safe update_* methods.
// ---------------------------------------------------------------------------
class MetricsManager : public rclcpp::Node {
public:
    explicit MetricsManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~MetricsManager() override = default;

    // ── Thread-safe metric update API (called from other nodes via shared ptr)
    void update_camera_metrics(const CameraMetrics& m);
    void update_model_metrics(const ModelMetrics& m);

private:
    void collect_and_publish();
    GpuMetrics read_gpu_metrics();

    // sysfs / proc paths (Jetson Orin)
    static constexpr const char* kGpuFreqPath =
        "/sys/devices/gpu.0/load";
    static constexpr const char* kGpuTempPath =
        "/sys/class/thermal/thermal_zone0/temp";
    static constexpr const char* kGpuMemPath =
        "/proc/meminfo";

    rclcpp::TimerBase::SharedPtr                      timer_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_;

    // Protected by mutex
    std::mutex                                         metrics_mutex_;
    std::unordered_map<uint32_t, CameraMetrics>        camera_metrics_;
    std::unordered_map<std::string, ModelMetrics>      model_metrics_;

    float publish_rate_hz_{1.0f};
};

} // namespace autodriver::system
