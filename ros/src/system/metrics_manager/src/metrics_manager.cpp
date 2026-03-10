#include "metrics_manager/metrics_manager.hpp"

#include <fstream>
#include <sstream>

namespace autodriver::system {

MetricsManager::MetricsManager(const rclcpp::NodeOptions& options)
    : rclcpp::Node("metrics_manager", options)
{
    declare_parameter("publish_rate_hz", 1.0);
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

    pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/system/metrics", 10);

    const auto period_ms = static_cast<int>(1000.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        [this]{ collect_and_publish(); });
}

void MetricsManager::update_camera_metrics(const CameraMetrics& m)
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    camera_metrics_[m.camera_id] = m;
}

void MetricsManager::update_model_metrics(const ModelMetrics& m)
{
    std::lock_guard<std::mutex> lock(metrics_mutex_);
    model_metrics_[m.model_name] = m;
}

void MetricsManager::collect_and_publish()
{
    auto gpu = read_gpu_metrics();

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = now();

    // ── GPU status ────────────────────────────────────────────────────────
    {
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name    = "GPU";
        status.hardware_id = "jetson_orin_gpu";
        status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;

        auto kv = [](const std::string& k, const std::string& v) {
            diagnostic_msgs::msg::KeyValue kval;
            kval.key   = k;
            kval.value = v;
            return kval;
        };
        status.values.push_back(kv("utilization_pct", std::to_string(gpu.gpu_utilization_pct)));
        status.values.push_back(kv("memory_used_mb",  std::to_string(gpu.gpu_memory_used_mb)));
        status.values.push_back(kv("memory_total_mb", std::to_string(gpu.gpu_memory_total_mb)));
        status.values.push_back(kv("temperature_c",   std::to_string(gpu.temperature_c)));
        msg.status.push_back(status);
    }

    // ── Camera metrics ────────────────────────────────────────────────────
    {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        for (const auto& [id, m] : camera_metrics_) {
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = "Camera/" + std::to_string(id);
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "fps";  kv.value = std::to_string(m.fps);
            status.values.push_back(kv);
            kv.key = "frames_dropped"; kv.value = std::to_string(m.frames_dropped);
            status.values.push_back(kv);
            msg.status.push_back(status);
        }

        for (const auto& [name, m] : model_metrics_) {
            diagnostic_msgs::msg::DiagnosticStatus status;
            status.name = "Model/" + name;
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;

            diagnostic_msgs::msg::KeyValue kv;
            kv.key = "avg_latency_ms"; kv.value = std::to_string(m.avg_latency_ms);
            status.values.push_back(kv);
            kv.key = "avg_batch_size"; kv.value = std::to_string(m.avg_batch_size);
            status.values.push_back(kv);
            msg.status.push_back(status);
        }
    }

    pub_->publish(msg);
}

GpuMetrics MetricsManager::read_gpu_metrics()
{
    GpuMetrics m{};

    // ── GPU load from sysfs ────────────────────────────────────────────────
    {
        std::ifstream f(kGpuFreqPath);
        if (f) { f >> m.gpu_utilization_pct; }
    }

    // ── Temperature ───────────────────────────────────────────────────────
    {
        std::ifstream f(kGpuTempPath);
        if (f) {
            int milli_c = 0;
            f >> milli_c;
            m.temperature_c = static_cast<float>(milli_c) / 1000.0f;
        }
    }

    // ── GPU memory via cudaMemGetInfo ─────────────────────────────────────
    size_t free_bytes = 0, total_bytes = 0;
    if (cudaMemGetInfo(&free_bytes, &total_bytes) == cudaSuccess) {
        m.gpu_memory_total_mb = static_cast<float>(total_bytes) / (1024.0f * 1024.0f);
        m.gpu_memory_used_mb  = m.gpu_memory_total_mb -
                                static_cast<float>(free_bytes) / (1024.0f * 1024.0f);
    }

    return m;
}

} // namespace autodriver::system
