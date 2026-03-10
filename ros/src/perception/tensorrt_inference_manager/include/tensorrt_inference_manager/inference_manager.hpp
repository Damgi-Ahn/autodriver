#pragma once

#include "model_runner.hpp"
#include "nvbuf_pool.hpp"
#include "scheduler.hpp"

#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// InferenceManager — ROS2 node
//
// Responsibilities:
//   1. Load models.yaml + system.yaml
//   2. Initialise NvBufSurfacePool
//   3. Initialise CUDA streams
//   4. Construct one ModelRunner per model
//   5. Construct HybridScheduler, register model queues
//   6. Connect to camera_manager via Unix socket
//   7. Run IPC receiver loop (dedicated thread)
//   8. Forward batches from HybridScheduler to ModelRunner
//   9. Publish inference results via ROS2 topics
// ---------------------------------------------------------------------------
class InferenceManager : public rclcpp::Node {
public:
    explicit InferenceManager(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~InferenceManager() override;

private:
    // ── Initialisation ──────────────────────────────────────────────────────
    void load_configs();
    void init_cuda_streams();
    void init_nvbuf_pool();
    void init_model_runners();
    void init_scheduler();
    void connect_ipc();

    // ── IPC receiver loop ────────────────────────────────────────────────────
    void ipc_receive_loop();

    // ── Inference result handling ─────────────────────────────────────────────
    void on_inference_result(InferenceResult result);

    // ── ROS2 publish helpers ─────────────────────────────────────────────────
    void publish_detections(const InferenceResult& result);
    void publish_lanes(const InferenceResult& result);
    void publish_segmentation(const InferenceResult& result);

    // ── Shutdown ─────────────────────────────────────────────────────────────
    void shutdown();

    // ── Config ───────────────────────────────────────────────────────────────
    std::vector<ModelConfig> model_configs_;
    uint32_t                 num_cuda_streams_{8};
    uint32_t                 nvbuf_pool_size_{128};
    std::string              ipc_socket_path_{"/tmp/autodriver/frames.sock"};

    // ── GPU resources ────────────────────────────────────────────────────────
    std::vector<cudaStream_t>                              cuda_streams_;
    std::unique_ptr<NvBufSurfacePool>                      nvbuf_pool_;
    std::unordered_map<std::string,
                       std::unique_ptr<ModelRunner>>       model_runners_;
    std::unique_ptr<HybridScheduler>                       scheduler_;

    // ── IPC ──────────────────────────────────────────────────────────────────
    int           ipc_fd_{-1};
    std::thread   ipc_thread_;
    std::atomic<bool> ipc_running_{false};

    // ── ROS2 publishers (per model) ───────────────────────────────────────────
    std::unordered_map<std::string,
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> result_pubs_;
};

} // namespace autodriver::inference
