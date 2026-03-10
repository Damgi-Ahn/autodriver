#pragma once

#include "model_runner.hpp"
#include "nvbuf_pool.hpp"
#include "scheduler.hpp"

#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// InferenceManager — ROS 2 lifecycle node for multi-model TRT inference.
//
// IPC role: SERVER (binds Unix socket; camera_manager connects as client).
//
// Startup sequence:
//   1. LoadConfigs()     — parse models.yaml + system.yaml
//   2. InitCudaStreams() — create N CUDA streams
//   3. InitNvBufPool()   — pre-allocate NvBufSurface pool
//   4. InitModelRunners()— deserialise TRT engines, alloc device buffers
//   5. InitScheduler()   — register queues, start per-model drain threads
//   6. BindIpcServer()   — CreateServerSocket (non-blocking bind+listen)
//   7. init_thread_      — AcceptConnection (blocks until camera_manager)
//                        → then runs IpcReceiveLoop
//
// Shutdown sequence (destructor):
//   - ipc_running_ = false; close peer_fd_ → IpcReceiveLoop exits
//   - Join init_thread_
//   - scheduler_->Stop()
//   - Destroy CUDA streams; CloseServerSocket
// ---------------------------------------------------------------------------
class InferenceManager : public rclcpp::Node {
 public:
  explicit InferenceManager(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~InferenceManager() override;

 private:
  void LoadConfigs();
  void InitCudaStreams();
  void InitNvBufPool();
  void InitModelRunners();
  void InitScheduler();
  void BindIpcServer();

  void AcceptAndReceive();   ///< Runs in init_thread_
  void IpcReceiveLoop();     ///< Called by AcceptAndReceive after accept

  void OnInferenceResult(InferenceResult result);
  void PublishResult(const InferenceResult& result);

  void Shutdown();

  // ── Config ───────────────────────────────────────────────────────────────
  std::vector<ModelConfig> model_configs_;
  uint32_t                 num_cuda_streams_{8};
  uint32_t                 nvbuf_pool_size_{128};
  std::string              ipc_socket_path_{"/tmp/autodriver/frames.sock"};

  // ── GPU resources ────────────────────────────────────────────────────────
  std::vector<cudaStream_t>                        cuda_streams_;
  std::unique_ptr<NvBufSurfacePool>                nvbuf_pool_;
  std::unordered_map<std::string,
                     std::unique_ptr<ModelRunner>> model_runners_;
  std::unique_ptr<HybridScheduler>                 scheduler_;

  // ── IPC ──────────────────────────────────────────────────────────────────
  int                server_fd_{-1};   ///< Bound listening fd
  int                peer_fd_{-1};     ///< Accepted camera_manager fd
  std::atomic<bool>  ipc_running_{false};
  std::thread        init_thread_;

  // ── ROS 2 publishers ─────────────────────────────────────────────────────
  std::unordered_map<std::string,
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> result_pubs_;
};

}  // namespace autodriver::inference
