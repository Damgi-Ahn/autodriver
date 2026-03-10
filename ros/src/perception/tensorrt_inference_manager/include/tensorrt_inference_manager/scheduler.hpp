#pragma once

#include "nvbuf_pool.hpp"
#include <ipc_unix_socket/ipc_unix_socket.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// A single frame entry held in a model queue.
// ---------------------------------------------------------------------------
struct QueuedFrame {
    int      dmabuf_fd;
    ipc::FrameMeta  meta;
    uint32_t pool_slot;       ///< NvBufSurface pool slot index
    uint64_t enqueue_time_ns; ///< For latency measurement
};

// ---------------------------------------------------------------------------
// BatchReadyCallback — invoked when a batch is ready for inference.
// `model_name`  : identifies which TRT engine to run
// `frames`      : batch of QueuedFrames (already imported into pool)
// ---------------------------------------------------------------------------
using BatchReadyCallback =
    std::function<void(const std::string& model_name,
                       std::vector<QueuedFrame> frames)>;

// ---------------------------------------------------------------------------
// Per-model queue configuration
// ---------------------------------------------------------------------------
struct ModelQueueConfig {
    std::string  model_name;
    uint32_t     batch_size{4};
    uint32_t     timeout_ms{5};
    std::vector<uint32_t> subscribed_cameras;  ///< empty = all cameras
};

// ---------------------------------------------------------------------------
// HybridScheduler
//
// Routes incoming frames to model-specific lock-free queues.
// Fires a BatchReadyCallback when:
//   - queue.size() >= batch_size, OR
//   - timeout_ms has elapsed since the first frame was enqueued.
//
// Each model queue has a dedicated drain thread.
// ---------------------------------------------------------------------------
class HybridScheduler {
public:
    explicit HybridScheduler(NvBufSurfacePool& pool);
    ~HybridScheduler();

    // Non-copyable
    HybridScheduler(const HybridScheduler&)            = delete;
    HybridScheduler& operator=(const HybridScheduler&) = delete;

    /// Register a model queue before calling start().
    void add_model_queue(const ModelQueueConfig& config);

    /// Set the callback invoked when a batch is ready.
    void set_batch_ready_callback(BatchReadyCallback cb);

    /// Start all drain threads.
    void start();

    /// Stop all drain threads and flush remaining frames.
    void stop();

    /// Submit a newly received frame. Called from IPC receiver thread.
    /// Imports DMABUF into the NvBufSurface pool and routes to queue(s).
    void submit_frame(int dmabuf_fd, const ipc::FrameMeta& meta);

    // Stats
    uint64_t total_frames_routed()  const { return total_routed_.load();  }
    uint64_t total_batches_fired()  const { return total_batches_.load(); }

private:
    struct ModelQueue {
        ModelQueueConfig          config;
        std::deque<QueuedFrame>   queue;
        mutable std::mutex        mutex;
        std::condition_variable   cv;
        std::thread               drain_thread;
        std::atomic<bool>         running{false};
        std::chrono::steady_clock::time_point first_enqueue_time;
        bool                      timer_armed{false};
    };

    void drain_loop(ModelQueue& mq);
    bool should_route_to(const ModelQueueConfig& cfg, uint32_t camera_id) const;

    NvBufSurfacePool&                               pool_;
    BatchReadyCallback                              batch_ready_cb_;
    std::unordered_map<std::string,
                       std::unique_ptr<ModelQueue>> queues_;

    std::atomic<uint64_t> total_routed_{0};
    std::atomic<uint64_t> total_batches_{0};
};

} // namespace autodriver::inference
