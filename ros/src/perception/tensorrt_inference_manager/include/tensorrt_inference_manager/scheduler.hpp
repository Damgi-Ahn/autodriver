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
// QueuedFrame — one frame entry inside a model queue.
// ---------------------------------------------------------------------------
struct QueuedFrame {
  int             dmabuf_fd{-1};   ///< Closed after import; -1 = already released
  ipc::FrameMeta  meta{};
  uint32_t        pool_slot{NvBufSurfacePool::kInvalidSlot};
  uint64_t        enqueue_ns{0};   ///< For end-to-end latency measurement
};

// ---------------------------------------------------------------------------
// BatchReadyCallback — fired when a batch is ready for inference.
// Called from the model's dedicated drain thread.
// ---------------------------------------------------------------------------
using BatchReadyCallback = std::function<void(
    const std::string&       model_name,
    std::vector<QueuedFrame> frames)>;

// ---------------------------------------------------------------------------
// ModelQueueConfig — per-model scheduling parameters.
// ---------------------------------------------------------------------------
struct ModelQueueConfig {
  std::string              model_name;
  uint32_t                 batch_size{4};
  uint32_t                 timeout_ms{5};
  std::vector<uint32_t>    subscribed_cameras;  ///< empty = subscribe to all
};

// ---------------------------------------------------------------------------
// HybridScheduler
//
// Routes frames to per-model lock-protected queues.
// Fires BatchReadyCallback when:
//   - queue depth >= batch_size, OR
//   - timeout_ms elapsed since the first frame in the queue was enqueued.
//
// DMABUF import path (hot path — called from IPC thread):
//   SubmitFrame(dmabuf_fd, meta):
//     1. NvBufSurfacePool::ImportFd(dmabuf_fd)   → import_surface
//     2. pool.AcquireSlot()                       → slot
//     3. pool.CopyFromImport(import_surface, slot) → deep copy
//     4. NvBufSurfaceDestroy(import_surface)
//     5. close(dmabuf_fd)                        → camera_manager ref released
//     6. Route QueuedFrame{slot, meta} to all matching queues
//
// Thread-safety:
//   SubmitFrame() is called from a single IPC receiver thread.
//   Drain threads lock their own queue mutex independently.
//   BatchReadyCallback is called from drain threads (one per model).
// ---------------------------------------------------------------------------
class HybridScheduler {
 public:
  explicit HybridScheduler(NvBufSurfacePool& pool);
  ~HybridScheduler();

  HybridScheduler(const HybridScheduler&)            = delete;
  HybridScheduler& operator=(const HybridScheduler&) = delete;

  /// Register a model queue.  Must be called before Start().
  void AddModelQueue(const ModelQueueConfig& config);

  /// Set callback invoked when a batch is ready.  Must be set before Start().
  void SetBatchReadyCallback(BatchReadyCallback cb);

  /// Start drain threads.
  void Start();

  /// Stop drain threads and flush in-flight frames.
  void Stop();

  /// Full import + route path called from IPC receiver thread.
  void SubmitFrame(int dmabuf_fd, const ipc::FrameMeta& meta);

  [[nodiscard]] uint64_t total_frames_routed() const noexcept;
  [[nodiscard]] uint64_t total_batches_fired() const noexcept;

 protected:
  // Route a pre-imported QueuedFrame to all matching queues.
  // Exposed as protected for unit tests (allows bypassing GPU import).
  void SubmitPreImported(QueuedFrame frame);

 private:
  struct ModelQueue {
    ModelQueueConfig    config;
    std::deque<QueuedFrame>   queue;
    mutable std::mutex        mutex;
    std::condition_variable   cv;
    std::thread               drain_thread;
    std::atomic<bool>         running{false};

    // Timeout tracking (guarded by mutex)
    std::chrono::steady_clock::time_point first_enqueue_time{};
    bool                                  timer_armed{false};
  };

  void DrainLoop(ModelQueue& mq);

  [[nodiscard]] bool ShouldRoute(const ModelQueueConfig& cfg,
                                  uint32_t camera_id) const noexcept;

  NvBufSurfacePool&  pool_;
  BatchReadyCallback batch_ready_cb_;

  std::unordered_map<std::string, std::unique_ptr<ModelQueue>> queues_;

  std::atomic<uint64_t> total_routed_{0};
  std::atomic<uint64_t> total_batches_{0};
};

}  // namespace autodriver::inference
