#include "tensorrt_inference_manager/scheduler.hpp"

#include <unistd.h>

namespace autodriver::inference {

HybridScheduler::HybridScheduler(NvBufSurfacePool& pool)
    : pool_(pool)
{}

HybridScheduler::~HybridScheduler()
{
  Stop();
}

void HybridScheduler::AddModelQueue(const ModelQueueConfig& config)
{
  auto mq    = std::make_unique<ModelQueue>();
  mq->config = config;
  queues_[config.model_name] = std::move(mq);
}

void HybridScheduler::SetBatchReadyCallback(BatchReadyCallback cb)
{
  batch_ready_cb_ = std::move(cb);
}

void HybridScheduler::Start()
{
  for (auto& [name, mq] : queues_) {
    mq->running.store(true);
    mq->drain_thread = std::thread([this, &mq = *mq] { DrainLoop(mq); });
  }
}

void HybridScheduler::Stop()
{
  for (auto& [name, mq] : queues_) {
    mq->running.store(false);
    mq->cv.notify_all();
    if (mq->drain_thread.joinable()) mq->drain_thread.join();
  }
}

// ---------------------------------------------------------------------------
// SubmitFrame — full import path (called from IPC receiver thread)
// ---------------------------------------------------------------------------

void HybridScheduler::SubmitFrame(int dmabuf_fd, const ipc::FrameMeta& meta)
{
  // 1. Import DMABUF into a temporary NvBufSurface (GPU handle, no copy yet).
  NvBufSurface* import_surf = NvBufSurfacePool::ImportFd(dmabuf_fd);
  if (!import_surf) {
    ::close(dmabuf_fd);
    return;
  }

  // 2. Acquire a pre-allocated pool slot (will be the copy target).
  uint32_t slot = pool_.AcquireSlot(/*timeout_ms=*/20);
  if (slot == NvBufSurfacePool::kInvalidSlot) {
    NvBufSurfaceDestroy(import_surf);
    ::close(dmabuf_fd);
    return;
  }

  // 3. Deep-copy imported surface into pool slot (GPU-to-GPU via NvBufSurface).
  if (!pool_.CopyFromImport(import_surf, slot)) {
    NvBufSurfaceDestroy(import_surf);
    pool_.ReleaseSlot(slot);
    ::close(dmabuf_fd);
    return;
  }

  // 4. Release the temporary import surface and camera_manager's fd reference.
  NvBufSurfaceDestroy(import_surf);
  ::close(dmabuf_fd);

  // 5. Build QueuedFrame and route to all matching model queues.
  const uint64_t now_ns = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count());

  QueuedFrame frame;
  frame.dmabuf_fd  = -1;   // Already closed above
  frame.meta       = meta;
  frame.pool_slot  = slot;
  frame.enqueue_ns = now_ns;

  SubmitPreImported(frame);
}

// ---------------------------------------------------------------------------
// SubmitPreImported — route a QueuedFrame (already in pool) to model queues.
// Protected: called by SubmitFrame and directly by unit tests.
// ---------------------------------------------------------------------------

void HybridScheduler::SubmitPreImported(QueuedFrame frame)
{
  bool routed_at_least_once = false;

  for (auto& [name, mq] : queues_) {
    if (!ShouldRoute(mq->config, frame.meta.camera_id)) continue;

    {
      std::lock_guard<std::mutex> lock(mq->mutex);
      mq->queue.push_back(frame);
      if (!mq->timer_armed) {
        mq->first_enqueue_time = std::chrono::steady_clock::now();
        mq->timer_armed        = true;
      }
    }
    mq->cv.notify_one();
    total_routed_.fetch_add(1, std::memory_order_relaxed);
    routed_at_least_once = true;
  }

  // If no model subscribed to this camera, release the pool slot immediately.
  if (!routed_at_least_once && frame.pool_slot != NvBufSurfacePool::kInvalidSlot)
    pool_.ReleaseSlot(frame.pool_slot);
}

// ---------------------------------------------------------------------------
// DrainLoop — dedicated thread per model queue
// ---------------------------------------------------------------------------

void HybridScheduler::DrainLoop(ModelQueue& mq)
{
  while (mq.running.load()) {
    std::unique_lock<std::mutex> lock(mq.mutex);

    const auto timeout_dur = std::chrono::milliseconds(mq.config.timeout_ms);

    mq.cv.wait_for(lock, timeout_dur, [&mq, &timeout_dur] {
      if (!mq.running.load()) return true;
      if (mq.queue.size() >= mq.config.batch_size) return true;
      if (mq.timer_armed &&
          (std::chrono::steady_clock::now() - mq.first_enqueue_time) >= timeout_dur)
        return true;
      return false;
    });

    if (mq.queue.empty()) continue;

    // Dequeue up to batch_size frames atomically under the lock.
    const size_t n = std::min(static_cast<size_t>(mq.config.batch_size),
                               mq.queue.size());
    std::vector<QueuedFrame> batch;
    batch.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      batch.push_back(std::move(mq.queue.front()));
      mq.queue.pop_front();
    }

    // Reset timer: arm again only if frames remain.
    if (!mq.queue.empty()) {
      mq.first_enqueue_time = std::chrono::steady_clock::now();
      mq.timer_armed        = true;
    } else {
      mq.timer_armed = false;
    }

    lock.unlock();

    if (batch_ready_cb_) {
      batch_ready_cb_(mq.config.model_name, std::move(batch));
      total_batches_.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

bool HybridScheduler::ShouldRoute(const ModelQueueConfig& cfg,
                                   uint32_t camera_id) const noexcept
{
  if (cfg.subscribed_cameras.empty()) return true;
  for (auto id : cfg.subscribed_cameras)
    if (id == camera_id) return true;
  return false;
}

uint64_t HybridScheduler::total_frames_routed() const noexcept
{
  return total_routed_.load(std::memory_order_relaxed);
}

uint64_t HybridScheduler::total_batches_fired() const noexcept
{
  return total_batches_.load(std::memory_order_relaxed);
}

}  // namespace autodriver::inference
