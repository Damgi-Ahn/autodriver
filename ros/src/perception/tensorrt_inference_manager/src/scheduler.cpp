#include "tensorrt_inference_manager/scheduler.hpp"

#include <nvbufsurface.h>

namespace autodriver::inference {

HybridScheduler::HybridScheduler(NvBufSurfacePool& pool)
    : pool_(pool)
{}

HybridScheduler::~HybridScheduler()
{
    stop();
}

void HybridScheduler::add_model_queue(const ModelQueueConfig& config)
{
    auto mq      = std::make_unique<ModelQueue>();
    mq->config   = config;
    queues_[config.model_name] = std::move(mq);
}

void HybridScheduler::set_batch_ready_callback(BatchReadyCallback cb)
{
    batch_ready_cb_ = std::move(cb);
}

void HybridScheduler::start()
{
    for (auto& [name, mq] : queues_) {
        mq->running.store(true);
        mq->drain_thread = std::thread([this, &mq = *mq]{ drain_loop(mq); });
    }
}

void HybridScheduler::stop()
{
    for (auto& [name, mq] : queues_) {
        mq->running.store(false);
        mq->cv.notify_all();
        if (mq->drain_thread.joinable()) mq->drain_thread.join();
    }
}

void HybridScheduler::submit_frame(int dmabuf_fd, const ipc::FrameMeta& meta)
{
    // Import DMABUF into NvBufSurface pool
    uint32_t slot = pool_.acquire(/*timeout_ms=*/20);
    if (slot == NvBufSurfacePool::kInvalidSlot) {
        ::close(dmabuf_fd);
        return;
    }

    // TODO(Stage 6): NvBufSurfaceFromFd(dmabuf_fd) → fill pool slot
    // For now: track the fd in the queued frame and handle in model_runner

    const uint64_t now_ns = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());

    QueuedFrame frame{dmabuf_fd, meta, slot, now_ns};

    // Route to all matching queues
    for (auto& [name, mq] : queues_) {
        if (!should_route_to(mq->config, meta.camera_id)) continue;

        {
            std::lock_guard<std::mutex> lock(mq->mutex);
            mq->queue.push_back(frame);
            if (!mq->timer_armed) {
                mq->first_enqueue_time = std::chrono::steady_clock::now();
                mq->timer_armed = true;
            }
        }
        mq->cv.notify_one();
        total_routed_.fetch_add(1, std::memory_order_relaxed);
    }
}

bool HybridScheduler::should_route_to(const ModelQueueConfig& cfg,
                                        uint32_t camera_id) const
{
    if (cfg.subscribed_cameras.empty()) return true;
    for (auto id : cfg.subscribed_cameras)
        if (id == camera_id) return true;
    return false;
}

void HybridScheduler::drain_loop(ModelQueue& mq)
{
    while (mq.running.load()) {
        std::unique_lock<std::mutex> lock(mq.mutex);

        const auto timeout = std::chrono::milliseconds(mq.config.timeout_ms);
        mq.cv.wait_for(lock, timeout, [&mq]{
            return !mq.running.load() ||
                   mq.queue.size() >= mq.config.batch_size ||
                   (mq.timer_armed &&
                    (std::chrono::steady_clock::now() - mq.first_enqueue_time) >=
                    std::chrono::milliseconds(mq.config.timeout_ms));
        });

        if (mq.queue.empty()) continue;

        // Dequeue up to batch_size frames
        std::vector<QueuedFrame> batch;
        const size_t n = std::min(static_cast<size_t>(mq.config.batch_size),
                                   mq.queue.size());
        batch.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            batch.push_back(mq.queue.front());
            mq.queue.pop_front();
        }
        mq.timer_armed = !mq.queue.empty();
        if (mq.timer_armed) mq.first_enqueue_time = std::chrono::steady_clock::now();

        lock.unlock();

        if (batch_ready_cb_) {
            batch_ready_cb_(mq.config.model_name, std::move(batch));
            total_batches_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

} // namespace autodriver::inference
