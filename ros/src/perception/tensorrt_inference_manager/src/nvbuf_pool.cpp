#include "tensorrt_inference_manager/nvbuf_pool.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace autodriver::inference {

NvBufSurfacePool::NvBufSurfacePool(uint32_t pool_size,
                                    uint32_t width,
                                    uint32_t height,
                                    NvBufSurfaceColorFormat format)
    : pool_size_(pool_size), width_(width), height_(height), format_(format)
{
    surfaces_.resize(pool_size_, nullptr);
    free_list_.reserve(pool_size_);
}

NvBufSurfacePool::~NvBufSurfacePool()
{
    for (auto* surf : surfaces_) {
        if (surf) NvBufSurfaceDestroy(surf);
    }
}

bool NvBufSurfacePool::init()
{
    NvBufSurfaceCreateParams params{};
    params.width       = width_;
    params.height      = height_;
    params.colorFormat = format_;
    params.layout      = NVBUF_LAYOUT_PITCH;
    params.memType     = NVBUF_MEM_DEFAULT;  // GPU-accessible memory
    params.isContiguous = true;
    params.size        = 0;  // Auto

    for (uint32_t i = 0; i < pool_size_; ++i) {
        if (NvBufSurfaceCreate(&surfaces_[i], /*batchSize=*/1, &params) != 0) {
            return false;
        }
        free_list_.push_back(i);
    }

    free_count_.store(pool_size_);
    initialised_ = true;
    return true;
}

uint32_t NvBufSurfacePool::acquire(int timeout_ms)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);

    while (true) {
        {
            std::lock_guard<std::mutex> lock(free_list_mutex_);
            if (!free_list_.empty()) {
                uint32_t slot = free_list_.back();
                free_list_.pop_back();
                free_count_.fetch_sub(1, std::memory_order_relaxed);
                return slot;
            }
        }
        if (std::chrono::steady_clock::now() >= deadline) return kInvalidSlot;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void NvBufSurfacePool::release(uint32_t slot)
{
    std::lock_guard<std::mutex> lock(free_list_mutex_);
    free_list_.push_back(slot);
    free_count_.fetch_add(1, std::memory_order_relaxed);
}

NvBufSurface* NvBufSurfacePool::surface(uint32_t slot)
{
    return surfaces_[slot];
}

} // namespace autodriver::inference
