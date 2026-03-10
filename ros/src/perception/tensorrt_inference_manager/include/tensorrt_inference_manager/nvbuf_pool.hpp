#pragma once

#include <nvbufsurface.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace autodriver::inference {

// ---------------------------------------------------------------------------
// NvBufSurfacePool
//
// Pre-allocates a fixed number of NvBufSurface slots at startup.
// Callers acquire a slot index, use the surface, then release it.
// All allocation/deallocation happens at init/shutdown — never in the
// hot inference path.
//
// Thread-safety: acquire/release are thread-safe via a free-list mutex.
// ---------------------------------------------------------------------------
class NvBufSurfacePool {
public:
    static constexpr uint32_t kDefaultPoolSize = 128;

    explicit NvBufSurfacePool(uint32_t pool_size  = kDefaultPoolSize,
                              uint32_t width       = 1920,
                              uint32_t height      = 1080,
                              NvBufSurfaceColorFormat format = NVBUF_COLOR_FORMAT_NV12);
    ~NvBufSurfacePool();

    // Non-copyable
    NvBufSurfacePool(const NvBufSurfacePool&)            = delete;
    NvBufSurfacePool& operator=(const NvBufSurfacePool&) = delete;

    /// Initialise the pool — allocate all surfaces.
    /// Must be called once before acquire().
    bool init();

    /// Acquire a free slot. Blocks (spin-waits briefly) if pool is exhausted.
    /// Returns slot index [0, pool_size), or kInvalidSlot on timeout.
    static constexpr uint32_t kInvalidSlot = UINT32_MAX;
    uint32_t acquire(int timeout_ms = 50);

    /// Return a slot to the free list.
    void release(uint32_t slot);

    /// Direct access to a surface by slot index.
    NvBufSurface* surface(uint32_t slot);

    uint32_t pool_size()   const { return pool_size_;   }
    uint32_t free_count()  const { return free_count_.load(); }

private:
    uint32_t                         pool_size_;
    uint32_t                         width_;
    uint32_t                         height_;
    NvBufSurfaceColorFormat          format_;

    std::vector<NvBufSurface*>       surfaces_;   ///< Indexed by slot
    std::vector<uint32_t>            free_list_;
    std::mutex                       free_list_mutex_;
    std::atomic<uint32_t>            free_count_{0};
    bool                             initialised_{false};
};

} // namespace autodriver::inference
