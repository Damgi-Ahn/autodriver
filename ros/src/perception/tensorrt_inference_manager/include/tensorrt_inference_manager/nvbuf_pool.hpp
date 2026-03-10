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
// Pre-allocates `pool_size` NvBufSurface objects at startup (GPU memory).
// Provides acquire/release semantics for batch-safe zero-latency buffer access.
//
// Usage pattern (per incoming DMABUF frame):
//   1. pool.AcquireSlot(timeout_ms)      → slot index
//   2. pool.CopyFromImport(src, slot)    → deep-copy external surface to slot
//   3. Pass slot to QueuedFrame          → drain thread picks it up
//   4. ModelRunner copies from slot to  device input buffer
//   5. pool.ReleaseSlot(slot)            → slot back to free list
//
// Thread-safety: AcquireSlot / ReleaseSlot are mutex-protected.
// Surface() is const-like; caller must not concurrently acquire the same slot.
// ---------------------------------------------------------------------------
class NvBufSurfacePool {
 public:
  static constexpr uint32_t kDefaultPoolSize = 128;
  static constexpr uint32_t kInvalidSlot     = UINT32_MAX;

  explicit NvBufSurfacePool(
      uint32_t                pool_size = kDefaultPoolSize,
      uint32_t                width     = 1920,
      uint32_t                height    = 1080,
      NvBufSurfaceColorFormat format    = NVBUF_COLOR_FORMAT_NV12);
  ~NvBufSurfacePool();

  NvBufSurfacePool(const NvBufSurfacePool&)            = delete;
  NvBufSurfacePool& operator=(const NvBufSurfacePool&) = delete;

  /// Allocate all pool surfaces. Must be called once before any Acquire.
  /// Returns false if any surface allocation fails.
  [[nodiscard]] bool Init() noexcept;

  /// Import an external DMABUF fd into a temporary NvBufSurface.
  /// Caller must destroy the returned surface with NvBufSurfaceDestroy().
  /// Returns nullptr on failure.
  [[nodiscard]] static NvBufSurface* ImportFd(int dmabuf_fd) noexcept;

  /// Acquire a free slot.  Spin-waits at most `timeout_ms`.
  /// Returns kInvalidSlot if all slots are busy.
  [[nodiscard]] uint32_t AcquireSlot(int timeout_ms = 50) noexcept;

  /// Deep-copy `src` into the pre-allocated surface at `slot`.
  /// src must have the same format/resolution as the pool surfaces.
  /// Returns false on NvBufSurfaceCopy failure.
  [[nodiscard]] bool CopyFromImport(NvBufSurface* src,
                                     uint32_t      slot) noexcept;

  /// Return `slot` to the free list.
  void ReleaseSlot(uint32_t slot) noexcept;

  /// Direct read-only access to a pool surface.
  /// Only valid while the slot is held (between Acquire and Release).
  [[nodiscard]] NvBufSurface* Surface(uint32_t slot) const noexcept;

  [[nodiscard]] uint32_t pool_size()  const noexcept { return pool_size_;  }
  [[nodiscard]] uint32_t FreeCount()  const noexcept;

 private:
  uint32_t                        pool_size_;
  uint32_t                        width_;
  uint32_t                        height_;
  NvBufSurfaceColorFormat         format_;

  std::vector<NvBufSurface*>      surfaces_;   ///< Indexed by slot
  std::vector<uint32_t>           free_list_;
  mutable std::mutex              free_list_mutex_;
  std::atomic<uint32_t>           free_count_{0};
  bool                            initialised_{false};
};

}  // namespace autodriver::inference
