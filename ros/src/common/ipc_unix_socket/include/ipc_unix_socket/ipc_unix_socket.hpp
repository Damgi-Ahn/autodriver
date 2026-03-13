#pragma once

#include <cstdint>
#include <string>

// ---------------------------------------------------------------------------
// autodriver::ipc
//
// High-performance Unix Domain Socket IPC with SCM_RIGHTS DMABUF FD passing.
// Designed for zero-copy frame transport between camera_manager and
// tensorrt_inference_manager on NVIDIA Jetson Orin (JetPack 6.0).
//
// Thread-safety contract:
//   - One SendFd() per connected socket fd at a time (caller's responsibility).
//   - Multiple INDEPENDENT sockets may be used concurrently from different threads.
//   - Create/Close functions are NOT thread-safe with concurrent I/O on the same fd.
//
// Error handling:
//   - No exceptions thrown. All errors returned as IpcStatus.
//   - Never discard [[nodiscard]] return values.
// ---------------------------------------------------------------------------

namespace autodriver::ipc {

// ---------------------------------------------------------------------------
// FrameMeta — fixed-size POD transmitted alongside every DMABUF fd.
// ABI-stable: do not reorder fields or add virtual members.
// ---------------------------------------------------------------------------
#pragma pack(push, 4)
struct FrameMeta {
  uint32_t camera_id;  ///< Source camera index [0, 11]
  uint64_t timestamp;  ///< CLOCK_MONOTONIC nanoseconds
  uint32_t width;      ///< Frame width in pixels
  uint32_t height;     ///< Frame height in pixels
  uint32_t format;     ///< NvBufSurfaceColorFormat cast to uint32_t
};
#pragma pack(pop)

static_assert(sizeof(FrameMeta) == 24,
              "FrameMeta layout changed — synchronise sender and receiver");
static_assert(alignof(FrameMeta) == 4,
              "FrameMeta alignment changed — check IPC padding");

// ---------------------------------------------------------------------------
// IpcStatus — explicit error codes.  Never use raw integer comparisons.
// ---------------------------------------------------------------------------
enum class IpcStatus : int32_t {
  kOk           =  0,
  kSocketError  = -1,   ///< socket() syscall failed
  kBindError    = -2,   ///< bind() failed
  kListenError  = -3,   ///< listen() failed
  kConnectError = -4,   ///< All retry attempts exhausted
  kAcceptError  = -5,   ///< accept() failed
  kSendError    = -6,   ///< sendmsg() returned non-recoverable error
  kRecvError    = -7,   ///< recvmsg() returned non-recoverable error
  kPeerClosed   = -8,   ///< Remote end closed the connection (recv == 0)
  kNoFdReceived = -9,   ///< recvmsg() had no SCM_RIGHTS ancillary fd
  kWouldBlock   = -10,  ///< O_NONBLOCK set and operation would block
  kInvalidArg   = -11,  ///< Null pointer or invalid fd
  kSysError     = -12,  ///< fcntl / setsockopt failed
};

/// True iff `s` is kOk.
constexpr bool IsOk(IpcStatus s) noexcept { return s == IpcStatus::kOk; }

/// Stable human-readable label for logging.  Never returns nullptr.
[[nodiscard]] const char* StatusString(IpcStatus status) noexcept;

// ---------------------------------------------------------------------------
// Server-side socket lifecycle
// ---------------------------------------------------------------------------

/// Create, bind, and listen on a Unix domain socket at `path`.
/// Removes any stale socket file before binding.
[[nodiscard]] int CreateServerSocket(const std::string& path,
                                     IpcStatus* out_status = nullptr,
                                     int backlog = 16) noexcept;

/// Block until one client connects.  Returns peer fd or -1.
[[nodiscard]] int AcceptConnection(int server_fd,
                                   IpcStatus* out_status = nullptr) noexcept;

/// Close `fd` and unlink `path`.  Safe when fd == -1.
void CloseServerSocket(int fd, const std::string& path) noexcept;

// ---------------------------------------------------------------------------
// Client-side socket lifecycle
// ---------------------------------------------------------------------------

/// Connect to Unix socket server at `path`.
/// Retries `max_retries` times with `retry_delay_ms` ms between attempts.
[[nodiscard]] int CreateClientSocket(const std::string& path,
                                     IpcStatus* out_status = nullptr,
                                     int max_retries    = 10,
                                     int retry_delay_ms = 100) noexcept;

/// Close socket fd.  Safe when fd == -1.
void CloseSocket(int fd) noexcept;

// ---------------------------------------------------------------------------
// Zero-copy FD transfer (SCM_RIGHTS)
// ---------------------------------------------------------------------------

/// Send `dmabuf_fd` and `meta` atomically over `sock_fd` via sendmsg(2).
/// Retries on EINTR.  Both fd and meta travel in a single sendmsg so the
/// receiver always gets both together.
///
/// Returns kWouldBlock (not an error) when O_NONBLOCK is set and buffer full.
[[nodiscard]] IpcStatus SendFd(int sock_fd, int dmabuf_fd,
                                const FrameMeta& meta) noexcept;

/// Receive fd + FrameMeta from `sock_fd` via recvmsg(2).
/// Caller owns *out_dmabuf_fd on success and must close it.
/// Retries on EINTR.
///
/// Returns kPeerClosed on EOF, kWouldBlock if no data and O_NONBLOCK set,
/// kNoFdReceived if ancillary SCM_RIGHTS was absent.
[[nodiscard]] IpcStatus RecvFd(int sock_fd, int* out_dmabuf_fd,
                                FrameMeta* out_meta) noexcept;

// ---------------------------------------------------------------------------
// Socket option utilities
// ---------------------------------------------------------------------------

/// Apply O_NONBLOCK to `fd` via fcntl.
[[nodiscard]] IpcStatus SetNonBlocking(int fd) noexcept;

/// Set SO_SNDBUF and SO_RCVBUF to `size_bytes`.
[[nodiscard]] IpcStatus SetSocketBufferSize(int fd, int size_bytes) noexcept;

// ---------------------------------------------------------------------------
// Clock helper (exposed for profiling / tests)
// ---------------------------------------------------------------------------

/// CLOCK_MONOTONIC time in nanoseconds.  Never returns 0.
[[nodiscard]] uint64_t NowNs() noexcept;

}  // namespace autodriver::ipc
