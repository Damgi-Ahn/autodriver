#pragma once

#include <cstdint>
#include <string>

namespace autodriver::ipc {

// ---------------------------------------------------------------------------
// Frame metadata transmitted alongside each DMABUF file descriptor.
// Layout must be identical in sender (camera_manager) and receiver
// (tensorrt_inference_manager) — do not add virtual methods or padding.
// ---------------------------------------------------------------------------
struct FrameMeta {
    uint32_t camera_id;   ///< Source camera index [0, 11]
    uint64_t timestamp;   ///< Capture time in nanoseconds (CLOCK_MONOTONIC)
    uint32_t width;       ///< Frame width in pixels
    uint32_t height;      ///< Frame height in pixels
    uint32_t format;      ///< NvBufSurfaceColorFormat cast to uint32_t
};
static_assert(sizeof(FrameMeta) == 24, "FrameMeta layout changed — update both sides");

// ---------------------------------------------------------------------------
// Server-side socket lifecycle
// ---------------------------------------------------------------------------

/// Create a Unix domain socket at `path`, bind, and listen.
/// Removes any stale socket file before binding.
/// @return listening socket fd, or -1 on failure (errno set).
int create_server_socket(const std::string& path);

/// Accept a single incoming connection on `server_fd` (blocking).
/// @return connected peer fd, or -1 on failure.
int accept_connection(int server_fd);

/// Close `fd` and unlink the socket file at `path`.
void close_server_socket(int fd, const std::string& path);

// ---------------------------------------------------------------------------
// Client-side socket lifecycle
// ---------------------------------------------------------------------------

/// Connect to the Unix domain socket server at `path`.
/// Retries up to `max_retries` times with `retry_delay_ms` between attempts
/// to handle server startup races.
/// @return connected socket fd, or -1 on failure.
int create_client_socket(const std::string& path,
                         int max_retries    = 10,
                         int retry_delay_ms = 100);

/// Close a client-side socket fd.
void close_socket(int fd);

// ---------------------------------------------------------------------------
// Zero-copy FD transfer (SCM_RIGHTS)
// ---------------------------------------------------------------------------

/// Send `dmabuf_fd` and `meta` over `sock_fd` using sendmsg + SCM_RIGHTS.
/// The call is non-blocking (SOCK_NONBLOCK is set on server accept path);
/// returns false with errno == EAGAIN if the send buffer is full.
/// @return true on success, false on failure.
bool send_fd(int sock_fd, int dmabuf_fd, const FrameMeta& meta);

/// Receive a file descriptor and FrameMeta from `sock_fd` using recvmsg.
/// On success, `*dmabuf_fd` holds the imported fd (must be closed by caller
/// after the GPU buffer is no longer needed).
/// @return true on success, false on EOF or error.
bool recv_fd(int sock_fd, int* dmabuf_fd, FrameMeta* meta);

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

/// Set a socket to non-blocking mode.
/// @return true on success.
bool set_nonblocking(int fd);

/// Set SO_SNDBUF / SO_RCVBUF to `size_bytes`.
bool set_socket_buffer_size(int fd, int size_bytes);

} // namespace autodriver::ipc
