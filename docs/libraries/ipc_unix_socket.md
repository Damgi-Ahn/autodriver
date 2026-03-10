# ipc_unix_socket

**Package:** `ros/src/common/ipc_unix_socket`
**Type:** Shared library (no ROS runtime dependency)
**CMake target:** `autodriver::ipc_unix_socket`
**Namespace:** `autodriver::ipc`

---

## Overview

High-performance Unix Domain Socket library for DMABUF file descriptor passing
between processes on NVIDIA Jetson Orin. Uses `sendmsg(2)` / `recvmsg(2)` with
`SCM_RIGHTS` ancillary data to transfer DMABUF FDs alongside a fixed-size
`FrameMeta` payload in a single syscall.

**Key properties:**
- No exceptions — all errors returned as `IpcStatus`
- No dynamic allocation on the send/receive path
- `[[nodiscard]]` on all status-returning functions
- Thread-safe: independent sockets safe to use concurrently; same socket must be
  serialised by the caller

---

## FrameMeta

```cpp
struct FrameMeta {
  uint32_t camera_id;   // Source camera index [0, 11]
  uint64_t timestamp;   // CLOCK_MONOTONIC nanoseconds
  uint32_t width;       // Frame width in pixels
  uint32_t height;      // Frame height in pixels
  uint32_t format;      // NvBufSurfaceColorFormat cast to uint32_t
};
// sizeof(FrameMeta) == 24, alignof == 4 (verified by static_assert)
```

`FrameMeta` is ABI-stable. Do not reorder fields or add virtual members.

---

## IpcStatus Codes

| Code | Value | Meaning |
|---|---|---|
| `kOk` | 0 | Success |
| `kSocketError` | -1 | `socket()` failed |
| `kBindError` | -2 | `bind()` failed |
| `kListenError` | -3 | `listen()` failed |
| `kConnectError` | -4 | All retry attempts exhausted |
| `kAcceptError` | -5 | `accept()` failed |
| `kSendError` | -6 | `sendmsg()` non-recoverable error |
| `kRecvError` | -7 | `recvmsg()` non-recoverable error |
| `kPeerClosed` | -8 | Remote end closed connection |
| `kNoFdReceived` | -9 | SCM_RIGHTS ancillary fd absent |
| `kWouldBlock` | -10 | O_NONBLOCK set and would block |
| `kInvalidArg` | -11 | Null pointer or invalid fd |
| `kSysError` | -12 | `fcntl` / `setsockopt` failed |

---

## API

### Server lifecycle

```cpp
// Create, bind, listen. Removes stale socket file before binding.
int server_fd = ipc::CreateServerSocket("/tmp/autodriver/frames.sock");

// Block until client connects. Returns peer fd or -1.
int peer_fd = ipc::AcceptConnection(server_fd);

// Close fd and unlink path.
ipc::CloseServerSocket(server_fd, "/tmp/autodriver/frames.sock");
```

### Client lifecycle

```cpp
// Connect with retry (10 retries, 100ms apart by default).
int sock_fd = ipc::CreateClientSocket("/tmp/autodriver/frames.sock");

ipc::CloseSocket(sock_fd);
```

### Frame transfer

```cpp
// Send (camera_manager side)
FrameMeta meta{camera_id, timestamp_ns, width, height, format};
IpcStatus s = ipc::SendFd(sock_fd, dmabuf_fd, meta);
if (s == IpcStatus::kWouldBlock) { /* drop or retry */ }

// Receive (inference_manager side)
int dmabuf_fd = -1;
FrameMeta meta{};
IpcStatus s = ipc::RecvFd(peer_fd, &dmabuf_fd, &meta);
if (s == IpcStatus::kPeerClosed) { /* shutdown */ }
// Caller owns dmabuf_fd — must close() after use
```

### Utilities

```cpp
ipc::SetNonBlocking(fd);             // Apply O_NONBLOCK
ipc::SetSocketBufferSize(fd, 24*1024*1024);  // SO_SNDBUF + SO_RCVBUF
uint64_t ns = ipc::NowNs();          // CLOCK_MONOTONIC nanoseconds
```

---

## Build Integration

`ipc_unix_socket` exports its CMake target with the `autodriver::` namespace:

```cmake
find_package(ipc_unix_socket REQUIRED)
target_link_libraries(my_target PRIVATE autodriver::ipc_unix_socket)
```

---

## Tests

`test/test_ipc_unix_socket.cpp` — tests server create/accept/close, client connect
with retry, `SendFd` + `RecvFd` round-trip, `FrameMeta` integrity, error status
codes, non-blocking behaviour, and buffer size settings.
