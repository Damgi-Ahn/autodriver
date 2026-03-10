#include "ipc_unix_socket/ipc_unix_socket.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <thread>
#include <chrono>

// ---------------------------------------------------------------------------
// Implementation notes:
//
//  1. sendmsg / recvmsg are retried on EINTR (signal interruption).
//  2. EAGAIN / EWOULDBLOCK are mapped to IpcStatus::kWouldBlock — callers
//     distinguish this from hard errors to implement flow-control policies.
//  3. All allocated fds are closed on every failure path to prevent leaks.
//  4. No heap allocation occurs in the hot path (SendFd / RecvFd).
//  5. Google C++ Style: CamelCase public functions, snake_case locals.
// ---------------------------------------------------------------------------

namespace autodriver::ipc {

// ---------------------------------------------------------------------------
// StatusString
// ---------------------------------------------------------------------------

const char* StatusString(IpcStatus status) noexcept {
  switch (status) {
    case IpcStatus::kOk:           return "OK";
    case IpcStatus::kSocketError:  return "SocketError";
    case IpcStatus::kBindError:    return "BindError";
    case IpcStatus::kListenError:  return "ListenError";
    case IpcStatus::kConnectError: return "ConnectError";
    case IpcStatus::kAcceptError:  return "AcceptError";
    case IpcStatus::kSendError:    return "SendError";
    case IpcStatus::kRecvError:    return "RecvError";
    case IpcStatus::kPeerClosed:   return "PeerClosed";
    case IpcStatus::kNoFdReceived: return "NoFdReceived";
    case IpcStatus::kWouldBlock:   return "WouldBlock";
    case IpcStatus::kInvalidArg:   return "InvalidArg";
    case IpcStatus::kSysError:     return "SysError";
    default:                       return "Unknown";
  }
}

// ---------------------------------------------------------------------------
// Internal helper: fill sockaddr_un from path.
// Returns false if path is too long for sun_path.
// ---------------------------------------------------------------------------
namespace {

bool FillSockAddr(sockaddr_un* addr, const std::string& path) noexcept {
  if (path.size() >= sizeof(addr->sun_path)) return false;
  std::memset(addr, 0, sizeof(*addr));
  addr->sun_family = AF_UNIX;
  std::strncpy(addr->sun_path, path.c_str(), sizeof(addr->sun_path) - 1);
  return true;
}

bool IsWouldBlock() noexcept {
  return (errno == EAGAIN || errno == EWOULDBLOCK);
}

}  // namespace

// ---------------------------------------------------------------------------
// Server
// ---------------------------------------------------------------------------

int CreateServerSocket(const std::string& path,
                       IpcStatus* out_status,
                       int backlog) noexcept {
  auto set = [&](IpcStatus s) noexcept {
    if (out_status) *out_status = s;
  };

  // Remove stale socket file — failure is benign (file may not exist).
  ::unlink(path.c_str());

  const int server_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (server_fd < 0) { set(IpcStatus::kSocketError); return -1; }

  sockaddr_un addr{};
  if (!FillSockAddr(&addr, path)) {
    ::close(server_fd);
    set(IpcStatus::kInvalidArg);
    return -1;
  }

  if (::bind(server_fd, reinterpret_cast<const sockaddr*>(&addr),
             sizeof(addr)) < 0) {
    ::close(server_fd);
    set(IpcStatus::kBindError);
    return -1;
  }

  if (::listen(server_fd, backlog) < 0) {
    ::close(server_fd);
    set(IpcStatus::kListenError);
    return -1;
  }

  set(IpcStatus::kOk);
  return server_fd;
}

int AcceptConnection(int server_fd, IpcStatus* out_status) noexcept {
  auto set = [&](IpcStatus s) noexcept {
    if (out_status) *out_status = s;
  };

  int peer_fd = -1;
  do {
    peer_fd = ::accept(server_fd, nullptr, nullptr);
  } while (peer_fd < 0 && errno == EINTR);

  if (peer_fd < 0) {
    set(IsWouldBlock() ? IpcStatus::kWouldBlock : IpcStatus::kAcceptError);
    return -1;
  }
  set(IpcStatus::kOk);
  return peer_fd;
}

void CloseServerSocket(int fd, const std::string& path) noexcept {
  if (fd >= 0) ::close(fd);
  if (!path.empty()) ::unlink(path.c_str());
}

// ---------------------------------------------------------------------------
// Client
// ---------------------------------------------------------------------------

int CreateClientSocket(const std::string& path,
                       IpcStatus* out_status,
                       int max_retries,
                       int retry_delay_ms) noexcept {
  auto set = [&](IpcStatus s) noexcept {
    if (out_status) *out_status = s;
  };

  const int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) { set(IpcStatus::kSocketError); return -1; }

  sockaddr_un addr{};
  if (!FillSockAddr(&addr, path)) {
    ::close(fd);
    set(IpcStatus::kInvalidArg);
    return -1;
  }

  for (int attempt = 0; attempt < max_retries; ++attempt) {
    if (::connect(fd, reinterpret_cast<const sockaddr*>(&addr),
                  sizeof(addr)) == 0) {
      set(IpcStatus::kOk);
      return fd;
    }
    if (errno != ENOENT && errno != ECONNREFUSED) {
      // Hard failure — stop retrying.
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
  }

  ::close(fd);
  set(IpcStatus::kConnectError);
  return -1;
}

void CloseSocket(int fd) noexcept {
  if (fd >= 0) ::close(fd);
}

// ---------------------------------------------------------------------------
// SendFd — sendmsg with SCM_RIGHTS, EINTR-safe
// ---------------------------------------------------------------------------

IpcStatus SendFd(int sock_fd, int dmabuf_fd, const FrameMeta& meta) noexcept {
  if (sock_fd < 0 || dmabuf_fd < 0) return IpcStatus::kInvalidArg;

  // Stack-allocated ancillary buffer — zero-initialised.
  alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))]{};

  // iov carries the FrameMeta payload (plain data, no pointers).
  iovec iov{};
  iov.iov_base = const_cast<FrameMeta*>(&meta);  // sendmsg doesn't mutate
  iov.iov_len  = sizeof(FrameMeta);

  msghdr msg{};
  msg.msg_iov        = &iov;
  msg.msg_iovlen     = 1;
  msg.msg_control    = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  // Populate the SCM_RIGHTS control message.
  cmsghdr* const cmsg = CMSG_FIRSTHDR(&msg);
  cmsg->cmsg_level    = SOL_SOCKET;
  cmsg->cmsg_type     = SCM_RIGHTS;
  cmsg->cmsg_len      = CMSG_LEN(sizeof(int));
  std::memcpy(CMSG_DATA(cmsg), &dmabuf_fd, sizeof(int));

  // EINTR-safe send loop.
  ssize_t sent;
  do {
    sent = ::sendmsg(sock_fd, &msg, MSG_NOSIGNAL);
  } while (sent < 0 && errno == EINTR);

  if (sent < 0) {
    return IsWouldBlock() ? IpcStatus::kWouldBlock : IpcStatus::kSendError;
  }
  // sendmsg is atomic for Unix sockets: it either sends all or errors.
  if (sent != static_cast<ssize_t>(sizeof(FrameMeta))) {
    return IpcStatus::kSendError;
  }

  return IpcStatus::kOk;
}

// ---------------------------------------------------------------------------
// RecvFd — recvmsg with SCM_RIGHTS, EINTR-safe
// ---------------------------------------------------------------------------

IpcStatus RecvFd(int sock_fd, int* out_dmabuf_fd, FrameMeta* out_meta) noexcept {
  if (sock_fd < 0 || !out_dmabuf_fd || !out_meta) return IpcStatus::kInvalidArg;

  *out_dmabuf_fd = -1;

  alignas(struct cmsghdr) char cmsg_buf[CMSG_SPACE(sizeof(int))]{};

  iovec iov{};
  iov.iov_base = out_meta;
  iov.iov_len  = sizeof(FrameMeta);

  msghdr msg{};
  msg.msg_iov        = &iov;
  msg.msg_iovlen     = 1;
  msg.msg_control    = cmsg_buf;
  msg.msg_controllen = sizeof(cmsg_buf);

  // EINTR-safe receive loop.
  ssize_t received;
  do {
    received = ::recvmsg(sock_fd, &msg, 0);
  } while (received < 0 && errno == EINTR);

  if (received == 0) return IpcStatus::kPeerClosed;
  if (received < 0) {
    return IsWouldBlock() ? IpcStatus::kWouldBlock : IpcStatus::kRecvError;
  }
  if (received != static_cast<ssize_t>(sizeof(FrameMeta))) {
    return IpcStatus::kRecvError;
  }

  // Extract the file descriptor from ancillary data.
  const cmsghdr* const cmsg = CMSG_FIRSTHDR(&msg);
  if (!cmsg || cmsg->cmsg_level != SOL_SOCKET ||
      cmsg->cmsg_type != SCM_RIGHTS ||
      cmsg->cmsg_len  != CMSG_LEN(sizeof(int))) {
    return IpcStatus::kNoFdReceived;
  }

  std::memcpy(out_dmabuf_fd, CMSG_DATA(cmsg), sizeof(int));
  if (*out_dmabuf_fd < 0) return IpcStatus::kNoFdReceived;

  return IpcStatus::kOk;
}

// ---------------------------------------------------------------------------
// Socket option utilities
// ---------------------------------------------------------------------------

IpcStatus SetNonBlocking(int fd) noexcept {
  if (fd < 0) return IpcStatus::kInvalidArg;
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0) return IpcStatus::kSysError;
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) return IpcStatus::kSysError;
  return IpcStatus::kOk;
}

IpcStatus SetSocketBufferSize(int fd, int size_bytes) noexcept {
  if (fd < 0 || size_bytes <= 0) return IpcStatus::kInvalidArg;
  if (::setsockopt(fd, SOL_SOCKET, SO_SNDBUF,
                   &size_bytes, sizeof(size_bytes)) < 0) return IpcStatus::kSysError;
  if (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF,
                   &size_bytes, sizeof(size_bytes)) < 0) return IpcStatus::kSysError;
  return IpcStatus::kOk;
}

// ---------------------------------------------------------------------------
// Monotonic clock helper
// ---------------------------------------------------------------------------

uint64_t NowNs() noexcept {
  struct timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace autodriver::ipc
