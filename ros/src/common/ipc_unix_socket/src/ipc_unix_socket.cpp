#include "ipc_unix_socket/ipc_unix_socket.hpp"

#include <sys/socket.h>
#include <sys/un.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <chrono>

namespace autodriver::ipc {

// ---------------------------------------------------------------------------
// Server
// ---------------------------------------------------------------------------

int create_server_socket(const std::string& path)
{
    // Remove stale socket file
    ::unlink(path.c_str());

    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    ::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        ::close(fd);
        return -1;
    }

    if (::listen(fd, /*backlog=*/16) < 0) {
        ::close(fd);
        return -1;
    }

    return fd;
}

int accept_connection(int server_fd)
{
    return ::accept(server_fd, nullptr, nullptr);
}

void close_server_socket(int fd, const std::string& path)
{
    if (fd >= 0) ::close(fd);
    ::unlink(path.c_str());
}

// ---------------------------------------------------------------------------
// Client
// ---------------------------------------------------------------------------

int create_client_socket(const std::string& path, int max_retries, int retry_delay_ms)
{
    int fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    ::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    for (int attempt = 0; attempt < max_retries; ++attempt) {
        if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0) {
            return fd;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
    }

    ::close(fd);
    return -1;
}

void close_socket(int fd)
{
    if (fd >= 0) ::close(fd);
}

// ---------------------------------------------------------------------------
// SCM_RIGHTS FD transfer
// ---------------------------------------------------------------------------

bool send_fd(int sock_fd, int dmabuf_fd, const FrameMeta& meta)
{
    // Ancillary data buffer sized for one fd
    alignas(struct cmsghdr)
    char cmsg_buf[CMSG_SPACE(sizeof(int))]{};

    iovec iov{};
    iov.iov_base = const_cast<FrameMeta*>(&meta);
    iov.iov_len  = sizeof(FrameMeta);

    msghdr msg{};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);

    cmsghdr* cmsg   = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type  = SCM_RIGHTS;
    cmsg->cmsg_len   = CMSG_LEN(sizeof(int));
    ::memcpy(CMSG_DATA(cmsg), &dmabuf_fd, sizeof(int));

    ssize_t sent = ::sendmsg(sock_fd, &msg, MSG_NOSIGNAL);
    return sent == static_cast<ssize_t>(sizeof(FrameMeta));
}

bool recv_fd(int sock_fd, int* dmabuf_fd, FrameMeta* meta)
{
    alignas(struct cmsghdr)
    char cmsg_buf[CMSG_SPACE(sizeof(int))]{};

    iovec iov{};
    iov.iov_base = meta;
    iov.iov_len  = sizeof(FrameMeta);

    msghdr msg{};
    msg.msg_iov        = &iov;
    msg.msg_iovlen     = 1;
    msg.msg_control    = cmsg_buf;
    msg.msg_controllen = sizeof(cmsg_buf);

    ssize_t received = ::recvmsg(sock_fd, &msg, 0);
    if (received != static_cast<ssize_t>(sizeof(FrameMeta))) return false;

    cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    if (!cmsg || cmsg->cmsg_type != SCM_RIGHTS) return false;

    ::memcpy(dmabuf_fd, CMSG_DATA(cmsg), sizeof(int));
    return true;
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

bool set_nonblocking(int fd)
{
    int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return false;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool set_socket_buffer_size(int fd, int size_bytes)
{
    bool ok = true;
    ok &= (::setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &size_bytes, sizeof(size_bytes)) == 0);
    ok &= (::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &size_bytes, sizeof(size_bytes)) == 0);
    return ok;
}

} // namespace autodriver::ipc
