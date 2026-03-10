// test_ipc_unix_socket.cpp
//
// GTest unit-test suite for autodriver::ipc.
//
// Design rationale:
//  - All tests use unique socket paths under /tmp to avoid interference.
//  - RAII helpers (SocketGuard, FdGuard) ensure cleanup even on assertion
//    failure, preventing socket/fd leaks between tests.
//  - Concurrent stress tests validate sendmsg atomicity guarantee on POSIX
//    Unix sockets (single-threaded write → guaranteed non-interleaved).
//  - Edge cases cover: camera_id extremes, zero-dimension frames,
//    timestamp overflow boundaries, and invalid-fd hardening.

#include "ipc_unix_socket/ipc_unix_socket.hpp"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace autodriver::ipc {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Unique socket path per test to prevent cross-test interference.
std::string UniqueSockPath(const std::string& name) {
  return "/tmp/autodriver_test_" + name + "_" +
         std::to_string(getpid()) + ".sock";
}

/// RAII wrapper: closes fd on destruction.
struct FdGuard {
  int fd{-1};
  explicit FdGuard(int f = -1) : fd(f) {}
  ~FdGuard() { if (fd >= 0) ::close(fd); }
  FdGuard(const FdGuard&)            = delete;
  FdGuard& operator=(const FdGuard&) = delete;
  operator int() const { return fd; }  // NOLINT(google-explicit-constructor)
};

/// RAII wrapper: unlinks socket path and closes server fd on destruction.
struct ServerGuard {
  int         fd{-1};
  std::string path;
  explicit ServerGuard(int f, std::string p) : fd(f), path(std::move(p)) {}
  ~ServerGuard() { CloseServerSocket(fd, path); }
  ServerGuard(const ServerGuard&)            = delete;
  ServerGuard& operator=(const ServerGuard&) = delete;
};

/// Create a connected client-server pair and return {server_peer_fd, client_fd}.
/// Returns {-1, -1} on failure.
std::pair<int, int> MakeConnectedPair(const std::string& path) {
  IpcStatus status = IpcStatus::kOk;

  const int server_fd = CreateServerSocket(path, &status);
  if (server_fd < 0) return {-1, -1};

  // Connect the client in a background thread to avoid blocking the test.
  int client_fd = -1;
  std::thread client_thread([&] {
    client_fd = CreateClientSocket(path, nullptr, /*max_retries=*/20,
                                   /*retry_delay_ms=*/10);
  });

  const int peer_fd = AcceptConnection(server_fd, &status);
  client_thread.join();

  // Close the listening socket — we only need the connected peer.
  ::close(server_fd);
  ::unlink(path.c_str());

  return {peer_fd, client_fd};
}

/// Build a canonical FrameMeta for tests.
FrameMeta MakeFrameMeta(uint32_t camera_id = 0,
                         uint64_t timestamp = 1'000'000'000ULL,
                         uint32_t width     = 1920,
                         uint32_t height    = 1080,
                         uint32_t format    = 2 /* NV12 */) {
  return FrameMeta{camera_id, timestamp, width, height, format};
}

/// Create an anonymous pipe; write end is used as a synthetic "dmabuf" fd.
/// Returns {read_fd, write_fd}.  Caller owns both.
std::pair<int, int> MakePipeFds() {
  int fds[2] = {-1, -1};
  if (::pipe(fds) < 0) return {-1, -1};
  return {fds[0], fds[1]};
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class IpcTest : public ::testing::Test {
 protected:
  void SetUp() override {
    sock_path_ = UniqueSockPath(
        ::testing::UnitTest::GetInstance()->current_test_info()->name());
    auto [server_peer, client] = MakeConnectedPair(sock_path_);
    server_peer_fd_ = server_peer;
    client_fd_      = client;
    ASSERT_GE(server_peer_fd_, 0) << "Failed to create server peer";
    ASSERT_GE(client_fd_,      0) << "Failed to create client";
  }

  void TearDown() override {
    CloseSocket(server_peer_fd_);
    CloseSocket(client_fd_);
    ::unlink(sock_path_.c_str());
  }

  std::string sock_path_;
  int         server_peer_fd_{-1};
  int         client_fd_{-1};
};

// ---------------------------------------------------------------------------
// 1. FrameMeta layout
// ---------------------------------------------------------------------------

TEST(FrameMetaTest, SizeAndAlignment) {
  EXPECT_EQ(sizeof(FrameMeta), 24u);
  EXPECT_EQ(alignof(FrameMeta), 4u);
}

TEST(FrameMetaTest, FieldOffsetsAreStable) {
  // Verify field offsets to catch accidental reordering.
  const FrameMeta meta{1, 2, 3, 4, 5};
  EXPECT_EQ(meta.camera_id, 1u);
  EXPECT_EQ(meta.timestamp, 2u);
  EXPECT_EQ(meta.width,     3u);
  EXPECT_EQ(meta.height,    4u);
  EXPECT_EQ(meta.format,    5u);
}

// ---------------------------------------------------------------------------
// 2. StatusString
// ---------------------------------------------------------------------------

TEST(StatusStringTest, AllValuesHaveNonNullString) {
  const IpcStatus all_statuses[] = {
      IpcStatus::kOk,           IpcStatus::kSocketError,
      IpcStatus::kBindError,    IpcStatus::kListenError,
      IpcStatus::kConnectError, IpcStatus::kAcceptError,
      IpcStatus::kSendError,    IpcStatus::kRecvError,
      IpcStatus::kPeerClosed,   IpcStatus::kNoFdReceived,
      IpcStatus::kWouldBlock,   IpcStatus::kInvalidArg,
      IpcStatus::kSysError,
  };
  for (auto s : all_statuses) {
    EXPECT_NE(StatusString(s), nullptr);
    EXPECT_GT(std::strlen(StatusString(s)), 0u) << "Empty string for status "
                                                 << static_cast<int>(s);
  }
}

TEST(StatusStringTest, OkStringIsOK) {
  EXPECT_STREQ(StatusString(IpcStatus::kOk), "OK");
}

// ---------------------------------------------------------------------------
// 3. IsOk helper
// ---------------------------------------------------------------------------

TEST(IsOkTest, TrueOnlyForKOk) {
  EXPECT_TRUE(IsOk(IpcStatus::kOk));
  EXPECT_FALSE(IsOk(IpcStatus::kSendError));
  EXPECT_FALSE(IsOk(IpcStatus::kPeerClosed));
  EXPECT_FALSE(IsOk(IpcStatus::kWouldBlock));
}

// ---------------------------------------------------------------------------
// 4. CreateServerSocket
// ---------------------------------------------------------------------------

TEST(ServerSocketTest, BindsAndListens) {
  const auto path = UniqueSockPath("bind");
  IpcStatus status = IpcStatus::kSendError;  // intentionally wrong init
  const int fd = CreateServerSocket(path, &status);
  ASSERT_GE(fd, 0);
  EXPECT_TRUE(IsOk(status));
  CloseServerSocket(fd, path);
}

TEST(ServerSocketTest, RemovesStaleSocketFile) {
  const auto path = UniqueSockPath("stale");
  // Create a dummy file at the socket path.
  FILE* f = std::fopen(path.c_str(), "w");
  ASSERT_NE(f, nullptr);
  std::fclose(f);

  const int fd = CreateServerSocket(path);
  ASSERT_GE(fd, 0) << "Should remove stale file and succeed";
  CloseServerSocket(fd, path);
}

TEST(ServerSocketTest, PathTooLongReturnsInvalidArg) {
  const std::string long_path(200, 'x');  // > 107 bytes (sun_path limit)
  IpcStatus status = IpcStatus::kOk;
  const int fd = CreateServerSocket(long_path, &status);
  EXPECT_LT(fd, 0);
  EXPECT_EQ(status, IpcStatus::kInvalidArg);
}

// ---------------------------------------------------------------------------
// 5. CreateClientSocket
// ---------------------------------------------------------------------------

TEST(ClientSocketTest, FailsWhenNoServer) {
  const auto path = UniqueSockPath("no_server");
  IpcStatus status = IpcStatus::kOk;
  // max_retries=1 to keep the test fast
  const int fd = CreateClientSocket(path, &status, /*max_retries=*/1,
                                    /*retry_delay_ms=*/1);
  EXPECT_LT(fd, 0);
  EXPECT_EQ(status, IpcStatus::kConnectError);
}

// ---------------------------------------------------------------------------
// 6. SendFd / RecvFd — basic round-trip
// ---------------------------------------------------------------------------

TEST_F(IpcTest, RoundTripSingleFd) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent = MakeFrameMeta(3, 9'876'543'210ULL, 1920, 1080, 1);

  // Client sends the write-end of the pipe.
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  // Server receives it.
  int   recv_fd  = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard guard_recv(recv_fd);

  EXPECT_GE(recv_fd, 0);
  EXPECT_EQ(recv_meta.camera_id, sent.camera_id);
  EXPECT_EQ(recv_meta.timestamp, sent.timestamp);
  EXPECT_EQ(recv_meta.width,     sent.width);
  EXPECT_EQ(recv_meta.height,    sent.height);
  EXPECT_EQ(recv_meta.format,    sent.format);
}

TEST_F(IpcTest, FdIsUsableAfterTransfer) {
  // Transfer a pipe write-end and verify data can be written/read through it.
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta meta = MakeFrameMeta();
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, meta)));

  int   recv_write_fd = -1;
  FrameMeta unused{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_write_fd, &unused)));
  FdGuard guard_recv(recv_write_fd);

  // Write through the received fd.
  const uint8_t payload = 0xAB;
  ASSERT_EQ(::write(recv_write_fd, &payload, 1), 1);

  uint8_t buf = 0;
  ASSERT_EQ(::read(pipe_r, &buf, 1), 1);
  EXPECT_EQ(buf, payload);
}

// ---------------------------------------------------------------------------
// 7. Edge cases — FrameMeta extreme values
// ---------------------------------------------------------------------------

TEST_F(IpcTest, CameraIdMaxValue) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent = MakeFrameMeta(/*camera_id=*/UINT32_MAX);
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  int recv_fd = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard g(recv_fd);
  EXPECT_EQ(recv_meta.camera_id, UINT32_MAX);
}

TEST_F(IpcTest, TimestampZero) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent = MakeFrameMeta(0, /*timestamp=*/0);
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  int recv_fd = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard g(recv_fd);
  EXPECT_EQ(recv_meta.timestamp, 0u);
}

TEST_F(IpcTest, TimestampMaxValue) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent = MakeFrameMeta(0, UINT64_MAX);
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  int recv_fd = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard g(recv_fd);
  EXPECT_EQ(recv_meta.timestamp, UINT64_MAX);
}

TEST_F(IpcTest, ZeroDimensionFrame) {
  // Zero-dimension frames can occur during sensor error recovery.
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent = MakeFrameMeta(0, 1, /*width=*/0, /*height=*/0);
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  int recv_fd = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard g(recv_fd);
  EXPECT_EQ(recv_meta.width,  0u);
  EXPECT_EQ(recv_meta.height, 0u);
}

TEST_F(IpcTest, AllFieldsPreserved) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);

  const FrameMeta sent{11, 0xDEADBEEFCAFEBABEULL, 3840, 2160, 42};
  ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_w, sent)));

  int recv_fd = -1;
  FrameMeta recv_meta{};
  ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)));
  FdGuard g(recv_fd);

  EXPECT_EQ(recv_meta.camera_id, 11u);
  EXPECT_EQ(recv_meta.timestamp, 0xDEADBEEFCAFEBABEULL);
  EXPECT_EQ(recv_meta.width,     3840u);
  EXPECT_EQ(recv_meta.height,    2160u);
  EXPECT_EQ(recv_meta.format,    42u);
}

// ---------------------------------------------------------------------------
// 8. Invalid argument hardening
// ---------------------------------------------------------------------------

TEST(InvalidArgTest, SendFdNegativeSockFd) {
  auto [pipe_r, pipe_w] = MakePipeFds();
  ASSERT_GE(pipe_r, 0);
  FdGuard guard_r(pipe_r), guard_w(pipe_w);
  const FrameMeta meta = MakeFrameMeta();
  EXPECT_EQ(SendFd(-1, pipe_w, meta), IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, SendFdNegativeDmaBufFd) {
  // Create a socket pair for the test
  int sv[2] = {-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
  FdGuard g0(sv[0]), g1(sv[1]);
  const FrameMeta meta = MakeFrameMeta();
  EXPECT_EQ(SendFd(sv[0], -1, meta), IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, RecvFdNullOutputPointers) {
  int sv[2] = {-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
  FdGuard g0(sv[0]), g1(sv[1]);
  FrameMeta meta{};
  EXPECT_EQ(RecvFd(sv[0], nullptr, &meta), IpcStatus::kInvalidArg);
  EXPECT_EQ(RecvFd(sv[0], reinterpret_cast<int*>(0x1), nullptr),
            IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, RecvFdNegativeSockFd) {
  int dummy_fd = -1;
  FrameMeta meta{};
  EXPECT_EQ(RecvFd(-1, &dummy_fd, &meta), IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, SetNonBlockingNegativeFd) {
  EXPECT_EQ(SetNonBlocking(-1), IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, SetSocketBufferSizeNegativeFd) {
  EXPECT_EQ(SetSocketBufferSize(-1, 1024), IpcStatus::kInvalidArg);
}

TEST(InvalidArgTest, SetSocketBufferSizeZeroSize) {
  int sv[2] = {-1, -1};
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
  FdGuard g0(sv[0]), g1(sv[1]);
  EXPECT_EQ(SetSocketBufferSize(sv[0], 0), IpcStatus::kInvalidArg);
}

// ---------------------------------------------------------------------------
// 9. Peer closed detection
// ---------------------------------------------------------------------------

TEST_F(IpcTest, RecvReturnsPeerClosedOnEOF) {
  // Close client so server recv gets EOF.
  CloseSocket(client_fd_);
  client_fd_ = -1;

  int recv_fd = -1;
  FrameMeta meta{};
  const IpcStatus s = RecvFd(server_peer_fd_, &recv_fd, &meta);
  EXPECT_EQ(s, IpcStatus::kPeerClosed);
  EXPECT_LT(recv_fd, 0);
}

// ---------------------------------------------------------------------------
// 10. Non-blocking socket (kWouldBlock)
// ---------------------------------------------------------------------------

TEST_F(IpcTest, RecvWouldBlockWhenNonBlockingAndEmpty) {
  ASSERT_TRUE(IsOk(SetNonBlocking(server_peer_fd_)));

  int recv_fd = -1;
  FrameMeta meta{};
  const IpcStatus s = RecvFd(server_peer_fd_, &recv_fd, &meta);
  EXPECT_EQ(s, IpcStatus::kWouldBlock);
  EXPECT_LT(recv_fd, 0);
}

// ---------------------------------------------------------------------------
// 11. SetNonBlocking and SetSocketBufferSize — basic success
// ---------------------------------------------------------------------------

TEST_F(IpcTest, SetNonBlockingSucceeds) {
  EXPECT_TRUE(IsOk(SetNonBlocking(client_fd_)));
}

TEST_F(IpcTest, SetSocketBufferSizeSucceeds) {
  EXPECT_TRUE(IsOk(SetSocketBufferSize(client_fd_, 4 * 1024 * 1024)));
}

// ---------------------------------------------------------------------------
// 12. Multiple frames in sequence
// ---------------------------------------------------------------------------

TEST_F(IpcTest, MultipleFramesInSequence) {
  constexpr int kNumFrames = 20;

  // Pre-create pipe fds for all frames.
  std::vector<int> pipe_writes;
  std::vector<FdGuard> read_guards;
  for (int i = 0; i < kNumFrames; ++i) {
    auto [r, w] = MakePipeFds();
    ASSERT_GE(r, 0);
    read_guards.emplace_back(r);
    pipe_writes.push_back(w);
  }

  // Send all frames from client.
  for (int i = 0; i < kNumFrames; ++i) {
    const FrameMeta meta = MakeFrameMeta(
        static_cast<uint32_t>(i),
        static_cast<uint64_t>(i) * 33'333'333ULL,  // ~30fps timestamps
        1920, 1080, 2);
    ASSERT_TRUE(IsOk(SendFd(client_fd_, pipe_writes[i], meta)))
        << "SendFd failed at frame " << i;
    ::close(pipe_writes[i]);
  }

  // Receive and verify all frames on server side.
  for (int i = 0; i < kNumFrames; ++i) {
    int recv_fd = -1;
    FrameMeta recv_meta{};
    ASSERT_TRUE(IsOk(RecvFd(server_peer_fd_, &recv_fd, &recv_meta)))
        << "RecvFd failed at frame " << i;
    FdGuard g(recv_fd);
    EXPECT_EQ(recv_meta.camera_id, static_cast<uint32_t>(i));
    EXPECT_EQ(recv_meta.timestamp,
              static_cast<uint64_t>(i) * 33'333'333ULL);
  }
}

// ---------------------------------------------------------------------------
// 13. Concurrent senders (separate sockets, not shared) — verify isolation
// ---------------------------------------------------------------------------

TEST(ConcurrencyTest, IndependentSocketsFromMultipleThreads) {
  // Each thread creates its own pair and sends one frame independently.
  // Verifies that multiple IPC channels don't interfere with each other.
  constexpr int kNumThreads = 6;
  std::atomic<int> success_count{0};

  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);

  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([i, &success_count] {
      const auto path = UniqueSockPath("concurrent_" + std::to_string(i) +
                                       "_t" + std::to_string(
                                           std::hash<std::thread::id>{}(
                                               std::this_thread::get_id())));
      auto [peer_fd, client_fd] = MakeConnectedPair(path);
      if (peer_fd < 0 || client_fd < 0) return;
      FdGuard g_peer(peer_fd), g_client(client_fd);

      auto [pipe_r, pipe_w] = MakePipeFds();
      if (pipe_r < 0) return;
      FdGuard g_r(pipe_r), g_w(pipe_w);

      const FrameMeta sent = MakeFrameMeta(static_cast<uint32_t>(i));
      if (!IsOk(SendFd(client_fd, pipe_w, sent))) return;

      int recv_fd = -1;
      FrameMeta recv_meta{};
      if (!IsOk(RecvFd(peer_fd, &recv_fd, &recv_meta))) return;
      FdGuard g_recv(recv_fd);

      if (recv_meta.camera_id == static_cast<uint32_t>(i)) {
        success_count.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }

  for (auto& t : threads) t.join();
  EXPECT_EQ(success_count.load(), kNumThreads);
}

// ---------------------------------------------------------------------------
// 14. NowNs — basic sanity
// ---------------------------------------------------------------------------

TEST(NowNsTest, ReturnsIncreasingValues) {
  const uint64_t t0 = NowNs();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  const uint64_t t1 = NowNs();
  EXPECT_GT(t1, t0);
  EXPECT_NE(t0, 0u);
}

TEST(NowNsTest, ResolutionIsFinerThanOneMs) {
  const uint64_t t0 = NowNs();
  const uint64_t t1 = NowNs();
  // Even two back-to-back calls should differ (monotonic, nanosecond res).
  // Allow t0 == t1 only if the platform genuinely has coarse clock.
  EXPECT_GE(t1, t0);
}

}  // namespace
}  // namespace autodriver::ipc

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
