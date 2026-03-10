// test_scheduler.cpp
//
// Unit tests for HybridScheduler.
// Uses SubmitPreImported() to inject QueuedFrames directly — bypasses all
// GPU (NvBufSurface) calls so tests run on any host without Jetson hardware.
//
// Test categories:
//   Routing     — subscribed_cameras filtering
//   Batching    — fires when queue reaches batch_size
//   Timeout     — fires when timeout_ms elapses before batch_size is reached
//   Concurrent  — frames from multiple "camera" threads interleaved
//   EdgeCases   — empty batch, single-camera model, pool-exhausted scenario

#include <gtest/gtest.h>

#include "tensorrt_inference_manager/scheduler.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

using namespace autodriver::inference;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// MockPool — stub that satisfies NvBufSurfacePool interface without GPU.
// ---------------------------------------------------------------------------
class MockPool : public NvBufSurfacePool {
 public:
  MockPool() : NvBufSurfacePool(128, 1920, 1080) {}

  // Override Init so no real NvBufSurface allocations happen.
  bool Init() noexcept override { return true; }

  // Simulate AcquireSlot without real surfaces.
  uint32_t AcquireSlot(int /*timeout_ms*/) noexcept override {
    if (free_count_mock_.load() == 0) return kInvalidSlot;
    free_count_mock_.fetch_sub(1);
    return next_slot_.fetch_add(1) % 128;
  }

  void ReleaseSlot(uint32_t /*slot*/) noexcept override {
    free_count_mock_.fetch_add(1);
  }

  NvBufSurface* Surface(uint32_t /*slot*/) const noexcept override {
    return nullptr;
  }

  static NvBufSurface* ImportFd(int /*dmabuf_fd*/) noexcept { return nullptr; }

  uint32_t FreeCount() const noexcept override {
    return free_count_mock_.load();
  }

 private:
  std::atomic<uint32_t> free_count_mock_{128};
  std::atomic<uint32_t> next_slot_{0};
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static QueuedFrame MakeFrame(uint32_t camera_id, uint32_t slot = 0)
{
  QueuedFrame f;
  f.dmabuf_fd  = -1;
  f.meta.camera_id = camera_id;
  f.pool_slot  = slot;
  f.enqueue_ns = 0;
  return f;
}

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------

class SchedulerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    pool_      = std::make_unique<MockPool>();
    scheduler_ = std::make_unique<HybridScheduler>(*pool_);
  }

  void TearDown() override {
    if (scheduler_) scheduler_->Stop();
  }

  void AddQueue(const std::string& model,
                uint32_t batch_size,
                uint32_t timeout_ms,
                std::vector<uint32_t> cameras = {})
  {
    ModelQueueConfig cfg;
    cfg.model_name         = model;
    cfg.batch_size         = batch_size;
    cfg.timeout_ms         = timeout_ms;
    cfg.subscribed_cameras = cameras;
    scheduler_->AddModelQueue(cfg);
  }

  struct BatchCapture {
    std::mutex                       mtx;
    std::condition_variable          cv;
    std::vector<std::vector<QueuedFrame>> batches;

    void Append(std::vector<QueuedFrame> b) {
      std::lock_guard<std::mutex> lk(mtx);
      batches.push_back(std::move(b));
      cv.notify_all();
    }

    bool WaitForCount(size_t n, std::chrono::milliseconds timeout) {
      std::unique_lock<std::mutex> lk(mtx);
      return cv.wait_for(lk, timeout,
                         [this, n] { return batches.size() >= n; });
    }
  };

  std::unique_ptr<MockPool>         pool_;
  std::unique_ptr<HybridScheduler>  scheduler_;
};

// ---------------------------------------------------------------------------
// Routing tests
// ---------------------------------------------------------------------------

TEST_F(SchedulerTest, RoutesAllCamerasWhenSubscribedCamerasEmpty)
{
  BatchCapture cap;
  AddQueue("yolo", /*batch_size=*/2, /*timeout_ms=*/100);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  scheduler_->SubmitPreImported(MakeFrame(0));
  scheduler_->SubmitPreImported(MakeFrame(7));  // any camera_id

  ASSERT_TRUE(cap.WaitForCount(1, 500ms));
  EXPECT_EQ(scheduler_->total_frames_routed(), 2u);
}

TEST_F(SchedulerTest, FiltersFramesBySubscribedCameras)
{
  BatchCapture cap;
  // Only subscribe to camera 0 and 1; camera 3 should be ignored.
  AddQueue("lane", /*batch_size=*/2, /*timeout_ms=*/100, {0, 1});
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  scheduler_->SubmitPreImported(MakeFrame(3));  // ignored
  scheduler_->SubmitPreImported(MakeFrame(0));
  scheduler_->SubmitPreImported(MakeFrame(1));

  ASSERT_TRUE(cap.WaitForCount(1, 500ms));
  EXPECT_EQ(scheduler_->total_frames_routed(), 2u);
}

TEST_F(SchedulerTest, MultiModelIndependentRouting)
{
  // yolo: all cameras; lane: camera 0 only
  BatchCapture yolo_cap, lane_cap;
  AddQueue("yolo", 2, 200);
  AddQueue("lane", 2, 200, {0});

  scheduler_->SetBatchReadyCallback(
      [&yolo_cap, &lane_cap](const std::string& model,
                              std::vector<QueuedFrame> b) {
        if (model == "yolo") yolo_cap.Append(std::move(b));
        else                  lane_cap.Append(std::move(b));
      });
  scheduler_->Start();

  // camera 0: routes to both
  scheduler_->SubmitPreImported(MakeFrame(0));
  scheduler_->SubmitPreImported(MakeFrame(0));
  // camera 5: routes to yolo only
  scheduler_->SubmitPreImported(MakeFrame(5));
  scheduler_->SubmitPreImported(MakeFrame(5));

  ASSERT_TRUE(yolo_cap.WaitForCount(2, 500ms));
  ASSERT_TRUE(lane_cap.WaitForCount(1, 500ms));

  EXPECT_EQ(yolo_cap.batches.size(), 2u);
  EXPECT_EQ(lane_cap.batches.size(), 1u);
}

// ---------------------------------------------------------------------------
// Batching tests
// ---------------------------------------------------------------------------

TEST_F(SchedulerTest, FiresBatchWhenQueueReachesBatchSize)
{
  BatchCapture cap;
  AddQueue("yolo", /*batch_size=*/4, /*timeout_ms=*/5000);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  for (int i = 0; i < 4; ++i)
    scheduler_->SubmitPreImported(MakeFrame(0, static_cast<uint32_t>(i)));

  ASSERT_TRUE(cap.WaitForCount(1, 500ms));
  ASSERT_EQ(cap.batches.size(), 1u);
  EXPECT_EQ(cap.batches[0].size(), 4u);
}

TEST_F(SchedulerTest, MultipleBatchesFiredForLargeSubmission)
{
  BatchCapture cap;
  AddQueue("seg", /*batch_size=*/2, /*timeout_ms=*/5000);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  for (int i = 0; i < 6; ++i)
    scheduler_->SubmitPreImported(MakeFrame(0));

  ASSERT_TRUE(cap.WaitForCount(3, 500ms));
  EXPECT_EQ(cap.batches.size(), 3u);
}

// ---------------------------------------------------------------------------
// Timeout tests
// ---------------------------------------------------------------------------

TEST_F(SchedulerTest, FiresPartialBatchOnTimeout)
{
  BatchCapture cap;
  // batch_size=4 but only 1 frame submitted → timeout fires partial batch
  AddQueue("yolo", /*batch_size=*/4, /*timeout_ms=*/30);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  scheduler_->SubmitPreImported(MakeFrame(0));

  ASSERT_TRUE(cap.WaitForCount(1, 300ms));
  ASSERT_EQ(cap.batches.size(), 1u);
  EXPECT_EQ(cap.batches[0].size(), 1u);  // partial batch
}

TEST_F(SchedulerTest, TimeoutResetAfterPartialBatch)
{
  // After a timeout-fired partial batch, subsequent frames should form a new batch.
  BatchCapture cap;
  AddQueue("yolo", /*batch_size=*/4, /*timeout_ms=*/30);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  // First partial batch
  scheduler_->SubmitPreImported(MakeFrame(0));
  ASSERT_TRUE(cap.WaitForCount(1, 300ms));

  // Second partial batch after another timeout
  scheduler_->SubmitPreImported(MakeFrame(1));
  ASSERT_TRUE(cap.WaitForCount(2, 300ms));

  EXPECT_EQ(cap.batches.size(), 2u);
}

// ---------------------------------------------------------------------------
// Concurrent submission tests
// ---------------------------------------------------------------------------

TEST_F(SchedulerTest, ConcurrentSubmissionsAreAllRouted)
{
  constexpr int kFrameCount  = 100;
  constexpr int kThreadCount = 4;

  BatchCapture cap;
  AddQueue("yolo", /*batch_size=*/5, /*timeout_ms=*/50);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  // Submit kFrameCount frames from kThreadCount parallel threads.
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreadCount; ++t) {
    threads.emplace_back([this, kFrameCount] {
      for (int i = 0; i < kFrameCount / 4; ++i)
        scheduler_->SubmitPreImported(MakeFrame(0));
    });
  }
  for (auto& th : threads) th.join();

  // All frames must have been routed (some via batching, remainder via timeout).
  ASSERT_TRUE(cap.WaitForCount(kFrameCount / 5, 2000ms));
  EXPECT_EQ(scheduler_->total_frames_routed(),
            static_cast<uint64_t>(kFrameCount));
}

// ---------------------------------------------------------------------------
// Edge cases
// ---------------------------------------------------------------------------

TEST_F(SchedulerTest, NoCallbackDoesNotCrash)
{
  AddQueue("yolo", 2, 100);
  // No callback set intentionally.
  scheduler_->Start();
  scheduler_->SubmitPreImported(MakeFrame(0));
  scheduler_->SubmitPreImported(MakeFrame(0));
  std::this_thread::sleep_for(150ms);  // let drain thread run
  SUCCEED();
}

TEST_F(SchedulerTest, StopWithPendingFramesDoesNotDeadlock)
{
  BatchCapture cap;
  AddQueue("yolo", /*batch_size=*/100, /*timeout_ms=*/10000);
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  // Submit frames but batch_size will never be reached before Stop().
  for (int i = 0; i < 10; ++i)
    scheduler_->SubmitPreImported(MakeFrame(0));

  // Stop should unblock drain thread cleanly.
  ASSERT_NO_THROW(scheduler_->Stop());
}

TEST_F(SchedulerTest, ZeroCamerasSubscribedReceivesNothingFromNonMatchingCamera)
{
  // Model subscribes to camera 99, frame from camera 0 → not routed.
  BatchCapture cap;
  AddQueue("lane", 1, 50, {99});
  scheduler_->SetBatchReadyCallback(
      [&cap](const std::string&, std::vector<QueuedFrame> b) {
        cap.Append(std::move(b));
      });
  scheduler_->Start();

  scheduler_->SubmitPreImported(MakeFrame(0));
  scheduler_->SubmitPreImported(MakeFrame(1));

  std::this_thread::sleep_for(150ms);
  EXPECT_EQ(scheduler_->total_frames_routed(), 0u);
  EXPECT_EQ(cap.batches.size(), 0u);
}

TEST_F(SchedulerTest, HighFrequencyFramesDontDropBatches)
{
  // Autonomous driving: 30fps × 8 cameras = 240 frames/sec burst
  constexpr int kFrames = 240;
  std::atomic<int> received{0};

  AddQueue("seg", /*batch_size=*/4, /*timeout_ms=*/50);
  scheduler_->SetBatchReadyCallback(
      [&received](const std::string&, std::vector<QueuedFrame> b) {
        received.fetch_add(static_cast<int>(b.size()));
      });
  scheduler_->Start();

  for (int i = 0; i < kFrames; ++i)
    scheduler_->SubmitPreImported(MakeFrame(static_cast<uint32_t>(i % 8)));

  // Wait for all frames to be consumed (batched + remainder via timeout).
  const auto deadline = std::chrono::steady_clock::now() + 2000ms;
  while (received.load() < kFrames &&
         std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(10ms);

  EXPECT_EQ(received.load(), kFrames);
}

