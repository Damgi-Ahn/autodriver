// test_gpu_watchdog.cpp
//
// Unit tests for GpuWatchdog::Evaluate() — pure logic with no hardware I/O.
// All tests inject synthetic GpuSnapshot values and verify AlertLevel transitions,
// stall accumulator arithmetic, and threshold boundary conditions.

#include <gtest/gtest.h>
#include "gpu_watchdog/gpu_watchdog.hpp"

using namespace autodriver::system;

// ---------------------------------------------------------------------------
// Fixture — creates a GpuWatchdog with test defaults via rclcpp::NodeOptions
// ---------------------------------------------------------------------------
class GpuWatchdogEvalTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      rclcpp::Parameter("poll_rate_hz",      10.0),   // 0.1 s period
      rclcpp::Parameter("stall_threshold_s",  1.0),   // stall after 10 zero-load polls
      rclcpp::Parameter("mem_warn_pct",       90.0),
      rclcpp::Parameter("throttle_temp_c",    85.0),
    });
    watchdog_ = std::make_shared<GpuWatchdog>(opts);
  }

  void TearDown() override {
    watchdog_.reset();
    rclcpp::shutdown();
  }

  GpuSnapshot Normal() const {
    GpuSnapshot s;
    s.gpu_load_pct    = 50.0f;
    s.gpu_temp_c      = 60.0f;
    s.gpu_mem_used_kb = 1000;
    s.gpu_mem_total_kb= 8000;
    s.throttling      = false;
    return s;
  }

  std::shared_ptr<GpuWatchdog> watchdog_;
};

// ---------------------------------------------------------------------------
// Nominal operation
// ---------------------------------------------------------------------------

TEST_F(GpuWatchdogEvalTest, NominalReturnsOk)
{
  EXPECT_EQ(watchdog_->Evaluate(Normal()), AlertLevel::kOk);
}

// ---------------------------------------------------------------------------
// Stall detection
// ---------------------------------------------------------------------------

TEST_F(GpuWatchdogEvalTest, ZeroLoadAccumulatesStall)
{
  GpuSnapshot zero_load = Normal();
  zero_load.gpu_load_pct = 0.0f;

  // poll_rate=10Hz → period=0.1s → need 10 polls to reach 1.0s threshold
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(watchdog_->Evaluate(zero_load), AlertLevel::kOk)
        << "Unexpected ERROR at poll " << i;
  }
  // 10th poll: accumulator >= threshold → ERROR
  EXPECT_EQ(watchdog_->Evaluate(zero_load), AlertLevel::kError);
}

TEST_F(GpuWatchdogEvalTest, StallResetOnNonZeroLoad)
{
  GpuSnapshot zero_load = Normal();
  zero_load.gpu_load_pct = 0.0f;

  // Accumulate near-threshold
  for (int i = 0; i < 8; ++i) watchdog_->Evaluate(zero_load);
  EXPECT_GT(watchdog_->stall_accumulator_s(), 0.0f);

  // Single frame with non-zero load resets accumulator
  EXPECT_EQ(watchdog_->Evaluate(Normal()), AlertLevel::kOk);
  EXPECT_NEAR(watchdog_->stall_accumulator_s(), 0.0f, 1e-6f);
}

TEST_F(GpuWatchdogEvalTest, StallClearedAfterReset)
{
  GpuSnapshot zero_load = Normal();
  zero_load.gpu_load_pct = 0.0f;

  for (int i = 0; i < 15; ++i) watchdog_->Evaluate(zero_load);
  EXPECT_EQ(watchdog_->Evaluate(zero_load), AlertLevel::kError);

  // Recovery
  watchdog_->ResetStallAccumulator();
  EXPECT_EQ(watchdog_->Evaluate(Normal()), AlertLevel::kOk);
}

TEST_F(GpuWatchdogEvalTest, LowLoadBelowOnePercentCountsAsStall)
{
  GpuSnapshot tiny = Normal();
  tiny.gpu_load_pct = 0.9f;  // < 1.0 threshold

  for (int i = 0; i < 9; ++i) watchdog_->Evaluate(tiny);
  EXPECT_EQ(watchdog_->Evaluate(tiny), AlertLevel::kError);
}

// ---------------------------------------------------------------------------
// Thermal throttling
// ---------------------------------------------------------------------------

TEST_F(GpuWatchdogEvalTest, ThermalThrottleReturnsWarn)
{
  GpuSnapshot hot = Normal();
  hot.gpu_temp_c = 85.0f;  // == throttle_temp_c → WARN
  EXPECT_EQ(watchdog_->Evaluate(hot), AlertLevel::kWarn);
}

TEST_F(GpuWatchdogEvalTest, ThermalBelowThresholdIsOk)
{
  GpuSnapshot warm = Normal();
  warm.gpu_temp_c = 84.9f;  // just below threshold
  EXPECT_EQ(watchdog_->Evaluate(warm), AlertLevel::kOk);
}

TEST_F(GpuWatchdogEvalTest, ThrottlingFlagReturnsWarn)
{
  GpuSnapshot throttled = Normal();
  throttled.throttling = true;
  EXPECT_EQ(watchdog_->Evaluate(throttled), AlertLevel::kWarn);
}

// ---------------------------------------------------------------------------
// Memory pressure
// ---------------------------------------------------------------------------

TEST_F(GpuWatchdogEvalTest, MemoryAboveThresholdReturnsWarn)
{
  GpuSnapshot oom = Normal();
  oom.gpu_mem_used_kb  = 9100;  // 91% of 10000
  oom.gpu_mem_total_kb = 10000;
  EXPECT_EQ(watchdog_->Evaluate(oom), AlertLevel::kWarn);
}

TEST_F(GpuWatchdogEvalTest, MemoryAtExactThresholdReturnsWarn)
{
  GpuSnapshot at_thresh = Normal();
  at_thresh.gpu_mem_used_kb  = 9000;  // exactly 90%
  at_thresh.gpu_mem_total_kb = 10000;
  EXPECT_EQ(watchdog_->Evaluate(at_thresh), AlertLevel::kWarn);
}

TEST_F(GpuWatchdogEvalTest, MemoryBelowThresholdIsOk)
{
  GpuSnapshot safe_mem = Normal();
  safe_mem.gpu_mem_used_kb  = 8990;  // 89.9%
  safe_mem.gpu_mem_total_kb = 10000;
  EXPECT_EQ(watchdog_->Evaluate(safe_mem), AlertLevel::kOk);
}

TEST_F(GpuWatchdogEvalTest, ZeroTotalMemoryDoesNotDivideByZero)
{
  GpuSnapshot no_mem = Normal();
  no_mem.gpu_mem_used_kb  = 0;
  no_mem.gpu_mem_total_kb = 0;
  EXPECT_NO_THROW(watchdog_->Evaluate(no_mem));
  EXPECT_EQ(watchdog_->Evaluate(no_mem), AlertLevel::kOk);
}

// ---------------------------------------------------------------------------
// Alert level priority (stall > thermal > memory)
// ---------------------------------------------------------------------------

TEST_F(GpuWatchdogEvalTest, StallOutranksWarn)
{
  GpuSnapshot stall_and_hot;
  stall_and_hot.gpu_load_pct    = 0.0f;
  stall_and_hot.gpu_temp_c      = 90.0f;  // also thermal alert
  stall_and_hot.gpu_mem_used_kb = 9000;
  stall_and_hot.gpu_mem_total_kb= 10000;

  for (int i = 0; i < 10; ++i) watchdog_->Evaluate(stall_and_hot);
  // After threshold, stall ERROR must dominate
  EXPECT_EQ(watchdog_->Evaluate(stall_and_hot), AlertLevel::kError);
}

// ---------------------------------------------------------------------------
// Boundary: Jetson boot — GPU load is 0 briefly, should not immediately stall
// ---------------------------------------------------------------------------
TEST_F(GpuWatchdogEvalTest, BootLoadZeroBelowThresholdIsOk)
{
  GpuSnapshot boot;
  boot.gpu_load_pct = 0.0f;
  boot.gpu_temp_c   = 40.0f;

  // Just below stall threshold (9 polls × 0.1s = 0.9s < 1.0s)
  for (int i = 0; i < 9; ++i)
    EXPECT_EQ(watchdog_->Evaluate(boot), AlertLevel::kOk);
}

