#include "hybrid_localization_evaluation_tool/kpi_engine.hpp"

#include <gtest/gtest.h>

namespace autodriver::tools {

TEST(KpiEngineTest, ComputesUpdateRateAndOutputAvailability)
{
  KpiEngine engine(10.0, 5.0);

  DiagSample sample;
  sample.stamp = rclcpp::Time(1, 0);
  sample.has_gnss_pos_update_applied = true;
  sample.gnss_pos_update_applied = true;
  sample.gnss_pos_update_reason = "ok";
  sample.gnss_pos_nis = 2.0;
  engine.AddSample(sample);

  sample.stamp = rclcpp::Time(2, 0);
  sample.gnss_pos_update_applied = false;
  sample.gnss_pos_update_reason = "skip";
  sample.gnss_pos_nis = 4.0;
  engine.AddSample(sample);

  engine.AddOutputStamp(rclcpp::Time(1, 0));
  engine.AddOutputStamp(rclcpp::Time(2, 0));

  const auto snapshot = engine.ComputeSnapshot(rclcpp::Time(3, 0));
  EXPECT_EQ(snapshot.gnss_pos_update.total, 2u);
  EXPECT_EQ(snapshot.gnss_pos_update.applied, 1u);
  EXPECT_NEAR(snapshot.gnss_pos_update.ratio, 0.5, 1e-6);

  EXPECT_EQ(snapshot.gnss_pos_nis.count, 2u);
  EXPECT_NEAR(snapshot.gnss_pos_nis.mean, 3.0, 1e-6);

  EXPECT_TRUE(snapshot.output_availability.has_output);
  EXPECT_EQ(snapshot.output_availability.output_count, 2u);
  EXPECT_NEAR(snapshot.output_availability.expected_count, 50.0, 1e-6);
  EXPECT_NEAR(snapshot.output_availability.ratio, 0.04, 1e-6);
}

}  // namespace autodriver::tools
