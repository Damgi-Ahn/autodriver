#include "hybrid_localization_evaluation_tool/diagnostic_parser.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>

namespace autodriver::tools {

TEST(DiagnosticParserTest, ParsesHybridLocalizationStatus)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "hybrid_localization";
  status.values.resize(3);
  status.values[0].key = "is_activated";
  status.values[0].value = "true";
  status.values[1].key = "gnss_pos_update_applied";
  status.values[1].value = "1";
  status.values[2].key = "gnss_pos_nis";
  status.values[2].value = "12.5";

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.status.push_back(status);

  DiagnosticParser parser;
  DiagSample sample;
  std::string error;
  ASSERT_TRUE(parser.Parse(msg, &sample, &error));
  EXPECT_TRUE(sample.is_activated);
  EXPECT_TRUE(sample.has_gnss_pos_update_applied);
  EXPECT_TRUE(sample.gnss_pos_update_applied);
  ASSERT_TRUE(sample.gnss_pos_nis.has_value());
  EXPECT_DOUBLE_EQ(sample.gnss_pos_nis.value(), 12.5);
}

TEST(DiagnosticParserTest, MissingStatusReturnsFalse)
{
  diagnostic_msgs::msg::DiagnosticArray msg;
  DiagnosticParser parser;
  DiagSample sample;
  std::string error;
  EXPECT_FALSE(parser.Parse(msg, &sample, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace autodriver::tools
