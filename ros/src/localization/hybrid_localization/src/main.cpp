#include <rclcpp/rclcpp.hpp>
#include "hybrid_localization/hybrid_localization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<hybrid_localization::HybridLocalizationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
