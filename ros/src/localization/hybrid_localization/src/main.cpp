#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "hybrid_localization/hybrid_localization_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<hybrid_localization::HybridLocalizationNode>(options);

  // 3개 스레드: IMU 전용(200Hz) / 저주파 센서 / 발행 타이머
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
