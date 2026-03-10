#include "tensorrt_inference_manager/inference_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<autodriver::inference::InferenceManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
