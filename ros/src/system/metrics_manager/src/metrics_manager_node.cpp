#include "metrics_manager/metrics_manager.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<autodriver::system::MetricsManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
