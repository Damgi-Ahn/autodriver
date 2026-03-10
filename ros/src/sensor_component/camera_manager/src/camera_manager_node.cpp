#include "camera_manager/camera_manager.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // GStreamer must be initialised before any GstElement is created.
  // Passing nullptr, nullptr so GStreamer ignores argv/argc.
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  // Allow parameter overrides from launch files and ROS2 CLI.
  options.automatically_declare_parameters_from_overrides(false);

  try {
    auto node = std::make_shared<autodriver::camera::CameraManager>(options);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_manager"),
                 "Fatal error: %s", e.what());
    gst_deinit();
    rclcpp::shutdown();
    return 1;
  }

  gst_deinit();
  rclcpp::shutdown();
  return 0;
}
