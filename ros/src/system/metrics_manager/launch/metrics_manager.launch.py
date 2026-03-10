from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="1.0",
        description="Metrics publish rate in Hz",
    )

    node = Node(
        package="metrics_manager",
        executable="metrics_manager_node",
        name="metrics_manager",
        output="screen",
        parameters=[{"publish_rate_hz": LaunchConfiguration("publish_rate_hz")}],
    )

    return LaunchDescription([publish_rate_arg, node])
