from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("poll_rate_hz",      default_value="2.0"),
        DeclareLaunchArgument("stall_threshold_s", default_value="5.0"),
        DeclareLaunchArgument("mem_warn_pct",      default_value="90.0"),
        DeclareLaunchArgument("throttle_temp_c",   default_value="85.0"),
    ]

    node = Node(
        package="gpu_watchdog",
        executable="gpu_watchdog_node",
        name="gpu_watchdog",
        output="screen",
        parameters=[{
            "poll_rate_hz":      LaunchConfiguration("poll_rate_hz"),
            "stall_threshold_s": LaunchConfiguration("stall_threshold_s"),
            "mem_warn_pct":      LaunchConfiguration("mem_warn_pct"),
            "throttle_temp_c":   LaunchConfiguration("throttle_temp_c"),
        }],
    )

    return LaunchDescription(args + [node])
