from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("camera_manager")

    cameras_yaml_arg = DeclareLaunchArgument(
        "cameras_yaml",
        default_value=PathJoinSubstitution([pkg_share, "config", "cameras.yaml"]),
        description="Path to cameras.yaml",
    )
    ipc_socket_arg = DeclareLaunchArgument(
        "ipc_socket_path",
        default_value="/tmp/autodriver/frames.sock",
        description="Unix domain socket path for DMABUF IPC",
    )
    debug_stream_arg = DeclareLaunchArgument(
        "debug_stream",
        default_value="false",
        description="Enable NVJPEG debug image stream",
    )

    camera_manager_node = Node(
        package="camera_manager",
        executable="camera_manager_node",
        name="camera_manager",
        output="screen",
        parameters=[
            {
                "cameras_yaml": LaunchConfiguration("cameras_yaml"),
                "ipc_socket_path": LaunchConfiguration("ipc_socket_path"),
                "debug_stream": LaunchConfiguration("debug_stream"),
            }
        ],
    )

    return LaunchDescription([
        cameras_yaml_arg,
        ipc_socket_arg,
        debug_stream_arg,
        camera_manager_node,
    ])
