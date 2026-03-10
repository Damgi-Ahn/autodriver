from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tensorrt_inference_manager")

    models_yaml_arg = DeclareLaunchArgument(
        "models_yaml",
        default_value=PathJoinSubstitution([pkg_share, "config", "models.yaml"]),
        description="Path to models.yaml",
    )
    system_yaml_arg = DeclareLaunchArgument(
        "system_yaml",
        default_value="/opt/autodriver/config/system.yaml",
        description="Path to system.yaml",
    )
    ipc_socket_arg = DeclareLaunchArgument(
        "ipc_socket_path",
        default_value="/tmp/autodriver/frames.sock",
        description="Unix domain socket path for DMABUF IPC",
    )

    inference_node = Node(
        package="tensorrt_inference_manager",
        executable="tensorrt_inference_manager_node",
        name="tensorrt_inference_manager",
        output="screen",
        parameters=[
            {
                "models_yaml": LaunchConfiguration("models_yaml"),
                "system_yaml": LaunchConfiguration("system_yaml"),
                "ipc_socket_path": LaunchConfiguration("ipc_socket_path"),
            }
        ],
    )

    return LaunchDescription([
        models_yaml_arg,
        system_yaml_arg,
        ipc_socket_arg,
        inference_node,
    ])
