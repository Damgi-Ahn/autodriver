from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="pipeline_profiler",
        executable="pipeline_profiler_node",
        name="pipeline_profiler",
        output="screen",
    )

    return LaunchDescription([node])
