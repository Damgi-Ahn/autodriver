"""
perception_system.launch.py

Top-level launch file for the autodriver perception system.
Starts all nodes in the correct order:
  1. tensorrt_inference_manager  (IPC server side — must be ready first)
  2. camera_manager              (connects to inference_manager via IPC)
  3. metrics_manager
  4. pipeline_profiler
  5. gpu_watchdog
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Config paths ──────────────────────────────────────────────────────
    cameras_yaml_arg = DeclareLaunchArgument(
        "cameras_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("camera_manager"), "config", "cameras.yaml"]
        ),
    )
    models_yaml_arg = DeclareLaunchArgument(
        "models_yaml",
        default_value=PathJoinSubstitution(
            [FindPackageShare("tensorrt_inference_manager"), "config", "models.yaml"]
        ),
    )
    system_yaml_arg = DeclareLaunchArgument(
        "system_yaml",
        default_value="/opt/autodriver/config/system.yaml",
    )
    ipc_socket_arg = DeclareLaunchArgument(
        "ipc_socket_path",
        default_value="/tmp/autodriver/frames.sock",
    )
    debug_stream_arg = DeclareLaunchArgument(
        "debug_stream",
        default_value="false",
    )

    # ── Inference manager (start first — IPC server) ──────────────────────
    inference_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("tensorrt_inference_manager"),
                "launch",
                "inference_manager.launch.py",
            ])
        ),
        launch_arguments={
            "models_yaml":    LaunchConfiguration("models_yaml"),
            "system_yaml":    LaunchConfiguration("system_yaml"),
            "ipc_socket_path": LaunchConfiguration("ipc_socket_path"),
        }.items(),
    )

    # ── Camera manager (delayed 2s to allow inference_manager to bind socket)
    camera_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("camera_manager"),
                    "launch",
                    "camera_manager.launch.py",
                ])
            ),
            launch_arguments={
                "cameras_yaml":   LaunchConfiguration("cameras_yaml"),
                "ipc_socket_path": LaunchConfiguration("ipc_socket_path"),
                "debug_stream":   LaunchConfiguration("debug_stream"),
            }.items(),
        )]
    )

    # ── System monitoring nodes ───────────────────────────────────────────
    metrics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("metrics_manager"), "launch", "metrics_manager.launch.py"
            ])
        )
    )
    profiler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("pipeline_profiler"), "launch", "pipeline_profiler.launch.py"
            ])
        )
    )
    watchdog_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("gpu_watchdog"), "launch", "gpu_watchdog.launch.py"
            ])
        )
    )

    return LaunchDescription([
        cameras_yaml_arg,
        models_yaml_arg,
        system_yaml_arg,
        ipc_socket_arg,
        debug_stream_arg,
        # Launch order: inference → camera (2s delay) → monitoring
        inference_launch,
        camera_launch,
        metrics_launch,
        profiler_launch,
        watchdog_launch,
    ])
