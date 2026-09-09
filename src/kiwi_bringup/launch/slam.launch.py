"""Launch online asynchronous SLAM Toolbox without robot hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start SLAM Toolbox as the sole map-to-odom publisher."""
    default_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "slam_params.yaml",
    ])
    upstream_launch = PathJoinSubstitution([
        FindPackageShare("slam_toolbox"),
        "launch",
        "online_async_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time instead of the hardware clock.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_params,
            description="SLAM Toolbox parameter file.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(upstream_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "slam_params_file": LaunchConfiguration("slam_params_file"),
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ])
