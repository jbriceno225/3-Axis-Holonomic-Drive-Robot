"""Compose hardware with optional SLAM and navigation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _validate_composition(context):
    enable_slam = LaunchConfiguration("enable_slam").perform(context).lower()
    enable_navigation = LaunchConfiguration(
        "enable_navigation"
    ).perform(context).lower()
    enable_exploration = LaunchConfiguration(
        "enable_exploration"
    ).perform(context).lower()
    mode = LaunchConfiguration("navigation_mode").perform(context)
    truthy = ("true", "1", "yes")
    if (
        enable_slam in truthy
        and enable_navigation in truthy
        and mode == "localization"
    ):
        raise RuntimeError(
            "Do not enable SLAM with localization mode: both publish map->odom"
        )
    if enable_exploration in truthy and (
        enable_slam not in truthy
        or enable_navigation not in truthy
        or mode != "mapping"
    ):
        raise RuntimeError(
            "Exploration requires enable_slam:=true, "
            "enable_navigation:=true, and navigation_mode:=mapping"
        )
    return []


def generate_launch_description():
    """Build the compositional robot bringup."""
    hardware_launch = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "launch",
        "hardware.launch.py",
    ])
    slam_launch = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "launch",
        "slam.launch.py",
    ])
    navigation_launch = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "launch",
        "navigation.launch.py",
    ])
    exploration_launch = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "launch",
        "exploration.launch.py",
    ])
    default_bridge_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "bridge_params.yaml",
    ])
    default_slam_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "slam_params.yaml",
    ])
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "nav2_params.yaml",
    ])
    default_explore_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "explore_params.yaml",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_slam = LaunchConfiguration("enable_slam")
    enable_navigation = LaunchConfiguration("enable_navigation")
    enable_exploration = LaunchConfiguration("enable_exploration")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time (normally false on hardware).",
        ),
        DeclareLaunchArgument(
            "bridge_params_file",
            default_value=default_bridge_params,
            description="Parameter file for the Kiwi ESP32 serial bridge.",
        ),
        DeclareLaunchArgument(
            "laser_port",
            default_value="/dev/kiwi_lidar",
            description="Stable udev symlink for the LD19 serial device.",
        ),
        DeclareLaunchArgument(
            "laser_x",
            default_value="0.114",
            description="Provisional laser X offset; physically verify.",
        ),
        DeclareLaunchArgument(
            "laser_y",
            default_value="0.0",
            description="Provisional laser Y offset; physically verify.",
        ),
        DeclareLaunchArgument(
            "laser_z",
            default_value="0.08477",
            description="Provisional laser Z offset; physically verify.",
        ),
        DeclareLaunchArgument(
            "laser_yaw",
            default_value="-1.5707963267948966",
            description="Provisional laser yaw; physically verify.",
        ),
        DeclareLaunchArgument(
            "enable_slam",
            default_value="false",
            description="Start online asynchronous SLAM Toolbox.",
        ),
        DeclareLaunchArgument(
            "enable_navigation",
            default_value="false",
            description="Start the Nav2 navigation stack.",
        ),
        DeclareLaunchArgument(
            "enable_exploration",
            default_value="false",
            description=(
                "Start frontier exploration; requires SLAM and Nav2 mapping."
            ),
        ),
        DeclareLaunchArgument(
            "navigation_mode",
            default_value="mapping",
            description="'mapping' for live SLAM or 'localization' for AMCL.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute saved-map YAML path for localization mode.",
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params,
            description="SLAM Toolbox parameter file.",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=default_nav2_params,
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "exploration_params_file",
            default_value=default_explore_params,
            description="explore_lite parameter file.",
        ),
        DeclareLaunchArgument(
            "exploration_startup_delay",
            default_value="8.0",
            description="Grace period for SLAM and Nav2 lifecycle startup.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with LaserScan and base_link/base_laser axes.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("kiwi_bringup"),
                "rviz",
                "kiwi.rviz",
            ]),
            description="RViz config file for physical hardware and mapping.",
        ),
        OpaqueFunction(function=_validate_composition),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hardware_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "bridge_params_file": LaunchConfiguration(
                    "bridge_params_file"
                ),
                "laser_port": LaunchConfiguration("laser_port"),
                "laser_x": LaunchConfiguration("laser_x"),
                "laser_y": LaunchConfiguration("laser_y"),
                "laser_z": LaunchConfiguration("laser_z"),
                "laser_yaw": LaunchConfiguration("laser_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(enable_slam),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": LaunchConfiguration("slam_params_file"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            condition=IfCondition(enable_navigation),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": LaunchConfiguration("nav2_params_file"),
                "mode": LaunchConfiguration("navigation_mode"),
                "map": LaunchConfiguration("map"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(exploration_launch),
            condition=IfCondition(enable_exploration),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": LaunchConfiguration(
                    "exploration_params_file"
                ),
                "startup_delay": LaunchConfiguration(
                    "exploration_startup_delay"
                ),
            }.items(),
        ),
    ])
