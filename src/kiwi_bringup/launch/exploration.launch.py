"""Launch explore_lite after the mapping navigation stack is ready."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start frontier exploration with a configurable startup grace period."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    startup_delay = LaunchConfiguration("startup_delay")

    explore_node = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[
            ParameterFile(params_file, allow_substs=True),
            {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time instead of the hardware clock.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("kiwi_bringup"),
                "config",
                "explore_params.yaml",
            ]),
            description="explore_lite parameter file.",
        ),
        DeclareLaunchArgument(
            "startup_delay",
            default_value="8.0",
            description=(
                "Seconds to allow SLAM and Nav2 lifecycle activation before "
                "starting explore_lite."
            ),
        ),
        TimerAction(period=startup_delay, actions=[explore_node]),
    ])
