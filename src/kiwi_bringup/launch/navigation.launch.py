"""Launch Nav2 for mapping navigation or saved-map localization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def _validate_arguments(context):
    mode = LaunchConfiguration("mode").perform(context)
    map_file = LaunchConfiguration("map").perform(context)
    if mode not in ("mapping", "localization"):
        raise RuntimeError("mode must be 'mapping' or 'localization'")
    if mode == "localization" and not map_file:
        raise RuntimeError("localization mode requires map:=/absolute/map.yaml")
    return []


def generate_launch_description():
    """Build a non-composed Nav2 stack with one velocity output chain."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    localization = IfCondition(PythonExpression([
        "'", LaunchConfiguration("mode"), "' == 'localization'",
    ]))
    configured_params = ParameterFile(params_file, allow_substs=True)
    common_parameters = [
        configured_params,
        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
    ]
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    navigation_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    def nav_node(package, executable, name=None, remappings=None):
        return Node(
            package=package,
            executable=executable,
            name=name or executable,
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings + (remappings or []),
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
                "nav2_params.yaml",
            ]),
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="mapping",
            description="'mapping' uses a live SLAM map; 'localization' uses AMCL.",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Absolute map YAML path, required in localization mode.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes.",
        ),
        OpaqueFunction(function=_validate_arguments),
        Node(
            condition=localization,
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=common_parameters + [{
                "yaml_filename": LaunchConfiguration("map"),
            }],
            remappings=tf_remappings,
        ),
        Node(
            condition=localization,
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=common_parameters,
            remappings=tf_remappings,
        ),
        Node(
            condition=localization,
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": ["map_server", "amcl"],
            }],
        ),
        nav_node(
            "nav2_controller",
            "controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        nav_node("nav2_planner", "planner_server"),
        nav_node(
            "nav2_behaviors",
            "behavior_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        nav_node("nav2_bt_navigator", "bt_navigator"),
        nav_node("nav2_waypoint_follower", "waypoint_follower"),
        nav_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        nav_node("nav2_collision_monitor", "collision_monitor"),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": navigation_nodes,
            }],
        ),
    ])
