"""Launch the physical Kiwi robot hardware stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Build the physical-hardware launch description."""
    description_file = PathJoinSubstitution([
        FindPackageShare("kiwi_drive_description"),
        "urdf",
        "Kiwi_Drive_Full_Assembly_Copy_Copy.urdf.xacro",
    ])
    default_bridge_params = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "config",
        "bridge_params.yaml",
    ])

    bridge_params = LaunchConfiguration("bridge_params_file")
    laser_port = LaunchConfiguration("laser_port")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_yaw = LaunchConfiguration("laser_yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare("kiwi_bringup"),
        "rviz",
        "kiwi.rviz",
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            description_file,
            " use_mock_hardware:=false",
            " laser_x:=",
            laser_x,
            " laser_y:=",
            laser_y,
            " laser_z:=",
            laser_z,
            " laser_yaw:=",
            laser_yaw,
        ]),
        value_type=str,
    )

    return LaunchDescription([
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
            description=(
                "Provisional base_link-to-base_laser X offset (m); "
                "physically verify before navigation."
            ),
        ),
        DeclareLaunchArgument(
            "laser_y",
            default_value="0.0",
            description=(
                "Provisional base_link-to-base_laser Y offset (m); "
                "physically verify before navigation."
            ),
        ),
        DeclareLaunchArgument(
            "laser_z",
            default_value="0.10977",
            description=(
                "Provisional base_link-to-base_laser Z offset (m); "
                "physically verify before navigation."
            ),
        ),
        DeclareLaunchArgument(
            "laser_yaw",
            default_value="-1.5707963267948966",
            description=(
                "Provisional base_link-to-base_laser yaw (rad); "
                "physically verify before navigation."
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time (normally false on hardware).",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz with LaserScan and base_link/base_laser axes.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="RViz config file for physical hardware and mapping.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "rate": 20.0,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="kiwi_serial_bridge",
            executable="cmd_vel_bridge",
            name="cmd_vel_serial_bridge",
            output="screen",
            parameters=[
                bridge_params,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
        ),
        Node(
            package="ldlidar_ros2",
            executable="ldlidar_ros2_node",
            name="ldlidar_publisher_ld19",
            output="screen",
            parameters=[{
                "product_name": "LDLiDAR_LD19",
                "laser_scan_topic_name": "scan",
                "point_cloud_2d_topic_name": "pointcloud2d",
                "frame_id": "base_laser",
                "port_name": laser_port,
                "serial_baudrate": 230400,
                "laser_scan_dir": True,
                "enable_angle_crop_func": False,
                "angle_crop_min": 135.0,
                "angle_crop_max": 225.0,
                "range_min": 0.02,
                "range_max": 12.0,
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
            arguments=["-d", rviz_config],
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        # Do not include the vendor ld19.launch.py: it also publishes the
        # base_link -> base_laser TF, which belongs to this robot's URDF.
    ])
