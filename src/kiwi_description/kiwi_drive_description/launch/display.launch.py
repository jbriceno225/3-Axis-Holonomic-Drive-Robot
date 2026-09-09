"""Visualization-only launch using generated mock ros2_control hardware.

Usage:
    ros2 launch kiwi_drive_description display.launch.py

Do not use this launch file for the physical robot. Use
``kiwi_bringup hardware.launch.py`` for physical hardware.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the generated model with mock ros2_control hardware."""
    pkg_dir = get_package_share_directory('kiwi_drive_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'Kiwi_Drive_Full_Assembly_Copy_Copy.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')
    controllers_file = os.path.join(pkg_dir, 'config', 'ros2_controllers.yaml')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_mock_hardware:=true',
        ]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            parameters=[
                {'robot_description': robot_description},
                controllers_file,
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_joint_state_broadcaster',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                'controller_manager',
            ],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            name='spawn_velocity_controller',
            arguments=[
                'velocity_controller',
                '--controller-manager',
                'controller_manager',
                '--param-file',
                controllers_file,
            ],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            output='screen',
        ),
    ])

