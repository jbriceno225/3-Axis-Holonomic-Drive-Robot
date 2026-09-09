"""Unit tests for Kiwi coordinate conversion and kinematics."""

import math

import pytest

from kiwi_serial_bridge.kiwi_kinematics import (
    command_for_age,
    twist_to_command,
    wheel_rpm_to_body_velocity,
)
from kiwi_serial_bridge.serial_protocol import VelocityCommand


def test_twist_to_command_maps_ros_axes_and_clamps() -> None:
    """ROS forward/left/yaw map to ESP32 y/x/rotation."""
    command = twist_to_command(
        linear_x=0.15,
        linear_y=-0.6,
        angular_z=2.0,
        max_linear_speed=0.3,
        max_angular_speed=1.0,
    )
    assert command == VelocityCommand(-1.0, 0.5, 1.0)


def test_command_timeout_stops_only_when_stale() -> None:
    """The timeout boundary preserves a fresh command then stops it."""
    command = VelocityCommand(0.1, 0.2, 0.3)
    assert command_for_age(command, 0.25, 0.25) == command
    assert command_for_age(command, 0.251, 0.25) == VelocityCommand(
        0.0, 0.0, 0.0
    )


def test_equal_wheel_speeds_produce_rotation_only() -> None:
    """Equal wheel speeds invert to pure body rotation."""
    velocity = wheel_rpm_to_body_velocity(
        60.0,
        60.0,
        60.0,
        wheel_radius=0.050,
        wheel_distance=0.2921,
    )
    assert velocity.vx == pytest.approx(0.0)
    assert velocity.vy == pytest.approx(0.0)
    assert velocity.wz == pytest.approx(
        2.0 * math.pi * 0.050 / 0.2921
    )


def test_direction_parameters_flip_odometry_axes() -> None:
    """Configured direction multipliers flip lateral and yaw velocity."""
    normal = wheel_rpm_to_body_velocity(
        10.0, 20.0, 5.0, 0.050, 0.2921
    )
    flipped = wheel_rpm_to_body_velocity(
        10.0,
        20.0,
        5.0,
        0.050,
        0.2921,
        y_direction=1.0,
        yaw_direction=-1.0,
    )
    assert flipped.vx == pytest.approx(normal.vx)
    assert flipped.vy == pytest.approx(-normal.vy)
    assert flipped.wz == pytest.approx(-normal.wz)
