"""Coordinate conversion and inverse kinematics for a Kiwi drive."""

import math
from dataclasses import dataclass

from .serial_protocol import VelocityCommand


@dataclass(frozen=True)
class BodyVelocity:
    """ROS base_link velocity."""

    vx: float
    vy: float
    wz: float


def clamp_normalized(value: float) -> float:
    """Clamp a normalized command to the controller's accepted range."""
    return max(-1.0, min(value, 1.0))


def twist_to_command(
    linear_x: float,
    linear_y: float,
    angular_z: float,
    max_linear_speed: float,
    max_angular_speed: float,
    sideways_direction: float = 1.0,
    forward_direction: float = 1.0,
    rotation_direction: float = 1.0,
) -> VelocityCommand:
    """Convert a ROS twist to the ESP32's normalized coordinate convention."""
    return VelocityCommand(
        x=clamp_normalized(
            linear_y / max_linear_speed * sideways_direction
        ),
        y=clamp_normalized(
            linear_x / max_linear_speed * forward_direction
        ),
        rotation=clamp_normalized(
            angular_z / max_angular_speed * rotation_direction
        ),
    )


def command_for_age(
    command: VelocityCommand,
    age: float,
    timeout: float,
) -> VelocityCommand:
    """Return a stop command when the latest ROS command is stale."""
    if age > timeout:
        return VelocityCommand(0.0, 0.0, 0.0)
    return command


def wheel_rpm_to_body_velocity(
    rpm1: float,
    rpm2: float,
    rpm3: float,
    wheel_radius: float,
    wheel_distance: float,
    y_direction: float = -1.0,
    yaw_direction: float = 1.0,
) -> BodyVelocity:
    """Convert measured wheel RPM to velocity in ROS base_link axes."""
    rpm_to_linear = 2.0 * math.pi / 60.0 * wheel_radius
    wheel1 = rpm1 * rpm_to_linear
    wheel2 = rpm2 * rpm_to_linear
    wheel3 = rpm3 * rpm_to_linear

    robot_right = (2.0 * wheel2 - wheel1 - wheel3) / 3.0
    robot_forward = (wheel1 - wheel3) / math.sqrt(3.0)
    robot_angular = (
        (wheel1 + wheel2 + wheel3) / (3.0 * wheel_distance)
    )

    return BodyVelocity(
        vx=robot_forward,
        vy=-robot_right * y_direction,
        wz=robot_angular * yaw_direction,
    )
