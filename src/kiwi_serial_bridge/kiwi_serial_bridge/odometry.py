"""Planar odometry integration utilities."""

import math
from dataclasses import dataclass

from .kiwi_kinematics import BodyVelocity


@dataclass(frozen=True)
class Pose2D:
    """Planar robot pose."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Convert planar yaw to an x, y, z, w quaternion."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def integrate_pose(
    pose: Pose2D,
    velocity: BodyVelocity,
    dt: float,
) -> Pose2D:
    """Integrate body-frame velocity into a world-frame planar pose."""
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    world_vx = velocity.vx * cos_yaw - velocity.vy * sin_yaw
    world_vy = velocity.vx * sin_yaw + velocity.vy * cos_yaw

    return Pose2D(
        x=pose.x + world_vx * dt,
        y=pose.y + world_vy * dt,
        yaw=normalize_angle(pose.yaw + velocity.wz * dt),
    )
