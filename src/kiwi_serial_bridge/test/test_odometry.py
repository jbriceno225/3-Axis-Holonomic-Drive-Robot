"""Unit tests for planar odometry utilities."""

import math

import pytest

from kiwi_serial_bridge.kiwi_kinematics import BodyVelocity
from kiwi_serial_bridge.odometry import (
    Pose2D,
    integrate_pose,
    normalize_angle,
    yaw_to_quaternion,
)


def test_integrates_body_velocity_in_world_frame() -> None:
    """Body forward velocity follows the current world heading."""
    pose = integrate_pose(
        Pose2D(yaw=math.pi / 2.0),
        BodyVelocity(vx=1.0, vy=0.0, wz=0.5),
        dt=2.0,
    )
    assert pose.x == pytest.approx(0.0, abs=1e-12)
    assert pose.y == pytest.approx(2.0)
    assert pose.yaw == pytest.approx(
        normalize_angle(math.pi / 2.0 + 1.0)
    )


def test_angle_and_quaternion_are_normalized() -> None:
    """Yaw wraps and produces the expected planar quaternion."""
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert yaw_to_quaternion(math.pi) == pytest.approx(
        (0.0, 0.0, 1.0, 0.0),
        abs=1e-12,
    )
