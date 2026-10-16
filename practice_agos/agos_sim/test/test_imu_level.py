"""
Tests for agos_sim/imu_level.py (the heading rate from a tilted gyro).

    cd agos_sim && python3 -m pytest test/test_imu_level.py     (or: colcon test)

Needs ROS 2 sourced (the module imports rclpy); skipped without it.
"""
import math

import pytest

imu_level = pytest.importorskip('agos_sim.imu_level')


def quaternion(roll, pitch, yaw):
    """Return (x, y, z, w) for ZYX angles in radians."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def body_rates(roll, pitch, yaw_rate):
    """Return the gyro's (p, q, r) for a robot so tilted, turning about the vertical."""
    return (-math.sin(pitch) * yaw_rate,
            math.sin(roll) * math.cos(pitch) * yaw_rate,
            math.cos(roll) * math.cos(pitch) * yaw_rate)


def test_roll_pitch_of_a_quaternion():
    for roll, pitch, yaw in [(0.0, 0.0, 0.0), (0.2, -0.1, 2.5), (-0.4, 0.3, -1.0)]:
        r, p = imu_level.roll_pitch(*quaternion(roll, pitch, yaw))
        assert r == pytest.approx(roll, abs=1e-9)
        assert p == pytest.approx(pitch, abs=1e-9)


def test_level_robot_keeps_the_gyro_rate():
    assert imu_level.heading_rate(0.3, 0.5, 0.0, 0.0) == pytest.approx(0.5)


def test_tilted_robot_gets_its_whole_turn_rate():
    for roll, pitch in [(math.radians(12), 0.0), (math.radians(-15), math.radians(5)),
                        (0.0, math.radians(-8))]:
        _, q, r = body_rates(roll, pitch, 0.5)
        assert r < 0.5                  # the gyro's z axis alone is short
        assert imu_level.heading_rate(q, r, roll, pitch) == pytest.approx(0.5)
