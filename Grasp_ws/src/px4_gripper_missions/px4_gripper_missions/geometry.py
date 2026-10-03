"""Small rigid-body helpers (quaternions as (w, x, y, z), vectors as 3-tuples).

Pure numpy, no ROS/Gazebo imports, so it is unit-testable on its own.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # w, x, y, z


def quat_to_rot(q: Sequence[float]) -> np.ndarray:
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quat:
    """SDFormat convention, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def quat_to_rpy(q: Sequence[float]) -> Vec3:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def transform_point(pos: Sequence[float], q: Sequence[float], p_local: Sequence[float]) -> np.ndarray:
    """World coordinates of p_local expressed in the frame (pos, q)."""
    return np.asarray(pos, dtype=float) + quat_to_rot(q) @ np.asarray(p_local, dtype=float)


def compose(pos_a, q_a, pos_b, q_b):
    """Pose of frame b (given relative to a) in a's parent frame."""
    r_a = quat_to_rot(q_a)
    pos = np.asarray(pos_a, dtype=float) + r_a @ np.asarray(pos_b, dtype=float)
    # quaternion product q_a * q_b
    w1, x1, y1, z1 = q_a
    w2, x2, y2, z2 = q_b
    q = (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )
    return pos, q


def axis_tilt_deg(q: Sequence[float], local_axis: Sequence[float] = (0.0, 0.0, 1.0)) -> float:
    """Angle (deg) between a body axis and the horizontal plane.

    For the test cylinder the symmetry axis is the local z axis; lying on
    the table this is ~0 deg, hanging vertically 90 deg.
    """
    a = quat_to_rot(q) @ np.asarray(local_axis, dtype=float)
    return math.degrees(math.asin(min(1.0, abs(a[2]) / (np.linalg.norm(a) or 1.0))))


# PX4 local frame is NED, Gazebo world is ENU (x east, y north, z up).
def enu_to_ned(v: Sequence[float]) -> np.ndarray:
    return np.array([v[1], v[0], -v[2]], dtype=float)


def ned_to_enu(v: Sequence[float]) -> np.ndarray:
    return np.array([v[1], v[0], -v[2]], dtype=float)
