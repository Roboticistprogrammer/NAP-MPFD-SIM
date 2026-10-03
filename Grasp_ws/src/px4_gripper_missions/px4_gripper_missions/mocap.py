"""Motion-capture emulation: Gazebo ground truth -> PX4 external vision.

A grasp needs the gripper placed to a few millimetres; with simulated GPS
the PX4 estimate (and so the hover) wanders by centimetres. Like a lab
with a mocap system, this publishes the vehicle's true pose as
px4_msgs/VehicleOdometry on /fmu/in/vehicle_visual_odometry (50 Hz, the
Gazebo pose rate), and EKF2 is configured to fuse it instead of GPS/baro
(see MOCAP_PARAMS). Everything downstream - estimator, controllers,
thrust/torque setpoints, motor outputs - is still PX4's own.
"""
from __future__ import annotations

import math

import numpy as np

from px4_gripper_missions.geometry import enu_to_ned, quat_to_rot, transform_point

# x500_base: base_link (IMU/body origin) sits 0.24 m above the model origin.
BODY_IN_MODEL = (0.0, 0.0, 0.24)

# ENU->NED axis swap and FLU->FRD body flip.
_M = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_F = np.diag([1.0, -1.0, -1.0])

MOCAP_PARAMS = {
    'EKF2_EV_CTRL': 11,       # fuse EV horizontal + vertical position and yaw
    'EKF2_HGT_REF': 3,        # height reference: vision
    'EKF2_GPS_CTRL': 0,
    'EKF2_BARO_CTRL': 0,
    'EKF2_EV_NOISE_MD': 1,    # use the noise parameters below
    'EKF2_EVP_NOISE': 0.01,   # [m]
    'EKF2_EVA_NOISE': 0.05,   # [rad]
    'EKF2_EV_DELAY': 0,
}


def rot_to_quat(r: np.ndarray):
    """Rotation matrix -> (w, x, y, z)."""
    t = r[0, 0] + r[1, 1] + r[2, 2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        return (0.25 / s, (r[2, 1] - r[1, 2]) * s, (r[0, 2] - r[2, 0]) * s, (r[1, 0] - r[0, 1]) * s)
    i = int(np.argmax([r[0, 0], r[1, 1], r[2, 2]]))
    if i == 0:
        s = 2.0 * math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        return ((r[2, 1] - r[1, 2]) / s, 0.25 * s, (r[0, 1] + r[1, 0]) / s, (r[0, 2] + r[2, 0]) / s)
    if i == 1:
        s = 2.0 * math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        return ((r[0, 2] - r[2, 0]) / s, (r[0, 1] + r[1, 0]) / s, 0.25 * s, (r[1, 2] + r[2, 1]) / s)
    s = 2.0 * math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
    return ((r[1, 0] - r[0, 1]) / s, (r[0, 2] + r[2, 0]) / s, (r[1, 2] + r[2, 1]) / s, 0.25 * s)


def gz_pose_to_px4(pos_enu, q_enu_flu):
    """Model pose in Gazebo (ENU world, FLU body) -> body position (NED)
    and attitude quaternion (FRD body -> NED)."""
    body = transform_point(pos_enu, q_enu_flu, BODY_IN_MODEL)
    r_ned_frd = _M @ quat_to_rot(q_enu_flu) @ _F
    return enu_to_ned(body), rot_to_quat(r_ned_frd)


class MocapBridge:
    """GzWorld listener that republishes the drone pose to PX4."""

    def __init__(self, node):
        from px4_msgs.msg import VehicleOdometry

        from px4_gripper_missions.px4_offboard import PX4_QOS
        self._msg_type = VehicleOdometry
        self._node = node
        self._pub = node.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', PX4_QOS)
        self.enabled = False

    def __call__(self, s) -> None:
        if not self.enabled:
            return
        pos, q = gz_pose_to_px4(s.drone_pos, s.drone_q)
        m = self._msg_type()
        now = int(self._node.get_clock().now().nanoseconds / 1000)
        m.timestamp = now
        m.timestamp_sample = now
        m.pose_frame = self._msg_type.POSE_FRAME_NED
        m.position = [float(v) for v in pos]
        m.q = [float(v) for v in q]
        m.velocity_frame = self._msg_type.VELOCITY_FRAME_UNKNOWN
        nan = float('nan')
        m.velocity = [nan, nan, nan]
        m.angular_velocity = [nan, nan, nan]
        m.position_variance = [1e-4, 1e-4, 1e-4]
        m.orientation_variance = [2.5e-3, 2.5e-3, 2.5e-3]
        m.velocity_variance = [nan, nan, nan]
        try:
            self._pub.publish(m)
        except Exception:      # context already shut down (process exiting)
            self.enabled = False
