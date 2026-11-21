"""Direct Gazebo Transport interface for the grasp experiment.

Talks to gz-sim natively (python3-gz-transport13 / gz-msgs10) instead of
through ros_gz_bridge: ground-truth poses at the SceneBroadcaster rate,
Armbot joint commands, finger force commands, and model teleports.

NOTE: the system gz-msgs Python modules are generated for the distro's
protobuf 3.12. A newer protobuf in ~/.local breaks their import, so run
anything importing this module with PYTHONNOUSERSITE=1 (utility/
experiment.sh does).
"""
from __future__ import annotations

import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Sequence

import numpy as np

from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from px4_gripper_missions.geometry import compose, rpy_to_quat, transform_point

# Grasp point (centre between the finger pads) in the Armbot model frame
# at zero joint angles, see the gripper comment in Armbot/model.sdf. It is
# rigidly attached to gripper_base_link, whose frame coincides with the
# model frame at zero joint angles.
GRASP_POINT_IN_GRIPPER_LINK = (0.3025, 0.0, 0.110)

ARM_JOINTS = ('base_yaw_joint', 'shoulder_pitch_joint', 'elbow_pitch_joint')
FINGER_JOINTS = ('finger_left_joint', 'finger_right_joint')          # force_fingers gripper
JAW_JOINTS = ('jaw_left_joint', 'jaw_right_joint')                   # servo_claw gripper
# Pad contact sensors of the servo claw (the simulated FSRs), see Armbot/model.sdf.
PADS = tuple((jaw, pad) for jaw in ('jaw_left', 'jaw_right') for pad in ('pad_a', 'pad_b'))
PAD_FRESH_S = 0.05     # a pad without a contact message for this long carries no force


@dataclass
class WorldState:
    sim_t: float = 0.0
    drone_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    drone_q: tuple = (1.0, 0.0, 0.0, 0.0)
    arm_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    arm_q: tuple = (1.0, 0.0, 0.0, 0.0)
    gripper_rel_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gripper_rel_q: tuple = (1.0, 0.0, 0.0, 0.0)
    cyl_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    cyl_q: tuple = (1.0, 0.0, 0.0, 0.0)
    joints: Dict[str, float] = field(default_factory=dict)
    fsr_l: float = float('nan')     # normal contact force on the left / right jaw pads [N]
    fsr_r: float = float('nan')     # (servo_claw only; nan for the force_fingers gripper)
    valid: bool = False

    @property
    def grasp_point(self) -> np.ndarray:
        g_pos, g_q = compose(self.arm_pos, self.arm_q, self.gripper_rel_pos, self.gripper_rel_q)
        return transform_point(g_pos, g_q, GRASP_POINT_IN_GRIPPER_LINK)


def _quat(p) -> tuple:
    return (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)


def _vec(p) -> np.ndarray:
    return np.array([p.position.x, p.position.y, p.position.z])


class GzWorld:
    def __init__(self, world: str, drone: str, arm: str, cylinder: str,
                 gripper_type: str = 'force_fingers'):
        self.world = world
        self.drone = drone
        self.arm = arm
        self.cylinder = cylinder
        self._node = Node()
        self._lock = threading.Lock()
        self._state = WorldState()
        self._joints: Dict[str, float] = {}
        self._listeners: List[Callable[[WorldState], None]] = []
        self._joint_pubs = {}
        self._finger_pubs = []
        self.last_grip_force = 0.0
        self.gripper_type = gripper_type
        self._jaw_pubs = []
        self._pads = {k: (0.0, -1.0) for k in PADS}     # (normal force, sim stamp)
        self._pad_cbs = []                              # keep the callbacks alive

    # -- setup ------------------------------------------------------------

    def start(self, timeout_s: float = 10.0) -> None:
        if not self._node.subscribe(Pose_V, f'/world/{self.world}/pose/info', self._on_pose):
            raise RuntimeError('could not subscribe to gz pose/info')
        if not self._node.subscribe(Model, f'/model/{self.arm}/joint_states', self._on_joints):
            raise RuntimeError('could not subscribe to gz joint_states')
        for j in ARM_JOINTS:
            self._joint_pubs[j] = self._node.advertise(
                f'/model/{self.arm}/joint/{j}/cmd_pos', Double)
        if self.gripper_type == 'servo_claw':
            for j in JAW_JOINTS:
                self._jaw_pubs.append(self._node.advertise(
                    f'/model/{self.arm}/joint/{j}/cmd_pos', Double))
            for k in PADS:
                cb = (lambda m, k=k: self._on_contact(k, m))
                self._pad_cbs.append(cb)
                if not self._node.subscribe(Contacts, f'/model/{self.arm}/{k[0]}/{k[1]}/contact', cb):
                    raise RuntimeError(f'could not subscribe to the {k} contact sensor')
        else:
            for j in FINGER_JOINTS:
                self._finger_pubs.append(self._node.advertise(
                    f'/model/{self.arm}/joint/{j}/cmd_force', Double))
        deadline = time.monotonic() + timeout_s
        while not self.state().valid:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f'no pose/info for {self.drone}/{self.arm}/{self.cylinder} '
                    f'in world {self.world} - is the sim running?')
            time.sleep(0.05)

    def add_listener(self, fn: Callable[[WorldState], None]) -> None:
        self._listeners.append(fn)

    # -- callbacks (gz transport thread) ---------------------------------

    def _on_joints(self, msg: Model) -> None:
        joints = {j.name: j.axis1.position for j in msg.joint}
        with self._lock:
            self._joints = joints

    def _on_contact(self, key, msg: Contacts) -> None:
        """Sum of the normal contact force over the contact points of one pad."""
        f = 0.0
        for c in msg.contact:
            for i, n in enumerate(c.normal):
                if i < len(c.wrench):
                    w = c.wrench[i].body_1_wrench.force
                    f += abs(w.x * n.x + w.y * n.y + w.z * n.z)
        with self._lock:
            self._pads[key] = (f, msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9)

    def _jaw_forces(self, t: float):
        out = []
        for jaw in ('jaw_left', 'jaw_right'):
            out.append(sum(f for (j, _), (f, st) in self._pads.items()
                           if j == jaw and t - st < PAD_FRESH_S))
        return out

    def _on_pose(self, msg: Pose_V) -> None:
        s = WorldState(sim_t=msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9)
        found = set()
        for p in msg.pose:
            if p.name == self.drone:
                s.drone_pos, s.drone_q = _vec(p), _quat(p)
            elif p.name == self.arm:
                s.arm_pos, s.arm_q = _vec(p), _quat(p)
            elif p.name == self.cylinder:
                s.cyl_pos, s.cyl_q = _vec(p), _quat(p)
            elif p.name == 'gripper_base_link':
                s.gripper_rel_pos, s.gripper_rel_q = _vec(p), _quat(p)
            else:
                continue
            found.add(p.name)
        if len(found) < 4:
            return
        with self._lock:
            s.joints = dict(self._joints)
            if self.gripper_type == 'servo_claw':
                s.fsr_l, s.fsr_r = self._jaw_forces(s.sim_t)
            s.valid = True
            self._state = s
        for fn in self._listeners:
            fn(s)

    # -- queries ------------------------------------------------------------

    def state(self) -> WorldState:
        with self._lock:
            return self._state

    def sim_time(self) -> float:
        return self.state().sim_t

    def sleep_sim(self, dt: float, wall_timeout_factor: float = 20.0) -> None:
        """Sleep dt seconds of *simulation* time (robust to RTF < 1)."""
        t_end = self.sim_time() + dt
        wall_deadline = time.monotonic() + dt * wall_timeout_factor + 5.0
        while self.sim_time() < t_end:
            if time.monotonic() > wall_deadline:
                raise TimeoutError('simulation clock stopped advancing')
            time.sleep(0.01)

    # -- commands -----------------------------------------------------------

    def set_arm_pose(self, pose: Dict[str, float]) -> None:
        names = {'base_yaw': 'base_yaw_joint', 'shoulder_pitch': 'shoulder_pitch_joint',
                 'elbow_pitch': 'elbow_pitch_joint'}
        for short, rad in pose.items():
            self._joint_pubs[names.get(short, short)].publish(Double(data=float(rad)))

    def set_grip_force(self, newtons: float) -> None:
        """Same force on both fingers; + closes, - opens."""
        self.last_grip_force = float(newtons)
        for pub in self._finger_pubs:
            pub.publish(Double(data=float(newtons)))

    def set_jaws(self, rad: float) -> None:
        """Servo claw: same jaw angle command on both jaws [rad]; + closes, - opens."""
        self.last_grip_force = float(rad)
        for pub in self._jaw_pubs:
            pub.publish(Double(data=float(rad)))

    def set_grip(self, value: float) -> None:
        """Grip command for whichever gripper the model has: jaw angle [rad] for the
        servo claw, finger force [N] for the force-controlled fingers."""
        if self.gripper_type == 'servo_claw':
            self.set_jaws(value)
        else:
            self.set_grip_force(value)

    def set_model_pose(self, name: str, xyz: Sequence[float], rpy: Sequence[float]) -> bool:
        """Teleport a model (this also zeroes its velocity).

        Goes through the `gz service` CLI rather than Node.request(): the
        Python binding blocks with the GIL held while gz-transport's shared
        callback thread may be waiting for the GIL to deliver a pose message
        - a stall of the full timeout that freezes the offboard heartbeat
        and the mocap stream (PX4 offboard-loss failsafe after 1 s). A
        subprocess wait releases the GIL. ~0.3 s per call."""
        w, x, y, z = rpy_to_quat(*rpy)
        req = (f'name: "{name}", position: {{x: {xyz[0]}, y: {xyz[1]}, z: {xyz[2]}}}, '
               f'orientation: {{w: {w}, x: {x}, y: {y}, z: {z}}}')
        for _ in range(3):
            r = subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world}/set_pose',
                 '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1000', '--req', req],
                capture_output=True, text=True, timeout=10)
            if 'data: true' in r.stdout:
                return True
        return False
