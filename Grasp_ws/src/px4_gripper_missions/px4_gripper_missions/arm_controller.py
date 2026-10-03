"""Armbot joint-command helper.

Armbot's joints are native Gazebo Transport topics
(/model/armbot/joint/<name>/cmd_pos, gz.msgs.Double), not ROS 2 topics —
see wayfinder/MAP.md. The normal path here rides on ros_gz_bridge
(config/bridge.yaml) so this can publish plain std_msgs/Float64 on the
same rclpy.Node that's driving PX4Offboard; use_gz_transport_fallback
shells out to `gz topic pub` directly instead, for a machine where the
bridge isn't running (both are confirmed installed on this dev machine).

No joint-state feedback loop yet: Armbot's JointPositionController
plugins don't expose a "reached target" signal, and while
config/bridge.yaml does bridge /model/armbot/joint_states, this class
doesn't consume it yet. set_pose's settle_s is a fixed dwell, not a
real convergence check — flagged here as a known, deliberate
placeholder (upgrade path: subscribe to the bridged joint_states topic
and wait for measured position to approach commanded position instead
of sleeping a fixed duration).
"""
from __future__ import annotations

import subprocess
import time
from typing import Dict, Optional

from rclpy.node import Node
from std_msgs.msg import Float64

from px4_gripper_missions.config_loader import JointTopic, ObjectDef, WORLD_NAME

# Ticket 002's mount decision: joints-at-zero already hangs the gripper
# straight down (see wayfinder/tickets/002-arm-mount-point.md), so the
# rest/home pose is simply all-zero on the three commandable arm joints
# (gripper excluded — handled separately via open/close).
HOME_POSE: Dict[str, float] = {
    'base_yaw': 0.0,
    'shoulder_pitch': 0.0,
    'elbow_pitch': 0.0,
}


class ArmController:
    """Drives Armbot's four commandable joints (base_yaw, shoulder_pitch,
    elbow_pitch, gripper) via bridged/gz-native cmd_pos topics.

    Takes a *shared* rclpy.Node, same composition pattern as
    PX4OffboardController — one mission node, two helpers.
    """

    def __init__(
        self,
        node: Node,
        joint_topic_map: Dict[str, JointTopic],
        gripper_open_position: float = 0.0,
        use_gz_transport_fallback: bool = False,
        dry_run: bool = False,
    ):
        self._node = node
        self._log = node.get_logger()
        self._joint_topic_map = joint_topic_map
        self._gripper_open_position = gripper_open_position
        self._use_gz_transport_fallback = use_gz_transport_fallback
        self._dry_run = dry_run

        self._current_pose: Dict[str, float] = dict(HOME_POSE)
        self._publishers: Dict[str, object] = {}

        if not dry_run and not use_gz_transport_fallback:
            for name, topic in joint_topic_map.items():
                self._publishers[name] = node.create_publisher(
                    Float64, topic.ros_topic_name, 10)

    # -- low-level ------------------------------------------------------------

    def set_joint_position(self, joint_name: str, radians: float) -> None:
        if joint_name not in self._joint_topic_map:
            raise KeyError(
                f"Unknown joint '{joint_name}'; known: {sorted(self._joint_topic_map)}"
            )

        if self._dry_run:
            self._log.info(f'[dry-run] would set {joint_name} = {radians:.4f} rad')
            return

        if self._use_gz_transport_fallback:
            gz_topic = self._joint_topic_map[joint_name].gz_topic_name
            subprocess.run(
                [
                    'gz', 'topic', '-t', gz_topic,
                    '-m', 'gz.msgs.Double',
                    '-p', f'data: {radians}',
                ],
                check=True,
                timeout=5.0,
            )
        else:
            msg = Float64()
            msg.data = float(radians)
            self._publishers[joint_name].publish(msg)

        if joint_name != 'gripper':
            self._current_pose[joint_name] = radians

    # -- pose-level -------------------------------------------------------------

    def set_pose(self, pose: Dict[str, float], settle_s: float = 0.5) -> None:
        """Command the given {joint_name: radians} pairs, dwell settle_s for
        the per-joint position controllers to converge (see module
        docstring re: the fixed-dwell placeholder)."""
        for joint_name, radians in pose.items():
            self.set_joint_position(joint_name, radians)
        if settle_s > 0:
            time.sleep(settle_s)

    def approach(self, target_pose: Dict[str, float], settle_s: float = 1.0) -> None:
        """Named alias for set_pose, matching Scenarios.md's own verb for
        SC-02/SC-04's pre-grasp move."""
        self.set_pose(target_pose, settle_s=settle_s)

    def home(self, settle_s: float = 1.0) -> None:
        self.set_pose(dict(HOME_POSE), settle_s=settle_s)

    def lift(self, delta: Dict[str, float], settle_s: float = 1.0) -> None:
        new_pose = dict(self._current_pose)
        for joint_name, d in delta.items():
            new_pose[joint_name] = new_pose.get(joint_name, 0.0) + d
        self.set_pose(new_pose, settle_s=settle_s)

    def lower(self, delta: Dict[str, float], settle_s: float = 1.0) -> None:
        """Reverses lift(delta) by applying the same delta dict negated —
        scenario scripts pass the same config value to both calls."""
        new_pose = dict(self._current_pose)
        for joint_name, d in delta.items():
            new_pose[joint_name] = new_pose.get(joint_name, 0.0) - d
        self.set_pose(new_pose, settle_s=settle_s)

    # -- gripper ------------------------------------------------------------------

    def close_gripper(self, closed_position: float) -> None:
        self.set_joint_position('gripper', closed_position)

    def open_gripper(self, open_position: Optional[float] = None) -> None:
        self.set_joint_position(
            'gripper', open_position if open_position is not None else self._gripper_open_position
        )

    def release(self) -> None:
        """Named alias for open_gripper, matching Scenarios.md's "release"."""
        self.open_gripper()

    # -- hold -------------------------------------------------------------------

    def hold(self, duration_s: float, outcome_monitor=None, sample_interval_s: float = 0.5) -> None:
        """Dwell for duration_s. If outcome_monitor is given, calls its
        .sample() on sample_interval_s ticks — the single call site for
        grasp-outcome polling during a hold, instead of duplicating it
        per scenario script."""
        elapsed = 0.0
        while elapsed < duration_s:
            step = min(sample_interval_s, duration_s - elapsed)
            time.sleep(step)
            elapsed += step
            if outcome_monitor is not None:
                outcome_monitor.sample()


def _rpy_to_quaternion(roll: float, pitch: float, elbow_yaw: float):
    """Standard aerospace RPY -> quaternion (matches SDFormat's own pose
    convention: R = Rz(yaw) * Ry(pitch) * Rx(roll))."""
    import math
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(elbow_yaw / 2), math.sin(elbow_yaw / 2)
    return {
        'w': cr * cp * cy + sr * sp * sy,
        'x': sr * cp * cy - cr * sp * sy,
        'y': cr * sp * cy + sr * cp * sy,
        'z': cr * cp * sy - sr * sp * cy,
    }


def place_object(world_name: str, model_name: str, pose: Dict[str, float], dry_run: bool = False) -> None:
    """SC-04's one new capability: reposition the grasp-target object
    between trials. ros_gz_bridge doesn't carry gz *services* (only
    topics) in config/bridge.yaml, so this shells out to `gz service`
    directly rather than adding a service-bridge dependency for a
    single one-off call per trial.

    pose keys: x, y, z, roll, pitch, yaw (world frame, matches
    objects.yaml / sc04.yaml's orientation_overrides ordering).
    """
    q = _rpy_to_quaternion(pose.get('roll', 0.0), pose.get('pitch', 0.0), pose.get('yaw', 0.0))
    req = (
        f"name: '{model_name}', "
        f"position: {{x: {pose['x']}, y: {pose['y']}, z: {pose['z']}}}, "
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}"
    )
    if dry_run:
        print(f"[dry-run] would call: gz service -s /world/{world_name}/set_pose "
              f"--req \"{req}\"")
        return

    subprocess.run(
        [
            'gz', 'service', '-s', f'/world/{world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000',
            '--req', req,
        ],
        check=True,
        timeout=5.0,
    )


def generate_object_sdf(obj: ObjectDef) -> str:
    """Inline SDF for a grasp-target object, generated from objects.yaml
    rather than requiring a separate model directory under
    PX4-Autopilot/Tools/simulation/gz/models/ — no such directory exists
    for OBJ-A/B/C yet (wayfinder ticket 003 is still open on the real
    object). Only 'box' is implemented since that's all objects.yaml
    currently defines; extend here if ticket 003 lands on a different
    shape or a real mesh.
    """
    if obj.shape != 'box':
        raise NotImplementedError(
            f"generate_object_sdf only supports shape='box' so far, got '{obj.shape}'"
        )
    sx, sy, sz = obj.size_m
    m = obj.mass_kg
    ixx = m * (sy ** 2 + sz ** 2) / 12.0
    iyy = m * (sx ** 2 + sz ** 2) / 12.0
    izz = m * (sx ** 2 + sy ** 2) / 12.0
    return f"""<sdf version="1.9">
  <model name="{obj.object_id}">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>{m}</mass>
        <inertia>
          <ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <surface>
          <friction><ode><mu>{obj.friction.mu}</mu><mu2>{obj.friction.mu2}</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>"""


def spawn_object(obj: ObjectDef, world_name: str = WORLD_NAME, dry_run: bool = False) -> None:
    """Spawns obj into the running world via the same
    /world/<world>/create EntityFactory service PX4's own
    px4-rc.gzsim uses to spawn the vehicle (see wayfinder/MAP.md) —
    consistent mechanism, no new spawn machinery invented."""
    sdf = generate_object_sdf(obj)
    x, y, z, roll, pitch, yaw = obj.world_pose
    q = _rpy_to_quaternion(roll, pitch, yaw)
    req = (
        f'name: "{obj.object_id}", allow_renaming: false, '
        f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
        f"orientation: {{x: {q['x']}, y: {q['y']}, z: {q['z']}, w: {q['w']}}}}}, "
        f"sdf: '{sdf}'"
    )
    if dry_run:
        print(f'[dry-run] would call: gz service -s /world/{world_name}/create '
              f'--req "{req}"')
        return

    subprocess.run(
        [
            'gz', 'service', '-s', f'/world/{world_name}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req,
        ],
        check=True,
        timeout=8.0,
    )


def remove_object(object_id: str, world_name: str = WORLD_NAME, dry_run: bool = False) -> None:
    """Removes a spawned object (SC-05's payload swap between objects in
    one continuous session — objects.yaml currently gives OBJ-A/B/C the
    same world_pose, so only one can be physically present at a time)."""
    req = f'name: "{object_id}", type: MODEL'
    if dry_run:
        print(f'[dry-run] would call: gz service -s /world/{world_name}/remove '
              f'--req "{req}"')
        return

    subprocess.run(
        [
            'gz', 'service', '-s', f'/world/{world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req', req,
        ],
        check=True,
        timeout=8.0,
    )
