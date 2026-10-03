"""Shared node/executor scaffolding composing PX4OffboardController +
ArmController + GraspOutcomeMonitor onto one rclpy.Node.

Not called out as its own file in the original plan (which described
this boilerplate inline as part of each scenario's sequence) — split
out during implementation once it was clearly the same ~20 lines
repeated verbatim across all five scenario scripts. Nothing about the
plan's approved module interfaces changes; this just avoids five copies
of "construct a node, spin it on a background thread, tear it down
cleanly."
"""
from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from px4_gripper_missions.arm_controller import ArmController
from px4_gripper_missions.config_loader import JointTopic
from px4_gripper_missions.grasp_outcome import GraspOutcomeMonitor
from px4_gripper_missions.px4_offboard import PX4OffboardController, Setpoint


class MissionRig:
    """Context manager: on __enter__, brings up rclpy + one Node with
    .offboard / .arm / .outcome composed onto it, spinning on a
    background thread. On __exit__, tears everything down cleanly
    regardless of how the mission sequence exited."""

    def __init__(
        self,
        node_name: str,
        hover_pose: Setpoint,
        joint_topic_map: dict[str, JointTopic],
        gripper_open_position: float,
        pose_topic: str,
        z_drop_threshold_m: float,
        use_gz_transport_fallback: bool = False,
        dry_run: bool = False,
    ):
        self._node_name = node_name
        self._hover_pose = hover_pose
        self._joint_topic_map = joint_topic_map
        self._gripper_open_position = gripper_open_position
        self._pose_topic = pose_topic
        self._z_drop_threshold_m = z_drop_threshold_m
        self._use_gz_transport_fallback = use_gz_transport_fallback
        self._dry_run = dry_run

        self._node = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

        self.offboard: Optional[PX4OffboardController] = None
        self.arm: Optional[ArmController] = None
        self.outcome: Optional[GraspOutcomeMonitor] = None

    def __enter__(self) -> 'MissionRig':
        rclpy.init(args=None)
        self._node = rclpy.create_node(self._node_name)

        self.offboard = PX4OffboardController(
            self._node, self._hover_pose, dry_run=self._dry_run)
        self.arm = ArmController(
            self._node,
            self._joint_topic_map,
            gripper_open_position=self._gripper_open_position,
            use_gz_transport_fallback=self._use_gz_transport_fallback,
            dry_run=self._dry_run,
        )
        self.outcome = GraspOutcomeMonitor(
            self._node,
            self._pose_topic,
            self._z_drop_threshold_m,
            dry_run=self._dry_run,
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if self.offboard is not None:
            self.offboard.shutdown()
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
