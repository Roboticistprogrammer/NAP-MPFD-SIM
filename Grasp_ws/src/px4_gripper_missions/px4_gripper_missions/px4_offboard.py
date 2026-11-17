"""Reusable PX4 ROS 2 offboard-control helper.

Genuinely new code for this project (see wayfinder/MAP.md and the plan
this package was built from — no PX4 Python offboard example existed
anywhere in this PX4-Autopilot checkout or this project). Structurally
adapted from the standard PX4 offboard pattern (10-cycle
OffboardControlMode warmup, then VEHICLE_CMD_DO_SET_MODE + arm) rather
than invented from scratch — encapsulated into a class usable alongside
ArmController on one shared rclpy.Node, instead of one hardcoded
Node-subclass script with a single fixed takeoff/land sequence.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# PX4 appends _v<N> to a uORB topic's DDS name when its message has
# MESSAGE_VERSION > 0 (msg/versioned/*.msg). Current PX4 main:
# VehicleLocalPosition v1, VehicleStatus v4; the /fmu/in topics used here
# are still v0. Check `ros2 topic list | grep fmu` after a PX4 update.
TOPIC_LOCAL_POSITION = '/fmu/out/vehicle_local_position_v1'
TOPIC_VEHICLE_STATUS = '/fmu/out/vehicle_status_v4'

# PX4 requires OffboardControlMode to stream for this many cycles before
# it will accept the mode switch (matches the reference pattern found
# this session, offboard_ctrl_example.py's offboard_setpoint_counter).
OFFBOARD_WARMUP_CYCLES = 10


@dataclass
class Setpoint:
    x: float
    y: float
    z: float
    yaw: float = 0.0
    # Velocity feed-forward (NED, m/s) added to the position loop's velocity
    # demand. 0 = none: PX4 then tracks the position setpoint with its P gain alone.
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class PX4OffboardController:
    """Wraps PX4's uXRCE-DDS offboard-control topics for one vehicle.

    Takes a *shared* rclpy.Node rather than being a Node subclass, so a
    mission script can compose this with ArmController on one node/one
    executor instead of running two competing nodes.
    """

    def __init__(
        self, node: Node, hover_pose: Setpoint, setpoint_rate_hz: float = 10.0,
        dry_run: bool = False,
    ):
        self._node = node
        self._log = node.get_logger()
        self._setpoint = hover_pose
        self._lock = threading.Lock()
        self._warmup_count = 0
        self._offboard_engaged = False
        self._dry_run = dry_run

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        self._offboard_mode_pub = node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self._trajectory_pub = node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self._vehicle_command_pub = node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        node.create_subscription(
            VehicleLocalPosition, TOPIC_LOCAL_POSITION,
            self._on_local_position, PX4_QOS)
        node.create_subscription(
            VehicleStatus, TOPIC_VEHICLE_STATUS,
            self._on_vehicle_status, PX4_QOS)

        self._heartbeat_timer = None
        self._setpoint_period_s = 1.0 / setpoint_rate_hz

    # -- subscriptions ----------------------------------------------------

    def _on_local_position(self, msg: VehicleLocalPosition) -> None:
        self.vehicle_local_position = msg

    def _on_vehicle_status(self, msg: VehicleStatus) -> None:
        self.vehicle_status = msg

    # -- heartbeat / setpoint streaming ------------------------------------

    def start_heartbeat(self) -> None:
        """Start streaming OffboardControlMode + TrajectorySetpoint.

        After OFFBOARD_WARMUP_CYCLES, automatically calls
        engage_offboard_mode() then arm() together, in that order —
        matching the reference pattern (offboard_ctrl_example.py) this
        was adapted from, where both fire at the same warmup-complete
        tick. Do NOT call arm() separately right after start_heartbeat()
        — PX4 needs setpoints streaming *before* it will accept either
        the mode switch or the arm command, so calling arm() early
        sends it before offboard mode is engaged, deviating from the
        tested sequence. wait_until_armed_and_offboard() is the
        intended way to block until both have actually taken effect.
        """
        self._heartbeat_timer = self._node.create_timer(
            self._setpoint_period_s, self._on_heartbeat_tick)

    def _on_heartbeat_tick(self) -> None:
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        if self._warmup_count == OFFBOARD_WARMUP_CYCLES and not self._offboard_engaged:
            self.engage_offboard_mode()
            self.arm()
            self._offboard_engaged = True
        if self._warmup_count <= OFFBOARD_WARMUP_CYCLES:
            self._warmup_count += 1

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._now_us()
        self._offboard_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self) -> None:
        with self._lock:
            sp = self._setpoint
        msg = TrajectorySetpoint()
        # px4_msgs' rosidl-generated setter rejects non-float elements
        # (e.g. a plain Python int slipping in from a YAML value like
        # `x: 0` instead of `x: 0.0`, or an int literal in a caller's
        # Setpoint(...)) — explicit float() avoids that crashing at
        # publish time (caught by test_offboard_engages_then_arms_together_at_warmup_tick).
        msg.position = [float(sp.x), float(sp.y), float(sp.z)]
        msg.velocity = [float(sp.vx), float(sp.vy), float(sp.vz)]
        msg.yaw = float(sp.yaw)
        msg.timestamp = self._now_us()
        self._trajectory_pub.publish(msg)

    def _now_us(self) -> int:
        return int(self._node.get_clock().now().nanoseconds / 1000)

    # -- commands -----------------------------------------------------------

    def _publish_vehicle_command(self, command: int, **params) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)
        msg.param6 = params.get('param6', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._vehicle_command_pub.publish(msg)

    def arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self._log.info('PX4: arm command sent')

    def disarm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self._log.info('PX4: disarm command sent')

    def engage_offboard_mode(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self._log.info('PX4: switching to offboard mode')

    def land(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self._log.info('PX4: switching to land mode')

    # -- motion ---------------------------------------------------------------

    def goto_position(self, x: float, y: float, z: float, yaw: float = 0.0,
                      vx: float = 0.0, vy: float = 0.0, vz: float = 0.0) -> None:
        """Update the streamed setpoint (optionally with a NED velocity
        feed-forward). Non-blocking — the heartbeat timer (already running)
        picks it up on its next tick."""
        with self._lock:
            self._setpoint = Setpoint(x, y, z, yaw, vx, vy, vz)

    def hold_position(self, duration_s: float) -> None:
        """Block for duration_s while the heartbeat timer keeps
        streaming the current setpoint on the node's executor thread."""
        time.sleep(duration_s)

    # -- state waits ------------------------------------------------------------

    def wait_until_armed_and_offboard(self, timeout_s: float = 10.0) -> None:
        if self._dry_run:
            self._log.info('[dry-run] would wait for ARMED+OFFBOARD; assuming success')
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.vehicle_status
            if (status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                return
            time.sleep(0.1)
        raise TimeoutError(
            f'Vehicle did not reach ARMED+OFFBOARD within {timeout_s}s '
            f'(last arming_state={self.vehicle_status.arming_state}, '
            f'nav_state={self.vehicle_status.nav_state})'
        )

    def wait_until_landed(self, timeout_s: float = 15.0) -> None:
        if self._dry_run:
            self._log.info('[dry-run] would wait for landed/disarmed; assuming success')
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                return
            time.sleep(0.2)
        raise TimeoutError(f'Vehicle did not report disarmed/landed within {timeout_s}s')

    # -- lifecycle ------------------------------------------------------------

    def shutdown(self) -> None:
        if self._heartbeat_timer is not None:
            self._heartbeat_timer.cancel()
            self._heartbeat_timer = None
