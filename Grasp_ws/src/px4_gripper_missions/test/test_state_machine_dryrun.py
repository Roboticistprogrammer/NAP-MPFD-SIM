"""State-machine ordering tests for PX4OffboardController / ArmController,
against a stubbed Node (no rclpy graph, no Gazebo, no PX4).

Requires rclpy + px4_msgs importable (the message *types* are used, just
never actually sent over DDS) — skipped outright if that's not available
outside a sourced ROS 2 environment.
"""
import time

import pytest

rclpy = pytest.importorskip('rclpy')
px4_msgs = pytest.importorskip('px4_msgs.msg')

from px4_gripper_missions.arm_controller import HOME_POSE, ArmController  # noqa: E402
from px4_gripper_missions.config_loader import JointTopic  # noqa: E402
from px4_gripper_missions.px4_offboard import (  # noqa: E402
    OFFBOARD_WARMUP_CYCLES,
    PX4OffboardController,
    Setpoint,
)


class FakePublisher:
    def __init__(self, name):
        self.name = name
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeClock:
    def now(self):
        class _Time:
            nanoseconds = int(time.time() * 1e9)
        return _Time()


class FakeLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass


class FakeNode:
    """Minimal stand-in for rclpy.Node: records what gets created/published
    without touching any real ROS graph."""

    def __init__(self):
        self.publishers = {}
        self.subscriptions = []
        self.timers = []
        self.call_log = []  # shared ordering log across all fakes

    def create_publisher(self, msg_type, topic, qos):
        pub = FakePublisher(topic)
        self.publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((topic, callback))
        return object()

    def create_timer(self, period_s, callback):
        timer = _FakeTimer(callback)
        self.timers.append(timer)
        return timer

    def get_logger(self):
        return FakeLogger()

    def get_clock(self):
        return FakeClock()


class _FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def tick(self):
        self.callback()

    def cancel(self):
        self.cancelled = True


# ---------------------------------------------------------------------------
# PX4OffboardController: engage_offboard_mode + arm only fire together,
# only after OFFBOARD_WARMUP_CYCLES heartbeat ticks — never before.
# ---------------------------------------------------------------------------

def test_offboard_does_not_arm_or_engage_before_warmup_completes():
    node = FakeNode()
    controller = PX4OffboardController(node, Setpoint(0, 0, -2.0, 0.0))
    controller.start_heartbeat()
    timer = node.timers[0]

    for _ in range(OFFBOARD_WARMUP_CYCLES):  # one short of the trigger tick
        timer.tick()

    vehicle_command_pub = node.publishers['/fmu/in/vehicle_command']
    assert len(vehicle_command_pub.messages) == 0, (
        'arm()/engage_offboard_mode() fired before the warmup count completed'
    )


def test_offboard_engages_then_arms_together_at_warmup_tick():
    node = FakeNode()
    controller = PX4OffboardController(node, Setpoint(0, 0, -2.0, 0.0))
    controller.start_heartbeat()
    timer = node.timers[0]

    for _ in range(OFFBOARD_WARMUP_CYCLES + 1):
        timer.tick()

    vehicle_command_pub = node.publishers['/fmu/in/vehicle_command']
    commands = [m.command for m in vehicle_command_pub.messages]
    # Both fired, in this order, and exactly once each — matches the
    # reference pattern (engage_offboard_mode() then arm() at the same tick).
    assert commands == [
        px4_msgs.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        px4_msgs.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    ]


def test_offboard_does_not_re_engage_on_later_ticks():
    node = FakeNode()
    controller = PX4OffboardController(node, Setpoint(0, 0, -2.0, 0.0))
    controller.start_heartbeat()
    timer = node.timers[0]

    for _ in range(OFFBOARD_WARMUP_CYCLES + 5):
        timer.tick()

    vehicle_command_pub = node.publishers['/fmu/in/vehicle_command']
    assert len(vehicle_command_pub.messages) == 2  # not re-fired on every later tick


def test_offboard_heartbeat_streams_setpoint_and_control_mode_every_tick():
    node = FakeNode()
    controller = PX4OffboardController(node, Setpoint(1.0, 2.0, -3.0, 0.5))
    controller.start_heartbeat()
    timer = node.timers[0]

    timer.tick()
    timer.tick()

    setpoint_pub = node.publishers['/fmu/in/trajectory_setpoint']
    mode_pub = node.publishers['/fmu/in/offboard_control_mode']
    assert len(setpoint_pub.messages) == 2
    assert len(mode_pub.messages) == 2
    assert list(setpoint_pub.messages[0].position) == [1.0, 2.0, -3.0]
    assert mode_pub.messages[0].position is True


def test_offboard_goto_position_updates_streamed_setpoint():
    node = FakeNode()
    controller = PX4OffboardController(node, Setpoint(0, 0, -2.0, 0.0))
    controller.start_heartbeat()
    timer = node.timers[0]

    controller.goto_position(5.0, 6.0, -7.0, yaw=1.0)
    timer.tick()

    setpoint_pub = node.publishers['/fmu/in/trajectory_setpoint']
    last = setpoint_pub.messages[-1]
    assert list(last.position) == [5.0, 6.0, -7.0]
    assert last.yaw == 1.0


# ---------------------------------------------------------------------------
# ArmController: close_gripper only after set_pose(grasp_pose) — checked
# via publish call ordering across the joint publishers.
# ---------------------------------------------------------------------------

def _joint_topic_map():
    return {
        name: JointTopic(ros_topic_name=f'armbot/joint/{name}/cmd_pos', gz_topic_name='')
        for name in ('base_yaw', 'shoulder_pitch', 'elbow_pitch', 'gripper')
    }


def test_arm_home_commands_all_zero():
    node = FakeNode()
    arm = ArmController(node, _joint_topic_map())

    arm.home(settle_s=0)

    for joint_name, radians in HOME_POSE.items():
        pub = node.publishers[f'armbot/joint/{joint_name}/cmd_pos']
        assert pub.messages[-1].data == pytest.approx(radians)


def test_close_gripper_only_fires_after_set_pose():
    node = FakeNode()
    arm = ArmController(node, _joint_topic_map())

    arm.set_pose({'base_yaw': 0.0, 'shoulder_pitch': 0.3, 'elbow_pitch': -0.2}, settle_s=0)
    arm.close_gripper(1.2)

    shoulder_pub = node.publishers['armbot/joint/shoulder_pitch/cmd_pos']
    gripper_pub = node.publishers['armbot/joint/gripper/cmd_pos']
    assert len(shoulder_pub.messages) == 1  # grasp pose was commanded
    assert len(gripper_pub.messages) == 1  # gripper closed after
    assert gripper_pub.messages[0].data == pytest.approx(1.2)


def test_lift_then_lower_returns_to_starting_pose():
    node = FakeNode()
    arm = ArmController(node, _joint_topic_map())

    arm.set_pose({'shoulder_pitch': 0.3, 'elbow_pitch': -0.2}, settle_s=0)
    arm.lift({'shoulder_pitch': -0.2, 'elbow_pitch': 0.1}, settle_s=0)
    arm.lower({'shoulder_pitch': -0.2, 'elbow_pitch': 0.1}, settle_s=0)

    shoulder_pub = node.publishers['armbot/joint/shoulder_pitch/cmd_pos']
    elbow_pub = node.publishers['armbot/joint/elbow_pitch/cmd_pos']
    # set_pose(0.3) -> lift(0.3-0.2=0.1) -> lower(0.1+0.2=0.3): back to start
    assert [round(m.data, 4) for m in shoulder_pub.messages] == [0.3, 0.1, 0.3]
    assert [round(m.data, 4) for m in elbow_pub.messages] == [-0.2, -0.1, -0.2]


def test_hold_samples_outcome_monitor_on_interval():
    node = FakeNode()
    arm = ArmController(node, _joint_topic_map())

    calls = []

    class FakeOutcomeMonitor:
        def sample(self):
            calls.append(time.monotonic())

    arm.hold(0.15, outcome_monitor=FakeOutcomeMonitor(), sample_interval_s=0.05)

    assert len(calls) == 3  # 0.05, 0.10, 0.15
