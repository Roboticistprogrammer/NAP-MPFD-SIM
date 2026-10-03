"""Placeholder grasp-success/failure heuristic.

No contact-sensor or attachment-state signal exists anywhere in this
project yet (see wayfinder ticket 003 — real grasp-outcome detection is
explicitly future work, not resolved by this package). This is a crude
but honest placeholder: track the target object's world-frame Z height
from grasp-close through the hold window, and call it a FAILURE if it
ever drops more than z_drop_threshold_m below where it started (object
fell / was never actually held). It cannot detect "held but slipping
sideways" or a grip that looks fine but never actually contacted the
object — replace with a real contact/attachment signal once ticket 003
picks one, without needing to change trial_logger's schema (the
SUCCESS/FAILURE/UNKNOWN vocabulary stays the same).

The classification logic (classify_outcome) is deliberately a pure
function with no ROS dependency, so it's unit-testable against
synthetic Z-height series with no sim/topics involved
(test/test_grasp_outcome.py).
"""
from __future__ import annotations

from typing import List, Optional

from rclpy.node import Node
from tf2_msgs.msg import TFMessage

SUCCESS = 'SUCCESS'
FAILURE = 'FAILURE'
UNKNOWN = 'UNKNOWN'


def classify_outcome(
    z_readings: List[float], baseline_z: Optional[float], z_drop_threshold_m: float
) -> str:
    if baseline_z is None or not z_readings:
        return UNKNOWN
    for z in z_readings:
        if z < baseline_z - z_drop_threshold_m:
            return FAILURE
    return SUCCESS


class GraspOutcomeMonitor:
    """Tracks one object's world-frame Z position via the bridged
    /world/.../pose/info -> TFMessage topic (config/bridge.yaml),
    filtering by child_frame_id == object model name.

    NOTE: the exact child_frame_id string ros_gz_bridge assigns for a
    given gz entity name has not been confirmed against a live topic
    echo in this planning/implementation pass — matching is
    best-effort (exact match or suffix match) and logged loudly the
    first time a sample succeeds so a live run can confirm/correct it.
    """

    def __init__(
        self,
        node: Node,
        pose_topic: str,
        z_drop_threshold_m: float,
        dry_run: bool = False,
    ):
        self._node = node
        self._log = node.get_logger()
        self._z_drop_threshold_m = z_drop_threshold_m
        self._dry_run = dry_run

        self._object_frame_id: Optional[str] = None
        self._latest_z: Optional[float] = None
        self._baseline_z: Optional[float] = None
        self._readings: List[float] = []
        self._logged_first_match = False

        if not dry_run:
            node.create_subscription(TFMessage, pose_topic, self._on_tf, 10)

    def _on_tf(self, msg: TFMessage) -> None:
        if self._object_frame_id is None:
            return
        for transform in msg.transforms:
            frame = transform.child_frame_id
            if frame == self._object_frame_id or frame.endswith('::' + self._object_frame_id):
                self._latest_z = transform.transform.translation.z
                if not self._logged_first_match:
                    self._log.info(
                        f"GraspOutcomeMonitor: matched frame '{frame}' for "
                        f"object_id '{self._object_frame_id}'"
                    )
                    self._logged_first_match = True
                return

    def start(self, object_id: str, reference_pose_z: Optional[float] = None) -> None:
        """Call at grasp-close time. reference_pose_z, if given (e.g.
        from objects.yaml's world_pose), seeds the baseline immediately;
        otherwise the first live sample() becomes the baseline."""
        self._object_frame_id = object_id
        self._readings = []
        self._baseline_z = reference_pose_z
        self._latest_z = None
        self._logged_first_match = False

    def sample(self) -> None:
        if self._dry_run:
            return
        if self._latest_z is None:
            return
        if self._baseline_z is None:
            self._baseline_z = self._latest_z
        self._readings.append(self._latest_z)

    def outcome(self) -> str:
        if self._dry_run:
            return UNKNOWN
        return classify_outcome(self._readings, self._baseline_z, self._z_drop_threshold_m)
