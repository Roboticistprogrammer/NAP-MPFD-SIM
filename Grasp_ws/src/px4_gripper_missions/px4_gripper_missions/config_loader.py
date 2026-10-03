"""Pure YAML -> dataclass loading for objects/bridge/scenario configs.

Deliberately has no rclpy/ROS-node dependency, so it's testable with
plain pytest (see test/test_config_loader.py) with no ROS graph, no
Gazebo, no PX4 running.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # pragma: no cover - only missing outside a sourced ROS env
    get_package_share_directory = None

PACKAGE_NAME = 'px4_gripper_missions'

# Matches the <world name="..."> in
# PX4-Autopilot/Tools/simulation/gz/worlds/x500_gripper_armbot.sdf —
# single source of truth for the world name used to build gz
# service/topic paths (e.g. /world/<name>/set_pose,
# /world/<name>/pose/info in config/bridge.yaml).
WORLD_NAME = 'x500_gripper_armbot'


class ConfigError(ValueError):
    """Raised for malformed or inconsistent config content."""


def get_config_dir() -> str:
    """Locate the installed config/ dir; fall back to the source tree.

    The fallback lets --dry-run and the unit tests run straight out of
    a `git clone` / IDE, without requiring `colcon build` first.
    """
    if get_package_share_directory is not None:
        try:
            return os.path.join(get_package_share_directory(PACKAGE_NAME), 'config')
        except Exception:
            pass
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'config')


# --------------------------------------------------------------------------
# Objects
# --------------------------------------------------------------------------

@dataclass
class GripperRange:
    open_position: float
    closed_position: float


@dataclass
class Friction:
    mu: float
    mu2: float


@dataclass
class ObjectDef:
    object_id: str
    description: str
    shape: str
    size_m: List[float]
    mass_kg: float
    friction: Friction
    world_pose: List[float]
    grasp_pose: Dict[str, float]
    gripper: GripperRange


@dataclass
class ObjectsConfig:
    active_object: str
    objects: Dict[str, ObjectDef]

    def get(self, object_id: Optional[str] = None) -> ObjectDef:
        key = object_id or self.active_object
        if key not in self.objects:
            raise ConfigError(
                f"Unknown object_id '{key}'; known objects: {sorted(self.objects)}"
            )
        return self.objects[key]


def load_objects_config(path: Optional[str] = None) -> ObjectsConfig:
    path = path or os.path.join(get_config_dir(), 'objects.yaml')
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)

    objects = {}
    for object_id, obj in raw.get('objects', {}).items():
        objects[object_id] = ObjectDef(
            object_id=object_id,
            description=obj.get('description', ''),
            shape=obj['shape'],
            size_m=list(obj['size_m']),
            mass_kg=float(obj['mass_kg']),
            friction=Friction(**obj['friction']),
            world_pose=list(obj['world_pose']),
            grasp_pose=dict(obj['grasp_pose']),
            gripper=GripperRange(**obj['gripper']),
        )

    active_object = raw.get('active_object')
    if active_object is None or active_object not in objects:
        raise ConfigError(
            f"objects.yaml's active_object '{active_object}' is not a defined object"
        )

    return ObjectsConfig(active_object=active_object, objects=objects)


# --------------------------------------------------------------------------
# Bridge (joint command topic map)
# --------------------------------------------------------------------------

@dataclass
class JointTopic:
    ros_topic_name: str
    gz_topic_name: str


def load_joint_topic_map(path: Optional[str] = None) -> Dict[str, JointTopic]:
    """Parse bridge.yaml, return {short_joint_name: JointTopic(ros, gz)} for
    every ROS_TO_GZ .../joint/<short>/cmd_pos entry.

    Keeps ArmController from hardcoding topic names that already live in
    bridge.yaml — one source of truth for both the ROS-side topic string
    (normal path) and the raw gz topic (use_gz_transport_fallback path).
    """
    path = path or os.path.join(get_config_dir(), 'bridge.yaml')
    with open(path, 'r') as f:
        entries = yaml.safe_load(f) or []

    topic_map: Dict[str, JointTopic] = {}
    prefix = 'armbot/joint/'
    suffix = '/cmd_pos'
    for entry in entries:
        if entry.get('direction') != 'ROS_TO_GZ':
            continue
        ros_topic = entry.get('ros_topic_name', '')
        if ros_topic.startswith(prefix) and ros_topic.endswith(suffix):
            short_name = ros_topic[len(prefix):-len(suffix)]
            topic_map[short_name] = JointTopic(
                ros_topic_name=ros_topic,
                gz_topic_name=entry.get('gz_topic_name', ''),
            )

    if not topic_map:
        raise ConfigError(f"No .../joint/<name>/cmd_pos ROS_TO_GZ entries found in {path}")

    return topic_map


def load_bridged_pose_topic(path: Optional[str] = None) -> str:
    """Parse bridge.yaml, return the ROS-side topic name for the
    GZ_TO_ROS /world/<world>/pose/info entry — what
    GraspOutcomeMonitor subscribes to."""
    path = path or os.path.join(get_config_dir(), 'bridge.yaml')
    with open(path, 'r') as f:
        entries = yaml.safe_load(f) or []

    for entry in entries:
        gz_topic = entry.get('gz_topic_name', '')
        if entry.get('direction') == 'GZ_TO_ROS' and gz_topic.endswith('/pose/info'):
            return entry['ros_topic_name']

    raise ConfigError(f"No .../pose/info GZ_TO_ROS entry found in {path}")


# --------------------------------------------------------------------------
# Scenario configs
# --------------------------------------------------------------------------

@dataclass
class HoverPose:
    x: float
    y: float
    z: float
    yaw: float = 0.0


@dataclass
class BaseScenarioConfig:
    scenario_id: str
    hover_pose: HoverPose
    hold_duration_s: float
    condition_label: str
    land_at_end: bool
    outcome_z_drop_threshold_m: float
    outcome_sample_interval_s: float


@dataclass
class SC01Config(BaseScenarioConfig):
    pass


@dataclass
class SC02Config(BaseScenarioConfig):
    approach_pose: Dict[str, float] = field(default_factory=dict)
    approach_settle_s: float = 1.0
    lift_delta: Dict[str, float] = field(default_factory=dict)
    lower_settle_s: float = 1.0


@dataclass
class SC03Config(SC02Config):
    num_cycles: int = 1
    inter_cycle_pause_s: float = 0.0
    bag_per_cycle: bool = False


@dataclass
class SC04Config(BaseScenarioConfig):
    approach_pose: Dict[str, float] = field(default_factory=dict)
    approach_settle_s: float = 1.0
    orientation_overrides: List[List[float]] = field(default_factory=list)


@dataclass
class SC05Config(SC02Config):
    object_sequence: List[str] = field(default_factory=list)
    inter_object_pause_s: float = 0.0


_SCENARIO_CLASSES = {
    'SC01': SC01Config,
    'SC02': SC02Config,
    'SC03': SC03Config,
    'SC04': SC04Config,
    'SC05': SC05Config,
}

_BASE_FIELDS = {f for f in BaseScenarioConfig.__dataclass_fields__}


def load_scenario_config(scenario_id: str, path: Optional[str] = None) -> BaseScenarioConfig:
    scenario_id = scenario_id.upper()
    if scenario_id not in _SCENARIO_CLASSES:
        raise ConfigError(
            f"Unknown scenario_id '{scenario_id}'; known: {sorted(_SCENARIO_CLASSES)}"
        )

    path = path or os.path.join(
        get_config_dir(), 'scenarios', f'{scenario_id.lower()}.yaml'
    )
    with open(path, 'r') as f:
        raw = yaml.safe_load(f)

    if raw.get('scenario_id', scenario_id).upper() != scenario_id:
        raise ConfigError(
            f"{path}'s scenario_id '{raw.get('scenario_id')}' does not match "
            f"requested '{scenario_id}'"
        )

    raw = dict(raw)
    raw['hover_pose'] = HoverPose(**raw['hover_pose'])
    cls = _SCENARIO_CLASSES[scenario_id]
    known_fields = set(cls.__dataclass_fields__)
    kwargs = {k: v for k, v in raw.items() if k in known_fields}
    return cls(**kwargs)


def validate_sc05_ordering(config: SC05Config, objects_config: ObjectsConfig) -> None:
    """SC-05 must sweep in ascending mass order per Scenarios.md's
    definition ("ascending weight order") — catches a config edit that
    silently breaks that invariant."""
    masses = [objects_config.get(obj_id).mass_kg for obj_id in config.object_sequence]
    if masses != sorted(masses):
        raise ConfigError(
            f"sc05.yaml's object_sequence {config.object_sequence} is not in "
            f"ascending mass_kg order (masses: {masses})"
        )
