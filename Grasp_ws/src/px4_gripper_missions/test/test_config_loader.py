"""Pure-logic tests for config_loader.py — no ROS/Gazebo/PX4 needed."""
import os
import textwrap

import pytest

from px4_gripper_missions import config_loader

REPO_CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'
)


# ---------------------------------------------------------------------------
# Synthetic-data tests: exercise parsing/validation logic in isolation.
# ---------------------------------------------------------------------------

MINIMAL_OBJECT_YAML = textwrap.dedent("""
    active_object: OBJ-X
    objects:
      OBJ-X:
        description: "test object"
        shape: box
        size_m: [0.03, 0.03, 0.03]
        mass_kg: 0.05
        friction: {mu: 0.8, mu2: 0.8}
        world_pose: [0.1, 0.0, 0.05, 0, 0, 0]
        grasp_pose: {base_yaw: 0.0, shoulder_pitch: 0.3, elbow_pitch: -0.2}
        gripper: {open_position: 0.0, closed_position: 1.2}
""")


def test_load_objects_config_parses_minimal_yaml(tmp_path):
    p = tmp_path / 'objects.yaml'
    p.write_text(MINIMAL_OBJECT_YAML)

    cfg = config_loader.load_objects_config(str(p))

    assert cfg.active_object == 'OBJ-X'
    obj = cfg.get()
    assert obj.mass_kg == pytest.approx(0.05)
    assert obj.grasp_pose == {'base_yaw': 0.0, 'shoulder_pitch': 0.3, 'elbow_pitch': -0.2}
    assert obj.gripper.closed_position == pytest.approx(1.2)


def test_load_objects_config_unknown_active_object_raises(tmp_path):
    p = tmp_path / 'objects.yaml'
    p.write_text(MINIMAL_OBJECT_YAML.replace('active_object: OBJ-X', 'active_object: NOPE'))

    with pytest.raises(config_loader.ConfigError):
        config_loader.load_objects_config(str(p))


def test_objects_config_get_unknown_id_raises(tmp_path):
    p = tmp_path / 'objects.yaml'
    p.write_text(MINIMAL_OBJECT_YAML)
    cfg = config_loader.load_objects_config(str(p))

    with pytest.raises(config_loader.ConfigError):
        cfg.get('NOT-A-REAL-OBJECT')


MINIMAL_BRIDGE_YAML = textwrap.dedent("""
    - ros_topic_name: "armbot/joint/base_yaw/cmd_pos"
      gz_topic_name: "/model/armbot/joint/base_yaw_joint/cmd_pos"
      ros_type_name: "std_msgs/msg/Float64"
      gz_type_name: "gz.msgs.Double"
      direction: ROS_TO_GZ
    - ros_topic_name: "armbot/joint/gripper/cmd_pos"
      gz_topic_name: "/model/armbot/joint/gripper_joint/cmd_pos"
      ros_type_name: "std_msgs/msg/Float64"
      gz_type_name: "gz.msgs.Double"
      direction: ROS_TO_GZ
    - ros_topic_name: "armbot/joint_states"
      gz_topic_name: "/model/armbot/joint_states"
      ros_type_name: "sensor_msgs/msg/JointState"
      gz_type_name: "gz.msgs.Model"
      direction: GZ_TO_ROS
    - ros_topic_name: "world/testworld/pose_info"
      gz_topic_name: "/world/testworld/pose/info"
      ros_type_name: "tf2_msgs/msg/TFMessage"
      gz_type_name: "gz.msgs.Pose_V"
      direction: GZ_TO_ROS
""")


def test_load_joint_topic_map(tmp_path):
    p = tmp_path / 'bridge.yaml'
    p.write_text(MINIMAL_BRIDGE_YAML)

    topic_map = config_loader.load_joint_topic_map(str(p))

    assert set(topic_map) == {'base_yaw', 'gripper'}
    assert topic_map['base_yaw'].ros_topic_name == 'armbot/joint/base_yaw/cmd_pos'
    assert topic_map['base_yaw'].gz_topic_name == '/model/armbot/joint/base_yaw_joint/cmd_pos'


def test_load_joint_topic_map_empty_raises(tmp_path):
    p = tmp_path / 'bridge.yaml'
    p.write_text('[]')
    with pytest.raises(config_loader.ConfigError):
        config_loader.load_joint_topic_map(str(p))


def test_load_bridged_pose_topic(tmp_path):
    p = tmp_path / 'bridge.yaml'
    p.write_text(MINIMAL_BRIDGE_YAML)
    assert config_loader.load_bridged_pose_topic(str(p)) == 'world/testworld/pose_info'


MINIMAL_SC01_YAML = textwrap.dedent("""
    scenario_id: SC01
    hover_pose: {x: 0.0, y: 0.0, z: -2.0, yaw: 0.0}
    hold_duration_s: 5.0
    condition_label: normal
    land_at_end: true
    outcome_z_drop_threshold_m: 0.02
    outcome_sample_interval_s: 0.5
""")


def test_load_scenario_config_sc01(tmp_path):
    p = tmp_path / 'sc01.yaml'
    p.write_text(MINIMAL_SC01_YAML)

    cfg = config_loader.load_scenario_config('SC01', str(p))

    assert cfg.scenario_id == 'SC01'
    assert cfg.hover_pose.z == pytest.approx(-2.0)
    assert cfg.hold_duration_s == pytest.approx(5.0)


def test_load_scenario_config_mismatched_scenario_id_raises(tmp_path):
    p = tmp_path / 'sc01.yaml'
    p.write_text(MINIMAL_SC01_YAML)

    with pytest.raises(config_loader.ConfigError):
        config_loader.load_scenario_config('SC02', str(p))


def test_load_scenario_config_unknown_id_raises():
    with pytest.raises(config_loader.ConfigError):
        config_loader.load_scenario_config('SC99')


def test_validate_sc05_ordering_accepts_ascending_mass(tmp_path):
    objects_path = tmp_path / 'objects.yaml'
    objects_path.write_text(textwrap.dedent("""
        active_object: OBJ-A
        objects:
          OBJ-A: {description: "", shape: box, size_m: [0.03,0.03,0.03], mass_kg: 0.05,
                  friction: {mu: 0.8, mu2: 0.8}, world_pose: [0.1,0,0.05,0,0,0],
                  grasp_pose: {base_yaw: 0.0}, gripper: {open_position: 0.0, closed_position: 1.0}}
          OBJ-B: {description: "", shape: box, size_m: [0.03,0.03,0.03], mass_kg: 0.10,
                  friction: {mu: 0.8, mu2: 0.8}, world_pose: [0.1,0,0.05,0,0,0],
                  grasp_pose: {base_yaw: 0.0}, gripper: {open_position: 0.0, closed_position: 1.0}}
    """))
    objects_cfg = config_loader.load_objects_config(str(objects_path))

    sc05_path = tmp_path / 'sc05.yaml'
    sc05_path.write_text(textwrap.dedent("""
        scenario_id: SC05
        hover_pose: {x: 0, y: 0, z: -2.0, yaw: 0}
        hold_duration_s: 5.0
        condition_label: normal
        land_at_end: true
        outcome_z_drop_threshold_m: 0.02
        outcome_sample_interval_s: 0.5
        object_sequence: [OBJ-A, OBJ-B]
        inter_object_pause_s: 1.0
    """))
    sc05_cfg = config_loader.load_scenario_config('SC05', str(sc05_path))

    config_loader.validate_sc05_ordering(sc05_cfg, objects_cfg)  # should not raise

    sc05_cfg.object_sequence = ['OBJ-B', 'OBJ-A']  # descending -> invalid
    with pytest.raises(config_loader.ConfigError):
        config_loader.validate_sc05_ordering(sc05_cfg, objects_cfg)


# ---------------------------------------------------------------------------
# Real bundled config: catches drift between config_loader and the actual
# committed YAML files, not just synthetic fixtures.
# ---------------------------------------------------------------------------

@pytest.mark.skipif(not os.path.isdir(REPO_CONFIG_DIR), reason='repo config/ dir not found')
class TestRealBundledConfig:
    def test_objects_yaml_loads(self):
        cfg = config_loader.load_objects_config(os.path.join(REPO_CONFIG_DIR, 'objects.yaml'))
        assert set(cfg.objects) == {'OBJ-A', 'OBJ-B', 'OBJ-C'}

    def test_bridge_yaml_loads(self):
        topic_map = config_loader.load_joint_topic_map(os.path.join(REPO_CONFIG_DIR, 'bridge.yaml'))
        assert set(topic_map) == {'base_yaw', 'shoulder_pitch', 'elbow_pitch', 'gripper'}

    def test_bridge_yaml_pose_topic_loads(self):
        topic = config_loader.load_bridged_pose_topic(os.path.join(REPO_CONFIG_DIR, 'bridge.yaml'))
        assert topic == 'world/x500_gripper_armbot/pose_info'

    @pytest.mark.parametrize('scenario_id', ['SC01', 'SC02', 'SC03', 'SC04', 'SC05'])
    def test_all_scenario_yamls_load(self, scenario_id):
        path = os.path.join(REPO_CONFIG_DIR, 'scenarios', f'{scenario_id.lower()}.yaml')
        cfg = config_loader.load_scenario_config(scenario_id, path)
        assert cfg.scenario_id == scenario_id

    def test_sc05_object_sequence_is_ascending_mass(self):
        objects_cfg = config_loader.load_objects_config(
            os.path.join(REPO_CONFIG_DIR, 'objects.yaml'))
        sc05_cfg = config_loader.load_scenario_config(
            'SC05', os.path.join(REPO_CONFIG_DIR, 'scenarios', 'sc05.yaml'))
        config_loader.validate_sc05_ordering(sc05_cfg, objects_cfg)  # should not raise
