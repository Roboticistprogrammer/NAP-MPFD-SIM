"""SC-04: Variable approach orientation (Scenarios.md).

"Object rotated/offset between trials, same grasp target" — the one
scenario that moves the *object* rather than just the arm. Selects one
entry from sc04.yaml's orientation_overrides (by --orientation-index,
see run_scenario.py) and spawns the object directly at that pose (via
arm_controller.spawn_object with an overridden world_pose, rather than
spawning at the default pose and repositioning it — simpler and doesn't
assume anything was already spawned) before running the same
approach/grasp/hold/release sequence as SC-01/02, with a fixed
grasp_pose throughout (per the scenario definition: target stays fixed,
only the object's presented orientation/offset changes).

arm_controller.place_object also exists as a standalone
reposition-an-already-spawned-object primitive, provided for other
scenarios/future use even though this script doesn't need it.
"""
from __future__ import annotations

import dataclasses

from px4_gripper_missions.arm_controller import remove_object, spawn_object
from px4_gripper_missions.config_loader import JointTopic, ObjectsConfig, SC04Config, WORLD_NAME
from px4_gripper_missions.mission_rig import MissionRig
from px4_gripper_missions.px4_offboard import Setpoint
from px4_gripper_missions.trial_logger import TrialLogger


def main(
    config: SC04Config,
    objects_config: ObjectsConfig,
    joint_topic_map: dict[str, JointTopic],
    pose_topic: str,
    trial_logger: TrialLogger,
    object_id: str = None,
    orientation_index: int = 0,
    dry_run: bool = False,
    use_gz_transport_fallback: bool = False,
) -> str:
    obj = objects_config.get(object_id)
    hover = config.hover_pose

    if not (0 <= orientation_index < len(config.orientation_overrides)):
        raise IndexError(
            f'orientation_index {orientation_index} out of range for '
            f'{len(config.orientation_overrides)} entries in sc04.yaml'
        )
    ox, oy, oz, oroll, opitch, oyaw = config.orientation_overrides[orientation_index]
    trial_logger.extra_metadata['orientation_index'] = orientation_index
    positioned_obj = dataclasses.replace(obj, world_pose=[ox, oy, oz, oroll, opitch, oyaw])

    trial_logger.start()
    outcome = 'UNKNOWN'
    try:
        spawn_object(positioned_obj, world_name=WORLD_NAME, dry_run=dry_run)

        with MissionRig(
            node_name='sc04_variable_orientation',
            hover_pose=Setpoint(hover.x, hover.y, hover.z, hover.yaw),
            joint_topic_map=joint_topic_map,
            gripper_open_position=obj.gripper.open_position,
            pose_topic=pose_topic,
            z_drop_threshold_m=config.outcome_z_drop_threshold_m,
            use_gz_transport_fallback=use_gz_transport_fallback,
            dry_run=dry_run,
        ) as rig:
            rig.offboard.start_heartbeat()
            rig.offboard.wait_until_armed_and_offboard(timeout_s=10.0)
            rig.offboard.goto_position(hover.x, hover.y, hover.z, hover.yaw)

            rig.arm.home()
            rig.arm.approach(config.approach_pose, settle_s=config.approach_settle_s)
            # Fixed grasp target regardless of orientation_index — only
            # the object moved, per the scenario definition.
            rig.arm.set_pose(obj.grasp_pose)

            rig.outcome.start(obj.object_id, reference_pose_z=oz)
            rig.arm.close_gripper(obj.gripper.closed_position)

            rig.arm.hold(
                config.hold_duration_s,
                outcome_monitor=rig.outcome,
                sample_interval_s=config.outcome_sample_interval_s,
            )
            outcome = rig.outcome.outcome()

            rig.arm.open_gripper()

            if config.land_at_end:
                rig.offboard.land()
                rig.offboard.wait_until_landed(timeout_s=15.0)
    finally:
        remove_object(obj.object_id, world_name=WORLD_NAME, dry_run=dry_run)
        trial_logger.stop(outcome)

    return outcome
