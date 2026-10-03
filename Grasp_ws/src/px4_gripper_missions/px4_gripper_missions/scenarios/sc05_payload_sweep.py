"""SC-05: Graduated payload sweep (Scenarios.md).

"Same grasp motion across OBJ-A -> OBJ-C in ascending weight order,
single session" — one continuous flight (arm stays airborne across all
objects, no land-and-relaunch between them), swapping which object is
present via spawn_object/remove_object between sub-trials since
objects.yaml gives OBJ-A/B/C the same world_pose (only one is
physically present at a time). Reuses SC-02's
approach/grasp/lift/hold/lower/release motion per object, unchanged —
proving the shared-primitive set covers this without new arm/PX4
capabilities, only a new outer loop (same conclusion as SC-03).
"""
from __future__ import annotations

import time

from px4_gripper_missions.arm_controller import remove_object, spawn_object
from px4_gripper_missions.config_loader import (
    JointTopic,
    ObjectsConfig,
    SC05Config,
    WORLD_NAME,
    validate_sc05_ordering,
)
from px4_gripper_missions.mission_rig import MissionRig
from px4_gripper_missions.px4_offboard import Setpoint
from px4_gripper_missions.trial_logger import TrialLogger


def main(
    config: SC05Config,
    objects_config: ObjectsConfig,
    joint_topic_map: dict[str, JointTopic],
    pose_topic: str,
    trial_logger: TrialLogger,
    dry_run: bool = False,
    use_gz_transport_fallback: bool = False,
) -> str:
    validate_sc05_ordering(config, objects_config)
    hover = config.hover_pose

    trial_logger.start()
    per_object_outcomes = []
    trial_logger.extra_metadata['objects'] = per_object_outcomes
    final_outcome = 'UNKNOWN'
    try:
        with MissionRig(
            node_name='sc05_payload_sweep',
            hover_pose=Setpoint(hover.x, hover.y, hover.z, hover.yaw),
            joint_topic_map=joint_topic_map,
            # objects.yaml's per-object gripper.open_position is expected
            # to match across OBJ-A/B/C (same gripper hardware) — using
            # the first object's value for the whole session.
            gripper_open_position=objects_config.get(config.object_sequence[0]).gripper.open_position,
            pose_topic=pose_topic,
            z_drop_threshold_m=config.outcome_z_drop_threshold_m,
            use_gz_transport_fallback=use_gz_transport_fallback,
            dry_run=dry_run,
        ) as rig:
            rig.offboard.start_heartbeat()
            rig.offboard.wait_until_armed_and_offboard(timeout_s=10.0)
            rig.offboard.goto_position(hover.x, hover.y, hover.z, hover.yaw)

            for index, object_id in enumerate(config.object_sequence):
                obj = objects_config.get(object_id)
                spawn_object(obj, world_name=WORLD_NAME, dry_run=dry_run)

                rig.arm.home()
                rig.arm.approach(config.approach_pose, settle_s=config.approach_settle_s)
                rig.arm.set_pose(obj.grasp_pose)

                rig.outcome.start(obj.object_id, reference_pose_z=obj.world_pose[2])
                rig.arm.close_gripper(obj.gripper.closed_position)
                rig.arm.lift(config.lift_delta)

                rig.arm.hold(
                    config.hold_duration_s,
                    outcome_monitor=rig.outcome,
                    sample_interval_s=config.outcome_sample_interval_s,
                )
                object_outcome = rig.outcome.outcome()
                per_object_outcomes.append({
                    'object_id': object_id,
                    'mass_kg': obj.mass_kg,
                    'grasp_outcome': object_outcome,
                })
                final_outcome = object_outcome

                rig.arm.lower(config.lift_delta, settle_s=config.lower_settle_s)
                rig.arm.open_gripper()
                remove_object(object_id, world_name=WORLD_NAME, dry_run=dry_run)

                if index < len(config.object_sequence) - 1 and config.inter_object_pause_s > 0:
                    time.sleep(config.inter_object_pause_s)

            if config.land_at_end:
                rig.offboard.land()
                rig.offboard.wait_until_landed(timeout_s=15.0)
    finally:
        trial_logger.stop(final_outcome)

    return final_outcome
