"""SC-03: Repeated-cycle endurance (Scenarios.md).

"Same object, N consecutive grasp cycles" — wraps SC-02's
approach/grasp/lift/hold/lower/release sequence in one continuous
flight (single arm/offboard session, not a land-and-relaunch per
cycle), tagging each cycle's outcome under trial_logger.extra_metadata
so per-cycle degradation trends are visible in the one sidecar JSON.

sc03.yaml's bag_per_cycle flag is currently informational only — this
implementation always uses the single trial_logger passed in (one bag
for the whole trial); splitting into one bag per cycle would need a
TrialLogger factory rather than one instance, deferred as a known
limitation rather than built speculatively (see README.md).
"""
from __future__ import annotations

import time

from px4_gripper_missions.arm_controller import remove_object, spawn_object
from px4_gripper_missions.config_loader import JointTopic, ObjectsConfig, SC03Config, WORLD_NAME
from px4_gripper_missions.mission_rig import MissionRig
from px4_gripper_missions.px4_offboard import Setpoint
from px4_gripper_missions.trial_logger import TrialLogger


def main(
    config: SC03Config,
    objects_config: ObjectsConfig,
    joint_topic_map: dict[str, JointTopic],
    pose_topic: str,
    trial_logger: TrialLogger,
    object_id: str = None,
    dry_run: bool = False,
    use_gz_transport_fallback: bool = False,
) -> str:
    obj = objects_config.get(object_id)
    hover = config.hover_pose

    trial_logger.start()
    cycle_outcomes = []
    trial_logger.extra_metadata['cycles'] = cycle_outcomes
    final_outcome = 'UNKNOWN'
    try:
        spawn_object(obj, world_name=WORLD_NAME, dry_run=dry_run)

        with MissionRig(
            node_name='sc03_repeated_cycle',
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

            for cycle_index in range(config.num_cycles):
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
                cycle_outcome = rig.outcome.outcome()
                cycle_outcomes.append({'cycle_index': cycle_index, 'grasp_outcome': cycle_outcome})
                final_outcome = cycle_outcome

                rig.arm.lower(config.lift_delta, settle_s=config.lower_settle_s)
                rig.arm.open_gripper()

                if cycle_index < config.num_cycles - 1 and config.inter_cycle_pause_s > 0:
                    time.sleep(config.inter_cycle_pause_s)

            if config.land_at_end:
                rig.offboard.land()
                rig.offboard.wait_until_landed(timeout_s=15.0)
    finally:
        remove_object(obj.object_id, world_name=WORLD_NAME, dry_run=dry_run)
        trial_logger.stop(final_outcome)

    return final_outcome
