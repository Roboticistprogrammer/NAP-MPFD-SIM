"""SC-01: Static grasp-and-hold (Scenarios.md).

"Arm stationary, close on object, hold N seconds, release" — the arm
moves to its grasp pose exactly ONCE, up front; only the gripper joint
is actuated afterward for close/hold/release. This is the actual
differentiator from SC-02 (which adds a real approach + lift/lower).

This one-shot arm.set_pose(grasp_pose) — no incremental approach — is
itself a candidate answer to wayfinder ticket 003's open choreography
question (px4_gazebo_sim_setup/wayfinder/tickets/003-grasp-object-and-choreography.md),
worth reflecting back into that ticket once tried against a live sim.
"""
from __future__ import annotations

from px4_gripper_missions.arm_controller import remove_object, spawn_object
from px4_gripper_missions.config_loader import JointTopic, ObjectsConfig, SC01Config, WORLD_NAME
from px4_gripper_missions.mission_rig import MissionRig
from px4_gripper_missions.px4_offboard import Setpoint
from px4_gripper_missions.trial_logger import TrialLogger


def main(
    config: SC01Config,
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
    outcome = 'UNKNOWN'
    try:
        spawn_object(obj, world_name=WORLD_NAME, dry_run=dry_run)

        with MissionRig(
            node_name='sc01_static_grasp_hold',
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

            # Station-keeping only — SC-01 doesn't translate, the
            # heartbeat timer just keeps streaming this hover setpoint
            # for the whole trial.
            rig.offboard.goto_position(hover.x, hover.y, hover.z, hover.yaw)

            rig.arm.home()
            rig.arm.set_pose(obj.grasp_pose)

            rig.outcome.start(obj.object_id, reference_pose_z=obj.world_pose[2])
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
