"""CLI entrypoint: `run_scenario --scenario SC01 --trial 1 [...]`.

Loads config, builds a TrialLogger, and dispatches to the matching
scenarios/scXX module's main(). Doesn't itself talk to ROS/Gazebo — all
of that lives in the dispatched module + the shared helpers it composes
(mission_rig.MissionRig, arm_controller, px4_offboard, grasp_outcome).
"""
from __future__ import annotations

import argparse
import sys

from px4_gripper_missions import config_loader
from px4_gripper_missions.scenarios import (
    sc01_static_grasp_hold,
    sc02_full_cycle,
    sc03_repeated_cycle,
    sc04_variable_orientation,
    sc05_payload_sweep,
)
from px4_gripper_missions.trial_logger import TrialLogger

_SCENARIO_MODULES = {
    'SC01': sc01_static_grasp_hold,
    'SC02': sc02_full_cycle,
    'SC03': sc03_repeated_cycle,
    'SC04': sc04_variable_orientation,
    'SC05': sc05_payload_sweep,
}

DEFAULT_OUTPUT_DIR = 'sim_data'


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one Scenarios.md grasp-scenario trial (SC-01..SC-05)."
    )
    parser.add_argument('--scenario', required=True, choices=sorted(_SCENARIO_MODULES),
                         help='Which scenario to run.')
    parser.add_argument('--trial', type=int, required=True,
                         help='Trial number, used in the bag/metadata naming convention.')
    parser.add_argument('--object', default=None,
                         help='Object id from objects.yaml (default: objects.yaml active_object). '
                              'Ignored by SC05, which always sweeps object_sequence.')
    parser.add_argument('--condition-label', default=None,
                         help='Overrides the scenario config file\'s condition_label '
                              '(e.g. normal, fault_gps_stuck).')
    parser.add_argument('--orientation-index', type=int, default=0,
                         help='SC04 only: index into sc04.yaml\'s orientation_overrides.')
    parser.add_argument('--output-dir', default=DEFAULT_OUTPUT_DIR,
                         help=f'Where bags + sidecar metadata are written (default: {DEFAULT_OUTPUT_DIR}).')
    parser.add_argument('--use-gz-transport-fallback', action='store_true',
                         help='Command Armbot joints via `gz topic pub` instead of the '
                              'ros_gz_bridge topics in config/bridge.yaml.')
    parser.add_argument('--dry-run', action='store_true',
                         help='Exercise the full CLI/config/state-machine/logging flow '
                              'without touching ROS/Gazebo/PX4 — see README.md\'s '
                              'verification ladder.')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    objects_config = config_loader.load_objects_config()
    scenario_config = config_loader.load_scenario_config(args.scenario)
    joint_topic_map = config_loader.load_joint_topic_map()

    if args.condition_label:
        scenario_config.condition_label = args.condition_label

    if args.scenario == 'SC05' and args.object is not None:
        print('--object is ignored for SC05 (it sweeps object_sequence from sc05.yaml)',
              file=sys.stderr)

    object_id_for_logging = (
        '+'.join(scenario_config.object_sequence) if args.scenario == 'SC05'
        else (args.object or objects_config.active_object)
    )

    trial_logger = TrialLogger(
        output_dir=args.output_dir,
        condition_label=scenario_config.condition_label,
        scenario_id=scenario_config.scenario_id,
        trial_number=args.trial,
        object_id=object_id_for_logging,
        dry_run=args.dry_run,
    )

    module = _SCENARIO_MODULES[args.scenario]
    kwargs = dict(
        config=scenario_config,
        objects_config=objects_config,
        joint_topic_map=joint_topic_map,
        pose_topic=config_loader.load_bridged_pose_topic(),
        trial_logger=trial_logger,
        dry_run=args.dry_run,
        use_gz_transport_fallback=args.use_gz_transport_fallback,
    )
    if args.scenario == 'SC04':
        kwargs['orientation_index'] = args.orientation_index
    if args.scenario != 'SC05':
        kwargs['object_id'] = args.object

    outcome = module.main(**kwargs)

    print(f'Trial complete: scenario={args.scenario} trial={args.trial} '
          f'outcome={outcome} bag={trial_logger.bag_path}')
    return 0 if outcome in ('SUCCESS', 'UNKNOWN') else 1


if __name__ == '__main__':
    sys.exit(main())
