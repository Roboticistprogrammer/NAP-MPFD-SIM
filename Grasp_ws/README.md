# Grasp_ws

Colcon workspace containing `px4_gripper_missions`: scripted Python
mission programs automating `../Scenarios.md`'s five grasp scenarios
(SC-01..SC-05) against the combined `x500_gripper` + `Armbot` Gazebo
world (`~/PX4-Autopilot/Tools/simulation/gz/worlds/x500_gripper_armbot.sdf`,
see `../wayfinder/MAP.md`'s "Implemented" section for how that world was
built).

This workspace is fully self-contained — it does not overlay any other
workspace on the machine, and `px4_msgs` is pinned as a git submodule
rather than reused from elsewhere.

## Two-scenario grasp experiment (current)

`run_experiment` (`px4_gripper_missions/experiment.py`) runs the A centred /
B off-centre / E empty-control protocol against the table + cylinder in the
world, labels every trial from ground truth and records per-trial data;
`export_trials` merges it with the PX4 ULog. Drive it with
`utility/experiment.sh` - full description in `../07_grasp_experiment.md`.
Config: `config/experiment.yaml`. The sections below describe the older
SC-01..SC-05 scripts (`run_scenario`), which still target the removed
single-jaw `gripper_joint`.

## Prerequisites

- ROS 2 Humble, sourced (`source /opt/ros/humble/setup.bash`).
- Gazebo Harmonic (gz-sim) + PX4-Autopilot, per `../01_environment_setup.md`.
- `ros_gz_bridge` / `ros_gz_sim`: **not installed by
  `PX4-Autopilot/Tools/setup/ubuntu.sh`** — install separately:
  ```bash
  sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim
  ```
- `PyYAML` (`pip install pyyaml` if not already present via ROS).

## Setup

```bash
cd Grasp_ws
git submodule update --init --recursive   # fetches px4_msgs
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### px4_msgs version-match verification

`px4_msgs` is pinned (not floating) to a commit chosen to match a
specific `PX4-Autopilot` checkout's message schema — see `src/px4_msgs`'s
pinned commit and this workspace's git history for which PX4-Autopilot
commit it was matched against. If you've since updated `PX4-Autopilot`,
confirm they still agree before trusting a run:

```bash
# 1. Field-list diff for the messages this package actually uses
diff <(grep -v '^#' ~/PX4-Autopilot/msg/versioned/VehicleCommand.msg) \
     <(grep -v '^#' Grasp_ws/src/px4_msgs/msg/VehicleCommand.msg)
# repeat for OffboardControlMode.msg, TrajectorySetpoint.msg,
# VehicleLocalPosition.msg, VehicleStatus.msg

# 2. Runtime smoke test, once PX4 SITL + the XRCE-DDS agent are running
ros2 topic echo /fmu/out/vehicle_status --once
# fields should be non-zero/sane, not silently garbage
```

If they've drifted, re-pin: `cd src/px4_msgs && git fetch && git checkout <new-commit>`,
picking a `px4_msgs` `main`-branch commit whose own commit message
references a `PX4-Autopilot` commit that's an ancestor of your checkout's
`HEAD` (`git merge-base --is-ancestor <that-commit> HEAD` from inside
`PX4-Autopilot`) — same method used to pin the current commit.

## Running

1. **Unit tests** (no Gazebo/PX4 needed):
   ```bash
   colcon test --packages-select px4_gripper_missions
   colcon test-result --verbose
   ```
2. **Dry run** (exercises the full CLI -> config -> state-machine ->
   bag-naming -> sidecar-JSON flow with no real ROS/Gazebo/PX4 topics
   touched):
   ```bash
   ros2 run px4_gripper_missions run_scenario --scenario SC01 --trial 1 --dry-run
   ```
3. **Bring up the bridge** against an already-running combined world
   (per `../02_run_sitl_baseline.md`, extended with
   `PX4_GZ_WORLD=x500_gripper_armbot PX4_GZ_MODEL_NAME=x500_gripper`):
   ```bash
   ros2 launch px4_gripper_missions bridge.launch.py
   ros2 topic list   # confirm both /fmu/* and armbot/* + world/*/pose_info show up
   ```
4. **One real trial**:
   ```bash
   ros2 run px4_gripper_missions run_scenario --scenario SC01 --trial 1
   ```
   Inspect the produced bag + `sim_data/normal_SC01_trial01/metadata_trial.json`.

Other scenarios: `--scenario SC02`..`SC05`. SC-04 additionally takes
`--orientation-index N` (see `config/scenarios/sc04.yaml`). SC-05 always
sweeps `object_sequence` from `config/scenarios/sc05.yaml`, ignoring
`--object`. `--use-gz-transport-fallback` commands Armbot's joints via
`gz topic pub` directly instead of the ros_gz_bridge topics, if you'd
rather not run the bridge.

## Known placeholders / limitations (read before trusting a run)

- **The grasp-target object is entirely a placeholder** pending
  `../wayfinder/tickets/003-grasp-object-and-choreography.md` — shape,
  mass, friction, and `grasp_pose` joint angles in `config/objects.yaml`
  are unverified guesses, never tried against a live sim. Tune that one
  file once ticket 003 resolves; no script changes needed.
- **No object SDF model exists on disk** — `arm_controller.spawn_object`
  generates a box's SDF inline from `objects.yaml` at spawn time (via
  the same `/world/<world>/create` service PX4's own launch script uses)
  rather than requiring a model directory under
  `PX4-Autopilot/Tools/simulation/gz/models/`.
- **Grasp-outcome detection is a crude Z-drop heuristic**
  (`grasp_outcome.py`), not a real contact/attachment signal — no such
  signal exists anywhere in this project yet. It also hasn't been
  confirmed that ros_gz_bridge's `Pose_V -> TFMessage` conversion
  actually names the `child_frame_id` the way `GraspOutcomeMonitor`
  expects (exact match or `<model>::<name>` suffix) — first live run
  should check the log line it prints on first successful match.
- **No joint-state feedback loop** — `ArmController.set_pose`'s
  `settle_s` is a fixed dwell, not a real "reached target" check, even
  though `config/bridge.yaml` already bridges `armbot/joint_states`.
- **SC-03's `bag_per_cycle: true` config flag is not wired up** — every
  run currently uses one bag for the whole trial regardless of that
  setting; per-cycle outcomes are still tagged in the one sidecar JSON's
  `extra.cycles` list.
- **SC-04's orientation overrides only really exercise yaw** — the
  roll/pitch/yaw -> quaternion conversion (`arm_controller._rpy_to_quaternion`)
  is implemented and correct, but `config/scenarios/sc04.yaml`'s sample
  entries only vary yaw; roll/pitch paths are unexercised.
- **PX4 offboard control here is new code**, structurally adapted from
  the standard PX4 pattern (10-cycle `OffboardControlMode` warmup, then
  `engage_offboard_mode()` + `arm()` together — see `px4_offboard.py`'s
  docstring) rather than reused from anywhere in this project or the
  `PX4-Autopilot` checkout. Verify `wait_until_armed_and_offboard`
  actually succeeds before trusting anything downstream of it.
