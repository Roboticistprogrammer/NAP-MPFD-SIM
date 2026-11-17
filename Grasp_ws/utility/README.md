# utility/

Bash automation for the `x500_gripper` + Armbot sim
(`~/PX4-Autopilot/Tools/simulation/gz/worlds/x500_gripper_armbot.sdf`).
No ROS bridge needed: PX4 is driven through its `px4-*` shell clients and
the arm through Gazebo transport topics.

## Quick start

```bash
./demo.sh                 # launch everything, take off, run gripper demo, land
./demo.sh --no-land       # same, but stay hovering so you can play with it
./stop_all.sh             # tear it all down
```

## Step by step

| Script | What it does |
|---|---|
| `launch_sim.sh [--headless] [--fg] [--build]` | XRCE-DDS agent + PX4 SITL + Gazebo world + GUI, waits for EKF |
| `takeoff.sh [alt_m]` | arm (retries until preflight passes) + takeoff, default 2.5 m |
| `gripper.sh <cmd>` | `open`, `close [N]` (finger force), `home` `reach` `grab` `carry` `sweep` `demo` `state`, `pose <yaw> <shoulder> <elbow>`, `joint <name> <rad>` |
| `experiment.sh <cmd>` | grasp experiment: `single A\|B\|E`, `tune A\|B --grip ...`, `protocol`, `export <session>`, `dataset <session>` - see `../../07_grasp_experiment.md` |
| `land.sh` | land in place, wait for disarm |
| `status.sh` | processes, armed/altitude/EKF, DDS link, RAM, GPU |
| `px4.sh <module> [args]` | any pxh command against the daemon, e.g. `./px4.sh commander status` |
| `start_gui.sh` | (re)start only the Gazebo GUI, e.g. if it froze |
| `fetch_assets.sh` | pre-download the world's Gazebo Fuel models (run automatically) |
| `stop_all.sh [--keep-agent]` | stop PX4, Gazebo, bridge, agent |

Logs go to `utility/logs/` (`px4.log`, `gz_gui.log`, `agent.log`).
Settings (paths, world/model names, port) live in `common.sh` and can be
overridden from the environment, e.g. `USE_NVIDIA=0 ./launch_sim.sh`.

## Notes

- **Mouse not moving the Gazebo camera**: PX4's own launcher sends a camera
  FOLLOW lock and starts the GUI on the integrated GPU. `launch_sim.sh`
  sets `PX4_GZ_NO_FOLLOW=1` and starts the GUI itself with NVIDIA PRIME
  offload. Camera: left-drag pan, right-drag/scroll zoom, shift+left-drag orbit.
  With < ~2 GB free RAM the GUI still lags, so close other apps or use `--headless`.
- **Arm directions**: Armbot is mounted upside-down (roll = pi). Negative
  `shoulder_pitch`/`elbow_pitch` = gripper moves *down*. Positive values swing
  it into the props, so `gripper.sh` clamps them to +0.2 rad unless `FORCE=1`.
- **Scenery**: the world includes two Fuel assets, Jersey Barrier (perimeter
  at +/-7 m) and Construction Cone (3.5 m takeoff zone). The pre-edit world is
  saved as `x500_gripper_armbot.sdf.orig` next to it.
- **Gripper**: two force-controlled fingers (replaced the original single
  hinged jaw, which could not hold anything; old model kept as
  `Armbot/model.sdf.orig`). `gripper.sh close 1.5` squeezes with 1.5 N.
- **Experiment parameters**: `experiment.sh` temporarily switches PX4 to
  emulated motion capture (EKF2 fuses ground truth, GPS/baro off) and
  restores the parameters when it lands. If a session crashes,
  `launch_sim.sh` detects the leftover mocap setup and restores the defaults.
- ROS 2 side: local position is `/fmu/out/vehicle_local_position_v1` on this
  PX4 version (versioned topic name).
