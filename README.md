# NAP-MPFD-SIM

PX4 SITL + Gazebo Harmonic simulation of an **x500 quadrotor carrying a
gripper arm (Armbot)**, built to generate labelled grasp / grasp-failure
data. The flight-side signals (thrust and torque setpoints, motor outputs,
IMU) come from PX4. Gazebo ground truth labels when and how a grasp failed.

The current experiment is the two-scenario grasp protocol: the drone lifts
a cylinder from a table with a **centred** grasp (A) or an **off-centre**
one (B), plus empty-gripper controls (E), over many repeats. See
[07_grasp_experiment.md](07_grasp_experiment.md).

## Layout

| Path | Content |
|---|---|
| `00_…06_*.md`, `Scenarios.md` | setup guides and earlier scenario notes |
| `07_grasp_experiment.md` | the A/B/E grasp experiment: how to run it, data format, design notes |
| `wayfinder/` | design decision tickets (arm mount, cross-model joint, grasp object) |
| `px4/` | PX4-side files: Armbot (two-finger force-controlled gripper), x500_gripper, world with table/cylinder/jig, airframe 4022. Install with `px4/install.sh` |
| `Grasp_ws/` | ROS 2 (Humble) colcon workspace: `px4_gripper_missions` (experiment runner, ULog exporter, mocap emulation), `px4_msgs` (submodule), cylinder model, `utility/` bash automation |

## Setup

Prerequisites: Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, a PX4-Autopilot
checkout (main), Micro-XRCE-DDS Agent. See `01_environment_setup.md`.

```bash
git clone --recurse-submodules https://github.com/Roboticistprogrammer/NAP-MPFD-SIM.git
cd NAP-MPFD-SIM
px4/install.sh ~/PX4-Autopilot          # or --link to symlink the files
make -C ~/PX4-Autopilot px4_sitl

cd Grasp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

`px4_msgs` must match your PX4 checkout's message definitions. The
submodule is pinned; see `Grasp_ws/README.md` for how to check and re-pin.

## Quick start

```bash
cd Grasp_ws/utility
./launch_sim.sh                 # agent + PX4 + Gazebo (+ GUI)
./demo.sh --no-land             # take off and exercise the gripper
./stop_all.sh

./launch_sim.sh --headless
./experiment.sh single A        # one grasp trial
./experiment.sh protocol        # full A/B/E protocol
./experiment.sh export ../sim_data/<session>
```

All scripts are listed in `Grasp_ws/utility/README.md`.
