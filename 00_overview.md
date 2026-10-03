# PX4 + Gazebo Simulation — Overview

## Scope of this simulation stage

This replaces Isaac Lab as the simulation environment for this project.
Its role in the overall data strategy is deliberately limited to two things:

1. **Software pre-validation** — running the ROS 2 / EKF fusion stack
   against a simulated vehicle before it ever touches real hardware.
2. **Unsafe fault-class generation** — actuator/sensor failure modes too
   risky to induce on the real UAV in flight (e.g. in-flight motor
   dropout). Sim is the only safe source for these classes.

**Not in scope:** using Gazebo as the primary training-data source, or
building a kinematically accurate model of the ArmBot gripper. The
physical fault-injection hierarchy already decided for the ArmBot
(controlled physical faults → this simulation for the unsafe classes →
semi-supervised anomaly detection fallback) is unchanged.

## Platform

- Airframe: Holybro X500 — already a first-class PX4 SITL model
  (`gz_x500`), shipped in the PX4-Autopilot repo. No need to source or
  build a custom drone model.
- Gripper: represented as a point-mass + CG offset on the X500 airframe,
  plus scripted force disturbances at the mount point to approximate a
  grasp event — not a modeled multi-link arm. See `03_gripper_payload_model.md`.

## Assumed target system

- Ubuntu 22.04 (ROS 2 Humble)
- PX4-Autopilot `main` branch (or latest stable release tag) v1.18
- Gazebo Harmonic (the version PX4's `Tools/setup/ubuntu.sh` installs by default on current PX4 releases)
- Ros Humble

## File order

1. `01_environment_setup.md` — install PX4, Gazebo, ROS 2, micro-XRCE-DDS agent
2. `02_run_sitl_baseline.md` — first working simulation + ROS 2 bridge
3. `03_gripper_payload_model.md` — add gripper mass/disturbance to the X500 model
4. `04_fault_injection.md` — inject sensor/actuator faults in SITL
5. `05_ros2_data_logging.md` — record and export simulation data
6. `06_next_steps.md` — how this feeds the fusion training pipeline
