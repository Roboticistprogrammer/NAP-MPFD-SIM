# 02 — Run the Baseline SITL Simulation

Three terminals needed. Source ROS 2 in every terminal that runs a
`ros2` command (`source /opt/ros/<distro>/setup.bash`).

## Terminal 1 — micro-XRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

Leave running. This bridges PX4's uORB topics to ROS 2 DDS.

## Terminal 2 — PX4 SITL + Gazebo, X500 airframe

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This builds (first run only) and launches both the PX4 flight stack and
the Gazebo GUI with the X500 quadrotor spawned in the default world.

Useful run-time flags:

```bash
HEADLESS=1 make px4_sitl gz_x500          # no Gazebo GUI, faster, lower resource use
PX4_SIM_SPEED_FACTOR=2 make px4_sitl gz_x500   # run at 2x real-time speed
```

If you get `ninja: error: unknown target 'gz_x500'`, run `make
distclean` once and retry — this is a known stale-build-cache issue.

## Terminal 3 — verify the ROS 2 bridge

```bash
ros2 topic list
```

You should see PX4 topics under `/fmu/out/...`, e.g.:

```
/fmu/out/vehicle_local_position
/fmu/out/vehicle_attitude
/fmu/out/sensor_combined
/fmu/out/battery_status
/fmu/out/estimator_status
```

Spot-check one is actually publishing data:

```bash
ros2 topic echo /fmu/out/vehicle_local_position
```

## Optional — connect QGroundControl

If QGroundControl is installed, it should auto-detect and connect to
the SITL instance over UDP, showing the vehicle on the map and telemetry
in the HUD. Useful for a visual sanity check that the vehicle is
healthy before running scripted tests.

## Checklist before moving on

- [ ] Gazebo launches with the X500 visible (or runs headless without errors)
- [ ] `ros2 topic list` shows `/fmu/out/*` topics
- [ ] At least one topic streams data via `ros2 topic echo`
- [ ] (Optional) QGroundControl connects and shows vehicle state
