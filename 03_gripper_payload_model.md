# 03 — Approximating the Gripper (No Arm Kinematics Needed)

Goal: capture the two effects that actually perturb the flight
controller/EKF — (1) the static mass/CG shift from carrying the gripper
+ payload, and (2) the dynamic disturbance of a grasp event — without
modeling the ArmBot's linkage.

## 1. Work on a copy of the model, not the original

Never edit the shipped `x500` model directly — keep it as your clean
baseline.

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz/models
cp -r x500 x500_gripper
```

Register the copy as its own airframe so `gz_x500` (unmodified) and
`gz_x500_gripper` (custom) both remain runnable:

```bash
cd ~/PX4-Autopilot
cp ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500 \
   ROMFS/px4fmu_common/init.d-posix/airframes/4011_gz_x500_gripper
```

Edit the new airframe file's `PX4_SIM_MODEL` line to point at
`x500_gripper` (adjust to the value your PX4 version uses — check the
existing `4001_gz_x500` file for the exact variable/line to change).
Rebuild after registering a new airframe file:

```bash
make px4_sitl
```

## 2. Static mass/CG offset

Edit `x500_gripper/model.sdf`. Find the `<link name="base_link">`
block's `<inertial>` element and adjust `<mass>` and `<pose>` (the pose
offset moves the CG toward the gripper mount location, typically
downward/forward on the airframe):

```xml
<inertial>
  <mass>2.1</mass> <!-- original X500 mass + estimated gripper+payload mass -->
  <pose>0.03 0 -0.08 0 0 0</pose> <!-- shifted toward gripper mount; tune to your geometry -->
  <inertia>
    <ixx>0.02</ixx>
    <iyy>0.02</iyy>
    <izz>0.04</izz>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyz>0</iyz>
  </inertia>
</inertial>
```
My calculations:

<model name="armbot_gripper">  #look for correct link and joints name from Armbot urdf file
  <!-- Gripper Link -->
  <link name="gripper_link">
    <pose>0 0 0.3 0 0 0</pose> <!-- Adjust based on your 30cm arm reach -->
    <inertial>
      <mass>0.03</mass>
      <inertia>
        <ixx>0.0000130</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.0000152</iyy>
        <iyz>0</iyz>
        <izz>0.0000102</izz>
      </inertia>
    </inertial>
    
    <collision name="collision">
      <geometry>
        <box><size>0.05 0.04 0.06</size></box>
      </geometry>
    </collision>
    
    <visual name="visual">
      <geometry>
        <!-- Replace with your actual mesh file path if you have one -->
        <box><size>0.05 0.04 0.06</size></box>
      </geometry>
    </visual>
  </link>

  <!-- Gripper Joint -->
  <joint name="gripper_joint" type="revolute">
    <parent>arm_wrist_link</parent>
    <child>gripper_link</child>
    <axis>
      <xyz>0 0 1</xyz> <!-- Change to 1 0 0 or 0 1 0 depending on servo orientation -->
      <limit>
        <lower>0.0</lower>
        <upper>3.1416</upper>
        <effort>0.157</effort>
        <velocity>1.5</velocity> <!-- Typical speed for 9g servo is ~0.1s/60deg -->
      </limit>
    </axis>
  </joint>
</model>


## 3. Dynamic disturbance: simulating a grasp event

Rather than modeling the gripper's motion, apply a timed force impulse
at the mount link using Gazebo's built-in `ApplyLinkWrench` system —
this reproduces the disturbance signature nta real grasp event would
produce (a sudden force/torque transie) without any arm geometry.

Add the plugin to the world file
(`~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf`, or your
custom world) at the `<world>` level:

```xml
<plugin filename="gz-sim-apply-link-wrench-system"
        name="gz::sim::systems::ApplyLinkWrench">
</plugin>
```

Trigger a one-off wrench (simulating a grasp/payload-pickup jolt) from
a terminal once the sim is running:

```bash
gz topic -t "/world/<world_name>/wrench" \
  -m gz.msgs.EntityWrench \
  -p "entity: {name: 'x500_gripper::base_link', type: LINK}, wrench: {force: {x: 0, y: 0, z: -8}, torque: {x: 0.5, y: 0, z: 0}}"
```

Replace `<world_name>` with your actual world name (check with `gz
topic -l | grep wrench` while the sim is running to find the exact
topic path). Tune force/torque magnitude and duration to represent a
plausible grasp-event transient — start conservative and compare
against the vibration/disturbance magnitude you've measured on the real
ArmBot for a sanity check.

For scripted/repeatable trials, publish this from a small Python node
(`rclpy` + `ros_gz_interfaces/msg/EntityWrench` via the ROS-Gazebo
bridge) instead of the manual `gz topic` command, so trial timing is
reproducible and can be tagged with a `trial_id`/`scenario_id` the same
way your ArmBot logger does.

## Checklist before moving on

- [ ] `x500_gripper` model exists as a separate copy from `x500`
- [ ] New airframe boots via `make px4_sitl gz_x500_gripper` (or your chosen target name)
- [ ] Mass/CG change is visible in flight behavior (e.g. slightly different hover throttle vs. baseline `gz_x500`)
- [ ] A manually triggered wrench pulse produces a visible attitude/IMU disturbance in `ros2 topic echo /fmu/out/sensor_combined`
