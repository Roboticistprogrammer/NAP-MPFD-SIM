---
id: 003
title: Grasp-target object + SC-01 choreography
type: grilling
status: open
assignee: null
blocked_by: []
---

## Question

Now that [Arm mount point](002-arm-mount-point.md) is resolved (Armbot
hangs underslung, gripper pointing down at rest), two tightly-coupled
decisions remain before SC-01 (static grasp-and-hold) can actually run:

1. **The grasp-target object** — what gets grasped. Needs a concrete
   SDF model: shape/size (small enough for `jaw_link`'s ~0.05×0.04×0.06 m
   jaw opening per the current placeholder collision box), mass, and
   friction coefficients tuned for reliable contact against dartsim
   (confirmed physics backend, see
   [Cross-model rigid joint mechanism](001-cross-model-joint-mechanism.md)).
   Where does it sit in the world relative to the hovering drone (on
   the ground, on a stand/table at hover-reachable height)?
2. **Choreography/trigger mechanism** — how the approach → close → hold
   → release sequence actually gets driven during the demo: manual
   `gz topic pub` commands typed live, a scripted sequence (Python/ROS 2
   node) that times joint position commands against
   `/model/armbot/joint/*/cmd_pos`, or something else. Given the flight
   state decision (airborne/hovering, from the original grilling
   session — see [Map](../MAP.md)), does the drone need to be
   station-keeping via PX4 offboard control concurrently while the arm
   script runs, and if so is that offboard-hold already solved from the
   `02_run_sitl_baseline.md` baseline work, or does it need fresh setup?

## Answer

_(filled in on resolution)_
