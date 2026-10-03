# Map: Airborne arm-grasp demo (x500_gripper + Armbot)

Status: OPEN
Label: wayfinder:map

## Destination

One working Gazebo demo: `x500_gripper` (PX4 SITL, airframe
`4011_gz_x500_gripper`) hovers, with the kinematic `Armbot` arm rigidly
attached via a world-level fixed joint, and the arm executes Scenario
SC-01 (static grasp-and-hold, from `Scenarios.md`) on a test object
using real Gazebo contact physics — proving the combined airborne-grasp
architecture works end-to-end, one full cycle, once.

This supersedes the "no arm kinematics needed" approximation from
`00_overview.md` / `03_gripper_payload_model.md` (point-mass/CG-shift +
scripted wrench). That approach is dropped for this effort in favor of
real kinematics and real contact physics, per the grilling session that
opened this map — see Decisions so far.

The remaining four `Scenarios.md` cases, fault-injection layering, and
the fused-CSV → ML training pipeline (`06_next_steps.md`) are explicitly
**not** this map's destination — see Out of scope.

## Notes

- Domain: PX4 SITL + Gazebo Harmonic (gz-sim) — quadrotor + 5-DOF arm
  physical/control integration.
- **Tracker**: no git repo, no issue tracker in this environment.
  Using wayfinder's local-markdown fallback — this file is the map,
  `tickets/*.md` are child tickets. "Frontier" = tickets with
  `status: open` and empty/all-closed `blocked_by`. "Claim" = set
  `assignee` in a ticket's frontmatter before working it.
- **Skill gaps**: `/grilling`, `/domain-modeling`, and `/research` are
  not installed here. HITL tickets get resolved via direct
  conversation (AskUserQuestion where a genuine decision point comes
  up) instead of a dedicated grilling skill. AFK/research tickets get
  resolved via a general-purpose or Explore subagent instead of a
  dedicated `/research` skill.
- Standing facts (don't re-investigate):
  - `4011_gz_x500_gripper` airframe file already exists and correctly
    sets `PX4_SIM_MODEL=x500_gripper`. Registration is done.
  - `x500_gripper/model.sdf` and `model.config` are currently an
    untouched copy of stock `x500` (internal model name still `x500`,
    metadata still describes the HoverGames kit). No gripper-specific
    edits have landed yet.
  - `Armbot/model.sdf` is a complete 5-joint kinematic arm
    (`base_yaw_joint` → `shoulder_pitch_joint` → `elbow_pitch_joint` →
    `wrist_joint` [fixed] → `gripper_joint`), real STL meshes, real
    inertials, `JointPositionController` plugins per joint. Currently
    `<static>true</static>` and not spawned into any world alongside
    `x500_gripper`.
  - x500 rotor arms sit at roughly (±0.13–0.22, ±0.13–0.22, 0.03) m;
    Armbot's reach is ~0.30 m from its own base along +X. Mount
    location/orientation has real propeller-clearance stakes — not
    just cosmetic.
  - Physics engine is dartsim, not ode — `default.sdf`'s
    `<physics type="ode">` is a cosmetic legacy attribute gz-sim
    ignores; confirm with `gz sim -v 4` at runtime rather than trusting
    the string. (Found while resolving
    [Cross-model rigid joint mechanism](tickets/001-cross-model-joint-mechanism.md).)

## Decisions so far

- Initial grilling (this session, un-ticketed — settled while naming
  the destination) — kinematic arm + real contact physics (not the
  point-mass/wrench approximation); grasp happens airborne/hovering;
  models stay separate, joined by a world-level fixed joint; destination
  is one working demo scenario, not all five.
- [Cross-model rigid joint mechanism](tickets/001-cross-model-joint-mechanism.md)
  — a `<joint>` written directly under `<world>` (sibling to the two
  `<include>`s) natively joins links across two independently-spawned
  models in gz-sim Harmonic — no plugin needed; `DetachableJoint` is a
  documented fallback if that misbehaves. Either way, `Armbot`'s
  `<static>true</static>` is a confirmed hard blocker and must become
  `false` (via a model.sdf edit or a per-spawn `<include><static>`
  override) before any joint can move it. Concrete snippets for both
  options are in the ticket.
- [Arm mount point](tickets/002-arm-mount-point.md) — `Armbot` spawns
  centered underslung on `x500_gripper`'s belly (`0 0 -0.04` from
  `base_link`, clear of rotor arms and legs), rotated pitch +90° about
  the mount's Y axis so joints-at-zero already hangs the gripper
  straight down. No new bracket link — the attach joint connects
  `x500_gripper::base_link` to `armbot::base_link` directly. The 03 doc's
  "My calculations" inertia block is confirmed superseded by Armbot's
  real per-link inertials — left as historical notes, not used.

## Implemented

Tickets 001 and 002's decisions are now landed in the actual PX4-Autopilot
tree (this map's Notes override "plan, don't do" for this one step, at
the user's explicit request — see conversation):

- `Tools/simulation/gz/models/Armbot/model.sdf` — `<static>` flipped
  `true` → `false`.
- `Tools/simulation/gz/worlds/x500_gripper_armbot.sdf` (new) — statically
  includes both `x500_gripper` (spawn pose `0 0 0.5 0 0 0`) and `Armbot`
  (spawn pose `0 0 0.70 0 1.5708 0`, i.e. 0.04 m below x500_gripper's
  `base_link` — which itself sits +0.24 m above the include pose due to
  `x500_base`'s own merge-include offset — rotated pitch +90°), plus the
  world-level `arm_attach_joint` (`type="fixed"`, native SDF joint per
  ticket 001, no plugin) connecting `x500_gripper::base_link` to
  `armbot::base_link`.
- PX4 normally spawns its vehicle dynamically at runtime
  (`px4-rc.gzsim`'s `/world/<world>/create` service call), which can't
  reach a second model or a world-authored joint. This world sidesteps
  that: both models + the joint are baked into the world file at load
  time, and PX4 is meant to **attach** to the already-running
  `x500_gripper` entity instead of spawning its own — launch with
  `PX4_GZ_WORLD=x500_gripper_armbot PX4_GZ_MODEL_NAME=x500_gripper make
  px4_sitl gz_x500_gripper` (uses `px4-rc.gzsim`'s existing "connect to
  existing model" branch, no PX4/ROMFS changes needed).

**Empirically smoke-tested** (headless `gz sim -s -r`, this session,
no PX4 attached): world loads with no structural errors; dartsim log
confirms the fixed joint successfully merged both models into one
composite skeleton (`base_link` name collision → auto-renamed
`base_link(1)` — this only happens when both links join one skeleton,
so it's positive evidence the cross-model joint actually works, not a
problem); spawn pose for `x500_gripper` matched `0 0 0.5` exactly before
gravity took over. A passive drop (no flight controller running) fell
and toppled within ~3s, as expected with no active stabilization and
only 0.5 m of starting clearance — **not** yet verified: PX4 actually
attaching to it, hovering, or the arm's own joints being commanded.
That needs a real interactive `make px4_sitl` run (in progress by the
user, separately from this map).

### Mission scripts (`Grasp_ws/`)

At the user's explicit direction — scope beyond this map's original
destination (see Out of scope's note on SC-02..SC-05) — a full Python
mission-scripts package now exists for all five `Scenarios.md` cases,
not just SC-01: `Grasp_ws/` is a self-contained colcon workspace
(`ament_python` package `px4_gripper_missions`), with `px4_msgs` pinned
as a git submodule rather than reused from any other workspace on the
machine — verified an *exact* schema match (zero commits touched
`msg/versioned/` between the pinned `px4_msgs` commit's sync point and
this `PX4-Autopilot` checkout's `HEAD`, not just "close in time").

Structure: `px4_offboard.py` (genuinely new PX4 ROS 2 offboard-control
code — none existed anywhere in this project or the PX4-Autopilot
checkout), `arm_controller.py` (Armbot joint commands via
`ros_gz_bridge`, plus `spawn_object`/`place_object`/`remove_object`
using the same `/world/<world>/create` service PX4's own launch script
uses), `grasp_outcome.py` (placeholder Z-drop heuristic —
SUCCESS/FAILURE/UNKNOWN), `trial_logger.py` (wraps `ros2 bag record`
with `05_ros2_data_logging.md`'s exact naming convention + a sidecar
JSON matching `06_next_steps.md`'s schema), and one `scenarios/scXX_*.py`
per scenario. Full detail, prerequisites, and known limitations are in
`Grasp_ws/README.md` — not duplicated here.

**Verified this session**: 43 unit tests pass (`colcon test`); all five
scenarios' `--dry-run` complete end-to-end with a clean exit, including
SC-05's full ascending-mass sweep. Three real bugs were caught and
fixed by actually running this rather than trusting the code read-through:
`px4_msgs`' rosidl setters reject non-strict-float list elements (a
Python `int` in a position list crashes at publish time); `trial_logger`'s
`extra or {}` silently created a *new* dict whenever the caller's
`extra_metadata` was still empty at `start()` time, breaking the
documented "mutate after start(), see it in the sidecar" contract; and
`--dry-run` wasn't actually wired into `PX4OffboardController` at all,
so it still blocked for real on vehicle-status data that would never
arrive outside a live sim.

**Connects to [Grasp-target object + SC-01 choreography](tickets/003-grasp-object-and-choreography.md)**,
still open: SC-01's implemented sequence (one-shot `arm.set_pose(grasp_pose)`,
no incremental approach) is a concrete candidate answer to that ticket's
choreography question, worth confirming once tried live. The object
itself is still fully placeholder — `Grasp_ws/src/px4_gripper_missions/config/objects.yaml`
is the single file that changes once the ticket resolves; no script
edits needed either way.

## Not yet specified

- Whether Armbot's STL mesh collision geometry is stable enough for
  reliable grasp contact in gz-sim/dartsim, or needs primitive-shape
  (box/cylinder) collision approximation on `jaw_link` /
  `gripper_base_link` — depends on first physics test results, can't
  be answered without running it once.
- Whether the arm's added mass/motion (mount point now known: `0 0 -0.04`
  from `base_link`, ~0.39 kg total per Armbot's per-link masses) needs
  any PX4 tuning (e.g. `MPC_THR_HOVER`, attitude gains) to hold a stable
  hover — can't be answered without actually flying the combined model
  once.

## Out of scope

- SC-02 through SC-05 **as a data-collection destination** (running
  real trials, fault-injection layering, fused-CSV export, ML
  training-pipeline integration) — still deferred to a follow-on effort
  once this base integration is proven working, per this map's original
  destination. Their mission-*script scaffolding* now exists regardless
  (see Implemented → Mission scripts), built ahead of schedule at the
  user's explicit direction — the scripts existing doesn't mean the
  scenarios have actually been run or the data pipeline is in scope
  yet, only that they're ready for when it is.
- Rewriting `00_overview.md` / `03_gripper_payload_model.md` to reflect
  the kinematic-arm decision — a documentation cleanup, not part of
  reaching this destination.
- `grasp_outcome` success/failure labeling logic for the ML pipeline —
  belongs to the later data-pipeline effort, not this integration demo.
