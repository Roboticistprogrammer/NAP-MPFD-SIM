---
id: 002
title: Arm mount point
type: grilling
status: closed
assignee: user
blocked_by: []
---

## Question

Where on `x500_gripper` does `Armbot` mount, and does that geometry
actually clear the propellers?

x500's rotor arms sit at roughly (±0.13–0.22, ±0.13–0.22, 0.03) m from
the airframe center (see `x500_base/model.sdf`). `Armbot`'s reach is
~0.30 m from its own base along +X once fully extended (base_link at
origin through to `jaw_link` at local x=0.3025).

Resolve:

1. Mount location on the airframe (top/bottom/front/underslung) and
   orientation (which way does `Armbot`'s +X point relative to the
   drone's forward axis).
2. Whether that placement, combined with the arm's joint limits
   (`base_yaw_joint`: ±3.14, `shoulder_pitch_joint`/`elbow_pitch_joint`:
   ±1.57), keeps the arm's swept volume clear of the rotor disks and
   propeller guards through the full range of motion needed for SC-01.
3. Does the real physical ArmBot + drone rig (if one exists, per
   `06_next_steps.md`'s mention of physical fault injection being
   primary) already have a chosen mount configuration this should just
   match, rather than deciding fresh in sim?
4. Concretely: what link/pose gets added to `x500_gripper/model.sdf` to
   serve as the attachment frame (the parent-side anchor for the joint
   from [Cross-model rigid joint mechanism](001-cross-model-joint-mechanism.md)).

## Answer

Resolved by direct conversation (no real physical rig reference was
available to shortcut this — point 3 stays unconfirmed, treat this as
a sim-first decision to be checked against real hardware later if a
mount configuration is chosen there).

**1. Location & orientation.** Underslung, centered on the belly:
`Armbot` spawns at `x=0 y=0 z=-0.04` relative to `x500_gripper`'s
`base_link` origin. x=0/y=0 keeps it clear of both the rotor arms
(±0.174, ±0.174) and the landing legs (y=±0.098/±0.132, which are also
much lower at z=-0.123/-0.2195) — centered underneath, nothing to
collide with at the attach point itself.

**2. Rest-pose rotation.** `Armbot` was modeled as a benchtop arm:
at all-joints-zero it reaches horizontally along its own local +X, with
`base_yaw_joint` spun about local Z (azimuthal, like a lazy susan) and
`shoulder_pitch_joint`/`elbow_pitch_joint` bending about local Y. User
wants the whole gripper actually movable (confirmed: `<static>` must go
to `false`, both for this attach joint to hold *and* for Armbot's own
`JointPositionController`-driven joints to actuate at all — a static
model doesn't respond to joint commands regardless) and wants the rest
pose already hanging down rather than relying on joint limits to get
there.

Chosen fix: rotate the whole `Armbot` spawn by **pitch = +90°
(1.5708 rad) about the mount's Y axis**. This maps local +X (reach
direction) to world/parent -Z, so joints-at-zero already hangs the
gripper straight down, matching the "movable, rest = hanging" request
instead of parking the arm at a joint limit.

**Known side effect, not a blocker:** this same rotation carries
`base_yaw_joint`'s axis from local-vertical (azimuthal sweep as
designed) to the mount's local X — a different sweep plane than "yaw"
conventionally implies once re-mounted this way.
`shoulder_pitch_joint`/`elbow_pitch_joint` (local Y) are unaffected and
remain the vertical-plane bend that actually brings the gripper down to
grasp height. For SC-01 (one fixed grasp point) this is fine — the
target is reachable via *some* combination of the three joints — but if
a later scenario needs a true "sweep the approach direction around the
vertical hang axis" motion, this reorientation is worth revisiting.

**3. Real hardware reference:** none available — noted as an open
assumption, not a blocker for the sim demo.

**4. Attachment mechanism:** no new bracket/adapter link. The
world-level joint (or `DetachableJoint`, per
[Cross-model rigid joint mechanism](001-cross-model-joint-mechanism.md))
connects `x500_gripper::base_link` directly to `armbot::base_link`,
using the pose above (`0 0 -0.04 0 1.5708 0`, roll-pitch-yaw) as
`Armbot`'s spawn `<include><pose>`.

The "My calculations" inertia block in `03_gripper_payload_model.md`
(single 0.03 kg `gripper_link`) is confirmed **superseded** — it
predates the real 5-link `Armbot` model, which already carries its own
per-link inertials. Left as historical scratch notes in that doc, not
folded into anything.
