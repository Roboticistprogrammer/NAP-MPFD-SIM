---
id: 001
title: Cross-model rigid joint mechanism
type: research
status: closed
assignee: research-subagent
blocked_by: []
---

## Question

`Armbot` and `x500_gripper` are two independently-spawned Gazebo models
(per this map's "two models + world-level fixed joint" decision — see
[Map](../MAP.md)). How, concretely, do you rigidly join two
separately-spawned gz-sim (Gazebo Harmonic) models so the child model
moves with the parent for the full simulation?

Specifically resolve:

1. Does plain SDF `<joint>` support parent/child links that live in two
   different top-level models spawned into the same world, or is SDF
   `<joint>` scoped to links within a single `<model>`?
2. If not, is `gz-sim-detachable-joint-system`
   (`gz::sim::systems::DetachableJoint`) the correct mechanism for a
   joint that's never meant to detach (i.e., used purely as a permanent
   rigid attach), or is there a more appropriate plugin/approach for
   "permanently fixed, never detaches"?
3. Does `Armbot`'s current `<static>true</static>` prevent it from being
   carried along by any such joint (i.e., do static models get excluded
   from the dynamics tree regardless of joints), meaning it must be
   flipped to `<static>false</static>` before this works?
4. What does the world file (or a spawn/bridge script) need to contain
   to spawn both models and establish the joint at simulation start —
   concretely, an SDF/plugin snippet good enough to try.

## Answer

Researched against gz-sim's actual source (the `gz-sim8` branch, which is the
branch corresponding to Gazebo Harmonic), SDFormat's spec files (`sdf14`
branch / spec 1.11), and gz-physics (`gz-physics6`, also the Harmonic
branch). Confirmed via source/spec text where noted; flagged as inferred
otherwise. Nothing here was actually run in Gazebo — smoke-test before
relying on it.

### 1. Is SDF `<joint>` strictly model-scoped?

Partially confirmed, partially corrected the ticket's premise:

- A `<joint>` **nested inside a `<model>`** is scoped to that model (plus
  its nested sub-models via `child_model::link` `::`-scoped names, which is
  for models nested *inside* that model, not sibling top-level models). So
  yes: a joint written inside `x500_gripper`'s `<model>` block cannot reach
  into `Armbot`'s links, and vice versa. The original premise is correct
  for this case.
- **However**, SDFormat has a *separate* mechanism the ticket didn't
  anticipate: a `<joint>` can also be a **direct child of `<world>`**
  (`//world/joint`), sitting as a sibling to the `<model>`/`<include>`
  elements it connects. This is exactly designed to rigidly join two
  independently-spawned top-level models. It was removed in SDFormat 1.7,
  reintroduced in SDFormat 1.10 (sdformat PR
  [#1117](https://github.com/gazebosim/sdformat/pull/1117)), and wired
  into gz-sim's entity-creation/physics pipeline in gz-sim PR
  [#1949](https://github.com/gazebosim/gz-sim/pull/1949) (targeting Garden,
  gz-sim7 — carried forward into Harmonic/gz-sim8) plus dartsim-side
  support in gz-physics PR
  [#501](https://github.com/gazebosim/gz-physics/pull/501), which
  explicitly targets `gz-physics6` — confirmed as the Harmonic branch.
  Verified this is live on the `gz-sim8` branch: `src/LevelManager.cc` and
  `src/systems/physics/Physics.cc` (`WorldModelFeatureList` /
  `GetWorldModel()`) both have world-joint-specific code, and
  `test/worlds/joints_in_world.sdf` on that branch is a working example
  (fetched verbatim, see snippet below for the pattern it uses).
- Parent/child references use plain model names (resolves to that model's
  canonical link) or `model_name::link_name` for a specific link — this
  syntax is a sibling-scoped reference, i.e. exactly what's needed to name
  a link inside another top-level model from world scope.
- Caveat: this is a newer, less-traveled feature than model-scoped joints
  (added ~2022/2023) and its own PR notes systems like
  `JointPositionController` don't work if instantiated as a world joint
  (irrelevant here — Armbot's own arm joints stay inside Armbot's model;
  only the one attach joint is at world scope).

### 2. Is DetachableJoint the right mechanism, and what are its real params?

DetachableJoint is a legitimate, well-worn choice for this pattern (used
in-tree for vehicle-carries-cargo / breadcrumb scenarios), but there is now
also the simpler native option from Q1. Both work for "attach once, never
detach" — recommend trying the native world-joint first since it has zero
plugin/topic surface and *cannot* detach by construction, with
DetachableJoint as the documented fallback if the native path proves
flaky.

Params confirmed by reading the actual doxygen header on the `gz-sim8`
branch (`src/systems/detachable_joint/DetachableJoint.hh`):

- `<parent_link>` (required) — link in the model the plugin is attached
  to (the plugin lives inside the **parent** model's SDF).
- `<child_model>` (required) — name of the *other*, independently-spawned
  model.
- `<child_link>` (required) — link in that child model.
- `<topic>` (optional, legacy) / `<detach_topic>` (optional, preferred) —
  publish an `Empty` message here to detach. **Never publish to it and
  the joint is permanent** — this is how you get "attach once, never
  detach" out of a system designed around detaching.
- `<attach_topic>`, `<output_topic>` (optional) — only needed if you want
  re-attach or state-broadcast; irrelevant for a permanent joint.
- `<suppress_child_warning>` (optional, default `false`) — silences the
  "child model doesn't exist yet" warning; matters only if child spawns
  after the parent (both spawn together here, so not a concern, but cheap
  to set).
- No `initial_attach` parameter exists on the Harmonic branch — that's a
  later addition (PR #3268, found via search) whose target version I did
  not verify; **do not use `<initial_attach>` on Harmonic**, it will
  likely be silently ignored or rejected. Default behavior (attached
  immediately at spawn) is what's wanted anyway.
- Constraint confirmed from the tutorial: resulting kinematic topology
  must be a tree (no loops) — a non-issue with one joint; and the parent
  model needs `<self_collide>true</self_collide>` only if the two models'
  collision geometry is expected to touch — not directly relevant to the
  attach joint itself but worth remembering once the arm/gripper is doing
  contact physics near the airframe.

### 3. Does `Armbot`'s `<static>true</static>` block this?

**Yes — confirmed, not inferred.** SDFormat's own spec text (`model.sdf`,
`static` element, fetched from the `sdf14`/1.11 spec) says, verbatim:

> "If set to true, the model is immovable; i.e., a dynamics engine will
> not update its position. This will also overwrite this model's
> `@canonical_link` and instead attach the model's implicit frame to the
> world's implicit frame. This holds even if this model is nested (or
> included) by another model instead of being a direct child of
> `//world`."

A static model's pose is pinned directly by the engine, outside the
constraint/joint solver entirely — it isn't merely "hard to move," it's
excluded from the dynamics tree full stop, and the spec explicitly says
this holds regardless of nesting. That reasoning applies uniformly to
*any* joint mechanism (native world-joint or DetachableJoint) since it's a
physics-engine-level property of the model, not something either joint
system special-cases around. **`Armbot` must have `<static>` set to
`false` before either mechanism can work.**

Two ways to do that without necessarily hand-editing `Armbot/model.sdf`
directly: either flip `<static>true</static>` → `<static>false</static>`
in the model file itself, or override it per-spawn — SDFormat's
`<include>` element supports a `<static>` child that overrides the
included model's own static value (confirmed via the SDFormat `include`
spec). The snippet below uses the `<include>` override so the change is
visible at the world-file level; either is valid, the model.sdf edit is
probably clearer long-term. (Not applied — out of scope for this research
ticket per the instructions.)

### 4. Concrete world-file snippet

Both `x500_gripper` (internal model name still `x500`, merges in
`model://x500_base` which has `<link name="base_link">`) and `Armbot`
(internal model name `armbot`, top link also named `base_link`) were
checked directly — both already have a link literally named `base_link`,
convenient for the snippet below. Spawn names are set explicitly via
`<include><name>` so the joint's `model_name::link_name` references are
predictable regardless of each model's internal `<model name=...>` value.

**Option A — native world-level fixed joint (recommended first try, no plugin):**

```xml
<!-- inside <world name="default"> in default.sdf (or a custom world) -->

<include>
  <uri>model://x500_gripper</uri>
  <name>x500_gripper</name>
  <pose>0 0 0.5 0 0 0</pose>
</include>

<include>
  <uri>model://Armbot</uri>
  <name>armbot</name>
  <!-- overrides Armbot/model.sdf's <static>true</static> for this spawn -->
  <static>false</static>
  <!-- mount pose relative to world; retune once ticket 002 (mount point)
       is resolved -->
  <pose>0.05 0 0.08 0 0 0</pose>
</include>

<!-- world-level joint: direct sibling of the two <include> blocks above,
     NOT nested inside either model -->
<joint name="arm_attach_joint" type="fixed">
  <parent>x500_gripper::base_link</parent>
  <child>armbot::base_link</child>
</joint>
```

**Option B — DetachableJoint plugin (fallback if Option A misbehaves), plugin goes inside x500_gripper's spawned instance:**

```xml
<include>
  <uri>model://x500_gripper</uri>
  <name>x500_gripper</name>
  <pose>0 0 0.5 0 0 0</pose>
  <plugin filename="gz-sim-detachable-joint-system"
          name="gz::sim::systems::DetachableJoint">
    <parent_link>base_link</parent_link>
    <child_model>armbot</child_model>
    <child_link>base_link</child_link>
    <suppress_child_warning>true</suppress_child_warning>
    <!-- intentionally no <detach_topic>: never publish to it and this
         is a permanent joint for the life of the sim -->
  </plugin>
</include>

<include>
  <uri>model://Armbot</uri>
  <name>armbot</name>
  <static>false</static>
  <pose>0.05 0 0.08 0 0 0</pose>
</include>
```

Note `default.sdf` currently declares `<physics type="ode">` — this `type`
attribute is a legacy/cosmetic holdover from Gazebo-classic SDF; gz-sim
does not ship an "ode" gz-physics plugin and loads dartsim by default
absent an explicit physics-plugin override, which is what was confirmed
above to have `WorldModelFeature` support in Harmonic. No physics-plugin
override was found in `default.sdf`, so Option A should be exercised
against whatever the actual default physics plugin resolves to at
runtime — worth double-checking with `gz sim -v 4` output when this is
first tried, not just assumed from the `type="ode"` string.

### Confidence summary

- Q1 (world-level joint exists and reaches across sibling models): grounded in source (PRs, branch code, working test SDF) — solid.
- Q2 (DetachableJoint params): grounded directly in the Harmonic-branch header source — solid. The "never publish to detach_topic = permanent" framing is my inference from the documented behavior, not a literal doc statement, but follows directly from how the system is described to work.
- Q3 (static blocks dynamics): grounded in verbatim SDFormat spec text — solid.
- Q4 (snippet): sketch only, as scoped by the ticket — not run, link/model names verified against the real `model.sdf` files but pose values are placeholders pending ticket 002.
