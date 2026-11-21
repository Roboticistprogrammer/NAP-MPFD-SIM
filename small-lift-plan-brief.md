# Planning Brief — Small-Lift Grasp-Outcome Prediction in Gazebo (NAP-MPFD)

**Request to the receiving session:** Read this brief and the repository, then produce a **detailed implementation plan only**. Do **not** write or commit implementation code until Amir explicitly approves the plan.

- Repository: https://github.com/Roboticistprogrammer/NAP-MPFD (branch `main`)
- Reference study: Zimmer, Hellebrekers, Asfour, Majidi, Kroemer (2019), *Predicting Grasp Success with a Soft Robotic Hand using a Data-Driven Classifier* — 1.5 s / 10 mm small-lift (SL) phase predicts full-lift success (~91–92%) better than the gripper-closing (CG) phase.

---

## 1. Research objective

Adapt the Zimmer et al. small-lift protocol to a UAV-mounted gripper: from a short, standardised 1.5 s lift, predict (a) whether the full lift will succeed and (b) whether the grasp survives a stability manoeuvre — using primarily **UAV telemetry** fused with **Tier-1 (commanded-angle) gripper data**. Prototype entirely in Gazebo / PX4 SITL first.

Standing project decisions:
- Two scenarios with many repetitions (not many object types).
- Maximise and evaluate the contribution of UAV data.
- Gazebo + PX4 SITL; ROS 2 Jazzy; PX4 v1.16 / `px4_msgs` release/1.16 (already vendored under `src/px4_msgs`).
- Hardware target: Holybro X500 V2 + CODLAI/JSumo ArmBot (5-DOF, Arduino Nano, open-loop servos), Pixhawk 6C, Raspberry Pi 5.

## 2. Current repository state (relevant)

- `gripper/` — Arduino logger + Python host logger; Tier-1 CSV schema at ~50 Hz (20 ms): `host_timestamp, device_t_us, trial_id, object_id, scenario_id, condition_label, base_deg, shoulder_deg, elbow_deg, gripper_deg, unused_deg, grip_state, notes`. Gripper is closed at ~96° and open at both extremes.
- `ml/` — scaffold only (no pipeline, no model).
- `src/` — real-hardware ROS 2 workspace (OpenVINS, `px4_msgs`, VIO↔PX4 bridge). No simulation packages in this repo yet.
- An ArmBot Gazebo model (per-link STLs, `model.config`, `model.sdf` with revolute joints and topic-based joint position control) exists separately and must be integrated as a UAV payload.
- No UAV telemetry is captured as a dataset yet — the simulation recorder should define the channel schema the hardware recorder will later replicate.

## 3. Proposed protocol (to be planned)

| Zimmer et al. | UAV–gripper adaptation |
|---|---|
| Arm lifts 10 mm in 1.5 s | UAV climbs Δz_SL ≈ 0.10 m in 1.5 s, minimum-jerk profile |
| USB scale under tray | Pedestal with `force_torque` sensor (ground truth of load transfer) |
| Finger IMU/pressure, 30 Hz | PX4 telemetry + commanded gripper angles, resampled to 50 Hz (75 samples/window) |
| Success = object in hand after lift | `y_success` = object retained after full +1.0 m lift |
| Stability = survives wrist shake | `y_stable` = survives two lateral ±0.3 m steps (only if `y_success = 1`) |

**Trial state machine (ROS 2 offboard node via `px4_msgs`):**
P0 settle hover ≥ 3 s → P1 descend to grasp height → P2 close gripper (CG window logged; attach only if geometric validity check passes, else label *miss* and exclude) → P3 small lift 1.5 s (SL window; t₀ anchor) → P4 full lift → label `y_success` → P5 lateral shake → label `y_stable` → P6 reset/respawn.

**Grasp-retention model (recommended Option C):** compliant, friction-limited attachment — object linked through a prismatic "slip" joint with Coulomb friction F_avail = 2μN plus damping; `DetachableJoint` released when accumulated slip exceeds a threshold. Optional revolute slip DOF for rotation-in-grasp. Must be validated (hanging-mass test) and disclosed as a modelling assumption; cross-check against a small pure-contact subset.

**Per-trial randomisation (seeded, logged):** object mass within scenario class, μ, N, grasp offset vs. CoM, gripper closure deficit (`condition_label = fault_*`), wind/gust. Tune ranges for ~40–60% success. SL excitation profile stays fixed.

**Recording:** rosbag2 (MCAP) + JSON sidecar (seed, hidden params, phase timestamps, labels, `t_slip`) → `rosbags` → Parquet keyed by `trial_id`, 50 Hz grid anchored at t₀, sim time.
- Model-input channels (hardware-available only): `sensor_combined`, `vehicle_attitude`, `vehicle_local_position` (z, v_z, a_z), `actuator_motors`, `vehicle_thrust_setpoint`, `hover_thrust_estimate`, commanded arm/gripper angles, `grip_state`. Verify topic names (v1.16 may suffix some).
- Privileged channels (labelling/analysis only): slip displacement, F/T, μ, N, offsets; gripper joint effort only for a Tier-2 upper-bound ablation.

## 4. Learning formulation (to be planned)

- Input X ∈ ℝ^(75 × C), C ≈ 18–22; subtract pre-SL hover mean per channel; z-score with training-fold statistics.
- Baselines replicating Zimmer: RBF-SVM (flattened), FFNN 200/40/10, LSTM (20 units). Thesis model: 1-D CNN → LSTM with multi-task heads; loss = BCE(success) + λ·𝟙[success]·BCE(stable).
- Stratified 5-fold CV (mean ± sd) + held-out out-of-range parameter set. Metrics: accuracy vs. majority baseline, balanced accuracy, ROC-AUC, Brier, **false-positive rate at operating point**.
- Runtime use: onboard node evaluates p̂ at t₀ + 1.5 s; continue if p̂ ≥ τ, else abort/descend/re-grasp. τ chosen by cost-weighted FP/FN (C_FP ≫ C_FN). Closed-loop evaluation: in-flight drop rate with vs. without predictor.

**Experiment matrix:** E1 window comparison (CG / SL / CG+SL; SL 0.5–2.0 s) · E2 model comparison + learning curves · E3 sensor ablation (UAV IMU / motor+thrust / attitude / gripper Tier-1 / full fusion / +Tier-2 sim-only) · E4 fault-severity coupling · E5 generalisation + closed loop.

## 5. Known risks to address in the plan

- Detectability floor: light objects (paper cup, 5–10 g) may be below motor-command noise — plan a pilot measuring hover noise vs. expected Δthrust per object class before fixing the two scenarios.
- Tier-1 data does not observe grip force/friction margin; UAV dynamics observe mass, CoM offset, swing/slip transients.
- Sim-to-real: idealised motor/sensor models (match Pixhawk 6C noise), no ground effect by default.


## 6. What the plan should contain

1. Proposed directory/package layout within NAP-MPFD (e.g. `sim/` world + model templates, `sim_runner/` ROS 2 package, `ml/` pipeline) and how it coexists with the existing `src/` hardware workspace.
2. Ordered milestones with acceptance criteria (e.g. M1 composite model hovers stably; M2 retention law validated; M3 single trial end-to-end with labels; M4 batch generation + pilot; M5 dataset pipeline; M6 baselines; M7 CNN-LSTM + ablations; M8 closed-loop eval).
3. Dependencies/versions (Gazebo Harmonic, PX4 v1.16 SITL, ROS 2 Jazzy, Python libs), Docker or native.
4. Open decisions requiring Amir's input, listed explicitly.
5. Estimated effort per milestone and the riskiest assumption to test first.

**Output format:** formal, concise, structured plan suitable for review; end with the list of decisions needed for approval. Do not implement until approved.
