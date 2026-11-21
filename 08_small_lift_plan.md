# 08 — Small-lift grasp-outcome prediction: implementation plan

**Status: PLAN ONLY. No implementation code is written until Amir approves this plan.**
Branch: `feature/small-lift-protocol` (`main` keeps the tuned A/B/E experiment untouched).
Source brief: `small-lift-plan-brief.md`. Reference study: Zimmer, Hellebrekers, Asfour, Majidi, Kroemer (2019).

## 1. Decisions already taken

| # | Decision | Consequence |
|---|---|---|
| Q1 | Extend the protocol and labels; keep the existing sim, gripper and cylinder | No change to the world, Armbot or `x500_gripper` models |
| Q2 | Keep real contact grasping | The brief's `DetachableJoint` + slip-joint model (Option C) is dropped; no new modelling assumption to validate |
| Q3 | Two scenarios, many repetitions, as Zimmer did | A (centred) and B (off-centre) are the classes; E stays as an empty-gripper control |
| Q4 | Keep current randomisation; add nothing until further notice | Only grip force (±8 %) and grasp offset (±3 mm). No mass, μ, wind or fault jitter |
| Q5 | Follow the workspace versions | ROS 2 Humble, Ubuntu 22.04, Gazebo Harmonic, PX4 `main`, pinned `px4_msgs` (not Jazzy / v1.16) |
| Q6 | Work and plan inside this repo (`NAP-MPFD-SIM`) | Plan lives here; hardware repo `NAP-MPFD` is not touched |
| — | All development on a separate branch | `feature/small-lift-protocol`; merge to `main` only on Amir's request |

## 2. Objective

From a short standardised lift, predict (a) whether the grasp survives the full lift and (b) whether it survives a lateral stability manoeuvre, using **drone-side signals only** (thrust, torque, motors, IMU, attitude, vertical state) plus Tier-1 gripper information. The two scenarios are repeated many times at the tuned grip forces (A 1.40 N, B 2.40 N, about 50 % failures each).

## 3. What exists today (verified in the repo)

- `experiment.py` runs one trial as: baseline 1 s → descend → settle 1 s → close 1 s → lift 0.40 m at 0.3 m/s → hold 4 s → release. A trial takes about 20 s.
- Outcome (`classify_outcome`) is computed from cylinder displacement and tilt relative to the grasp point over the lift and hold window: slip at 10 mm, drop at 40 mm, tilt failure at 45°. This is the existing success label.
- Per trial it writes a 50 Hz Gazebo ground-truth CSV with a `phase` column; the session ULog (`flight.ulg`) holds the full-rate PX4 signals.
- `export_trials.py` slices the ULog per trial onto a 100 Hz sample-and-hold grid and merges it with the ground truth and labels. Signals exported: thrust and torque setpoints, motors 0–3, attitude quaternion, body rates, accelerometer, `lpos_z`, `lpos_vz`.
- Only a 10-trial protocol session exists locally (`sim_data/20261003_102712_protocol`). The full 300-trial run and any dataset do not exist yet.
- There is no small-lift phase, no stability test, no windowing, no `ml/` code.

## 4. Protocol changes

### 4.1 Trial state machine

| Phase | Today | Planned |
|---|---|---|
| baseline, descend, settle, close | as is | unchanged (re-tuned forces stay valid) |
| **small lift (SL)** | — | new: grasp point rises 0.10 m in 1.5 s, minimum-jerk profile. Start of SL = **t₀**, the window anchor |
| full lift | one 0.40 m ramp at 0.3 m/s | continues to +1.0 m in total (D1) at 0.3 m/s, about 3 s after the SL |
| hold | 4 s | unchanged; `y_success` is labelled at the end of hold |
| **shake** | — | new, only if `y_success = 1`: two lateral ±0.3 m steps; `y_stable` labelled over the shake window |
| release | puts a held cylinder back down | unchanged |

Pre-SL baseline for per-channel mean subtraction: the last 0.5 s of the close phase. The cylinder still rests on the table then, so load transfer happens inside the SL window, which is the analogue of Zimmer's scale under the tray.

### 4.2 Labels

- `y_success`: the existing `classify_outcome` result (success / fail) over SL + full lift + hold. Definition unchanged so earlier tuning remains meaningful.
- `y_stable`: `classify_outcome` re-run over the shake window with displacement and tilt re-referenced to the start of the shake. Missing (not 0) when `y_success = 0`.
- Kept as metadata: failure mode (slide / tilt), `t_slip`, `t_drop`, applied grip force, actual grasp offset.
- Invalid trials (missed grasp, alignment timeout) stay excluded, as now.

### 4.3 Recording additions

- Explicit phase timestamps in `trials.csv`: `t_close`, `t_lift` (= t₀), `t_sl_end`, `t_shake`.
- `session.json`: git commit and config hash, so every dataset is reproducible from the code that made it.
- `channels.yaml`: the single definition of model-input and privileged channels. The hardware recorder should later replicate it.

### 4.4 Ground-truth force sensor

Dropped (D6). Labels come from relative displacement, so no force/torque pedestal is built.

## 5. Data pipeline

Stay on the ULog path (decision D4). It already works, is time-aligned with Gazebo (SITL lockstep), and records motor outputs at full rate.

**Windows** (new `windows.py`, 50 Hz grid anchored at t₀, bin-averaged rather than sample-and-hold to avoid aliasing high-rate motor signals):

- SL window: t₀ to t₀ + 1.5 s → 75 samples.
- CG window: the close phase → about 50 samples.
- Output: one array per trial plus a table of labels and metadata, keyed by `trial_id`, stored as Parquet or `.npz`.

**Model-input channels (C = 21, all available on the real drone):**

| Group | Channels |
|---|---|
| Thrust and torque setpoints | 3 + 3 |
| Motor commands | 4 |
| Body rates | 3 |
| Accelerometer | 3 |
| Attitude | roll, pitch (2) |
| Vertical state | z, v_z, a_z (3) |

`hover_thrust_estimate` was checked on the pilot session and is constant (0.8000, sd 0.0000) in this sim, so it is not an input. Tier-1 gripper input is defined in decision D3.

**Privileged channels (labelling and analysis only):** cylinder displacement and tilt, cylinder pose, applied grip force, actual grasp offset, finger travel, scenario identity.

**Leakage control (important).** The applied grip force is what decides success in this sim. It must never reach the model as an input, and the scenario label (A/B) must not either. Report accuracy per scenario against that scenario's own majority-class baseline, so a model cannot score by recognising A versus B.

## 6. Learning plan (`ml/`, plain Python package outside the colcon workspace)

- Preprocessing: subtract the pre-SL mean per channel; z-score with training-fold statistics only.
- Baselines as in Zimmer: RBF-SVM on flattened windows, FFNN (200/40/10), LSTM (20 units).
- Thesis model: 1-D CNN → LSTM, multi-task heads; loss = BCE(success) + λ · 𝟙[success] · BCE(stable).
- Evaluation: stratified 5-fold CV (mean ± sd), stratified by scenario × label. Metrics: accuracy against the majority baseline, balanced accuracy, ROC-AUC, Brier, false-positive rate at the operating point.
- Held-out set: a short extra session at grip forces shifted outside the training range (reuses the existing `--grip` override; needs no new randomisation).

| Experiment | Scope under Q4 |
|---|---|
| E1 window comparison: CG / SL / CG+SL, SL length 0.5–2.0 s | In scope |
| E2 model comparison and learning curves | In scope |
| E3 sensor ablation: IMU / motors + thrust / attitude / Tier-1 / full fusion / + Tier-2 | In scope; the sim-only Tier-2 arm uses finger travel |
| E4 fault-severity coupling | Deferred: needs fault randomisation, which is frozen |
| E5 generalisation and closed loop (predictor at t₀ + 1.5 s aborts a bad grasp) | Held-out set in scope; closed loop optional at the end |

## 7. Layout on the branch

```
08_small_lift_plan.md                         this plan
Grasp_ws/src/px4_gripper_missions/
  config/experiment.yaml                      + small_lift:, shake:, protocol.variant: legacy|small_lift
  px4_gripper_missions/
    trajectory.py                             new: minimum-jerk and shake setpoint generators (unit-tested)
    experiment.py                             modified: SL and shake phases behind protocol.variant
    export_trials.py                          modified: extra signals, phase times
    windows.py                                new: SL / CG window extraction
  test/                                       new tests for trajectory, labels, windows
ml/                                           new: preprocessing, splits, baselines, CNN-LSTM, evaluation
07_grasp_experiment.md                        updated at the end to describe both variants
```

`protocol.variant` defaults to `legacy` until M5 passes, so the A/B/E experiment keeps working unchanged on the branch.

## 8. Milestones

| ID | Milestone | Acceptance criteria | Effort |
|---|---|---|---|
| M0 | Freeze baseline | `main` untouched; branch created; legacy `single A/B` runs on the branch exactly as on `main` | 0.25 d |
| **M1** | **Detectability pilot (riskiest assumption, run first)** | Using existing and a fresh 40-trial legacy session: the thrust and torque change during load transfer (success trials) versus E controls is at least 3× the hover-noise standard deviation, per scenario. If not, stop and review (see D10) | 0.5 d |
| M2 | SL trajectory | Minimum-jerk unit test passes; in sim the SL completes in 1.5 s ± 0.05 s; over 20 trials per scenario the failure rate stays in 30–60 % | 1 d |
| M3 | Shake and labels | One trial writes `y_success` and `y_stable` end to end; pilot of 40 trials reports the class balance of `y_stable`; thresholds fixed | 1.5 d |
| M4 | Recording | Phase timestamps, config hash and `channels.yaml` present; dry-run tests pass | 0.5 d |
| M5 | Batch generation | N trials (D5) with at least 95 % valid; failure rate per scenario 30–60 %; `y_stable` positives ≥ 25 % of successes; ULog archived | 1 d wall-clock plus monitoring |
| M6 | Dataset pipeline | `(trials, 75, 21)` arrays reproducible from a session; leakage test shows scenario and applied force are absent from inputs | 2 d |
| M7 | Baselines and E1 | SVM, FFNN, LSTM on CG / SL / CG+SL with CV tables | 2 d |
| M8 | CNN-LSTM and E2, E3 | Multi-task model, learning curves, ablation table | 3 d |
| M9 | Held-out set and E5 | Shifted-force session evaluated; closed loop only if approved | 3 d |

Total about 15 working days. Compute is modest: the largest models see a few hundred 75 × 21 windows, so CPU training is enough.

## 9. Dependencies

Native install, no Docker (as today). Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, PX4-Autopilot `main` with the pinned `px4_msgs`, Micro-XRCE-DDS Agent. Python: numpy, pyyaml, pyulog (export), pandas, pyarrow, scikit-learn, PyTorch (CPU). The host disk is about 92 % full (noted in `07`), so data and ULog sizes must be watched; `sim_data/` is git-ignored.

## 10. Risks

1. **Detectability.** One 0.15 kg cylinder; the load-transfer signature may be weak against motor-command noise. M1 checks this before anything else is built.
2. **Leakage.** Grip force and scenario identity determine the label in this sim (section 5).
3. **Sample size.** A few hundred trials per class is small for a CNN-LSTM; the plan relies on the baselines being competitive and reports learning curves.
4. **`y_stable` balance.** Marginal grasps may fail every shake, or the shake may be too gentle; M3 sets the shake before the batch run.
5. **Shake disturbs the mocap-emulated hover.** A ±0.3 m step could excite the position loop; the step profile is tuned in M3.
6. **Sim-to-real gaps.** No rotor downwash, stiff arm joints, force-controlled gripper versus the real open-loop servo gripper (`07`, Limitations).

## 11. Decisions

Answered by Amir on 2026-10-16:

| # | Decision | Answer | Consequence |
|---|---|---|---|
| D1 | Full-lift height | **+1.0 m** (the brief's value) | The old 0.40 m tuning no longer covers the whole window. The extra 0.6 m may shift the failure rate, so M2 re-checks 30–60 % and re-tunes grip forces with `experiment.sh tune` if needed. A trial grows to about 35 s |
| D2 | Shake | Along the cylinder axis (world Y): centre → +0.3 m → −0.3 m → centre, about 1 s per leg | Confirmed in M3 |
| D3 | Tier-1 gripper input | Binary `grip_state` plus commanded arm pose | Applied force and finger travel are excluded; finger travel is the sim-only Tier-2 ablation |
| D4 | Recording format | ULog only | No rosbag2 |
| D5 | Number of trials | **10 first**, to check the dataset is usable | See below |
| D6 | Force/torque pedestal | Not needed | Removed from scope (section 4.4) |
| D8 | Branching | Old protocol stays the default; new one is developed on `feature/small-lift-protocol` | `main` stays as is |
| D10 | If the signal is too weak | Change the cylinder mass | Grip forces would then need re-tuning |

Consequence of D5: ten trials (about 4 A, 5 B, 1 E) cannot train or statistically validate a model. The first session is a usability check only: all 21 channels present and aligned, SL windows exactly 75 samples from t₀, labels written, and a visible payload step in the thrust and motor signals. The 3× signal-to-noise test in M1 and the class-balance targets in M5 move to the scale-up run, whose size is decided after this check.

Still open:

| # | Decision | Recommendation |
|---|---|---|
| D7 | Held-out set at shifted grip forces | Decide after the 10-trial check; it only matters once there is enough data to train a model |
| D9 | Role of E (empty-gripper) trials | Use them only as the no-payload baseline for the detectability check, not as classifier data |

## 12. Implementation status (2026-10-16, branch `feature/small-lift-protocol`)

Built and tested (87 unit tests pass, including a fake-clock test of the whole lift, hold and shake sequence):

| Item | Where |
|---|---|
| Minimum-jerk and shake profiles | `trajectory.py` |
| `small_lift` variant: small lift → lift to +1.0 m → hold → shake (only if held), labels `y_success` / `y_stable`, phase times, git commit and config hash in `session.json`, `--variant`, `--empty` | `experiment.py`, `config/experiment.yaml` (`protocol.variant` stays `legacy`) |
| Relabel for small_lift sessions | `relabel_session` |
| 50 Hz SL / CG windows, 21 input channels, labels in Parquet | `windows.py`, `config/channels.yaml`, `experiment.sh dataset <session>` |

Not built yet: `ml/` (preprocessing, baselines, CNN-LSTM). Deliberately waiting for a usable dataset. E-based residual templates and the held-out set (D7) are out of scope by instruction.

### Pilot session `sim_data/20261016_164146_protocol` (10 trials: 5 A, 5 B, no E)

- The pipeline works end to end: 10 trials, `(10, 75, 21)` SL windows, no NaNs, labels written. About 26–30 s per trial; ULog 105 MB.
- **Result: 9 of 10 failed** (A 4/5, B 5/5); the only success was then unstable in the shake. The forces tuned for the 0.40 m lift (A 1.40 N, B 2.40 N) no longer give 30–60 % failures. Most drops happen 2–5 s after t₀, i.e. after the SL window.
- **The commanded SL is not realised by the drone.** PX4 receives the setpoint correctly (−97 mm at t₀ + 1.5 s), but the vehicle first has to carry the cylinder: thrust rises 3–4 % (about the cylinder's weight) with no motion for ~1.3 s, and the vehicle has risen only 6 mm by t₀ + 1.5 s and 22 mm by t₀ + 2.0 s. The position loop has a slow response (`MPC_Z_P` = 1.0) and receives no velocity feed-forward. The full 1.0 m rise is reached only about 7 s after t₀.
- The SL window therefore contains the load transfer (thrust ramp-up) but little actual climb. That is close to Zimmer's "weight on the scale", but it is not "0.10 m in 1.5 s".

### New decisions needed

| # | Decision | Recommendation |
|---|---|---|
| D11 | Make the drone actually follow the SL profile (send velocity feed-forward with the position setpoint), or keep the current command-level profile? | Keep it for now: the window is identical every trial and contains the load transfer. Revisit if the signal turns out too weak |
| D12 | Re-tune grip forces for the small_lift variant (`experiment.sh tune A/B --variant small_lift`, about 20 min per scenario for 4 levels × 10 trials) and store them separately from the legacy forces | Yes: add `small_lift.grip_force_n` overrides so `legacy` stays untouched |

### SL profile comparison (2026-10-16, scenario A at 1.40 N, 5–6 trials per profile)

Question: can the SL be made flyable by shortening it to 0.06 m or lengthening it? Raw sessions are in `Grasp_ws/sim_data/20261016_22*` (git-ignored).

| Profile | Drone's rise at end of SL (% of commanded) | Lag to half the rise | Grasps held | Drops inside the SL window |
|---|---|---|---|---|
| 0.10 m / 1.5 s | 13 % | 1.7 s | 1 of 5 | 1 |
| 0.06 m / 1.5 s | 0 % | 2.1 s | 1 of 5 | 0 |
| 0.10 m / 3.0 s | 37 % | 2.0 s | 0 of 6 | 1 |
| 0.10 m / 4.0 s | 49 % | 2.2 s | 1 of 6 | 2 |
| 0.06 m / 3.0 s | 14 % | 2.5 s | 0 of 6 | 1 |
| **0.10 m / 1.5 s + velocity feed-forward** | **61 %** | **0.5 s** | **3 of 5** | 2 |
| 0.06 m / 1.5 s + velocity feed-forward | 36 % | 1.5 s | 0 of 6 | 0 |

- The payload's thrust signature (about +2.3 to 2.7 % of thrust at the end of the window, hover noise 0.03–0.07 %) is the same in every profile, so detectability is not the deciding factor.
- Shortening the distance does not help: the lag is mostly load transfer plus the slow position loop, not distance.
- Lengthening helps tracking only a little (49 % at 4 s) and eats prediction lead time: failures happen 3–5 s after t₀, so a 3–4 s window already contains them.
- Velocity feed-forward (`small_lift.velocity_feedforward`, default off) is the only change that makes the drone follow the profile, and it brought the held rate to a usable 3 of 5. All three held grasps then failed the shake, so the shake needs gentler parameters (M3).
- Caveat: 5–6 trials each, one grip force, scenario A only; treat as indicative.

Update: feed-forward is now the default (`small_lift.velocity_feedforward: true`), the shake legs are 2.0 s (peak acceleration of the long leg 0.9 instead of 3.5 m/s²), and a 0.15 m / 2.5 s run with them tracked 90 % of the command. All its failures (7 of 7) fell 1.4–1.8 s after t₀, inside the window. See `result.md` for the full table, findings and next steps.

Update 2026-10-17: small-lift defaults are now 0.15 m / 1.5 s, feed-forward on, put-down off, shake legs 2.0 s, grip forces A 1.46 N / B 2.75 N (`small_lift.grip_force_n`; legacy forces unchanged). Confirmation of 30 trials gave 33 % failures per scenario and 3 of 30 invalid. Open issues: `y_stable` is confounded with the scenario, and most failures occur at about 0.98 s, leaving little lead time. Details in `result.md` sections 6 and 7.
