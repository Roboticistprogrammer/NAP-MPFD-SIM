# Results so far: Zimmer-style small lift on the A/B cylinder grasp experiment

Date: 2026-10-16 to 2026-10-17 · Branch: `feature/small-lift-protocol` · Plan: `08_small_lift_plan.md`

All numbers come from short pilot runs (5–10 trials per row, mostly scenario A at 1.40 N). They show direction, not statistics.

## 1. What exists now

| Item | State |
|---|---|
| `legacy` A/B/E experiment (0.40 m lift) | Unchanged, still the default `protocol.variant` |
| `small_lift` variant | Built and run in the sim: small lift (SL) → lift to +1.0 m → hold 4 s → lateral shake (only if the cylinder is still held) |
| Labels | `y_success` (survives lift and hold), `y_stable` (survives the shake; empty when `y_success = 0`) |
| Velocity feed-forward | Implemented, **now the default** for the small-lift variant (`small_lift.velocity_feedforward: true`) |
| Shake | Made smoother: each leg 2.0 s (was 1.0 s), amplitude ±0.3 m along the cylinder axis, sequence centre → +0.3 → −0.3 → centre |
| Dataset builder | `./experiment.sh dataset <session>` writes 50 Hz windows (SL and gripper-closing), 21 drone-side channels and a labels table |
| Tests | 88 unit tests pass (profiles, labels, windows, a fake-clock run of the whole sequence) |
| Not started | `ml/` (preprocessing, SVM / FFNN / LSTM / CNN-LSTM). Waiting for a dataset with usable class balance |

Defaults for the small-lift variant in `experiment.yaml` (as of 2026-10-17): SL 0.15 m in 1.5 s with feed-forward, shake legs 2.0 s, no put-down, grip forces A 1.46 N and B 2.75 N (the legacy 0.40 m forces, A 1.40 N and B 2.40 N, are kept for `legacy`).

## 2. Why the SL profile changed: the drone does not follow the command

The runner commands a smooth rise of 0.10 m in 1.5 s. PX4 receives it correctly, but the drone barely moves: it first has to take the cylinder's weight (thrust rises about 3 %, with no motion for about 1.3 s), and its position loop adds roughly a second of lag. Trial 8 of the first pilot:

| Time after t₀ | Commanded rise | Actual rise |
|---|---|---|
| 0.5 s | 19 mm | 0 mm |
| 1.0 s | 77 mm | 0 mm |
| 1.5 s | 100 mm | 6 mm |
| 2.0 s | 104 mm | 22 mm |

## 3. Profile comparison

Scenario A, 1.40 N, valid trials only. "Rise" is how much of the commanded height the drone had actually climbed at the end of the SL window. "Lag" is the delay until it had climbed half of it. "Held" means `y_success = 1`.

| Profile | Rise at end of SL | Lag to half rise | Grasps held | Drops inside the SL window |
|---|---|---|---|---|
| 0.10 m / 1.5 s | 13 % | 1.7 s | 1 of 5 (20 %) | 1 of 5 (20 %) |
| 0.06 m / 1.5 s | 0 % | 2.1 s | 1 of 5 (20 %) | 0 of 5 (0 %) |
| 0.10 m / 3.0 s | 37 % | 2.0 s | 0 of 6 (0 %) | 1 of 6 (17 %) |
| 0.10 m / 4.0 s | 49 % | 2.2 s | 1 of 6 (17 %) | 2 of 6 (33 %) |
| 0.06 m / 3.0 s | 14 % | 2.5 s | 0 of 6 (0 %) | 1 of 6 (17 %) |
| 0.10 m / 1.5 s + feed-forward | 61 % | 0.5 s | 3 of 5 (60 %) | 2 of 5 (40 %) |
| 0.06 m / 1.5 s + feed-forward | 36 % | 1.5 s | 0 of 6 (0 %) | 0 of 6 (0 %) |
| 0.15 m / 2.5 s + feed-forward + smoother shake (A and B, 10 trials) | 90 % | 0.5 s | 2 of 9 (22 %) | 7 of 9 (78 %) |
| **0.15 m / 1.5 s + feed-forward + smoother shake (A and B, 10 trials)** | **85 %** | **0.35 s** | 1 of 10 (10 %) | 7 of 10 (70 %) |

Invalid trials (alignment) are excluded. All rows used the grip forces tuned for the old 0.40 m lift (A 1.40 N, B 2.40 N), which is why few grasps were held in the last two rows; section 6 re-tunes them. The 0.15 m / 1.5 s profile was chosen: it tracks nearly as well as 0.15 m / 2.5 s without lengthening the window.

## 4. What this shows

1. **Shortening the distance does not help.** The lag is load transfer plus the slow position loop, not distance. A 6 cm move is swallowed (0 % tracked).
2. **Lengthening the time helps only a little** (49 % at 4 s) and the window then contains the failures.
3. **Feed-forward is what makes the drone follow the profile**: lag drops from about 2 s to 0.5 s, and 0.15 m / 2.5 s is tracked to 90 %. It is a standard PX4 offboard feature (position plus velocity setpoint), so it can be flown on the real drone.
4. **Payload signal is large.** Thrust rises by about 2.3–2.7 % when the cylinder is picked up, against 0.03–0.07 % hover noise, in every profile.
5. **With feed-forward, failures happen at lift-off, 1.3–1.8 s after t₀**, almost independent of the profile (all 7 failures in the last run, 1.4–1.8 s). A window longer than about 1.3 s therefore already contains the failure, and thrust visibly falls back once the cylinder is lost. A long window makes the task easy (detecting a drop), not predictive. Window length must be a studied variable, which the plan already has (SL 0.5–2.0 s in experiment E1).
6. **The old grip forces do not fit the longer lift** (10–22 % held). Section 6 re-tunes them.
7. **The smoother shake works**: stable grasps now occur, but see the A/B confound in section 7.

## 5. What the `trials/tNNNN_X.csv` columns are, and how they feed an LSTM or SVM

Each file is the simulator's own record of one trial at 50 Hz (Gazebo ground truth). **It is not the model input.** The model input is the drone's own telemetry (thrust, torque, motors, IMU, attitude, vertical state) taken from the PX4 log, as `dataset/windows.npz` (`X_sl`: trials × 75 × 21). These CSVs supply the timing, the labels and the privileged analysis variables.

| Columns | What they are | Use in training |
|---|---|---|
| `t_sim`, `phase` | sim time; which step the trial is in (`baseline`, `descend`, `settle`, `close`, `small_lift`, `lift`, `hold`, `shake`, `release`) | Cut the windows: t₀ is the first `small_lift` row, the gripper-closing window is the `close` rows. Not features |
| `rel_dx/dy/dz`, `rel_disp`, `tilt_change_deg` | cylinder displacement and tilt relative to the grasp point, counted from when the fingers closed | **Define the labels**: slip at 10 mm, drop at 40 mm, tilt failure at 45° (`y_success`; the same rules over the shake rows give `y_stable`). Could also be auxiliary targets, such as time to slip or peak displacement. Never inputs |
| `cyl_x/y/z`, `cyl_tilt_deg`, `grasp_x/y/z` | cylinder and gripper-point poses | Same role as above (labelling, plots, checking alignment). Privileged |
| `drone_x/y/z`, `drone_roll/pitch/yaw` | the drone's true pose | Ground truth for what the onboard estimate (`lpos_z`, `att_roll`, `att_pitch` in the PX4 log) measures. Use for tracking error and sanity checks, not as inputs |
| `cmd_x/y/z` | the position the drone was commanded to | Identical in every trial of a profile, so no information. Use to check tracking and the t₀ alignment |
| `finger_l`, `finger_r` | measured finger travel | Simulator-only (the real Armbot servos have no feedback). Use only as the "Tier-2" upper-bound ablation |
| `shoulder`, `elbow` | measured arm joint angles | Nearly constant. The Tier-1 input is the *commanded* arm pose, stored once per trial as a static feature |
| `grip_force_cmd` | the applied grip force (nominal times random jitter) | **Do not feed to a model.** In this simulator it largely decides the outcome, and the scenario (A or B) is a proxy for it. Use only to stratify folds and for analysis |

Practical use:

- **LSTM** (planned: 20 units as in Zimmer, then CNN→LSTM). Input is `X_sl` as a sequence of 75 steps × 21 channels. Subtract each channel's pre-SL hover mean (`base_sl`), then z-score with statistics from the training folds only. Labels `y_success` and, for held grasps, `y_stable` (mask the second loss where `y_stable` is empty). With a few hundred trials expect to need noise augmentation, class weights and a small network.
- **SVM** (RBF). Flatten the same window to 75 × 21 = 1575 numbers, standardise, optionally PCA. With few trials, hand-made features per channel often work better than raw samples: change from baseline at the end of the window, peak, time of peak, slope, area, standard deviation. About 150 numbers per trial.
- **Both**: stratify folds by scenario and label, report accuracy per scenario against that scenario's own majority class, and repeat with window ends at 0.5, 1.0 and 1.5 s so a model cannot score by simply seeing the cylinder already fall (point 5 above).

## 6. Grip-force re-tuning and yield fix (2026-10-17)

Profile: 0.15 m / 1.5 s, feed-forward, +1.0 m lift, shake legs 2.0 s. Valid trials only. "Applied" force includes the ±8 % per-trial jitter.

| Scenario | Sweep (nominal N → held) | Fit over 127 valid trials | Chosen nominal force |
|---|---|---|---|
| A | 1.40 → 40 % (n=5), 1.45 → 25 % (n=8), 1.55 → 86 % (n=7), 1.65–2.30 → 100 % | 50 % held at 1.46 N applied, very sharp (30 % at 1.44, 60 % at 1.48) | **1.46 N** |
| B | 2.40 → 17 %, 2.80 → 17 %, 2.90 → 50 % (n=8), 3.05 → 88 %, 3.20 → 77 %, 3.60 → 100 % | 50 % held at 2.75 N applied, shallower (30 % at 2.49, 60 % at 2.87) | **2.75 N** |

Confirmation at 30 trials (15 A, 15 B, no E): at 1.50 / 2.90 N, A held 7 of 14 and B held 12 of 13 (B too easy); after moving to 1.46 / 2.75 N, **A held 10 of 15 and B 8 of 12**, i.e. 33 % failures each, inside the 30–60 % target.

**Invalid trials.** In the first tuning runs 8 of 24 A trials were invalid ("grasp alignment did not converge"), always right after a trial that was held *and* stable. Cause: after the shake the runner set the cylinder "gently" back on the table, a 1 m descent that left the arm swinging and the cylinder dropped from ~15 cm. The cylinder is teleported to its jig before every trial anyway, so the small-lift variant now skips the put-down (`small_lift.put_down: false`). Six held-and-stable trials in a row then ran without an invalid one. In the final confirmation 3 of 30 trials were invalid (alignment errors 6–10 mm against a 4 mm tolerance), down from about a third.

## 7. Is the dataset usable? Check on session `20261017_004908_protocol`

27 valid trials (3 invalid dropped): `windows.npz` has `X_sl` of shape 27 × 75 × 21 with no gaps; labels: `y_success` 18 held / 9 failed; `y_stable` 8 stable / 10 unstable among the 18 held.

Open problems:

1. **`y_stable` is confounded with the scenario.** Among held grasps, A is stable 1 of 10 times and B 7 of 8. A model could predict `y_stable` by recognising A versus B. The shake or the forces need adjusting so each scenario has a mix, or `y_stable` has to be evaluated within a scenario.
2. **Failures happen at lift-off, about 0.9–1.1 s after t₀ (median 0.98 s) for most.** One simple feature (change in thrust, motors) separates held from failed well only once the window reaches the drop (AUC 0.88 at 1.0 s; in scenario A 1.00), but poorly before it (pooled AUC 0.44 at 0.4 s, 0.65 at 0.6 s, 0.70 at 0.8 s; scenario A 0.50–0.64 before 1.0 s). So with this feature there is little warning before the drop; whether a trained model does better is the question the next step answers. B gives some early signal (AUC 0.88–1.00 at 0.6–0.8 s), but with 12 trials this is noise-level evidence.
3. **Sample size.** 27 trials is enough to test the pipeline, not to train a CNN-LSTM. The plan suggests about 600.

## 8. Next steps

1. Fix the `y_stable` confound (adjust the shake or the forces so held grasps of each scenario are mixed). Done: section 9 (per-scenario shake intensity); the 150-trial batch is in section 10.
2. Study early prediction first: run SVM and small LSTM baselines with window ends at 0.4, 0.6, 0.8, 1.0 and 1.5 s on a larger session.
3. Generate that session (the plan suggests 600 trials, about 6 hours at 32 s per trial) and build `ml/`.
4. Reduce the remaining invalid trials (alignment tolerance or averaging window).

## 9. Fixing the `y_stable` confound: per-scenario shake intensity (2026-11-11)

**Cause.** With one shared shake (legs of 2.0 s), the stability of a held grasp depends on the grip force, and the two scenarios sit at very different points on that curve. From earlier sessions (valid, held trials): A was stable 2 of 17 at about 1.4 N and 7 of 26 at about 1.6 N, but 8 of 8 from 2.0 N; B was stable 12 of 14 at about 2.8 N. At the forces chosen for a 30–60 % lift failure rate, the same shake is too harsh for A and too gentle for B, so `y_stable` mostly says "A or B".

**Result.** `experiment.yaml` now carries `shake.per_scenario`: A 2.25 s, B 1.25 s. Pilot 3 aborted once on a PX4 estimator fault (phantom 12 m/s speed, "High Accelerometer Bias" failsafe) after the sim had run two full sessions; a restart fixed it, so the batch driver restarts the sim between sessions.

**Change.** A new `shake.per_scenario.<A|B>` block in the config overrides keys of the shared `shake` block (code: `scenario_shake()` in `experiment.py`, one unit test). The shake is carried out after the SL window the models see, so a per-scenario intensity does not enter the inputs; it is a difficulty knob like the per-scenario grip force. Only `leg_duration_s` is varied (a longer leg means lower peak acceleration); amplitude stays at ±0.3 m. Forces stay at A 1.46 N and B 2.75 N. **`experiment.yaml` has not been changed yet; pilots use copies of it.**

**Pilots** (30–40 trials each, A and B interleaved, no E, forces as above, valid trials only):

| Pilot | A legs | B legs | A: lift fail | A: held, stable / unstable | B: lift fail | B: held, stable / unstable |
|---|---|---|---|---|---|---|
| Earlier sessions (shared 2.0 s) | 2.0 s | 2.0 s | 33 % | 1 stable, 9 unstable (of 10 held) | 33 % | 7 stable, 1 unstable (of 8 held) |
| 1 | 3.0 s | 1.25 s | 10 of 19 | 9 / **0** | 8 of 20 | 6 / **6** |
| 2 | 2.4 s | 1.25 s | 8 of 19 | 10 / **1** | 9 of 20 | 8 / **3** |
| 3 | 2.15 s | 1.25 s | 12 of 20 | 3 / **5** | 8 of 19 | 9 / **2** |

**What this shows**
1. **B is fixed by 1.25 s legs.** Across pilots 1 and 2, 9 of 23 held B grasps were unstable (39 %), above the 25 % gate. All were tilts. The two pilots differ (50 % and 27 %), which is the sampling noise to expect from 11–12 held grasps per pilot.
2. **A needs something between 2.0 s and 2.4 s.** At 3.0 s none of 9 held grasps was unstable, at 2.4 s 1 of 11 (9 %), and at 2.0 s about 75 %. Pilot 3 (2.15 s) gave 5 of 8 unstable (62 %); interpolating the four points puts 50 % near 2.25 s, which is now in `experiment.yaml`.
3. **Lift failure rates are within 30–60 % in all pilots** (A 42–53 %, B 40–45 %), so the forces from section 6 hold up. One invalid trial per pilot (a missed grasp, and an alignment failure).
4. Each pilot has about 10 held grasps per scenario, so a result of 3 versus 5 unstable is not distinguishable. The main batch (150 trials, about 40 held per scenario) will show the real balance, and the gate is checked on it.

**Practical note.** Disk: each 40-trial session writes about 450 MB of ULog; at about 15 GB free that allows about 1.7 GB for a 150-trial batch and about 6.7 GB for 600.

## 10. Main batch: 150 trials, M5 gates (2026-11-11 to 11-12)

Four sessions (40, 40, 40, 30 trials; seeds 101–104), one battery of 10 per 10 trials = 9 A/B + 1 E, `small_lift` variant with the settings now in `experiment.yaml` (forces A 1.46 N / B 2.75 N, shake legs A 2.25 s / B 1.25 s). `utility/batch.sh "4 4 4 3" 101` restarts the sim stack before each session, because a sim that has run two sessions in a row once drifted into a PX4 estimator fault (phantom 12 m/s speed, "High Accelerometer Bias" failsafe, pilot 3). Sessions: `sim_data/20261111_233750_protocol`, `..._235915_...`, `20261112_002046_...`, `..._004210_...` (about 1.7 GB in all).

| Gate (M5) | Required | A | B | Result |
|---|---|---|---|---|
| Valid trials | at least 95 % | 68 of 68 | 63 of 67 (4 alignment errors) | **97.0 %** of 135 A/B |
| Lift failure rate | 30–60 % per scenario | 34 of 68 (**50 %**) | 26 of 63 (**41 %**) | pass |
| Unstable among held (`y_stable = 0`) | at least 25 % | 13 of 34 (**38 %**) | 14 of 37 (**38 %**) | pass |
| E trials | no payload | | | all 15 came out "empty" |

Datasets (`experiment.sh dataset <session>`, one `dataset/` per session): 131 valid A/B windows in total, each `X_sl` of 75 × 21; `y_success` 71 held / 60 failed; `y_stable` 44 stable / 27 unstable among the 71 held. The `y_stable` confound from section 7 is gone: both scenarios are now about 38 % unstable. The four sessions are merged by `ml/data.py` (section 13).

**Not yet checked at that point** (done in section 12): the leakage test (scenario and applied force absent from the model inputs, M6). The 131 windows are enough for SVM and small-LSTM baselines, not for the CNN-LSTM; the plan's 600 trials is still the target for that.

## 11. Detectability check against the E controls (M1, 2026-11-12)

Question: is the payload visible in the drone's own telemetry? `utility/detectability.py` (new) cuts the same 50 Hz window from t0 for every valid A/B trial and for the 15 E controls, which fly the same motion with nothing in the gripper. The payload step is the mean change over the window relative to the 0.5 s hover before t0, minus the same for E. Two noise measures are used: the hover noise (pooled standard deviation of the channel in the 0.5 s hover, the plan's criterion) and the stricter scatter of the E trials among themselves. Run on all four batch sessions: 68 A and 63 B payload trials against 15 E.

| Channel | Window | A: diff / hover noise | B: diff / hover noise | A: diff / E scatter | B: diff / E scatter |
|---|---|---|---|---|---|
| thrust_sp_z (= motor mean) | whole 1.5 s, held grasps | **210** | **198** | 16.8 | 15.8 |
| thrust_sp_z (= motor mean) | first 0.8 s, all trials | 48 | 26 | 4.2 | **2.3** |
| torque_sp_y | whole 1.5 s, held grasps | 49 | 43 | 14.7 | 13.1 |
| torque_sp_y | first 0.8 s, all trials | 27 | 21 | 7.1 | 5.6 |
| acc_z | first 0.8 s, all trials | 32 | 31 | 13.5 | 13.1 |
| torque_sp_x | whole 1.5 s, held grasps | 0.3 | 3.1 | 0.2 | 1.5 |

**Result: the M1 gate (at least 3× hover noise, per scenario) passes** on thrust, motors, pitch torque (`torque_sp_y`), vertical acceleration and its estimate (`acc_z`, `lpos_az`). `torque_sp_x` carries little payload signal in A (0.3×), probably because of the geometry (the cylinder lies along the world Y axis, so a centred grasp adds little roll moment); not tested. So D10 (change the cylinder mass) is not needed.

Caveats:
- **The simulator hover is very quiet** (thrust noise about 9e-5 per sample), so ratios of 200× will not carry over to the real drone. The E-scatter column, which includes trial-to-trial variation, is the more realistic bar: thrust is 16× over the whole window but only 2.3× (B) to 4.2× (A) in the first 0.8 s, where most failures have not happened yet. B's thrust in that early window is just below 3×; pitch torque and vertical acceleration clear it.
- **The step is the load transfer**: the cylinder rests on the table during the baseline and is lifted during the window. The signal should therefore grow over the window (the whole-window figures are much larger than the first-0.8 s ones, which supports this; the timing was not measured), which links to the early-prediction problem in section 7.
- E is not classifier data (D9); it only supports this check.

## 12. Leakage check (M6, 2026-11-12)

`utility/leakage.py` on the 131 valid A/B windows of the four batch sessions (5-fold CV repeated 10 times; shuffled-label controls give AUC 0.52–0.54). The question is whether a model could read the scenario or the applied grip force off its inputs, because in this simulator the force largely decides `y_success`.

| Check | Result |
|---|---|
| Privileged variables among the 21 input channels | none |
| Static features (`grip_state`, commanded arm pose) | identical in A and B |
| **Scenario (A vs B) from the inputs** | **readable**: AUC 0.99 (logistic and forest), also from the first 0.6 s only (0.99) |
| Grip force within A, all trials, whole window | readable: R² 0.55–0.59 |
| Grip force within A, **held grasps only** | not readable (R² negative) |
| Grip force within A, **first 0.6 s only** | weak: R² 0.17 (ridge) / 0.25 (forest) |
| Grip force within B (any variant) | not readable (R² negative) |

**What the shortcut would be worth.**

| Label | Positive rate | Scenario alone (AUC) | Grip force alone (AUC) |
|---|---|---|---|
| `y_success` | 0.54 (A 0.50, B 0.59) | 0.535 | 0.725 (within A 0.95, within B 0.78) |
| `y_stable` (held only) | 0.62 (A 0.62, B 0.62) | 0.400 | 0.397 (within A 0.49, within B 0.62) |

**Reading.**
1. **The literal M6 wording ("scenario absent from the inputs") does not hold.** A and B are easy to tell apart from the telemetry, even before any failure (the 30 mm grasp offset changes torque and attitude, and would on hardware too). Removing it would mean removing real physics.
2. **The shortcut is worth nothing for the labels.** Scenario alone gives AUC 0.535 for `y_success` and 0.40 for `y_stable`, so the confound of section 7 is gone. Knowing the scenario does not help predict either label.
3. **Force is readable only through the outcome.** In A the force is predictable over the whole window, but not among held grasps, so it comes from the visible drop of the failed (low-force) trials. The small signal in the first 0.6 s of A (R² 0.2) is weak and not tested further; it may be real physics (a loosely held cylinder moves slightly before it drops), but with n = 68 it is not distinguishable from noise.
4. Within A the force by itself predicts `y_success` very well (AUC 0.95), which is why it must stay out of the inputs and why folds should be stratified by scenario and label.

**Consequence for `ml/`.** Report results per scenario and pooled, stratify folds by scenario and `y_success`, and never feed the force. Do not interpret a high pooled score as skill without the within-scenario score next to it. The M6 acceptance criterion should be reworded as: "scenario and force are not inputs, and the scenario alone gives no label information (AUC within 0.1 of 0.5)".

## 13. First baselines (`ml/`, 2026-11-14)

**Merge.** `ml/data.py` concatenates the four per-session datasets into 131 × 75 × 21 windows (hover mean subtracted) with the session name kept as a column; the merged copy is in `Grasp_ws/sim_data/merged_batch1/` (git-ignored with `sim_data/`). `python3 -m ml.run_baselines <session dirs>` (from the repo root) runs the baselines below and writes `ml/results/baselines.json`.

**Setup.** RBF-SVM on 126 per-channel summary features (mean, last 0.1 s, min, max, std, second-half minus first-half), and a 20-unit LSTM on the raw window (training-fold z-scoring, class weights, light noise). Window cut to 0.4 / 0.6 / 0.8 / 1.0 / 1.5 s after t0. Stratified 5-fold CV by scenario × label, repeated (SVM 5×, LSTM 2×), plus leave-one-session-out for the SVM. No hyper-parameter tuning (SVM C = 1; LSTM fixed), so nothing is optimistic from tuning. Inputs are the 21 drone channels only; scenario and force are not inputs (section 12).

**`y_success`** (131 trials, 54 % held; AUC, pooled / A / B; accuracy pooled against the 54 % majority):

| Window end | SVM AUC | LSTM AUC | SVM accuracy | Share of failures already dropped | Reading |
|---|---|---|---|---|---|
| 0.4 s | 0.51 / 0.44 / 0.59 | 0.50 | 0.52 | 0 % | no information |
| 0.6 s | 0.64 / 0.66 / 0.61 | 0.63 | 0.60 | 0 % | weak |
| 0.8 s | 0.74 / 0.74 / 0.75 | 0.75 | 0.67 | 0 % | partial |
| 1.0 s | 0.91 / 0.93 / 0.88 | 0.89 | 0.84 | 20 % (A 35 %, B 0 %) | strong |
| 1.5 s | 0.95 / 0.96 / 0.93 | 0.94 | 0.90 | 92 % | strong |

**How to read this.** Failures drop 0.9–1.1 s after t0 (median 1.06 s, 90 % by 1.4 s), and slip begins at 0.66–0.82 s. A 1.5 s window therefore contains the drop of 92 % of the failures, and the 1.0 s window contains the start of the slip in all of them, so 0.9+ AUC there is detection of a failure under way, not prediction. Genuine early warning is the 0.4–0.8 s rows: none at 0.4 s, weak at 0.6 s (AUC 0.64, 6 points above the majority accuracy), partial at 0.8 s (0.74), where slip has started but nothing has fallen. This confirms the open problem of section 7 with a learned model: with these inputs and this task the lead time before the drop is about 0.2–0.4 s.

**`y_stable`** (71 held grasps, 62 % stable): not predictable from the SL window. AUC 0.45–0.61 for the SVM and 0.43–0.60 for the LSTM at every window end, accuracy at or below the 62 % majority rate, and worse under leave-one-session-out (0.38–0.51). Values below 0.5 are what cross-validation on about 70 noisy trials produces; they are not a finding. The label is decided by the later shake, so a weak link to the SL window is plausible; what this does show is that nothing is learnable here yet.

**Other checks.**
1. **No session drift**: leave-one-session-out matches the repeated CV (1.0 s: 0.90 vs 0.91; 1.5 s: 0.95 vs 0.95).
2. **LSTM = SVM** at this size (131 trials); there is no sign the sequence model helps yet, so a CNN-LSTM is not justified on this data alone.
3. **Uncertainty**: the ± across repeats (0.00–0.05) reuses the same 131 trials and understates the real uncertainty; with this n an AUC has a standard error of about 0.03–0.05.
4. Within each scenario the scores are about as good as pooled (A and B both listed above), so the models are not just telling A from B.

**What this means for the plan.**
- The dataset is usable for `y_success` as a **detection / short-lead** task, which is the honest framing for the advisor: predicting at 0.8 s is possible to a degree (AUC 0.75), at 0.6 s barely, at 0.4 s not.
- Options to buy more lead: (a) a lower payload or lower grip force so the failure comes later and slower; (b) a slower or longer small lift; (c) a richer input (finger travel, the Tier-2 ablation; or, as you suggested, pressure sensors). All three are protocol choices that need your decision.
- `y_stable` needs either more data or a rethink of what the SL window can say about the shake (for example, predicting it from the first 2 s of the lift instead).

## 14. Trying to buy lead time (2026-11-14): lower force, slower lift, closing window

Goal: more warning before the drop, which section 13 put at about 0.2–0.4 s.

**1. Lower force does not help, it shortens the lead.** Over the 60 failed trials of the batch, the failure comes earlier at lower applied force (scenario A: Spearman 0.80 between force and drop time; lowest third of forces drops at a median 0.92 s, highest third at 1.10 s; B: 0.47, median 1.06–1.12 s). Drops stay in a narrow band of 0.9–1.1 s whatever the force, so a weaker grasp mainly fails sooner.

**2. A slower small lift (0.15 m in 2.5 s instead of 1.5 s) delays the failure but not the warning.** Force sweeps with feed-forward on (`sim_data/20261114_200141_tuneA`, `..._201351_tuneB`; 5 trials per level, applied forces include the ±8 % jitter, small_lift shake unchanged):

| | 1.5 s profile (batch) | 2.5 s profile (sweep) |
|---|---|---|
| A: force for 50 % held (logistic fit) | 1.46 N (tuning) | **1.49 N** |
| B: force for 50 % held | 2.75 N | **2.72 N** |
| Slip onset, median | 0.75 s | **1.2 s** (A 1.14–1.32, B 1.1 to 4.8) |
| Drop time of failures, median | 1.06 s | **1.6 s (A) / 1.7 s (B)**; range 1.4–2.2 s, plus two slow creeps in B at 4.9 s |

The forces do not come out lower (the 50 % points are unchanged), so no force retuning is needed. The whole failure is shifted about 0.5 s later.

But the models do not see it earlier. SVM on the 40 sweep trials (only 40, AUC standard error about 0.1; held 25, failed 15):

| Window end | 0.6 s | 0.8 s | 1.0 s | 1.2 s | 1.5 s | 2.5 s |
|---|---|---|---|---|---|---|
| Pooled AUC | 0.59 | 0.50 | 0.47 | 0.45 | **0.88** | 0.96 |

AUC is at chance until the window reaches the slip onset (1.2 s) and jumps once slip and the first drops are inside it (1.5 s). Compare the 1.5 s profile: chance until 0.6 s, 0.74 at 0.8 s (just after the slip onset at 0.75 s). In both cases the signal appears when the cylinder starts to move, not before. The slower lift moved the signal and the failure together, so the lead time did not grow (drop median 1.6 s against first useful window 1.5 s, against 1.06 s and 0.8 s before).

**3. The gripper-closing window carries no information.** The 1.0 s while the fingers close (`X_cg`, ends at t0, 131 trials, SVM, 10× repeated CV): pooled AUC 0.49 (0.47 on the first 0.5 s), A 0.48, B 0.49. Nothing about the later outcome is visible in the drone's telemetry before the lift starts.

**Reading.** With drone-side inputs only, the grasp margin is invisible until the cylinder starts to slip. Changing the timing (slower lift) moves the signal and the failure by the same amount, and force and closing-time telemetry do not contain the margin. So the realistic result is "detection of an impending drop about 0.2–0.4 s ahead at best", and more lead needs an input that senses the grasp itself. In this simulator a force or pressure sensor would measure essentially the commanded grip force, which already predicts `y_success` very well within A (AUC 0.95, section 12), so a sim result with it would be nearly circular; the real test of that idea needs hardware, or a sim sensor with noise and compliance modelled on a real one.

## 15. Finger travel as an input (sim-only Tier-2 check, 2026-11-14)

SVM on the finger positions alone (left, right and their sum, baseline-subtracted, from the 50 Hz trial CSVs; 131 trials; 10× repeated CV): pooled AUC 0.43 / 0.45 / 0.46 / 0.61 / 0.85 at window ends 0.4 / 0.6 / 0.8 / 1.0 / 1.5 s (A 0.44 → 0.94, B 0.42 → 0.70). It behaves like the drone channels: chance until the cylinder starts to slip, then it rises. So finger travel does not add early warning even in the simulator, where it is measured exactly. The real Armbot servos are open-loop (no position feedback; the hardware logger records the commanded gripper angle and `grip_state`), so finger travel is not available on the real gripper without a hardware change. Conclusion: not worth pursuing for lead time.

**Other input combinations (same day).** Pooled AUC of the SVM at window ends 0.4 / 0.6 / 0.8 / 1.0 s, 131 trials, 10× repeated CV: SL window alone 0.50 / 0.65 / 0.75 / 0.91; plus the closing-window features 0.46 / 0.59 / 0.70 / 0.90; plus absolute hover levels 0.45 / 0.64 / 0.71 / 0.90; plus the Tier-1 statics (`grip_state` = 1, shoulder −1.45, elbow −0.12 in every trial) identical to the reference, because they never vary. None improves the early result; the extra features only add noise at this sample size. All baselines use the 21 drone-side channels only.

## 16. Other PX4 log signals (2026-11-14)

`ml/extra_signals.py` cuts further ULog topics into the same 75-sample windows from t0 (hover mean subtracted) for the 131 trials and scores each group with the same SVM and repeated CV. All topics are logged by PX4 on the real drone too. Groups: **tracking** (setpoint minus measured position, velocity, acceleration; 9 channels), **controller** (position-controller thrust output, rate setpoints, rate errors, angular acceleration; 12), **hover_est** (hover-thrust estimator: estimate, variance, innovation, noise; 4), **innov** (EKF2 external-vision innovations; 4), **accel_batt** (filtered body acceleration, battery current; 4). No channel had gaps.

Pooled AUC (A / B), window end:

| Input | 0.4 s | 0.6 s | 0.8 s | 1.0 s |
|---|---|---|---|---|
| 21 channels (reference) | 0.50 (0.44/0.57) | 0.65 (0.66/0.63) | 0.75 (0.74/0.77) | 0.91 (0.93/0.89) |
| tracking | **0.62** (0.60/0.66) | 0.61 | 0.49 | 0.82 |
| controller | 0.47 | 0.59 | 0.74 | 0.89 |
| hover_est | 0.47 | 0.54 | 0.51 | 0.67 |
| innov | 0.44 | 0.44 | 0.44 | 0.51 |
| accel_batt | 0.44 | 0.66 | 0.68 | 0.89 |
| all 33 new | 0.52 | 0.47 | 0.70 | 0.89 |
| 21 + all 33 | 0.52 | 0.54 | 0.75 | 0.90 |

**No group beats the 21 channels from 0.6 s on**, and combining them adds nothing (more features, same 131 trials). From 0.8 s the controller and acceleration groups simply reach the same level as the reference: they see the same event (the slip). The hover-thrust estimator and the vision innovations carry little.

**One weak exception, not established.** The tracking errors at 0.4 s give AUC 0.62–0.64. A permutation test (labels shuffled within scenario, 150 permutations) gives null mean 0.49 ± 0.06, one-sided p = 0.013. But about 20 group-and-window cells were tested, so one cell at p ≈ 0.01 is what chance would produce; no single tracking channel is above 0.56 alone (the signal, if any, is joint); and it does not grow with the window (0.61 at 0.6 s, 0.49 at 0.8 s). It is a hypothesis for a fresh batch, not a finding.

## 17. Summary for the advisor: what changed since the deck, questions, ways forward

**Done since the deck (Oct 17).** M3 (shake) and M5 (batch): `y_stable` confound fixed with a per-scenario shake; 150 trials (131 valid A/B + 15 E) pass all gates (lift failures A 50 % / B 41 %, unstable 38 % / 38 % of held grasps, 97 % valid). M1: the payload is detectable (26–210× the hover noise; 2–4× the trial-to-trial scatter in the first 0.8 s). M6: the scenario is readable from the inputs (AUC 0.99) but carries no label information (AUC 0.54 / 0.40); grip force is readable only through the visible drop. M7 (partly): SVM and LSTM baselines.

**Result.** Detection of an impending drop works (AUC 0.91 at a 1.0 s window, 0.95 at 1.5 s); early prediction does not: AUC 0.51 / 0.64 / 0.74 at 0.4 / 0.6 / 0.8 s, a lead of about 0.2–0.4 s before the drop (median 1.06 s). `y_stable` is not predictable from the small-lift window (AUC about 0.5, 71 held grasps). The LSTM equals the SVM at this size.

**Tried to buy more lead time, none worked** (sections 14–16): lower grip force (failures come earlier), slower lift 2.5 s (slip and drop move together, +0.5 s, no gain), closing window (AUC 0.49), finger travel (0.43–0.46 early), Tier-1 statics (constant), other PX4 signals (none better; one unconfirmed hint in the tracking error).

**Questions for the advisor**

*Framing*
1. Is "detecting an impending drop about 0.2–0.4 s ahead" a sufficient contribution, or must the thesis show earlier prediction? What lead time would be useful on the real system, i.e. what is the abort or re-grasp action and how long does it take?
2. Should the metric change from AUC at fixed window ends to an online detector scored by detection delay against false-alarm rate? It matches how a closed loop would use it.

*Sensing and the real gripper*
3. Is a grasp-sensing channel in scope for the real gripper (force-sensitive pad, servo current or load, or a bus servo with load feedback)? The simulator cannot validate it credibly (its force is commanded). Should I model a noisy, compliant sensor in the simulator first, or go straight to hardware?
4. The real gripper is position-commanded (servo angle), the simulated one force-commanded. Is rebuilding the simulated gripper as a position-controlled, torque-limited servo worth it so that the varied quantity matches the real one?

*Validation*
5. How many real flights are needed to claim transfer, and should the first real test only measure the payload step against the real hover noise (about 20 flights)?
6. Is a simulation-only result acceptable for this chapter if the real flights only confirm the signal level?

*Data and models*
7. Spend about 6 h of simulation on a fresh, held-out batch to test the one weak hint (tracking error at 0.4 s) as a pre-stated hypothesis? Or scale to 600 trials for the planned models?
8. Does the multi-task CNN-LSTM stay in scope, given that the LSTM equals the SVM at 131 trials?
9. Keep `y_stable` as a second task (currently unpredictable)? Alternatives: predict it from the first 2 s of the lift, or redefine stability.

*Physical choices*
10. Is changing the payload (mass, friction, compliant pads) allowed under the "two scenarios, many repetitions" decision, if it makes failures slower or creep-like? (B has a few slow-creep failures at 4–7 s; whether those are predictable is untested.)

**Ways forward** (my assessment)

| Option | Effort | Chance of more lead time | Comment |
|---|---|---|---|
| Reframe as detection with delay versus false alarms | low | n/a | Honest with today's data; cheap |
| Real-hardware signal-to-noise check, about 20 flights | low-medium, needs the drone | n/a | De-risks deployment; the sim's 200× margin will not carry over |
| Replicate the tracking-error hint on a fresh batch | 6 h sim | low | Cheap hypothesis test; likely a null |
| Add a grasp-sensing input (simulator sensor model, then hardware) | high | highest | The only route that carries information about the margin before slip |
| Match the real knob (position-controlled gripper) | high | unknown | Improves realism; changes the failure physics |
| Change the payload or pads for slower failures | medium (re-tune forces) | unknown | Quick to try in sim |

My recommendation is to do the first three now, and make the sensing input the main thing to decide with the advisor.

## 18. Simulated fingertip force sensor (FSR), Stage 1: offline sensor model (2026-11-15)

Plan: `09_fsr_plan.md`. Code: `ml/fsr_model.py` (sensor model), `ml/config/fsr402.yaml` (parameters, cited), `ml/fsr_stage1.py` (evaluation), `ml/tests/test_fsr_model.py` (7 tests; the 91 existing tests still pass). Results: `ml/results/fsr_stage1.json`.

**Sensor model.** Interlink FSR 402 class, from the *FSR 400 Series Data Sheet* (P/N PDS-10004-C): power-law resistance curve read from the datasheet figure (about 22 kΩ at 1 N, exponent 0.8; approximate), 0.2 N actuation, 20 N range, single-part repeatability ±2 %, part-to-part ±6 %, drift < 5 % per decade of time; 5 V divider with R_M = 10 kΩ; 10-bit ADC (Arduino Nano) with 1 LSB noise (assumption). The swept per-trial gain error stands for placement and contact coupling, which the datasheet does not give. The input is the grip force of each trial (CSV column `grip_force_cmd`), in the closing window (1.0 s before t0) and the small-lift window. Each sensor-quality level is averaged over 20 random sensor realizations (the spread between realizations is large: SD up to 0.08 AUC).

**Limit, by construction.** In this simulator the grip force is constant from the close onwards, so the synthetic signal carries the force level and nothing about timing. Stage 1 asks how precisely the force must be read for it to predict `y_success`; it cannot show that a contact force warns earlier. The ideal sensor and `grip_force_cmd` itself are upper bounds, not results.

**Resolution is not the bottleneck.** At the scenario forces (1.46 N and 2.75 N per finger) the reading is about 390 and 517 counts, and an 8 % force change (one SD of the per-trial jitter) moves it by about 15 counts, against 1 count of ADC noise.

**AUC by sensor quality** (FSR in the closing window, i.e. a prediction before lift-off; pooled (A / B); mean ± SD over 20 realizations):

| Per-trial gain error | One FSR | One FSR per finger | Drone + FSR, window to 0.6 s |
|---|---|---|---|
| ideal sensor (upper bound) | 0.82 (0.92 / 0.73) | | |
| `grip_force_cmd` alone (upper bound) | 0.83 (0.92 / 0.74) | | |
| 0 % (datasheet errors only) | 0.79 ± 0.03 (0.88 / 0.68) | 0.78 | 0.69 |
| 5 % | 0.73 ± 0.06 (0.80 / 0.63) | 0.73 | 0.69 |
| 10 % | 0.61 ± 0.08 (0.65 / 0.55) | 0.63 | 0.69 |
| 25 % | 0.53 ± 0.07 (0.53 / 0.51) | 0.49 | 0.66 |
| 50 % | 0.49 ± 0.05 (0.50 / 0.47) | 0.48 | 0.64 |

Label permutation within scenario (FSR closing window, 10 %): null AUC 0.47 ± 0.04. The drone channels alone give 0.64 at 0.6 s.

**AUC by input and window end** (gain error 10 %; pooled (A / B)):

| Input | 0.4 s | 0.6 s | 0.8 s | 1.0 s | 1.5 s |
|---|---|---|---|---|---|
| 21 drone channels | 0.49 (0.42/0.57) | 0.64 (0.66/0.61) | 0.74 (0.73/0.75) | 0.91 (0.93/0.89) | 0.95 (0.96/0.94) |
| FSR alone | 0.58 (0.60/0.54) | 0.58 (0.62/0.52) | 0.59 (0.61/0.55) | 0.58 (0.61/0.53) | 0.58 (0.61/0.53) |
| Drone + FSR | 0.46 (0.42/0.52) | 0.69 (0.70/0.66) | 0.76 (0.76/0.77) | 0.91 (0.92/0.89) | 0.95 (0.96/0.94) |
| FSR, closing window only | 0.61 (0.65/0.55), before lift-off | | | | |

**Reading.**
1. **A force reading would warn before lift-off, but only if it is precise.** With datasheet errors only, the FSR reaches AUC 0.79 (A 0.88) from the closing window, close to the ceiling of the force itself (0.82). At 5 % per-trial error it is 0.73, at 10 % 0.61 and at 25 % chance level.
2. **The tolerance is set by the force spread, not by the sensor alone.** Within a scenario the applied force varies by only ±8 % (one SD) between trials, and that variation is what decides the outcome. A per-trial reading error of the same size hides it. In this simulator the sensor needs a per-trial error of about 5 % or less to be useful. On the real gripper the trial-to-trial squeeze variation is unknown; if it is larger, a given sensor error matters less.
3. **The datasheet's own errors are not the problem; mounting is.** Single-part repeatability (±2 %) and part-to-part spread (±6 %, a fixed offset for one sensor) cost little. The unknown is how repeatably the force reaches the sensor from trial to trial (placement, contact geometry, pad compliance), which no datasheet gives and only a bench test can measure.
4. **Fusion as done here adds little.** Drone + FSR gains about 0.05 AUC at 0.6 s (0.69 against 0.64) when the sensor error is 10 % or less, and nothing at 0.4 s, where it even falls below the FSR alone (0.46 against 0.58): the SVM on 132 summary features dilutes the 6 FSR features. A better fusion (feature selection, or combining two models) is untested.
5. **The FSR signal is flat in time** (0.58 at every window end), as expected from a constant force. One FSR per finger adds nothing here, because both fingers carry the same force in this simulator.
6. **Caveats.** The resistance curve is read by eye from a datasheet figure (±30 %); the ADC noise is assumed; the ±8 % force jitter is a modelling choice of this simulator (frozen randomisation). The result is an upper bound on what a real FSR could give, under the assumption that the squeeze force decides the outcome; it is not evidence that the sensor works on hardware.

**What this does and does not show.** It shows that *if* the grip force decides the outcome and a sensor reads it to within about 5 % from trial to trial, a prediction before lift-off with AUC around 0.75–0.8 is possible. It does not show that a real FSR on the ArmBot fingertip reaches that precision, nor that a contact force changes before slip (Stage 2), nor that the squeeze force decides the outcome on the real gripper.

