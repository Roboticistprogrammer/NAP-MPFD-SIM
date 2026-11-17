# 09 — Simulated fingertip force sensor (FSR): implementation plan

Source: handoff brief "Simulated Fingertip Force Sensor (FSR) for Early Grasp-Failure Warning" (2026-11-15). Builds on `08_small_lift_plan.md` and `result.md` sections 12–17. Stage 1 is done (results: `result.md` section 18). **Stage 2 waits for Amir's approval.**

Note: the brief names the repository `NAP-MPFD`; this work stays in `NAP-MPFD-SIM` (plan decision Q6: the hardware repo is not touched).

## Question

Does a realistic fingertip force signal, alone or with the drone channels, predict `y_success` earlier than the drone channels alone, and how does that depend on sensor quality?

## Stage 1 — offline sensor model (done, no simulator change)

- `ml/fsr_model.py`, parameters in `ml/config/fsr402.yaml`, cited to the Interlink *FSR 400 Series Data Sheet* (P/N PDS-10004-C): power-law resistance curve read from the datasheet figure, 0.2 N actuation, 20 N range, single-part repeatability ±2 %, part-to-part ±6 %, drift < 5 % per decade of time, 5 V divider with R_M = 10 kΩ, 10-bit ADC, 1 LSB noise (assumption). Per-trial gain error (placement and contact coupling) is swept over 0, 5, 10, 25 and 50 %.
- Input force: the per-row grip force of the trial CSVs, through the closing window (50 samples) and the small-lift window (75 samples).
- `ml/fsr_stage1.py` evaluates with the protocol of section 13; `ml/tests/test_fsr_model.py` (7 tests).
- **Limit of Stage 1, by construction.** In this simulator the grip force is a constant from the close onwards, so the synthetic signal has no time structure. Stage 1 answers *how precisely the grip force must be read* to predict the outcome; it cannot show an earlier warning. That needs a measured contact force (Stage 2).

## Stage 2 — contact force in Gazebo (needs approval)

1. **Spike test first (one trial, no batch).** Each finger already touches the cylinder through two pad collisions (`pad_a`, `pad_b` on `finger_{left,right}_link` in `Armbot/model.sdf`). Try both ways of measuring the force and keep the one that gives a usable normal force:
   - a Gazebo contact sensor on the pad collisions (`gz-sim-contact-system`, normal component of the contact wrench). It is not yet checked that the DART engine fills the contact wrench;
   - a force–torque sensor on each finger's prismatic joint (`gz-sim-forcetorque-system`). The component along the joint axis is largely the actuator command; the other components carry the friction load.
2. **Expected problem, to check in the spike.** Both fingers are force-controlled and free to slide together, so in equilibrium the normal contact force equals the command, and the contact signal may be a copy of `grip_force_cmd` again. It can differ when a finger reaches its joint limit (seen in the batch: in a held trial the left finger hits 0 mm while the cylinder shifts 9.6 mm during the small lift) and during fast transients. Whether that happens *before* the 10 mm slip threshold is the question that decides Stage 2.
3. If the spike shows a usable signal: log `fsr_l` / `fsr_r` in the trial CSVs, pass them through the Stage 1 sensor model, and add them to the dataset builder as a separate Tier-2 group. All of it sits behind `small_lift.fsr.enabled` (default `false`); the `legacy` and current `small_lift` defaults do not change.
4. One fresh batch with `utility/batch.sh` (sim restarted between sessions). Disk: 8.6 GB free now; 150 trials ≈ 1.7 GB of ULog, 600 trials ≈ 6.7 GB, which would leave under 2 GB.
5. Both model files (`px4/Tools/...` in this repo and `~/PX4-Autopilot/Tools/...`) are identical today and must be edited together.

## Evaluation (both stages)

Inputs: 21 drone channels (reference), FSR alone, drone + FSR, FSR in the closing window only (a prediction before lift-off). Window ends 0.4 / 0.6 / 0.8 / 1.0 / 1.5 s; RBF-SVM with the existing summary features; stratified 5-fold CV by scenario × label, repeated; AUC pooled and per scenario. Controls: label permutation within scenario; the ideal sensor and `grip_force_cmd` alone as upper bounds; every sensor-quality level reported. `y_stable` as a secondary check only.

## Decisions (Amir, 2026-11-17)

1. **No mass or friction variation.**
2. **One sensor per finger:** Pololu #1696, which is the Interlink FSR 402 (same datasheet as the Stage 1 model). Pololu states a force accuracy of 5–25 % and "qualitative results"; Stage 1 says the sensor needs about 5 % per-trial error to be useful, so the realistic range is the 5–25 % rows of the Stage 1 table (AUC 0.73 down to chance).
3. **Disk:** free space by clearing caches and old files (candidates measured: uv cache 1.7 GB, pip cache 0.25 GB, raw flight logs of the 31 non-batch sessions 4.9 GB; ROS logs are only 2 MB). Deletions only after Amir confirms the list.
4. **Rebuild the simulated gripper** to match the real ArmBot: a two-jaw claw driven like a position-commanded, torque-limited hobby servo, with a contact-force reading at each jaw pad. Stage 2 runs on the rebuilt gripper.
5. **Realistic per-trial sensor error:** not known. Proposed: a bench test with the real sensor on the real jaw (see the reply of 2026-11-17).

## Still open

- Hardware facts for the rebuild: servo model (stall torque, speed), claw geometry (jaw length, pivot spacing, one servo with gears or two), the commanded open and closed angles, where the FSR sits on the jaw.
- The per-trial randomisation knob after the rebuild: jitter on the commanded closing angle (the servo's analogue of today's ±8 % force jitter), keeping the ±3 mm grasp offset.
- `y_stable` design on the new gripper: per-scenario shake as now, or a step-stress shake (same escalating shake for every grasp).
