# 07 — Two-scenario grasp experiment in simulation

Simulates the protocol from the "Two-Scenario Grasp Experiment" slides:
one cylinder, one mass, one surface; only the grasp position changes.

| Scenario | Grasp | Fails by | Drone sees |
|---|---|---|---|
| **A** centred | middle of the cylinder | sliding straight down out of the fingers | a thrust step |
| **B** off-centre | ¼ of the length from the centre (30 mm) | tilting / slipping around the fingers | thrust + a roll moment (torque setpoint, uneven motors) |
| **E** empty control | same motion as A, cylinder parked out of reach | — | baseline |

The difficulty knob is the **grip force** (newtons per finger), set per
scenario so that 30–60 % of trials fail. The values tuned on 2026-10-03 are
**A = 1.40 N** and **B = 2.4 N**, each giving about 50 % failures:

| | below | around the threshold | above |
|---|---|---|---|
| A | 1.30–1.39 N: all fail, clean slides | ~1.40 N | ≥ 1.45 N mostly hold. At 1.5–1.6 N failures become "slip, then pivot and hang" |
| B | 1.8–2.2 N: all fail, pivoting round the fingers until it hangs from one end | ~2.4 N | 2.6 N fails 1/4. At 3–4 N it holds after rotating 9–22° |

## What is in the world

`x500_gripper_armbot.sdf`, 3 m in front of the takeoff point:

- **Table**: Gazebo Fuel `OpenRobotics/Table`, top surface at z = 1.015 m.
- **Cylinder**: `Grasp_ws/model/cylinder` (r 25 mm, L 120 mm, 0.15 kg, μ 1.0),
  lying with its axis along world Y, across the drone's heading.
- **Jig**: two 4 mm rails that cradle the cylinder. The simulator has no
  rolling resistance, so without the jig the cylinder rolls away between
  trials. A second jig holds it at the park position used for E trials.
- **Gripper**: Armbot's original single hinged jaw (a rod against a small
  block) had no opposing surface and could not hold anything. It is now a
  two-finger parallel gripper. Each finger is force controlled and touches
  the object through two friction pads 20 mm apart. The pad layout fixes the
  friction torque around the pinch axis, which is what scenario B's tilt
  failure depends on. The old model is kept as `Armbot/model.sdf.orig`.

## Running it

From `Grasp_ws/utility/`:

```bash
./launch_sim.sh --headless                      # or with GUI to watch
./experiment.sh single A                        # one trial, quick check
./experiment.sh tune A --grip 1.3 1.4 1.5 --trials 10      # to re-check / re-tune
./experiment.sh tune B --grip 2.0 2.4 2.8 --trials 10
#   -> put the levels giving 30-60 % failures into
#      src/px4_gripper_missions/config/experiment.yaml (scenarios.A/B.grip_force_n)
./experiment.sh protocol                        # 30 batteries x 10 trials
./experiment.sh export ../sim_data/<session>    # per-trial merged tables
./experiment.sh relabel ../sim_data/<session>   # recompute outcomes after changing thresholds
```

`--grip A=0.85 B=2.2` overrides the YAML for `single`/`protocol`. `--debug`
prints the alignment loop. Ctrl+C at any time: the drone flies home, lands,
and everything recorded so far is kept.

A trial takes about 20 s, so the full protocol (300 trials) takes roughly
1 h 45 min. Run it headless.

### One trial

1. Gripper over the target at 12 cm. The cylinder is teleported into its jig,
   which also zeroes its velocity (E: into the park jig).
2. **baseline**: 1 s hover (recording starts here).
3. **descend** until the grasp point is 6 mm above the cylinder axis, with
   closed-loop alignment to < 4 mm.
4. **settle** for 1 s, then **close**: grip force applied, 1 s.
5. **lift**: 0.40 m at 0.3 m/s, then **hold** for 4 s. This is the outcome window.
6. **release**: a held cylinder is set back down before opening; a dropped
   one is just let go.

A protocol battery has 10 trials: A and B interleaved (5/4, then 4/5) plus
one E, shuffled with a fixed seed.

### Small-lift variant (branch `feature/small-lift-protocol`)

`protocol.variant: small_lift` (or `--variant small_lift`) replaces step 5 by a
0.10 m / 1.5 s minimum-jerk small lift (t0 = its start), then a minimum-jerk
rise to +1.0 m, a 4 s hold, and, if the cylinder is still held, a lateral
shake of ±0.3 m along the cylinder axis. `trials.csv` gains `t_close`,
`t_sl_end`, `t_shake`, `y_success`, `y_stable`. See `08_small_lift_plan.md`.

```bash
./experiment.sh protocol --batteries 1 --empty 0 --variant small_lift   # 10 trials, A and B only
./experiment.sh dataset ../sim_data/<session>                           # dataset/windows.npz + labels.parquet
```

## Outputs

`Grasp_ws/sim_data/<YYYYmmdd_HHMMSS>_<mode>/`

| File | Content |
|---|---|
| `session.json` | config snapshot, trial plan, PX4 parameters used, ULog path |
| `trials.csv` | one row per trial: scenario, battery/slot, grip force (nominal and as applied), commanded and actual grasp offset, `outcome` (success / fail / empty / invalid), `failure_mode` (slide / tilt), `slipped`, `t_onset_s` / `t_drop_s` (after lift start), peak displacement and tilt, validity and reason |
| `trials/tNNNN_X.csv` | 50 Hz Gazebo ground truth with phase labels: drone pose, grasp point, cylinder pose, cylinder displacement and tilt relative to the grip, finger travel, commanded grip force |
| `flight.ulg` | the session's PX4 log (thrust and torque setpoints and motor outputs at full rate) |
| `merged/tNNNN_X.csv` | after `export`: a 100 Hz grid per trial. Drone-side signals (thrust/torque setpoints, `motor_0..3`, attitude, body rates, accelerometer, EKF z/vz), joined with the ground truth and the trial labels. This is the table to train or evaluate a detector on |

Outcome labelling uses the cylinder's position relative to the grasp point,
compared with where it was when the fingers closed. It counts as **slipped**
after 10 mm and as **dropped** (failure) after 40 mm. It also counts as a
failure if its axis rotates more than 45°, because an off-centre cylinder
can swing around the fingers and hang there. A failure is **tilt** if the
axis rotated more than 15° by the time it failed, otherwise **slide**. The thresholds are in `experiment.yaml`, under `outcome`.

## How the simulation is set up, and why

- **Emulated motion capture.** Grasping needs the gripper placed to a few
  millimetres. With the simulated GPS, PX4's estimate wanders by
  centimetres, and the hover wanders with it. Like a lab with a mocap
  system, the runner publishes the true pose to
  `/fmu/in/vehicle_visual_odometry`, and EKF2 fuses it with GPS and baro off
  (`mocap.py`). The estimator, controllers, setpoints and motor outputs are
  all still PX4's own. Set `px4.mocap: false` to fly on GPS.
- **PX4 parameters are changed for the session and restored afterwards**:
  - mocap EKF2 setup;
  - `NAV_DLL_ACT=0` and `COM_RCL_EXCEPT=4`, so it can fly headless without a GCS or RC;
  - high-rate logging (`SDLOG_PROFILE |= 16`);
  - `SDLOG_ROTATE=0` and `SDLOG_MAX_SIZE=0`.

  The last two matter on this machine. The disk is about 92 % full and
  `SDLOG_ROTATE=90` makes PX4 delete old ULogs every time logging starts. The
  session ULog is copied into the session folder for the same reason. If a
  session crashes before restoring, `launch_sim.sh` resets the EKF settings
  on the next start.
- **Per-trial variability** (`randomization` in the YAML): the grip force
  varies by ±8 % (1σ) and the grasp position by ±3 mm (1σ) along the axis.
  Without this, a simulator gives the same outcome for every trial at a given
  grip force. These two stand in for real-world scatter. The applied values
  are logged per trial.
- **Timing**: PX4 timestamps equal Gazebo sim time (SITL lockstep), so ULog
  data and ground truth line up without any offset.

## Limitations

- **No rotor downwash** on the cylinder or table.
- **Contact friction** comes from Gazebo's DART solver, with coefficient 1.0
  on both the pads and the cylinder. In an isolated pinch test (same fingers
  and cylinder, no drone), the static hold threshold was **1.35 N per
  finger**, against 0.74 N from μ = 1, so the effective μ is about 0.54. The
  thresholds are sharp: below them the cylinder never leaves the jig. Tune
  the absolute grip levels in sim, not from real-world values.
- **Arm joints**: the shoulder and elbow are stiff position-controlled
  joints, so the arm doesn't comply the way a real servo arm would.
- **Batteries**: a "battery" is only a block of trials here. There is no
  battery swap, and SITL battery drain doesn't affect the thrust signals.
- **Old scenario scripts**: the earlier `run_scenario` SC01–SC05 scripts
  still command the removed `gripper_joint`. The experiment runner
  (`run_experiment` / `experiment.sh`) replaces them.
