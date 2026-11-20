"""Two-scenario grasp experiment runner (A centred / B off-centre / E empty).

    run_experiment protocol [--batteries N] [--grip A=0.8 B=2.0]
    run_experiment tune --scenario A --grip 0.7 0.8 0.9 [--trials 10]
    run_experiment single --scenario B [--grip B=2.0]    one trial, for debugging

Flies the x500 + Armbot to the table, grasps the cylinder with the force-
controlled fingers, lifts, holds, labels the outcome from Gazebo ground
truth and resets the cylinder - repeatedly, from the same start state.

Per session (sim_data/<session>/):
    session.json        config snapshot, trial plan, PX4 ULog path
    trials.csv          one row per trial: scenario, grip, outcome, timings
    trials/tNNNN_X.csv  50 Hz Gazebo ground truth + phase labels per trial
The drone-side signals (thrust/torque setpoints, motor outputs, IMU) are
in the PX4 ULog of the session; `export_trials <session>` slices it per
trial and merges it with the ground truth (PX4 time == sim time in SITL
lockstep).

Run with PYTHONNOUSERSITE=1 (see gz_world.py); utility/experiment.sh
does this and checks the sim is up first.
"""
from __future__ import annotations

import argparse
import csv
import glob
import hashlib
import json
import math
import os
import random
import re
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

from px4_gripper_missions.config_loader import get_config_dir
from px4_gripper_missions.geometry import axis_tilt_deg, enu_to_ned, quat_to_rpy
from px4_gripper_missions.trajectory import (duration_for_peak_speed, min_jerk, min_jerk_rate,
                                           shake_legs)

SCENARIOS = ('A', 'B', 'E')
VARIANTS = ('legacy', 'small_lift')
# Phases whose cylinder motion decides y_success, per protocol variant.
OUTCOME_PHASES = {'legacy': ('lift', 'hold'), 'small_lift': ('small_lift', 'lift', 'hold')}


class SessionAbort(RuntimeError):
    """Unrecoverable: PX4 left offboard, sim stopped, ... - land and stop."""


class TrialInvalid(RuntimeError):
    """This trial cannot be labelled (missed grasp, alignment timeout, ...)."""


# ---------------------------------------------------------------- planning

@dataclass
class TrialSpec:
    index: int
    battery: int
    slot: int
    scenario: str
    grip_force_n: Optional[float] = None   # None -> scenario default


def generate_protocol_plan(batteries: int, per_battery: int, empty_per_battery: int,
                           seed: int, scenarios: Sequence[str] = ('A', 'B')) -> List[TrialSpec]:
    """Batteries of `per_battery` trials: A and B interleaved (counts
    alternate 5/4, 4/5 when odd), `empty_per_battery` E controls, order
    shuffled within each battery."""
    rng = random.Random(seed)
    plan: List[TrialSpec] = []
    work = per_battery - empty_per_battery
    if work < 0:
        raise ValueError('empty_per_battery > trials_per_battery')
    for b in range(batteries):
        slots: List[str] = []
        if len(scenarios) == 2:
            first = work // 2 + (work % 2 if b % 2 == 0 else 0)
            slots += [scenarios[0]] * first + [scenarios[1]] * (work - first)
        else:
            slots += [scenarios[i % len(scenarios)] for i in range(work)]
        slots += ['E'] * empty_per_battery
        rng.shuffle(slots)
        for s_i, scen in enumerate(slots):
            plan.append(TrialSpec(len(plan), b + 1, s_i + 1, scen))
    return plan


def gripper_type(cfg: dict) -> str:
    """'servo_claw' (jaw angle command, rad) or 'force_fingers' (finger force, N)."""
    return (cfg.get('gripper') or {}).get('type', 'force_fingers')


def scenario_grip_force(cfg: dict, scenario: str) -> float:
    """Nominal grip command for a scenario: finger force [N] for the force_fingers
    gripper, jaw angle [rad] for the servo_claw (small_lift.claw_cmd_rad). The
    small_lift variant may carry its own tuned forces (small_lift.grip_force_n);
    otherwise the legacy ones apply."""
    if gripper_type(cfg) == 'servo_claw':
        return float(cfg['small_lift']['claw_cmd_rad'][scenario])
    if cfg['protocol'].get('variant', 'legacy') == 'small_lift':
        own = (cfg.get('small_lift') or {}).get('grip_force_n') or {}
        if scenario in own:
            return float(own[scenario])
    return float(cfg['scenarios'][scenario]['grip_force_n'])


def jittered_grip(cfg: dict, nominal: float, rng) -> float:
    """Per-trial grip command: the finger force varies by a fraction (force_fingers),
    the jaw angle command by an absolute amount in rad (servo_claw)."""
    rz = cfg['randomization']
    if gripper_type(cfg) == 'servo_claw':
        return float(nominal + rz['claw_cmd_jitter_rad'] * rng.standard_normal())
    return float(nominal * (1.0 + rz['grip_force_jitter_frac'] * rng.standard_normal()))


def scenario_shake(cfg: dict, scenario: str) -> dict:
    """Shake settings for a scenario: the shared `shake` block, with any keys from
    `shake.per_scenario.<scenario>` (e.g. leg_duration_s, amplitude_m) overriding it.
    The shake only decides y_stable, which is labelled after the window the models
    see, so a per-scenario intensity is a difficulty knob like the grip force."""
    sh = dict(cfg['shake'])
    over = (sh.pop('per_scenario', None) or {}).get(scenario) or {}
    sh.update(over)
    return sh


def set_scenario_grip_force(cfg: dict, scenario: str, value: float) -> None:
    """Command-line override: write where scenario_grip_force() will read."""
    if gripper_type(cfg) == 'servo_claw':
        cfg['small_lift'].setdefault('claw_cmd_rad', {})[scenario] = float(value)
        return
    if cfg['protocol'].get('variant', 'legacy') == 'small_lift':
        cfg.setdefault('small_lift', {}).setdefault('grip_force_n', {})[scenario] = float(value)
    else:
        cfg['scenarios'][scenario]['grip_force_n'] = float(value)


# ---------------------------------------------------------------- outcome

@dataclass
class Outcome:
    outcome: str                  # success | fail | empty | invalid
    failure_mode: str = ''        # slide | tilt (fail only)
    slipped: bool = False         # moved > slip_onset but still held
    t_onset_s: float = float('nan')   # after lift start
    t_drop_s: float = float('nan')
    peak_disp_m: float = 0.0
    peak_tilt_change_deg: float = 0.0


def classify_outcome(t: Sequence[float], disp: Sequence[float], tilt_deg: Sequence[float],
                     t_lift: float, slip_onset_m: float, drop_m: float,
                     tilt_threshold_deg: float, tilt_fail_deg: float = 45.0) -> Outcome:
    """Label a grasp from the cylinder's displacement relative to the grasp
    point (and its axis tilt), over the lift + hold window.

    disp/tilt are measured relative to their values when the fingers had
    closed, so 0 = still exactly where it was gripped. Failed = dropped
    (disp > drop_m) or rotated out of the grasp (|tilt| > tilt_fail_deg -
    an off-centre cylinder can swing round the fingers and hang there)."""
    t = np.asarray(t, dtype=float)
    disp = np.asarray(disp, dtype=float)
    tilt = np.asarray(tilt_deg, dtype=float)
    if len(t) == 0:
        return Outcome('invalid')
    o = Outcome('success', peak_disp_m=float(disp.max()),
                peak_tilt_change_deg=float(np.abs(tilt).max()))
    onset = np.nonzero(disp > slip_onset_m)[0]
    over_disp = np.nonzero(disp > drop_m)[0]
    over_tilt = np.nonzero(np.abs(tilt) > tilt_fail_deg)[0]
    if len(onset):
        o.t_onset_s = float(t[onset[0]] - t_lift)
        o.slipped = True
    if len(over_disp) or len(over_tilt):
        i_disp = over_disp[0] if len(over_disp) else len(t)
        i_tilt = over_tilt[0] if len(over_tilt) else len(t)
        i_fail = min(i_disp, i_tilt)
        o.outcome = 'fail'
        o.t_drop_s = float(t[i_fail] - t_lift)
        # How it let go. "tilt": the cylinder pivoted round the fingers -
        # it rotated past tilt_fail_deg before moving drop_m (an off-centre
        # cylinder swings until it hangs from one end), or had already
        # turned tilt_threshold_deg at slip onset. "slide": it fell out
        # with little rotation (late rotation while leaving the pads does
        # not count).
        i0 = onset[0] if len(onset) else i_fail
        pivoted = i_tilt <= i_disp
        o.failure_mode = 'tilt' if pivoted or abs(tilt[i0]) > tilt_threshold_deg else 'slide'
    return o


def classify_stability(t: Sequence[float], d_xyz: Sequence[Sequence[float]],
                       tilt_deg: Sequence[float], t_shake: float, slip_onset_m: float,
                       drop_m: float, tilt_threshold_deg: float,
                       tilt_fail_deg: float = 45.0) -> Outcome:
    """Label the stability shake (y_stable): the same rules as classify_outcome,
    applied to the cylinder's motion relative to where it was when the shake
    began. `d_xyz` is the per-sample cylinder offset from the grasp point
    (rel_dx, rel_dy, rel_dz); 'success' here means it stayed in the grasp."""
    t = np.asarray(t, dtype=float)
    if len(t) == 0:
        return Outcome('invalid')
    d = np.asarray(d_xyz, dtype=float).reshape(-1, 3)
    tilt = np.asarray(tilt_deg, dtype=float)
    return classify_outcome(t, np.linalg.norm(d - d[0], axis=1), tilt - tilt[0], t_shake,
                            slip_onset_m, drop_m, tilt_threshold_deg, tilt_fail_deg)


# ---------------------------------------------------------------- recording

TRIAL_COLUMNS = [
    't_sim', 'phase',
    'drone_x', 'drone_y', 'drone_z', 'drone_roll', 'drone_pitch', 'drone_yaw',
    'grasp_x', 'grasp_y', 'grasp_z',
    'cyl_x', 'cyl_y', 'cyl_z', 'cyl_tilt_deg',
    'rel_dx', 'rel_dy', 'rel_dz', 'rel_disp', 'tilt_change_deg',
    'finger_l', 'finger_r', 'shoulder', 'elbow',
    'grip_force_cmd', 'cmd_x', 'cmd_y', 'cmd_z',
    'fsr_l', 'fsr_r',
]
# finger_l / finger_r: finger travel [m] (force_fingers) or jaw angle [rad] (servo_claw).
# grip_force_cmd: the grip command (N or rad, same split). fsr_l / fsr_r: normal
# contact force on the left / right jaw pads [N] (servo_claw; nan otherwise).


class TrialRecorder:
    """Collects one row per gz pose message (50 Hz) while active."""

    def __init__(self):
        self._lock = threading.Lock()
        self.rows: List[list] = []
        self.active = False
        self.phase = ''
        self.rel0: Optional[np.ndarray] = None
        self.tilt0 = 0.0
        self.cmd = np.zeros(3)
        self.grip = 0.0

    def start(self) -> None:
        with self._lock:
            self.rows, self.active, self.rel0, self.tilt0 = [], True, None, 0.0

    def stop(self) -> List[list]:
        with self._lock:
            self.active = False
            rows, self.rows = self.rows, []
            return rows

    def set_reference(self, rel0: np.ndarray, tilt0: float) -> None:
        with self._lock:
            self.rel0, self.tilt0 = rel0, tilt0

    def __call__(self, s) -> None:
        if not self.active:
            return
        g = s.grasp_point
        rel = s.cyl_pos - g
        tilt = axis_tilt_deg(s.cyl_q)
        with self._lock:
            if not self.active:
                return
            if self.rel0 is not None:
                d = rel - self.rel0
                disp, dtilt = float(np.linalg.norm(d)), tilt - self.tilt0
            else:
                d, disp, dtilt = np.zeros(3), 0.0, 0.0
            r, p, y = quat_to_rpy(s.drone_q)
            j = s.joints
            self.rows.append([
                round(s.sim_t, 4), self.phase,
                *np.round(s.drone_pos, 5), round(r, 5), round(p, 5), round(y, 5),
                *np.round(g, 5), *np.round(s.cyl_pos, 5), round(tilt, 3),
                *np.round(d, 5), round(disp, 5), round(dtilt, 3),
                round(j.get('finger_left_joint', j.get('jaw_left_joint', float('nan'))), 5),
                round(j.get('finger_right_joint', j.get('jaw_right_joint', float('nan'))), 5),
                round(j.get('shoulder_pitch_joint', float('nan')), 4),
                round(j.get('elbow_pitch_joint', float('nan')), 4),
                self.grip, *np.round(self.cmd, 4),
                round(s.fsr_l, 4), round(s.fsr_r, 4),
            ])


# ---------------------------------------------------------------- runner

def load_experiment_config(path: Optional[str] = None) -> dict:
    path = path or os.path.join(get_config_dir(), 'experiment.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cfg['_path'] = os.path.abspath(path)
    return cfg


class Experiment:
    def __init__(self, cfg: dict, out_dir: str, label: str):
        self.cfg = cfg
        self.m = cfg['motion']
        self.variant = cfg['protocol'].get('variant', 'legacy')
        if self.variant not in VARIANTS:
            raise ValueError(f'protocol.variant must be one of {VARIANTS}, got {self.variant!r}')
        self.session = f'{time.strftime("%Y%m%d_%H%M%S")}_{label}'
        self.out = os.path.join(out_dir, self.session)
        os.makedirs(os.path.join(self.out, 'trials'), exist_ok=True)
        self.rng = np.random.default_rng(cfg['randomization']['seed'])
        self.recorder = TrialRecorder()
        self.offset_ned: Optional[np.ndarray] = None
        self.cmd_world = np.zeros(3)
        self.ulog_path = ''
        self.results: List[dict] = []
        self._summary_file = None
        self._summary = None

    # -- infrastructure ------------------------------------------------------

    def _px4(self, *args: str) -> str:
        exe = os.path.join(os.path.expanduser(self.cfg['px4']['bin_dir']), f'px4-{args[0]}')
        r = subprocess.run([exe, *args[1:]], capture_output=True, text=True, timeout=10)
        return r.stdout + r.stderr

    def _param_get(self, name: str) -> Optional[str]:
        m = re.search(rf'{name} \[[^\]]*\] : (\S+)', self._px4('param', 'show', name))
        return m.group(1) if m else None

    def _param_set(self, name: str, value) -> None:
        """Set a PX4 parameter, remembering the original for restore_px4().
        (SITL saves parameters, so anything left changed survives a restart.)"""
        if name not in self._param_backup:
            self._param_backup[name] = self._param_get(name)
        self._px4('param', 'set', name, str(value))

    def _set_px4_params(self) -> Dict[str, int]:
        self._param_backup: Dict[str, Optional[str]] = {}
        applied = {}
        cur = self._param_get('SDLOG_PROFILE')
        prof = (int(cur) if cur else 1) | int(self.cfg['px4'].get('sdlog_profile_or', 0))
        self._param_set('SDLOG_PROFILE', prof)
        applied['SDLOG_PROFILE'] = prof
        # One ULog for the whole session, and never deleted by PX4's
        # free-space log rotation (SDLOG_ROTATE wipes old logs at every log
        # start when the disk is fuller than its threshold).
        for k, v in (('SDLOG_ROTATE', 0), ('SDLOG_MAX_SIZE', 0)):
            self._param_set(k, v)
            applied[k] = v
        for k, v in (self.cfg['px4'].get('params') or {}).items():
            self._param_set(k, v)
            applied[k] = v
        # The logger picks its topic set at start-up, and SITL starts logging
        # at boot; restart it so the session ULog (written while armed) uses
        # the profile above.
        self._restart_logger()
        return applied

    def _restart_logger(self) -> None:
        self._px4('logger', 'stop')
        time.sleep(0.5)
        self._px4('logger', 'start', '-b', '1000', '-t', '-m', 'file')

    def restore_px4(self) -> None:
        """Put back every parameter this session changed (mocap EKF setup,
        failsafe tweaks, log profile) and restart EKF2/logger to apply."""
        backup = getattr(self, '_param_backup', {})
        if not backup:
            return
        from px4_msgs.msg import VehicleStatus
        if (hasattr(self, 'offboard') and self.offboard.vehicle_status.arming_state
                == VehicleStatus.ARMING_STATE_ARMED):
            # Never restart EKF2 / switch estimator sources while flying.
            print('[exp] WARNING: vehicle still armed - PX4 parameters NOT restored. Land, '
                  'then restart the sim (launch_sim.sh restores the EKF defaults).')
            return
        if getattr(self, 'mocap', None) is not None:
            self.mocap.enabled = False
        from px4_gripper_missions.mocap import MOCAP_PARAMS
        for name, value in backup.items():
            if value is None:
                continue
            if name in MOCAP_PARAMS and float(value) == float(MOCAP_PARAMS[name]):
                # PX4 was already in mocap mode when the session started (a
                # crashed earlier session): go back to the airframe default.
                self._px4('param', 'reset', name)
            else:
                self._px4('param', 'set', name, value)
        if any(n.startswith('EKF2_') for n in backup):
            self._px4('ekf2', 'stop')
            time.sleep(1.0)
            self._px4('ekf2', 'start')
        self._restart_logger()
        print(f'[exp] restored PX4 parameters: {", ".join(backup)}')

    def connect(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        from px4_gripper_missions.gz_world import GzWorld
        from px4_gripper_missions.px4_offboard import PX4OffboardController, Setpoint

        mdl = self.cfg['models']
        self.gz = GzWorld(self.cfg['world'], mdl['drone'], mdl['arm'], mdl['cylinder'],
                          gripper_type=gripper_type(self.cfg))
        self.gz.start()
        self.gz.add_listener(self.recorder)

        from rclpy.signals import SignalHandlerOptions
        # rclpy's own SIGINT handler shuts the context down at once, which
        # would kill the offboard heartbeat and mocap stream mid-flight
        # (PX4 failsafe). Let Ctrl+C raise KeyboardInterrupt instead, so
        # main() can fly home and land with ROS still up.
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('grasp_experiment')
        self.offboard = PX4OffboardController(
            self.node, Setpoint(0.0, 0.0, 0.0, self.m['heading_ned_rad']), setpoint_rate_hz=20.0)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

        if self.cfg['px4'].get('mocap', True):
            self._start_mocap()
        deadline = time.monotonic() + 15
        while not self._local_position_valid():
            if time.monotonic() > deadline:
                raise SessionAbort('no valid PX4 local position over ROS 2 '
                                   '(is the XRCE-DDS agent running? px4_msgs matching?)')
            time.sleep(0.1)
        self._update_offset(force=True)

    def _start_mocap(self) -> None:
        """Feed ground truth to EKF2 as external vision and switch EKF2 to
        it (GPS/baro off), then check the estimate matches the truth."""
        from px4_gripper_missions.mocap import MOCAP_PARAMS, MocapBridge, gz_pose_to_px4
        self.mocap = MocapBridge(self.node)
        self.gz.add_listener(self.mocap)
        self.mocap.enabled = True
        time.sleep(1.0)
        for k, v in MOCAP_PARAMS.items():
            self._param_set(k, v)
        self._px4('ekf2', 'stop')
        time.sleep(1.0)
        self._px4('ekf2', 'start')
        deadline = time.monotonic() + 30
        while True:
            time.sleep(0.5)
            lp = self.offboard.vehicle_local_position
            s = self.gz.state()
            truth, _ = gz_pose_to_px4(s.drone_pos, s.drone_q)
            err = np.linalg.norm(np.array([lp.x, lp.y, lp.z]) - truth)
            if self._local_position_valid() and err < 0.02:
                print(f'[exp] mocap emulation active (EKF vs truth {err * 1000:.1f} mm)')
                return
            if time.monotonic() > deadline:
                raise SessionAbort(f'EKF2 did not converge to the mocap pose (error {err:.3f} m)')

    def disconnect(self) -> None:
        import rclpy
        if getattr(self, 'mocap', None) is not None:
            self.mocap.enabled = False
            time.sleep(0.1)
        try:
            self.offboard.shutdown()
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass

    def _local_position_valid(self) -> bool:
        lp = self.offboard.vehicle_local_position
        return bool(lp.xy_valid and lp.z_valid and lp.timestamp > 0)

    def _update_offset(self, force: bool = False) -> None:
        """offset = PX4 local NED - Gazebo truth (in NED axes), so targets
        can be given in world coordinates. With mocap emulation this is a
        constant (body origin above the model origin); without it, it also
        absorbs slow EKF drift. Only updated while nearly stationary, where
        the latency between the two streams does not matter."""
        lp = self.offboard.vehicle_local_position
        if not self._local_position_valid():
            return
        speed = math.sqrt(lp.vx ** 2 + lp.vy ** 2 + lp.vz ** 2)
        est = np.array([lp.x, lp.y, lp.z]) - enu_to_ned(self.gz.state().drone_pos)
        if force or self.offset_ned is None:
            self.offset_ned = est
        elif speed < 0.05:
            self.offset_ned = 0.97 * self.offset_ned + 0.03 * est

    def _check_px4(self) -> None:
        from px4_msgs.msg import VehicleStatus
        st = self.offboard.vehicle_status
        if self._armed_once and (st.arming_state != VehicleStatus.ARMING_STATE_ARMED
                                 or st.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            raise SessionAbort(f'PX4 left armed offboard flight (arming_state='
                               f'{st.arming_state}, nav_state={st.nav_state})')

    def _command(self, p_world: np.ndarray, v_world=None) -> None:
        """Command the drone to p_world (ENU). `v_world` (ENU m/s) is an optional
        velocity feed-forward; None = position only."""
        self.cmd_world = np.asarray(p_world, dtype=float)
        self.recorder.cmd = self.cmd_world
        sp = enu_to_ned(self.cmd_world) + self.offset_ned
        v = enu_to_ned(v_world) if v_world is not None else np.zeros(3)
        self.offboard.goto_position(float(sp[0]), float(sp[1]), float(sp[2]),
                                    self.m['heading_ned_rad'], float(v[0]), float(v[1]), float(v[2]))

    # -- motion primitives -----------------------------------------------

    def _ramp_drone_to(self, target: np.ndarray, speed: float) -> None:
        """Move the commanded drone position to `target` at `speed` (sim time)."""
        start = self.cmd_world.copy()
        dist = float(np.linalg.norm(target - start))
        if dist < 1e-4:
            self._command(target)
            return
        t0 = self.gz.sim_time()
        while True:
            self._check_px4()
            self._update_offset()
            frac = min(1.0, (self.gz.sim_time() - t0) * speed / dist)
            self._command(start + (target - start) * frac)
            if frac >= 1.0:
                return
            time.sleep(0.02)

    def _run_min_jerk(self, delta, duration_s: float) -> None:
        """Move the commanded drone position by `delta` (world ENU) along a
        minimum-jerk profile lasting `duration_s` of sim time; the move starts
        and ends at rest. With small_lift.velocity_feedforward the matching
        velocity is sent along with each position setpoint."""
        start = self.cmd_world.copy()
        delta = np.asarray(delta, dtype=float)
        ff = bool(self.cfg.get('small_lift', {}).get('velocity_feedforward', False))
        t0 = self.gz.sim_time()
        while True:
            self._check_px4()
            self._update_offset()
            elapsed = self.gz.sim_time() - t0
            pos = start + delta * float(min_jerk(elapsed / duration_s))
            if ff:
                self._command(pos, delta * float(min_jerk_rate(elapsed / duration_s)) / duration_s)
            else:
                self._command(pos)
            if elapsed >= duration_s:
                return
            time.sleep(0.02)

    def _wait_drone_at(self, target: np.ndarray, tol: float, timeout_s: float) -> None:
        t_end = self.gz.sim_time() + timeout_s
        while np.linalg.norm(self.gz.state().drone_pos - target) > tol:
            self._check_px4()
            self._update_offset()
            self._command(target)
            if self.gz.sim_time() > t_end:
                raise TrialInvalid(f'drone did not reach {np.round(target, 2)}')
            time.sleep(0.05)

    def fly_drone_to(self, target, speed: float, tol: float = 0.08, timeout_s: float = 30.0) -> None:
        target = np.asarray(target, dtype=float)
        self._ramp_drone_to(target, speed)
        self._wait_drone_at(target, tol, timeout_s)

    def move_grasp_to(self, target_grasp, speed: float, fine: bool = True,
                      axes=(True, True, True)) -> None:
        """Move so that the gripper's grasp point reaches target_grasp:
        ramp using the current drone->grasp offset, then (fine=True) close
        the remaining error from ground truth until within align_tol."""
        target_grasp = np.asarray(target_grasp, dtype=float)
        s = self.gz.state()
        r = s.grasp_point - s.drone_pos
        self._ramp_drone_to(target_grasp - r, speed)
        if not fine:
            return
        mask = np.array(axes, dtype=float)
        tol = self.m['align_tol_m']
        t_end = self.gz.sim_time() + self.m['align_timeout_s']
        debug = bool(os.environ.get('EXP_DEBUG'))
        for _ in range(20):
            # Move-and-settle: wait until the vehicle is at rest, measure the
            # grasp-point error averaged over 0.4 s (the arm hangs 0.5 m
            # below the drone, so tiny tilts swing it), then correct once.
            # A continuous integrator here fights the position loop's lag.
            self._wait_settled(t_end)
            errs = []
            t_win = self.gz.sim_time() + 0.4
            while self.gz.sim_time() < t_win:
                errs.append((target_grasp - self.gz.state().grasp_point) * mask)
                self.gz.sleep_sim(0.04)
            err = np.mean(errs, axis=0)
            if debug:
                print(f'  [align] t={self.gz.sim_time():7.2f} err_mm={np.round(err * 1000, 1)}')
            if np.linalg.norm(err) < tol:
                return
            self._command(self.cmd_world + 0.9 * err)
        raise TrialInvalid(f'grasp alignment did not converge (error {np.round(err * 1000, 1)} mm)')

    def _wait_settled(self, t_end: float, speed_tol: float = 0.03, hold_s: float = 0.6) -> None:
        t_ok = None
        while True:
            self._check_px4()
            self._update_offset()
            self._command(self.cmd_world)
            lp = self.offboard.vehicle_local_position
            speed = math.sqrt(lp.vx ** 2 + lp.vy ** 2 + lp.vz ** 2)
            now = self.gz.sim_time()
            if speed < speed_tol:
                t_ok = t_ok if t_ok is not None else now
                if now - t_ok >= hold_s:
                    return
            else:
                t_ok = None
            if now > t_end:
                raise TrialInvalid(f'vehicle did not settle (speed {speed:.3f} m/s)')
            self.gz.sleep_sim(0.04)

    # -- session -------------------------------------------------------------

    def _grip_open(self) -> float:
        g = self.cfg['gripper']
        return g['claw']['open_cmd_rad'] if gripper_type(self.cfg) == 'servo_claw' else g['open_force_n']

    def takeoff_to_station(self) -> None:
        self._armed_once = False
        self.gz.set_grip(self._grip_open())
        self.gz.set_arm_pose(self.cfg['arm']['stow_pose'])
        start = self.gz.state().drone_pos.copy()
        self._command(start)
        self.offboard.start_heartbeat()        # arms + engages offboard after warm-up
        # Preflight checks (e.g. EKF heading innovation right after the EKF
        # restart for mocap) can reject the first attempts - keep retrying.
        deadline = time.monotonic() + 45.0
        while True:
            try:
                self.offboard.wait_until_armed_and_offboard(timeout_s=2.0)
                break
            except TimeoutError as e:
                if time.monotonic() > deadline:
                    raise SessionAbort(f'could not arm in offboard: {e} '
                                       f'(see `utility/px4.sh commander check`)')
                self.offboard.engage_offboard_mode()
                self.offboard.arm()
        self._armed_once = True
        self.session_t0 = time.time()
        self.home_world = start
        time.sleep(1.0)                        # logger opens its file on arming
        self.ulog_path = self._find_ulog()
        print('[exp] armed in offboard, taking off')
        self.fly_drone_to(start + [0, 0, self.m['cruise_alt_m'] - start[2]], 0.8)
        self.gz.set_arm_pose(self.cfg['arm']['reach_pose'])
        self.gz.sleep_sim(2.0)
        station = self._target_for('A')
        self.fly_drone_to([station[0], station[1], self.m['cruise_alt_m']],
                          self.m['transit_speed_mps'], tol=0.10)

    def land_home(self) -> None:
        try:
            self.gz.set_grip(self._grip_open())
            self.fly_drone_to([self.cmd_world[0], self.cmd_world[1], self.m['cruise_alt_m']],
                              self.m['transit_speed_mps'], tol=0.15)
            self.gz.set_arm_pose(self.cfg['arm']['stow_pose'])
            self.fly_drone_to([self.home_world[0], self.home_world[1], self.m['cruise_alt_m']],
                              self.m['transit_speed_mps'], tol=0.15)
        except Exception as e:  # still try to land
            print(f'[exp] return-home problem: {e}')
        # Stop streaming offboard setpoints first: they share the uORB
        # trajectory_setpoint topic with AUTO_LAND and would keep the
        # land detector from ever triggering.
        self.offboard.shutdown()
        time.sleep(0.2)
        self.offboard.land()
        try:
            self.offboard.wait_until_landed(timeout_s=60.0)
            print('[exp] landed and disarmed')
        except TimeoutError as e:
            print(f'[exp] {e}')

    def archive_ulog(self) -> None:
        """Copy the session ULog next to the trial data (PX4 may rotate its
        own copy away). The logger closes the file on disarm."""
        import shutil
        if not self.ulog_path or not os.path.exists(self.ulog_path):
            print('[exp] warning: session ULog not found - drone-side data unavailable')
            return
        time.sleep(2.0)
        dst = os.path.join(self.out, 'flight.ulg')
        shutil.copy2(self.ulog_path, dst)
        print(f'[exp] ULog archived: {dst} ({os.path.getsize(dst) / 1e6:.1f} MB)')
        self.ulog_path = dst

    def _find_ulog(self) -> str:
        logs = glob.glob(os.path.join(os.path.expanduser(self.cfg['px4']['log_dir']), '*', '*.ulg'))
        logs = [p for p in logs if os.path.getmtime(p) >= self.session_t0 - 10]
        return max(logs, key=os.path.getmtime) if logs else ''

    # -- one trial ----------------------------------------------------------

    def _target_for(self, scenario: str, offset_jitter: float = 0.0) -> np.ndarray:
        c = self.cfg['cylinder']
        centre = np.array(c['start_pose'][:3], dtype=float)
        axis = np.array(c['axis_world'], dtype=float)
        off = self.cfg['scenarios'][scenario]['grasp_offset_m'] + offset_jitter
        return centre + axis * off + np.array([0, 0, self.m['grasp_z_offset_m']])

    def _unjam_claw(self) -> bool:
        """After the open command a cylinder can stay wedged between the claw's jaws (pad
        force remains, jaws do not open). It is reset to its jig before the next trial
        anyway, so lift it out (park pose) and let the jaws open; otherwise the closed
        jaws hit it on the next descent. Returns True if it had stuck."""
        s = self.gz.state()
        open_q = self._grip_open()
        stuck = (s.fsr_l + s.fsr_r) > 0.2 or min(s.joints.get('jaw_left_joint', open_q),
                                                 s.joints.get('jaw_right_joint', open_q)) > open_q + 0.15
        if stuck:
            pose = self.cfg['cylinder']['park_pose']
            self.gz.set_model_pose(self.cfg['models']['cylinder'], pose[:3], pose[3:])
            self.gz.set_grip(open_q)
            self.gz.sleep_sim(0.8)
        return stuck

    def _place_cylinder(self, scenario: str) -> None:
        c = self.cfg['cylinder']
        pose = c['park_pose'] if scenario == 'E' else c['start_pose']
        for _ in range(3):
            if not self.gz.set_model_pose(self.cfg['models']['cylinder'], pose[:3], pose[3:]):
                continue
            self.gz.sleep_sim(0.5)
            if np.linalg.norm(self.gz.state().cyl_pos - np.array(pose[:3])) < 0.003:
                return
        raise TrialInvalid('cylinder did not settle at its start pose')

    def _classify_lift(self, t_lift: float, scen: str) -> Outcome:
        """Label y_success from the recorded rows of the current trial."""
        if scen == 'E':
            return Outcome('empty')
        col = TRIAL_COLUMNS.index
        win = [r for r in self.recorder.rows if r[col('phase')] in OUTCOME_PHASES[self.variant]]
        o = self.cfg['outcome']
        return classify_outcome(
            [r[col('t_sim')] for r in win],
            [r[col('rel_disp')] for r in win],
            [r[col('tilt_change_deg')] for r in win],
            t_lift, o['slip_onset_m'], o['drop_m'], o['tilt_threshold_deg'],
            o.get('tilt_fail_deg', 45.0))

    def _lift_hold_shake(self, row: dict, scen: str):
        """Small-lift variant, from the closed gripper to the end of the shake.

        small_lift (t0 .. t0 + sl_duration) -> lift (rest of the rise) -> hold
        -> y_success; if the grasp held, shake -> y_stable. Fills the timing and
        label columns of `row`; returns (outcome, still_holding)."""
        rec, m = self.recorder, self.m
        sl, sh, o = self.cfg['small_lift'], scenario_shake(self.cfg, scen), self.cfg['outcome']

        rec.phase = 'small_lift'
        row['t_lift'] = self.gz.sim_time()                      # t0
        self._run_min_jerk([0, 0, sl['sl_height_m']], sl['sl_duration_s'])
        row['t_sl_end'] = self.gz.sim_time()

        rec.phase = 'lift'
        rest = sl['full_lift_height_m'] - sl['sl_height_m']
        self._run_min_jerk([0, 0, rest],
                           duration_for_peak_speed(rest, sl['full_lift_peak_speed_mps']))
        rec.phase = 'hold'
        self.gz.sleep_sim(m['hold_s'])

        outcome = self._classify_lift(row['t_lift'], scen)
        if outcome.outcome in ('success', 'fail'):
            row['y_success'] = int(outcome.outcome == 'success')
        if outcome.outcome != 'success':
            return outcome, False

        rec.phase = 'shake'
        row['t_shake'] = self.gz.sim_time()
        col = TRIAL_COLUMNS.index

        def stability():
            win = [r for r in rec.rows if r[col('phase')] == 'shake']
            return classify_stability(
                [r[col('t_sim')] for r in win],
                [[r[col('rel_dx')], r[col('rel_dy')], r[col('rel_dz')]] for r in win],
                [r[col('tilt_change_deg')] for r in win],
                row['t_shake'], o['slip_onset_m'], o['drop_m'], o['tilt_threshold_deg'],
                o.get('tilt_fail_deg', 45.0))

        if sh.get('mode', 'per_scenario') == 'step_stress':
            # Same escalating shake for every held grasp: one level per leg duration,
            # gentle to harsh; stop at the first level the grasp does not survive.
            # stable_level = levels survived; y_stable = survived at least min_level.
            ss = sh['step_stress']
            level, st = 0, None
            for leg_s in ss['levels_leg_s']:
                for delta, duration in shake_legs(sh['amplitude_m'], sh['sequence'],
                                                  leg_s, sh['axis_world']):
                    self._run_min_jerk(delta, duration)
                self.gz.sleep_sim(sh['settle_s'])
                st = stability()
                if st.outcome != 'success':
                    break
                level += 1
            row['stable_level'] = level
            row['y_stable'] = int(level >= ss['y_stable_min_level'])
            row['shake_failure_mode'] = st.failure_mode
            row['shake_t_drop_s'] = st.t_drop_s
            return outcome, st.outcome == 'success'

        for delta, duration in shake_legs(sh['amplitude_m'], sh['sequence'],
                                          sh['leg_duration_s'], sh['axis_world']):
            self._run_min_jerk(delta, duration)
        self.gz.sleep_sim(sh['settle_s'])
        st = stability()
        row['y_stable'] = int(st.outcome == 'success')
        row['shake_failure_mode'] = st.failure_mode
        row['shake_t_drop_s'] = st.t_drop_s
        return outcome, st.outcome == 'success'

    def run_trial(self, spec: TrialSpec) -> dict:
        cfg, m, g = self.cfg, self.m, self.cfg['gripper']
        scen = spec.scenario
        sc = cfg['scenarios'][scen]
        rz = cfg['randomization']
        grip_nominal = (spec.grip_force_n if spec.grip_force_n is not None
                        else scenario_grip_force(cfg, scen))
        grip = jittered_grip(cfg, grip_nominal, self.rng)
        jitter = 0.0 if scen == 'E' else float(rz['grasp_offset_jitter_m'] * self.rng.standard_normal())
        target = self._target_for(scen, jitter)
        above = target + [0, 0, m['approach_clearance_m']]
        row = dict(index=spec.index, battery=spec.battery, slot=spec.slot, scenario=scen,
                   grip_force_nominal_n=round(grip_nominal, 4), grip_force_n=round(grip, 4),
                   grasp_offset_cmd_m=round(sc['grasp_offset_m'] + jitter, 5),
                   grasp_offset_actual_m=float('nan'), valid=True, invalid_reason='',
                   t_start=float('nan'), t_lift=float('nan'), t_end=float('nan'), csv='')
        if self.variant == 'small_lift':
            # t_lift is t0, the start of the small lift. y_* are '' when not applicable
            # (E, invalid trial, or y_stable after a failed grasp).
            row.update(t_close=float('nan'), t_sl_end=float('nan'), t_shake=float('nan'),
                       y_success='', y_stable='', stable_level='', release_stuck='', shake_failure_mode='',
                       shake_t_drop_s=float('nan'))
        rec = self.recorder
        rec.grip = self._grip_open()
        self.gz.set_grip(rec.grip)
        outcome = Outcome('invalid')
        held = False
        try:
            # Reset: gripper up at the approach height, cylinder at its start pose.
            self.move_grasp_to(above, m['transit_speed_mps'], fine=False)
            self._place_cylinder(scen)
            self.move_grasp_to(above, m['descend_speed_mps'])

            rec.start()
            row['t_start'] = self.gz.sim_time()
            rec.phase = 'baseline'
            self.gz.sleep_sim(m['baseline_s'])

            rec.phase = 'descend'
            self.move_grasp_to(target, m['descend_speed_mps'])
            rec.phase = 'settle'
            self.gz.sleep_sim(m['settle_s'])

            rec.phase = 'close'
            rec.grip = grip
            if self.variant == 'small_lift':
                row['t_close'] = self.gz.sim_time()
            self.gz.set_grip(grip)
            self.gz.sleep_sim(m['close_wait_s'])
            s = self.gz.state()
            claw = gripper_type(cfg) == 'servo_claw'
            fl = s.joints.get('jaw_left_joint' if claw else 'finger_left_joint', 0)
            fr = s.joints.get('jaw_right_joint' if claw else 'finger_right_joint', 0)
            axis = np.array(cfg['cylinder']['axis_world'], dtype=float)
            if scen != 'E':
                row['grasp_offset_actual_m'] = round(float((s.grasp_point - s.cyl_pos) @ axis), 5)
                # Claw: the pads must feel the cylinder. The jaw angle alone cannot tell a soft
                # command that stopped at the cylinder from one that closed on nothing.
                if claw and (s.fsr_l + s.fsr_r) < g['claw']['missed_grasp_force_n']:
                    raise TrialInvalid(f'missed grasp (pad force {s.fsr_l:.2f}/{s.fsr_r:.2f} N, '
                                       f'jaws at {fl:.3f}/{fr:.3f} rad)')
                if not claw and max(fl, fr) > g['missed_grasp_travel_m']:
                    raise TrialInvalid(f'missed grasp (fingers at {fl * 1000:.1f}/{fr * 1000:.1f} mm)')
            rec.set_reference(s.cyl_pos - s.grasp_point, axis_tilt_deg(s.cyl_q))

            if self.variant == 'small_lift':
                outcome, held = self._lift_hold_shake(row, scen)
            else:
                rec.phase = 'lift'
                row['t_lift'] = self.gz.sim_time()
                self.move_grasp_to(target + [0, 0, m['lift_height_m']], m['lift_speed_mps'],
                                   fine=False)
                rec.phase = 'hold'
                self.gz.sleep_sim(m['hold_s'])
                outcome = self._classify_lift(row['t_lift'], scen)
                held = outcome.outcome == 'success'

            rec.phase = 'release'
            # Put a held cylinder back down gently instead of dropping it. The small_lift
            # variant skips this by default: the cylinder is teleported to its jig before
            # every trial anyway, and after a 1 m lift the descent leaves the arm swinging,
            # which made the next trial's alignment fail.
            if held and (self.variant == 'legacy'
                         or self.cfg['small_lift'].get('put_down', False)):
                self.move_grasp_to(target + [0, 0, 0.01], m['lift_speed_mps'], fine=False)
                self.gz.sleep_sim(0.5)
            rec.grip = self._grip_open()
            self.gz.set_grip(rec.grip)
            self.gz.sleep_sim(0.5)
            if gripper_type(cfg) == 'servo_claw':
                row['release_stuck'] = int(self._unjam_claw())
        except TrialInvalid as e:
            row['valid'] = False
            row['invalid_reason'] = str(e)
            outcome = Outcome('invalid')
            self.gz.set_grip(self._grip_open())
        finally:
            rows = rec.stop()
            row['t_end'] = self.gz.sim_time()

        row.update({k: (round(v, 4) if isinstance(v, float) else v)
                    for k, v in asdict(outcome).items()})
        if rows:
            name = f't{spec.index:04d}_{scen}.csv'
            with open(os.path.join(self.out, 'trials', name), 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(TRIAL_COLUMNS)
                w.writerows(rows)
            row['csv'] = f'trials/{name}'
        # back up to the approach height for the next reset
        try:
            self.move_grasp_to(above, m['lift_speed_mps'], fine=False)
        except TrialInvalid:
            pass
        return row

    # -- bookkeeping -----------------------------------------------------------

    def write_session(self, plan: List[TrialSpec], mode: str, params: dict, extra: dict) -> None:
        cfg = {k: v for k, v in self.cfg.items() if not k.startswith('_')}
        meta = dict(session=self.session, mode=mode, config_file=self.cfg.get('_path'),
                    variant=self.variant, git_commit=_git_commit(),
                    config_sha256=hashlib.sha256(
                        json.dumps(cfg, sort_keys=True, default=str).encode()).hexdigest(),
                    config=cfg, px4_params=params, ulog=self.ulog_path,
                    px4_time_offset_s=0.0,
                    note='PX4 timestamps equal Gazebo sim time in SITL lockstep',
                    plan=[asdict(p) for p in plan], started=time.strftime('%Y-%m-%d %H:%M:%S'),
                    **extra)
        with open(os.path.join(self.out, 'session.json'), 'w') as f:
            json.dump(meta, f, indent=2)

    def append_summary(self, row: dict) -> None:
        if self._summary is None:
            self._summary_file = open(os.path.join(self.out, 'trials.csv'), 'w', newline='')
            self._summary = csv.DictWriter(self._summary_file, fieldnames=list(row))
            self._summary.writeheader()
        self._summary.writerow(row)
        self._summary_file.flush()
        self.results.append(row)


def _git_commit() -> str:
    """Commit of the code that produced a session ('+dirty' if it has local changes)."""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        sha = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=here, text=True,
                             capture_output=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', 'status', '--porcelain', '--', here], cwd=here, text=True,
                               capture_output=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''
    return sha + ('+dirty' if sha and dirty else '')


def relabel_session(session_dir: str, cfg: dict) -> List[dict]:
    """Recompute outcome columns of trials.csv from the per-trial ground
    truth with the current `outcome` thresholds (e.g. after changing them)."""
    path = os.path.join(session_dir, 'trials.csv')
    with open(path) as f:
        rows = list(csv.DictReader(f))
    o = cfg['outcome']
    small_lift = 'y_success' in rows[0]          # session written by the small_lift variant
    phases = OUTCOME_PHASES['small_lift' if small_lift else 'legacy']
    out = []
    for r in rows:
        if r['scenario'] != 'E' and r['valid'] == 'True' and r['csv']:
            with open(os.path.join(session_dir, r['csv'])) as f:
                gz = list(csv.DictReader(f))
            win = [g for g in gz if g['phase'] in phases]
            res = classify_outcome([float(g['t_sim']) for g in win],
                                   [float(g['rel_disp']) for g in win],
                                   [float(g['tilt_change_deg']) for g in win],
                                   float(r['t_lift']), o['slip_onset_m'], o['drop_m'],
                                   o['tilt_threshold_deg'], o.get('tilt_fail_deg', 45.0))
            r.update({k: (round(v, 4) if isinstance(v, float) else v)
                      for k, v in asdict(res).items()})
            if small_lift:
                r['y_success'] = int(res.outcome == 'success')
                shake = [g for g in gz if g['phase'] == 'shake']
                if res.outcome == 'success' and shake:
                    st = classify_stability(
                        [float(g['t_sim']) for g in shake],
                        [[float(g['rel_dx']), float(g['rel_dy']), float(g['rel_dz'])]
                         for g in shake],
                        [float(g['tilt_change_deg']) for g in shake],
                        float(r['t_shake']), o['slip_onset_m'], o['drop_m'],
                        o['tilt_threshold_deg'], o.get('tilt_fail_deg', 45.0))
                    r.update(y_stable=int(st.outcome == 'success'),
                             shake_failure_mode=st.failure_mode,
                             shake_t_drop_s=round(st.t_drop_s, 4))
                else:
                    r.update(y_stable='', shake_failure_mode='', shake_t_drop_s='nan')
        out.append(r)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(out)
    for r in out:   # types for print_report
        r['valid'] = r['valid'] in (True, 'True')
        r['grip_force_nominal_n'] = float(r['grip_force_nominal_n'])
        r['t_drop_s'] = float(r['t_drop_s'])
    return out


def print_report(results: List[dict]) -> None:
    from collections import defaultdict
    by = defaultdict(list)
    for r in results:
        by[(r['scenario'], r['grip_force_nominal_n'])].append(r)
    print('\nscenario  grip    trials  valid  fail  fail%   slide/tilt  mean t_drop[s]')
    for (scen, grip), rs in sorted(by.items()):
        v = [r for r in rs if r['valid']]
        f = [r for r in v if r['outcome'] == 'fail']
        modes = f"{sum(r['failure_mode'] == 'slide' for r in f)}/{sum(r['failure_mode'] == 'tilt' for r in f)}"
        td = [r['t_drop_s'] for r in f if not math.isnan(r['t_drop_s'])]
        pct = f'{100 * len(f) / len(v):5.1f}' if v and scen != 'E' else '   - '
        print(f'{scen:^8}  {grip:6.2f}  {len(rs):6d}  {len(v):5d}  {len(f):4d}  {pct}  '
              f'{modes:>10}  {np.mean(td) if td else float("nan"):8.2f}')


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--config', help='experiment.yaml (default: installed package config)')
    common.add_argument('--output-dir', default='sim_data')
    common.add_argument('--grip', nargs='+', default=[],
                        help='protocol/single: SCEN=N overrides, e.g. A=0.8 B=2.0; '
                             'tune: grip forces [N] to sweep')
    common.add_argument('--seed', type=int, help='override randomization.seed')
    common.add_argument('--variant', choices=VARIANTS,
                        help='override protocol.variant (legacy | small_lift)')
    common.add_argument('--no-land', action='store_true',
                        help='stay airborne at the end (PX4 params are then NOT restored)')
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    sub = p.add_subparsers(dest='mode', required=True)
    pr = sub.add_parser('protocol', parents=[common], help='interleaved A/B batteries with E controls')
    pr.add_argument('--batteries', type=int)
    pr.add_argument('--empty', type=int, help='override protocol.empty_per_battery '
                                              '(0 = no E controls)')
    tu =sub.add_parser('tune', parents=[common], help='sweep grip force for one scenario')
    tu.add_argument('--scenario', choices=('A', 'B'), required=True)
    tu.add_argument('--trials', type=int, default=10, help='trials per grip level')
    si = sub.add_parser('single', parents=[common], help='one trial')
    si.add_argument('--scenario', choices=SCENARIOS, required=True)
    rl = sub.add_parser('relabel', parents=[common],
                        help='recompute outcomes of a session with the current thresholds')
    rl.add_argument('session')
    return p


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    import signal
    # Ctrl+C / kill -> KeyboardInterrupt -> fly home and land (see the
    # finally block). Set explicitly: a process started in the background
    # from a script inherits SIGINT as *ignored*.
    signal.signal(signal.SIGINT, _interrupt)
    signal.signal(signal.SIGTERM, _interrupt)
    args = build_parser().parse_args(argv)
    cfg = load_experiment_config(args.config)
    if args.seed is not None:
        cfg['randomization']['seed'] = args.seed
    if args.variant is not None:
        cfg['protocol']['variant'] = args.variant

    if args.mode == 'relabel':
        print_report(relabel_session(args.session, cfg))
        return 0

    if args.mode == 'tune':
        levels = [float(x) for x in args.grip]
        if not levels:
            print('tune needs --grip F1 F2 ... (newtons)', file=sys.stderr)
            return 2
        plan = [TrialSpec(i * args.trials + k, i + 1, k + 1, args.scenario, lvl)
                for i, lvl in enumerate(levels) for k in range(args.trials)]
        label = f'tune{args.scenario}'
    else:
        for kv in args.grip:
            scen, val = kv.split('=')
            set_scenario_grip_force(cfg, scen, float(val))
        if args.mode == 'single':
            plan = [TrialSpec(0, 1, 1, args.scenario)]
            label = f'single{args.scenario}'
        else:
            pc = cfg['protocol']
            empty = pc['empty_per_battery'] if args.empty is None else args.empty
            plan = generate_protocol_plan(args.batteries or pc['batteries'], pc['trials_per_battery'],
                                          empty, cfg['randomization']['seed'])
            label = 'protocol'

    exp = Experiment(cfg, args.output_dir, label)
    print(f'[exp] session {exp.out}: {len(plan)} trials')
    params = exp._set_px4_params()
    try:
        exp.connect()
        exp.takeoff_to_station()
        exp.write_session(plan, args.mode, params, {})
        for spec in plan:
            t0 = time.monotonic()
            row = exp.run_trial(spec)
            exp.append_summary(row)
            detail = row['invalid_reason'] if not row['valid'] else (
                f"{row['failure_mode']} t_drop={row['t_drop_s']:.2f}s" if row['outcome'] == 'fail'
                else ('slipped' if row['slipped'] else ''))
            if row.get('y_stable', '') != '':
                detail += f" {'stable' if row['y_stable'] else 'unstable'}"
            print(f"[exp] {spec.index + 1:4d}/{len(plan)} batt {spec.battery:2d} {spec.scenario} "
                  f"grip {row['grip_force_n']:.3f} {'rad' if gripper_type(cfg) == 'servo_claw' else 'N'} -> {row['outcome']:7s} {detail} "
                  f"({time.monotonic() - t0:.0f}s)", flush=True)
    except KeyboardInterrupt:
        print('\n[exp] interrupted - returning home')
    except SessionAbort as e:
        print(f'[exp] ABORT: {e}')
    except (TrialInvalid, TimeoutError, RuntimeError) as e:   # e.g. during takeoff
        print(f'[exp] ABORT: {type(e).__name__}: {e}')
    finally:
        if getattr(exp, '_armed_once', False) and not args.no_land:
            exp.land_home()
            exp.archive_ulog()
        if not args.no_land:
            exp.restore_px4()
        if exp.results:
            exp.write_session(plan, args.mode, params, {'finished': time.strftime('%Y-%m-%d %H:%M:%S'),
                                                        'completed_trials': len(exp.results)})
            print_report(exp.results)
        if hasattr(exp, 'node'):
            exp.disconnect()
        print(f'[exp] data: {exp.out}')
    return 0


if __name__ == '__main__':
    code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    # gz-transport's C++ threads abort on normal interpreter teardown
    # ("terminate called without an active exception"); skip it.
    os._exit(code)
