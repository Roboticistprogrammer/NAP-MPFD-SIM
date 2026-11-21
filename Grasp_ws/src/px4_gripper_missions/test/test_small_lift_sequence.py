"""Drive Experiment._lift_hold_shake against a fake clock and fake Gazebo:
checks the commanded motion, phase order, timing and labels of the small-lift
variant without a simulator."""
import os
from types import SimpleNamespace

import numpy as np
import pytest

from px4_gripper_missions import experiment as ex

SRC_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')


class FakeGz:
    """Sim clock that advances only when the code sleeps; feeds the recorder at 50 Hz."""

    def __init__(self, exp, cyl_offset):
        self.t = 100.0
        self.exp = exp
        self.cyl_offset = cyl_offset        # (t, phase) -> (dx, dy, dz, tilt_deg)

    def sim_time(self):
        return self.t

    def sleep_sim(self, dt):
        self.advance(dt)

    def advance(self, dt):
        end = self.t + dt
        rec = self.exp.recorder
        while self.t < end - 1e-9:
            self.t += min(0.02, end - self.t)
            if rec.active:
                dx, dy, dz, tilt = self.cyl_offset(self.t, rec.phase)
                row = dict.fromkeys(ex.TRIAL_COLUMNS, 0.0)
                row.update(t_sim=self.t, phase=rec.phase, rel_dx=dx, rel_dy=dy, rel_dz=dz,
                           rel_disp=float(np.linalg.norm([dx, dy, dz])), tilt_change_deg=tilt)
                rec.rows.append([row[c] for c in ex.TRIAL_COLUMNS])


def make_exp(monkeypatch, tmp_path, cyl_offset):
    cfg = ex.load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    cfg['protocol']['variant'] = 'small_lift'
    exp = ex.Experiment(cfg, str(tmp_path), 'seq')
    exp.gz = FakeGz(exp, cyl_offset)
    commands = []

    def command(p, v=None):
        exp.cmd_world = np.asarray(p, dtype=float)
        commands.append((exp.gz.t, exp.cmd_world.copy()))

    exp.cmd_world = np.zeros(3)
    exp._command = command
    exp._check_px4 = lambda: None
    exp._update_offset = lambda: None
    monkeypatch.setattr(ex, 'time', SimpleNamespace(sleep=lambda dt: exp.gz.advance(dt)))
    exp.recorder.start()
    row = dict(t_lift=float('nan'), t_sl_end=float('nan'), t_shake=float('nan'),
               y_success='', y_stable='', shake_failure_mode='', shake_t_drop_s=float('nan'))
    return exp, row, commands


def phases_in_order(exp):
    seen = []
    for r in exp.recorder.rows:
        p = r[ex.TRIAL_COLUMNS.index('phase')]
        if not seen or seen[-1] != p:
            seen.append(p)
    return seen


def test_held_and_stable_trial(monkeypatch, tmp_path):
    exp, row, commands = make_exp(monkeypatch, tmp_path, lambda t, ph: (0, 0, 0, 0))
    outcome, holding = exp._lift_hold_shake(row, 'A')

    assert outcome.outcome == 'success' and holding
    assert row['y_success'] == 1 and row['y_stable'] == 1
    assert phases_in_order(exp) == ['small_lift', 'lift', 'hold', 'shake']
    # t0 is the start of the small lift; it lasts 1.5 s of sim time
    assert row['t_lift'] == pytest.approx(100.0)
    assert row['t_sl_end'] - row['t_lift'] == pytest.approx(
        exp.cfg['small_lift']['sl_duration_s'], abs=0.04)
    # the drone has risen sl_height_m at the end of the SL window ...
    sl_h = exp.cfg['small_lift']['sl_height_m']
    z_at_sl_end = next(p[2] for t, p in commands if t >= row['t_sl_end'] - 1e-6)
    assert z_at_sl_end == pytest.approx(sl_h, abs=1e-6)
    # ... and 1.0 m in total by the hold, and the shake returns to the hold position
    z_hold = next(p[2] for t, p in commands if t >= row['t_sl_end'] + 5.7)
    assert z_hold == pytest.approx(1.0, abs=1e-3)
    assert commands[-1][1] == pytest.approx([0, 0, 1.0], abs=1e-6)
    # the shake went +0.3 m then -0.3 m along world Y
    ys = [p[1] for t, p in commands if t >= row['t_shake']]
    assert max(ys) == pytest.approx(0.3, abs=1e-6) and min(ys) == pytest.approx(-0.3, abs=1e-6)


def test_cylinder_lost_in_shake_is_unstable_and_not_held(monkeypatch, tmp_path):
    def cyl(t, phase):                       # slides 10 cm out of the grasp during the shake
        return (0, 0.1 * min(1.0, (t - 114.0) / 1.0), 0, 0) if phase == 'shake' else (0, 0, 0, 0)
    exp, row, _ = make_exp(monkeypatch, tmp_path, cyl)
    outcome, holding = exp._lift_hold_shake(row, 'B')

    assert outcome.outcome == 'success' and not holding     # it survived the lift...
    assert row['y_success'] == 1 and row['y_stable'] == 0   # ...but not the shake
    assert row['shake_failure_mode'] == 'slide'
    assert row['shake_t_drop_s'] > 0


def test_failed_grasp_skips_the_shake(monkeypatch, tmp_path):
    def cyl(t, phase):                       # falls out during the lift
        return (0, 0, -0.2 * min(1.0, max(0.0, (t - 101.0))), 0)
    exp, row, _ = make_exp(monkeypatch, tmp_path, cyl)
    outcome, holding = exp._lift_hold_shake(row, 'A')

    assert outcome.outcome == 'fail' and not holding
    assert row['y_success'] == 0 and row['y_stable'] == ''
    assert 'shake' not in phases_in_order(exp)
    assert np.isnan(row['t_shake'])


def test_empty_control_runs_the_lift_but_has_no_labels(monkeypatch, tmp_path):
    exp, row, _ = make_exp(monkeypatch, tmp_path, lambda t, ph: (0, 0, 0, 0))
    outcome, holding = exp._lift_hold_shake(row, 'E')

    assert outcome.outcome == 'empty' and not holding
    assert row['y_success'] == '' and row['y_stable'] == ''
    assert phases_in_order(exp) == ['small_lift', 'lift', 'hold']


def test_velocity_feedforward_sends_the_profile_velocity(monkeypatch, tmp_path):
    exp, row, _ = make_exp(monkeypatch, tmp_path, lambda t, ph: (0, 0, 0, 0))
    exp.cfg['small_lift']['velocity_feedforward'] = True
    sent = []

    def command(p, v=None):
        exp.cmd_world = np.asarray(p, dtype=float)
        sent.append((exp.gz.t, None if v is None else np.asarray(v, dtype=float)))
    exp._command = command

    exp._run_min_jerk([0, 0, 0.10], 1.5)

    assert all(v is not None for _, v in sent)
    vz = np.array([v[2] for _, v in sent])
    assert vz.max() == pytest.approx(1.875 * 0.10 / 1.5, rel=0.05)      # peak 0.125 m/s
    assert abs(vz[-1]) < 1e-9                                            # ends at rest
    assert np.allclose([v[0] for _, v in sent], 0.0)


def _slides_out_after(shake_elapsed_s):
    """Cylinder slides 10 cm out once the shake has lasted shake_elapsed_s."""
    start = {}

    def cyl(t, phase):
        if phase != 'shake':
            return (0, 0, 0, 0)
        start.setdefault('t', t)
        e = t - start['t'] - shake_elapsed_s
        return (0, 0.1 * min(1.0, max(0.0, e)), 0, 0)
    return cyl


def test_step_stress_counts_the_levels_survived(monkeypatch, tmp_path):
    exp, row, _ = make_exp(monkeypatch, tmp_path, _slides_out_after(1e9))
    ss = exp.cfg['shake']['step_stress']
    exp.cfg['shake']['mode'] = 'step_stress'
    outcome, holding = exp._lift_hold_shake(row, 'A')
    assert holding and row['stable_level'] == len(ss['levels_leg_s']) and row['y_stable'] == 1


@pytest.mark.parametrize('elapsed, expect_level', [(3.0, 0), (17.5, 2)])
def test_step_stress_stops_at_the_first_level_not_survived(monkeypatch, tmp_path, elapsed, expect_level):
    exp, row, _ = make_exp(monkeypatch, tmp_path, _slides_out_after(elapsed))
    sh = exp.cfg['shake']
    sh['mode'] = 'step_stress'
    legs = sh['step_stress']['levels_leg_s']
    # level k lasts 3 legs x leg time + settle; 17.5 s falls inside level 3 of the default ladder
    assert sum(3 * l + sh['settle_s'] for l in legs[:2]) < 17.5 < sum(3 * l + sh['settle_s'] for l in legs[:3])
    outcome, holding = exp._lift_hold_shake(row, 'B')
    assert outcome.outcome == 'success' and not holding
    assert row['stable_level'] == expect_level
    assert row['y_stable'] == int(expect_level >= sh['step_stress']['y_stable_min_level'])
    assert row['shake_failure_mode'] == 'slide'
