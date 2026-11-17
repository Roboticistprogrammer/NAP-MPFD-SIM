"""Tests for the small-lift variant: stability labels, config, window dataset, relabel
(no ROS graph, Gazebo, PX4 or real ULog needed)."""
import csv
import json
import math
import os

import numpy as np
import pytest

from px4_gripper_missions import windows
from px4_gripper_missions.experiment import (
    TRIAL_COLUMNS,
    Experiment,
    classify_stability,
    generate_protocol_plan,
    load_experiment_config,
    relabel_session,
    scenario_grip_force,
    set_scenario_grip_force,
)

SRC_CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
KW = dict(slip_onset_m=0.010, drop_m=0.040, tilt_threshold_deg=15.0)


# -- stability label ----------------------------------------------------------

def _shake(d_axis, tilt=None, dt=0.02, t0=20.0):
    n = len(d_axis)
    t = t0 + dt * np.arange(n)
    d = np.zeros((n, 3))
    d[:, 1] = d_axis
    return t, d, np.zeros(n) if tilt is None else np.asarray(tilt, float)


def test_unmoved_cylinder_is_stable():
    t, d, tilt = _shake([0.001] * 300)
    assert classify_stability(t, d, tilt, 20.0, **KW).outcome == 'success'


def test_offset_before_the_shake_does_not_count():
    # It had already slipped 25 mm during the lift; only motion DURING the shake matters.
    t, d, tilt = _shake([0.025] * 300, [10.0] * 300)
    o = classify_stability(t, d, tilt, 20.0, **KW)
    assert o.outcome == 'success' and not o.slipped


def test_sliding_out_during_the_shake_is_unstable():
    t, d, tilt = _shake([0.0] * 100 + list(np.linspace(0, 0.1, 50)) + [0.1] * 150)
    o = classify_stability(t, d, tilt, 20.0, **KW)
    assert o.outcome == 'fail' and o.failure_mode == 'slide'
    assert 2.0 < o.t_drop_s < 3.5


def test_rotating_out_during_the_shake_is_unstable_tilt():
    t, d, _ = _shake([0.0] * 300)
    tilt = [0.0] * 100 + list(np.linspace(0, 80, 100)) + [80.0] * 100
    o = classify_stability(t, d, tilt, 20.0, **KW)
    assert o.outcome == 'fail' and o.failure_mode == 'tilt'


def test_empty_shake_window_is_invalid():
    assert classify_stability([], np.zeros((0, 3)), [], 20.0, **KW).outcome == 'invalid'


# -- protocol plan and config -------------------------------------------------

def test_plan_without_empty_controls_is_all_ab():
    plan = generate_protocol_plan(batteries=1, per_battery=10, empty_per_battery=0, seed=1)
    assert len(plan) == 10
    assert {t.scenario for t in plan} == {'A', 'B'}


def test_shipped_config_defaults_to_legacy_and_defines_small_lift():
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    assert cfg['protocol']['variant'] == 'legacy'
    sl, sh = cfg['small_lift'], cfg['shake']
    assert sl['sl_height_m'] == pytest.approx(0.15)
    assert sl['sl_duration_s'] == pytest.approx(1.5)
    assert sl['full_lift_height_m'] == pytest.approx(1.0)
    assert sl['sl_duration_s'] * sl['window_rate_hz'] == pytest.approx(75)
    assert sh['amplitude_m'] == pytest.approx(0.3)
    assert sh['sequence'] == [1.0, -1.0, 0.0]
    assert sl['velocity_feedforward'] is True
    assert sh['leg_duration_s'] == pytest.approx(2.0)


def test_unknown_variant_is_rejected(tmp_path):
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    cfg['protocol']['variant'] = 'nonsense'
    with pytest.raises(ValueError):
        Experiment(cfg, str(tmp_path), 'x')


def test_channels_yaml_has_the_planned_inputs():
    spec = windows.load_channel_spec()
    names = [c['name'] for c in spec['inputs']]
    assert len(names) == len(set(names)) == 21
    # nothing the real drone cannot see, and no applied-force leakage
    assert 'grip_force_n' not in names and 'scenario' not in names
    assert set(spec['privileged']) >= {'grip_force_n', 'scenario', 'finger_l'}


# -- window extraction --------------------------------------------------------

def test_bin_average_means_each_bin():
    t = 0.0025 + 0.005 * np.arange(200)      # 200 Hz, samples away from the bin edges
    out = windows.bin_average(t, t.copy(), 0.0, 4, 0.1)
    assert np.allclose(out, [0.05, 0.15, 0.25, 0.35])


def test_bin_average_holds_last_value_for_slow_topics_and_nan_before_first_sample():
    t = np.array([0.0, 0.55])
    v = np.array([1.0, 2.0])
    out = windows.bin_average(t, v, -0.1, 5, 0.2)    # bins start at -0.1, 0.1, 0.3, 0.5, 0.7
    assert list(out) == [1.0, 1.0, 1.0, 2.0, 2.0]    # empty bins hold the earlier sample
    # before the first sample there is nothing to hold
    assert math.isnan(windows.bin_average(t, v, -0.5, 1, 0.2)[0])


def test_quaternion_to_roll_pitch():
    from px4_gripper_missions.geometry import rpy_to_quat
    q = np.array([rpy_to_quat(0.2, -0.1, 0.7)])
    roll, pitch = windows.quat_to_roll_pitch(q)
    assert roll[0] == pytest.approx(0.2) and pitch[0] == pytest.approx(-0.1)


# -- end-to-end dataset build on a synthetic session --------------------------

def _fake_series(spec):
    t = np.arange(0.0, 80.0, 0.004)
    return {c['name']: (t, np.sin(0.1 * i + t) * 0.01 + i) for i, c in enumerate(spec['inputs'])}


def _make_session(tmp_path):
    sess = tmp_path / 'sim_data' / '20261016_000000_protocol'
    sess.mkdir(parents=True)
    ulog = sess / 'flight.ulg'
    ulog.write_bytes(b'')
    (sess / 'session.json').write_text(json.dumps(dict(
        session=sess.name, variant='small_lift', ulog=str(ulog), px4_time_offset_s=0.0,
        git_commit='abc1234', config_sha256='f' * 64,
        config=dict(motion=dict(close_wait_s=1.0),
                    arm=dict(reach_pose=dict(base_yaw=0.0, shoulder_pitch=-1.45,
                                             elbow_pitch=-0.12))))))
    cols = ['index', 'battery', 'slot', 'scenario', 'valid', 'invalid_reason', 'grip_force_n',
            'grasp_offset_actual_m', 't_close', 't_lift', 't_sl_end', 't_shake', 'outcome',
            'failure_mode', 't_onset_s', 't_drop_s', 'y_success', 'y_stable',
            'shake_failure_mode', 'shake_t_drop_s']

    def row(i, scen, valid='True', ys='1', yst='1', t0=10.0):
        return dict(index=i, battery=1, slot=i + 1, scenario=scen, valid=valid,
                    invalid_reason='' if valid == 'True' else 'missed grasp',
                    grip_force_n=1.41, grasp_offset_actual_m=0.001, t_close=t0 - 1.1, t_lift=t0,
                    t_sl_end=t0 + 1.5, t_shake='nan', outcome='success', failure_mode='',
                    t_onset_s='nan', t_drop_s='nan', y_success=ys, y_stable=yst,
                    shake_failure_mode='', shake_t_drop_s='nan')
    rows = [row(0, 'A'), row(1, 'B', ys='0', yst='', t0=30.0), row(2, 'E', ys='', yst='', t0=50.0),
            row(3, 'A', valid='False', ys='', yst='', t0=60.0)]
    with open(sess / 'trials.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)
    return str(sess)


def test_build_windows_end_to_end(tmp_path, monkeypatch):
    spec = windows.load_channel_spec()
    monkeypatch.setattr(windows, 'load_ulog_channels', lambda path, s: _fake_series(s))
    sess = _make_session(tmp_path)

    meta = windows.build_windows(sess)

    assert meta['n_trials'] == 2                     # the A and the B; E and invalid left out
    assert [d['trial_id'] for d in meta['dropped']] == [f"{os.path.basename(sess)}_t0003"]
    data = np.load(os.path.join(sess, 'dataset', 'windows.npz'))
    c = len(spec['inputs'])
    assert data['X_sl'].shape == (2, 75, c)          # 1.5 s at 50 Hz
    assert data['X_cg'].shape == (2, 50, c)          # 1.0 s close phase
    assert data['base_sl'].shape == (2, c)
    assert not np.isnan(data['X_sl']).any()
    assert list(data['channels']) == [x['name'] for x in spec['inputs']]

    import pandas as pd
    labels = pd.read_parquet(os.path.join(sess, 'dataset', 'labels.parquet'))
    assert list(labels['scenario']) == ['A', 'B']
    assert list(labels['y_success']) == [1, 0]
    assert labels['y_stable'].iloc[0] == 1 and pd.isna(labels['y_stable'].iloc[1])
    assert labels['arm_shoulder_cmd'].iloc[0] == pytest.approx(-1.45)
    assert meta['git_commit'] == 'abc1234'


def test_sl_window_starts_exactly_at_t0(tmp_path, monkeypatch):
    spec = windows.load_channel_spec()
    series = _fake_series(spec)
    # make one channel a ramp in sim time so the window position is visible
    name = spec['inputs'][0]['name']
    t = series[name][0]
    series[name] = (t, t.copy())
    monkeypatch.setattr(windows, 'load_ulog_channels', lambda path, s: series)
    sess = _make_session(tmp_path)
    windows.build_windows(sess)
    x = np.load(os.path.join(sess, 'dataset', 'windows.npz'))['X_sl'][0, :, 0]
    assert x[0] == pytest.approx(10.0 + 0.01, abs=0.004)     # centre of the first 20 ms bin
    assert np.allclose(np.diff(x), 0.02, atol=1e-6)


def test_build_windows_rejects_legacy_sessions(tmp_path):
    sess = tmp_path / 's'
    sess.mkdir()
    (sess / 'session.json').write_text(json.dumps(dict(session='s', config={})))
    (sess / 'trials.csv').write_text('index,scenario,valid\n0,A,True\n')
    with pytest.raises(ValueError):
        windows.build_windows(str(sess))


# -- relabel of a small_lift session ------------------------------------------

def test_relabel_recomputes_y_success_and_y_stable(tmp_path):
    sess = tmp_path / 's'
    (sess / 'trials').mkdir(parents=True)
    t_lift, t_shake = 10.0, 16.0
    gz_cols = ['t_sim', 'phase', 'rel_dx', 'rel_dy', 'rel_dz', 'rel_disp', 'tilt_change_deg']
    rows = []
    for k in range(400):                                   # 8 s at 50 Hz from t = 10 s
        t = t_lift + 0.02 * k
        phase = 'small_lift' if t < 11.5 else 'lift' if t < 14 else 'hold' if t < t_shake else 'shake'
        dy = 0.0 if phase != 'shake' else min(0.1, 0.05 * (t - t_shake))   # slides out in the shake
        rows.append(dict(t_sim=t, phase=phase, rel_dx=0, rel_dy=dy, rel_dz=0, rel_disp=dy,
                         tilt_change_deg=0))
    with open(sess / 'trials' / 't0000_A.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=gz_cols)
        w.writeheader()
        w.writerows(rows)
    fields = ['index', 'scenario', 'valid', 'csv', 't_lift', 't_shake', 'grip_force_nominal_n',
              'outcome', 'failure_mode', 'slipped', 't_onset_s', 't_drop_s', 'peak_disp_m',
              'peak_tilt_change_deg', 'y_success', 'y_stable', 'shake_failure_mode',
              'shake_t_drop_s']
    with open(sess / 'trials.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerow(dict(index=0, scenario='A', valid='True', csv='trials/t0000_A.csv',
                        t_lift=t_lift, t_shake=t_shake, grip_force_nominal_n=1.4, outcome='?',
                        failure_mode='', slipped='False', t_onset_s='nan', t_drop_s='nan',
                        peak_disp_m=0, peak_tilt_change_deg=0, y_success='', y_stable='',
                        shake_failure_mode='', shake_t_drop_s=''))
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))

    out = relabel_session(str(sess), cfg)

    assert out[0]['outcome'] == 'success' and out[0]['y_success'] == 1
    assert out[0]['y_stable'] == 0                          # dropped during the shake
    assert out[0]['shake_failure_mode'] == 'slide'


def test_trial_columns_include_what_the_labelling_reads():
    for c in ('t_sim', 'phase', 'rel_dx', 'rel_dy', 'rel_dz', 'rel_disp', 'tilt_change_deg'):
        assert c in TRIAL_COLUMNS


# -- per-variant grip forces ----------------------------------------------------

def test_grip_force_uses_legacy_values_unless_small_lift_has_its_own():
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    cfg['gripper']['type'] = 'force_fingers'          # the earlier gripper's force values
    assert scenario_grip_force(cfg, 'A') == pytest.approx(1.40)
    cfg['protocol']['variant'] = 'small_lift'
    assert scenario_grip_force(cfg, 'A') == pytest.approx(1.46)      # tuned small_lift values
    assert scenario_grip_force(cfg, 'B') == pytest.approx(2.75)
    del cfg['small_lift']['grip_force_n']['B']
    assert scenario_grip_force(cfg, 'B') == pytest.approx(2.40)      # falls back to legacy
    cfg['protocol']['variant'] = 'legacy'
    assert scenario_grip_force(cfg, 'A') == pytest.approx(1.40)      # legacy untouched


def test_command_line_override_goes_to_the_active_variant():
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    cfg['protocol']['variant'] = 'small_lift'
    set_scenario_grip_force(cfg, 'B', 3.1)
    assert scenario_grip_force(cfg, 'B') == pytest.approx(3.1)
    assert cfg['scenarios']['B']['grip_force_n'] == pytest.approx(2.40)   # legacy value kept


def test_scenario_shake_overrides_only_the_named_keys():
    from px4_gripper_missions.experiment import scenario_shake
    cfg = {'shake': {'amplitude_m': 0.3, 'leg_duration_s': 2.0,
                     'per_scenario': {'B': {'leg_duration_s': 1.25}}}}
    assert scenario_shake(cfg, 'A') == {'amplitude_m': 0.3, 'leg_duration_s': 2.0}
    assert scenario_shake(cfg, 'B') == {'amplitude_m': 0.3, 'leg_duration_s': 1.25}
    assert scenario_shake({'shake': {'leg_duration_s': 2.0}}, 'A') == {'leg_duration_s': 2.0}


# -- servo claw: jaw angle command and its jitter --------------------------------

def test_claw_uses_jaw_angle_commands_with_additive_jitter():
    import numpy as np
    from px4_gripper_missions.experiment import jittered_grip, set_scenario_grip_force
    cfg = load_experiment_config(os.path.join(SRC_CONFIG, 'experiment.yaml'))
    cfg['gripper']['type'] = 'servo_claw'
    assert scenario_grip_force(cfg, 'A') == pytest.approx(cfg['small_lift']['claw_cmd_rad']['A'])
    set_scenario_grip_force(cfg, 'B', -0.2)
    assert scenario_grip_force(cfg, 'B') == pytest.approx(-0.2)
    sigma = cfg['randomization']['claw_cmd_jitter_rad']
    z = np.random.default_rng(3).standard_normal()
    assert jittered_grip(cfg, -0.22, np.random.default_rng(3)) == pytest.approx(-0.22 + sigma * z)
    cfg['gripper']['type'] = 'force_fingers'                   # force: relative jitter
    frac = cfg['randomization']['grip_force_jitter_frac']
    assert jittered_grip(cfg, 2.0, np.random.default_rng(3)) == pytest.approx(2.0 * (1 + frac * z))


def test_trial_columns_carry_the_pad_forces():
    assert TRIAL_COLUMNS[-2:] == ['fsr_l', 'fsr_r']
