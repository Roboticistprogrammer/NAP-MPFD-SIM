"""Slice a grasp-experiment session's PX4 ULog per trial and merge it with
the Gazebo ground truth into one table per trial.

    export_trials sim_data/<session> [--rate 100] [--ulog path.ulg]

Writes sim_data/<session>/merged/tNNNN_X.csv: a uniform time grid (sim
time, default 100 Hz) over the trial window with
  - drone-side signals (what a detector on the vehicle could see):
    thrust/torque setpoints, motor commands, attitude, body rates,
    accelerometer, vertical position/velocity
  - ground truth + labels from the runner's per-trial CSV (phase,
    cylinder displacement/tilt, finger travel, ...) and the trial's
    scenario / grip / outcome columns from trials.csv.
Values are sample-and-hold (last sample at or before each grid time).

Needs pyulog (pip install pyulog). Unlike run_experiment it does not use
Gazebo, so run it with your normal Python environment.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

# (ulog topic, instance, [(field, column name)])
PX4_SIGNALS = [
    ('vehicle_thrust_setpoint', 0, [('xyz[0]', 'thrust_sp_x'), ('xyz[1]', 'thrust_sp_y'),
                                    ('xyz[2]', 'thrust_sp_z')]),
    ('vehicle_torque_setpoint', 0, [('xyz[0]', 'torque_sp_x'), ('xyz[1]', 'torque_sp_y'),
                                    ('xyz[2]', 'torque_sp_z')]),
    ('actuator_motors', 0, [(f'control[{i}]', f'motor_{i}') for i in range(4)]),
    ('vehicle_attitude', 0, [('q[0]', 'att_qw'), ('q[1]', 'att_qx'), ('q[2]', 'att_qy'),
                             ('q[3]', 'att_qz')]),
    ('vehicle_angular_velocity', 0, [('xyz[0]', 'rate_x'), ('xyz[1]', 'rate_y'),
                                     ('xyz[2]', 'rate_z')]),
    ('sensor_combined', 0, [('accelerometer_m_s2[0]', 'acc_x'), ('accelerometer_m_s2[1]', 'acc_y'),
                            ('accelerometer_m_s2[2]', 'acc_z')]),
    ('vehicle_local_position', 0, [('z', 'lpos_z'), ('vz', 'lpos_vz')]),
]

GZ_COLUMNS = ['phase', 'drone_roll', 'drone_pitch', 'grasp_z', 'cyl_z', 'cyl_tilt_deg',
              'rel_disp', 'tilt_change_deg', 'finger_l', 'finger_r', 'grip_force_cmd']
LABEL_COLUMNS = ['index', 'battery', 'scenario', 'grip_force_n', 'grasp_offset_actual_m',
                 'outcome', 'failure_mode', 't_onset_s', 't_drop_s']


def hold_sample(t_src: np.ndarray, v_src: np.ndarray, t_grid: np.ndarray) -> np.ndarray:
    """Last value at or before each grid time (NaN before the first sample)."""
    idx = np.searchsorted(t_src, t_grid, side='right') - 1
    out = np.full(t_grid.shape, np.nan)
    ok = idx >= 0
    out[ok] = v_src[idx[ok]]
    return out


def load_px4(ulog_path: str):
    from pyulog import ULog
    names = sorted({t for t, _, _ in PX4_SIGNALS})
    ulog = ULog(ulog_path, names)
    series = {}
    for topic, inst, fields in PX4_SIGNALS:
        ds = next((d for d in ulog.data_list if d.name == topic and d.multi_id == inst), None)
        if ds is None:
            print(f'warning: {topic} not in ULog (check SDLOG_PROFILE)', file=sys.stderr)
            continue
        t = ds.data['timestamp'] * 1e-6
        for fld, col in fields:
            if fld in ds.data:
                series[col] = (t, ds.data[fld].astype(float))
    return series


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('session', help='sim_data/<session> directory')
    ap.add_argument('--rate', type=float, default=100.0, help='output grid rate [Hz]')
    ap.add_argument('--ulog', help='override the ULog path recorded in session.json')
    args = ap.parse_args(argv)

    with open(os.path.join(args.session, 'session.json')) as f:
        meta = json.load(f)
    ulog_path = args.ulog or meta.get('ulog')
    if not ulog_path or not os.path.exists(ulog_path):
        print(f'ULog not found ({ulog_path!r}); pass --ulog', file=sys.stderr)
        return 1
    offset = float(meta.get('px4_time_offset_s', 0.0))
    print(f'loading {ulog_path}')
    px4 = load_px4(ulog_path)

    with open(os.path.join(args.session, 'trials.csv')) as f:
        trials = list(csv.DictReader(f))
    out_dir = os.path.join(args.session, 'merged')
    os.makedirs(out_dir, exist_ok=True)
    px4_cols = [c for _, _, flds in PX4_SIGNALS for _, c in flds if c in px4]

    n = 0
    for tr in trials:
        if not tr['csv']:
            continue
        with open(os.path.join(args.session, tr['csv'])) as f:
            gz_rows = list(csv.DictReader(f))
        if not gz_rows:
            continue
        t_gz = np.array([float(r['t_sim']) for r in gz_rows])
        grid = np.arange(t_gz[0], t_gz[-1], 1.0 / args.rate)
        cols = {'t_sim': grid, 't_rel_lift': grid - float(tr['t_lift'] or 'nan')}
        for c in px4_cols:
            t, v = px4[c]
            cols[c] = hold_sample(t - offset, v, grid)
        idx = np.clip(np.searchsorted(t_gz, grid, side='right') - 1, 0, len(gz_rows) - 1)
        name = os.path.basename(tr['csv'])
        with open(os.path.join(out_dir, name), 'w', newline='') as f:
            w = csv.writer(f)
            header = list(cols) + GZ_COLUMNS + LABEL_COLUMNS
            w.writerow(header)
            labels = [tr.get(k, '') for k in LABEL_COLUMNS]
            for i in range(len(grid)):
                g = gz_rows[idx[i]]
                w.writerow([f'{cols[c][i]:.6g}' if not math.isnan(cols[c][i]) else ''
                            for c in cols] + [g[c] for c in GZ_COLUMNS] + labels)
        n += 1
    print(f'wrote {n} merged trial tables to {out_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
