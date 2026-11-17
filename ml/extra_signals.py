"""Do other PX4 log signals warn earlier than the 21 channels of channels.yaml?

Cuts additional ULog topics into the same 50 Hz small-lift windows (75 samples from t0, hover
mean of the 0.5 s before t0 subtracted) for the trials of the merged dataset, derives tracking
errors, and scores each group of signals with the same SVM / repeated stratified CV as
run_baselines. All topics are logged by PX4 on the real drone too (offboard flight uses the
same controllers); none depends on the simulator.

Groups:
  tracking    setpoint minus measured: position, velocity, acceleration (local position loop)
  controller  position-controller thrust output, rate setpoints, rate errors, angular accel
  hover_est   PX4 hover-thrust estimator: estimate, variance, innovation, noise
  innov       EKF2 external-vision innovations (position and velocity)
  accel_batt  filtered body acceleration and battery current

usage (repo root):
  PYTHONPATH=Grasp_ws/src/px4_gripper_missions python3 -m ml.extra_signals \\
      <merged dir> --sessions-root Grasp_ws/sim_data [--out ml/results/extra_signals.json]
"""
import argparse
import json
import os
import sys
import warnings

import numpy as np
import pandas as pd
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from ml.data import strata
from ml.run_baselines import cv_scores, features
from px4_gripper_missions.windows import extract_window, load_ulog_channels

warnings.filterwarnings('ignore')
RATE, N_SL, N_BASE = 50.0, 75, 25

LPS, LP, RS, AV = ('vehicle_local_position_setpoint', 'vehicle_local_position',
                   'vehicle_rates_setpoint', 'vehicle_angular_velocity')
RAW = (
    [(f'lps_{k}', LPS, k) for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')]
    + [(f'lps_a{i}', LPS, f'acceleration[{i}]') for i in range(3)]
    + [(f'lps_thr{i}', LPS, f'thrust[{i}]') for i in range(3)]
    + [(f'lp_{k}', LP, k) for k in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')]
    + [(f'rsp_{k}', RS, k) for k in ('roll', 'pitch', 'yaw')]
    + [(f'rate_{i}', AV, f'xyz[{i}]') for i in range(3)]
    + [(f'angacc_{i}', AV, f'xyz_derivative[{i}]') for i in range(3)]
    + [('ht', 'hover_thrust_estimate', 'hover_thrust'),
       ('ht_var', 'hover_thrust_estimate', 'hover_thrust_var'),
       ('ht_innov', 'hover_thrust_estimate', 'accel_innov'),
       ('ht_noise', 'hover_thrust_estimate', 'accel_noise_var')]
    + [('ev_vpos', 'estimator_innovations', 'ev_vpos'),
       ('ev_hpos0', 'estimator_innovations', 'ev_hpos[0]'),
       ('ev_hpos1', 'estimator_innovations', 'ev_hpos[1]'),
       ('ev_vvel', 'estimator_innovations', 'ev_vvel')]
    + [(f'vacc_{i}', 'vehicle_acceleration', f'xyz[{i}]') for i in range(3)]
    + [('batt_i', 'battery_status', 'current_a')]
)
NAMES = [r[0] for r in RAW]


def derive(w: dict) -> dict:
    """Named (T,) arrays -> the groups' channels."""
    g = {}
    g['tracking'] = {f'err_{k}': w[f'lps_{k}'] - w[f'lp_{k}'] for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')}
    g['tracking'].update({f'err_a{i}': w[f'lps_a{i}'] - w[f'lp_a{"xyz"[i]}'] for i in range(3)})
    g['controller'] = {f'lps_thr{i}': w[f'lps_thr{i}'] for i in range(3)}
    g['controller'].update({f'rsp_{k}': w[f'rsp_{k}'] for k in ('roll', 'pitch', 'yaw')})
    g['controller'].update({f'rate_err{i}': w[f'rsp_{("roll", "pitch", "yaw")[i]}'] - w[f'rate_{i}'] for i in range(3)})
    g['controller'].update({f'angacc_{i}': w[f'angacc_{i}'] for i in range(3)})
    g['hover_est'] = {k: w[k] for k in ('ht', 'ht_var', 'ht_innov', 'ht_noise')}
    g['innov'] = {k: w[k] for k in ('ev_vpos', 'ev_hpos0', 'ev_hpos1', 'ev_vvel')}
    g['accel_batt'] = {**{f'vacc_{i}': w[f'vacc_{i}'] for i in range(3)}, 'batt_i': w['batt_i']}
    return g


def build(merged_dir: str, sessions_root: str):
    lab = pd.read_parquet(os.path.join(merged_dir, 'labels.parquet'))
    spec = {'inputs': [dict(name=n, topic=t, field=f) for n, t, f in RAW]}
    rows = []
    for sess, grp in lab.groupby('session', sort=False):
        sdir = os.path.join(sessions_root, sess)
        info = json.load(open(os.path.join(sdir, 'session.json')))
        print(f'loading {sess} ({len(grp)} trials)', file=sys.stderr, flush=True)
        series = load_ulog_channels(info['ulog'], spec)
        off = float(info.get('px4_time_offset_s', 0.0))
        for i, r in grp.iterrows():
            win = extract_window(series, NAMES, r.t0, N_SL, RATE, off)
            base = extract_window(series, NAMES, r.t0 - 0.5, N_BASE, RATE, off)
            rows.append((i, win, base))
        del series
    rows.sort(key=lambda x: x[0])
    win = np.array([r[1] for r in rows])
    base = np.array([r[2] for r in rows])
    return lab, win, base


def to_groups(win, base):
    """win/base: (N,T,len(NAMES)). Returns {group: (names, X baseline-subtracted)}."""
    out, dropped = {}, []
    # derive per trial with baseline subtraction applied after deriving errors
    per = {}
    for n in range(win.shape[0]):
        wd = {nm: win[n, :, j] for j, nm in enumerate(NAMES)}
        bd = {nm: base[n, :, j] for j, nm in enumerate(NAMES)}
        gw, gb = derive(wd), derive(bd)
        for grp in gw:
            for ch in gw[grp]:
                per.setdefault((grp, ch), []).append(gw[grp][ch] - np.nanmean(gb[grp][ch]))
    for (grp, ch), v in per.items():
        a = np.array(v)
        if np.isnan(a).any():
            dropped.append((grp, ch))
            continue
        out.setdefault(grp, {})[ch] = a
    groups = {g: (list(d), np.stack([d[c] for c in d], axis=2)) for g, d in out.items()}
    return groups, dropped


def svm_fp(xtr, ytr, xte, seed):
    m = make_pipeline(StandardScaler(), SVC(C=1.0, gamma='scale', class_weight='balanced',
                                            probability=True, random_state=0))
    m.fit(xtr, ytr)
    return m.predict_proba(xte)[:, 1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('merged', help='merged dataset dir (labels.parquet, windows.npz)')
    ap.add_argument('--sessions-root', default='Grasp_ws/sim_data')
    ap.add_argument('--ends', nargs='+', type=float, default=[0.4, 0.6, 0.8, 1.0])
    ap.add_argument('--repeats', type=int, default=10)
    ap.add_argument('--out', default='ml/results/extra_signals.json')
    a = ap.parse_args()

    lab, win, base = build(a.merged, a.sessions_root)
    groups, dropped = to_groups(win, base)
    if dropped:
        print('dropped (NaN in some window):', dropped)
    np.savez_compressed(os.path.join(a.merged, 'extra_windows.npz'),
                        **{f'{g}__{nm}': groups[g][1][:, :, k] for g in groups
                           for k, nm in enumerate(groups[g][0])})

    d0 = np.load(os.path.join(a.merged, 'windows.npz'), allow_pickle=True)
    x21 = d0['X_sl']
    y = lab.y_success.astype(int).to_numpy()
    sc = lab.scenario.to_numpy()
    st = strata(lab, 'y_success')

    sets = {'base 21 (reference)': x21}
    for g in groups:
        sets[f'{g} ({len(groups[g][0])} ch)'] = groups[g][1]
    allnew = np.concatenate([groups[g][1] for g in groups], axis=2)
    sets[f'all new ({allnew.shape[2]} ch)'] = allnew
    sets['base 21 + all new'] = np.concatenate([x21, allnew], axis=2)

    res = {}
    print(f'{len(y)} trials; pooled AUC (A / B) at window end, SVM, {a.repeats}x repeated 5-fold CV')
    print(f'{"input":28}' + ''.join(f'{e:>16.1f}s' for e in a.ends))
    for name, X in sets.items():
        cells = []
        for end in a.ends:
            n = int(round(end * RATE))
            r = cv_scores(features(X[:, :n]), y, sc, st, svm_fp, a.repeats)
            res[f'{name}|{end}'] = r
            cells.append(f"{r['pooled']['auc'][0]:.2f} ({r['A']['auc'][0]:.2f}/{r['B']['auc'][0]:.2f})")
        print(f'{name:28}' + ''.join(f'{c:>17}' for c in cells), flush=True)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, 'w'), indent=1)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
