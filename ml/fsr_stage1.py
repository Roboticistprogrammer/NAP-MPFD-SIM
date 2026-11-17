"""FSR study, Stage 1: an offline fingertip-force sensor on the existing 131 batch trials.

The synthetic sensor reads the force applied to the fingers (the per-row `grip_force_cmd` of the
trial CSVs: negative = opening command, i.e. no contact) through the FSR 402 model in
ml/fsr_model.py. In this simulator that force is constant from the close onwards, so the signal
has no time structure: Stage 1 measures how precisely the grip force must be read for it to
predict y_success, not whether a contact force warns earlier (that needs Stage 2, a measured
contact force). The ideal sensor and grip_force_cmd itself are upper bounds, never the result.

usage (repo root): python3 -m ml.fsr_stage1 [--merged Grasp_ws/sim_data/merged_batch1]
                   [--sessions-root Grasp_ws/sim_data] [--realizations 5] [--repeats 3]
"""
import argparse
import csv
import json
import os
import warnings

import numpy as np
import pandas as pd
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from ml.data import strata
from ml.fsr_model import FSRParams, draw_gains, ideal_counts, read
from ml.run_baselines import cv_scores, features

warnings.filterwarnings('ignore')
RATE = 50.0
ENDS = [0.4, 0.6, 0.8, 1.0, 1.5]
GAINS = [0.0, 0.05, 0.10, 0.25, 0.50]
MAIN_GAIN = 0.10


def force_window(t, cmd, t_start, n, rate=RATE):
    """(force, time since contact) on the 50 Hz grid from t_start; zero-order hold.
    Negative commands (opening) mean no contact force."""
    t = np.asarray(t, dtype=float); cmd = np.asarray(cmd, dtype=float)
    ts = t_start + np.arange(n) / rate
    idx = np.clip(np.searchsorted(t, ts + 1e-9, side='right') - 1, 0, len(t) - 1)
    f = np.clip(cmd[idx], 0.0, None)
    on = np.nonzero(cmd > 0)[0]
    t_contact = t[on[0]] if len(on) else np.inf
    return f, ts - t_contact


def load(merged, sessions_root):
    lab = pd.read_parquet(os.path.join(merged, 'labels.parquet'))
    xd = np.load(os.path.join(merged, 'windows.npz'), allow_pickle=True)['X_sl']
    f_sl, t_sl, f_cg, t_cg = [], [], [], []
    for _, r in lab.iterrows():
        path = os.path.join(sessions_root, r.session, 'trials', f"t{int(r['index']):04d}_{r.scenario}.csv")
        rows = list(csv.DictReader(open(path)))
        t = [float(x['t_sim']) for x in rows]; cmd = [float(x['grip_force_cmd']) for x in rows]
        a, b = force_window(t, cmd, r.t0, 75); c, d = force_window(t, cmd, r.t_close, 50)
        f_sl.append(a); t_sl.append(b); f_cg.append(c); t_cg.append(d)
    return lab, xd, *map(np.array, (f_sl, t_sl, f_cg, t_cg))


def svm_fp(xtr, ytr, xte, seed):
    m = make_pipeline(StandardScaler(), SVC(C=1.0, gamma='scale', class_weight='balanced',
                                            probability=True, random_state=0))
    m.fit(xtr, ytr)
    return m.predict_proba(xte)[:, 1]


def auc3(r):
    return {k: round(r[k]['auc'][0], 3) for k in ('pooled', 'A', 'B') if k in r}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--merged', default='Grasp_ws/sim_data/merged_batch1')
    ap.add_argument('--sessions-root', default='Grasp_ws/sim_data')
    ap.add_argument('--realizations', type=int, default=20)
    ap.add_argument('--repeats', type=int, default=3)
    ap.add_argument('--permutations', type=int, default=50)
    ap.add_argument('--out', default='ml/results/fsr_stage1.json')
    a = ap.parse_args()

    lab, xd, f_sl, t_sl, f_cg, t_cg = load(a.merged, a.sessions_root)
    y = lab.y_success.astype(int).to_numpy(); sc = lab.scenario.to_numpy(); st = strata(lab, 'y_success')
    cv = lambda X, yy=y, ss=st, reps=a.repeats: cv_scores(X, yy, sc, ss, svm_fp, reps)
    p0 = FSRParams.from_yaml()
    res = {'n_trials': len(y), 'params': vars(p0), 'main_gain': MAIN_GAIN}

    # resolution: what an 8 % force step (the jitter's 1 sigma) does to the reading
    for f in (1.46, 2.75):
        c0, c1 = ideal_counts(np.array([f, f * 1.08]), p0)
        print(f'reading at {f} N: {c0:.0f} counts; +8 % force -> +{c1 - c0:.1f} counts')

    def sensor(gain, seed, n_sensors=1):
        p = FSRParams.from_yaml(per_trial_gain=gain)
        rng = np.random.default_rng(seed)
        out_sl, out_cg = [], []
        for _ in range(n_sensors):
            g = draw_gains(len(y), p, rng)
            out_sl.append(read(f_sl, t_sl, p, rng, g)); out_cg.append(read(f_cg, t_cg, p, rng, g))
        return np.stack(out_sl, 2), np.stack(out_cg, 2)

    # --- table 1: input x window end at the main gain error, plus the upper bounds
    t1 = {}
    for e in ENDS:
        n = int(e * RATE)
        t1[f'drone|{e}'] = auc3(cv(features(xd[:, :n])))
    real = [sensor(MAIN_GAIN, 1000 + k) for k in range(a.realizations)]   # same draws as table 2
    for name in ('fsr', 'drone+fsr'):
        for e in ENDS:
            n = int(e * RATE); runs = []
            for s_sl, _ in real:
                X = features(s_sl[:, :n]) if name == 'fsr' else np.c_[features(xd[:, :n]), features(s_sl[:, :n])]
                runs.append(auc3(cv(X)))
            t1[f'{name}|{e}'] = {k: round(float(np.mean([r[k] for r in runs])), 3) for k in runs[0]}
            t1[f'{name}|{e}']['sd'] = round(float(np.std([r['pooled'] for r in runs])), 3)
    runs = [auc3(cv(features(s_cg))) for _, s_cg in real]
    t1['fsr_closing_window'] = {k: round(float(np.mean([r[k] for r in runs])), 3) for k in runs[0]}
    t1['fsr_closing_window']['sd'] = round(float(np.std([r['pooled'] for r in runs])), 3)
    t1['ideal_sensor_closing_window'] = auc3(cv(features(f_cg[:, :, None])))
    t1['grip_force_cmd_alone'] = auc3(cv(lab.grip_force_n.to_numpy()[:, None]))
    res['table1'] = t1

    # --- table 2: sensor quality sweep (closing window = before lift-off; and drone+FSR at 0.6 s)
    t2 = {}
    for gain in GAINS:
        rows = {'fsr_cg': [], 'fsr_x2_cg': [], 'drone+fsr_0.6': []}
        for k in range(a.realizations):
            s_sl, s_cg = sensor(gain, 1000 + k)
            _, s2_cg = sensor(gain, 5000 + k, n_sensors=2)
            rows['fsr_cg'].append(auc3(cv(features(s_cg))))
            rows['fsr_x2_cg'].append(auc3(cv(features(s2_cg))))
            rows['drone+fsr_0.6'].append(auc3(cv(np.c_[features(xd[:, :30]), features(s_sl[:, :30])])))
        t2[str(gain)] = {name: {k: [round(float(np.mean([r[k] for r in v])), 3), round(float(np.std([r[k] for r in v])), 3)]
                                for k in v[0]} for name, v in rows.items()}
        print(f'gain error {gain:4.2f}: ' + '  '.join(f"{nm} {t2[str(gain)][nm]['pooled'][0]:.2f} "
              f"(A {t2[str(gain)][nm]['A'][0]:.2f}, B {t2[str(gain)][nm]['B'][0]:.2f})" for nm in rows), flush=True)
    res['table2'] = t2

    # --- control: label permutation within scenario, FSR closing window at the main gain error
    _, s_cg = real[0]; Xp = features(s_cg); rng = np.random.default_rng(7); null = []
    for _ in range(a.permutations):
        yp = y.copy()
        for s in ('A', 'B'):
            m = sc == s; yp[m] = rng.permutation(y[m])
        null.append(cv(Xp, yp, strata(lab.assign(y_success=yp), 'y_success'), 1)['pooled']['auc'][0])
    res['permutation_null'] = [round(float(np.mean(null)), 3), round(float(np.std(null)), 3)]

    print('\ntable 1 (gain error 0.10, mean of', a.realizations, 'sensor realizations): pooled (A / B)')
    for k, v in t1.items():
        print(f'  {k:32} {v["pooled"]:.2f} ({v.get("A", float("nan")):.2f} / {v.get("B", float("nan")):.2f})' + (f'  sd {v["sd"]:.3f}' if 'sd' in v else ''))
    print('permutation null (fsr closing window):', res['permutation_null'])
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, 'w'), indent=1)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
