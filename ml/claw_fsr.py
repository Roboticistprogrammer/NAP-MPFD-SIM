"""Servo-claw batch: do the jaw-pad forces (simulated FSRs) warn earlier than the drone channels?

Inputs per trial (windows.npz of each session, built by `experiment.sh dataset`):
  X_sl       21 drone channels, 75 samples from t0 (hover mean subtracted)
  X_fsr_sl   left / right jaw pad normal force [N], same grid, absolute
  X_fsr_cg   the same during the 1.0 s gripper close (ends at t0)
The pad forces are passed through the FSR 402 sensor model (ml/fsr_model.py) at several per-trial
gain errors, independently per jaw; 'ideal' is the raw contact force (upper bound).

Also reports, for every failed grasp, when the pad force first falls by 20 % from its value at t0
compared with slip onset (10 mm) and the drop (40 mm).

usage (repo root): python3 -m ml.claw_fsr <session dir> [...] [--out ml/results/claw_fsr.json]
"""
import argparse
import json
import os
import warnings

import numpy as np
import pandas as pd
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from ml.data import strata
from ml.fsr_model import FSRParams, draw_gains, read
from ml.run_baselines import cv_scores, features

warnings.filterwarnings('ignore')
RATE = 50.0
ENDS = [0.2, 0.4, 0.6, 0.8, 1.0, 1.5]
GAINS = [0.05, 0.10, 0.25]


def load(sessions):
    xs, fs, fc, labs = [], [], [], []
    for s in sessions:
        d = np.load(os.path.join(s, 'dataset', 'windows.npz'), allow_pickle=True)
        if 'X_fsr_sl' not in d.files:
            raise ValueError(f'{s}: no pad forces (not a servo_claw session?)')
        xs.append(d['X_sl'] - d['base_sl'][:, None, :])
        fs.append(d['X_fsr_sl']); fc.append(d['X_fsr_cg'])
        labs.append(pd.read_parquet(os.path.join(s, 'dataset', 'labels.parquet')))
    return np.concatenate(xs), np.concatenate(fs), np.concatenate(fc), pd.concat(labs, ignore_index=True)


def svm_fp(xtr, ytr, xte, seed):
    m = make_pipeline(StandardScaler(), SVC(C=1.0, gamma='scale', class_weight='balanced',
                                            probability=True, random_state=0))
    m.fit(xtr, ytr)
    return m.predict_proba(xte)[:, 1]


def sensed(f_sl, f_cg, gain, seed):
    """FSR 402 readings (ADC counts) of both jaws, independent sensors and gain draws."""
    p = FSRParams.from_yaml(per_trial_gain=gain)
    rng = np.random.default_rng(seed)
    out_sl, out_cg = [], []
    for j in range(f_sl.shape[2]):
        g = draw_gains(f_sl.shape[0], p, rng)
        t_cg = np.arange(f_cg.shape[1]) / RATE                 # time since the close started
        t_sl = t_cg[-1] + 1 / RATE + np.arange(f_sl.shape[1]) / RATE
        out_cg.append(read(f_cg[:, :, j], np.broadcast_to(t_cg, f_cg.shape[:2]), p, rng, g))
        out_sl.append(read(f_sl[:, :, j], np.broadcast_to(t_sl, f_sl.shape[:2]), p, rng, g))
    return np.stack(out_sl, 2), np.stack(out_cg, 2)


def first_fall(f, frac=0.2):
    """Index of the first sample where the summed pad force is below (1 - frac) x its t0 value."""
    tot = f.sum(axis=2)
    out = np.full(len(tot), np.nan)
    for i, row in enumerate(tot):
        if row[0] <= 0.05:
            continue
        k = np.nonzero(row < (1 - frac) * row[0])[0]
        if len(k):
            out[i] = k[0] / RATE
    return out


def auc3(r):
    return {k: round(r[k]['auc'][0], 3) for k in ('pooled', 'A', 'B') if k in r}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--repeats', type=int, default=5)
    ap.add_argument('--realizations', type=int, default=10)
    ap.add_argument('--out', default='ml/results/claw_fsr.json')
    a = ap.parse_args()

    xd, f_sl, f_cg, lab = load(a.sessions)
    y = lab.y_success.astype(int).to_numpy(); sc = lab.scenario.to_numpy(); st = strata(lab, 'y_success')
    cv = lambda X, reps=a.repeats: auc3(cv_scores(X, y, sc, st, svm_fp, reps))
    print(f'{len(y)} trials (A {int((sc == "A").sum())}, B {int((sc == "B").sum())}), held {int(y.sum())}')
    res = {'n_trials': len(y), 'held': int(y.sum())}

    # --- timing: pad force fall vs slip onset and drop, failed grasps
    fail = y == 0
    fall = first_fall(f_sl[fail])
    on = lab.t_onset_s.to_numpy()[fail]; dr = lab.t_drop_s.to_numpy()[fail]
    ok = ~np.isnan(fall) & ~np.isnan(on) & ~np.isnan(dr)
    held_fall = first_fall(f_sl[~fail])
    timing = dict(n_failed=int(fail.sum()), n_with_fall=int(ok.sum()),
                  fall_median_s=round(float(np.nanmedian(fall)), 3),
                  onset_median_s=round(float(np.nanmedian(on)), 3),
                  drop_median_s=round(float(np.nanmedian(dr)), 3),
                  fall_before_onset_frac=round(float(np.mean(fall[ok] < on[ok])), 3),
                  lead_before_drop_median_s=round(float(np.median(dr[ok] - fall[ok])), 3),
                  lead_before_onset_median_s=round(float(np.median(on[ok] - fall[ok])), 3),
                  held_with_20pct_fall_in_window=int(np.sum(~np.isnan(held_fall))), n_held=int((~fail).sum()))
    res['timing'] = timing
    print('timing (failed grasps):', timing)

    # --- AUC by input and window end
    t = {}
    for e in ENDS:
        n = int(round(e * RATE))
        t[f'drone|{e}'] = cv(features(xd[:, :n]))
        t[f'fsr_ideal|{e}'] = cv(features(f_sl[:, :n]))
        t[f'drone+fsr_ideal|{e}'] = cv(np.c_[features(xd[:, :n]), features(f_sl[:, :n])])
    t['fsr_ideal|closing'] = cv(features(f_cg))
    for g in GAINS:
        runs = {}
        for k in range(a.realizations):
            s_sl, s_cg = sensed(f_sl, f_cg, g, 100 * k + int(g * 1000))
            for e in ENDS:
                n = int(round(e * RATE))
                runs.setdefault(f'fsr_{g}|{e}', []).append(cv(features(s_sl[:, :n]), 1))
                runs.setdefault(f'drone+fsr_{g}|{e}', []).append(cv(np.c_[features(xd[:, :n]), features(s_sl[:, :n])], 1))
            runs.setdefault(f'fsr_{g}|closing', []).append(cv(features(s_cg), 1))
        for key, rs in runs.items():
            t[key] = {k: round(float(np.mean([r[k] for r in rs])), 3) for k in rs[0]}
            t[key]['sd'] = round(float(np.std([r['pooled'] for r in rs])), 3)
    res['auc'] = t
    print(f'{"input":24}' + ''.join(f'{e:>8}' for e in ENDS) + '   closing')
    for name in ['drone', 'fsr_ideal', 'drone+fsr_ideal'] + [f'{p}_{g}' for g in GAINS for p in ('fsr', 'drone+fsr')]:
        cells = ''.join(f'{t[f"{name}|{e}"]["pooled"]:8.2f}' for e in ENDS)
        cl = t.get(f'{name}|closing', {}).get('pooled')
        print(f'{name:24}{cells}   {"" if cl is None else f"{cl:.2f}"}')
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    json.dump(res, open(a.out, 'w'), indent=1)
    print('wrote', a.out)


if __name__ == '__main__':
    main()
