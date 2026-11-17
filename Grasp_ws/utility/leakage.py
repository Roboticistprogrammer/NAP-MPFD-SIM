#!/usr/bin/env python3
"""M6 leakage check: can a model read the scenario or the applied grip force off its inputs?

In this simulator the applied grip force (nominal x random jitter) largely decides y_success,
and the scenario (A/B) is a proxy for it. Neither may reach the model, directly or indirectly.

Static checks (channels.yaml, windows.npz):
  1. no privileged variable (grip force, scenario, finger travel, displacement, tilt, ...) is
     an input channel;
  2. the per-trial static features (grip_state, commanded arm pose) are identical in A and B.
Empirical checks on the merged windows (repeated stratified 5-fold CV):
  3. scenario (A vs B) from the inputs: AUC; 0.5 = not readable;
  4. applied grip force, within each scenario, from the inputs: cross-validated R^2; 0 = not readable;
  5. how much the shortcut could be worth: y_success and y_stable predicted from the scenario
     alone, and from the grip force alone, against the majority-class rate.
Two input sets: `delta` = window minus its pre-SL hover mean (what the plan feeds the models) and
`delta+base` = the same plus the absolute hover mean of each channel.

usage: PYTHONPATH=src/px4_gripper_missions python3 leakage.py <session dir> [<session dir> ...]
"""
import os
import sys
import warnings

import numpy as np
import pandas as pd
import yaml
from sklearn.ensemble import RandomForestClassifier, RandomForestRegressor
from sklearn.linear_model import LogisticRegression, Ridge
from sklearn.model_selection import (RepeatedKFold, RepeatedStratifiedKFold, cross_val_score)
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

from px4_gripper_missions.windows import default_channels_path

warnings.filterwarnings('ignore')
FORBIDDEN = {'grip_force_n', 'grip_force_cmd', 'grasp_offset_actual_m', 'scenario', 'rel_disp',
             'tilt_change_deg', 'finger_l', 'finger_r', 'rel_dx', 'rel_dy', 'rel_dz',
             'cyl_x', 'cyl_y', 'cyl_z', 'cyl_tilt_deg'}


def load(sessions):
    xs, bs, labs, names = [], [], [], None
    for s in sessions:
        d = np.load(os.path.join(s, 'dataset', 'windows.npz'), allow_pickle=True)
        lab = pd.read_parquet(os.path.join(s, 'dataset', 'labels.parquet'))
        assert list(d['trial_id']) == list(lab['trial_id'])
        xs.append(d['X_sl'] - d['base_sl'][:, None, :])
        bs.append(d['base_sl'])
        labs.append(lab)
        names = list(d['channels'])
    return np.concatenate(xs), np.concatenate(bs), pd.concat(labs, ignore_index=True), names


def features(delta, base=None):
    """Per-channel summary statistics of the baseline-subtracted window (+ hover means)."""
    n = delta.shape[1]
    f = [delta.mean(1), delta[:, -5:].mean(1), delta.min(1), delta.max(1), delta.std(1),
         delta[:, n // 2:].mean(1) - delta[:, :n // 2].mean(1)]
    if base is not None:
        f.append(base)
    return np.concatenate(f, axis=1)


def auc_cv(model, X, y, seed=0):
    cv = RepeatedStratifiedKFold(n_splits=5, n_repeats=10, random_state=seed)
    return cross_val_score(model, X, y, cv=cv, scoring='roc_auc')


def shuffled_auc(model, X, y, n=5):
    rng = np.random.default_rng(0)
    return np.mean([auc_cv(model, X, rng.permutation(y), seed=i).mean() for i in range(n)])


def r2_cv(model, X, y, seed=0):
    cv = RepeatedKFold(n_splits=5, n_repeats=10, random_state=seed)
    return cross_val_score(model, X, y, cv=cv, scoring='r2')


def main(sessions):
    spec = yaml.safe_load(open(default_channels_path()))
    inputs = [c['name'] for c in spec['inputs']]
    bad = sorted(set(inputs) & FORBIDDEN)
    print('== static checks ==')
    print(f'1. privileged names among the {len(inputs)} input channels: {bad or "none"}')
    delta, base, lab, names = load(sessions)
    assert names == inputs, 'dataset channels differ from channels.yaml'
    a, b = lab[lab.scenario == 'A'], lab[lab.scenario == 'B']
    st = {c: (a[c].nunique() + b[c].nunique() == 2 and a[c].iloc[0] == b[c].iloc[0])
          for c in ('grip_state', 'arm_shoulder_cmd', 'arm_elbow_cmd')}
    print(f'2. static features constant and identical across A and B: {st}')
    print(f'\n{len(lab)} valid A/B trials ({len(a)} A, {len(b)} B), {len(sessions)} sessions')

    sc = (lab.scenario == 'B').to_numpy().astype(int)
    sets = {'delta': features(delta), 'delta+base': features(delta, base)}
    lr = make_pipeline(StandardScaler(), LogisticRegression(C=0.05, max_iter=2000))
    rf = RandomForestClassifier(200, min_samples_leaf=3, random_state=0, n_jobs=1)

    print('\n== 3. scenario (A vs B) from the inputs, AUC (0.5 = not readable) ==')
    for nm, X in sets.items():
        for mn, m in (('logistic', lr), ('forest', rf)):
            print(f'  {nm:11} {mn:9} {auc_cv(m, X, sc).mean():.3f}   (label-shuffled {shuffled_auc(m, X, sc):.3f})')
    X06 = features(delta[:, :30], base)
    for mn, m in (('logistic', lr), ('forest', rf)):
        print(f'  first 0.6 s {mn:9} {auc_cv(m, X06, sc).mean():.3f}   (label-shuffled {shuffled_auc(m, X06, sc):.3f})')

    print('\n== 4. applied grip force within a scenario, cross-validated R^2 (0 = not readable) ==')
    print('   (low force -> failed lift -> a visible drop, so force can be readable through the outcome;')
    print('    the held-only and first-0.6 s rows remove that path: no failure has happened yet)')
    rr = make_pipeline(StandardScaler(), Ridge(alpha=300.0))
    rfr = RandomForestRegressor(200, min_samples_leaf=3, random_state=0, n_jobs=1)
    held = (lab.y_success == 1).to_numpy()
    variants = (('all trials, whole window', np.ones(len(lab), bool), delta.shape[1]),
                ('held grasps only, whole window', held, delta.shape[1]),
                ('all trials, first 0.6 s only', np.ones(len(lab), bool), 30))
    for vn, vmask, cut in variants:
        for scen in 'AB':
            m = (lab.scenario == scen).to_numpy() & vmask
            y = lab.grip_force_n.to_numpy()[m]
            X = features(delta[m][:, :cut], base[m])
            out = [f'{mn} {r2_cv(mod, X, y).mean():+.2f}' for mn, mod in (('ridge', rr), ('forest', rfr))]
            print(f'  {scen} {vn:32} n={m.sum():3d}  R2: ' + '   '.join(out))

    print('\n== 5. what the shortcut would be worth (majority = best guess without it) ==')
    for lbl in ('y_success', 'y_stable'):
        m = lab[lbl].notna().to_numpy()
        y = lab[lbl][m].astype(int).to_numpy()
        print(f'  {lbl}: n={m.sum()}, positive rate {y.mean():.2f}, majority-class accuracy '
              f'{max(y.mean(), 1 - y.mean()):.2f}')
        for scen in 'AB':
            ms = (lab.scenario[m] == scen).to_numpy()
            print(f'     within {scen}: positive rate {y[ms].mean():.2f} (n={ms.sum()})')
        for nm, col in (('scenario alone', sc[m, None]),
                        ('grip force alone', lab.grip_force_n.to_numpy()[m, None]),
                        ('scenario + grip force', np.c_[sc[m], lab.grip_force_n.to_numpy()[m]])):
            try:
                auc = auc_cv(make_pipeline(StandardScaler(), LogisticRegression()), col, y).mean()
                print(f'     {nm:22} AUC {auc:.3f}')
            except ValueError:
                print(f'     {nm:22} AUC n/a')
        for scen in 'AB':          # force within a scenario: the real hidden driver
            ms = (lab.scenario[m] == scen).to_numpy()
            f = lab.grip_force_n.to_numpy()[m][ms, None]
            if len(set(y[ms])) > 1 and ms.sum() >= 10:
                auc = auc_cv(make_pipeline(StandardScaler(), LogisticRegression()), f, y[ms]).mean()
                print(f'     grip force alone, within {scen}: AUC {auc:.3f}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
