"""Baselines for y_success / y_stable from the small-lift (SL) window: RBF-SVM and a small LSTM.

For each window end (0.4 ... 1.5 s after t0) the window is cut to that length, so a model cannot
score by simply seeing the cylinder already fall (result.md section 4, point 5). Cross-validation
is stratified by scenario x label (plan, section 6), repeated; a leave-one-session-out run checks
for drift between sim restarts. Scores are shown pooled and per scenario, each next to the
majority-class rate, because the scenario alone is easy to read (result.md section 12) and the
grip force, which stays out of the inputs, decides most of y_success.

usage (repo root):  python3 -m ml.run_baselines <session dir> [<session dir> ...]
                    [--targets y_success y_stable] [--ends 0.4 0.6 0.8 1.0 1.5]
                    [--repeats 5] [--lstm-repeats 2] [--no-lstm] [--out ml/results]
"""
import argparse
import json
import os
import time
import warnings

import numpy as np
from sklearn.metrics import balanced_accuracy_score, roc_auc_score
from sklearn.model_selection import LeaveOneGroupOut, RepeatedStratifiedKFold
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC

from ml.data import RATE_HZ, load_sessions, save_merged, strata

warnings.filterwarnings('ignore')


def features(x: np.ndarray) -> np.ndarray:
    """Per-channel summary of a baseline-subtracted window (T = x.shape[1])."""
    n = x.shape[1]
    h = n // 2
    return np.concatenate([x.mean(1), x[:, -5:].mean(1), x.min(1), x.max(1), x.std(1),
                           x[:, h:].mean(1) - x[:, :h].mean(1)], axis=1)


def svm_model():
    return make_pipeline(StandardScaler(), SVC(kernel='rbf', C=1.0, gamma='scale',
                                               class_weight='balanced', probability=True,
                                               random_state=0))


def fit_predict_svm(xtr, ytr, xte, seed):
    m = svm_model()
    m.fit(features(xtr), ytr)
    return m.predict_proba(features(xte))[:, 1]


def fit_predict_lstm(xtr, ytr, xte, seed, epochs=60):
    import torch
    from torch import nn
    torch.manual_seed(seed)
    torch.set_num_threads(2)
    mu, sd = xtr.mean((0, 1)), xtr.std((0, 1)) + 1e-8        # training-fold statistics only
    f = lambda a: torch.tensor((a - mu) / sd, dtype=torch.float32)
    xt, xe, yt = f(xtr), f(xte), torch.tensor(ytr, dtype=torch.float32)

    class Net(nn.Module):
        def __init__(self, c):
            super().__init__()
            self.l = nn.LSTM(c, 20, batch_first=True)
            self.o = nn.Linear(20, 1)

        def forward(self, x):
            h, _ = self.l(x)
            return self.o(h[:, -1]).squeeze(-1)

    net = Net(xt.shape[2])
    opt = torch.optim.Adam(net.parameters(), lr=3e-3, weight_decay=1e-3)
    pos = float(ytr.mean())
    w = torch.tensor((1 - pos) / max(pos, 1e-6))
    lossf = nn.BCEWithLogitsLoss(pos_weight=w)
    g = torch.Generator().manual_seed(seed)
    for _ in range(epochs):
        net.train()
        for idx in torch.randperm(len(xt), generator=g).split(16):
            xb = xt[idx] + 0.05 * torch.randn_like(xt[idx])   # light noise augmentation
            opt.zero_grad()
            lossf(net(xb), yt[idx]).backward()
            nn.utils.clip_grad_norm_(net.parameters(), 1.0)
            opt.step()
    net.eval()
    with torch.no_grad():
        return torch.sigmoid(net(xe)).numpy()


def metrics(y, p, scen):
    """Pooled and per-scenario accuracy (vs majority), balanced accuracy and AUC."""
    out = {}
    for name, m in (('pooled', np.ones(len(y), bool)), ('A', scen == 'A'), ('B', scen == 'B')):
        yy, pp = y[m], p[m]
        if len(set(yy)) < 2:
            continue
        out[name] = dict(n=int(m.sum()), majority=float(max(yy.mean(), 1 - yy.mean())),
                         acc=float(((pp > 0.5) == yy).mean()),
                         bal_acc=float(balanced_accuracy_score(yy, pp > 0.5)),
                         auc=float(roc_auc_score(yy, pp)))
    return out


def summarise(runs):
    """Mean +- sd over repeats of each metric, per subset."""
    res = {}
    for sub in runs[0]:
        res[sub] = {k: (float(np.mean([r[sub][k] for r in runs if sub in r])),
                        float(np.std([r[sub][k] for r in runs if sub in r])))
                    for k in ('majority', 'acc', 'bal_acc', 'auc')}
        res[sub]['n'] = runs[0][sub]['n']
    return res


def cv_scores(X, y, scen, strata_, fitpred, repeats, seed0=0):
    runs = []
    cv = RepeatedStratifiedKFold(n_splits=5, n_repeats=repeats, random_state=seed0)
    oof = np.zeros(len(y))
    for i, (tr, te) in enumerate(cv.split(X, strata_)):
        oof[te] = fitpred(X[tr], y[tr], X[te], seed0 + i)
        if (i + 1) % 5 == 0:                                   # one full pass over the data
            runs.append(metrics(y, oof, scen))
    return summarise(runs)


def logo_scores(X, y, scen, groups, fitpred):
    oof = np.zeros(len(y))
    for tr, te in LeaveOneGroupOut().split(X, y, groups):
        if len(set(y[tr])) < 2:
            return None
        oof[te] = fitpred(X[tr], y[tr], X[te], 0)
    return metrics(y, oof, scen)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('sessions', nargs='+')
    ap.add_argument('--targets', nargs='+', default=['y_success', 'y_stable'])
    ap.add_argument('--ends', nargs='+', type=float, default=[0.4, 0.6, 0.8, 1.0, 1.5])
    ap.add_argument('--repeats', type=int, default=5)
    ap.add_argument('--lstm-repeats', type=int, default=2)
    ap.add_argument('--no-lstm', action='store_true')
    ap.add_argument('--out', default='ml/results')
    ap.add_argument('--save-merged', default=None, help='write the merged dataset to this dir')
    a = ap.parse_args()

    X, lab, ch = load_sessions(a.sessions)
    print(f'{len(lab)} trials from {len(a.sessions)} sessions, X {X.shape}')
    if a.save_merged:
        save_merged(a.save_merged, X, lab, ch)
        print(f'merged dataset written to {a.save_merged}')
    scen = lab['scenario'].to_numpy()
    groups = lab['session'].to_numpy()
    results = {'n_trials': len(lab), 'sessions': a.sessions, 'cv': {}, 'logo': {}}
    models = [('svm', fit_predict_svm, a.repeats)]
    if not a.no_lstm:
        models.append(('lstm', fit_predict_lstm, a.lstm_repeats))

    for target in a.targets:
        keep = lab[target].notna().to_numpy()
        y = lab[target][keep].astype(int).to_numpy()
        Xt, st, sc, gr = X[keep], strata(lab[keep], target), scen[keep], groups[keep]
        print(f'\n=== {target}: {keep.sum()} trials, positive rate {y.mean():.2f} ===')
        for end in a.ends:
            n = int(round(end * RATE_HZ))
            for name, fp, reps in models:
                t = time.time()
                r = cv_scores(Xt[:, :n], y, sc, st, fp, reps)
                results['cv'][f'{target}|{name}|{end}'] = r
                p = r['pooled']
                line = (f'{target} {name:4} end {end:3.1f}s  pooled acc {p["acc"][0]:.2f} '
                        f'(maj {p["majority"][0]:.2f}) AUC {p["auc"][0]:.2f}+-{p["auc"][1]:.2f}')
                for s in 'AB':
                    if s in r:
                        line += f' | {s} AUC {r[s]["auc"][0]:.2f} acc {r[s]["acc"][0]:.2f}/{r[s]["majority"][0]:.2f}'
                print(line + f'  ({time.time() - t:.0f}s)', flush=True)
            r = logo_scores(Xt[:, :n], y, sc, gr, fit_predict_svm)
            results['logo'][f'{target}|svm|{end}'] = r
            if r:
                print(f'   leave-one-session-out svm end {end:3.1f}s: pooled AUC {r["pooled"]["auc"]:.2f}'
                      + ''.join(f' {s} {r[s]["auc"]:.2f}' for s in 'AB' if s in r), flush=True)

    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'baselines.json'), 'w') as f:
        json.dump(results, f, indent=1)
    print(f'\nwrote {a.out}/baselines.json')


if __name__ == '__main__':
    main()
