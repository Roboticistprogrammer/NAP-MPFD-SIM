"""Load and merge the per-session datasets written by `experiment.sh dataset <session>`.

Each session has dataset/windows.npz (X_sl: trials x 75 x 21 at 50 Hz, base_sl: pre-SL hover
mean per channel) and dataset/labels.parquet. `load_sessions` concatenates them, subtracts the
hover mean (the plan's preprocessing) and keeps the session name as a column, so folds can be
grouped by session (each session is a separate sim start).

Never returned as inputs: scenario, grip force, offsets (see result.md section 12). They stay in
the labels table for stratifying folds and for analysis only.
"""
import os
from typing import List, Sequence, Tuple

import numpy as np
import pandas as pd

RATE_HZ = 50.0


def load_sessions(session_dirs: Sequence[str]) -> Tuple[np.ndarray, pd.DataFrame, List[str]]:
    xs, labs, names = [], [], None
    for s in session_dirs:
        d = np.load(os.path.join(s, 'dataset', 'windows.npz'), allow_pickle=True)
        lab = pd.read_parquet(os.path.join(s, 'dataset', 'labels.parquet'))
        if list(d['trial_id']) != list(lab['trial_id']):
            raise ValueError(f'{s}: windows.npz and labels.parquet are out of step')
        ch = [str(c) for c in d['channels']]
        if names is not None and ch != names:
            raise ValueError(f'{s}: channel list differs from the first session')
        names = ch
        xs.append(d['X_sl'] - d['base_sl'][:, None, :])
        labs.append(lab)
    return np.concatenate(xs), pd.concat(labs, ignore_index=True), names


def save_merged(path: str, X: np.ndarray, labels: pd.DataFrame, channels: List[str]) -> None:
    os.makedirs(path, exist_ok=True)
    np.savez_compressed(os.path.join(path, 'windows.npz'), X_sl=X, channels=np.array(channels),
                        trial_id=labels['trial_id'].to_numpy())
    labels.to_parquet(os.path.join(path, 'labels.parquet'), index=False)


def strata(labels: pd.DataFrame, target: str) -> np.ndarray:
    """Fold strata: scenario x label (the plan's stratification)."""
    return (labels['scenario'].eq('B').astype(int) * 2 + labels[target].astype(int)).to_numpy()
