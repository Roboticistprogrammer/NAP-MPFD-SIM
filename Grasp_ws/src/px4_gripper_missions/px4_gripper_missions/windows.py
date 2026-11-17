"""Cut a small_lift session into model-ready windows.

    build_windows sim_data/<session> [--out DIR] [--sl-s 1.5] [--rate 50]

Per trial (scenario A or B, valid, labelled), on a uniform grid anchored at
the trial's own phase times:
  SL window  t0 .. t0 + sl_s, t0 = start of the small lift (75 samples at 50 Hz)
  CG window  t_close .. t_close + close_wait_s (the gripper-closing phase)
  baseline   mean of the `baseline_s` hover just before each window

Each grid sample is the mean of the PX4 samples inside its bin; a bin with no
sample (slow topics, e.g. hover_thrust_estimate) holds the last earlier value.
Empty-gripper (E) trials carry no label and are left out.

Writes <session>/dataset/:
    windows.npz   X_sl (N, n_sl, C), X_cg (N, n_cg, C), base_sl, base_cg (N, C),
                  trial_id (N,), channels (C,)   - raw values, nothing normalised
    labels.parquet  one row per trial: labels, timings, privileged parameters and
                    the static Tier-1 gripper inputs
    meta.json     channels, window geometry, session provenance, dropped trials

Channels are defined in config/channels.yaml. Needs pyulog, pandas, pyarrow.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from px4_gripper_missions.config_loader import get_config_dir

Series = Tuple[np.ndarray, np.ndarray]     # (time [s], values)


def default_channels_path() -> str:
    """config/channels.yaml of the source tree when running from a checkout (the
    scripts run the package from source), else the installed copy - which can be
    older than the code after an edit without `colcon build`."""
    src = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'config', 'channels.yaml')
    return src if os.path.exists(src) else os.path.join(get_config_dir(), 'channels.yaml')


def load_channel_spec(path: Optional[str] = None) -> dict:
    path = path or default_channels_path()
    with open(path) as f:
        return yaml.safe_load(f)


def quat_to_roll_pitch(q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Roll and pitch [rad] from quaternions (rows of w, x, y, z)."""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    return roll, pitch


def load_ulog_channels(ulog_path: str, spec: dict) -> Dict[str, Series]:
    """Read every input channel of `spec` from the ULog, keyed by channel name."""
    from pyulog import ULog
    topics = sorted({c['topic'] for c in spec['inputs']})
    ulog = ULog(ulog_path, topics)
    out: Dict[str, Series] = {}
    for ch in spec['inputs']:
        ds = next((d for d in ulog.data_list if d.name == ch['topic'] and d.multi_id == 0), None)
        if ds is None:
            raise KeyError(f"topic {ch['topic']} (channel {ch['name']}) not in {ulog_path}")
        t = ds.data['timestamp'] * 1e-6
        if 'derived' in ch:
            q = np.column_stack([ds.data[f'q[{i}]'] for i in range(4)]).astype(float)
            roll, pitch = quat_to_roll_pitch(q)
            out[ch['name']] = (t, {'roll': roll, 'pitch': pitch}[ch['derived']])
        else:
            out[ch['name']] = (t, ds.data[ch['field']].astype(float))
    return out


def bin_average(t_src: np.ndarray, v_src: np.ndarray, t_start: float, n: int,
                dt: float) -> np.ndarray:
    """n samples; sample k is the mean of the source values in
    [t_start + k dt, t_start + (k+1) dt). An empty bin holds the last earlier
    source value (NaN if there is none)."""
    edges = t_start + dt * np.arange(n + 1)
    idx = np.searchsorted(t_src, edges, side='left')      # first source index >= edge
    out = np.empty(n)
    for k in range(n):
        lo, hi = idx[k], idx[k + 1]
        if hi > lo:
            out[k] = v_src[lo:hi].mean()
        elif lo > 0:
            out[k] = v_src[lo - 1]
        else:
            out[k] = np.nan
    return out


def extract_window(series: Dict[str, Series], names: Sequence[str], t_start: float, n: int,
                   rate_hz: float, time_offset_s: float = 0.0) -> np.ndarray:
    """(n, len(names)) window starting at sim time `t_start`."""
    dt = 1.0 / rate_hz
    cols = [bin_average(series[nm][0] - time_offset_s, series[nm][1], t_start, n, dt)
            for nm in names]
    return np.column_stack(cols)


def _num(x: str, default: float = float('nan')) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def build_windows(session_dir: str, out_dir: Optional[str] = None, spec_path: Optional[str] = None,
                  sl_s: float = 1.5, rate_hz: float = 50.0, baseline_s: float = 0.5,
                  ulog_path: Optional[str] = None) -> dict:
    """Build and write the dataset for one session; returns the meta dict."""
    import pandas as pd

    spec = load_channel_spec(spec_path)
    names = [c['name'] for c in spec['inputs']]
    with open(os.path.join(session_dir, 'session.json')) as f:
        session = json.load(f)
    with open(os.path.join(session_dir, 'trials.csv')) as f:
        trials = list(csv.DictReader(f))
    if not trials or 'y_success' not in trials[0]:
        raise ValueError(f'{session_dir} is not a small_lift session (no y_success column)')
    ulog_path = ulog_path or session.get('ulog')
    if not ulog_path or not os.path.exists(ulog_path):
        raise FileNotFoundError(f'ULog not found ({ulog_path!r}); pass --ulog')

    cfg = session['config']
    offset = float(session.get('px4_time_offset_s', 0.0))
    n_sl = int(round(sl_s * rate_hz))
    n_cg = int(round(float(cfg['motion']['close_wait_s']) * rate_hz))
    n_base = int(round(baseline_s * rate_hz))
    reach = cfg['arm']['reach_pose']
    series = load_ulog_channels(ulog_path, spec)

    kept: List[dict] = []
    x_sl, x_cg, b_sl, b_cg = [], [], [], []
    dropped: List[dict] = []
    for tr in trials:
        tid = f"{session['session']}_t{int(tr['index']):04d}"
        if tr['scenario'] == 'E':
            continue                                         # no label: nothing was grasped
        if tr['valid'] != 'True' or tr['y_success'] == '':
            dropped.append(dict(trial_id=tid, reason=tr.get('invalid_reason') or 'unlabelled'))
            continue
        t0, t_close = _num(tr['t_lift']), _num(tr['t_close'])
        sl = extract_window(series, names, t0, n_sl, rate_hz, offset)
        cg = extract_window(series, names, t_close, n_cg, rate_hz, offset)
        base_sl = extract_window(series, names, t0 - baseline_s, n_base, rate_hz, offset)
        base_cg = extract_window(series, names, t_close - baseline_s, n_base, rate_hz, offset)
        if any(np.isnan(a).any() for a in (sl, cg, base_sl, base_cg)):
            dropped.append(dict(trial_id=tid, reason='NaN in window (ULog gap?)'))
            continue
        x_sl.append(sl)
        x_cg.append(cg)
        b_sl.append(base_sl.mean(axis=0))
        b_cg.append(base_cg.mean(axis=0))
        kept.append(dict(
            trial_id=tid, session=session['session'], index=int(tr['index']),
            battery=int(tr['battery']), scenario=tr['scenario'],
            y_success=int(tr['y_success']),
            y_stable=int(tr['y_stable']) if tr['y_stable'] != '' else pd.NA,
            failure_mode=tr['failure_mode'], t_onset_s=_num(tr['t_onset_s']),
            t_drop_s=_num(tr['t_drop_s']), shake_failure_mode=tr['shake_failure_mode'],
            shake_t_drop_s=_num(tr['shake_t_drop_s']),
            t0=t0, t_close=t_close, t_sl_end=_num(tr['t_sl_end']), t_shake=_num(tr['t_shake']),
            grip_force_n=_num(tr['grip_force_n']),
            grasp_offset_actual_m=_num(tr['grasp_offset_actual_m']),
            grip_state=1, arm_shoulder_cmd=float(reach['shoulder_pitch']),
            arm_elbow_cmd=float(reach['elbow_pitch'])))

    out_dir = out_dir or os.path.join(session_dir, 'dataset')
    os.makedirs(out_dir, exist_ok=True)
    c = len(names)
    np.savez_compressed(
        os.path.join(out_dir, 'windows.npz'),
        X_sl=np.array(x_sl).reshape(-1, n_sl, c), X_cg=np.array(x_cg).reshape(-1, n_cg, c),
        base_sl=np.array(b_sl).reshape(-1, c), base_cg=np.array(b_cg).reshape(-1, c),
        trial_id=np.array([k['trial_id'] for k in kept]), channels=np.array(names))
    labels = pd.DataFrame(kept)
    if len(labels):
        labels['y_stable'] = labels['y_stable'].astype('Int64')
    labels.to_parquet(os.path.join(out_dir, 'labels.parquet'), index=False)
    meta = dict(
        session=session['session'], variant=session.get('variant'),
        git_commit=session.get('git_commit'), config_sha256=session.get('config_sha256'),
        ulog=ulog_path, channels=names, rate_hz=rate_hz, n_sl=n_sl, n_cg=n_cg,
        baseline_s=baseline_s, n_trials=len(kept), dropped=dropped,
        y_success_counts={str(k): int(v) for k, v in
                          labels['y_success'].value_counts().items()} if len(labels) else {},
        y_stable_counts={str(k): int(v) for k, v in
                         labels['y_stable'].dropna().value_counts().items()} if len(labels) else {})
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=2)
    return meta


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('session', help='sim_data/<session> directory (small_lift variant)')
    ap.add_argument('--out', help='output directory (default <session>/dataset)')
    ap.add_argument('--ulog', help='override the ULog path recorded in session.json')
    ap.add_argument('--channels', help='channels.yaml (default: installed package config)')
    ap.add_argument('--sl-s', type=float, default=1.5, help='SL window length [s]')
    ap.add_argument('--rate', type=float, default=50.0, help='window grid rate [Hz]')
    args = ap.parse_args(argv)
    try:
        meta = build_windows(args.session, args.out, args.channels, args.sl_s, args.rate,
                             ulog_path=args.ulog)
    except (ValueError, FileNotFoundError, KeyError) as e:
        print(f'error: {e}', file=sys.stderr)
        return 1
    print(f"{meta['n_trials']} trials, {len(meta['channels'])} channels, "
          f"SL window {meta['n_sl']} samples, CG window {meta['n_cg']} samples")
    print(f"y_success {meta['y_success_counts']}  y_stable {meta['y_stable_counts']}")
    for d in meta['dropped']:
        print(f"  dropped {d['trial_id']}: {d['reason']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
