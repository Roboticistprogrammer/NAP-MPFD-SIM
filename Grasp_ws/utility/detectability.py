#!/usr/bin/env python3
"""M1 detectability check: is the payload visible in the drone's own telemetry?

For every valid A/B trial and every E (empty-gripper) control, cut the same 50 Hz window
starting at t0 (start of the small lift) plus the 0.5 s hover before it, from the ULog.
E flies the same motion without a payload, so (A/B window - E window) isolates the payload.

  step  = |mean over the window of (channel - pre-t0 hover mean)| for payload trials
          minus the same for E trials
  noise = pooled within-window standard deviation of the channel during the 0.5 s hover
          (the plan's "hover-noise standard deviation")
  ratio = step / noise            gate (08_small_lift_plan.md, M1): at least 3, per scenario

Also reported: the ratio against the E-to-E scatter of the same statistic (a stricter bar,
since it includes trial-to-trial variation), and a second row restricted to the first
0.8 s of the window (before most grasps fail, about 1.0 s after t0).

usage: python3 detectability.py <session dir> [<session dir> ...]
(needs pyulog; run it with PYTHONPATH=src/px4_gripper_missions)
"""
import csv
import json
import os
import sys

import numpy as np

from px4_gripper_missions.windows import extract_window, load_channel_spec, load_ulog_channels

CHANNELS = ['thrust_sp_z', 'motor_mean', 'torque_sp_x', 'torque_sp_y', 'acc_z', 'lpos_az']
RATE, N_SL, BASE_S = 50.0, 75, 0.5


def session_windows(session_dir):
    spec = load_channel_spec()
    names = [c['name'] for c in spec['inputs']]
    session = json.load(open(os.path.join(session_dir, 'session.json')))
    trials = list(csv.DictReader(open(os.path.join(session_dir, 'trials.csv'))))
    series = load_ulog_channels(session['ulog'], spec)
    offset = float(session.get('px4_time_offset_s', 0.0))
    n_base = int(round(BASE_S * RATE))
    out = []
    for tr in trials:
        if tr['valid'] != 'True':
            continue
        t0 = float(tr['t_lift'])
        win = extract_window(series, names, t0, N_SL, RATE, offset)
        base = extract_window(series, names, t0 - BASE_S, n_base, RATE, offset)
        if np.isnan(win).any() or np.isnan(base).any():
            continue
        out.append(dict(scenario=tr['scenario'], held=tr['y_success'] == '1',
                        win=win, base=base, names=names))
    return out


def channel(x, names, ch):
    if ch == 'motor_mean':
        return np.mean([x[:, names.index(f'motor_{i}')] for i in range(4)], axis=0)
    return x[:, names.index(ch)]


def step_and_noise(trials, ch, upto):
    """Per-trial mean of (window - hover mean) over the first `upto` samples, and hover noise."""
    steps, noise = [], []
    for t in trials:
        names = t['names']
        b, w = channel(t['base'], names, ch), channel(t['win'], names, ch)
        steps.append(float(np.mean(w[:upto] - b.mean())))
        noise.append(float(np.var(b, ddof=1)))
    return np.array(steps), float(np.sqrt(np.mean(noise)))


def main(sessions):
    allw = []
    for s in sessions:
        print(f'loading {s}', file=sys.stderr)
        allw += session_windows(s)
    empty = [t for t in allw if t['scenario'] == 'E']
    print(f'E controls: {len(empty)}; payload trials: '
          f'A {sum(t["scenario"] == "A" for t in allw)}, B {sum(t["scenario"] == "B" for t in allw)}')
    for label, upto, pick in (('whole 1.5 s window, held grasps only', N_SL, lambda t: t['held']),
                              ('first 0.8 s, all valid trials', int(0.8 * RATE), lambda t: True)):
        print(f'\n== {label} ==')
        print(f'{"scen":4} {"channel":12} {"E step":>10} {"payload step":>13} {"diff":>10} '
              f'{"noise":>9} {"diff/noise":>10} {"diff/E-scatter":>14}')
        for scen in 'AB':
            pay = [t for t in allw if t['scenario'] == scen and pick(t)]
            for ch in CHANNELS:
                e, noise = step_and_noise(empty, ch, upto)
                p, _ = step_and_noise(pay, ch, upto)
                diff = abs(p.mean() - e.mean())
                sc = e.std(ddof=1)
                print(f'{scen:4} {ch:12} {e.mean():10.4g} {p.mean():13.4g} {diff:10.4g} '
                      f'{noise:9.3g} {diff / noise:10.1f} {diff / sc:14.1f}  (n={len(pay)})')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
