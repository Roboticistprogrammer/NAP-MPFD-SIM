"""Build Grasp_ws/zimmer-presentation.html: a standalone 1920x1080 slide deck whose charts are
drawn from the session data in sim_data (session names are listed in FINAL and in the slide code).
Re-run after new sessions:  python3 utility/build_presentation.py   (needs numpy, pandas, pyarrow)"""
import csv
import html
import math
import os
import numpy as np

WS = '/home/roboticistprogrammer/Projects/px4_gazebo_sim_setup/Grasp_ws/'
SD = WS + 'sim_data/'
OUT = WS + 'zimmer-presentation.html'

C = dict(navy='#16202E', navy2='#243246', light='#F6F5F0', card='#FDFCF9', border='#DDDAD0',
         ink='#1E2530', soft='#4A5568', orange_l='#B8501A', orange_d='#F08A4B', teal_l='#1F6F7F',
         teal_d='#6CC3D1', mute_d='#C3CCD8', grid_d='#33445C', grid_l='#DDDAD0', good='#2E7D4F')
TH = {'dark': dict(fg='#F6F5F0', muted='#C3CCD8', grid='#33445C', a=C['orange_d'], b=C['teal_d'],
                   c='#E8C547', axis='#8091A8'),
      'light': dict(fg=C['ink'], muted=C['soft'], grid=C['grid_l'], a=C['orange_l'], b=C['teal_l'],
                    c='#8A6D00', axis='#6A7179')}


# ----------------------------------------------------------------------------- data
def rows(session):
    return list(csv.DictReader(open(SD + session + '/trials.csv')))


def lag_series(session, idx, t_max=4.0):
    tr = {r['index']: r for r in rows(session)}[str(idx)]
    g = list(csv.DictReader(open(f"{SD}{session}/{tr['csv']}")))
    t = np.array([float(r['t_sim']) for r in g]); cz = np.array([float(r['cmd_z']) for r in g])
    dz = np.array([float(r['drone_z']) for r in g]); t0 = float(tr['t_lift'])
    k0 = int(np.argmin(abs(t - t0)))
    m = (t >= t0) & (t <= t0 + t_max)
    return t[m] - t0, (cz[m] - cz[k0]) * 1000, (dz[m] - dz[k0]) * 1000


FINAL = ['20261016_231647_protocol', '20261016_232201_tuneA', '20261016_233436_tuneB',
         '20261016_235828_tuneA', '20261017_000309_tuneA', '20261017_001614_tuneB',
         '20261017_003053_protocol', '20261017_004908_protocol']   # 0.15 m / 1.5 s, feed-forward


def held_vs_force():
    d = {'A': [], 'B': []}
    drops = []
    for s in FINAL:
        for r in rows(s):
            if r['valid'] == 'True' and r['y_success'] in ('0', '1') and r['scenario'] in d:
                d[r['scenario']].append((float(r['grip_force_n']), int(r['y_success'])))
                if r['y_success'] == '0':
                    drops.append(float(r['t_drop_s']))
    return d, drops


def logit_fit(x, y):
    X = np.column_stack([np.ones_like(x), x - x.mean()]); w = np.zeros(2)
    for _ in range(60):
        p = 1 / (1 + np.exp(-X @ w)); W = p * (1 - p) + 1e-9
        w += np.linalg.solve(X.T @ (X * W[:, None]) + 1e-6 * np.eye(2), X.T @ (y - p))
    return lambda f: 1 / (1 + np.exp(-(w[0] + w[1] * (f - x.mean()))))


def window_curves():
    import pandas as pd
    d = SD + '20261017_004908_protocol/dataset/'
    z = np.load(d + 'windows.npz'); lab = pd.read_parquet(d + 'labels.parquet')
    ch = list(z['channels']); i = ch.index('thrust_sp_z')
    base = z['base_sl'][:, i]
    pct = -(z['X_sl'][:, :, i] - base[:, None]) / np.abs(base[:, None]) * 100     # % thrust increase
    y = lab.y_success.to_numpy()
    fail = lab[lab.y_success == 0].t_drop_s.to_numpy()
    return pct[y == 1], pct[y == 0], fail, lab


# ----------------------------------------------------------------------------- svg helpers
def esc(s):
    return html.escape(str(s), quote=True)


def chart(w, h, xr, yr, series, xticks, yticks, xlabel='', ylabel='', th='light', vlines=(), shades=(),
          notes=(), margin=(110, 30, 40, 84), fmt_y=lambda v: f'{v:g}', fmt_x=lambda v: f'{v:g}', fs=26):
    t = TH[th]; ml, mr, mt, mb = margin
    pw, ph = w - ml - mr, h - mt - mb
    X = lambda v: ml + (v - xr[0]) / (xr[1] - xr[0]) * pw
    Y = lambda v: mt + (1 - (v - yr[0]) / (yr[1] - yr[0])) * ph
    cid = f'c{abs(hash((w, h, xr, yr, th, len(series)))) % 10**8}{len(slides)}'
    o = [f'<svg viewBox="0 0 {w} {h}" width="{w}" height="{h}" role="img" font-family="IBM Plex Sans, Arial, sans-serif">'
         f'<defs><clipPath id="{cid}"><rect x="{ml}" y="{mt}" width="{pw}" height="{ph}"/></clipPath></defs>']
    for a, b, col, op in shades:
        o.append(f'<rect x="{X(a):.1f}" y="{mt}" width="{X(b) - X(a):.1f}" height="{ph}" fill="{col}" opacity="{op}"/>')
    for v in yticks:
        o.append(f'<line x1="{ml}" x2="{ml + pw}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="{t["grid"]}" stroke-width="1.5"/>')
        o.append(f'<text x="{ml - 14}" y="{Y(v) + 9:.1f}" text-anchor="end" font-size="{fs}" fill="{t["muted"]}">{fmt_y(v)}</text>')
    for v in xticks:
        o.append(f'<text x="{X(v):.1f}" y="{mt + ph + 40}" text-anchor="middle" font-size="{fs}" fill="{t["muted"]}">{fmt_x(v)}</text>')
    o.append(f'<line x1="{ml}" x2="{ml + pw}" y1="{mt + ph}" y2="{mt + ph}" stroke="{t["axis"]}" stroke-width="2"/>')
    o.append(f'<line x1="{ml}" x2="{ml}" y1="{mt}" y2="{mt + ph}" stroke="{t["axis"]}" stroke-width="2"/>')
    for vl in vlines:
        v, col, lab, dash = vl[:4]; ly = mt + ph - 14 if len(vl) > 4 and vl[4] == 'bottom' else mt + 30
        o.append(f'<line x1="{X(v):.1f}" x2="{X(v):.1f}" y1="{mt}" y2="{mt + ph}" stroke="{col}" stroke-width="3" stroke-dasharray="{dash}"/>')
        if lab:
            o.append(f'<text x="{X(v) + 10:.1f}" y="{ly}" font-size="{fs - 2}" fill="{col}">{esc(lab)}</text>')
    o.append(f'<g clip-path="url(#{cid})">')
    for s in series:
        pts = s['pts']
        if s.get('band'):
            up = ' '.join(f'{X(px):.1f},{Y(py + s["band"][k]):.1f}' for k, (px, py) in enumerate(pts))
            dn = ' '.join(f'{X(px):.1f},{Y(py - s["band"][k]):.1f}' for k, (px, py) in reversed(list(enumerate(pts))))
            o.append(f'<polygon points="{up} {dn}" fill="{s["color"]}" opacity="0.18"/>')
        if s.get('dots'):
            for px, py, r in pts:
                o.append(f'<circle cx="{X(px):.1f}" cy="{Y(py):.1f}" r="{r}" fill="{s["color"]}" opacity="0.85"/>')
        else:
            p = ' '.join(f'{X(px):.1f},{Y(py):.1f}' for px, py in pts)
            o.append(f'<polyline points="{p}" fill="none" stroke="{s["color"]}" stroke-width="{s.get("width", 4)}" '
                     f'stroke-dasharray="{s.get("dash", "")}" stroke-linejoin="round" stroke-linecap="round"/>')
    o.append('</g>')
    for nt in notes:
        x, y, txt, col, anchor = nt[:5]
        halo = (f' stroke="{nt[5]}" stroke-width="9" paint-order="stroke" stroke-linejoin="round"' if len(nt) > 5 else '')
        o.append(f'<text x="{X(x):.1f}" y="{Y(y):.1f}" text-anchor="{anchor}" font-size="{fs}" fill="{col}"{halo}>{esc(txt)}</text>')
    if xlabel:
        o.append(f'<text x="{ml + pw / 2}" y="{h - 8}" text-anchor="middle" font-size="{fs}" fill="{t["muted"]}">{esc(xlabel)}</text>')
    if ylabel:
        o.append(f'<text transform="translate(26 {mt + ph / 2}) rotate(-90)" text-anchor="middle" font-size="{fs}" fill="{t["muted"]}">{esc(ylabel)}</text>')
    o.append('</svg>')
    return ''.join(o)


def legend(items, th):
    t = TH[th]
    return '<div class="legend">' + ''.join(
        f'<span><i style="background:{c}"></i>{esc(l)}</span>' for l, c in items) + '</div>'


def hbars(items, th, w=1000, maxv=100, unit=' %', hl=None, fs=26):
    """items: (label, value, annotation). Horizontal bars."""
    t = TH[th]; row = 66; lw = 440; h = row * len(items) + 30
    o = [f'<svg viewBox="0 0 {w} {h}" width="{w}" height="{h}" font-family="IBM Plex Sans, Arial, sans-serif">']
    for k, (lab, v, ann) in enumerate(items):
        y = 10 + k * row; bw = (w - lw - 190) * v / maxv
        col = t['a'] if (hl is not None and k == hl) else t['b'] if v >= 60 else '#8091A8'
        wt = '600' if k == hl else '400'
        o.append(f'<text x="{lw - 14}" y="{y + 36}" text-anchor="end" font-size="{fs}" fill="{t["fg"]}" font-weight="{wt}">{esc(lab)}</text>')
        o.append(f'<rect x="{lw}" y="{y + 8}" width="{max(bw, 3):.1f}" height="38" rx="6" fill="{col}"/>')
        o.append(f'<text x="{lw + bw + 12:.1f}" y="{y + 36}" font-size="{fs}" fill="{t["fg"]}" font-weight="600">{v:g}{unit}</text>')
        o.append(f'<text x="{w - 4}" y="{y + 36}" text-anchor="end" font-size="{fs - 2}" fill="{t["muted"]}">{esc(ann)}</text>')
    o.append('</svg>')
    return ''.join(o)


# ----------------------------------------------------------------------------- slide html
slides = []


def slide(theme, body, notes='', foot=''):
    n = len(slides) + 1
    f = f'<div class="foot">{foot}</div>' if foot else ''
    slides.append(f'<section class="slide {theme}" id="s{n}"><div class="pad">{body}</div>{f}'
                  f'<aside>{esc(notes)}</aside></section>')


def card(inner, cls=''):
    return f'<div class="card {cls}">{inner}</div>'


def table(head, body_rows, th_cls=''):
    h = ''.join(f'<th>{x}</th>' for x in head)
    b = ''.join('<tr>' + ''.join(f'<td>{x}</td>' for x in r) + '</tr>' for r in body_rows)
    return f'<table><tr>{h}</tr>{b}</table>'


# ---- 1 title
slide('dark', f'''
<p class="eyebrow">Progress update · NAP-MPFD-SIM</p>
<h1>Predicting grasp outcome from a short lift</h1>
<p class="lead">Adapting Zimmer et al. (2019) to a UAV-mounted gripper: protocol, simulation tests, a 150-trial dataset and what it can and cannot predict</p>
<div class="spacer"></div>
<p class="small">PX4 SITL · Gazebo Harmonic · x500 quadrotor with Armbot gripper · November 2026</p>
''', 'Where the Zimmer-style small-lift protocol stands in simulation: what was built, the 150-trial dataset, what the models can and cannot predict, and the decisions that are open.')

# ---- 2 research question
slide('light', f'''
<h2>Research question</h2>
<p class="statement">Can a drone's own flight data, recorded during a short standard lift, tell us whether a grasp will hold?</p>
<div class="cards3">
{card('<h3>Input</h3><p>Telemetry the real drone already has: thrust, torque, motors, IMU, attitude, vertical state, plus the commanded gripper state (Tier-1).</p>')}
{card('<h3>Output</h3><p><b>y_success</b>: the object survives the full lift.<br><b>y_stable</b>: it survives a lateral shake.</p>')}
{card('<h3>Why it matters</h3><p>The real gripper has open-loop servos and no force feedback. A prediction early in the lift could trigger abort and re-grasp before the payload is dropped.</p>')}
</div>
''', 'The grasp quality is not observable directly on the real arm; the drone dynamics are the only sensor we have in the base setup.')


# ---- later work: result.md sections 9-17 (numbers are read from the saved results, not typed)
import json
import pandas as pd

REPO = os.path.abspath(WS + '..') + '/'
BL = json.load(open(REPO + 'ml/results/baselines.json'))
SL25 = json.load(open(REPO + 'ml/results/sl25_sweep/baselines.json'))
EXTRA = json.load(open(REPO + 'ml/results/extra_signals.json'))
BATCH = pd.read_parquet(SD + 'merged_batch1/labels.parquet')
FAILED = BATCH[BATCH.y_success == 0]
SL25_SESS = ['20261114_200141_tuneA', '20261114_201351_tuneB']


def au(R, tgt, mdl, end, sub='pooled'):
    return R['cv'][f'{tgt}|{mdl}|{end}'][sub]['auc'][0]



# ---- 2b the short version
_y = [au(BL, 'y_success', 'svm', e) for e in (0.4, 0.6, 0.8, 1.0)]
slide('dark', f'''
<h2>The short version</h2>
<div class="cards3">
{card(f'<h3 class="tealh">What works</h3><p>The payload is clearly visible in the drone data (26–210× the hover noise).<br><br>150 trials, balanced labels, every acceptance gate passed.<br><br>A failing grasp is detected with AUC {_y[3]:.2f} once the window reaches 1.0 s.</p>', 'dk')}
{card(f'<h3 class="orh">What does not (yet)</h3><p>Early warning: AUC {_y[0]:.2f} / {_y[1]:.2f} / {_y[2]:.2f} at 0.4 / 0.6 / 0.8 s, a lead of only 0.2–0.4 s.<br><br>y_stable is not predictable from the small lift.<br><br>Six ways to buy lead time did not help.</p>', 'dk')}
{card('<h3>What decides next</h3><p>Is detection enough, or is a grasp-sensing input needed?<br><br>A real-drone check of the signal level.<br><br>Questions at the end.</p>', 'dk')}
</div>
''', 'The one-slide summary for people who only have time for it. Numbers are from the 150-trial batch (131 valid A/B trials): detection is the AUC of an SVM on the 21 drone channels with the window cut at the given time.', '')


zphases = (
 '<svg viewBox="0 0 1664 190" width="1664" height="190" font-family="IBM Plex Sans, Arial, sans-serif">'
 '<rect x="0" y="40" width="330" height="90" rx="12" fill="#8091A8"/><text x="165" y="94" text-anchor="middle" font-size="30" fill="#fff" font-weight="500">close gripper (CG)</text>'
 '<rect x="350" y="40" width="330" height="90" rx="12" fill="#B8501A"/><text x="515" y="94" text-anchor="middle" font-size="30" fill="#fff" font-weight="600">small lift (SL)</text>'
 '<text x="515" y="30" text-anchor="middle" font-size="26" fill="#B8501A" font-weight="600">1.5 s, 10 mm: best predictor</text>'
 '<rect x="700" y="40" width="450" height="90" rx="12" fill="#1F6F7F"/><text x="925" y="94" text-anchor="middle" font-size="30" fill="#fff" font-weight="500">full lift: success?</text>'
 '<rect x="1170" y="40" width="494" height="90" rx="12" fill="#7A4FA0"/><text x="1417" y="94" text-anchor="middle" font-size="30" fill="#fff" font-weight="500">wrist shake: stable?</text>'
 '<text x="0" y="170" font-size="26" fill="#4A5568">Classifier input: finger IMU and pressure (30 Hz) from one of the first two phases; labels from the last two.</text></svg>')

# ---- 3 Zimmer
slide('light', f'''
<h2>Zimmer et al. 2019: a short lift predicts grasp success</h2>
<div class="cards3">
{card('<p class="big or">1.5 s</p><p class="cap">small-lift (SL) phase</p>')}
{card('<p class="big or">10 mm</p><p class="cap">lift height</p>')}
{card('<p class="big or">~91–92 %</p><p class="cap">full-lift success predicted</p>')}
</div>
{zphases}
<ul>
<li>The SL window predicts better than the gripper-closing (CG) phase.</li>
<li>Soft robotic hand with a data-driven classifier; success = object still in hand after the lift.</li>
</ul>
''', 'Figures as summarised in our planning brief, not from a fresh read of the paper.',
      'Zimmer, Hellebrekers, Asfour, Majidi, Kroemer (2019). Predicting Grasp Success with a Soft Robotic Hand using a Data-Driven Classifier.')

# ---- 4 mapping
slide('dark', f'''
<h2>Mapping the protocol onto a UAV gripper</h2>
<div class="bigtable">{table(['Zimmer et al.', 'Our adaptation'], [
    ['Arm lifts 10 mm in 1.5 s', 'Drone climbs <b>0.15 m in 1.5 s</b> (minimum-jerk, with velocity feed-forward)'],
    ['USB scale under the tray', 'None: labels come from the simulator (cylinder displacement and tilt)'],
    ['Finger IMU and pressure, 30 Hz', 'PX4 telemetry, <b>21 channels at 50 Hz</b>, 75 samples per window'],
    ['Success: object in hand after lift', '<b>y_success</b>: held after a full +1.0 m lift and 4 s hold'],
    ['Stability: survives wrist shake', '<b>y_stable</b>: survives a ±0.3 m lateral shake (only if held)'],
])}</div>
''', 'The F/T pedestal in the original brief was dropped: relative displacement of the cylinder gives the labels directly.',
      'Pressure or force sensing on the gripper stays an option for later (see the open design question).')

# ---- 5 testbed (with sketch)
sketch = '''
<svg viewBox="0 0 760 660" width="760" height="660" font-family="IBM Plex Sans, Arial, sans-serif">
 <rect x="0" y="0" width="760" height="660" rx="16" fill="#FDFCF9" stroke="#DDDAD0"/>
 <!-- drone -->
 <rect x="250" y="70" width="230" height="30" rx="8" fill="#16202E"/>
 <line x1="210" y1="62" x2="290" y2="62" stroke="#1F6F7F" stroke-width="6"/><line x1="440" y1="62" x2="520" y2="62" stroke="#1F6F7F" stroke-width="6"/>
 <text x="365" y="52" text-anchor="middle" font-size="24" fill="#4A5568">x500 quadrotor</text>
 <!-- arm -->
 <polyline points="365,100 365,170 330,235 345,300" fill="none" stroke="#B8501A" stroke-width="12" stroke-linecap="round" stroke-linejoin="round"/>
 <rect x="318" y="296" width="14" height="52" rx="4" fill="#B8501A"/><rect x="358" y="296" width="14" height="52" rx="4" fill="#B8501A"/>
 <text x="395" y="190" font-size="24" fill="#B8501A">Armbot arm</text>
 <text x="395" y="222" font-size="24" fill="#B8501A">+ two-finger gripper</text>
 <!-- cylinder, table -->
 <circle cx="345" cy="338" r="22" fill="#1F6F7F"/>
 <rect x="200" y="368" width="320" height="18" fill="#8091A8"/><rect x="225" y="386" width="14" height="60" fill="#8091A8"/><rect x="480" y="386" width="14" height="60" fill="#8091A8"/>
 <text x="360" y="416" text-anchor="middle" font-size="24" fill="#4A5568">table + jig</text>
 <text x="440" y="344" font-size="24" fill="#1F6F7F">cylinder Ø 50 mm</text>
 <!-- top view -->
 <line x1="40" x2="720" y1="470" y2="470" stroke="#DDDAD0" stroke-width="2"/>
 <text x="40" y="512" font-size="24" fill="#4A5568">Top view of the cylinder (L = 120 mm)</text>
 <rect x="60" y="535" width="360" height="54" rx="27" fill="#1F6F7F" opacity="0.85"/>
 <line x1="240" y1="525" x2="240" y2="601" stroke="#B8501A" stroke-width="5"/><text x="240" y="636" text-anchor="middle" font-size="24" fill="#B8501A">A: centred</text>
 <line x1="330" y1="525" x2="330" y2="601" stroke="#16202E" stroke-width="5" stroke-dasharray="8 6"/><text x="430" y="636" text-anchor="middle" font-size="24" fill="#16202E">B: 30 mm off</text>
</svg>'''
slide('light', f'''
<h2>The simulated testbed</h2>
<div class="two">
<div>{sketch}</div>
<div>
<ul>
<li>PX4 SITL with Gazebo Harmonic; ground truth fed to EKF2 as emulated motion capture.</li>
<li>x500 with the Armbot arm and a force-controlled two-finger gripper.</li>
<li>One cylinder (r 25 mm, L 120 mm, 0.15 kg) on a table, held by two rails.</li>
<li><b>A</b>: grasp at the centre, fails by sliding out. <b>B</b>: grasp 30 mm off-centre, fails by pivoting round the fingers.</li>
<li><b>E</b>: empty-gripper control (not used for classification).</li>
</ul>
</div></div>
''', 'Everything runs headless; a trial takes about 30 s. Randomisation so far: grip force ±8 % and grasp position ±3 mm.',
      'Source: 07_grasp_experiment.md, experiment.yaml')

# ---- 6 timeline
def timeline(th):
    t = TH[th]
    legacy = [('base', 1), ('descend', 3), ('settle', 1), ('close', 1), ('lift', 1.3), ('hold', 4)]
    small = [('base', 1), ('descend', 3), ('settle', 1), ('close', 1), ('SL', 1.5), ('lift to +1.0 m', 5.3),
             ('hold', 4), ('shake', 6.5)]
    W, scale, x0 = 1664, 1664 / 24.0, 0
    cols = ['#5B6B82', '#5B6B82', '#5B6B82', '#8A6D00', '#B8501A', '#2F6FDE', '#1F6F7F', '#7A4FA0']
    o = [f'<svg viewBox="0 0 {W} 330" width="{W}" height="330" font-family="IBM Plex Sans, Arial, sans-serif">']
    for row, (name, seq) in enumerate([('Legacy', legacy), ('Small-lift', small)]):
        y = 20 + row * 150; x = 0
        o.append(f'<text x="0" y="{y}" font-size="28" font-weight="600" fill="{t["fg"]}">{name}</text>')
        for k, (lab, d) in enumerate(seq):
            w = d * scale
            col = cols[k] if name == 'Small-lift' else ['#5B6B82', '#5B6B82', '#5B6B82', '#8A6D00', '#2F6FDE', '#1F6F7F'][k]
            o.append(f'<rect x="{x:.1f}" y="{y + 14}" width="{w - 3:.1f}" height="74" rx="8" fill="{col}"/>')
            fs = 22 if w > 95 else 17
            txt = lab
            o.append(f'<text x="{x + (w - 3) / 2:.1f}" y="{y + 58}" text-anchor="middle" font-size="{fs}" fill="#fff" font-weight="500">{esc(txt)}</text>')
            o.append(f'<text x="{x + (w - 3) / 2:.1f}" y="{y + 114}" text-anchor="middle" font-size="22" fill="{t["muted"]}">{d:g} s</text>')
            x += w
    o.append('</svg>')
    return ''.join(o)


slide('dark', f'''
<h2>One trial, before and after</h2>
{timeline('dark')}
<ul>
<li><b>t₀</b> = start of the small lift (SL): the anchor of every window. Labels: y_success after the hold, y_stable after the shake.</li>
<li>The shake runs only if the cylinder is still held; durations are to scale, descend varies.</li>
</ul>
''', 'Legacy: single 0.40 m lift. Small-lift: 0.15 m in 1.5 s (SL), then a minimum-jerk rise to +1.0 m total, 4 s hold, then three 2 s legs of the lateral shake: centre to +0.3 m, to -0.3 m, back to centre.',
      'Source: experiment.yaml (small_lift, shake)')

scale7 = ('<svg viewBox="0 0 1664 170" width="1664" height="170" font-family="IBM Plex Sans, Arial, sans-serif">'
 '<line x1="40" x2="1620" y1="80" y2="80" stroke="#8091A8" stroke-width="6"/>'
 '<circle cx="40" cy="80" r="12" fill="#1F6F7F"/><text x="20" y="130" text-anchor="start" font-size="28" fill="#4A5568">0 mm: held</text>'
 '<circle cx="560" cy="80" r="12" fill="#8A6D00"/><text x="560" y="130" text-anchor="middle" font-size="28" fill="#8A6D00">10 mm: slip onset</text>'
 '<circle cx="1160" cy="80" r="12" fill="#B8501A"/><text x="1160" y="130" text-anchor="middle" font-size="28" fill="#B8501A">40 mm: dropped = failure</text>'
 '<text x="800" y="52" text-anchor="middle" font-size="28" fill="#4A5568">cylinder displacement relative to the grasp point (or axis tilt &gt; 45° = failure)</text></svg>')
# ---- 7 labels
slide('light', f'''
<h2>Labels, from simulator ground truth</h2>
<div class="two">
{card('<h3>y_success</h3><p>The cylinder stays in the grasp through the small lift, full lift and hold.</p><p class="small">Failure when it moves &gt; 40 mm relative to the grasp point, or its axis tilts &gt; 45°. A move &gt; 10 mm counts as slip.</p>')}
{card('<h3>y_stable</h3><p>Only for held grasps: the same rules, counted from the start of the lateral shake (±0.3 m along the cylinder axis).</p><p class="small">Empty (not 0) when the grasp already failed.</p>')}
</div>
<ul>
<li>Failure mode is also logged: <b>slide</b> (falls out) or <b>tilt</b> (pivots round the fingers).</li>
<li>Invalid trials (missed grasp, alignment timeout) are excluded.</li>
</ul>
{scale7}
''', 'Thresholds are in experiment.yaml under outcome and are unchanged from the original A/B experiment, so earlier tuning remains meaningful.',
      'Source: classify_outcome / classify_stability in experiment.py')

# ---- 8 inputs vs privileged
slide('dark', f'''
<h2>What the model may see, and what it may not</h2>
<div class="two">
{card('<h3 class="tealh">Model inputs (21 channels, 50 Hz)</h3><ul class="tight"><li>thrust and torque setpoints (3 + 3)</li><li>motor commands (4)</li><li>body rates (3), accelerometer (3)</li><li>roll, pitch (2)</li><li>height, vertical speed and acceleration (3)</li><li>static: gripper closed, commanded arm pose</li></ul>', 'dk')}
{card('<h3 class="orh">Privileged: labels and analysis only</h3><ul class="tight"><li><b>applied grip force</b>: decides the outcome in the simulator</li><li><b>scenario</b> (A or B): a proxy for it</li><li>cylinder displacement, tilt, pose</li><li>finger travel (sim-only Tier-2 ablation)</li></ul>', 'dk')}
</div>
<p class="statement2">Leakage control: results are reported per scenario against that scenario's own majority class, so a model cannot score by recognising A versus B.</p>
''', 'The hover thrust estimate was checked and is constant in this sim, so it was dropped from the inputs (21 channels instead of 22).',
      'Source: config/channels.yaml')

# ---- 9 lag figure
t1, c1, d1 = lag_series('20261016_164146_protocol', 8)
t2, c2, d2 = lag_series('20261017_004908_protocol', 5)
ch_args = dict(w=860, h=470, xr=(0, 4), yr=(0, 160), xticks=[0, 1, 2, 3, 4], yticks=[0, 50, 100, 150], xlabel='time after t₀ (s)',
               ylabel='rise (mm)', th='dark', shades=[(0, 1.5, '#FFFFFF', 0.06)], vlines=[(1.5, '#8091A8', '', '8 6')])
left = chart(series=[dict(pts=list(zip(t1, c1)), color=TH['dark']['c'], dash='10 8'),
                     dict(pts=list(zip(t1, d1)), color=TH['dark']['a'])],
             notes=[(0.08, 140, 'commanded 100 mm', TH['dark']['c'], 'start'), (2.2, 22, 'actual: 6 mm at 1.5 s', TH['dark']['a'], 'start')],
             **ch_args)
right = chart(series=[dict(pts=list(zip(t2, c2)), color=TH['dark']['c'], dash='10 8'),
                      dict(pts=list(zip(t2, d2)), color=TH['dark']['b'])],
              notes=[(0.08, 150, 'commanded 150 mm', TH['dark']['c'], 'start'), (2.0, 70, 'actual: ~85 % at 1.5 s', TH['dark']['b'], 'start')],
              **{**ch_args})
slide('dark', f'''
<h2>Finding 1: the drone does not follow the commanded lift</h2>
<div class="two"><div><h3 class="orh">Position setpoint only</h3>{left}</div><div><h3 class="tealh">With velocity feed-forward</h3>{right}</div></div>
<ul class="tight">
<li>First the drone must take the cylinder's weight (thrust +3–4 %, no motion for ~1.3 s); PX4's position loop then lags by ~2 s.</li>
<li><b>Feed-forward</b>: also send the velocity the profile needs. Standard PX4 offboard feature, flyable on the real drone.</li>
</ul>
''', 'Left: trial from the first pilot, 0.10 m / 1.5 s, position setpoint only. Right: trial from the final confirmation run, 0.15 m / 1.5 s with feed-forward. Shaded area is the 1.5 s small-lift window.',
      'Data: sim_data/20261016_164146_protocol (t0008), 20261017_004908_protocol (t0005)')

# ---- 10 profile comparison
items = [('0.10 m / 1.5 s', 13, 'lag 1.7 s'), ('0.06 m / 1.5 s', 0, 'lag 2.1 s'), ('0.10 m / 3.0 s', 37, 'lag 2.0 s'),
         ('0.10 m / 4.0 s', 49, 'lag 2.2 s'), ('0.06 m / 3.0 s', 14, 'lag 2.5 s'), ('0.10 m / 1.5 s + FF', 61, 'lag 0.5 s'),
         ('0.06 m / 1.5 s + FF', 36, 'lag 1.5 s'), ('0.15 m / 2.5 s + FF', 90, 'lag 0.5 s'), ('0.15 m / 1.5 s + FF', 85, 'lag 0.35 s')]
slide('light', f'''
<h2>Shorter distance or longer time? Neither. Feed-forward.</h2>
<div class="two wide-left">
<div>{hbars(items, 'light', w=1060, hl=8)}<p class="cap">Share of the commanded rise the drone had actually climbed at the end of the SL window (FF = velocity feed-forward).</p></div>
<div>
<ul class="tight">
<li><b>Shorter distance</b>: the lag is load transfer plus the slow position loop, so 6 cm is swallowed (0 %).</li>
<li><b>Longer time</b>: 49 % at 4 s, and the window then contains the failures.</li>
<li><b>Feed-forward</b> tracks 61–90 %. Chosen: <b>0.15 m / 1.5 s + FF</b> (85 %).</li>
</ul>
<p class="small">5–6 trials per row, scenario A. Indicative, not statistics.</p>
</div></div>
''', 'Scenario A at 1.40 N, valid trials only. The held/failed counts of these runs depend on the grip force, which was still tuned for the old 0.40 m lift, so they are not compared here.',
      'Source: result.md, section 3')

# ---- 11 payload signature
held, fail, fail_t, lab = window_curves()
tt = np.arange(75) / 50.0
mh, mf = held.mean(0), fail.mean(0)
sh, sf = held.std(0), fail.std(0)
med = float(np.median(fail_t))
peak = float(mh.max()); t_peak = float(tt[mh.argmax()]); dif = mh - mf
t_div = float(tt[np.argmax(dif > 0.5)]) if (dif > 0.5).any() else float('nan'); d15 = float(dif[-1])
fig = chart(w=1180, h=620, xr=(0, 1.5), yr=(-4, 8), xticks=[0, 0.5, 1.0, 1.5], yticks=[-4, 0, 4, 8],
            xlabel='time after t₀ (s)', ylabel='thrust change (% of hover)', th='dark',
            series=[dict(pts=list(zip(tt, mh)), color=TH['dark']['b'], band=list(sh)),
                    dict(pts=list(zip(tt, mf)), color=TH['dark']['a'], band=list(sf))],
            vlines=[(med, '#E8C547', f'median drop {med:.2f} s', '8 6')],
            notes=[(0.03, 6.7, f'held ({len(held)} trials)', TH['dark']['b'], 'start'), (0.03, 5.6, f'failed ({len(fail)} trials)', TH['dark']['a'], 'start')],
            fmt_y=lambda v: f'{v:+g}' if v else '0')
slide('dark', f'''
<h2>Finding 2: held and failed grasps look alike until the drop</h2>
<div class="two wide-left"><div>{fig}</div>
<div><ul class="tight">
<li>Thrust rises up to <b>~{peak:.0f} % of hover</b> (pick-up plus the climb), against 0.03–0.07 % hover noise.</li>
<li>Both groups follow the same curve until <b>~{t_div:.1f} s</b>; by 1.5 s held trials sit {d15:.1f} points higher.</li>
<li>The curves separate only around the drop: the warning <i>before</i> it is the hard part.</li>
</ul><p class="small">Mean ± 1 sd, session 20261017_004908 (27 valid trials).</p></div></div>
''', 'Thrust setpoint z, baseline-subtracted (last 0.5 s of the closing phase), as a percentage of the hover value. Earlier, without feed-forward, the steady payload signature was +2.3 to 2.7 % at the end of the window. Bands are +/- 1 standard deviation across trials.',
      'Data: sim_data/20261017_004908_protocol/dataset/windows.npz')

# ---- 12 grip force tuning
data, drops = held_vs_force()


def force_chart(sc, xr, xticks, step, nominal, f50):
    arr = np.array(data[sc]); x, y = arr[:, 0], arr[:, 1]
    f = logit_fit(x, y)
    bins = np.arange(xr[0], xr[1], step); pts = []
    for lo in bins:
        m = (x >= lo) & (x < lo + step)
        if m.sum() >= 2:
            pts.append((lo + step / 2, y[m].mean() * 100, 5 + 2.2 * math.sqrt(m.sum())))
    xs = np.linspace(xr[0], xr[1], 120)
    return chart(w=860, h=520, xr=xr, yr=(0, 100), xticks=xticks, yticks=[0, 50, 100], xlabel='applied grip force (N)',
                 ylabel='grasps held (%)', th='light',
                 series=[dict(pts=list(zip(xs, f(xs) * 100)), color=TH['light']['b'], width=4),
                         dict(pts=pts, color=TH['light']['a'], dots=True)],
                 vlines=[(nominal, '#1E2530', f'nominal {nominal:g} N', '8 6', 'bottom')], fmt_y=lambda v: f'{v:g}', fmt_x=lambda v: f'{v:.1f}'), len(x)


ca, na = force_chart('A', (1.1, 2.0), [1.2, 1.4, 1.6, 1.8, 2.0], 0.1, 1.46, 1.46)
cb, nb = force_chart('B', (2.2, 3.8), [2.4, 2.8, 3.2, 3.6], 0.2, 2.75, 2.75)
slide('light', f'''
<h2>Re-tuning the grip force for the new lift</h2>
<div class="two"><div><h3>A: centred ({na} trials)</h3>{ca}</div><div><h3>B: 30 mm off-centre ({nb} trials)</h3>{cb}</div></div>
<ul class="tight">
<li>The forces tuned for the 0.40 m lift held only 10–22 % of grasps. Logistic fit: 50 % held at <b>1.46 N</b> (A, sharp) and <b>2.75 N</b> (B, shallower).</li>
<li>Confirmation, 30 trials: 33 % failures in each scenario, inside the 30–60 % target.</li>
</ul>
''', 'Dots are the held fraction per force bin (size = number of trials, bins with fewer than 2 trials omitted); line is a logistic fit over all valid trials of the final profile. The applied force includes the +/-8 % per-trial jitter.',
      'Data: 8 sessions on the 0.15 m / 1.5 s profile (127 valid trials); result.md section 6')

# ---- 13 yield fix
slide('dark', f'''
<h2>Engineering fix: invalid trials</h2>
<div class="cards3">
{card('<p class="big or">8 / 24</p><p class="cap">A trials invalid before</p>', 'dk')}
{card('<p class="big tl">3 / 30</p><p class="cap">invalid after (A and B)</p>', 'dk')}
{card('<p class="big wh">0</p><p class="cap">invalid in 6 held-and-stable trials in a row after the fix</p>', 'dk')}
</div>
<pre class="tree">after a held-and-stable trial
  shake (±0.3 m)
  put cylinder back down      <span class="or">removed: 1 m descent, arm keeps swinging,
                              cylinder dropped from ~15 cm</span>
  open gripper
next trial
  align gripper to ±4 mm      <span class="or">failed: error stuck at 10–15 mm</span></pre>
<p class="small">The cylinder is teleported to its jig before every trial anyway, so the put-down only cost time.</p>
''', 'Every invalid trial had followed a trial that was both held and stable; trials after a dropped cylinder, which skip the put-down, were almost never invalid.',
      'Source: result.md section 6')

# ---- 14 y_stable is confounded (the problem, 27-trial session)
def stack(a, b, label, th='light'):
    t = TH[th]; w = 1100
    sa = a / (a + b) * w
    return (f'<div class="stackrow"><span class="stacklab">{label}</span><svg viewBox="0 0 {w} 90" width="{w}" height="90">'
            f'<rect x="0" y="6" width="{sa:.1f}" height="78" fill="{t["b"]}"/><rect x="{sa:.1f}" y="6" width="{w - sa:.1f}" height="78" fill="{t["a"]}"/>'
            f'<text x="{max(sa / 2, 24):.1f}" y="60" text-anchor="middle" font-size="36" fill="#fff" font-weight="600">{a}</text>'
            f'<text x="{sa + (w - sa) / 2:.1f}" y="60" text-anchor="middle" font-size="36" fill="#fff" font-weight="600">{b}</text></svg></div>')


slide('light', f'''
<h2>Problem: y_stable was confounded with the scenario</h2>
{stack(1, 9, 'A')}{stack(7, 1, 'B')}
{legend([('stable', TH['light']['b']), ('unstable', TH['light']['a'])], 'light')}
<ul>
<li>With one shared shake, A was stable 1 of 10 times and B 7 of 8 (held grasps), so a model could predict y_stable just by recognising A or B.</li>
<li>Cause: stability depends on the grip force, and A and B sit at very different points on that curve at the forces that give 30–60 % lift failures.</li>
</ul>
''', 'This is the state at the first dataset check, with 27 trials. In the earlier tuning data A at about 2.0 N was stable 6 of 6, so force strongly affects stability.',
      'Data: sim_data/20261017_004908_protocol')

# ---- 15 the fix: per-scenario shake
SHAKE_RUNS = [(['20261017_003053_protocol', '20261017_004908_protocol'], 2.0, 2.0),
              (['20261111_213750_protocol'], 3.0, 1.25),
              (['20261111_215940_protocol'], 2.4, 1.25),
              (['20261111_231601_protocol'], 2.15, 1.25),
              (['20261111_233750_protocol', '20261111_235915_protocol', '20261112_002046_protocol',
                '20261112_004210_protocol'], 2.25, 1.25)]


def shake_counts():
    acc = {}
    for sess, la, lb in SHAKE_RUNS:
        for s in sess:
            for r in rows(s):
                if (r['valid'] == 'True' and r['y_success'] == '1' and r['y_stable'] in ('0', '1')
                        and r['scenario'] in ('A', 'B')):
                    leg = la if r['scenario'] == 'A' else lb
                    a = acc.setdefault((r['scenario'], leg), [0, 0])
                    a[0] += 1
                    a[1] += r['y_stable'] == '0'
    return acc


SC = shake_counts()


def wilson(k, n, z=1.96):
    """95 % Wilson interval of a share k/n, in percent."""
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return 100 * (c - h), 100 * (c + h)


def sdots(sc_, legs, color):
    pts, notes, bars = [], [], []
    for leg in legs:
        n, u = SC[(sc_, leg)]
        p = 100 * u / n
        lo, hi = wilson(u, n)
        bars.append(dict(pts=[(leg, lo), (leg, hi)], color=color, width=4))
        r = 7 + 3.2 * math.sqrt(n)
        pts.append((leg, p, r))
        left = leg == 2.15            # keep this label clear of the dashed line at 2.25
        notes.append((leg - (r / 418 + 0.05) if left else leg + r / 418 + 0.05, p - 2.1, f'{u}/{n}', color, 'end' if left else 'start', '#F6F5F0'))
    return pts, notes, bars


pa, na_, ba_ = sdots('A', (2.0, 2.15, 2.25, 2.4, 3.0), TH['light']['a'])
pb, nb_, bb_ = sdots('B', (1.25, 2.0), TH['light']['b'])
fig = chart(w=1060, h=600, xr=(1.0, 3.2), yr=(-8, 108), xticks=[1.0, 1.5, 2.0, 2.5, 3.0], yticks=[0, 25, 50, 75, 100],
            xlabel='shake leg duration (s); longer = gentler', ylabel='share of held grasps that fail the shake (%)', th='light',
            series=[dict(pts=[(1.0, 25), (3.2, 25)], color='#6A7179', width=2.5, dash='8 6')] + ba_ + bb_ + [
                    dict(pts=pa, color=TH['light']['a'], dots=True), dict(pts=pb, color=TH['light']['b'], dots=True)],
            notes=na_ + nb_ + [(3.18, 21, 'gate: at least 25 %', '#6A7179', 'end', '#F6F5F0')])
slide('light', f'''
<h2>Fix: one shake intensity per scenario</h2>
<div class="two wide-left"><div>{fig}{legend([('scenario A', TH['light']['a']), ('scenario B', TH['light']['b'])], 'light')}</div>
<div><ul class="tight">
<li>New config block <b>shake.per_scenario</b>: only the leg time differs (the ±0.3 m amplitude and the grip forces stay).</li>
<li>A needs a harsher shake than the 3.0 s legs of the first pilot: {SC[('A', 3.0)][1]} of {SC[('A', 3.0)][0]} held grasps failed it; at 2.0 s, {SC[('A', 2.0)][1]} of {SC[('A', 2.0)][0]} did.</li>
<li>Chosen: <b>A 2.25 s, B 1.25 s</b>. In the 150-trial batch each scenario has 38 % unstable among its held grasps.</li>
</ul><p class="small">Each point is a count, not a rate over 100 trials: labels are unstable / held grasps, bars the 95 % interval. Pilots have only 8–20 held grasps, so their intervals are wide; the batch (A 13/34) is the reliable point.</p></div></div>
''', 'Each dot is the share of held grasps that failed the lateral shake, for one shake leg duration; the label gives the counts (unstable over held) and the bar is the 95 percent Wilson interval. The counts are real, pooled over all sessions with that setting, not out of 100 trials. The sessions: the two 30-trial sessions of 17 October at 2.0 s (both scenarios), then pilots at A 3.0, 2.4 and 2.15 s with B at 1.25 s, and the batch at A 2.25 s and B 1.25 s. The shake runs after the window the model sees, so a per-scenario intensity does not enter the inputs; it is a difficulty knob like the grip force.',
      'Data: sim_data sessions 20261017 to 20261112; result.md section 9')

# ---- 16 batch
nA = int((BATCH.scenario == 'A').sum()); nB = int((BATCH.scenario == 'B').sum())


def cnt(sc_):
    d = BATCH[BATCH.scenario == sc_]
    h = d[d.y_success == 1]
    return len(d), len(h), int((h.y_stable == 1).sum()), int((h.y_stable == 0).sum())


(vA, hA, sA, uA), (vB, hB, sB, uB) = cnt('A'), cnt('B')
slide('dark', f'''
<h2>Main batch: 150 trials, every acceptance gate passes</h2>
<div class="cards3">
{card(f'<p class="big sm or">{100 * (vA + vB) / 135:.0f} %</p><p class="cap">valid A/B trials<br>({vA + vB} of 135; gate ≥ 95 %)</p>', 'dk')}
{card(f'<p class="big sm tl">{100 * (vA - hA) / vA:.0f} % · {100 * (vB - hB) / vB:.0f} %</p><p class="cap">lift failures, A · B<br>(gate 30–60 %)</p>', 'dk')}
{card(f'<p class="big sm wh">{100 * uA / hA:.0f} % · {100 * uB / hB:.0f} %</p><p class="cap">unstable among held, A · B<br>(gate ≥ 25 %)</p>', 'dk')}
</div>
<div class="midtable">{table(['Scenario', 'Valid', 'Held', 'Stable', 'Unstable'], [
    ['<b>A</b> centred, 1.46 N', vA, hA, sA, uA], ['<b>B</b> 30 mm off, 2.75 N', vB, hB, sB, uB],
    ['<b>E</b> empty-gripper control', '15', 'all came out “empty”', '', '']])}</div>
<ul class="tight"><li>Four sessions (40, 40, 40, 30 trials), the sim restarted before each. 131 valid windows (75 × 21): 71 held, 60 failed, 44 of the held stable.</li></ul>
''', 'The gates are the M5 acceptance criteria of the plan. E trials are the empty-gripper control, used only for the detectability check, not as classifier data. One estimator fault, after two consecutive 40-trial sessions without a restart, aborted a pilot; the batch driver now restarts the simulator before each session.',
      'Source: result.md section 10; utility/batch.sh')

# ---- 17 detectability and leakage
slide('light', f'''
<h2>Is the payload visible? Is anything leaking?</h2>
<div class="two"><div><h3>Detectability (gate: ≥ 3× hover noise)</h3>
<div class="compact">{table(['Signal', 'Whole window<br>A / B', 'First 0.8 s<br>A / B'], [
    ['Thrust, motors', '<b>210× / 198×</b>', '48× / 26×'], ['Pitch torque', '49× / 43×', '27× / 21×'],
    ['Vertical acceleration', '3.9× / 4.7×', '32× / 31×'], ['Roll torque', '0.3× / 3.1×', '3.3× / 5.5×']])}</div>
<p class="small">Payload step against 15 empty-gripper controls. The sim hover is very quiet; against the trial-to-trial scatter of the controls thrust is 16× over the window but 2.3–4.2× in the first 0.8 s.</p></div>
<div><h3>Leakage check</h3>
<div class="compact">{table(['Check', 'Result'], [
    ['Privileged variables among the 21 inputs', 'none'],
    ['Scenario (A vs B) from the inputs', '<b>readable</b>, AUC 0.99'],
    ['Scenario alone → y_success / y_stable', 'AUC 0.54 / 0.40: no information'],
    ['Grip force, held grasps only', 'not readable'],
    ['Grip force within A, all trials', 'readable (R² 0.55) through the visible drop']])}</div></div></div>
<ul class="tight"><li>The payload is clearly detectable: no change of cylinder mass is needed.</li>
<li>A model can always tell A from B (the offset shows in torque), but that says nothing about the labels. Rule: stratify folds, report per scenario, never feed the force.</li></ul>
''', 'Detectability is the plan milestone M1. The ratios compare the payload step in the drone channels with the standard deviation of the 0.5 s hover before the small lift. Roll torque is weak in scenario A, probably because the cylinder lies along the world Y axis (untested). Leakage is milestone M6: scenario and force must not be inputs; the original wording that the scenario be absent from the inputs cannot hold, because the offset is real physics, so the criterion is reworded to no label information from the scenario alone.',
      'Source: result.md sections 11 and 12')

# ---- 18 baselines
FENDS = [0.4, 0.6, 0.8, 1.0, 1.5]
on_med = float(FAILED.t_onset_s.median()); dr_med = float(FAILED.t_drop_s.median())


def hist2(onset, drop, th='dark', W=860, H=430):
    t = TH[th]; ml, mb, mt = 90, 84, 30
    bins = np.arange(0.5, 3.0, 0.1)
    def counts(v):
        v = np.array(v)
        return [int(((v >= b) & (v < b + 0.1)).sum()) for b in bins] + [int((v >= 3.0).sum())]
    c1, c2 = counts(onset), counts(drop)
    pw, ph = W - ml - 20, H - mb - mt
    mx = max(c1 + c2) + 2; k = len(bins) + 1
    o = [f'<svg viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="IBM Plex Sans, Arial, sans-serif">']
    for arr, col, op in ((c1, t['a'], 0.9), (c2, t['b'], 0.8)):
        for j, c in enumerate(arr):
            x = ml + j * pw / k; bw = pw / k - 3; hh = c / mx * ph
            o.append(f'<rect x="{x:.1f}" y="{mt + ph - hh:.1f}" width="{bw:.1f}" height="{hh:.1f}" fill="{col}" opacity="{op}"/>')
    o.append(f'<line x1="{ml}" x2="{W - 20}" y1="{mt + ph}" y2="{mt + ph}" stroke="{t["axis"]}" stroke-width="2"/>')
    xs = lambda v: ml + (v - 0.5) / 0.1 * pw / k
    o.append(f'<line x1="{xs(1.5):.1f}" x2="{xs(1.5):.1f}" y1="{mt}" y2="{mt + ph}" stroke="{t["fg"]}" stroke-width="3" stroke-dasharray="8 6"/>')
    o.append(f'<text x="{xs(1.5) + 10:.1f}" y="{mt + 30}" font-size="24" fill="{t["fg"]}">end of SL window</text>')
    for v in (1.0, 1.5, 2.0, 2.5):
        o.append(f'<text x="{xs(v):.1f}" y="{mt + ph + 40}" font-size="26" fill="{t["muted"]}" text-anchor="middle">{v:g}</text>')
    o.append(f'<text x="{ml + (len(bins)) * pw / k + 20:.1f}" y="{mt + ph + 40}" font-size="26" fill="{t["muted"]}" text-anchor="middle">&gt;3</text>')
    o.append(f'<text x="{ml + pw / 2}" y="{H - 8}" text-anchor="middle" font-size="26" fill="{t["muted"]}">time after t₀ (s)</text>')
    o.append(f'<text transform="translate(26 {mt + ph / 2}) rotate(-90)" text-anchor="middle" font-size="26" fill="{t["muted"]}">failed grasps ({len(drop)})</text></svg>')
    return ''.join(o)


auc_fig = chart(w=860, h=430, xr=(0.3, 1.6), yr=(0, 1), xticks=FENDS, yticks=[0, 0.5, 1.0], xlabel='window end (s after t₀)',
                ylabel='AUC, held vs failed', th='dark',
                series=[dict(pts=[(0.3, 0.5), (1.6, 0.5)], color='#8091A8', width=2.5, dash='6 6'),
                        dict(pts=[(e, au(BL, 'y_success', 'svm', e)) for e in FENDS], color=TH['dark']['fg'], width=5),
                        dict(pts=[(e, au(BL, 'y_success', 'lstm', e)) for e in FENDS], color=TH['dark']['b'], width=4, dash='2 7'),
                        ],
                vlines=[(on_med, TH['dark']['a'], '', '8 6'), (dr_med, TH['dark']['b'], '', '8 6')],
                fmt_y=lambda v: f'{v:g}', fmt_x=lambda v: f'{v:g}',
                notes=[(on_med - 0.02, 0.1, 'slip starts', TH['dark']['a'], 'end'), (dr_med + 0.02, 0.1, 'median drop', TH['dark']['b'], 'start')])
y1 = [au(BL, 'y_success', 'svm', e) for e in FENDS]
ys = [au(BL, 'y_stable', 'svm', e) for e in FENDS]
slide('dark', f'''
<h2>Baselines: the drop is detectable, early warning is not</h2>
<div class="two"><div><h3>When grasps start to slip and fall</h3>{hist2(FAILED.t_onset_s.to_numpy(), FAILED.t_drop_s.to_numpy())}{legend([('slip starts', TH['dark']['a']), ('drops', TH['dark']['b'])], 'dark')}</div>
<div><h3>SVM and LSTM on the drone channels</h3>{auc_fig}{legend([('SVM', TH['dark']['fg']), ('LSTM (dotted)', TH['dark']['b'])], 'dark')}</div></div>
<ul class="tight">
<li>AUC {y1[0]:.2f} / {y1[1]:.2f} / {y1[2]:.2f} at 0.4 / 0.6 / 0.8 s, then {y1[3]:.2f} at 1.0 s and {y1[4]:.2f} at 1.5 s: the signal appears when the cylinder starts to slip (median {on_med:.2f} s), the drop follows at {dr_med:.2f} s. The lead is about 0.2–0.4 s.</li>
<li>LSTM ≈ SVM at 131 trials; A and B score alike; leave-one-session-out matches the cross-validation (no drift between sim restarts). <b>y_stable</b> is not predictable from this window (AUC {min(ys):.2f}–{max(ys):.2f}, 71 held grasps).</li></ul>
''', 'RBF-SVM on 126 summary features per window and a 20-unit LSTM on the raw window, both on the 21 drone channels; the window is cut at the end time shown, so a model cannot score by seeing the drop. Stratified 5-fold cross-validation by scenario and label, repeated. The uncertainty across repeats understates the real one: with 131 trials an AUC has a standard error of about 0.03 to 0.05. Hyper-parameters are not tuned. Left: the slip onset (cylinder moved 10 mm) and the drop (40 mm) of the 60 failed grasps. 92 percent of failures have dropped by 1.5 s.',
      'Data: ml/results/baselines.json; result.md section 13')

# ---- 19 slower lift: the failure moves, the warning does not
rng = np.random.default_rng(0)
srow = []
for s in SL25_SESS:
    srow += [r for r in rows(s) if r['valid'] == 'True' and r['y_success'] == '0']
slow_on = np.array([float(r['t_onset_s']) for r in srow if r['t_onset_s'] not in ('', 'nan')])
slow_dr = np.array([float(r['t_drop_s']) for r in srow])


def strip(v, yv, col, late_y=None):
    v = np.array(v); m = v <= 3.0
    return [(float(x), yv + float(rng.uniform(-0.18, 0.18)), 9) for x in v[m]]


fig_t = chart(w=860, h=470, xr=(0.5, 3.0), yr=(0.4, 2.6), xticks=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0], yticks=[1, 2],
              xlabel='time after t₀ (s)', th='light', margin=(210, 30, 30, 84),
              fmt_y=lambda v: {1: '1.5 s lift', 2: '2.5 s lift'}.get(v, ''),
              series=[dict(pts=strip(FAILED.t_onset_s, 1, 'a') + strip(slow_on, 2, 'a'), color=TH['light']['a'], dots=True),
                      dict(pts=strip(FAILED.t_drop_s, 1, 'b') + strip(slow_dr, 2, 'b'), color=TH['light']['b'], dots=True)],
              notes=[(2.97, 1.42, f'+{int((FAILED.t_drop_s > 3).sum())} beyond 3 s', TH['light']['muted'], 'end'),
                     (2.97, 2.42, f'+{int((slow_dr > 3).sum())} beyond 3 s', TH['light']['muted'], 'end')])
E2 = [0.6, 0.8, 1.0, 1.2, 1.5, 2.5]
fig_a = chart(w=860, h=470, xr=(0.3, 2.7), yr=(0, 1), xticks=[0.5, 1.0, 1.5, 2.0, 2.5], yticks=[0, 0.5, 1.0],
              xlabel='window end (s after t₀)', ylabel='AUC', th='light',
              series=[dict(pts=[(0.3, 0.5), (2.7, 0.5)], color='#8091A8', width=2.5, dash='6 6'),
                      dict(pts=[(e, au(BL, 'y_success', 'svm', e)) for e in FENDS], color=TH['light']['a'], width=5),
                      dict(pts=[(e, au(SL25, 'y_success', 'svm', e)) for e in E2], color=TH['light']['b'], width=5)],
              fmt_x=lambda v: f'{v:g}')
slide('light', f'''
<h2>Slower lift: the failure moves, the warning does not</h2>
<div class="two"><div><h3>Slip start and drop of the failed grasps</h3>{fig_t}{legend([('slip starts', TH['light']['a']), ('drops', TH['light']['b'])], 'light')}</div>
<div><h3>SVM AUC against window end</h3>{fig_a}{legend([('1.5 s lift (131 trials)', TH['light']['a']), ('2.5 s lift (40 trials, ±0.1)', TH['light']['b'])], 'light')}</div></div>
<ul class="tight">
<li>0.15 m in 2.5 s instead of 1.5 s: same grip force for 50 % held (A 1.49 N, B 2.72 N), but slip starts at ~{float(np.median(slow_on)):.1f} s and the drop at ~{float(np.median(slow_dr)):.1f} s. The model still needs the slip in its window: chance up to 1.2 s, AUC {au(SL25, 'y_success', 'svm', 1.5):.2f} at 1.5 s.</li>
<li><b>Lower grip force is worse</b>: in A the weakest third of the failures drop earliest (median 0.92 s against 1.10 s for the strongest).</li></ul>
''', 'A 40-trial force sweep with the 2.5 s small lift (5 trials per force level in each scenario) is used here, so its AUC has a standard error of about 0.1. The point is the pattern, not the exact values: the signal appears when the slip does, and the slower lift shifts both. Lowering the grip force does not help because a weaker grasp slips and falls sooner.',
      'Data: sim_data/20261114_200141_tuneA, 20261114_201351_tuneB; result.md section 14')

# ---- 20 other attempts
trk = EXTRA['tracking (9 ch)|0.4']['pooled']['auc'][0]
slide('dark', f'''
<h2>Four more attempts: nothing warns earlier</h2>
<div class="midtable">{table(['Attempt', 'Idea', 'Result'], [
    ['Closing-gripper window', 'The 1 s while the fingers close, before the lift', '<b>AUC 0.49</b> (131 trials)'],
    ['Finger travel (sim only)', 'The squeeze shows in the finger position', 'AUC 0.43–0.46 up to 0.8 s; and not measurable on the open-loop servos'],
    ['Tier-1 gripper state, hover levels', 'Static gripper information', 'No change (constant in every trial; hover levels add noise)'],
    ['33 more PX4 signals', 'Tracking errors, controller outputs, hover-thrust estimator, EKF innovations, acceleration, battery', 'None beats the 21 channels from 0.6 s on; combining adds nothing'],
    ['Follow-up: tracking errors at 0.4 s', 'A hint in the position loop', f'AUC {trk:.2f}, p = 0.013 but about 20 tests: a hypothesis for a fresh batch'],
])}</div>
<p class="statement2">With the drone's telemetry alone, the grasp margin is invisible until the cylinder starts to slip. More lead time needs an input that senses the grasp itself.</p>
''', 'Each attempt used the same SVM and cross-validation as the baselines. The extra PX4 signals are all logged on the real drone as well. The tracking-error hint was checked with a permutation test (labels shuffled within scenario, 150 permutations: null mean 0.49, sd 0.06), but about 20 group-and-window cells were tested, so one at p = 0.013 is what chance produces; it does not grow with the window and no single channel carries it.',
      'Source: result.md sections 14, 15 and 16')

# ---- 21 real gripper vs sim
slide('light', f'''
<h2>What the real gripper can and cannot give</h2>
<div class="compact">{table(['', 'Simulator', 'Real Armbot (open-loop servos)'], [
    ['<b>Actuation</b>', 'Two force-controlled fingers, 0–35 mm each; the grip level is a force in newtons', 'Hobby servo commanded by angle (closed at about 96°); the squeeze comes from servo stiffness and stall torque'],
    ['<b>Feedback</b>', 'Finger position is exact ground truth', 'None: the logger records the commanded angle and grip_state only'],
    ['<b>Grip force</b>', 'Commanded, so known', 'Not set, not measured'],
])}</div>
<div class="cards3">
{card('<h3>Finger travel</h3><p>Needs a hardware change (potentiometer tap, bus servo, encoder), and the simulator says it adds no early warning.</p>')}
{card('<h3>Squeeze proxies</h3><p>Servo current or load, a bus servo with load feedback, or a force pad: the only inputs that could carry the margin before slip.</p>')}
{card('<h3>Why not test it in sim</h3><p>A simulated force sensor reads the commanded force, which already predicts the outcome within A (AUC 0.95): nearly circular.</p>')}
</div>
''', 'Hardware per the project brief: Holybro X500 V2 with the CODLAI/JSumo ArmBot (5 joints, Arduino Nano, open-loop servos). The actual servo model was not inspected; please check it. The hardware logger already records the commanded gripper angle at 50 Hz in the Tier-1 CSV.',
      'Source: 07_grasp_experiment.md limitations; small-lift-plan-brief.md; result.md section 15')

# ---- 22 plan and status
chips = {'done': ('done', C['good']), 'part': ('partly', '#8A6D00'), 'todo': ('to do', '#6A7179')}
ms = [('M0', 'Freeze baseline, branch', 'done'), ('M1', 'Detectability of the payload (26–210× hover noise)', 'done'),
      ('M2', 'Small-lift trajectory (feed-forward, re-tuned forces)', 'done'),
      ('M3', 'Shake and labels (per-scenario shake, y_stable balanced)', 'done'),
      ('M4', 'Recording: phase times, config hash, channel list', 'done'),
      ('M5', 'Batch generation: 150 trials, all gates pass (600 only if justified)', 'done'),
      ('M6', 'Dataset pipeline and leakage check', 'done'),
      ('M7', 'Baselines: SVM and LSTM done; FFNN and closing+SL window missing', 'part'),
      ('M8', 'CNN-LSTM, ablations (not justified at 131 trials)', 'todo'), ('M9', 'Held-out forces, closed loop', 'todo')]
rows_ms = [[f'<b>{a}</b>', b, f'<span class="chip" style="background:{chips[s][1]}">{chips[s][0]}</span>'] for a, b, s in ms]
slide('light', f'''
<h2>Plan and status</h2>
<div class="compact">{table(['', 'Milestone', 'Status'], rows_ms)}</div>
<p class="small">91 unit tests pass. Everything lives on branch feature/small-lift-protocol; the original A/B/E experiment is untouched.</p>
''', 'M3 and M5 are now done. M7 is partly done: no FFNN and no combined closing-and-small-lift window, and the closing window alone carries no information. The CNN-LSTM is not justified because the LSTM equals the SVM at 131 trials.', '')

# ---- 23 ways forward
slide('dark', f'''
<h2>Ways forward</h2>
<div class="compact">{table(['Option', 'Effort', 'More lead time?', 'Comment'], [
    ['<b>Reframe as detection</b>: online detector scored by detection delay against false alarms', 'low', 'n/a', 'Honest with today’s data; matches how a closed loop would use it'],
    ['<b>Real-drone signal check</b>: about 20 flights, payload step against the real hover noise', 'low–medium', 'n/a', 'De-risks deployment: the sim’s 200× margin will not carry over'],
    ['<b>Replicate the tracking-error hint</b> on a fresh batch', '≈ 6 h sim', 'low', 'Cheap hypothesis test; probably a null'],
    ['<b>Add a grasp-sensing input</b> (sim sensor model, then hardware)', 'high', 'highest', 'The only route to information about the margin before slip'],
    ['<b>Match the real knob</b>: position-controlled sim gripper', 'high', 'unknown', 'More realistic; changes the failure physics'],
    ['<b>Change payload or pads</b> for slower, creep-like failures', 'medium', 'unknown', 'Quick to try; forces must be re-tuned'],
])}</div>
<p class="statement2">Proposed: do the first three now; decide the sensing input with you.</p>
''', 'The first two options cost little and are useful whatever else is chosen. The sensing input is the only option with a plausible large gain in lead time, but it is hardware work and cannot be validated honestly in the simulator.',
      'Source: result.md section 17')

# ---- 24, 25 questions
slide('dark', f'''
<h2>Questions for discussion (1/2)</h2>
<ol class="asks">
<li><b>Contribution</b>: is detecting an impending drop about 0.2–0.4 s ahead enough, or must the thesis show earlier prediction? What lead is useful on the real system: how long do abort and re-grasp take?</li>
<li><b>Metric</b>: move from AUC at fixed window ends to an online detector scored by detection delay against false alarms?</li>
<li><b>Sensing</b>: is a grasp-sensing channel in scope (force pad, servo current or load, bus servo)? Model it in the simulator first, or go straight to hardware?</li>
<li><b>Real knob</b>: rebuild the sim gripper as a position-controlled, torque-limited servo so the varied quantity matches the real one?</li>
</ol>
''', 'The answers to the first two decide whether the current result is already a result.', '')
slide('dark', f'''
<h2>Questions for discussion (2/2)</h2>
<ol class="asks">
<li><b>Real validation</b>: how many real flights are needed to claim transfer? Start with about 20 that only measure the payload step against the real hover noise?</li>
<li><b>Sim-only result</b>: acceptable for this chapter if the real flights only confirm the signal level?</li>
<li><b>Data</b>: spend about 6 h on a fresh, held-out batch to test the one weak hint, or scale to 600 trials for the planned models?</li>
<li><b>Models and y_stable</b>: keep the CNN-LSTM in scope (the LSTM equals the SVM today)? Keep y_stable as a second task (unpredictable so far) or redefine it?</li>
<li><b>Physical choices</b>: may the payload, friction or pads change under the “two scenarios, many repetitions” decision, if that gives slower, creep-like failures?</li>
</ol>
''', 'Question 7 asks for permission to treat the tracking-error hint as a pre-stated hypothesis on new data.', '')


# ----------------------------------------------------------------------------- page
CSS = '''
*{box-sizing:border-box}html,body{margin:0;height:100%;background:#000;font-family:"IBM Plex Sans",Arial,sans-serif}
#stage{position:fixed;left:50%;top:50%;width:1920px;height:1080px;transform-origin:center center}
.slide{position:absolute;inset:0;display:none;overflow:hidden;width:1920px;height:1080px}
.slide.on{display:block}
.slide.dark{background:#16202E;color:#F6F5F0}.slide.light{background:#F6F5F0;color:#1E2530}
.pad{position:absolute;inset:0;padding:96px 128px 140px;display:flex;flex-direction:column;gap:36px}
h1{font-size:104px;line-height:1.08;margin:0;font-weight:600;max-width:1500px}
h2{font-size:58px;line-height:1.12;margin:0;font-weight:600;max-width:1600px}
h3{font-size:36px;margin:0 0 10px;font-weight:600}
p{font-size:32px;line-height:1.4;margin:0}
ul,ol{font-size:34px;line-height:1.45;margin:0;padding-left:44px}li{margin-bottom:8px}
ul.tight{font-size:31px}ol.asks{font-size:30px}ol.asks{font-size:36px;line-height:1.5}ol.asks li{margin-bottom:22px}ol.asks.s{font-size:32px;line-height:1.45}ol.asks.s li{margin-bottom:18px}
.eyebrow{font-size:28px;font-weight:600;letter-spacing:4px;text-transform:uppercase;color:#F08A4B}
.lead{font-size:40px;color:#C3CCD8;max-width:1400px}
.statement{font-size:52px;line-height:1.3;font-weight:500;max-width:1500px;margin-top:20px}
.statement2{font-size:32px;line-height:1.4;color:inherit;font-weight:500;border-left:8px solid #F08A4B;padding-left:24px;max-width:1600px}
.small{font-size:26px;color:#6A7179}.dark .small{color:#C3CCD8}.cap{font-size:28px;color:#4A5568}.dark .cap{color:#C3CCD8}
.spacer{flex:1}
.cards3{display:grid;grid-template-columns:repeat(3,1fr);gap:28px}
.two{display:grid;grid-template-columns:1fr 1fr;gap:36px;align-items:start}.two.wide-left{grid-template-columns:1.55fr 1fr}
.card{background:#FDFCF9;border:1px solid #DDDAD0;border-radius:16px;padding:32px}
.card.dk{background:#243246;border-color:#33445C;color:#F6F5F0}
.big{font-size:84px;font-weight:600;line-height:1.1}.big.sm{font-size:62px;padding-top:12px}.or{color:#B8501A}.dark .or{color:#F08A4B}.tl{color:#1F6F7F}.dark .tl{color:#6CC3D1}.wh{color:#F6F5F0}
.orh{color:#F08A4B}.tealh{color:#6CC3D1}
table{border-collapse:collapse;width:100%;font-size:28px}.bigtable table{font-size:34px}.bigtable td,.bigtable th{padding:20px 22px}.midtable table{font-size:31px}.midtable td,.midtable th{padding:16px 20px}.compact td,.compact th{padding:9px 18px}
th{text-align:left;padding:14px 18px;font-weight:600;background:rgba(128,145,168,.22)}td{padding:14px 18px;border-top:1px solid rgba(128,145,168,.35);vertical-align:top;line-height:1.35}
.chip{display:inline-block;color:#fff;border-radius:999px;padding:2px 16px;font-size:24px}
.legend{display:flex;gap:36px;font-size:28px}.legend i{display:inline-block;width:26px;height:26px;border-radius:5px;margin-right:10px;vertical-align:-4px}
.stackrow{display:flex;align-items:center;gap:24px}.stacklab{font-size:44px;font-weight:600;width:60px}
.tree{font:26px/1.5 "IBM Plex Mono",ui-monospace,Menlo,monospace;background:#0F1824;border-radius:12px;padding:26px 32px;margin:0;color:#E6EAF0}
.tree .or{color:#F08A4B}
svg{display:block;max-width:100%;height:auto}
.foot{position:absolute;left:128px;right:128px;bottom:48px;font-size:24px;color:#6A7179}.dark .foot{color:#C3CCD8}
aside{display:none}
#pager{position:fixed;right:16px;bottom:14px;color:#fff;background:rgba(0,0,0,.6);border-radius:999px;padding:4px 14px;font:14px system-ui;opacity:.0;transition:opacity .3s}
body:hover #pager{opacity:1}
#notes{position:fixed;left:0;right:0;bottom:0;background:rgba(0,0,0,.85);color:#fff;font:20px/1.4 system-ui;padding:14px 24px;display:none;z-index:5}
#notes.on{display:block}
@page{size:1920px 1080px;margin:0}
@media print{html,body{background:none;height:auto}#stage{position:static;transform:none!important}
.slide{display:block!important;position:relative;break-after:page;page-break-after:always;-webkit-print-color-adjust:exact;print-color-adjust:exact}
#pager,#notes{display:none!important}}
'''
JS = '''
(function(){
 var slides=[].slice.call(document.querySelectorAll('.slide')),stage=document.getElementById('stage'),
 pager=document.getElementById('pager'),notes=document.getElementById('notes'),i=0;
 function go(n){i=Math.max(0,Math.min(slides.length-1,n));
  slides.forEach(function(s,k){s.classList.toggle('on',k===i)});
  pager.textContent=(i+1)+' / '+slides.length;
  var a=slides[i].querySelector('aside');notes.textContent=a?a.textContent:'';
  try{history.replaceState(null,'','#'+(i+1))}catch(e){}}
 function fit(){var s=Math.min(innerWidth/1920,innerHeight/1080);
  stage.style.transform='translate(-50%,-50%) scale('+s+')'}
 addEventListener('resize',fit);fit();
 addEventListener('keydown',function(e){
  if(['ArrowRight','ArrowDown','PageDown',' ','Enter'].indexOf(e.key)>-1){e.preventDefault();go(i+1)}
  else if(['ArrowLeft','ArrowUp','PageUp','Backspace'].indexOf(e.key)>-1){e.preventDefault();go(i-1)}
  else if(e.key==='Home')go(0);else if(e.key==='End')go(slides.length-1);
  else if(e.key==='n'||e.key==='N')notes.classList.toggle('on');
  else if(e.key==='f'||e.key==='F'){if(!document.fullscreenElement)document.documentElement.requestFullscreen&&document.documentElement.requestFullscreen();else document.exitFullscreen()}});
 addEventListener('click',function(e){go(e.clientX>innerWidth/2?i+1:i-1)});
 go((parseInt(location.hash.slice(1),10)||1)-1);
})();
'''
doc = ('<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
       '<title>Zimmer Small-Lift for UAV Grasping</title>'
       '<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Sans:ital,wght@0,400;0,500;0,600;1,400&amp;family=IBM+Plex+Mono&amp;display=swap">'
       f'<style>{CSS}</style></head><body><div id="stage">{"".join(slides)}</div><div id="pager"></div><div id="notes"></div>'
       f'<script>{JS}</script></body></html>')
open(OUT, 'w').write(doc)
print('wrote', OUT, len(slides), 'slides', len(doc) // 1024, 'KB')
