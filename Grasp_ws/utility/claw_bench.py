#!/usr/bin/env python3
"""Claw bench: does the servo claw hold the cylinder, and do the pad contact sensors (the
simulated FSRs) give a normal force that depends on the command and on what the cylinder does?

Needs the bench world running (no PX4):
  GZ_SIM_RESOURCE_PATH=<repo>/px4/Tools/simulation/gz/models:<repo>/Grasp_ws/model \
      gz sim -s -r Grasp_ws/worlds/claw_bench.sdf
then:
  PYTHONNOUSERSITE=1 PYTHONPATH=Grasp_ws/src/px4_gripper_missions \
      python3 Grasp_ws/utility/claw_bench.py --qcmd 0.30 0.10 0.0 --out <dir>

Per run: arm in the reach pose (pointing down), jaws open, cylinder placed at the grasp point
on a support block, jaws commanded to q_cmd [rad], support removed, 3 s recorded at 50 Hz.
"""
import argparse
import csv
import os
import subprocess
import threading
import time

import numpy as np
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from px4_gripper_missions.geometry import compose, quat_to_rot, transform_point

W = 'claw_bench'
GRASP = (0.3025, 0.0, 0.110)          # grasp point in the gripper link frame
PADS = [(j, p) for j in ('jaw_left', 'jaw_right') for p in ('pad_a', 'pad_b')]
REACH = {'base_yaw_joint': 0.0, 'shoulder_pitch_joint': -1.45, 'elbow_pitch_joint': -0.12}
OPEN = -0.6


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def gz_service(name, reqtype, req):
    r = subprocess.run(['gz', 'service', '-s', f'/world/{W}/{name}', '--reqtype', reqtype,
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       capture_output=True, text=True, timeout=10)
    return 'data: true' in r.stdout


class Bench:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()
        self.t = 0.0
        self.poses, self.q, self.eff = {}, {}, {}
        self.pad = {k: (0.0, -1.0, 0) for k in PADS}      # (normal force, stamp, n_wrench)
        self.node.subscribe(Pose_V, f'/world/{W}/pose/info', self._pose)
        self.node.subscribe(Model, '/model/armbot/joint_states', self._joints)
        for k in PADS:
            self.node.subscribe(Contacts, f'/model/armbot/{k[0]}/{k[1]}/contact',
                                lambda m, k=k: self._contact(k, m))
        self.pub = {j: self.node.advertise(f'/model/armbot/joint/{j}/cmd_pos', Double)
                    for j in list(REACH) + ['jaw_left_joint', 'jaw_right_joint']}

    def _pose(self, m):
        with self.lock:
            self.t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
            for p in m.pose:
                self.poses[p.name] = (np.array([p.position.x, p.position.y, p.position.z]),
                                      (p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z))

    def _joints(self, m):
        with self.lock:
            for j in m.joint:
                self.q[j.name] = j.axis1.position
                self.eff[j.name] = j.axis1.force

    def _contact(self, k, m):
        f, nw = 0.0, 0
        for c in m.contact:
            for i, n in enumerate(c.normal):
                if i < len(c.wrench):
                    w = c.wrench[i].body_1_wrench.force
                    f += abs(w.x * n.x + w.y * n.y + w.z * n.z); nw += 1
        with self.lock:
            self.pad[k] = (f, m.header.stamp.sec + m.header.stamp.nsec * 1e-9, nw)

    # -- helpers -----------------------------------------------------------
    def now(self):
        with self.lock:
            return self.t

    def wait(self, dt):
        t1 = self.now() + dt; dl = time.monotonic() + dt * 20 + 5
        while self.now() < t1:
            if time.monotonic() > dl:
                raise TimeoutError('sim clock stopped')
            time.sleep(0.005)

    def cmd(self, values, repeat=3):
        for _ in range(repeat):
            for j, v in values.items():
                self.pub[j].publish(Double(data=float(v)))
            time.sleep(0.05)

    def jaws(self, q):
        self.cmd({'jaw_left_joint': q, 'jaw_right_joint': q})

    def reach(self, targets, tol=0.01, timeout_s=8.0):
        """Re-send position commands until every joint is within tol [rad] (sim time)."""
        t_end = self.now() + timeout_s
        while self.now() < t_end:
            self.cmd(targets, repeat=1)
            with self.lock:
                err = max(abs(self.q.get(j, 1e9) - v) for j, v in targets.items())
            if err < tol:
                return True
            self.wait(0.1)
        raise TimeoutError(f'joints did not reach {targets}')

    def gripper(self):
        with self.lock:
            a_p, a_q = self.poses['armbot']; g_p, g_q = self.poses['gripper_base_link']
        pos, q = compose(a_p, a_q, g_p, g_q)
        return transform_point(pos, q, GRASP), q

    def jaw_force(self, jaw):
        with self.lock:
            t = self.t
            vals = [self.pad[(jaw, p)] for p in ('pad_a', 'pad_b')]
        return sum(f for f, st, _ in vals if t - st < 0.05), sum(n for _, _, n in vals)

    def cylinder(self):
        with self.lock:
            return self.poses['grasp_cylinder']


def run(b, q_cmd, out_dir):
    b.reach(REACH, tol=0.01); b.jaws(OPEN); b.wait(0.8)
    gp, gq = b.gripper()
    R = quat_to_rot(gq)
    cq = qmul(gq, (np.cos(-np.pi / 4), np.sin(-np.pi / 4), 0.0, 0.0))     # cylinder axis -> gripper y
    box = ("<sdf version='1.10'><model name='support'><static>true</static><link name='l'>"
           "<collision name='c'><geometry><box><size>0.02 0.02 0.02</size></box></geometry></collision>"
           "</link></model></sdf>")
    top = gp[2] - 0.025
    gz_service('create', 'gz.msgs.EntityFactory',
               f'sdf: "{box}", pose: {{position: {{x: {gp[0]}, y: {gp[1]}, z: {top - 0.0105}}}}}')
    gz_service('set_pose', 'gz.msgs.Pose',
               f'name: "grasp_cylinder", position: {{x: {gp[0]}, y: {gp[1]}, z: {gp[2] + 0.0005}}}, '
               f'orientation: {{w: {cq[0]}, x: {cq[1]}, y: {cq[2]}, z: {cq[3]}}}')
    b.wait(0.5)
    b.jaws(q_cmd); b.wait(1.0)
    rows = []

    def sample(phase):
        p, _ = b.cylinder(); gp_now, _ = b.gripper()
        fl, nl = b.jaw_force('jaw_left'); fr, nr = b.jaw_force('jaw_right')
        with b.lock:
            ql, qr = b.q.get('jaw_left_joint', np.nan), b.q.get('jaw_right_joint', np.nan)
        with b.lock:
            per = {f'{j[4]}{p[-1]}': round(b.pad[(j, p)][0], 3) for j, p in PADS}
            age = max(b.t - b.pad[k][1] for k in PADS)
        rows.append(dict(t=round(b.now(), 3), phase=phase, q_l=ql, q_r=qr, **per, max_age=round(age, 3),
                         F_l=fl, F_r=fr, n_wrench=nl + nr, disp=float(np.linalg.norm(p - gp_now))))

    t_end = b.now() + 0.5
    while b.now() < t_end:
        sample('closed'); b.wait(0.02)
    gz_service('remove', 'gz.msgs.Entity', 'name: "support", type: MODEL')
    t_end = b.now() + 3.0
    while b.now() < t_end:
        sample('hanging'); b.wait(0.02)
    b.jaws(OPEN)
    t_end = b.now() + 1.0
    while b.now() < t_end:
        sample('release'); b.wait(0.02)
    gz_service('set_pose', 'gz.msgs.Pose', 'name: "grasp_cylinder", position: {x: 1.0, y: 1.0, z: 0.03}, '
               'orientation: {w: 0.7071068, x: 0.7071068, y: 0, z: 0}')
    b.wait(0.3)
    path = os.path.join(out_dir, f'bench_q{q_cmd:+.2f}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    pre = [r for r in rows if r['phase'] == 'closed']; post = [r for r in rows if r['phase'] == 'hanging']
    m = lambda rs, k: float(np.nanmean([r[k] for r in rs])) if rs else float('nan')
    rel = [r for r in rows if r['phase'] == 'release']
    return dict(q_open_after_1s=round(rel[-1]['q_l'], 3) if rel else None, F_after_open=round(rel[-1]['F_l'], 2) if rel else None, q_cmd=q_cmd, q_contact=round(m(pre, 'q_l'), 3), F_l_closed=round(m(pre, 'F_l'), 2),
                F_r_closed=round(m(pre, 'F_r'), 2), q_r_contact=round(m(pre, 'q_r'), 3),
                F_l_hanging=round(m(post[-50:], 'F_l'), 2), F_r_hanging=round(m(post[-50:], 'F_r'), 2),
                disp_end_mm=round(1000 * post[-1]['disp'], 1), held=post[-1]['disp'] < 0.04,
                wrench_points=int(sum(r['n_wrench'] for r in rows)),
                pinch_axis=np.round(R[:, 2], 2).tolist(), cyl_axis=np.round(R[:, 1], 2).tolist(),
                grasp_point=np.round(gp, 3).tolist())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--qcmd', nargs='+', type=float, default=[0.30])
    ap.add_argument('--out', default='.')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    b = Bench()
    dl = time.monotonic() + 20
    while not all(k in b.poses for k in ('armbot', 'gripper_base_link', 'grasp_cylinder')):
        if time.monotonic() > dl:
            raise TimeoutError('no pose/info from the bench world; is it running?')
        time.sleep(0.1)
    for q in a.qcmd:
        print(run(b, q, a.out), flush=True)


if __name__ == '__main__':
    main()
