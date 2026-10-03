"""Pure-logic tests for the grasp experiment (no ROS graph, Gazebo or PX4)."""
import math
from collections import Counter

import numpy as np
import pytest

from px4_gripper_missions.experiment import classify_outcome, generate_protocol_plan
from px4_gripper_missions.geometry import axis_tilt_deg, enu_to_ned, ned_to_enu, rpy_to_quat
from px4_gripper_missions.mocap import gz_pose_to_px4, rot_to_quat

OUTCOME_KW = dict(slip_onset_m=0.010, drop_m=0.040, tilt_threshold_deg=15.0)


# -- protocol plan ------------------------------------------------------------

def test_plan_batteries_are_balanced_and_contain_one_empty_control():
    plan = generate_protocol_plan(batteries=4, per_battery=10, empty_per_battery=1, seed=3)
    assert len(plan) == 40
    for b in range(1, 5):
        c = Counter(t.scenario for t in plan if t.battery == b)
        assert c['E'] == 1
        assert sorted((c['A'], c['B'])) == [4, 5]
    total = Counter(t.scenario for t in plan)
    assert total['A'] == total['B'] == 18          # 5/4 alternates across batteries
    assert [t.index for t in plan] == list(range(40))


def test_plan_is_reproducible_and_seed_dependent():
    a = [t.scenario for t in generate_protocol_plan(3, 10, 1, seed=7)]
    b = [t.scenario for t in generate_protocol_plan(3, 10, 1, seed=7)]
    c = [t.scenario for t in generate_protocol_plan(3, 10, 1, seed=8)]
    assert a == b
    assert a != c


def test_plan_rejects_more_controls_than_slots():
    with pytest.raises(ValueError):
        generate_protocol_plan(1, 2, 3, seed=0)


# -- outcome labelling --------------------------------------------------------

def _series(disp, tilt=None, dt=0.02, t0=10.0):
    t = t0 + dt * np.arange(len(disp))
    return t, np.asarray(disp, float), np.zeros(len(disp)) if tilt is None else np.asarray(tilt, float)


def test_held_object_is_success():
    t, d, tilt = _series([0.0005] * 200)
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'success' and not o.slipped and math.isnan(o.t_drop_s)


def test_slipping_but_held_is_success_with_slip_flag():
    t, d, tilt = _series(np.linspace(0, 0.02, 200))
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'success' and o.slipped
    assert 0 < o.t_onset_s < 4


def test_straight_drop_is_slide_failure_with_timing():
    d = [0.0] * 50 + list(np.linspace(0, 0.2, 50)) + [0.2] * 100
    t, d, tilt = _series(d)
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'fail' and o.failure_mode == 'slide'
    assert o.t_onset_s < o.t_drop_s
    assert o.t_drop_s == pytest.approx(1.0 + 0.04 / 0.2 * 1.0, abs=0.05)


def test_rotation_before_drop_is_tilt_failure():
    # off-centre grasp pivoting round the fingers: centre 30 mm from the
    # pivot, so displacement = 2 * 0.03 * sin(theta / 2)
    theta = np.radians([0.0] * 50 + list(np.linspace(0, 120, 150)))
    d = 2 * 0.03 * np.sin(theta / 2) + np.concatenate([np.zeros(150), np.linspace(0, 0.1, 50)])
    t, d, tilt = _series(d, np.degrees(theta))
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'fail' and o.failure_mode == 'tilt'


def test_late_rotation_while_falling_out_is_still_a_slide():
    d = [0.0] * 50 + list(np.linspace(0, 0.06, 60)) + [0.06] * 90
    tilt = [0.0] * 80 + list(np.linspace(0, 30, 30)) + [30.0] * 90   # tilts only after ~30 mm
    t, d, tilt = _series(d, tilt)
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'fail' and o.failure_mode == 'slide'


def test_rotated_but_hanging_is_tilt_failure():
    t, d, tilt = _series([0.0] * 50 + [0.03] * 150, [0.0] * 50 + [60.0] * 150)
    o = classify_outcome(t, d, tilt, t_lift=10.0, **OUTCOME_KW)
    assert o.outcome == 'fail' and o.failure_mode == 'tilt'
    assert o.t_drop_s == pytest.approx(1.0, abs=0.03)


def test_empty_window_is_invalid():
    assert classify_outcome([], [], [], 0.0, **OUTCOME_KW).outcome == 'invalid'


# -- frames -------------------------------------------------------------------

def test_enu_ned_round_trip():
    v = (1.0, 2.0, 3.0)
    assert np.allclose(enu_to_ned(v), (2.0, 1.0, -3.0))
    assert np.allclose(ned_to_enu(enu_to_ned(v)), v)


def test_cylinder_axis_tilt():
    assert axis_tilt_deg(rpy_to_quat(math.pi / 2, 0, 0)) == pytest.approx(0.0, abs=1e-6)   # lying
    assert axis_tilt_deg(rpy_to_quat(0, 0, 0)) == pytest.approx(90.0)                       # upright
    assert axis_tilt_deg(rpy_to_quat(math.pi / 2 + math.radians(20), 0, 0)) == pytest.approx(20.0)


def test_mocap_facing_east_is_ned_yaw_90deg_and_body_above_model_origin():
    pos, q = gz_pose_to_px4((3.0, 1.0, 0.5), rpy_to_quat(0, 0, 0))
    assert np.allclose(pos, (1.0, 3.0, -(0.5 + 0.24)))
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    assert yaw == pytest.approx(math.pi / 2)


def test_mocap_yaw_follows_gazebo_heading():
    # Gazebo yaw +30 deg (ENU, counter-clockwise from east) -> NED yaw 60 deg
    _, (w, x, y, z) = gz_pose_to_px4((0, 0, 0), rpy_to_quat(0, 0, math.radians(30)))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    assert yaw == pytest.approx(60.0)


def test_rot_to_quat_identity_and_half_turn():
    assert np.allclose(rot_to_quat(np.eye(3)), (1, 0, 0, 0))
    q = rot_to_quat(np.diag([1.0, -1.0, -1.0]))       # 180 deg about x
    assert abs(q[1]) == pytest.approx(1.0)
