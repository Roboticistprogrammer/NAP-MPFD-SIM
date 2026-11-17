"""Pure-logic tests for the small-lift and shake setpoint profiles."""
import numpy as np
import pytest

from px4_gripper_missions.trajectory import (
    MIN_JERK_PEAK,
    duration_for_peak_speed,
    min_jerk,
    min_jerk_rate,
    shake_legs,
)


def test_min_jerk_endpoints_and_midpoint():
    assert min_jerk(0.0) == pytest.approx(0.0)
    assert min_jerk(1.0) == pytest.approx(1.0)
    assert min_jerk(0.5) == pytest.approx(0.5)


def test_min_jerk_is_clipped_outside_the_move():
    assert min_jerk(-0.3) == pytest.approx(0.0)
    assert min_jerk(1.7) == pytest.approx(1.0)


def test_min_jerk_starts_and_ends_at_rest():
    assert min_jerk_rate(0.0) == pytest.approx(0.0)
    assert min_jerk_rate(1.0) == pytest.approx(0.0)
    assert min_jerk_rate(0.5) == pytest.approx(MIN_JERK_PEAK)


def test_min_jerk_is_monotonic_and_matches_its_rate():
    s = np.linspace(0, 1, 2001)
    p = min_jerk(s)
    assert np.all(np.diff(p) >= 0)
    numeric = np.gradient(p, s)
    assert np.allclose(numeric[5:-5], min_jerk_rate(s)[5:-5], atol=1e-3)


def test_small_lift_covers_ten_centimetres_in_one_and_a_half_seconds():
    t = np.linspace(0, 1.5, 76)             # 50 Hz samples, t0 .. t0 + 1.5 s
    z = 0.10 * min_jerk(t / 1.5)
    assert z[0] == pytest.approx(0.0)
    assert z[-1] == pytest.approx(0.10)
    assert len(z) == 76


def test_duration_for_peak_speed():
    # 0.9 m at a 0.3 m/s peak -> 5.625 s
    assert duration_for_peak_speed(0.9, 0.3) == pytest.approx(5.625)
    # peak speed of the resulting move is the requested one
    d = duration_for_peak_speed(0.9, 0.3)
    assert 0.9 * MIN_JERK_PEAK / d == pytest.approx(0.3)
    with pytest.raises(ValueError):
        duration_for_peak_speed(1.0, 0.0)


def test_shake_default_sequence_is_plus_minus_and_back_to_centre():
    legs = shake_legs(0.3, [1, -1, 0], 1.0, [0, 1, 0])
    deltas = [d for d, _ in legs]
    assert np.allclose(deltas[0], (0, 0.3, 0))
    assert np.allclose(deltas[1], (0, -0.6, 0))
    assert np.allclose(deltas[2], (0, 0.3, 0))
    assert np.allclose(sum(deltas), 0.0)    # ends where it started
    assert all(dur == 1.0 for _, dur in legs)


def test_shake_axis_is_normalised_and_must_be_nonzero():
    legs = shake_legs(0.3, [1], 1.0, [0, 2, 0])
    assert np.allclose(legs[0][0], (0, 0.3, 0))
    with pytest.raises(ValueError):
        shake_legs(0.3, [1], 1.0, [0, 0, 0])
