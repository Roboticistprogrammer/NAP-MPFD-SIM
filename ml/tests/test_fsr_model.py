import numpy as np
import pytest

from ml.fsr_model import FSRParams, draw_gains, ideal_counts, read, resistance
from ml.fsr_stage1 import force_window


def quiet(**kw):
    """Parameters with every random and slow error switched off."""
    base = dict(noise_lsb=0.0, repeat_single_part=0.0, part_to_part=0.0, per_trial_gain=0.0,
                drift_per_decade=0.0)
    base.update(kw)
    return FSRParams(**base)


def test_config_loads_datasheet_values():
    p = FSRParams.from_yaml()
    assert p.actuation_force_n == pytest.approx(0.2)
    assert p.max_force_n == pytest.approx(20.0)
    assert p.repeat_single_part == pytest.approx(0.02)
    assert p.part_to_part == pytest.approx(0.06)
    assert p.full_scale == 1023
    assert isinstance(p.r_open_ohm, float) and p.r_open_ohm == pytest.approx(1e7)
    assert FSRParams.from_yaml(per_trial_gain=0.25).per_trial_gain == pytest.approx(0.25)


def test_resistance_falls_with_force_and_is_open_below_actuation():
    p = quiet()
    r = resistance(np.array([0.0, 0.1, 0.5, 1.0, 3.0, 10.0]), p)
    assert r[0] == r[1] == pytest.approx(p.r_open_ohm)
    assert np.all(np.diff(r[2:]) < 0)
    assert r[3] == pytest.approx(p.r_at_1n_ohm)


def test_counts_monotonic_quantised_and_saturating():
    p = quiet()
    f = np.linspace(0.0, 40.0, 400)[None, :]
    c = read(f, np.zeros_like(f), p, np.random.default_rng(0))
    assert np.all(np.diff(c[0]) >= 0)
    assert np.all(c == np.round(c)) and c.min() >= 0 and c.max() <= 1023
    assert c[0, -1] == c[0, -20]            # clipped above the 20 N range
    assert c[0, 0] <= 1                     # open sensor reads about zero


def test_noise_free_read_matches_ideal_curve():
    p = quiet()
    f = np.array([[1.0, 1.5, 2.75]])
    c = read(f, np.zeros_like(f), p, np.random.default_rng(0))
    assert np.allclose(c, np.round(ideal_counts(f, p)))


def test_gain_spread_matches_parameter():
    p = quiet(per_trial_gain=0.10)
    g = draw_gains(20000, p, np.random.default_rng(1))
    assert np.std(g) == pytest.approx(0.10, rel=0.05)
    assert np.mean(g) == pytest.approx(1.0, abs=0.01)


def test_drift_raises_the_reading_under_constant_load():
    p = quiet(drift_per_decade=0.05)
    f = np.full((1, 3), 1.5)
    t = np.array([[0.0, 1.0, 10.0]])
    c = read(f, t, p, np.random.default_rng(0))
    assert c[0, 0] < c[0, 2]


def test_force_window_samples_on_the_50hz_grid():
    t = np.arange(0.0, 5.0, 0.02)
    cmd = np.where(t >= 1.0, 1.4, -2.0)          # open command before the close, as in the CSVs
    w, tc = force_window(t, cmd, t_start=0.5, n=50)
    assert w.shape == (50,) and tc.shape == (50,)
    assert np.all(w[:25] == 0.0)                 # negative = opening command, no contact force
    assert np.allclose(w[26:], 1.4)
    assert tc[-1] == pytest.approx(0.48, abs=0.02)
