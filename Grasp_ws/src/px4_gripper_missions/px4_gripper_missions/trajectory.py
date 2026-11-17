"""Setpoint profiles for the small-lift protocol (pure numpy, no ROS/Gazebo).

    min_jerk(s)               position fraction of a minimum-jerk move, s in [0, 1]
    min_jerk_rate(s)          its derivative with respect to s (peak 1.875 at s = 0.5)
    duration_for_peak_speed   move duration that gives a chosen peak speed
    shake_legs                the (displacement, duration) legs of the stability shake
"""
from __future__ import annotations

from typing import List, Sequence, Tuple

import numpy as np

# Peak of min_jerk_rate: a minimum-jerk move of length D lasting T peaks at 1.875 * D / T.
MIN_JERK_PEAK = 1.875


def min_jerk(s):
    """10 s^3 - 15 s^4 + 6 s^5, clipped to s in [0, 1]. Zero velocity and
    acceleration at both ends."""
    s = np.clip(np.asarray(s, dtype=float), 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def min_jerk_rate(s):
    """d(min_jerk)/ds; zero outside [0, 1]."""
    s_arr = np.asarray(s, dtype=float)
    inside = (s_arr >= 0.0) & (s_arr <= 1.0)
    c = np.clip(s_arr, 0.0, 1.0)
    return np.where(inside, 30 * c ** 2 - 60 * c ** 3 + 30 * c ** 4, 0.0)


def duration_for_peak_speed(distance_m: float, peak_speed_mps: float) -> float:
    """Duration of a minimum-jerk move of `distance_m` whose peak speed is
    `peak_speed_mps`."""
    if peak_speed_mps <= 0:
        raise ValueError('peak_speed_mps must be positive')
    return MIN_JERK_PEAK * abs(distance_m) / peak_speed_mps


def shake_legs(amplitude_m: float, sequence: Sequence[float], leg_duration_s: float,
               axis: Sequence[float]) -> List[Tuple[np.ndarray, float]]:
    """Legs of the stability shake: (displacement vector, duration) pairs.

    `sequence` lists the positions to visit as fractions of the amplitude
    along `axis`, starting from the hold position (0). The default
    [1, -1, 0] is centre -> +A -> -A -> centre."""
    a = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(a))
    if norm == 0.0:
        raise ValueError('shake axis must be non-zero')
    a = a / norm
    legs: List[Tuple[np.ndarray, float]] = []
    prev = 0.0
    for frac in sequence:
        legs.append(((float(frac) - prev) * amplitude_m * a, float(leg_duration_s)))
        prev = float(frac)
    return legs
