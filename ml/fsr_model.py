"""Synthetic force-sensing resistor (FSR 402 class) read through a voltage divider and a 10-bit ADC.

Stage 1 of the FSR study: turn a per-trial force time series (newtons on the sensor) into the
ADC counts an Arduino Nano would log. All parameters live in ml/config/fsr402.yaml, which cites
the datasheet and marks the assumptions.

Error model, per trial (one actuation of the sensor):
    F_seen = F * g_part * g_trial * g_repeat
    g_part   ~ N(1, part_to_part)        one draw per sensor, fixed for the whole dataset
    g_trial  ~ N(1, per_trial_gain)      placement / contact coupling (the swept quantity)
    g_repeat ~ N(1, repeat_single_part)  datasheet single-part repeatability
then R_FSR(F_seen) with slow drift, the divider, ADC noise, 10-bit quantisation and saturation.
"""
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
import yaml

DEFAULT_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config', 'fsr402.yaml')


@dataclass
class FSRParams:
    r_at_1n_ohm: float = 22000.0
    alpha: float = 0.8
    actuation_force_n: float = 0.2
    max_force_n: float = 20.0
    r_open_ohm: float = 1.0e7
    v_supply: float = 5.0
    r_m_ohm: float = 10000.0
    adc_bits: int = 10
    noise_lsb: float = 1.0
    repeat_single_part: float = 0.02
    part_to_part: float = 0.06
    per_trial_gain: float = 0.0
    drift_per_decade: float = 0.05

    @classmethod
    def from_yaml(cls, path: Optional[str] = None, **overrides) -> 'FSRParams':
        c = yaml.safe_load(open(path or DEFAULT_CONFIG))
        p = cls(**c['force_curve'], v_supply=c['divider']['v_supply'], r_m_ohm=c['divider']['r_m_ohm'],
                adc_bits=c['adc']['bits'], noise_lsb=c['adc']['noise_lsb'], **c['errors'])
        for k, v in overrides.items():
            setattr(p, k, v)
        for k, default in vars(cls()).items():      # YAML may hand back strings (e.g. "1.0e7")
            setattr(p, k, type(default)(getattr(p, k)))
        return p

    @property
    def full_scale(self) -> int:
        return 2 ** self.adc_bits - 1


def resistance(force_n: np.ndarray, p: FSRParams) -> np.ndarray:
    """FSR resistance [ohm] for the force on the sensor (power law, open below actuation)."""
    f = np.clip(np.asarray(force_n, dtype=float), 0.0, p.max_force_n)
    r = np.full(f.shape, p.r_open_ohm)
    on = f >= p.actuation_force_n
    r[on] = p.r_at_1n_ohm * f[on] ** (-p.alpha)
    return r


def ideal_counts(force_n: np.ndarray, p: FSRParams) -> np.ndarray:
    """Noise-free, unquantised ADC reading (float counts) for a force: divider only."""
    r = resistance(force_n, p)
    return p.full_scale * p.r_m_ohm / (p.r_m_ohm + r)


def draw_gains(n_trials: int, p: FSRParams, rng: np.random.Generator) -> np.ndarray:
    """Per-trial multiplicative error on the force seen by the sensor (one sensor)."""
    g_part = 1.0 + p.part_to_part * rng.standard_normal()
    g = g_part * (1.0 + p.per_trial_gain * rng.standard_normal(n_trials)) \
        * (1.0 + p.repeat_single_part * rng.standard_normal(n_trials))
    return np.clip(g, 0.05, None)


def read(force_n: np.ndarray, t_since_contact_s: np.ndarray, p: FSRParams,
         rng: np.random.Generator, gains: Optional[np.ndarray] = None) -> np.ndarray:
    """ADC counts for force series of shape (trials, samples).

    `t_since_contact_s` has the same shape (time since the force first exceeded the actuation
    force; <= 0 before contact). `gains` are per-trial factors (draw_gains); None means 1."""
    f = np.asarray(force_n, dtype=float)
    g = np.ones(f.shape[0]) if gains is None else np.asarray(gains, dtype=float)
    f_seen = f * g[:, None]
    r = resistance(f_seen, p)
    t = np.clip(np.asarray(t_since_contact_s, dtype=float), 0.0, None)
    loaded = f_seen >= p.actuation_force_n
    r = np.where(loaded, r * (1.0 - p.drift_per_decade * np.log10(1.0 + t)), r)   # creep: R falls under load
    v = p.r_m_ohm / (p.r_m_ohm + r) * p.full_scale
    v = v + p.noise_lsb * rng.standard_normal(v.shape)
    return np.clip(np.round(v), 0, p.full_scale)
