"""Tests for the resource-layer clock.

Mirrors the pattern established for Imu (docs/refactor-navigation-split.md,
ADR 0009): declared elapsed_sigma_s covers only the white-noise part, and
the drift's Gauss-Markov behaviour is the resource's own true dynamics, not
something it declares.
"""

import dataclasses
import math

import numpy as np
import pytest

from ose import interfaces
from ose.resource.clock import Clock
from ose.resource.reference_configs.reference_clock import STANDARD


def test_satisfies_clock_sensor_protocol():
    clock = Clock(STANDARD, rng=np.random.default_rng(0))
    assert isinstance(clock, interfaces.ClockSensor)


def test_valid_time_equals_time_requested():
    clock = Clock(STANDARD, rng=np.random.default_rng(0))
    m = clock.sample(12.5, 0.05)
    assert m.valid_time_s == 12.5


def test_declared_sigma_matches_configured_white_noise():
    par = dataclasses.replace(STANDARD, white_noise_sigma_s=2.0e-7)
    clock = Clock(par, rng=np.random.default_rng(0))
    m = clock.sample(0.0, 0.05)
    assert m.elapsed_sigma_s == 2.0e-7


# --------------------------------------------------------------------------
# Declared uncertainty is honest
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def clock_draws():
    """Many draws at a fixed dt, reading the drift back out after each call
    so the residual isolates the white-noise term."""
    par = STANDARD
    rng = np.random.default_rng(1)
    clock = Clock(par, rng)

    dt = 0.05
    n = 6000
    residual = np.empty(n)
    t = 0.0
    for i in range(n):
        m = clock.sample(t, dt)
        residual[i] = m.elapsed_s - dt * (1.0 + clock.drift)
        t += dt
    return par, residual


def test_clock_mean_matches_true_elapsed_time_plus_drift(clock_draws):
    par, residual = clock_draws
    se = par.white_noise_sigma_s / math.sqrt(residual.shape[0])
    assert abs(residual.mean()) < 5.0 * se


def test_clock_std_matches_declared_sigma(clock_draws):
    par, residual = clock_draws
    assert abs(residual.std() - par.white_noise_sigma_s) < 0.1 * par.white_noise_sigma_s


def test_drift_reaches_gauss_markov_steady_state():
    par = dataclasses.replace(STANDARD, drift_sigma=1.0e-6)
    clock = Clock(par, rng=np.random.default_rng(3))

    dt = 20.0
    n_steps = 20000                     # T = 400,000 s, ~111 time constants
    drift = np.empty(n_steps)
    t = 0.0
    for i in range(n_steps):
        clock.sample(t, dt)
        drift[i] = clock.drift
        t += dt

    steady = drift[n_steps // 10 :]     # discard the initial transient
    ratio = np.var(steady) / par.drift_sigma**2
    assert 0.5 < ratio < 2.0


def test_drift_accumulates_a_systematic_offset_over_many_samples():
    """A biased drift should show up as a persistent sign, not average out --
    the whole point of modelling it separately from the white-noise term."""
    par = dataclasses.replace(STANDARD, drift_sigma=1.0e-6, white_noise_sigma_s=1.0e-9)
    clock = Clock(par, rng=np.random.default_rng(5))
    clock.drift = 1.0e-6                # force a known, fixed-for-this-test drift

    dt = 1.0
    t = 0.0
    total_true = 0.0
    total_reported = 0.0
    for _ in range(1000):
        m = clock.sample(t, dt)
        total_true += dt
        total_reported += m.elapsed_s
        t += dt

    # Drift decays slightly over the run (tau = 3600 s, run = 1000 s), so
    # check the accumulated offset is in the right ballpark and right sign
    # rather than pinning an exact value.
    offset = total_reported - total_true
    assert 0.0 < offset < 2.0e-6 * total_true
