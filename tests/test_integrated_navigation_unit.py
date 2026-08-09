"""Tests for the black-box integrated navigation unit.

IntegratedNavUnit is a deliberate layer collapse (see its docstring and ADR
0009) -- these tests only check that it behaves as the simple stand-in it
claims to be, not anything about navigation performance.
"""

import math

import numpy as np
import pytest

from ose.interfaces import OwnStateSource
from ose.resource.integrated_navigation_unit import IntegratedNavParameters, IntegratedNavUnit
from ose.resource.reference_configs.reference_integrated_navigation_unit import STANDARD
from ose.resource.vehicle import Disturbance, VehicleState


def test_satisfies_own_state_source():
    unit = IntegratedNavUnit(STANDARD, rng=np.random.default_rng(0))
    assert isinstance(unit, OwnStateSource)


def test_estimate_before_update_raises():
    unit = IntegratedNavUnit(STANDARD, rng=np.random.default_rng(0))
    with pytest.raises(RuntimeError):
        unit.estimate(0.0)


def test_update_corrupts_truth():
    unit = IntegratedNavUnit(STANDARD, rng=np.random.default_rng(0))
    state = VehicleState(1000.0, -500.0, math.radians(15.0), 250.0, 16000.0)
    dist = Disturbance(wind_x_mps=12.0, wind_y_mps=-18.0)
    est = unit.update(0.0, state, dist)
    assert abs(est.p_x_m - state.p_x_m) > 1e-9 or abs(est.p_y_m - state.p_y_m) > 1e-9


def test_estimate_returns_last_update():
    unit = IntegratedNavUnit(STANDARD, rng=np.random.default_rng(0))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    dist = Disturbance()
    est = unit.update(3.0, state, dist)
    assert unit.estimate(3.0) is est


def test_declared_uncertainty_matches_configured_sigma():
    par = IntegratedNavParameters(position_sigma_m=7.0, heading_sigma_rad=0.01, airspeed_sigma_mps=2.0)
    unit = IntegratedNavUnit(par, rng=np.random.default_rng(1))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    dist = Disturbance()

    n = 3000
    p_errors = np.empty((n, 2))
    for i in range(n):
        est = unit.update(float(i), state, dist)
        p_errors[i] = [est.p_x_m - state.p_x_m, est.p_y_m - state.p_y_m]

    assert np.all(np.abs(p_errors.std(axis=0) - par.position_sigma_m) < 0.1 * par.position_sigma_m)
