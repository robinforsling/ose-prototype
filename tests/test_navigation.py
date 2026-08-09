"""Tests for the navigation systems.

The important one is test_filter_is_consistent. A navigation filter that is
overconfident will silently corrupt every tracker and planner downstream, and
the fault is invisible in straight flight -- it only appears under turn. This
is exactly the class of defect the isolated lab environments exist to catch, so
it is checked here rather than left to inspection of a plot.
"""

import math

import numpy as np
import pytest

from ose.interfaces import NavigationSystem
from ose.resource.navigation import (
    AdditiveNoiseNavigation,
    AidingParameters,
    InsGnssNavigation,
)
from ose.resource.vehicle import (
    Disturbance,
    VehicleCommand,
    VehicleState,
    reference_fighter,
    step_rk4,
)

DT = 0.05


def _profile(vehicle, t, state):
    """Straight, then a sustained-rate turn, then straight again."""
    omega = vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg) if 40.0 <= t < 120.0 else 0.0
    thrust = vehicle.thrust_required_N(state.v_mps, state.mass_kg, omega)
    return vehicle.project_command(state, VehicleCommand(thrust, omega))[0]


def _fly(nav, vehicle, t_end, wind=(12.0, -18.0), outage=None, collect_from=0.0):
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    dist = Disturbance(wind_x_mps=wind[0], wind_y_mps=wind[1])
    rows = []
    t = 0.0
    while t < t_end:
        if outage is not None and hasattr(nav, "set_gnss_available"):
            nav.set_gnss_available(not (outage[0] <= t < outage[1]))
        cmd = _profile(vehicle, t, state)
        est = nav.update(t, DT, state, cmd, dist)
        if t >= collect_from:
            rows.append((t, est, state))
        state = step_rk4(vehicle, state, cmd, DT, dist)
        t += DT
    return rows


@pytest.fixture
def vehicle():
    return reference_fighter()


# --------------------------------------------------------------------------
# Interchangeability
# --------------------------------------------------------------------------

def test_both_implementations_satisfy_the_protocol(vehicle):
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    for nav in (
        AdditiveNoiseNavigation(rng=np.random.default_rng(0)),
        InsGnssNavigation(vehicle, state, rng=np.random.default_rng(0)),
    ):
        assert isinstance(nav, NavigationSystem)
        rows = _fly(nav, vehicle, 10.0)
        est = rows[-1][1]
        assert math.isfinite(est.p_x_m)
        assert est.covariance.shape == (4, 4)
        assert np.all(np.linalg.eigvals(est.covariance) > -1e-12)


def test_estimate_is_not_truth(vehicle):
    """A navigation system that passes truth through is not a sensor model."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(7))
    rows = _fly(nav, vehicle, 20.0)
    _, est, truth = rows[-1]
    assert abs(est.p_x_m - truth.p_x_m) > 1e-6


# --------------------------------------------------------------------------
# Filter consistency -- the one that matters
# --------------------------------------------------------------------------

@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_filter_is_consistent(vehicle, seed):
    """Normalised estimation error squared must be of order one.

    NEES far above one means the filter is overconfident. This test would have
    caught a one-step misalignment between the mechanised state and the truth
    used to form measurement residuals, which produced NEES near 850 while
    leaving straight-flight behaviour apparently perfect.
    """
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(seed))
    rows = _fly(nav, vehicle, 200.0, collect_from=130.0)

    nees = []
    for _, est, truth in rows:
        err = math.remainder(est.psi_rad - truth.psi_rad, 2.0 * math.pi)
        var = max(est.covariance[2, 2], 1e-24)
        nees.append(err * err / var)

    assert np.mean(nees) < 6.0, f"heading NEES = {np.mean(nees):.1f}, filter overconfident"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_position_error_within_three_sigma(vehicle, seed):
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(seed))
    rows = _fly(nav, vehicle, 200.0, collect_from=30.0)

    inside = [
        math.hypot(e.p_x_m - s.p_x_m, e.p_y_m - s.p_y_m) <= 3.0 * e.position_sigma_m
        for _, e, s in rows
    ]
    assert np.mean(inside) > 0.95


# --------------------------------------------------------------------------
# Observability structure
# --------------------------------------------------------------------------

def test_heading_unobservable_without_lateral_specific_force(vehicle):
    """In straight flight, GNSS position cannot observe heading.

    The heading variance must therefore not shrink appreciably before the
    first turn. If it does, the filter is claiming information it has no
    physical source for.
    """
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(0))
    rows = _fly(nav, vehicle, 35.0)                     # turn starts at t = 40
    first, last = rows[1][1], rows[-1][1]
    ratio = math.sqrt(last.covariance[2, 2] / first.covariance[2, 2])
    assert ratio > 0.9


def test_turning_makes_heading_observable(vehicle):
    """Lateral specific force couples heading error into velocity error."""
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(0))
    rows = _fly(nav, vehicle, 130.0)
    before = next(e for t, e, _ in rows if t >= 35.0)
    after = next(e for t, e, _ in rows if t >= 125.0)
    assert math.sqrt(after.covariance[2, 2] / before.covariance[2, 2]) < 0.1


def test_wind_is_estimated_after_a_turn(vehicle):
    """Wind needs heading diversity: unobservable straight, recovered by turning."""
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(0))
    rows = _fly(nav, vehicle, 200.0, wind=(12.0, -18.0))
    est = rows[-1][1]
    assert np.allclose(est.wind_estimate_mps, [12.0, -18.0], atol=3.0)


# --------------------------------------------------------------------------
# Degraded operation
# --------------------------------------------------------------------------

def test_gnss_outage_grows_uncertainty_and_recovers(vehicle):
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(0))
    rows = _fly(nav, vehicle, 320.0, outage=(150.0, 250.0))

    sigma = {t: e.position_sigma_m for t, e, _ in rows}
    before = sigma[min(sigma, key=lambda t: abs(t - 149.0))]
    during = sigma[min(sigma, key=lambda t: abs(t - 249.0))]
    after = sigma[min(sigma, key=lambda t: abs(t - 300.0))]

    assert during > 5.0 * before
    assert after < 2.0 * before


def test_outage_error_stays_within_its_own_bound(vehicle):
    """Dead reckoning must remain consistent, not merely bounded."""
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    nav = InsGnssNavigation(vehicle, state, rng=np.random.default_rng(1))
    rows = _fly(nav, vehicle, 260.0, outage=(150.0, 250.0), collect_from=150.0)
    inside = [
        math.hypot(e.p_x_m - s.p_x_m, e.p_y_m - s.p_y_m) <= 3.0 * e.position_sigma_m
        for _, e, s in rows
    ]
    assert np.mean(inside) > 0.95
