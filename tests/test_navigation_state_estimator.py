"""Tests for the navigation estimator.

The important one is test_filter_is_consistent. A navigation filter that is
overconfident will silently corrupt every tracker and planner downstream, and
the fault is invisible in straight flight -- it only appears under turn. This
is exactly the class of defect the isolated lab environments exist to catch, so
it is checked here rather than left to inspection of a plot.

test_estimator_cannot_see_truth and test_replay_determinism check the two
properties the equipment/subsystem split exists to establish: that the
estimator's signature contains no truth-carrying type, and that it is a pure
function of the measurement stream it is fed.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from _truth_boundary import (
    assert_no_truth_parameters,
    assert_no_truth_types,
    component_path,
)

from ose.equipment.air_data import AirDataSensor as AirDataSensorImpl
from ose.equipment.gnss import GnssReceiver
from ose.equipment.imu import Imu
from ose.equipment.reference_configs.reference_air_data import (
    STANDARD as AIR_DATA_STANDARD,
)
from ose.equipment.reference_configs.reference_gnss import STANDARD as GNSS_STANDARD
from ose.equipment.reference_configs.reference_imu import TACTICAL_GRADE
from ose.equipment.reference_configs.reference_vehicle import reference_fighter
from ose.equipment.vehicle import Disturbance, VehicleCommand, VehicleState
from ose.integration import step_rk4
from ose.interfaces import NavigationEstimator
from ose.subsystem.navigation_state_estimator import (
    GnssFix,
    InitialUncertainty,
    InsGnssEstimator,
)

DT = 0.05
INITIAL = InitialUncertainty()


def _profile(vehicle, t, state):
    """Straight, then a sustained-rate turn, then straight again."""
    omega = vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg) if 40.0 <= t < 120.0 else 0.0
    thrust = vehicle.thrust_required_N(state.v_mps, state.mass_kg, omega)
    return vehicle.project_command(state, VehicleCommand(thrust, omega))[0]


def _build_components(initial_state, seed=0):
    """Independent RNG streams per component (ADR 0005), plus one, thrown
    away afterwards, that corrupts the initial guess handed to the
    estimator -- a scenario-setup detail, not a component."""
    imu = Imu(TACTICAL_GRADE, np.random.default_rng(seed), reference_fighter())
    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(seed + 1))
    air = AirDataSensorImpl(AIR_DATA_STANDARD, rng=np.random.default_rng(seed + 2))

    rng_init = np.random.default_rng(seed + 100)
    psi0 = initial_state.psi_rad + float(rng_init.normal(0.0, INITIAL.heading_sigma_rad))
    p0 = np.array([initial_state.p_x_m, initial_state.p_y_m]) + rng_init.normal(
        0.0, INITIAL.position_sigma_m, size=2
    )
    v0 = initial_state.v_mps * np.array(
        [math.cos(psi0), math.sin(psi0)]
    ) + rng_init.normal(0.0, INITIAL.velocity_sigma_mps, size=2)

    estimator = InsGnssEstimator(p0, psi0, v0, initial_uncertainty=INITIAL)
    return imu, gnss, air, estimator


def _fly(vehicle, t_end, seed=0, wind=(12.0, -18.0), outage=None, collect_from=0.0, record=None):
    initial_state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    imu, gnss, air, estimator = _build_components(initial_state, seed)
    state = initial_state
    dist = Disturbance(wind_x_mps=wind[0], wind_y_mps=wind[1])
    rows = []
    t = 0.0
    while t < t_end:
        if outage is not None:
            gnss.set_gnss_available(not (outage[0] <= t < outage[1]))
        cmd = _profile(vehicle, t, state)

        imu_m = imu.sample(t, DT, state, cmd, dist)
        fix = gnss.sample(t, state, dist) if gnss.due(t) else None
        air_m = air.sample(t, state) if air.due(t) else None

        if fix is not None:
            estimator.ingest(fix)
            if record is not None:
                record.append(("ingest", fix))
        if air_m is not None:
            estimator.ingest(air_m)
            if record is not None:
                record.append(("ingest", air_m))

        est = estimator.estimate(t)
        if record is not None:
            record.append(("estimate", t))

        estimator.ingest(imu_m)
        if record is not None:
            record.append(("ingest", imu_m))

        if t >= collect_from:
            rows.append((t, est, state))
        state = step_rk4(vehicle, state, cmd, DT, dist)
        t += DT
    return rows, estimator


@pytest.fixture
def vehicle():
    return reference_fighter()


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_estimator_cannot_see_truth():
    """Blunt by design: fails loudly if truth is reintroduced for convenience."""
    path = component_path("subsystem", "navigation_state_estimator.py")
    assert_no_truth_types(path)
    assert_no_truth_parameters(path)


def test_estimator_satisfies_the_protocol():
    estimator = InsGnssEstimator(np.zeros(2), 0.0, np.array([250.0, 0.0]))
    assert isinstance(estimator, NavigationEstimator)


def test_estimate_is_not_truth(vehicle):
    """An estimator that passes truth through is not doing estimation."""
    rows, _ = _fly(vehicle, 20.0, seed=7)
    _, est, truth = rows[-1]
    assert abs(est.p_x_m - truth.p_x_m) > 1e-6


# --------------------------------------------------------------------------
# Ordering contract
# --------------------------------------------------------------------------

def test_out_of_order_ingestion_raises_value_error():
    estimator = InsGnssEstimator(np.zeros(2), 0.0, np.array([250.0, 0.0]))
    fix = GnssFix(5.0, np.zeros(2), 3.0, None, None)
    estimator.ingest(fix)
    stale = GnssFix(1.0, np.zeros(2), 3.0, None, None)
    with pytest.raises(ValueError):
        estimator.ingest(stale)


def test_ingesting_unknown_type_raises_type_error():
    estimator = InsGnssEstimator(np.zeros(2), 0.0, np.array([250.0, 0.0]))
    with pytest.raises(TypeError):
        estimator.ingest(object())


# --------------------------------------------------------------------------
# Purity: replay determinism
# --------------------------------------------------------------------------

def test_replay_determinism(vehicle):
    """Proves the estimator is a pure function of its measurement stream."""
    initial_state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    record: list = []
    rows, _ = _fly(vehicle, 20.0, seed=11, outage=(5.0, 10.0), record=record)
    live_results = [est for _, est, _ in rows]

    # A fresh estimator with the identical initial guess (same seed, so
    # _build_components reproduces it exactly), fed only the recorded
    # measurement stream -- no sensors, no truth.
    *_, replay_estimator = _build_components(initial_state, seed=11)
    replay_results = []
    for kind, payload in record:
        if kind == "ingest":
            replay_estimator.ingest(payload)
        else:
            replay_results.append(replay_estimator.estimate(payload))

    assert len(live_results) == len(replay_results)
    for a, b in zip(live_results, replay_results):
        assert a.t_s == b.t_s
        assert a.p_x_m == b.p_x_m
        assert a.p_y_m == b.p_y_m
        assert a.psi_rad == b.psi_rad
        assert a.v_air_mps == b.v_air_mps
        assert np.array_equal(a.ground_velocity_mps, b.ground_velocity_mps)
        assert np.array_equal(a.wind_estimate_mps, b.wind_estimate_mps)
        assert np.array_equal(a.covariance, b.covariance)
        assert a.gnss_available == b.gnss_available


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
    rows, _ = _fly(vehicle, 200.0, seed=seed, collect_from=130.0)

    nees = []
    for _, est, truth in rows:
        err = math.remainder(est.psi_rad - truth.psi_rad, 2.0 * math.pi)
        var = max(est.covariance[2, 2], 1e-24)
        nees.append(err * err / var)

    assert np.mean(nees) < 6.0, f"heading NEES = {np.mean(nees):.1f}, filter overconfident"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_position_error_within_three_sigma(vehicle, seed):
    rows, _ = _fly(vehicle, 200.0, seed=seed, collect_from=30.0)

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
    rows, _ = _fly(vehicle, 35.0, seed=0)                # turn starts at t = 40
    first, last = rows[1][1], rows[-1][1]
    ratio = math.sqrt(last.covariance[2, 2] / first.covariance[2, 2])
    assert ratio > 0.9


def test_turning_makes_heading_observable(vehicle):
    """Lateral specific force couples heading error into velocity error."""
    rows, _ = _fly(vehicle, 130.0, seed=0)
    before = next(e for t, e, _ in rows if t >= 35.0)
    after = next(e for t, e, _ in rows if t >= 125.0)
    assert math.sqrt(after.covariance[2, 2] / before.covariance[2, 2]) < 0.1


def test_wind_is_estimated_after_a_turn(vehicle):
    """Wind needs heading diversity: unobservable straight, recovered by turning."""
    rows, _ = _fly(vehicle, 200.0, seed=0, wind=(12.0, -18.0))
    est = rows[-1][1]
    assert np.allclose(est.wind_estimate_mps, [12.0, -18.0], atol=3.0)


# --------------------------------------------------------------------------
# Degraded operation
# --------------------------------------------------------------------------

def test_gnss_outage_grows_uncertainty_and_recovers(vehicle):
    rows, _ = _fly(vehicle, 320.0, seed=0, outage=(150.0, 250.0))

    sigma = {t: e.position_sigma_m for t, e, _ in rows}
    before = sigma[min(sigma, key=lambda t: abs(t - 149.0))]
    during = sigma[min(sigma, key=lambda t: abs(t - 249.0))]
    after = sigma[min(sigma, key=lambda t: abs(t - 300.0))]

    assert during > 5.0 * before
    assert after < 2.0 * before


def test_outage_error_stays_within_its_own_bound(vehicle):
    """Dead reckoning must remain consistent, not merely bounded."""
    rows, _ = _fly(vehicle, 260.0, seed=1, outage=(150.0, 250.0), collect_from=150.0)
    inside = [
        math.hypot(e.p_x_m - s.p_x_m, e.p_y_m - s.p_y_m) <= 3.0 * e.position_sigma_m
        for _, e, s in rows
    ]
    assert np.mean(inside) > 0.95
