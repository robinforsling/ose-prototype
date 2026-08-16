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

import math
from dataclasses import dataclass

import numpy as np
import pytest


from ose.equipment.air_data import AirDataSensor as AirDataSensorImpl
from ose.equipment.gnss import GnssReceiver
from ose.equipment.imu import Imu
from ose.equipment.reference_configs.reference_air_data import (
    STANDARD as AIR_DATA_STANDARD,
)
from ose.equipment.reference_configs.reference_gnss import STANDARD as GNSS_STANDARD
from ose.equipment.reference_configs.reference_imu import TACTICAL_GRADE
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
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


@dataclass
class Flight:
    """What a flight produced, beyond the published rows.

    `_fly` always returned a second value and every caller discarded it, so
    this fills that slot rather than changing nine signatures. `internal`
    carries what no consumer can see: the error in all ten error states
    against truth, paired with the filter's own covariance over them. Those
    states are where a defect can land without any published channel moving --
    see test_the_error_states_no_consumer_reads_are_consistent.
    """

    estimator: object
    imu: object
    gnss: object
    air: object
    internal: list                       # (error_10, P_10x10) per collected step


def _internal_error(estimator, imu, state, wind):
    """Error in the estimator's own error-state ordering, against truth.

    Ordering matches the module docstring of navigation_state_estimator:
    position, ground velocity, heading, accel bias, gyro bias, wind.
    """
    v_ground_true = state.v_mps * np.array(
        [math.cos(state.psi_rad), math.sin(state.psi_rad)]
    ) + np.array(wind)
    return np.concatenate([
        estimator.p - np.array([state.p_x_m, state.p_y_m]),
        estimator.v_ground - v_ground_true,
        [math.remainder(estimator.psi - state.psi_rad, 2.0 * math.pi)],
        estimator.bias_accel - imu.bias_accel,
        [estimator.bias_gyro - imu.bias_gyro],
        estimator.wind - np.array(wind),
    ])


def _fly(vehicle, t_end, seed=0, wind=(12.0, -18.0), outage=None, collect_from=0.0, record=None):
    initial_state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    imu, gnss, air, estimator = _build_components(initial_state, seed)
    state = initial_state
    dist = Disturbance(wind_x_mps=wind[0], wind_y_mps=wind[1])
    rows = []
    internal = []
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
            # Captured at the same instant as `est`, before the IMU sample is
            # ingested, so the bias compared against is the one that produced
            # the measurement just taken.
            internal.append((_internal_error(estimator, imu, state, wind),
                             estimator.P.copy()))
        state = step_rk4(vehicle, state, cmd, DT, dist)
        t += DT
    return rows, Flight(estimator, imu, gnss, air, internal)


@pytest.fixture
def vehicle():
    return reference_fighter()


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

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
@pytest.mark.performance
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


def _anees(errors, covariances):
    """Mean normalised estimation error squared, using the FULL covariance.

    np.linalg.solve rather than the diagonal, so correlations count. A filter
    can be honest on every diagonal and still be wrong about how the errors
    relate to each other, and a consumer forming any linear combination --
    range to a waypoint, cross-track error -- reads exactly that.
    """
    return float(np.mean([e @ np.linalg.solve(C, e) for e, C in zip(errors, covariances)]))


@pytest.mark.parametrize("seed", [0, 1, 2, 3])
@pytest.mark.performance
def test_the_whole_published_estimate_is_consistent(vehicle, seed):
    """All four published channels at once, against the full 4x4 covariance.

    test_filter_is_consistent above checks the heading channel alone. That is
    the channel the historical bug landed in, but it is one scalar out of the
    object consumers actually receive, and this repository has been caught
    three times by an assertion that covered the convenient part: the mass
    filter passed at fuel ANEES 1.05 while its full state sat at 9.07, and the
    damage had landed in the state nobody looked at.

    So this checks the object. Measured 2.2-4.8 across seeds against 4 degrees
    of freedom, mean 4.04 over eight seeds -- the filter is very slightly
    conservative. The bounds are wide enough not to be seed-fragile and narrow
    enough to catch what matters: the misalignment defect produced NEES near
    850, and a covariance inflated to hide an error would fall through the
    floor rather than the ceiling.
    """
    rows, _ = _fly(vehicle, 200.0, seed=seed, collect_from=130.0)

    errors = [
        np.array([
            e.p_x_m - s.p_x_m,
            e.p_y_m - s.p_y_m,
            math.remainder(e.psi_rad - s.psi_rad, 2.0 * math.pi),
            e.v_air_mps - s.v_mps,
        ])
        for _, e, s in rows
    ]
    anees = _anees(errors, [e.covariance for _, e, _ in rows])

    assert anees < 12.0, f"ANEES = {anees:.2f} against 4 dof, filter overconfident"
    assert anees > 0.8, f"ANEES = {anees:.2f} against 4 dof, covariance inflated"


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.performance
def test_the_published_ground_velocity_covariance_is_consistent(vehicle, seed):
    """Ground velocity is published now, so its uncertainty is a claim.

    It is the one a ground TRACK claim rests on: the 4x4 above is over heading
    and airspeed, both air-relative, and a track is not. Guidance projects this
    2x2 onto the track angle, so an overconfident block here becomes an
    overconfident hold accuracy two layers up, which is exactly the shape of
    failure the NEES tests in this repository exist for.

    Checked against the filter's own error block rather than against the
    published record, because the record carries a copy of it and comparing a
    copy to itself would prove nothing about the numbers.
    """
    _, flight = _fly(vehicle, 200.0, seed=seed, collect_from=130.0)
    errors = [e[slice(2, 4)] for e, _ in flight.internal]
    covariances = [P[slice(2, 4), slice(2, 4)] for _, P in flight.internal]

    # Measured across seeds: 0.50 - 2.24 against 2 dof, comparable to the wind
    # and bias blocks below.
    anees = _anees(errors, covariances)
    assert anees < 8.0, f"ground velocity ANEES = {anees:.2f} against 2 dof, overconfident"
    assert anees > 0.04, (
        f"ground velocity ANEES = {anees:.2f} against 2 dof, covariance inflated"
    )


def test_the_published_covariance_is_the_filters_own(vehicle):
    """The record carries the block, not a recomputation of it.

    Cheap, and it is the half the consistency test above cannot see: that test
    checks the filter's numbers are honest, this checks the published ones are
    those numbers.
    """
    _, flight = _fly(vehicle, 30.0, seed=0)
    estimator = flight.estimator
    published = estimator.estimate(30.0).ground_velocity_covariance

    assert published.shape == (2, 2)
    assert np.allclose(published, estimator.P[slice(2, 4), slice(2, 4)])
    # A copy, so a consumer holding the record cannot reach into the filter.
    published[0, 0] = 1.0e9
    assert estimator.P[2, 2] != 1.0e9


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.performance
def test_the_error_states_no_consumer_reads_are_consistent(vehicle, seed):
    """The IMU biases and the wind never reach a consumer, so no published
    channel moves when they go wrong -- and every test above would still pass.
    Ground velocity used to be in this set and is now published in its own
    right; it has its own check above.

    They are not inert: the accel bias is subtracted from every specific-force
    sample before mechanisation and the wind sets the airspeed residual, so an
    error here is integrated into position continuously. This is the same
    shape as the mass filter's tsfc_error state, and it is checked the same
    way, block by block against the filter's own covariance over them.

    Truth for the biases comes from the Imu, which propagates them as the
    Gauss-Markov processes the filter merely assumes. That the filter's prior
    and the sensor's actual behaviour agree is a property of this scenario,
    not of the filter -- see the estimator's module docstring.
    """
    _, flight = _fly(vehicle, 200.0, seed=seed, collect_from=130.0)
    errors = [e for e, _ in flight.internal]
    covariances = [P for _, P in flight.internal]

    # (name, slice, dof). Measured across seeds: accel 0.98-1.67, gyro
    # 0.14-1.06, wind 0.48-0.52 -- all conservative.
    for name, sl, dof in [("accel bias", slice(5, 7), 2),
                          ("gyro bias", slice(7, 8), 1),
                          ("wind", slice(8, 10), 2)]:
        anees = _anees([e[sl] for e in errors], [P[sl, sl] for P in covariances])
        assert anees < 4.0 * dof, (
            f"{name} ANEES = {anees:.2f} against {dof} dof, overconfident"
        )
        assert anees > 0.02 * dof, (
            f"{name} ANEES = {anees:.2f} against {dof} dof, covariance inflated "
            "to the point of carrying no information"
        )


@pytest.mark.slow
@pytest.mark.performance
def test_air_data_holds_airspeed_while_gnss_is_out(vehicle):
    """The aiding sources are not interchangeable, and losing one must degrade
    only what it was aiding.

    GNSS observes position and ground velocity; air data observes airspeed
    directly and keeps correcting throughout an outage. So position
    uncertainty should grow substantially while airspeed uncertainty stays
    down at the level the air-data sensor supports.

    The bound is ABSOLUTE, not a before/during ratio, and the difference
    matters. Airspeed is reconstructed as |v_ground - wind|, and with air data
    removed entirely the wind is never observed at all: its sigma stays pinned
    at the filter's 25 m/s prior, before the outage as well as during it. A
    ratio test is therefore flat in both configurations and passes with the
    air-data correction deleted -- which is how this test was first written,
    and what sabotaging _ingest_air revealed. Measured: 0.481 m/s with air
    data against 25.0 without, so the level discriminates by a factor of 50.

    STANDARD air data declares 1.0 m/s, so requiring better than 2.0 asserts
    the filter is genuinely using the measurement rather than merely holding a
    prior.
    """
    rows, _ = _fly(vehicle, 320.0, seed=0, outage=(150.0, 250.0))
    at = lambda tt: min(rows, key=lambda r: abs(r[0] - tt))[1]

    before, during = at(149.0), at(249.0)
    v_air_sigma = lambda e: math.sqrt(e.covariance[3, 3])

    assert during.position_sigma_m > 5.0 * before.position_sigma_m, (
        "position should degrade without GNSS"
    )
    assert v_air_sigma(during) < 2.0, (
        f"airspeed sigma is {v_air_sigma(during):.3f} m/s during the outage; "
        "air data declares 1.0 m/s, so the correction is not reaching the state"
    )
    assert v_air_sigma(during) < 1.5 * v_air_sigma(before), "and it must not drift"
    assert before.gnss_available and not during.gnss_available


@pytest.mark.parametrize("initial_error_deg", [5.0, 15.0])
@pytest.mark.performance
def test_a_badly_initialised_heading_converges(vehicle, initial_error_deg):
    """Alignment is somebody else's problem (the estimator takes a numeric
    guess, never truth) so it must tolerate a bad one.

    Fifteen degrees is far outside the 1 degree the nominal initialisation
    declares. The turn makes heading observable and the filter should recover
    to well inside its own final uncertainty; a filter that has convinced
    itself of a wrong heading is the failure this guards, since it would then
    reject the GNSS fixes that disagree.
    """
    initial_state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    imu, gnss, air, _ = _build_components(initial_state, seed=0)

    psi0 = initial_state.psi_rad + math.radians(initial_error_deg)
    estimator = InsGnssEstimator(
        [0.0, 0.0], psi0,
        initial_state.v_mps * np.array([math.cos(psi0), math.sin(psi0)]),
        initial_uncertainty=InitialUncertainty(
            heading_sigma_rad=math.radians(initial_error_deg)
        ),
    )

    state = initial_state
    dist = Disturbance(wind_x_mps=12.0, wind_y_mps=-18.0)
    t = 0.0
    while t < 200.0:
        cmd = _profile(vehicle, t, state)
        imu_m = imu.sample(t, DT, state, cmd, dist)
        if gnss.due(t):
            estimator.ingest(gnss.sample(t, state, dist))
        if air.due(t):
            estimator.ingest(air.sample(t, state))
        estimator.ingest(imu_m)
        state = step_rk4(vehicle, state, cmd, DT, dist)
        t += DT

    est = estimator.estimate(t)
    error_rad = math.remainder(est.psi_rad - state.psi_rad, 2.0 * math.pi)
    sigma_rad = math.sqrt(est.covariance[2, 2])

    assert abs(math.degrees(error_rad)) < 0.2, (
        f"heading still {math.degrees(error_rad):+.3f} deg out after 200 s"
    )
    assert abs(error_rad) < 3.0 * sigma_rad, "converged but overconfident about it"


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.performance
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


@pytest.mark.performance
def test_wind_is_estimated_after_a_turn(vehicle):
    """Wind needs heading diversity: unobservable straight, recovered by turning."""
    rows, _ = _fly(vehicle, 200.0, seed=0, wind=(12.0, -18.0))
    est = rows[-1][1]
    assert np.allclose(est.wind_estimate_mps, [12.0, -18.0], atol=3.0)


# --------------------------------------------------------------------------
# Degraded operation
# --------------------------------------------------------------------------

@pytest.mark.slow
@pytest.mark.performance
def test_gnss_outage_grows_uncertainty_and_recovers(vehicle):
    rows, _ = _fly(vehicle, 320.0, seed=0, outage=(150.0, 250.0))

    sigma = {t: e.position_sigma_m for t, e, _ in rows}
    before = sigma[min(sigma, key=lambda t: abs(t - 149.0))]
    during = sigma[min(sigma, key=lambda t: abs(t - 249.0))]
    after = sigma[min(sigma, key=lambda t: abs(t - 300.0))]

    assert during > 5.0 * before
    assert after < 2.0 * before


@pytest.mark.slow
@pytest.mark.performance
def test_outage_error_stays_within_its_own_bound(vehicle):
    """Dead reckoning must remain consistent, not merely bounded."""
    rows, _ = _fly(vehicle, 260.0, seed=1, outage=(150.0, 250.0), collect_from=150.0)
    inside = [
        math.hypot(e.p_x_m - s.p_x_m, e.p_y_m - s.p_y_m) <= 3.0 * e.position_sigma_m
        for _, e, s in rows
    ]
    assert np.mean(inside) > 0.95
