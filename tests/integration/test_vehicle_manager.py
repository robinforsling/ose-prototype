"""Tests for the vehicle manager.

The load-bearing one is test_the_filter_is_consistent_through_the_run.
Everything else here checks that a number is correct; that one checks that the
component's *stated uncertainty* is correct, which is the property the rest of
the system relies on and the only one that catches an overconfident filter.
It is set up as an ensemble because consistency is not a property of a single
run: the true burn coefficient and the true initial fuel are drawn from the
priors the filter assumes, which is what makes the question well posed.

It checks the whole state, not just the fuel channel a consumer reads, and it
flies a throttle profile as well as steady cruise. Both were added after
finding that a constant unmodelled fuel sink -- a power generator, the planned
case -- leaves the fuel channel at ANEES 1.05 while the full state sits at
9.07: the damage lands entirely in the burn-coefficient state, and a scalar
check on the other one passes.
test_the_whole_state_is_checked_not_just_the_fuel pins that.

test_only_the_vehicle_manager_binds_the_vehicle_model is ADR 0015's rule in
the form that can actually be checked.

test_capability_is_evaluated_at_the_believed_mass is why the component exists
at all: a belief that does not move the reported envelope is not being bound
to anything.
"""

import ast
import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from _truth_boundary import (
    is_truth_package,
    vehicle_model_names,
    assert_no_truth_parameters,
    assert_no_truth_types,
    component_path,
)

from ose.equipment.fuel_gauge import FuelGauge
from ose.equipment.reference_configs.reference_fuel_gauge import STANDARD as GAUGE
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
from ose.equipment.reference_configs.vehicle.planar_point_mass_with_booster import (
    reference_boosted_fighter,
)
from ose.equipment.vehicle import Mode, PlanarPointMass, VehicleCommand, VehicleState
from ose.integration import step_rk4
from ose.interfaces import (
    FuelMeasurement,
    MassEstimate,
    OwnStateEstimate,
    PromisedEnvelope,
)
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    BELIEVED_TSFC_KG_PER_N_S,
    STANDARD,
)
from ose.subsystem.vehicle_manager import VehicleManager


def _estimate(v_mps: float = 250.0, t_s: float = 0.0) -> OwnStateEstimate:
    v = v_mps * np.array([1.0, 0.0])
    return OwnStateEstimate(
        t_s=t_s,
        p_x_m=0.0,
        p_y_m=0.0,
        psi_rad=0.0,
        v_air_mps=v_mps,
        ground_velocity_mps=v,
        wind_estimate_mps=np.zeros(2),
        covariance=np.zeros((4, 4)),
    )


def _params(**overrides):
    return dataclasses.replace(STANDARD, **overrides)


def _believing(vehicle, fuel_kg: float) -> VehicleManager:
    """A manager confident of a particular fuel load.

    The tests below that are about mass binding rather than about filtering
    want the belief placed exactly, so they set it as the initial condition
    with a negligible sigma rather than driving measurements into it and
    reasoning about the resulting blend.
    """
    return VehicleManager(
        vehicle, _params(initial_fuel_kg=fuel_kg, initial_fuel_sigma_kg=1e-6)
    )


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def manager(vehicle):
    return VehicleManager(vehicle, STANDARD)


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# The mass belief
# --------------------------------------------------------------------------

def test_mass_is_the_sum_of_its_parts(vehicle, manager):
    est = manager.mass(0.0)
    assert isinstance(est, MassEstimate)
    assert est.dry_mass_kg == vehicle.lam.mass_dry_kg
    assert est.payload_mass_kg == STANDARD.payload_mass_kg
    assert est.fuel_mass_kg == STANDARD.initial_fuel_kg
    assert est.mass_kg == pytest.approx(
        est.dry_mass_kg + est.payload_mass_kg + est.fuel_mass_kg
    )
    # The reference platform every other test flies.
    assert est.mass_kg == pytest.approx(16000.0)


def test_payload_adds_to_the_mass(vehicle):
    """Not a placeholder field: it has to reach the sum, or an effector added
    later would weigh nothing."""
    loaded = VehicleManager(vehicle, _params(payload_mass_kg=750.0))
    assert loaded.mass_kg == pytest.approx(16750.0)


def test_answers_before_any_measurement(manager):
    """A platform has a mass from the instant it exists. Refusing until the
    gauge speaks would put a first-cycle special case in every consumer."""
    est = manager.mass(0.0)
    assert est.fuel_mass_kg == STANDARD.initial_fuel_kg
    assert est.mass_sigma_kg == pytest.approx(STANDARD.initial_fuel_sigma_kg)


@pytest.mark.performance
def test_mass_sigma_is_the_fuel_sigma(manager):
    """Dry mass and payload are exact, so the uncertainty in the mass is
    exactly the uncertainty in the fuel. Derived from the covariance rather
    than stored separately, so the two cannot drift apart."""
    manager.ingest(FuelMeasurement(1.0, 3200.0, 20.0))
    est = manager.mass(1.0)
    assert est.mass_sigma_kg == pytest.approx(math.sqrt(est.covariance[0, 0]))


def test_a_measurement_moves_the_belief_and_sharpens_it(manager):
    """A first reading against a much vaguer prior should nearly replace it,
    and must leave the belief sharper than either input."""
    prior = manager.mass(0.0)
    manager.ingest(FuelMeasurement(1.0, 3200.0, 20.0))
    post = manager.mass(1.0)

    assert post.fuel_mass_kg == pytest.approx(3200.0, abs=15.0)   # nearly there
    assert post.mass_sigma_kg < 20.0                              # and sharper
    assert post.mass_sigma_kg < prior.mass_sigma_kg


@pytest.mark.performance
def test_a_worse_declared_sigma_is_trusted_less(vehicle):
    """Invariant 4. The declared uncertainty travelling with the measurement
    is what sets the gain, not a configured value of the manager's own, so a
    reading that admits to being poor must move the belief less.

    Two managers in identical states, given readings that differ only in
    their declared sigma.
    """
    good, poor = VehicleManager(vehicle, STANDARD), VehicleManager(vehicle, STANDARD)
    good.ingest(FuelMeasurement(1.0, 3200.0, 20.0))
    poor.ingest(FuelMeasurement(1.0, 3200.0, 400.0))

    start = STANDARD.initial_fuel_kg
    moved_good = abs(good.mass(1.0).fuel_mass_kg - start)
    moved_poor = abs(poor.mass(1.0).fuel_mass_kg - start)

    assert moved_good > moved_poor
    assert good.mass(1.0).mass_sigma_kg < poor.mass(1.0).mass_sigma_kg


def test_rejects_measurements_it_cannot_use(manager):
    with pytest.raises(TypeError):
        manager.ingest(object())


def test_rejects_measurements_out_of_order(manager):
    manager.ingest(FuelMeasurement(5.0, 3200.0, 20.0))
    with pytest.raises(ValueError):
        manager.ingest(FuelMeasurement(4.0, 3200.0, 20.0))


# --------------------------------------------------------------------------
# Prediction
# --------------------------------------------------------------------------

def test_prediction_burns_fuel_at_the_commanded_thrust(manager):
    """The point of dead reckoning: between readings the belief follows the
    burn instead of sitting still."""
    before = manager.mass(0.0).fuel_mass_kg
    manager.predict(10.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
    after = manager.mass(10.0).fuel_mass_kg

    expected = BELIEVED_TSFC_KG_PER_N_S * 60_000.0 * 10.0      # 15 kg
    assert before - after == pytest.approx(expected)


def test_prediction_grows_the_uncertainty(manager):
    """A belief propagated without correction must get less certain, or the
    covariance is not describing anything."""
    before = manager.mass(0.0).mass_sigma_kg
    manager.predict(30.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
    assert manager.mass(30.0).mass_sigma_kg > before


def test_process_noise_grows_the_belief_even_with_no_burn(manager):
    """Isolates Qc. At zero thrust the state-transition coupling vanishes --
    F is all zeros, so Phi is the identity -- and the only thing left that can
    grow the covariance is the process noise. Without this the removal of Qc
    goes unnoticed, because at any non-zero thrust the tsfc coupling grows
    P[fuel, fuel] on its own and the sawtooth test still passes.
    """
    before = manager.mass(0.0).covariance[0, 0]
    manager.predict(10.0, 0.0, BELIEVED_TSFC_KG_PER_N_S)
    after = manager.mass(10.0).covariance[0, 0]

    assert after > before
    assert after - before == pytest.approx(
        STANDARD.fuel_walk_kg_per_sqrt_s**2 * 10.0
    )


def test_a_dry_tank_stops_inflating_the_fuel_uncertainty(vehicle):
    """The burn gate, isolated from the clamp that hides it.

    Clamping fuel at zero keeps the reported value sane whether or not the
    gate exists, so the value alone cannot show the gate working. The
    uncertainty can: with nothing being consumed there is no burn-coefficient
    error accumulating, so P[fuel, fuel] must grow by the walk term only, not
    by the tsfc coupling as well.
    """
    manager = _believing(vehicle, 0.0)
    before = manager.mass(0.0).covariance[0, 0]
    manager.predict(20.0, 130_000.0, BELIEVED_TSFC_KG_PER_N_S)
    after = manager.mass(20.0).covariance[0, 0]

    assert after - before == pytest.approx(
        STANDARD.fuel_walk_kg_per_sqrt_s**2 * 20.0
    )


def test_the_uncertainty_sawtooths(vehicle):
    """Grows while predicting, drops at every correction. That shape is what
    makes the sigma usable by a consumer: it says how stale the belief is,
    which the previous sum-only version could not."""
    manager = VehicleManager(vehicle, STANDARD)
    manager.ingest(FuelMeasurement(0.0, 4000.0, 20.0))

    sigmas = []
    for k in range(1, 6):
        t = float(k)
        manager.predict(t, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
        sigmas.append(("predicted", manager.mass(t).mass_sigma_kg))
        manager.ingest(FuelMeasurement(t, 4000.0 - 1.5 * t, 20.0))
        sigmas.append(("corrected", manager.mass(t).mass_sigma_kg))

    for k in range(1, len(sigmas)):
        kind, value = sigmas[k]
        previous = sigmas[k - 1][1]
        if kind == "predicted":
            assert value > previous, f"sigma did not grow while predicting at {k}"
        else:
            assert value < previous, f"sigma did not drop on correction at {k}"


def test_prediction_stops_at_a_dry_tank(vehicle):
    """The vehicle stops burning at dry mass, so the filter must too. Without
    the gate it predicts fuel through zero and reports a mass below the
    airframe's own dry mass, which is not a state the vehicle can be in."""
    manager = _believing(vehicle, 5.0)
    manager.predict(600.0, 130_000.0, BELIEVED_TSFC_KG_PER_N_S)

    assert manager.mass(600.0).fuel_mass_kg == 0.0
    assert manager.mass_kg == pytest.approx(vehicle.lam.mass_dry_kg)


def test_prediction_backwards_is_a_no_op(manager):
    manager.predict(10.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
    fuel = manager.mass(10.0).fuel_mass_kg
    manager.predict(5.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
    assert manager.mass(10.0).fuel_mass_kg == pytest.approx(fuel)


# --------------------------------------------------------------------------
# Honesty of the stated uncertainty
# --------------------------------------------------------------------------

# Thrust profiles for the consistency ensembles.
#
# The navigation filter's consistency test flies a turn on purpose -- its
# docstring records that the bug it was written for "is invisible in straight
# flight, it only appears under turn". These tests were written at constant
# thrust and did not carry that lesson over.
def cruise(t_s: float) -> float:
    return 60_000.0


def throttled(t_s: float) -> float:
    """Cruise, accelerate, near idle, cruise.

    A constant unmodelled fuel sink -- a power generator, say -- is exactly
    degenerate with a burn-coefficient error while thrust is constant, since
    both are then a fixed number of kilograms per second. Varying the throttle
    breaks that degeneracy, so the filter has to tell the two apart rather
    than being allowed to confuse them harmlessly.
    """
    if t_s < 40.0:
        return 60_000.0
    if t_s < 70.0:
        return 125_000.0
    if t_s < 100.0:
        return 12_000.0
    return 60_000.0


def _fly_one(seed: int, checkpoints, dt: float, thrust=cruise, gauge_par=GAUGE,
             extra_drain_kg_s: float = 0.0, payload_kg: float = 0.0):
    """One ensemble member. Returns (fuel NEES, full-state NEES) per checkpoint.

    Truth is drawn from the priors the filter declares -- the burn
    coefficient from tsfc_sigma_fraction, the initial fuel from
    initial_fuel_sigma_kg. That is what makes consistency a well-posed
    question rather than a statement about one arbitrary run.

    Both NEES values are returned because they answer different questions and
    the fuel channel alone can look healthy while the filter is not. See
    test_the_whole_state_is_checked_not_just_the_fuel.

    extra_drain_kg_s burns fuel that the filter's model knows nothing about.
    Zero for the consistency tests; non-zero only to prove those tests can
    actually fail.

    payload_kg exists because every fixture in this file used to leave it at
    zero, and a whole class of error is invisible there. The gauge reports mass
    above DRY, while the manager decomposes mass as dry + payload + fuel, so
    the two agree only when there is no payload -- and a filter corrected on a
    reading that silently includes payload absorbs it and reports a mass wrong
    by exactly that much, at a stated sigma of about 1.4 kg. This is the same
    shape as the navigation bug in CLAUDE.md, which was invisible in straight
    flight: excite the thing the fixture holds constant.
    """
    streams = np.random.SeedSequence(seed).spawn(2)
    truth_rng = np.random.default_rng(streams[0])
    gauge_rng = np.random.default_rng(streams[1])

    tsfc_error = float(truth_rng.normal(0.0, STANDARD.tsfc_sigma_fraction))
    fuel0 = STANDARD.initial_fuel_kg + float(
        truth_rng.normal(0.0, STANDARD.initial_fuel_sigma_kg)
    )

    nominal = reference_fighter()
    vehicle = PlanarPointMass(
        dataclasses.replace(
            nominal.theta, c_tsfc=BELIEVED_TSFC_KG_PER_N_S * (1.0 + tsfc_error)
        ),
        nominal.lam,
        nominal.eta,
    )
    gauge = FuelGauge(gauge_par, vehicle.lam.mass_dry_kg, gauge_rng)
    manager = VehicleManager(vehicle, _params(payload_mass_kg=payload_kg))

    state = VehicleState(
        0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg + payload_kg + fuel0
    )

    out, k, t = [], 0, 0.0
    while k < len(checkpoints):
        if gauge.due(t):
            manager.ingest(gauge.sample(t, state))
        thrust_N = thrust(t)
        manager.predict(t + dt, thrust_N, BELIEVED_TSFC_KG_PER_N_S)
        state = step_rk4(vehicle, state, VehicleCommand(thrust_N, 0.0), dt)
        if extra_drain_kg_s:
            state = dataclasses.replace(
                state,
                mass_kg=max(state.mass_kg - extra_drain_kg_s * dt,
                            vehicle.lam.mass_dry_kg + payload_kg),
            )
        t += dt
        if t >= checkpoints[k] - 1e-9:
            est = manager.mass(t)
            error = np.array([
                est.fuel_mass_kg
                - (state.mass_kg - vehicle.lam.mass_dry_kg - payload_kg),
                est.tsfc_error - tsfc_error,
            ])
            out.append((
                error[0] ** 2 / est.covariance[0, 0],
                float(error @ np.linalg.solve(est.covariance, error)),
            ))
            k += 1
    return out


CHECKPOINTS = [10.0, 30.0, 60.0, 100.0, 150.0]


@pytest.mark.parametrize("payload_kg", [0.0, 750.0], ids=["clean", "loaded"])
@pytest.mark.parametrize("thrust", [cruise, throttled], ids=["cruise", "throttled"])
@pytest.mark.slow
@pytest.mark.performance
def test_the_filter_is_consistent_through_the_run(thrust, payload_kg):
    """The honesty test. Ensemble-average NEES must sit near its expectation
    at every checkpoint, not only at the end.

    Below expectation the filter is overconservative and a consumer steering
    on the sigma gives away performance. Above it the filter is overconfident,
    which is the failure that silently corrupts everything downstream -- the
    INS/GNSS bug in CLAUDE.md was exactly this and finished thirty times
    overconfident while every plot looked correct. The bound is therefore
    two-sided, and checked through the run because a filter can be calibrated
    at the end while wrong in the middle.

    Both the fuel channel alone and the whole state are checked. They are not
    the same question: see test_the_whole_state_is_checked_not_just_the_fuel.

    Chi-square with k degrees of freedom has mean k and variance 2k, so the
    95 per cent band on the mean of N samples is k +- 1.96*sqrt(2k/N).
    """
    n_runs = 100
    rows = np.array([_fly_one(s, CHECKPOINTS, dt=0.25, thrust=thrust,
                              payload_kg=payload_kg)
                     for s in range(n_runs)])          # (run, checkpoint, 2)

    for dof, channel, label in ((1, 0, "fuel"), (2, 1, "whole state")):
        band = 1.96 * math.sqrt(2.0 * dof / n_runs)
        for t_s, column in zip(CHECKPOINTS, rows[:, :, channel].T):
            mean = column.mean()
            assert abs(mean - dof) < band, (
                f"{label} ANEES {mean:.2f} at t={t_s} s is outside "
                f"[{dof - band:.2f}, {dof + band:.2f}] -- the filter is "
                f"{'over' if mean > dof else 'under'}confident"
            )


@pytest.mark.slow
def test_the_whole_state_is_checked_not_just_the_fuel():
    """Why the test above checks two degrees of freedom rather than one.

    The filter carries two states, and an unmodelled disturbance does not
    have to land in the one a consumer reads. A constant fuel sink the model
    knows nothing about -- a power generator is the planned example -- is
    absorbed almost entirely by the burn-coefficient state: the fuel channel
    stays inside its band while the coefficient estimate quietly goes wrong,
    which is worse than failing.

    So this plants exactly that disturbance and asserts the fuel channel
    alone would NOT have caught it, while the full state does. If the fuel
    channel ever starts catching this on its own the test is over-specified
    and can be simplified; if the full state stops catching it, the
    consistency test above has lost its teeth.

    0.05 kg/s is about three per cent of the cruise burn -- a plausible
    generator, not a pathological one.
    """
    n_runs = 100
    rows = np.array([
        _fly_one(s, [150.0], dt=0.25, extra_drain_kg_s=0.05)
        for s in range(n_runs)
    ])
    fuel_anees = rows[:, 0, 0].mean()
    full_anees = rows[:, 0, 1].mean()

    fuel_band = 1.96 * math.sqrt(2.0 / n_runs)
    full_band = 1.96 * math.sqrt(4.0 / n_runs)

    assert abs(fuel_anees - 1.0) < fuel_band, (
        f"the fuel channel now detects a {0.05} kg/s unmodelled sink "
        f"(ANEES {fuel_anees:.2f}); this test's premise no longer holds"
    )
    assert full_anees - 2.0 > full_band, (
        f"full-state ANEES {full_anees:.2f} did not detect a 0.05 kg/s "
        "unmodelled fuel sink -- the consistency test above has lost its teeth"
    )


@pytest.mark.slow
def test_consistency_survives_a_worse_gauge():
    """The filter must be honest against whatever gauge is bolted to it, not
    only against the reference one.

    This matters because the gauge's role changed in kind. It used to BE the
    belief, so its sigma was the platform's mass sigma outright and its rate
    only controlled staleness. Now it is a correction source, and rate and
    noise trade off: the same 20 kg reading arriving at 0.05 Hz instead of
    1 Hz leaves a belief three times worse. A filter calibrated only at the
    reference rate would be a trap in a repository whose whole point is that
    people swap components out.

    Twenty times slower and three times noisier, which is a different
    prediction-to-correction balance rather than a rescaling of the same one.
    """
    degraded = dataclasses.replace(GAUGE, fuel_rate_hz=0.05, fuel_sigma_kg=60.0)
    n_runs = 80
    rows = np.array([
        _fly_one(s, [150.0], dt=0.25, gauge_par=degraded) for s in range(n_runs)
    ])

    for dof, channel, label in ((1, 0, "fuel"), (2, 1, "whole state")):
        band = 1.96 * math.sqrt(2.0 * dof / n_runs)
        mean = rows[:, 0, channel].mean()
        assert abs(mean - dof) < band, (
            f"{label} ANEES {mean:.2f} against a 0.05 Hz / 60 kg gauge is "
            f"outside [{dof - band:.2f}, {dof + band:.2f}] -- the filter is "
            "only calibrated for the reference gauge"
        )


@pytest.mark.slow
@pytest.mark.performance
def test_the_filter_beats_the_raw_gauge():
    """If filtering did not improve on a single reading there would be no
    reason to carry a filter. Eight-fold here, which is also why the
    conservative-envelope idea only became worth having once the sigma
    stopped being the raw 20 kg."""
    checkpoints = [150.0]
    errors = []
    for seed in range(40):
        streams = np.random.SeedSequence(seed).spawn(2)
        truth_rng = np.random.default_rng(streams[0])
        gauge_rng = np.random.default_rng(streams[1])
        tsfc_error = float(truth_rng.normal(0.0, STANDARD.tsfc_sigma_fraction))
        fuel0 = STANDARD.initial_fuel_kg + float(
            truth_rng.normal(0.0, STANDARD.initial_fuel_sigma_kg)
        )
        nominal = reference_fighter()
        vehicle = PlanarPointMass(
            dataclasses.replace(
                nominal.theta, c_tsfc=BELIEVED_TSFC_KG_PER_N_S * (1.0 + tsfc_error)
            ),
            nominal.lam,
            nominal.eta,
        )
        gauge = FuelGauge(GAUGE, vehicle.lam.mass_dry_kg, gauge_rng)
        manager = VehicleManager(vehicle, STANDARD)
        state = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg + fuel0)
        command = VehicleCommand(60_000.0, 0.0)

        t = 0.0
        while t < checkpoints[0]:
            if gauge.due(t):
                manager.ingest(gauge.sample(t, state))
            manager.predict(t + 0.25, command.thrust_N, BELIEVED_TSFC_KG_PER_N_S)
            state = step_rk4(vehicle, state, command, 0.25)
            t += 0.25
        errors.append(
            manager.mass(t).fuel_mass_kg - (state.mass_kg - vehicle.lam.mass_dry_kg)
        )

    rms = float(np.sqrt(np.mean(np.square(errors))))
    assert rms < 0.5 * GAUGE.fuel_sigma_kg, (
        f"filtered rms {rms:.2f} kg is no better than half the raw gauge's "
        f"{GAUGE.fuel_sigma_kg:.0f} kg -- the filter is not earning its place"
    )


# --------------------------------------------------------------------------
# The promised envelope
# --------------------------------------------------------------------------

# Every field of PromisedEnvelope, and which way the margin must move it.
# "narrower" means a heavier belief must not report a better number.
#
# Listed here rather than checked ad hoc so that adding a field to the record
# without deciding its direction fails the test below instead of silently
# joining the promise. That is the failure this table exists for: the first
# version of capability_bound() returned the vehicle's whole Capability, and
# endurance_s rode along reporting 674 seconds MORE than the point estimate
# while wearing the name of a bound.
_MARGIN_DIRECTION = {
    "max_turn_rate_rad_s": "lower",
    "sustained_turn_rate_rad_s": "lower",
    "min_turn_radius_m": "higher",        # needing more room is worse
    "load_factor_available": "lower",
    "min_speed_mps": "higher",            # a higher floor is a narrower band
    "max_speed_mps": "lower",
}
_PROVENANCE = {"mass_kg", "mass_margin_sigma"}


def test_every_promised_channel_has_a_declared_direction():
    """No field joins the promise without someone deciding which way the
    margin should move it."""
    fields = {f.name for f in dataclasses.fields(PromisedEnvelope)}
    undeclared = fields - set(_MARGIN_DIRECTION) - _PROVENANCE
    assert not undeclared, (
        f"PromisedEnvelope fields with no declared margin direction: "
        f"{sorted(undeclared)}"
    )


def test_the_promise_is_never_better_than_the_estimate(vehicle, manager):
    """The property that makes a single signed mass margin correct.

    Swept across the speed range rather than checked at one point, because
    the binding limit changes with speed -- structural at high speed, lift at
    low -- and a channel can be conservative in one regime and not the other.
    """
    for v_mps in (100.0, 150.0, 200.0, 250.0, 300.0, 400.0, 500.0, 590.0):
        est = _estimate(v_mps=v_mps)
        point = manager.capability(est)
        promise = manager.capability_bound(est)

        # The point-estimate equivalents, named as the vehicle names them.
        equivalent = {
            "max_turn_rate_rad_s": point.omega_available_rad_s,
            "sustained_turn_rate_rad_s": point.omega_sustained_rad_s,
            "min_turn_radius_m": point.turn_radius_min_m,
            "load_factor_available": point.load_factor_available,
            "min_speed_mps": point.v_min_achievable_mps,
            "max_speed_mps": point.v_max_achievable_mps,
        }
        for name, direction in _MARGIN_DIRECTION.items():
            promised, estimated = getattr(promise, name), equivalent[name]
            if direction == "lower":
                assert promised <= estimated + 1e-9, (
                    f"{name} promised {promised} exceeds the estimate "
                    f"{estimated} at {v_mps} m/s"
                )
            else:
                assert promised >= estimated - 1e-9, (
                    f"{name} promised {promised} is below the estimate "
                    f"{estimated} at {v_mps} m/s"
                )


def test_the_promise_excludes_the_channels_the_margin_would_corrupt(manager):
    """The exclusions are load-bearing, not tidiness.

    Fuel and endurance are anti-conservative under added mass, because mass
    uncertainty here IS fuel uncertainty and a heavier aircraft is carrying
    more of it. This test asserts that they really do move the wrong way in
    the underlying model -- so if that ever stops being true the exclusion
    can be revisited deliberately, and if someone adds them to the promise
    the test above fails.
    """
    est = _estimate()
    point = manager.capability(est)
    margined = dataclasses.replace(
        manager.beliefs(), mass_kg=manager.capability_bound(est).mass_kg
    )
    heavier = manager.vehicle.capability(manager.vehicle.state_from(est, margined))

    # Anti-conservative: the margined mass reports MORE of both.
    assert heavier.fuel_mass_kg > point.fuel_mass_kg
    assert heavier.endurance_s > point.endurance_s

    # And neither is reachable from the promise.
    promised = {f.name for f in dataclasses.fields(PromisedEnvelope)}
    assert "fuel_mass_kg" not in promised
    assert "endurance_s" not in promised
    # Accelerations are excluded for a different reason -- not monotone in
    # mass at all -- so no single signed margin could be right for them.
    assert "accel_max_mps2" not in promised
    assert "accel_min_mps2" not in promised


def test_the_promise_carries_its_own_provenance(vehicle, manager):
    """A consumer must be able to see what the envelope was evaluated at
    rather than inferring it from how the platform was configured -- the
    numbers coincide with the point estimate once a filter has converged."""
    est = _estimate()
    promise = manager.capability_bound(est)
    sigma = manager.mass(0.0).mass_sigma_kg

    assert promise.mass_margin_sigma == STANDARD.capability_margin_sigma
    assert promise.mass_kg == pytest.approx(
        manager.mass_kg + STANDARD.capability_margin_sigma * sigma
    )
    assert promise.mass_kg > manager.mass_kg


def test_the_margin_bites_only_when_the_belief_is_poor(vehicle):
    """A margin that never changes anything is decoration, and one that
    always changes things is a tax. This one is scaled by the live
    uncertainty, so it does neither.

    Before the first gauge reading the sigma is the configured 200 kg and
    three of them is 600 kg, worth about four per cent of the turn rate.
    After the filter converges to a few kilograms it is worth almost nothing,
    and the platform stops promising less than it can do.
    """
    est = _estimate(v_mps=150.0)
    manager = VehicleManager(vehicle, STANDARD)

    def narrowing() -> float:
        point = manager.capability(est).omega_available_rad_s
        promised = manager.capability_bound(est).max_turn_rate_rad_s
        return 1.0 - promised / point

    before = narrowing()
    assert before > 0.02, "the margin does nothing even with a 200 kg sigma"

    for k in range(1, 31):
        t = float(k)
        manager.predict(t, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)
        manager.ingest(FuelMeasurement(t, 4000.0 - 1.5 * t, 20.0))

    after = narrowing()
    assert after < 0.005, "the margin still costs performance after convergence"
    assert after < before


def test_a_zero_margin_is_the_point_estimate(vehicle):
    """The margin is a configuration choice, not something baked in, so
    turning it off must recover the point estimate exactly."""
    est = _estimate(v_mps=150.0)
    manager = VehicleManager(vehicle, _params(capability_margin_sigma=0.0))

    assert manager.capability_bound(est).max_turn_rate_rad_s == pytest.approx(
        manager.capability(est).omega_available_rad_s
    )


def test_feedforward_and_enforcement_do_not_use_the_promise(vehicle):
    """Only the promised envelope carries the margin.

    Thrust computed for an aircraft heavier than the real one is not
    cautious, it accelerates; and clipping against the promise would report
    the estimator's doubt as though it were the airframe's limit, which would
    make a Saturation finding mean two different things. Both must therefore
    be evaluated at the believed mass.

    Run with a 200 kg sigma so the point estimate and the promise are far
    enough apart for the difference to be visible.
    """
    est = _estimate(v_mps=150.0)
    manager = VehicleManager(vehicle, STANDARD)          # sigma still 200 kg
    believed = manager.mass_kg

    # Feedforward: the thrust for straight flight must be the one the
    # believed mass needs, not the heavier promise's.
    straight = manager.capability(est, omega_rad_s=0.0)
    assert straight.thrust_required_N == pytest.approx(
        vehicle.thrust_required_N(150.0, believed, 0.0)
    )
    assert straight.thrust_required_N != pytest.approx(
        vehicle.thrust_required_N(150.0, believed + 600.0, 0.0)
    )

    # Enforcement: a turn rate the airframe can fly at the believed mass must
    # not be clipped merely because the platform is unsure of its mass.
    reachable = manager.capability(est).omega_available_rad_s
    just_inside = VehicleCommand(60_000.0, 0.999 * reachable)
    assert just_inside.omega_rad_s > manager.capability_bound(est).max_turn_rate_rad_s

    _, sat = manager.project_command(est, just_inside)
    assert not sat.omega_clipped, (
        "enforcement clipped against the promised envelope rather than the "
        "airframe -- a saturation finding would then mean estimator doubt"
    )


# --------------------------------------------------------------------------
# One manager, either model
# --------------------------------------------------------------------------

def _boosted():
    return VehicleManager(reference_boosted_fighter(), STANDARD)


def test_the_manager_serves_either_model_without_asking_which(vehicle):
    """The modularity claim, checked rather than asserted in a docstring.

    Identical calls against a five-state model and a six-state one. If this
    needed a branch, guidance and the planner would need one too, and
    swapping a boosted vehicle in would ripple to the top of the stack.
    """
    est = _estimate()
    for manager in (VehicleManager(vehicle, STANDARD), _boosted()):
        believed = manager.believed_state(est)
        assert believed.mass_kg == pytest.approx(manager.mass_kg)
        assert manager.capability(est).omega_available_rad_s > 0.0
        assert manager.capability_bound(est).max_turn_rate_rad_s > 0.0
        cmd, sat = manager.project_command(est, VehicleCommand(60_000.0, 0.0))
        assert cmd.thrust_N > 0.0 and not sat.any
        manager.predict(1.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S, est)
        assert manager.mass(1.0).mass_kg < 16_000.0


def test_the_model_builds_its_own_state_shape(vehicle):
    """Five elements against six and a mode. The manager names neither."""
    est = _estimate()
    assert len(VehicleManager(vehicle, STANDARD).believed_state(est).to_array()) == 5
    assert len(_boosted().believed_state(est).to_array()) == 6
    assert _boosted().believed_state(est).mode is Mode.NOMINAL


def test_the_thermal_belief_is_dead_reckoned_from_the_commanded_mode():
    """The mass pattern, applied to the other consumable: something is spent
    at a rate the platform can predict and nothing measures it.

    The LAW stays on the model -- the manager integrates thermal_rate() and
    does not recompute sigma_q, which would be reimplementing a rule the
    vehicle owns.
    """
    est = _estimate()
    manager = _boosted()
    assert manager.beliefs().thermal == 0.0

    manager.select_mode(0.0, Mode.BOOST)
    for k in range(1, 151):                      # 15 s of boost at dt = 0.1
        manager.predict(0.1 * k, 60_000.0, BELIEVED_TSFC_KG_PER_N_S, est)
    hot = manager.beliefs().thermal
    assert hot == pytest.approx(15.0 / 30.0, abs=0.02)   # tau_h = 30 s

    manager.select_mode(15.0, Mode.NOMINAL)
    for k in range(151, 301):
        manager.predict(0.1 * k, 60_000.0, BELIEVED_TSFC_KG_PER_N_S, est)
    assert manager.beliefs().thermal < hot, "the thermal state did not recover"


def test_a_single_mode_model_never_grows_a_thermal_belief(vehicle):
    """The baseline has no thermal law, so there is nothing to integrate and
    the belief stays put. No branch in the manager says so -- it asks the
    model for a rate and the baseline has none."""
    est = _estimate()
    manager = VehicleManager(vehicle, STANDARD)
    for k in range(1, 101):
        manager.predict(0.1 * k, 60_000.0, BELIEVED_TSFC_KG_PER_N_S, est)
    assert manager.beliefs().thermal == 0.0


def test_predicting_without_a_state_is_an_error_for_a_thermal_model():
    """A silent no-op would be the worse failure: the thermal belief would
    never move, boost would look free, and capability would over-report how
    long it could be held with nothing failing. The argument stays optional
    so a single-mode platform need not pass it."""
    with pytest.raises(ValueError, match="thermal"):
        _boosted().predict(1.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S)

    # The baseline has no thermal state, so omitting it is fine.
    VehicleManager(reference_fighter(), STANDARD).predict(
        1.0, 60_000.0, BELIEVED_TSFC_KG_PER_N_S
    )


def test_the_manager_tracks_time_since_the_last_mode_change():
    """What the switching set needs and cannot hold itself. Re-selecting the
    same mode is not a transition."""
    manager = _boosted()
    manager.select_mode(10.0, Mode.BOOST)
    assert manager.since_mode_change_s(13.0) == pytest.approx(3.0)

    manager.select_mode(13.0, Mode.BOOST)            # same mode, no transition
    assert manager.since_mode_change_s(13.0) == pytest.approx(3.0)

    manager.select_mode(13.0, Mode.NOMINAL)
    assert manager.since_mode_change_s(13.0) == pytest.approx(0.0)


def test_the_believed_mode_reaches_the_model(vehicle):
    """The manager carries the mode without interpreting it, and the model
    interprets it. Available thrust is the visible consequence."""
    est = _estimate()
    manager = _boosted()

    nominal = manager.capability(est).thrust_available_N
    manager.select_mode(0.0, Mode.BOOST)
    boosted = manager.capability(est).thrust_available_N

    assert boosted > nominal


# --------------------------------------------------------------------------
# Sole consumer of the vehicle model
# --------------------------------------------------------------------------

def test_only_the_vehicle_manager_binds_the_vehicle_model():
    """ADR 0015's rule, in the form that can actually be checked.

    "Only the vehicle manager consumes vehicle capability" is not decidable
    from a call site without type inference, but holding a PlanarPointMass at all
    is, and it is the same rule: a component that cannot reach the model
    cannot query it, and a component that can will eventually need a mass to
    query it with. That is how the mass parameter got into guidance in the
    first place.

    Three exemptions, each for a different reason:

      ose/equipment/**       the equipment layer owns the vehicle. Imu holds one
                            for drag_N and is a peer, not a consumer above it.
      ose/integration.py    the integrator steps the model rather than asking
                            it what it can do. It is the simulation core's job
                            sitting outside the components, per ADR 0004.
      vehicle_manager.py    the one consumer this rule exists to name.

    Anything else importing PlanarPointMass is a component reaching past the
    manager for a mass-dependent answer.
    """
    root = Path(__file__).resolve().parents[2] / "src" / "ose"
    exempt = {
        root / "integration.py",
        root / "subsystem" / "vehicle_manager.py",
    }

    models = vehicle_model_names()
    holders = []
    for path in sorted(root.rglob("*.py")):
        if path in exempt or "equipment" in path.relative_to(root).parts:
            continue
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and is_truth_package(node.module):
                bound = {a.name for a in node.names} & models
                if bound:
                    holders.append(f"{path.relative_to(root)} ({sorted(bound)})")

    assert not holders, (
        "these bind the vehicle model directly instead of going through the "
        f"vehicle manager: {holders}"
    )

    # Not vacuous: the manager really does bind a model, so the walk is
    # looking at the right imports and would see another.
    manager_src = (root / "subsystem" / "vehicle_manager.py").read_text()
    assert any(m in manager_src for m in models)


# --------------------------------------------------------------------------
# Vehicle questions, answered at the believed mass
# --------------------------------------------------------------------------

def test_capability_is_evaluated_at_the_believed_mass(vehicle):
    """The whole point of the component. A belief that does not move the
    reported envelope is not being bound to anything, and consumers are back
    to supplying a mass themselves.

    Stall speed is the channel to check: it scales with sqrt(mass), so a
    heavier belief must raise it.
    """
    est = _estimate()
    light = _believing(vehicle, 1000.0).capability(est)     # 13 000 kg
    heavy = _believing(vehicle, 6000.0).capability(est)     # 18 000 kg

    assert heavy.v_stall_mps > light.v_stall_mps
    # And it really is the vehicle's own rule, evaluated at that mass, rather
    # than some approximation of it living here.
    assert heavy.v_stall_mps == pytest.approx(vehicle.v_stall_mps(18000.0))
    assert light.v_stall_mps == pytest.approx(vehicle.v_stall_mps(13000.0))


def test_capability_forwards_the_turn_rate(vehicle, manager):
    """Guidance feeds thrust forward at the rate it will actually fly, so the
    parametrised query has to survive the forwarding. Without it the manager
    could only report thrust for straight flight and the feedforward would be
    wrong in exactly the turns where it matters."""
    est = _estimate()
    straight = manager.capability(est, omega_rad_s=0.0)
    turning = manager.capability(est, omega_rad_s=math.radians(15.0))

    assert turning.thrust_required_N > straight.thrust_required_N
    assert turning.thrust_required_N == pytest.approx(
        vehicle.thrust_required_N(250.0, 16000.0, math.radians(15.0))
    )


def test_project_command_enforces_at_the_believed_mass(vehicle):
    """Enforcement must be evaluated at the mass the manager believes.

    The turn-rate limit is the channel to use. It is lift-limited at low
    speed and so moves strongly with mass -- 18.1 deg/s at 13 t against
    12.8 deg/s at 18 t, both at 150 m/s -- whereas thrust_available_N depends
    on mass only through a burning/not-burning gate and cannot show the
    difference at all. An earlier version of this test used thrust and passed
    against an implementation that enforced at a hardcoded mass.

    One command, two beliefs, opposite verdicts.
    """
    est = _estimate(v_mps=150.0)
    turn = VehicleCommand(thrust_N=60_000.0, omega_rad_s=math.radians(16.0))

    light_cmd, light_sat = _believing(vehicle, 1000.0).project_command(est, turn)
    assert not light_sat.omega_clipped
    assert light_cmd.omega_rad_s == pytest.approx(turn.omega_rad_s)

    heavy_cmd, heavy_sat = _believing(vehicle, 6000.0).project_command(est, turn)
    assert heavy_sat.omega_clipped
    assert heavy_cmd.omega_rad_s == pytest.approx(
        vehicle.omega_max_rad_s(150.0, 18000.0)
    )
    # The receipt still carries what was asked for (ADR 0006).
    assert heavy_sat.requested.omega_rad_s == pytest.approx(turn.omega_rad_s)


def test_believed_state_carries_the_believed_mass(vehicle):
    est = _estimate(v_mps=310.0)
    believed = _believing(vehicle, 2500.0).believed_state(est)

    assert believed.mass_kg == pytest.approx(14500.0)
    assert believed.v_mps == pytest.approx(310.0)
    assert believed.psi_rad == pytest.approx(est.psi_rad)
