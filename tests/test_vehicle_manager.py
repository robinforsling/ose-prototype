"""Tests for the vehicle manager.

The load-bearing one is test_fuel_estimate_is_consistent_through_the_run.
Everything else here checks that a number is correct; that one checks that the
component's *stated uncertainty* is correct, which is the property the rest of
the system relies on and the only one that catches an overconfident filter.
It is set up as an ensemble because consistency is not a property of a single
run: the true burn coefficient and the true initial fuel are drawn from the
priors the filter assumes, which is what makes the question well posed.

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

from ose.integration import step_rk4
from ose.interfaces import FuelMeasurement, MassEstimate, OwnStateEstimate
from ose.resource.fuel_gauge import FuelGauge
from ose.resource.reference_configs.reference_fuel_gauge import STANDARD as GAUGE
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import Vehicle2D, VehicleCommand, VehicleState
from ose.subsystem.reference_configs.reference_vehicle_manager import STANDARD
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

def test_manager_cannot_see_truth():
    path = (
        Path(__file__).resolve().parents[1]
        / "src" / "ose" / "subsystem" / "vehicle_manager.py"
    )
    tree = ast.parse(path.read_text())

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "ose.resource.vehicle":
            names = {alias.name for alias in node.names}
            leaked = names & {"Disturbance", "VehicleState"}
            assert not leaked, f"imports truth-carrying types: {leaked}"

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            params = [a.arg for a in node.args.args + node.args.kwonlyargs]
            leaked = [p for p in params if p.startswith("true_")]
            assert not leaked, f"public method {node.name} takes truth: {leaked}"


def test_the_filter_does_not_read_the_true_burn_coefficient():
    """Predicting with the coefficient the vehicle burns at would make the
    prediction exact by construction: the filter would look excellent for a
    reason that never holds on a real platform, and every consistency test
    here would be vacuous. The believed coefficient is the manager's own
    parameter, so the module must never reach for theta.c_tsfc."""
    path = (
        Path(__file__).resolve().parents[1]
        / "src" / "ose" / "subsystem" / "vehicle_manager.py"
    )
    tree = ast.parse(path.read_text())

    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and node.attr == "c_tsfc":
            raise AssertionError(
                "vehicle_manager reads the vehicle's true burn coefficient; "
                "it must predict with its own believed one"
            )


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
    manager.predict(10.0, 60_000.0)
    after = manager.mass(10.0).fuel_mass_kg

    expected = STANDARD.tsfc_kg_per_N_s * 60_000.0 * 10.0      # 15 kg
    assert before - after == pytest.approx(expected)


def test_prediction_grows_the_uncertainty(manager):
    """A belief propagated without correction must get less certain, or the
    covariance is not describing anything."""
    before = manager.mass(0.0).mass_sigma_kg
    manager.predict(30.0, 60_000.0)
    assert manager.mass(30.0).mass_sigma_kg > before


def test_process_noise_grows_the_belief_even_with_no_burn(manager):
    """Isolates Qc. At zero thrust the state-transition coupling vanishes --
    F is all zeros, so Phi is the identity -- and the only thing left that can
    grow the covariance is the process noise. Without this the removal of Qc
    goes unnoticed, because at any non-zero thrust the tsfc coupling grows
    P[fuel, fuel] on its own and the sawtooth test still passes.
    """
    before = manager.mass(0.0).covariance[0, 0]
    manager.predict(10.0, 0.0)
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
    manager.predict(20.0, 130_000.0)
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
        manager.predict(t, 60_000.0)
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
    manager.predict(600.0, 130_000.0)

    assert manager.mass(600.0).fuel_mass_kg == 0.0
    assert manager.mass_kg == pytest.approx(vehicle.lam.mass_dry_kg)


def test_prediction_backwards_is_a_no_op(manager):
    manager.predict(10.0, 60_000.0)
    fuel = manager.mass(10.0).fuel_mass_kg
    manager.predict(5.0, 60_000.0)
    assert manager.mass(10.0).fuel_mass_kg == pytest.approx(fuel)


# --------------------------------------------------------------------------
# Honesty of the stated uncertainty
# --------------------------------------------------------------------------

def _fly_one(seed: int, checkpoints, dt: float, thrust_N: float = 60_000.0,
             gauge_par=GAUGE):
    """One ensemble member. Returns NEES on the fuel channel at each
    checkpoint.

    Truth is drawn from the priors the filter declares -- the burn
    coefficient from tsfc_sigma_fraction, the initial fuel from
    initial_fuel_sigma_kg. That is what makes consistency a well-posed
    question rather than a statement about one arbitrary run.
    """
    streams = np.random.SeedSequence(seed).spawn(2)
    truth_rng = np.random.default_rng(streams[0])
    gauge_rng = np.random.default_rng(streams[1])

    tsfc_error = float(truth_rng.normal(0.0, STANDARD.tsfc_sigma_fraction))
    fuel0 = STANDARD.initial_fuel_kg + float(
        truth_rng.normal(0.0, STANDARD.initial_fuel_sigma_kg)
    )

    nominal = reference_fighter()
    vehicle = Vehicle2D(
        dataclasses.replace(
            nominal.theta, c_tsfc=STANDARD.tsfc_kg_per_N_s * (1.0 + tsfc_error)
        ),
        nominal.lam,
        nominal.eta,
    )
    gauge = FuelGauge(gauge_par, vehicle.lam.mass_dry_kg, gauge_rng)
    manager = VehicleManager(vehicle, STANDARD)

    state = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg + fuel0)
    command = VehicleCommand(thrust_N, 0.0)

    out, k, t = [], 0, 0.0
    while k < len(checkpoints):
        if gauge.due(t):
            manager.ingest(gauge.sample(t, state))
        manager.predict(t + dt, command.thrust_N)
        state = step_rk4(vehicle, state, command, dt)
        t += dt
        if t >= checkpoints[k] - 1e-9:
            est = manager.mass(t)
            error = est.fuel_mass_kg - (state.mass_kg - vehicle.lam.mass_dry_kg)
            out.append(error**2 / est.covariance[0, 0])
            k += 1
    return out


def test_fuel_estimate_is_consistent_through_the_run():
    """The honesty test. Ensemble-average NEES must sit near one at every
    checkpoint, not only at the end.

    Below one the filter is overconservative and a consumer steering on the
    sigma gives away performance. Above one it is overconfident, which is the
    failure that silently corrupts everything downstream -- the INS/GNSS bug
    described in CLAUDE.md was exactly this, and finished thirty times
    overconfident while every plot looked correct. The bound is therefore
    two-sided, and checked through the run because a filter can be calibrated
    at the end while being wrong in the middle.

    One degree of freedom, so each sample is chi-square with mean 1 and
    variance 2; the 95 per cent band on the mean of N is 1 +- 1.96*sqrt(2/N).
    """
    n_runs = 120
    checkpoints = [10.0, 30.0, 60.0, 100.0, 150.0]
    band = 1.96 * math.sqrt(2.0 / n_runs)

    rows = np.array([_fly_one(s, checkpoints, dt=0.25) for s in range(n_runs)])

    for t_s, column in zip(checkpoints, rows.T):
        assert abs(column.mean() - 1.0) < band, (
            f"ANEES {column.mean():.3f} at t={t_s} s is outside "
            f"[{1 - band:.2f}, {1 + band:.2f}] -- the filter's stated "
            f"uncertainty is {'over' if column.mean() > 1 else 'under'}confident"
        )


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
    band = 1.96 * math.sqrt(2.0 / n_runs)

    rows = np.array([
        _fly_one(s, [150.0], dt=0.25, gauge_par=degraded) for s in range(n_runs)
    ])
    anees = rows[:, 0].mean()

    assert abs(anees - 1.0) < band, (
        f"ANEES {anees:.3f} against a 0.05 Hz / 60 kg gauge is outside "
        f"[{1 - band:.2f}, {1 + band:.2f}] -- the filter is only calibrated "
        "for the reference gauge"
    )


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
        vehicle = Vehicle2D(
            dataclasses.replace(
                nominal.theta, c_tsfc=STANDARD.tsfc_kg_per_N_s * (1.0 + tsfc_error)
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
            manager.predict(t + 0.25, command.thrust_N)
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

def test_the_bound_is_never_wider_than_the_estimate(vehicle, manager):
    """The property that makes a single signed margin correct.

    capability_bound() adds mass rather than applying a per-channel rule,
    and that is only sound because heavier is uniformly worse: a heavier
    aircraft turns no faster, stalls no slower, and the airframe speed limit
    does not move with mass at all. If any channel were anti-conservative in
    mass the margin would widen the claim somewhere while narrowing it
    elsewhere, and the whole approach would be wrong.

    Swept across the envelope rather than checked at one point, because the
    binding limit changes with speed -- structural at high speed, lift at low
    -- and the property has to hold in both regimes.
    """
    for v_mps in (100.0, 150.0, 200.0, 250.0, 300.0, 400.0, 500.0, 590.0):
        est = _estimate(v_mps=v_mps)
        point = manager.capability(est)
        bound = manager.capability_bound(est)

        assert bound.omega_available_rad_s <= point.omega_available_rad_s + 1e-12
        assert bound.v_min_achievable_mps >= point.v_min_achievable_mps - 1e-12
        assert bound.v_max_achievable_mps <= point.v_max_achievable_mps + 1e-12


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
        bound = manager.capability_bound(est).omega_available_rad_s
        return 1.0 - bound / point

    before = narrowing()
    assert before > 0.02, "the margin does nothing even with a 200 kg sigma"

    for k in range(1, 31):
        t = float(k)
        manager.predict(t, 60_000.0)
        manager.ingest(FuelMeasurement(t, 4000.0 - 1.5 * t, 20.0))

    after = narrowing()
    assert after < 0.005, "the margin still costs performance after convergence"
    assert after < before


def test_a_zero_margin_is_the_point_estimate(vehicle):
    """The margin is a configuration choice, not something baked in, so
    turning it off must recover the point estimate exactly."""
    est = _estimate(v_mps=150.0)
    manager = VehicleManager(vehicle, _params(capability_margin_sigma=0.0))

    assert manager.capability_bound(est).omega_available_rad_s == pytest.approx(
        manager.capability(est).omega_available_rad_s
    )


def test_feedforward_and_enforcement_do_not_use_the_bound(vehicle):
    """Only the promised envelope carries the margin.

    Thrust computed for an aircraft heavier than the real one is not
    cautious, it accelerates; and clipping against the bound would report the
    estimator's doubt as though it were the airframe's limit, which would
    make a Saturation finding mean two different things. Both must therefore
    be evaluated at the believed mass.

    Run with a 200 kg sigma so the point estimate and the bound are far
    enough apart for the difference to be visible.
    """
    est = _estimate(v_mps=150.0)
    manager = VehicleManager(vehicle, STANDARD)          # sigma still 200 kg
    believed = manager.mass_kg

    # Feedforward: the thrust for straight flight must be the one the
    # believed mass needs, not the heavier bound's.
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
    assert just_inside.omega_rad_s > manager.capability_bound(est).omega_available_rad_s

    _, sat = manager.project_command(est, just_inside)
    assert not sat.omega_clipped, (
        "enforcement clipped against the promised envelope rather than the "
        "airframe -- a saturation finding would then mean estimator doubt"
    )


# --------------------------------------------------------------------------
# Sole consumer of the vehicle model
# --------------------------------------------------------------------------

def test_only_the_vehicle_manager_binds_the_vehicle_model():
    """ADR 0015's rule, in the form that can actually be checked.

    "Only the vehicle manager consumes vehicle capability" is not decidable
    from a call site without type inference, but holding a Vehicle2D at all
    is, and it is the same rule: a component that cannot reach the model
    cannot query it, and a component that can will eventually need a mass to
    query it with. That is how the mass parameter got into guidance in the
    first place.

    Three exemptions, each for a different reason:

      ose/resource/**       the resource layer owns the vehicle. Imu holds one
                            for drag_N and is a peer, not a consumer above it.
      ose/integration.py    the integrator steps the model rather than asking
                            it what it can do. It is the simulation core's job
                            sitting outside the components, per ADR 0004.
      vehicle_manager.py    the one consumer this rule exists to name.

    Anything else importing Vehicle2D is a component reaching past the
    manager for a mass-dependent answer.
    """
    root = Path(__file__).resolve().parents[1] / "src" / "ose"
    exempt = {
        root / "integration.py",
        root / "subsystem" / "vehicle_manager.py",
    }

    holders = []
    for path in sorted(root.rglob("*.py")):
        if path in exempt or "resource" in path.relative_to(root).parts:
            continue
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.ImportFrom)
                and node.module == "ose.resource.vehicle"
                and any(a.name == "Vehicle2D" for a in node.names)
            ):
                holders.append(str(path.relative_to(root)))

    assert not holders, (
        "these bind the vehicle model directly instead of going through the "
        f"vehicle manager: {holders}"
    )

    # Not vacuous: the manager really does hold one, so the walk is looking
    # at the right import and would see another.
    manager_src = (root / "subsystem" / "vehicle_manager.py").read_text()
    assert "Vehicle2D" in manager_src


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
