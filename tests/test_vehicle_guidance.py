"""Tests for vehicle guidance.

test_reports_saturation_when_setpoint_exceeds_envelope is the important one:
ADR 0006 exists so that a control law commanding outside the vehicle's
declared sets produces a visible finding rather than a silent clip, and
nothing exercised that path until this component existed.

test_guidance_cannot_see_truth checks the same truth-boundary property
established for the two estimators (ADR 0009): guidance only ever touches
OwnStateEstimate, never VehicleState or Disturbance directly.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from ose import interfaces
from ose.integration import step_rk4
from ose.interfaces import (
    HeadingSpeedSetpoint,
    OwnStateEstimate,
    TurnRateSpeedSetpoint,
)
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleState
from ose.subsystem.reference_configs.reference_vehicle_guidance import STANDARD
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_guidance import VehicleGuidance
from ose.subsystem.vehicle_manager import VehicleManager


def _perfect_estimate(t_s: float, state: VehicleState) -> OwnStateEstimate:
    """An OwnStateEstimate equal to truth -- stands in for a perfect
    navigation solution so these tests isolate guidance's own behaviour."""
    v_vec = state.v_mps * np.array([math.cos(state.psi_rad), math.sin(state.psi_rad)])
    return OwnStateEstimate(
        t_s=t_s,
        p_x_m=state.p_x_m,
        p_y_m=state.p_y_m,
        psi_rad=state.psi_rad,
        v_air_mps=state.v_mps,
        ground_velocity_mps=v_vec,
        wind_estimate_mps=np.zeros(2),
        covariance=np.zeros((4, 4)),
    )


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def manager(vehicle):
    # 12 000 kg dry plus 4 000 kg of fuel: the 16 000 kg platform every
    # state constructed below is built at, so these tests exercise the
    # same numbers they did when mass was a call argument.
    return VehicleManager(vehicle, MANAGER_STANDARD)


@pytest.fixture
def guidance(manager):
    return VehicleGuidance(manager, STANDARD)


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_guidance_cannot_see_truth():
    path = (
        Path(__file__).resolve().parents[1]
        / "src" / "ose" / "subsystem" / "vehicle_guidance.py"
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


def test_satisfies_the_protocol(guidance):
    assert isinstance(guidance, interfaces.VehicleGuidance)


def test_commanding_unknown_setpoint_type_raises_type_error(guidance):
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    with pytest.raises(TypeError):
        guidance.command(0.0, object(), _perfect_estimate(0.0, state))


# --------------------------------------------------------------------------
# The control law
# --------------------------------------------------------------------------

def test_zero_error_commands_steady_level_flight(vehicle, guidance):
    """At the setpoint already, guidance should ask for close to zero turn
    rate and close to the thrust needed to hold speed -- no unnecessary
    control action."""
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=state.psi_rad, v_cmd_mps=state.v_mps)

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state))

    assert not sat.any
    assert abs(cmd.omega_rad_s) < 1e-9
    assert cmd.thrust_N == pytest.approx(
        vehicle.thrust_required_N(state.v_mps, state.mass_kg, 0.0), rel=1e-9
    )


def test_holds_heading_and_speed_setpoint(vehicle, guidance):
    """Closed loop: start away from the setpoint, drive truth forward with
    guidance's own commands, and check it converges."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=math.radians(45.0), v_cmd_mps=260.0)

    dt = 0.05
    t = 0.0
    while t < 60.0:
        cmd, _ = guidance.command(t, setpoint, _perfect_estimate(t, state))
        state = step_rk4(vehicle, state, cmd, dt)
        t += dt

    heading_error = math.remainder(state.psi_rad - setpoint.psi_cmd_rad, 2.0 * math.pi)
    assert abs(math.degrees(heading_error)) < 1.0
    assert abs(state.v_mps - setpoint.v_cmd_mps) < 1.0


def test_feedforward_matches_the_turn_actually_commanded(vehicle, guidance):
    """Guidance asks capability what the vehicle can do and feedforwards
    thrust for THAT turn, not for the one the error term wished for.

    The regression this pins: induced drag scales with load factor
    squared, so feedforwarding an unachievable turn rate demands an absurd
    thrust. Evaluating drag at a raw 54 deg/s request asked for 1330 kN
    from a 130 kN engine. The flight condition here is chosen so the
    difference is externally visible -- the turn rate saturates, but the
    thrust for the achievable turn is inside the envelope, so a correct
    feedforward does not saturate thrust and an incorrect one does.
    """
    state = VehicleState(0.0, 0.0, 0.0, 150.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=math.pi, v_cmd_mps=state.v_mps)

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state))

    assert sat.omega_clipped
    assert not sat.thrust_clipped

    # Zero speed error, so the whole command is feedforward: it must equal
    # the thrust required to sustain the turn that was actually commanded.
    assert cmd.thrust_N == pytest.approx(
        vehicle.thrust_required_N(state.v_mps, state.mass_kg, cmd.omega_rad_s), rel=1e-9
    )


# --------------------------------------------------------------------------
# Feedforward on a moving heading setpoint
# --------------------------------------------------------------------------

def _fly(vehicle, guidance, state, setpoint_at, duration_s, dt=0.05):
    t = 0.0
    while t < duration_s:
        cmd, _ = guidance.command(
            t, setpoint_at(t), _perfect_estimate(t, state)
        )
        state = step_rk4(vehicle, state, cmd, dt)
        t += dt
    return state, t


def test_ramping_setpoint_without_feedforward_lags_by_rate_over_gain(vehicle, guidance):
    """The defect the feedforward field exists to remove, pinned so nobody
    removes the field believing it decorative.

    A proportional law chasing a ramp settles where the correction alone
    supplies the whole turn rate, i.e. at an error of rate/gain. Omit the
    rate and the vehicle trails the commanded heading permanently.
    """
    rate = math.radians(10.0)
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    flown, t_end = _fly(
        vehicle, guidance, state,
        lambda t: HeadingSpeedSetpoint(rate * t, 250.0),   # no rate declared
        40.0,
    )
    # Wrapped: after 40 s the command has passed 360 degrees while the
    # vehicle's heading wraps to +-180, so the raw difference is a full turn out.
    lag = math.degrees(math.remainder(rate * t_end - flown.psi_rad, 2.0 * math.pi))
    predicted = math.degrees(rate / STANDARD.heading_gain_per_s)
    assert lag == pytest.approx(predicted, rel=0.1)


def test_declaring_the_rate_removes_the_lag(vehicle, guidance):
    """Same sweep, same gains, with the setpoint's own rate declared: the
    heading error settles at zero instead of at rate/gain."""
    rate = math.radians(10.0)
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    flown, t_end = _fly(
        vehicle, guidance, state,
        lambda t: HeadingSpeedSetpoint(rate * t, 250.0, psi_rate_cmd_rad_s=rate),
        40.0,
    )
    lag = abs(math.degrees(math.remainder(rate * t_end - flown.psi_rad, 2.0 * math.pi)))
    assert lag < 1.0


# --------------------------------------------------------------------------
# Turn-rate setpoints
# --------------------------------------------------------------------------

def test_turn_rate_setpoint_is_commanded_directly(vehicle, guidance):
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    rate = math.radians(8.0)
    cmd, sat = guidance.command(
        0.0, TurnRateSpeedSetpoint(rate, 250.0), _perfect_estimate(0.0, state),
    )
    assert not sat.any
    assert cmd.omega_rad_s == pytest.approx(rate)


def test_unreachable_turn_rate_saturates_and_stays_saturated(
    vehicle, guidance, manager
):
    """The reason this setpoint type exists. A heading command asking for
    more than the airframe can give laps the vehicle, the error wraps
    through 180 and flips sign, and guidance reverses the turn. With no
    heading to chase there is no error to wrap, so an impossible rate simply
    pins against omega_available for as long as it is commanded.

    The limit is evaluated at the mass the platform BELIEVES, not the true
    one. Over these 60 seconds the vehicle burns several hundred kilograms
    while nothing feeds the manager a fuel measurement, so the belief stays
    at its initial 16 000 kg and the two diverge by about 0.4 t. Asserting
    against state.mass_kg here would be asserting that guidance can read
    truth, which is exactly what ADR 0015 removed. It is also the staleness
    the sum-only manager is documented as having.
    """
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    absurd = TurnRateSpeedSetpoint(math.radians(200.0), 250.0)

    signs = []
    t = 0.0
    while t < 60.0:
        cmd, sat = guidance.command(
            t, absurd, _perfect_estimate(t, state)
        )
        assert sat.omega_clipped
        assert cmd.omega_rad_s == pytest.approx(
            vehicle.omega_max_rad_s(state.v_mps, manager.mass_kg)
        )
        signs.append(math.copysign(1.0, cmd.omega_rad_s))
        state = step_rk4(vehicle, state, cmd, 0.05)
        t += 0.05

    assert len(set(signs)) == 1, "the turn reversed, which is the wrap bug"


def test_turn_rate_setpoint_still_holds_speed(vehicle, guidance):
    """It replaces the heading loop, not the speed loop."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    flown, _ = _fly(
        vehicle, guidance, state,
        lambda t: TurnRateSpeedSetpoint(math.radians(5.0), 280.0),
        90.0,
    )
    assert abs(flown.v_mps - 280.0) < 2.0


# --------------------------------------------------------------------------
# Capability: composed from the vehicle and from navigation
# --------------------------------------------------------------------------

def test_capability_reachability_comes_from_the_vehicle(vehicle, guidance):
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    envelope = vehicle.capability(state)
    cap = guidance.capability(_perfect_estimate(0.0, state))

    assert cap.max_turn_rate_rad_s == envelope.omega_available_rad_s
    assert cap.max_speed_mps == envelope.v_max_achievable_mps
    assert cap.min_speed_mps == envelope.v_min_achievable_mps


def test_capability_hold_accuracy_comes_from_navigation(vehicle, guidance):
    """The navigation half is read from the covariance travelling with the
    estimate, so a degraded estimate degrades the claim."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    est = _perfect_estimate(0.0, state)
    est.covariance = np.diag([100.0, 100.0, math.radians(2.0) ** 2, 1.5**2])

    cap = guidance.capability(est)
    assert cap.heading_hold_sigma_rad == pytest.approx(math.radians(2.0))
    assert cap.speed_hold_sigma_mps == pytest.approx(1.5)


def test_capability_degrades_when_navigation_does(vehicle, guidance):
    """A GNSS outage widens the covariance; the guidance claim must widen
    with it rather than staying at its fair-weather value."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)

    good = _perfect_estimate(0.0, state)
    good.covariance = np.diag([1.0, 1.0, math.radians(0.2) ** 2, 0.3**2])
    degraded = _perfect_estimate(0.0, state)
    degraded.covariance = np.diag([400.0, 400.0, math.radians(3.0) ** 2, 2.0**2])

    assert (
        guidance.capability(degraded).heading_hold_sigma_rad
        > guidance.capability(good).heading_hold_sigma_rad
    )
    # The vehicle half is unaffected -- the airframe does not care that the
    # navigation solution got worse.
    assert (
        guidance.capability(degraded).max_turn_rate_rad_s
        == guidance.capability(good).max_turn_rate_rad_s
    )


def test_admits_rejects_unholdable_speeds(vehicle, guidance):
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    cap = guidance.capability(_perfect_estimate(0.0, state))

    assert cap.admits(HeadingSpeedSetpoint(0.0, 250.0))
    assert not cap.admits(HeadingSpeedSetpoint(0.0, cap.min_speed_mps - 10.0))
    assert not cap.admits(HeadingSpeedSetpoint(0.0, cap.max_speed_mps + 10.0))
    # Any heading is reachable given time; only speed can be unholdable.
    assert cap.admits(HeadingSpeedSetpoint(math.pi, 250.0))


@pytest.mark.parametrize("nav_heading_error_deg", [1.0, 3.0, 5.0])
def test_claimed_hold_accuracy_is_honest(vehicle, guidance, nav_heading_error_deg):
    """The honesty test for a composed capability, and the reason the claim
    is worth making at all.

    Guidance drives the BELIEVED heading onto the setpoint, so whatever
    navigation is wrong by, the true heading is wrong by too. Fly it closed
    loop against an estimate carrying a known heading error and confirm the
    true steady-state error is what the capability claimed -- not better,
    which would mean the claim was pessimistic and a planner was leaving
    performance unused, and not worse, which would mean a planner trusting
    it flies tighter than the loop can actually hold.
    """
    error = math.radians(nav_heading_error_deg)
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=math.radians(45.0), v_cmd_mps=250.0)

    claimed = None
    t, dt = 0.0, 0.05
    while t < 90.0:
        est = _perfect_estimate(t, state)
        est.psi_rad = state.psi_rad + error       # navigation is off by `error`
        est.covariance = np.diag([0.0, 0.0, error**2, 0.0])
        if claimed is None:
            claimed = guidance.capability(est).heading_hold_sigma_rad
        cmd, _ = guidance.command(t, setpoint, est)
        state = step_rk4(vehicle, state, cmd, dt)
        t += dt

    true_error = abs(
        math.remainder(state.psi_rad - setpoint.psi_cmd_rad, 2.0 * math.pi)
    )
    assert claimed == pytest.approx(error)
    assert true_error == pytest.approx(claimed, rel=0.05)


def test_reports_saturation_when_setpoint_exceeds_envelope(vehicle, guidance):
    """A heading flip commands far more turn rate than the vehicle can
    deliver. The applied command must be clipped, and that clipping must be
    reported -- not silently absorbed, per ADR 0006."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=math.pi, v_cmd_mps=state.v_mps)

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state))

    assert sat.omega_clipped
    assert sat.any
    assert vehicle.admissible(state, cmd)
