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
from ose.interfaces import HeadingSpeedSetpoint, OwnStateEstimate
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleState
from ose.subsystem.reference_configs.reference_vehicle_guidance import STANDARD
from ose.subsystem.vehicle_guidance import VehicleGuidance


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
def guidance(vehicle):
    return VehicleGuidance(vehicle, STANDARD)


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
        guidance.command(0.0, object(), _perfect_estimate(0.0, state), state.mass_kg)


# --------------------------------------------------------------------------
# The control law
# --------------------------------------------------------------------------

def test_zero_error_commands_steady_level_flight(vehicle, guidance):
    """At the setpoint already, guidance should ask for close to zero turn
    rate and close to the thrust needed to hold speed -- no unnecessary
    control action."""
    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=state.psi_rad, v_cmd_mps=state.v_mps)

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state), state.mass_kg)

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
        cmd, _ = guidance.command(t, setpoint, _perfect_estimate(t, state), state.mass_kg)
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

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state), state.mass_kg)

    assert sat.omega_clipped
    assert not sat.thrust_clipped

    # Zero speed error, so the whole command is feedforward: it must equal
    # the thrust required to sustain the turn that was actually commanded.
    assert cmd.thrust_N == pytest.approx(
        vehicle.thrust_required_N(state.v_mps, state.mass_kg, cmd.omega_rad_s), rel=1e-9
    )


def test_reports_saturation_when_setpoint_exceeds_envelope(vehicle, guidance):
    """A heading flip commands far more turn rate than the vehicle can
    deliver. The applied command must be clipped, and that clipping must be
    reported -- not silently absorbed, per ADR 0006."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    setpoint = HeadingSpeedSetpoint(psi_cmd_rad=math.pi, v_cmd_mps=state.v_mps)

    cmd, sat = guidance.command(0.0, setpoint, _perfect_estimate(0.0, state), state.mass_kg)

    assert sat.omega_clipped
    assert sat.any
    assert vehicle.admissible(state, cmd)
