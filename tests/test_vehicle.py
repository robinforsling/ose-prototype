"""Pinning tests for the baseline vehicle model.

These check identities that follow from the model document, not numbers that
happened to come out of a run. A failure means either the model changed or the
document is no longer being honoured -- both worth knowing about.
"""

import math

import numpy as np
import pytest

from ose.resource.vehicle import (
    NO_DISTURBANCE,
    Disturbance,
    VehicleCommand,
    VehicleState,
    reference_fighter,
    step_rk4,
)

G = 9.80665


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def state():
    return VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)


# --------------------------------------------------------------------------
# Aerodynamics
# --------------------------------------------------------------------------

def test_load_factor_matches_coordinated_turn_relation(vehicle):
    """n^2 = 1 + (v omega / g)^2, equation (21) of the model document."""
    for v, omega in ((200.0, 0.1), (300.0, 0.25), (450.0, 0.05)):
        expected = math.sqrt(1.0 + (v * omega / G) ** 2)
        assert vehicle.load_factor(v, omega) == pytest.approx(expected, rel=1e-12)


def test_induced_drag_scales_with_load_factor_squared(vehicle, state):
    """Induced drag is proportional to n^2; parasite drag is independent of it."""
    v, m = state.v_mps, state.mass_kg
    d0 = vehicle.drag_N(v, m, 0.0)
    omega = 0.2
    d1 = vehicle.drag_N(v, m, omega)
    n2 = vehicle.load_factor(v, omega) ** 2

    parasite = vehicle.eta.rho * vehicle.theta.c_p * v**2
    induced_0 = d0 - parasite
    induced_1 = d1 - parasite
    assert induced_1 / induced_0 == pytest.approx(n2, rel=1e-12)


def test_stall_speed_closed_form(vehicle):
    """v_s = sqrt(n m g / (rho c_l)); at v_s the lift-limited load factor is n."""
    m, n = 16000.0, 3.0
    v_s = vehicle.v_stall_mps(m, n)
    assert v_s == pytest.approx(
        math.sqrt(n * m * vehicle.eta.g / (vehicle.eta.rho * vehicle.theta.c_l))
    )
    assert vehicle.lift_limited_load_factor(v_s, m) == pytest.approx(n, rel=1e-12)


def test_corner_speed_maximises_instantaneous_turn_rate(vehicle):
    """Turn rate peaks at v_corner = v_stall sqrt(n_max)."""
    m = 16000.0
    v_c = vehicle.v_corner_mps(m)
    peak = vehicle.omega_max_rad_s(v_c, m)
    for v in np.linspace(0.6 * v_c, 2.5 * v_c, 200):
        assert vehicle.omega_max_rad_s(float(v), m) <= peak + 1e-9


def test_turn_rate_limit_respects_structural_load_factor(vehicle):
    """Above corner speed the structural limit binds, so n never exceeds n_max."""
    m = 16000.0
    for v in (250.0, 350.0, 450.0, 550.0):
        omega = vehicle.omega_max_rad_s(v, m)
        assert vehicle.load_factor(v, omega) <= vehicle.lam.n_structural + 1e-9


# --------------------------------------------------------------------------
# Capability
# --------------------------------------------------------------------------

def test_thrust_required_equals_drag(vehicle, state):
    """T_req = D, equation (44) of the model document."""
    for omega in (0.0, 0.1, 0.2):
        assert vehicle.thrust_required_N(
            state.v_mps, state.mass_kg, omega
        ) == pytest.approx(vehicle.drag_N(state.v_mps, state.mass_kg, omega))


def test_sustained_turn_rate_balances_thrust_and_drag(vehicle, state):
    """At omega_sus the drag equals available thrust, so vdot is zero."""
    v, m = state.v_mps, state.mass_kg
    omega = vehicle.omega_sustained_rad_s(v, m)
    assert omega > 0.0
    assert vehicle.drag_N(v, m, omega) == pytest.approx(
        vehicle.lam.thrust_max_N, rel=1e-9
    )


def test_sustained_never_exceeds_instantaneous(vehicle):
    m = 16000.0
    for v in np.linspace(100.0, 600.0, 300):
        v = float(v)
        assert (
            vehicle.omega_sustained_rad_s(v, m)
            <= vehicle.omega_max_rad_s(v, m) + 1e-12
        )


def test_sustained_turn_actually_holds_speed(vehicle, state):
    """Integrate a sustained-rate turn: airspeed must not decay."""
    v0 = state.v_mps
    omega = vehicle.omega_sustained_rad_s(v0, state.mass_kg)
    cmd = VehicleCommand(vehicle.lam.thrust_max_N, omega)
    s = state
    for _ in range(int(30.0 / 0.02)):
        s = step_rk4(vehicle, s, cmd, 0.02)
    # Mass falls as fuel burns, so speed creeps up very slightly rather than down.
    assert s.v_mps >= v0 - 0.1
    assert s.v_mps <= v0 + 2.0


# --------------------------------------------------------------------------
# Dynamics and integration
# --------------------------------------------------------------------------

def test_frame_convention_heading_zero_is_north(vehicle, state):
    """psi = 0 must move the vehicle along +p_x (north) only."""
    d = vehicle.derivative(state, VehicleCommand(0.0, 0.0))
    assert d[0] == pytest.approx(state.v_mps)
    assert d[1] == pytest.approx(0.0, abs=1e-12)


def test_positive_omega_turns_right(vehicle, state):
    """Heading clockwise from north, so positive omega increases psi."""
    s = step_rk4(vehicle, state, VehicleCommand(50e3, 0.1), 1.0)
    assert s.psi_rad > state.psi_rad


def test_wind_enters_kinematics_but_not_drag(vehicle, state):
    """Airspeed drives drag; wind only displaces the ground track."""
    wind = Disturbance(wind_x_mps=20.0, wind_y_mps=-10.0)
    d_still = vehicle.derivative(state, VehicleCommand(50e3, 0.0), NO_DISTURBANCE)
    d_wind = vehicle.derivative(state, VehicleCommand(50e3, 0.0), wind)
    assert d_wind[0] - d_still[0] == pytest.approx(20.0)
    assert d_wind[1] - d_still[1] == pytest.approx(-10.0)
    assert d_wind[3] == pytest.approx(d_still[3])       # vdot unchanged


def test_rk4_is_fourth_order(vehicle, state):
    """Halving the step must reduce the error by roughly 2^4."""
    cmd = VehicleCommand(120e3, 0.15)

    def endpoint(dt):
        s = VehicleState(*state.to_array())
        for _ in range(int(round(20.0 / dt))):
            s = step_rk4(vehicle, s, cmd, dt)
        return s.to_array()

    ref = endpoint(0.001)
    e_coarse = np.linalg.norm(endpoint(0.2)[:2] - ref[:2])
    e_fine = np.linalg.norm(endpoint(0.1)[:2] - ref[:2])
    assert e_coarse / e_fine > 8.0


def test_fuel_flow_stops_at_dry_mass(vehicle):
    """Below dry mass the vehicle must not keep burning fuel."""
    empty = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg)
    d = vehicle.derivative(empty, VehicleCommand(100e3, 0.0))
    assert d[4] == pytest.approx(0.0)


# --------------------------------------------------------------------------
# Constraint declaration, not enforcement
# --------------------------------------------------------------------------

def test_vehicle_does_not_enforce_constraints(vehicle, state):
    """An inadmissible command must be integrated as given, not clipped.

    Enforcement belongs to guidance or to a runtime assurance layer. If this
    test starts failing, the separation described in the model document has
    been broken.
    """
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 5.0)
    assert not vehicle.admissible(state, absurd)
    s = step_rk4(vehicle, state, absurd, 0.02)
    assert s.psi_rad != pytest.approx(state.psi_rad)
    assert s.v_mps > state.v_mps + 1.0      # the absurd thrust was applied


def test_project_command_is_offered_not_applied(vehicle, state):
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 5.0)
    applied, sat = vehicle.project_command(state, absurd)
    assert sat.any
    assert applied.thrust_N == pytest.approx(vehicle.lam.thrust_max_N)
    assert abs(applied.omega_rad_s) == pytest.approx(
        vehicle.omega_max_rad_s(state.v_mps, state.mass_kg)
    )
