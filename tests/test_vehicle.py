"""Pinning tests for the baseline vehicle model.

These check identities that follow from the model document, not numbers that
happened to come out of a run. A failure means either the model changed or the
document is no longer being honoured -- both worth knowing about.
"""

import dataclasses
import math

import numpy as np
import pytest

from ose.environment import Environment
from ose.equipment.reference_configs.vehicle.planar_point_mass import (
    FIGHTER_GEOMETRY,
    FIGHTER_LIMITS,
    reference_fighter,
)
from ose.equipment.vehicle import (
    NO_DISTURBANCE,
    Disturbance,
    VehicleCommand,
    VehicleGeometry,
    VehicleState,
)
from ose.integration import step_rk4
from ose.reference_configs.reference_environment import ISA_SEA_LEVEL

G = 9.80665


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def state():
    return VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)


# --------------------------------------------------------------------------
# A configuration is data
# --------------------------------------------------------------------------

def test_the_configuration_is_records_not_a_factory():
    """What makes a vehicle configuration authorable by someone who only has
    numbers. Every other reference config in the package is a plain record;
    the vehicle used to be a function that derived parameters, chose an
    environment and constructed a component in one step."""
    assert isinstance(FIGHTER_GEOMETRY, VehicleGeometry)
    assert dataclasses.is_dataclass(FIGHTER_LIMITS)
    # And the geometry is the authored form, not the lumped one: a
    # contributor states a drag polar, not c_p and c_i.
    assert FIGHTER_GEOMETRY.wing_area_m2 == 38.0
    assert FIGHTER_GEOMETRY.cd0 == 0.022


def test_geometry_lumps_by_the_documented_formulas(vehicle):
    """VehicleGeometry.to_parameters is the bridge between the authored form
    and the form the dynamics integrate, and it must match section 2 of the
    model document rather than being a second opinion about the drag polar."""
    g = ISA_SEA_LEVEL.g
    theta = FIGHTER_GEOMETRY.to_parameters(ISA_SEA_LEVEL)

    assert theta.c_p == pytest.approx(0.5 * 38.0 * 0.022)
    assert theta.c_i == pytest.approx(2.0 * g**2 / (math.pi * 0.80 * 3.0 * 38.0))
    assert theta.c_l == pytest.approx(0.5 * 38.0 * 1.20)
    assert theta.c_tsfc == pytest.approx(2.5e-5)
    # And the assembled reference vehicle really is built from them.
    assert vehicle.theta == theta


def test_the_configuration_does_not_pin_an_environment():
    """The lumped induced-drag parameter carries g, so the same airframe is a
    different set of parameters under different gravity. A configuration that
    fixed an environment would be fixing an aeroplane to an altitude."""
    moon_ish = Environment(g=1.62, rho=ISA_SEA_LEVEL.rho)

    earth = reference_fighter()
    elsewhere = reference_fighter(moon_ish)

    assert elsewhere.eta.g == 1.62
    assert elsewhere.theta.c_i != earth.theta.c_i
    # Only the g-dependent term moves; the rest of the airframe is unchanged.
    assert elsewhere.theta.c_p == earth.theta.c_p
    assert elsewhere.theta.c_l == earth.theta.c_l
    assert elsewhere.lam == earth.lam


def test_a_variant_is_one_replace_away():
    """The ergonomics the split exists for: varying a bundled configuration
    should not mean copying a factory function and editing its body."""
    heavier = dataclasses.replace(FIGHTER_LIMITS, mass_dry_kg=14_000.0)

    assert heavier.mass_dry_kg == 14_000.0
    assert heavier.thrust_max_N == FIGHTER_LIMITS.thrust_max_N
    assert FIGHTER_LIMITS.mass_dry_kg == 12_000.0, "the bundled config was mutated"


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
    m = 16000.0
    for n in (2.0, 3.0, 4.0):
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
    test starts failing, the separation described in ADR 0006 has been broken.

    Note what the resulting motion looks like: 5 rad/s at 250 m/s implies a
    load factor near 128, and induced drag scales with n^2, so drag reaches
    some 36 MN against 1.3 MN of thrust and the vehicle decelerates violently.
    That number is physically meaningless -- which is the point. Outside the
    declared sets the model integrates faithfully but represents nothing.
    """
    dt = 0.02
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 5.0)
    assert not vehicle.admissible(state, absurd)
    assert absurd.omega_rad_s > 10 * vehicle.omega_max_rad_s(state.v_mps, state.mass_kg)

    s = step_rk4(vehicle, state, absurd, dt)

    # The commanded turn rate was integrated verbatim rather than projected.
    assert s.psi_rad == pytest.approx(
        state.psi_rad + absurd.omega_rad_s * dt, rel=1e-12
    )


def test_inadmissible_thrust_is_applied_unclipped(vehicle, state):
    """Separated from the turn rate, so induced drag does not mask the effect."""
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 0.0)
    assert not vehicle.admissible(state, absurd)
    s = step_rk4(vehicle, state, absurd, 0.05)
    assert s.v_mps > state.v_mps + 1.0        # +3.95 m/s in practice


def test_project_command_is_offered_not_applied(vehicle, state):
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 5.0)
    applied, sat = vehicle.project_command(state, absurd)
    assert sat.any
    assert applied.thrust_N == pytest.approx(vehicle.lam.thrust_max_N)
    assert abs(applied.omega_rad_s) == pytest.approx(
        vehicle.omega_max_rad_s(state.v_mps, state.mass_kg)
    )


def test_state_violations_is_silent_inside_the_envelope(vehicle, state):
    assert vehicle.state_violations(state) == []


def test_state_violations_reports_each_bound_it_owns(vehicle):
    """One case per branch. The check had no tests at all, and
    demo_vehicle.py's "envelope events: 0" could equally have meant no
    violations or a check that never fires."""
    m = 16000.0
    slow = VehicleState(0.0, 0.0, 0.0, 40.0, m)
    fast = VehicleState(0.0, 0.0, 0.0, vehicle.lam.v_max_mps + 50.0, m)
    light = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg - 500.0)

    assert any("below floor" in s for s in vehicle.state_violations(slow))
    assert any("above" in s for s in vehicle.state_violations(fast))
    assert any("dry mass" in s for s in vehicle.state_violations(light))


def test_state_violation_floor_follows_stall_not_just_the_hard_minimum(vehicle):
    """The floor is mass-dependent, so a speed legal when light is illegal
    when heavy. A check comparing only against v_min_mps would pass the
    heavy case and be wrong."""
    v_test = 95.0                       # above the 90 m/s hard minimum
    light = VehicleState(0.0, 0.0, 0.0, v_test, 16000.0)
    heavy = VehicleState(0.0, 0.0, 0.0, v_test, 30000.0)

    assert vehicle.state_violations(light) == []
    assert any("below floor" in s for s in vehicle.state_violations(heavy))


def test_state_violations_reports_but_does_not_correct(vehicle):
    """The state-side counterpart to project_command, and deliberately
    weaker: a command can be projected before it is applied, a state cannot
    be projected without falsifying the dynamics that produced it."""
    bad = VehicleState(0.0, 0.0, 0.0, 40.0, 16000.0)
    before = (bad.v_mps, bad.mass_kg)
    assert vehicle.state_violations(bad)
    assert (bad.v_mps, bad.mass_kg) == before


def test_integrating_to_fuel_exhaustion_is_reported_not_hidden(vehicle):
    """RK4 undershoots dry mass at exhaustion, and state_violations() is
    what makes that visible.

    Fuel flow is gated on `m > mass_dry_kg`, a discontinuity. RK4 assumes a
    smooth derivative, so stages still burning carry the weighted sum past
    the boundary. The undershoot is O(dt) -- tens of grams at the working
    step -- and once tripped it stays tripped, because mass then freezes
    just below dry. Small enough to ignore physically; the point is that the
    model does not pretend it did not happen.
    """
    state = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg + 200.0)
    cmd = VehicleCommand(vehicle.lam.thrust_max_N, 0.0)
    for _ in range(int(600.0 / 0.5)):
        state = step_rk4(vehicle, state, cmd, 0.5)

    undershoot = vehicle.lam.mass_dry_kg - state.mass_kg
    assert undershoot > 0.0                      # it really does overshoot
    assert undershoot < 5.0                      # but only by a little
    assert any("dry mass" in s for s in vehicle.state_violations(state))


def test_saturation_reports_what_was_requested(vehicle, state):
    """The pre-enforcement command must be recoverable as numbers, not only
    from the note strings. Without it a caller wanting to show what was asked
    for has to duplicate the control law, which is exactly what
    demo_vehicle_guidance.py did until this field existed -- and that
    duplicate went stale the first time the law changed."""
    absurd = VehicleCommand(10 * vehicle.lam.thrust_max_N, 5.0)
    applied, sat = vehicle.project_command(state, absurd)

    assert sat.requested is not None
    assert sat.requested.thrust_N == absurd.thrust_N
    assert sat.requested.omega_rad_s == absurd.omega_rad_s
    # And it is the *requested* command, not the projected one.
    assert sat.requested.thrust_N != applied.thrust_N
    assert sat.requested.omega_rad_s != applied.omega_rad_s


def test_saturation_reports_the_request_even_when_nothing_was_clipped(vehicle, state):
    """Always populated, so a consumer needs no None handling and no
    special case for the unclipped path. sat.any is what says whether the
    request differed from what was delivered."""
    fine = VehicleCommand(vehicle.thrust_required_N(state.v_mps, state.mass_kg), 0.05)
    applied, sat = vehicle.project_command(state, fine)

    assert not sat.any
    assert sat.requested is not None
    assert sat.requested.thrust_N == pytest.approx(applied.thrust_N)
    assert sat.requested.omega_rad_s == pytest.approx(applied.omega_rad_s)
