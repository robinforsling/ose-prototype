"""Pinning tests for the two-mode vehicle model.

Checked against section 5 of
docs/preliminary_models/vehicle/vehicle_model.pdf, not against
numbers that happened to come out of a run.

Two of these matter more than the rest.

test_turn_rate_bound_is_mode_independent pins remark 5.1, which records that
an earlier formulation making omega_max mode dependent was inconsistent with
the aerodynamic model. It is the intuitive guess and it is wrong: boost does
not let the aircraft turn tighter, it lets a given turn be held.

test_an_inadmissible_mode_is_reported_not_refused pins ADR 0006 for the
discrete input. The model declares S_q and offers a fallback; it does not
enforce, and integrating a mode outside S_q produces a visible finding rather
than a silent substitution.
"""

import dataclasses
import math

import numpy as np
import pytest

from ose.equipment.reference_configs.vehicle.planar_point_mass import (
    reference_fighter,
)
from ose.equipment.reference_configs.vehicle.planar_point_mass_with_booster import (
    FIGHTER_BOOST_LIMITS,
    reference_boosted_fighter,
)
from ose.equipment.vehicle import (
    BoostCapability,
    BoostState,
    Capability,
    Mode,
    VehicleCommand,
)
from ose.integration import rk4_step



def _at(state: BoostState, mode: Mode) -> BoostState:
    """The same state, in a given mode. The mode is a discrete state, so a
    test that varies only the mode says so by rebuilding the state."""
    return dataclasses.replace(state, mode=mode)


@pytest.fixture
def vehicle():
    return reference_boosted_fighter()


@pytest.fixture
def state():
    """16 t, 250 m/s, thermally cold."""
    return BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 0.0)


# --------------------------------------------------------------------------
# The same airframe
# --------------------------------------------------------------------------

def test_aerodynamics_are_identical_to_the_baseline(vehicle, state):
    """The document says the drag model is unchanged under boost. A comparison
    between the two models is only meaningful if that holds, and the shared
    geometry record is what makes it hold."""
    baseline = reference_fighter()
    for v_mps in (120.0, 250.0, 400.0):
        for omega in (0.0, math.radians(10.0)):
            assert vehicle.drag_N(v_mps, 16000.0, omega) == pytest.approx(
                baseline.drag_N(v_mps, 16000.0, omega)
            )
        assert vehicle.v_stall_mps(16000.0) == pytest.approx(
            baseline.v_stall_mps(16000.0)
        )


def test_turn_rate_bound_is_mode_independent(vehicle, state):
    """Remark 5.1. Lift and structure bound the instantaneous turn, and boost
    affects neither. The model takes no mode argument here at all, so this
    test pins the signature as much as the number -- and the baseline must
    agree, since the airframe is the same."""
    baseline = reference_fighter()
    for v_mps in (120.0, 180.0, 250.0, 400.0):
        assert vehicle.omega_max_rad_s(v_mps, 16000.0) == pytest.approx(
            baseline.omega_max_rad_s(v_mps, 16000.0)
        )


def test_boost_improves_the_sustained_turn_not_the_instantaneous_one(vehicle):
    """The physically correct channel. Boost buys thrust, thrust buys the
    ability to hold a turn without bleeding speed."""
    v, m = 250.0, 16000.0
    nominal = vehicle.omega_sustained_rad_s(v, m, Mode.NOMINAL)
    boosted = vehicle.omega_sustained_rad_s(v, m, Mode.BOOST)

    assert boosted > nominal
    # And neither exceeds the instantaneous bound, which did not move.
    assert boosted <= vehicle.omega_max_rad_s(v, m) + 1e-12


# --------------------------------------------------------------------------
# Mode-dependent dynamics
# --------------------------------------------------------------------------

def test_boost_burns_more_fuel_for_the_same_thrust(vehicle, state):
    """c_tsfc_boost > c_tsfc_nom, and it is the mode alone that changes."""
    command = VehicleCommand(100e3, 0.0)
    nom = vehicle.derivative(_at(state, Mode.NOMINAL), command)
    boost = vehicle.derivative(_at(state, Mode.BOOST), command)

    assert boost[4] < nom[4] < 0.0                       # mdot, both negative
    assert boost[4] == pytest.approx(-6.0e-5 * 100e3)
    assert nom[4] == pytest.approx(-2.5e-5 * 100e3)
    # Everything except fuel flow and the thermal state is untouched.
    assert boost[:4] == pytest.approx(nom[:4])


def test_the_thermal_state_fills_under_boost_and_recovers_otherwise(vehicle):
    """sigma_q(s): +1/tau_h engaged, -s/tau_c recovering."""
    cold = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 0.0)
    warm = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 0.6)

    assert vehicle.thermal_rate(_at(cold, Mode.BOOST)) == pytest.approx(1.0 / 30.0)
    assert vehicle.thermal_rate(_at(cold, Mode.NOMINAL)) == pytest.approx(0.0)
    assert vehicle.thermal_rate(_at(warm, Mode.NOMINAL)) == pytest.approx(-0.6 / 90.0)
    # Recovery is slower than accumulation, which is what makes boost costly.
    assert abs(vehicle.thermal_rate(_at(warm, Mode.NOMINAL))) < vehicle.thermal_rate(_at(warm, Mode.BOOST))


def test_sustained_boost_reaches_the_thermal_limit_in_tau_h(vehicle, state):
    """The accumulator is normalised and fills at 1/tau_h, so from cold it
    reaches the limit in tau_h seconds. Integrated rather than asserted on the
    rate, because this is the number that makes boost finite."""
    command = VehicleCommand(150e3, 0.0)

    def f(x):
        return vehicle.derivative(BoostState.from_array(x, Mode.BOOST), command)

    x = state.to_array()
    dt = 0.1
    for _ in range(int(30.0 / dt)):
        x = rk4_step(f, x, dt, normalise=vehicle.normalise_state)

    assert BoostState.from_array(x).thermal == pytest.approx(1.0, abs=1e-6)


# --------------------------------------------------------------------------
# The switching set: declared, not enforced
# --------------------------------------------------------------------------

def test_boost_is_inhibited_by_each_restriction_separately(vehicle):
    """All three conditions of the switching set, one at a time, so a change
    to any of them fails on its own rather than being masked."""
    ok = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 0.5)
    assert Mode.BOOST in vehicle.admissible_modes(ok, since_transition_s=10.0)

    hot = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 1.0)
    assert Mode.BOOST not in vehicle.admissible_modes(hot, 10.0)

    # Fuel reserve: dry mass plus reserve exactly is not "above" it.
    dry = FIGHTER_BOOST_LIMITS.nominal.mass_dry_kg
    low = BoostState(0.0, 0.0, 0.0, 250.0, dry + FIGHTER_BOOST_LIMITS.mass_reserve_kg, 0.0)
    assert Mode.BOOST not in vehicle.admissible_modes(low, 10.0)

    assert Mode.BOOST not in vehicle.admissible_modes(ok, since_transition_s=1.0)

    # Nominal is never refused, whatever the state.
    for s in (ok, hot, low):
        assert Mode.NOMINAL in vehicle.admissible_modes(s, 0.0)


def test_the_dwell_locks_the_current_mode_in_rather_than_out(vehicle):
    """The bug the document's concrete S_q has, found by building a demo.

    A single "otherwise {nom}" branch conflates two situations. With q =
    boost and the dwell not yet elapsed it says {nom}, so boost is granted on
    one step and revoked on the next -- the mode alternated every step and
    the thermal state never rose above 0.02. An anti-chatter rule that causes
    chatter.

    Dwell must lock the CURRENT mode in; only an exhausted thermal or fuel
    budget forces a mode out.
    """
    hot_enough_to_matter = 0.3
    boosting = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, hot_enough_to_matter,
                          Mode.BOOST)

    within_dwell = vehicle.admissible_modes(boosting, since_transition_s=1.0)
    assert within_dwell == frozenset({Mode.BOOST}), (
        "the dwell forced the aircraft out of boost instead of holding it there"
    )

    # From nominal it locks nominal in, symmetrically.
    cruising = dataclasses.replace(boosting, mode=Mode.NOMINAL)
    assert vehicle.admissible_modes(cruising, 1.0) == frozenset({Mode.NOMINAL})

    # And once the dwell elapses, both are on offer again.
    assert vehicle.admissible_modes(boosting, 10.0) == frozenset(Mode)


def test_the_thermal_limit_outranks_the_dwell(vehicle):
    """Order matters. If the dwell were checked first, an aircraft that hit
    its thermal limit one second after engaging would be held in boost past
    the limit -- the guard against chattering would defeat the guard against
    burning the engine."""
    hot = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 1.0, Mode.BOOST)
    assert vehicle.admissible_modes(hot, since_transition_s=0.1) == frozenset(
        {Mode.NOMINAL}
    ), "the dwell held the aircraft in boost past its thermal limit"


def test_an_inadmissible_mode_is_reported_not_refused(vehicle):
    """ADR 0006 for a discrete input.

    project_command OFFERS the fallback and says so; derivative() integrates
    whatever mode it is handed. A caller that ignores the offer gets boost
    dynamics and a state violation, not a silent substitution -- which is the
    whole point of the vehicle declaring rather than enforcing.
    """
    hot = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 1.0)
    command = VehicleCommand(150e3, 0.0)

    cmd, saturation = vehicle.project_command(
        hot, VehicleCommand(150e3, 0.0, mode=Mode.BOOST), 10.0)
    delivered = cmd.mode
    assert delivered is Mode.NOMINAL
    assert any("not admissible" in note for note in saturation.notes)

    # But the model still integrates boost if boost is what it is given.
    ignored = vehicle.derivative(_at(hot, Mode.BOOST), command)
    assert ignored[5] > 0.0, "thermal state kept accumulating, as commanded"
    assert ignored[4] == pytest.approx(-6.0e-5 * 150e3), "burned at the boost rate"

    over = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 1.4)
    assert any("thermal" in v for v in vehicle.state_violations(_at(over, Mode.BOOST)))


def test_the_thrust_ceiling_follows_the_delivered_mode(vehicle, state):
    """A command is clipped against the mode actually flown, not the one
    requested -- otherwise a denied boost would still raise the ceiling."""
    hot = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, 1.0)
    ask = VehicleCommand(170e3, 0.0, mode=Mode.BOOST)   # inside boost, outside nominal

    cmd, sat = vehicle.project_command(state, ask, 10.0)
    mode = cmd.mode
    assert mode is Mode.BOOST
    assert cmd.thrust_N == pytest.approx(170e3)
    assert not sat.thrust_clipped

    cmd, sat = vehicle.project_command(hot, ask, 10.0)
    mode = cmd.mode
    assert mode is Mode.NOMINAL
    assert cmd.thrust_N == pytest.approx(FIGHTER_BOOST_LIMITS.nominal.thrust_max_N)
    assert sat.thrust_clipped


def test_the_speed_ceiling_is_mode_dependent(vehicle):
    """v_max_boost > v_max_nom, so the same state is a violation in one mode
    and not the other."""
    fast = BoostState(0.0, 0.0, 0.0, 650.0, 16000.0, 0.2)
    assert any("above" in v for v in vehicle.state_violations(_at(fast, Mode.NOMINAL)))
    assert not any("above" in v for v in vehicle.state_violations(_at(fast, Mode.BOOST)))


def test_the_mass_ceiling_is_inherited_from_the_composed_limits(vehicle):
    """BoostConstraints composes the baseline's Constraints, so m_max is
    declared once and both models report against the same number."""
    ceiling = FIGHTER_BOOST_LIMITS.nominal.mass_max_kg
    over = BoostState(0.0, 0.0, 0.0, 250.0, ceiling + 100.0, 0.0, Mode.NOMINAL)
    assert any("above maximum" in v for v in vehicle.state_violations(over))


# --------------------------------------------------------------------------
# Capability
# --------------------------------------------------------------------------

def test_capability_is_a_capability(vehicle, state):
    """Subclassing keeps a consumer typed to Capability working; only a
    consumer that cares about boost needs the wider type."""
    c = vehicle.capability(_at(state, Mode.NOMINAL))
    assert isinstance(c, BoostCapability)
    assert isinstance(c, Capability)


@pytest.mark.performance
def test_boost_shortens_endurance_and_raises_available_thrust(vehicle, state):
    nom = vehicle.capability(_at(state, Mode.NOMINAL))
    boost = vehicle.capability(_at(state, Mode.BOOST))

    assert boost.thrust_available_N > nom.thrust_available_N
    assert boost.endurance_s < nom.endurance_s
    # Same fuel on board; it is the rate that differs.
    assert boost.fuel_mass_kg == pytest.approx(nom.fuel_mass_kg)


def test_capability_reports_how_long_boost_could_be_held(vehicle):
    """t_boost = tau_h (s_max - s), equation boost-available. From cold that
    is the full tau_h, and it falls to zero at the limit."""
    for thermal, expected in ((0.0, 30.0), (0.5, 15.0), (1.0, 0.0)):
        s = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, thermal)
        c = vehicle.capability(_at(s, Mode.NOMINAL), since_transition_s=10.0)
        assert c.boost_time_remaining_s == pytest.approx(expected)


def test_capability_reports_whether_boost_is_selectable(vehicle):
    """c_boost. It answers the switching question, so it must agree with
    admissible_modes rather than being a second opinion about it."""
    for thermal, dwell in ((0.0, 10.0), (1.0, 10.0), (0.0, 1.0)):
        s = BoostState(0.0, 0.0, 0.0, 250.0, 16000.0, thermal)
        c = vehicle.capability(_at(s, Mode.NOMINAL), since_transition_s=dwell)
        assert c.boost_available == (
            Mode.BOOST in vehicle.admissible_modes(s, dwell)
        )
