"""Capability honesty tests.

A capability model is a claim, the same way a published covariance is a
claim, and it deserves the same treatment: this repository's testing
philosophy says any component publishing an uncertainty must have a test
that the stated uncertainty is honest. Capability is the same argument.
An overconfident capability -- a sustained turn rate the vehicle cannot
actually hold, an endurance it cannot actually fly -- would silently
corrupt every planner that trusts it, and a planner is precisely the
consumer capability exists for.

So these tests do not check that capability() returns plausible numbers.
They integrate the dynamics forward and check the vehicle actually
delivers what it claimed, which is the one thing capability() promises to
answer without integrating.
"""

import dataclasses
import importlib
import inspect
import math
from pathlib import Path

import numpy as np
import pytest

from ose import interfaces
from ose.resource.air_data import AirDataSensor as AirDataSensorImpl
from ose.resource.clock import Clock
from ose.resource.fuel_gauge import FuelGauge
from ose.resource.gnss import GnssReceiver
from ose.resource.imu import Imu
from ose.resource.integrated_navigation_unit import IntegratedNavUnit
from ose.resource.reference_configs.reference_air_data import STANDARD as AIR_DATA_STANDARD
from ose.resource.reference_configs.reference_clock import STANDARD as CLOCK_STANDARD
from ose.resource.reference_configs.reference_fuel_gauge import STANDARD as FUEL_STANDARD
from ose.resource.reference_configs.reference_gnss import STANDARD as GNSS_STANDARD
from ose.resource.reference_configs.reference_imu import TACTICAL_GRADE
from ose.resource.reference_configs.reference_integrated_navigation_unit import (
    STANDARD as INTEGRATED_NAV_STANDARD,
)
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import Disturbance, VehicleCommand, VehicleState, step_rk4

DT = 0.02


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def state():
    return VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)


def _fly(vehicle, state, cmd, duration_s, dt=DT):
    s = state
    for _ in range(int(duration_s / dt)):
        s = step_rk4(vehicle, s, cmd, dt)
    return s


# --------------------------------------------------------------------------
# Protocol conformance
# --------------------------------------------------------------------------

def test_every_resource_module_defines_a_component_with_capability():
    """Discovered, not hand-listed, on purpose.

    An earlier version of this test enumerated resources by hand under the
    name "every resource" and quietly omitted IntegratedNavUnit, so a
    resource with no capability() at all passed unnoticed -- coverage in
    name only. Walking the package means adding a resource without a
    capability model fails here rather than going unremarked.
    """
    package = importlib.import_module("ose.resource")
    package_dir = Path(package.__file__).parent

    module_names = sorted(
        p.stem
        for p in package_dir.glob("*.py")
        if p.stem != "__init__"
    )
    assert module_names, "no resource modules discovered -- test is vacuous"

    for name in module_names:
        module = importlib.import_module(f"ose.resource.{name}")
        components = [
            obj
            for attr, obj in vars(module).items()
            if inspect.isclass(obj)
            and obj.__module__ == module.__name__
            and not attr.endswith("Parameters")
            and not dataclasses.is_dataclass(obj)
        ]
        assert components, f"ose.resource.{name} defines no component class"
        for component in components:
            assert hasattr(component, "capability"), (
                f"{component.__name__} in ose.resource.{name} has no "
                "capability(); every resource must be answerable"
            )


def test_constructed_resources_satisfy_the_capability_protocol(vehicle):
    """hasattr above is a structural check; this one instantiates and
    confirms the runtime-checkable protocol actually holds."""
    mass_dry_kg = vehicle.lam.mass_dry_kg
    rng = np.random.default_rng(0)
    for resource in (
        vehicle,
        Imu(TACTICAL_GRADE, rng, vehicle),
        GnssReceiver(GNSS_STANDARD, rng=rng),
        AirDataSensorImpl(AIR_DATA_STANDARD, rng=rng),
        Clock(CLOCK_STANDARD, rng=rng),
        FuelGauge(FUEL_STANDARD, mass_dry_kg, rng=rng),
        IntegratedNavUnit(INTEGRATED_NAV_STANDARD, rng=rng),
    ):
        assert isinstance(resource, interfaces.CapabilityModel)


def test_every_sensor_declares_at_least_one_channel(vehicle):
    """A capability with no channels declares nothing about accuracy."""
    rng = np.random.default_rng(0)
    for sensor in (
        Imu(TACTICAL_GRADE, rng, vehicle),
        GnssReceiver(GNSS_STANDARD, rng=rng),
        AirDataSensorImpl(AIR_DATA_STANDARD, rng=rng),
        Clock(CLOCK_STANDARD, rng=rng),
        FuelGauge(FUEL_STANDARD, vehicle.lam.mass_dry_kg, rng=rng),
        IntegratedNavUnit(INTEGRATED_NAV_STANDARD, rng=rng),
    ):
        cap = sensor.capability()
        assert cap.channels
        for c in cap.channels:
            assert c.name and c.units
            assert c.sigma > 0.0


# --------------------------------------------------------------------------
# Vehicle: claims checked against integrated dynamics
# --------------------------------------------------------------------------

def test_claimed_sustained_turn_rate_holds_speed(vehicle, state):
    """omega_sustained is claimed to be holdable without losing speed. Fly
    it and check. This is the claim a planner leans on hardest, because it
    is what makes a turn's duration predictable."""
    c = vehicle.capability(state)
    cmd = VehicleCommand(vehicle.lam.thrust_max_N, c.omega_sustained_rad_s)
    flown = _fly(vehicle, state, cmd, 30.0)

    # Fuel burn lightens the aircraft, so speed creeps up slightly rather
    # than decaying. What must not happen is a decay.
    assert flown.v_mps >= state.v_mps - 0.1


def test_claimed_instantaneous_turn_rate_is_not_sustainable(vehicle, state):
    """The converse claim, and the one that makes omega_available honest
    rather than merely optimistic: the instantaneous limit is achievable
    but bleeds energy. If it did not, omega_sustained would be redundant."""
    c = vehicle.capability(state)
    assert c.omega_available_rad_s > c.omega_sustained_rad_s

    cmd = VehicleCommand(vehicle.lam.thrust_max_N, c.omega_available_rad_s)
    flown = _fly(vehicle, state, cmd, 30.0)
    assert flown.v_mps < state.v_mps - 10.0


def test_claimed_turn_radius_matches_flown_radius(vehicle, state):
    """turn_radius_min_m is claimed as v / omega_available. Fly a full
    instantaneous-rate turn and measure the radius actually described."""
    c = vehicle.capability(state)
    omega = c.omega_available_rad_s
    cmd = VehicleCommand(vehicle.lam.thrust_max_N, omega)

    # A quarter turn keeps speed loss small enough that the claimed radius,
    # which assumes constant speed, remains a fair comparison.
    quarter_turn_s = (math.pi / 2.0) / omega
    s = state
    xs, ys = [s.p_x_m], [s.p_y_m]
    for _ in range(int(quarter_turn_s / DT)):
        s = step_rk4(vehicle, s, cmd, DT)
        xs.append(s.p_x_m)
        ys.append(s.p_y_m)

    # Centre of a right turn from heading 0 lies at (0, +r).
    chord_x = xs[-1] - xs[0]
    chord_y = ys[-1] - ys[0]
    flown_radius = math.hypot(chord_x, chord_y) / math.sqrt(2.0)  # quarter arc
    assert flown_radius == pytest.approx(c.turn_radius_min_m, rel=0.1)


def test_claimed_max_acceleration_is_achievable(vehicle, state):
    """accel_max_mps2 is claimed for the current instant at full thrust,
    wings level. Check the dynamics agree at that instant."""
    c = vehicle.capability(state)
    cmd = VehicleCommand(vehicle.thrust_available_N(state), 0.0)
    v_dot = vehicle.derivative(state, cmd)[3]
    assert v_dot == pytest.approx(c.accel_max_mps2, rel=1e-9)


def test_claimed_min_acceleration_is_achievable(vehicle, state):
    c = vehicle.capability(state)
    cmd = VehicleCommand(vehicle.lam.thrust_min_N, 0.0)
    v_dot = vehicle.derivative(state, cmd)[3]
    assert v_dot == pytest.approx(c.accel_min_mps2, rel=1e-9)


def test_claimed_thrust_required_actually_holds_speed(vehicle, state):
    """thrust_required_N is claimed to hold the current speed. Fly it
    wings-level and check speed is steady."""
    c = vehicle.capability(state)
    cmd = VehicleCommand(c.thrust_required_N, 0.0)
    flown = _fly(vehicle, state, cmd, 30.0)
    # Mass drops as fuel burns, so the same thrust accelerates slightly.
    assert abs(flown.v_mps - state.v_mps) < 2.0


def test_claimed_endurance_is_honest(vehicle, state):
    """endurance_s claims how long the fuel lasts at the current burn rate.

    Checked in both directions, because they fail differently and only one
    of them is dangerous. Understating endurance wastes fuel a planner
    could have used; overstating it puts the aircraft in the air with no
    fuel, and a naive test cannot see it: once the tanks are dry mdot
    stops, so simply flying past an overstated claim leaves fuel_left at
    zero and looks perfectly healthy. The short-of-the-claim assertion is
    what actually catches that.
    """
    c = vehicle.capability(state)
    cmd = VehicleCommand(c.thrust_required_N, 0.0)

    # Not overstated: shortly before the claim, fuel must still remain.
    early = _fly(vehicle, state, cmd, 0.95 * c.endurance_s, dt=0.5)
    assert early.mass_kg - vehicle.lam.mass_dry_kg > 0.02 * c.fuel_mass_kg

    # Not understated: at the claim, the tanks are essentially dry. At
    # constant commanded thrust the burn rate is constant, so this should
    # be near-exact -- it is a consistency check between capability()'s
    # endurance formula and derivative()'s mdot, and a loose bound would
    # not catch them drifting apart. Observed residual is ~0.3 kg of 4000.
    at_claim = _fly(vehicle, state, cmd, c.endurance_s, dt=0.5)
    fuel_left = at_claim.mass_kg - vehicle.lam.mass_dry_kg
    assert fuel_left >= -1.0
    assert fuel_left < 0.01 * c.fuel_mass_kg


def test_claimed_fuel_mass_matches_state(vehicle, state):
    c = vehicle.capability(state)
    assert c.fuel_mass_kg == pytest.approx(state.mass_kg - vehicle.lam.mass_dry_kg)


def test_capability_degrades_as_fuel_burns(vehicle, state):
    """Capability is a function of state, not a constant. A lighter
    aircraft turns harder -- if this fails, capability() is ignoring mass."""
    heavy = vehicle.capability(state)
    light = vehicle.capability(
        VehicleState(0.0, 0.0, 0.0, state.v_mps, vehicle.lam.mass_dry_kg)
    )
    assert light.omega_sustained_rad_s > heavy.omega_sustained_rad_s
    assert light.v_stall_mps < heavy.v_stall_mps
    assert light.fuel_mass_kg < heavy.fuel_mass_kg


def test_dry_tanks_claim_no_thrust_and_no_endurance(vehicle):
    """At dry mass the vehicle claims no thrust available. A planner that
    trusts a non-zero claim here would plan flight it cannot perform."""
    dry = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg)
    c = vehicle.capability(dry)
    assert c.thrust_available_N == 0.0
    assert c.fuel_mass_kg == 0.0
    assert c.endurance_s == 0.0


# --------------------------------------------------------------------------
# Sensors: claims checked against what they actually publish
# --------------------------------------------------------------------------

def test_sensor_capability_matches_published_measurements(vehicle):
    """A sensor's declared capability must agree with what its own
    measurements declare -- otherwise a planner and a filter reading the
    same sensor would disagree about its accuracy."""
    truth = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    fuelled = VehicleState(0.0, 0.0, 0.0, 250.0, vehicle.lam.mass_dry_kg + 4000.0)
    dist = Disturbance()

    air = AirDataSensorImpl(AIR_DATA_STANDARD, rng=np.random.default_rng(0))
    assert (
        air.capability().channel("airspeed").sigma
        == air.sample(0.0, truth).airspeed_sigma_mps
    )

    gauge = FuelGauge(FUEL_STANDARD, vehicle.lam.mass_dry_kg, rng=np.random.default_rng(0))
    assert (
        gauge.capability().channel("fuel_remaining").sigma
        == gauge.sample(0.0, fuelled).fuel_remaining_sigma_kg
    )

    clock = Clock(CLOCK_STANDARD, rng=np.random.default_rng(0))
    assert (
        clock.capability().channel("elapsed").sigma
        == clock.sample(0.0, 0.05).elapsed_sigma_s
    )

    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0))
    fix = gnss.sample(0.0, truth, dist)
    cap = gnss.capability()
    assert cap.channel("position").sigma == fix.position_sigma_m
    assert cap.channel("velocity").sigma == fix.velocity_sigma_mps


def test_gnss_declares_both_channels_not_just_position(vehicle):
    """The defect this record shape exists to prevent: GNSS declares two
    accuracies with different units, and a single-valued capability
    silently reported only the first."""
    cap = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0)).capability()
    assert {c.name for c in cap.channels} == {"position", "velocity"}
    assert cap.channel("position").units == "m"
    assert cap.channel("velocity").units == "m/s"


def test_gnss_without_velocity_aiding_declares_no_velocity_channel():
    """A receiver that will not publish velocity must not claim an accuracy
    for it -- an absent channel, not a channel with a meaningless number."""
    par = dataclasses.replace(GNSS_STANDARD, gnss_velocity_enabled=False)
    cap = GnssReceiver(par, rng=np.random.default_rng(0)).capability()
    assert {c.name for c in cap.channels} == {"position"}
    with pytest.raises(KeyError):
        cap.channel("velocity")


def test_imu_declares_densities_and_owns_no_rate():
    """The IMU cannot state a per-sample sigma without knowing the interval,
    so it declares densities and reports no rate. A consumer that assumed
    sigma here would be wrong by a factor of sqrt(dt)."""
    cap = Imu(TACTICAL_GRADE, np.random.default_rng(0), reference_fighter()).capability()
    assert cap.rate_hz is None
    assert cap.interval_s is None
    assert {c.name for c in cap.channels} == {"specific_force", "angular_rate"}
    assert cap.channel("specific_force").units.endswith("/sqrt(Hz)")
    assert cap.channel("angular_rate").units.endswith("/sqrt(Hz)")

    # The density really is the per-sample sigma scaled by sqrt(dt).
    dt = 0.05
    m = Imu(TACTICAL_GRADE, np.random.default_rng(0), reference_fighter()).sample(
        0.0, dt, VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0),
        VehicleCommand(0.0, 0.0), Disturbance(),
    )
    assert m.specific_force_sigma_mps2 == pytest.approx(
        cap.channel("specific_force").sigma / math.sqrt(dt)
    )


def test_clock_owns_no_rate_but_declares_a_true_sigma():
    """Unlike the IMU, the clock's white noise is a fixed per-reading
    jitter, so its declared sigma is directly comparable to what its
    measurements carry -- even though it owns no rate either."""
    cap = Clock(CLOCK_STANDARD, rng=np.random.default_rng(0)).capability()
    assert cap.rate_hz is None
    assert cap.channel("elapsed").units == "s"


def test_sensor_capability_rate_matches_due(vehicle):
    """rate_hz must be the rate due() actually gates on, or a planner
    scheduling against capability would mis-time its requests."""
    air = AirDataSensorImpl(AIR_DATA_STANDARD, rng=np.random.default_rng(0))
    truth = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)

    interval = air.capability().interval_s
    air.sample(0.0, truth)
    assert not air.due(0.9 * interval)
    assert air.due(interval)


def test_gnss_capability_reports_denial(vehicle):
    """The one resource whose capability is genuinely dynamic. A planner
    asking during an outage must be told the receiver cannot deliver."""
    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0))
    assert gnss.capability().available

    gnss.set_gnss_available(False)
    assert not gnss.capability().available

    gnss.set_gnss_available(True)
    assert gnss.capability().available
