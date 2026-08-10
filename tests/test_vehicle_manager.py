"""Tests for the vehicle manager.

Two properties carry most of the weight.

test_sigma_comes_from_the_measurement is invariant 4: the manager republishes
the uncertainty that travelled with the fuel reading and does not substitute a
configured value. It is checked by feeding two readings that differ only in
their declared sigma, because a manager that quietly used its own number would
otherwise be indistinguishable from a correct one.

test_capability_is_evaluated_at_the_believed_mass is the reason the component
exists at all. A biased fuel reading must move the reported envelope, or the
manager is not actually binding the mass it claims to own and every consumer
is back to reading truth.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from ose.interfaces import FuelMeasurement, MassEstimate, OwnStateEstimate
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleCommand
from ose.subsystem.reference_configs.reference_vehicle_manager import STANDARD
from ose.subsystem.vehicle_manager import VehicleManager, VehicleManagerParameters


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
    loaded = VehicleManagerParameters(
        payload_mass_kg=750.0, initial_fuel_kg=4000.0, initial_fuel_sigma_kg=200.0
    )
    assert VehicleManager(vehicle, loaded).mass_kg == pytest.approx(16750.0)


def test_answers_before_any_measurement(manager):
    """A platform has a mass from the instant it exists. Refusing until the
    gauge speaks would put a first-cycle special case in every consumer."""
    est = manager.mass(0.0)
    assert est.fuel_mass_kg == STANDARD.initial_fuel_kg
    assert est.mass_sigma_kg == STANDARD.initial_fuel_sigma_kg


def test_a_measurement_replaces_the_initial_guess(manager):
    manager.ingest(FuelMeasurement(1.0, 3200.0, 20.0))
    est = manager.mass(1.0)
    assert est.fuel_mass_kg == 3200.0
    assert est.mass_kg == pytest.approx(15200.0)


def test_sigma_comes_from_the_measurement(manager):
    """Invariant 4. The manager republishes the uncertainty that arrived with
    the reading; it does not keep a configured one of its own.

    Two readings differing only in declared sigma, because a component using
    its own number would look identical on any single reading.
    """
    manager.ingest(FuelMeasurement(1.0, 3200.0, 20.0))
    assert manager.mass(1.0).mass_sigma_kg == 20.0

    manager.ingest(FuelMeasurement(2.0, 3200.0, 95.0))
    assert manager.mass(2.0).mass_sigma_kg == 95.0


def test_sigma_ignores_the_exact_terms(vehicle):
    """Dry mass and payload are known constants, so neither may inflate the
    published uncertainty."""
    light = VehicleManagerParameters(0.0, 4000.0, 200.0)
    heavy = VehicleManagerParameters(2000.0, 4000.0, 200.0)
    reading = FuelMeasurement(1.0, 3200.0, 20.0)

    for par in (light, heavy):
        m = VehicleManager(vehicle, par)
        m.ingest(reading)
        assert m.mass(1.0).mass_sigma_kg == 20.0


def test_rejects_measurements_it_cannot_use(manager):
    with pytest.raises(TypeError):
        manager.ingest(object())


# --------------------------------------------------------------------------
# Vehicle questions, answered at the believed mass
# --------------------------------------------------------------------------

def test_capability_is_evaluated_at_the_believed_mass(vehicle, manager):
    """The whole point of the component. A fuel reading that moves the
    believed mass must move the reported envelope, or nothing is being bound
    and consumers are back to supplying a mass themselves.

    Stall speed is the channel to check: it scales with sqrt(mass), so a
    heavier belief must raise it.
    """
    est = _estimate()

    manager.ingest(FuelMeasurement(1.0, 1000.0, 20.0))     # 13 000 kg
    light = manager.capability(est)

    manager.ingest(FuelMeasurement(2.0, 6000.0, 20.0))     # 18 000 kg
    heavy = manager.capability(est)

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


def test_project_command_enforces_at_the_believed_mass(vehicle, manager):
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

    manager.ingest(FuelMeasurement(1.0, 1000.0, 20.0))         # 13 000 kg
    light_cmd, light_sat = manager.project_command(est, turn)
    assert not light_sat.omega_clipped
    assert light_cmd.omega_rad_s == pytest.approx(turn.omega_rad_s)

    manager.ingest(FuelMeasurement(2.0, 6000.0, 20.0))         # 18 000 kg
    heavy_cmd, heavy_sat = manager.project_command(est, turn)
    assert heavy_sat.omega_clipped
    assert heavy_cmd.omega_rad_s == pytest.approx(
        vehicle.omega_max_rad_s(150.0, 18000.0)
    )
    # The receipt still carries what was asked for (ADR 0006).
    assert heavy_sat.requested.omega_rad_s == pytest.approx(turn.omega_rad_s)


def test_believed_state_carries_the_believed_mass(manager):
    est = _estimate(v_mps=310.0)
    manager.ingest(FuelMeasurement(1.0, 2500.0, 20.0))
    believed = manager.believed_state(est)

    assert believed.mass_kg == pytest.approx(14500.0)
    assert believed.v_mps == pytest.approx(310.0)
    assert believed.psi_rad == pytest.approx(est.psi_rad)
