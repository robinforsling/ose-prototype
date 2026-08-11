"""Tests for the equipment-layer fuel gauge.

Mirrors the pattern established for AirDataSensor: declared
fuel_remaining_sigma_kg is honest -- sample mean and standard deviation
against many draws.
"""

import dataclasses

import numpy as np
import pytest

from ose import interfaces
from ose.equipment.fuel_gauge import FuelGauge
from ose.equipment.reference_configs.reference_fuel_gauge import STANDARD
from ose.equipment.reference_configs.reference_vehicle import reference_fighter
from ose.equipment.vehicle import VehicleState


@pytest.fixture
def mass_dry_kg():
    return reference_fighter().lam.mass_dry_kg


def test_satisfies_fuel_sensor_protocol(mass_dry_kg):
    gauge = FuelGauge(STANDARD, mass_dry_kg, rng=np.random.default_rng(0))
    assert isinstance(gauge, interfaces.FuelSensor)


def test_valid_time_equals_time_requested(mass_dry_kg):
    gauge = FuelGauge(STANDARD, mass_dry_kg, rng=np.random.default_rng(0))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, mass_dry_kg + 4000.0)
    m = gauge.sample(12.5, state)
    assert m.valid_time_s == 12.5


def test_reading_centres_on_true_remaining_fuel(mass_dry_kg):
    par = dataclasses.replace(STANDARD, fuel_sigma_kg=10.0)
    gauge = FuelGauge(par, mass_dry_kg, rng=np.random.default_rng(0))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, mass_dry_kg + 4000.0)
    m = gauge.sample(0.0, state)
    assert m.fuel_remaining_sigma_kg == 10.0
    assert abs(m.fuel_remaining_kg - 4000.0) < 100.0    # a handful of sigma


def test_fuel_noise_std_matches_declared_sigma(mass_dry_kg):
    par = STANDARD
    gauge = FuelGauge(par, mass_dry_kg, rng=np.random.default_rng(3))
    true_fuel_kg = 2500.0
    state = VehicleState(0.0, 0.0, 0.0, 250.0, mass_dry_kg + true_fuel_kg)

    n = 4000
    residual = np.empty(n)
    for i in range(n):
        m = gauge.sample(float(i), state)
        residual[i] = m.fuel_remaining_kg - true_fuel_kg

    assert abs(residual.mean()) < 5.0 * par.fuel_sigma_kg / np.sqrt(n)
    assert abs(residual.std() - par.fuel_sigma_kg) < 0.1 * par.fuel_sigma_kg


def test_due_respects_configured_rate(mass_dry_kg):
    par = dataclasses.replace(STANDARD, fuel_rate_hz=1.0)
    gauge = FuelGauge(par, mass_dry_kg, rng=np.random.default_rng(0))

    assert gauge.due(0.0)
    gauge.sample(0.0, VehicleState(0.0, 0.0, 0.0, 250.0, mass_dry_kg + 4000.0))
    assert not gauge.due(0.5)
    assert gauge.due(1.0)
