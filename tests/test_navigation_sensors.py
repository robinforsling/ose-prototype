"""Tests for the resource-layer navigation sensors.

Each sensor declares its own accuracy; these tests check that the declared
sigma is honest -- the property the whole split exists to make checkable, per
ADR 0009 and the testing philosophy in CLAUDE.md.
"""

import dataclasses
import math

import numpy as np
import pytest

from ose import interfaces
from ose.resource.air_data import AirDataSensor as AirDataSensorImpl
from ose.resource.gnss import GnssReceiver
from ose.resource.imu import Imu
from ose.resource.reference_configs.reference_air_data import STANDARD as AIR_DATA_STANDARD
from ose.resource.reference_configs.reference_gnss import STANDARD as GNSS_STANDARD
from ose.resource.reference_configs.reference_imu import TACTICAL_GRADE
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import Disturbance, VehicleCommand, VehicleState


@pytest.fixture
def vehicle():
    return reference_fighter()


def _scenario():
    """A fixed, non-trivial flight condition: turning, with a longitudinal gust."""
    vehicle = reference_fighter()
    state = VehicleState(0.0, 0.0, math.radians(15.0), 250.0, 16000.0)
    omega = 0.05
    thrust = vehicle.thrust_required_N(state.v_mps, state.mass_kg, omega)
    command = VehicleCommand(thrust, omega)
    dist = Disturbance(wind_x_mps=12.0, wind_y_mps=-18.0, force_long_N=120.0)
    return vehicle, state, command, dist


# --------------------------------------------------------------------------
# Protocol conformance
# --------------------------------------------------------------------------

def test_sensors_satisfy_their_protocols(vehicle):
    imu = Imu(TACTICAL_GRADE, np.random.default_rng(0), vehicle)
    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0))
    air = AirDataSensorImpl(AIR_DATA_STANDARD, rng=np.random.default_rng(0))
    assert isinstance(imu, interfaces.InertialSensor)
    assert isinstance(gnss, interfaces.PositioningSensor)
    assert isinstance(air, interfaces.AirDataSensor)


# --------------------------------------------------------------------------
# valid_time_s
# --------------------------------------------------------------------------

def test_valid_time_equals_time_requested(vehicle):
    _, state, command, dist = _scenario()
    imu = Imu(TACTICAL_GRADE, np.random.default_rng(0), vehicle)
    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0))
    air = AirDataSensorImpl(AIR_DATA_STANDARD, rng=np.random.default_rng(0))

    assert imu.sample(12.5, 0.02, state, command, dist).valid_time_s == 12.5
    assert gnss.sample(12.5, state, dist).valid_time_s == 12.5
    assert air.sample(12.5, state).valid_time_s == 12.5


# --------------------------------------------------------------------------
# IMU
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def imu_draws():
    """Many draws at a fixed flight condition, holding truth constant."""
    vehicle, state, command, dist = _scenario()
    par = TACTICAL_GRADE
    rng = np.random.default_rng(1)
    imu = Imu(par, rng, vehicle)
    f_true = imu.true_specific_force(state, command, dist)

    dt = 0.05
    n = 6000
    f_residual = np.empty((n, 2))
    omega_residual = np.empty(n)
    for i in range(n):
        m = imu.sample(i * dt, dt, state, command, dist)
        f_residual[i] = m.specific_force_body_mps2 - f_true - imu.bias_accel
        omega_residual[i] = m.angular_rate_rad_s - command.omega_rad_s - imu.bias_gyro
    return par, dt, f_residual, omega_residual


def test_imu_mean_matches_true_specific_force_plus_bias(imu_draws):
    par, dt, f_residual, omega_residual = imu_draws
    n = f_residual.shape[0]
    se_f = par.accel_noise_density / math.sqrt(dt) / math.sqrt(n)
    se_omega = par.gyro_noise_density / math.sqrt(dt) / math.sqrt(n)
    assert np.all(np.abs(f_residual.mean(axis=0)) < 5.0 * se_f)
    assert abs(omega_residual.mean()) < 5.0 * se_omega


def test_imu_std_matches_declared_sigma(imu_draws):
    par, dt, f_residual, omega_residual = imu_draws
    declared_f = par.accel_noise_density / math.sqrt(dt)
    declared_omega = par.gyro_noise_density / math.sqrt(dt)
    assert np.all(np.abs(f_residual.std(axis=0) - declared_f) < 0.1 * declared_f)
    assert abs(omega_residual.std() - declared_omega) < 0.1 * declared_omega


def test_imu_bias_reaches_gauss_markov_steady_state(vehicle):
    par = TACTICAL_GRADE
    rng = np.random.default_rng(3)
    imu = Imu(par, rng, vehicle)
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    command = VehicleCommand(vehicle.thrust_required_N(state.v_mps, state.mass_kg, 0.0), 0.0)
    dist = Disturbance()

    dt = 20.0
    n_steps = 20000                     # T = 400,000 s, ~111 time constants
    bias = np.empty(n_steps)
    t = 0.0
    for i in range(n_steps):
        imu.sample(t, dt, state, command, dist)
        bias[i] = imu.bias_gyro
        t += dt

    steady = bias[n_steps // 10 :]      # discard the initial transient
    ratio = np.var(steady) / par.gyro_bias_sigma**2
    assert 0.5 < ratio < 2.0


# --------------------------------------------------------------------------
# GNSS
# --------------------------------------------------------------------------

def test_gnss_returns_none_while_denied_and_fix_once_restored():
    gnss = GnssReceiver(GNSS_STANDARD, rng=np.random.default_rng(0))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    dist = Disturbance()

    assert gnss.sample(0.0, state, dist) is not None

    gnss.set_gnss_available(False)
    assert gnss.sample(1.0, state, dist) is None

    gnss.set_gnss_available(True)
    assert gnss.sample(2.0, state, dist) is not None


def test_gnss_fix_declares_configured_sigma():
    par = dataclasses.replace(GNSS_STANDARD, gnss_position_sigma_m=7.0, gnss_velocity_sigma_mps=0.3)
    gnss = GnssReceiver(par, rng=np.random.default_rng(0))
    fix = gnss.sample(0.0, VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0), Disturbance())
    assert fix.position_sigma_m == 7.0
    assert fix.velocity_sigma_mps == 0.3


def test_gnss_position_noise_std_matches_declared_sigma():
    par = GNSS_STANDARD
    gnss = GnssReceiver(par, rng=np.random.default_rng(4))
    state = VehicleState(1000.0, -500.0, math.radians(15.0), 250.0, 16000.0)
    dist = Disturbance(wind_x_mps=12.0, wind_y_mps=-18.0)
    true_p = np.array([state.p_x_m, state.p_y_m])

    n = 4000
    residual = np.empty((n, 2))
    for i in range(n):
        fix = gnss.sample(float(i), state, dist)
        residual[i] = fix.position_m - true_p

    assert np.all(np.abs(residual.std(axis=0) - par.gnss_position_sigma_m) < 0.1 * par.gnss_position_sigma_m)


def test_gnss_velocity_disabled_omits_velocity():
    par = dataclasses.replace(GNSS_STANDARD, gnss_velocity_enabled=False)
    gnss = GnssReceiver(par, rng=np.random.default_rng(0))
    fix = gnss.sample(0.0, VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0), Disturbance())
    assert fix.velocity_mps is None
    assert fix.velocity_sigma_mps is None


# --------------------------------------------------------------------------
# Air data
# --------------------------------------------------------------------------

def test_air_data_declares_configured_sigma():
    par = dataclasses.replace(AIR_DATA_STANDARD, air_data_sigma_mps=2.0)
    air = AirDataSensorImpl(par, rng=np.random.default_rng(0))
    m = air.sample(0.0, VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0))
    assert m.airspeed_sigma_mps == 2.0


def test_air_data_noise_std_matches_declared_sigma():
    par = AIR_DATA_STANDARD
    air = AirDataSensorImpl(par, rng=np.random.default_rng(5))
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)

    n = 4000
    residual = np.empty(n)
    for i in range(n):
        m = air.sample(float(i), state)
        residual[i] = m.airspeed_mps - state.v_mps

    assert abs(residual.std() - par.air_data_sigma_mps) < 0.1 * par.air_data_sigma_mps
