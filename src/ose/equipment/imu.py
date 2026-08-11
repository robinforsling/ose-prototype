"""
Inertial sensor model.

Equipment-layer: reads true_state, true_command and true_disturbance directly,
privileged access nothing above this layer has. Corrupts truth according to a
sensor error model and publishes ImuMeasurement, which carries no truth. See
ADR 0008.

In level flight the body-frame specific force decomposes into a longitudinal
component vdot and a lateral centripetal component v*omega. Gravity is absent
because the vertical axis is suppressed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.equipment.vehicle import Disturbance, Vehicle2D, VehicleCommand, VehicleState
from ose.interfaces import ImuMeasurement, MeasurementChannel, SensorCapability


@dataclass
class ImuParameters:
    """Shape only, no defaults -- a sensor grade (tactical, consumer, ...) is
    a choice, not a universal, so it belongs in a named reference config
    (equipment/reference_configs/reference_imu.py), not baked in here."""

    accel_noise_density: float      # [m/s^2 / sqrt(Hz)]
    accel_bias_sigma: float         # steady-state bias sigma [m/s^2]
    accel_bias_tau_s: float
    gyro_noise_density: float       # [rad/s / sqrt(Hz)]
    gyro_bias_sigma: float          # [rad/s]
    gyro_bias_tau_s: float


class Imu:
    """Generates corrupted specific force and angular rate from truth.

    Keeps its own true bias state, drawn once at construction and then
    propagated as a first-order Gauss-Markov process on every sample().
    """

    def __init__(
        self,
        parameters: ImuParameters,
        rng: np.random.Generator,
        vehicle: Vehicle2D,
    ) -> None:
        self.par = parameters
        self.rng = rng
        self.vehicle = vehicle
        # True biases, drawn once per instantiation and then propagated.
        self.bias_accel = rng.normal(0.0, parameters.accel_bias_sigma, size=2)
        self.bias_gyro = float(rng.normal(0.0, parameters.gyro_bias_sigma))

    def capability(self) -> SensorCapability:
        """Two channels with different units, and densities rather than
        per-sample sigmas: this sensor does not own its rate, so its
        per-sample accuracy (density / sqrt(dt)) is undefined until the
        caller picks an interval. Reporting the density is the honest
        invariant; the units say so.
        """
        return SensorCapability(
            rate_hz=None,
            channels=(
                MeasurementChannel(
                    "specific_force", self.par.accel_noise_density, "m/s^2/sqrt(Hz)"
                ),
                MeasurementChannel(
                    "angular_rate", self.par.gyro_noise_density, "rad/s/sqrt(Hz)"
                ),
            ),
            available=True,
        )

    def true_specific_force(
        self, state: VehicleState, command: VehicleCommand, dist: Disturbance
    ) -> np.ndarray:
        drag = self.vehicle.drag_N(state.v_mps, state.mass_kg, command.omega_rad_s)
        v_dot = (command.thrust_N - drag + dist.force_long_N) / state.mass_kg
        return np.array([v_dot, state.v_mps * command.omega_rad_s])

    def sample(
        self,
        t_s: float,
        dt_s: float,
        true_state: VehicleState,
        true_command: VehicleCommand,
        true_disturbance: Disturbance,
    ) -> ImuMeasurement:
        p = self.par

        # Propagate the true biases as first-order Gauss-Markov processes.
        for tau, sigma, attr in (
            (p.accel_bias_tau_s, p.accel_bias_sigma, "accel"),
            (p.gyro_bias_tau_s, p.gyro_bias_sigma, "gyro"),
        ):
            beta = math.exp(-dt_s / tau)
            q = sigma * math.sqrt(max(1.0 - beta**2, 0.0))
            if attr == "accel":
                self.bias_accel = beta * self.bias_accel + self.rng.normal(0, q, size=2)
            else:
                self.bias_gyro = beta * self.bias_gyro + float(self.rng.normal(0, q))

        f_true = self.true_specific_force(true_state, true_command, true_disturbance)
        specific_force_sigma = p.accel_noise_density / math.sqrt(dt_s)
        f_meas = (
            f_true
            + self.bias_accel
            + self.rng.normal(0.0, specific_force_sigma, size=2)
        )

        omega_true = true_command.omega_rad_s + true_disturbance.omega_dist_rad_s
        angular_rate_sigma = p.gyro_noise_density / math.sqrt(dt_s)
        omega_meas = (
            omega_true
            + self.bias_gyro
            + float(self.rng.normal(0.0, angular_rate_sigma))
        )

        return ImuMeasurement(
            valid_time_s=t_s,
            interval_s=dt_s,
            specific_force_body_mps2=f_meas,
            angular_rate_rad_s=omega_meas,
            specific_force_sigma_mps2=specific_force_sigma,
            angular_rate_sigma_rad_s=angular_rate_sigma,
        )
