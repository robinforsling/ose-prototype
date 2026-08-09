"""
Navigation systems: estimation of the vehicle's own state.

The simulation core owns the true vehicle state and the true forces acting on
it. A navigation system is a resource-layer component that reads truth --
privileged access no other component has -- corrupts it according to a sensor
error model, and publishes an estimate. Everything above the resource layer
consumes only the estimate.

Two implementations are provided behind a common interface:

  AdditiveNoiseNavigation
      Truth plus white noise. No dynamics, no drift, no outage behaviour.
      Cheap, and appropriate when the component under test is something else
      entirely and navigation error must simply be non-zero.

  InsGnssNavigation
      Strapdown inertial mechanisation driven by a modelled IMU, corrected by
      GNSS position and velocity and by air data, through a closed-loop
      error-state Kalman filter. Reproduces the behaviour that matters: error
      growth during GNSS outage, covariance that grows with it, heading error
      observable only under lateral specific force, and wind estimated from the
      disagreement between ground-referenced and air-referenced measurements.

Frame and sign conventions follow vehicle.py: p_x north, p_y east, psi
clockwise from north, omega positive right.

Error state ordering (10 states)
    0,1   delta position          north, east          [m]
    2,3   delta ground velocity   north, east          [m/s]
    4     delta heading                                [rad]
    5,6   delta accelerometer bias, body x and y       [m/s^2]
    7     delta gyro bias                              [rad/s]
    8,9   delta wind              north, east          [m/s]
"""

from __future__ import annotations

import math
from dataclasses import dataclass
import numpy as np

from ose.interfaces import NavigationSystem, OwnStateEstimate
from ose.resource.vehicle import (
    Disturbance,
    Vehicle2D,
    VehicleCommand,
    VehicleState,
)

I_P = slice(0, 2)
I_V = slice(2, 4)
I_PSI = 4
I_BA = slice(5, 7)
I_BG = 7
I_W = slice(8, 10)
N_ERR = 10


# ---------------------------------------------------------------------------
# What a navigation system publishes
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Implementation 1: additive noise
# ---------------------------------------------------------------------------

@dataclass
class AdditiveNoiseParameters:
    position_sigma_m: float = 15.0
    heading_sigma_rad: float = math.radians(0.5)
    airspeed_sigma_mps: float = 1.5


class AdditiveNoiseNavigation:
    """Truth plus zero-mean white noise. No temporal correlation."""

    def __init__(
        self,
        parameters: AdditiveNoiseParameters | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters or AdditiveNoiseParameters()
        self.rng = rng or np.random.default_rng(0)

    def update(self, t_s, dt_s, true_state, true_command, true_disturbance):
        p = self.par
        cov = np.diag(
            [
                p.position_sigma_m**2,
                p.position_sigma_m**2,
                p.heading_sigma_rad**2,
                p.airspeed_sigma_mps**2,
            ]
        )
        psi = true_state.psi_rad + self.rng.normal(0.0, p.heading_sigma_rad)
        v = true_state.v_mps + self.rng.normal(0.0, p.airspeed_sigma_mps)
        wind = np.array([true_disturbance.wind_x_mps, true_disturbance.wind_y_mps])
        return OwnStateEstimate(
            t_s=t_s,
            p_x_m=true_state.p_x_m + self.rng.normal(0.0, p.position_sigma_m),
            p_y_m=true_state.p_y_m + self.rng.normal(0.0, p.position_sigma_m),
            psi_rad=psi,
            v_air_mps=v,
            ground_velocity_mps=v * np.array([math.cos(psi), math.sin(psi)]) + wind,
            wind_estimate_mps=wind,
            covariance=cov,
        )


# ---------------------------------------------------------------------------
# Implementation 2: INS with GNSS and air data
# ---------------------------------------------------------------------------

@dataclass
class ImuParameters:
    """Tactical-grade defaults."""

    accel_noise_density: float = 1.0e-3      # [m/s^2 / sqrt(Hz)]
    accel_bias_sigma: float = 1.0e-3         # steady-state bias sigma [m/s^2]
    accel_bias_tau_s: float = 3600.0
    gyro_noise_density: float = 3.0e-5       # [rad/s / sqrt(Hz)]
    gyro_bias_sigma: float = 5.0e-6          # [rad/s]
    gyro_bias_tau_s: float = 3600.0


@dataclass
class AidingParameters:
    gnss_rate_hz: float = 1.0
    gnss_position_sigma_m: float = 3.0
    gnss_velocity_sigma_mps: float = 0.15
    gnss_velocity_enabled: bool = True
    air_data_rate_hz: float = 10.0
    air_data_sigma_mps: float = 1.0
    wind_sigma_mps: float = 25.0             # prior on wind aloft, deliberately loose
    wind_tau_s: float = 1800.0


@dataclass
class InitialUncertainty:
    position_sigma_m: float = 10.0
    velocity_sigma_mps: float = 1.0
    heading_sigma_rad: float = math.radians(1.0)


class ImuModel:
    """Generates corrupted specific force and angular rate from truth.

    In level flight the body-frame specific force decomposes into a
    longitudinal component vdot and a lateral centripetal component v*omega.
    Gravity is absent because the vertical axis is suppressed.
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

    def true_specific_force(
        self, state: VehicleState, command: VehicleCommand, dist: Disturbance
    ) -> np.ndarray:
        drag = self.vehicle.drag_N(state.v_mps, state.mass_kg, command.omega_rad_s)
        v_dot = (command.thrust_N - drag + dist.force_long_N) / state.mass_kg
        return np.array([v_dot, state.v_mps * command.omega_rad_s])

    def measure(
        self,
        dt_s: float,
        state: VehicleState,
        command: VehicleCommand,
        dist: Disturbance,
    ) -> tuple[np.ndarray, float]:
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

        f_true = self.true_specific_force(state, command, dist)
        f_meas = (
            f_true
            + self.bias_accel
            + self.rng.normal(0.0, p.accel_noise_density / math.sqrt(dt_s), size=2)
        )
        omega_true = command.omega_rad_s + dist.omega_dist_rad_s
        omega_meas = (
            omega_true
            + self.bias_gyro
            + float(self.rng.normal(0.0, p.gyro_noise_density / math.sqrt(dt_s)))
        )
        return f_meas, omega_meas


def rotation(psi: float) -> np.ndarray:
    """Body to navigation frame. Body x forward, y right; nav x north, y east."""
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def d_rotation(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[-s, -c], [c, -s]])


class InsGnssNavigation:
    """Closed-loop error-state Kalman filter over an inertial mechanisation."""

    def __init__(
        self,
        vehicle: Vehicle2D,
        initial_state: VehicleState,
        imu: ImuParameters | None = None,
        aiding: AidingParameters | None = None,
        initial: InitialUncertainty | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.imu_par = imu or ImuParameters()
        self.aid = aiding or AidingParameters()
        self.init = initial or InitialUncertainty()
        self.rng = rng or np.random.default_rng(0)
        self.imu = ImuModel(self.imu_par, self.rng, vehicle)

        # Nominal (mechanised) state, initialised with error.
        self.p = np.array([initial_state.p_x_m, initial_state.p_y_m]) + self.rng.normal(
            0.0, self.init.position_sigma_m, size=2
        )
        self.psi = initial_state.psi_rad + float(
            self.rng.normal(0.0, self.init.heading_sigma_rad)
        )
        self.v_ground = initial_state.v_mps * np.array(
            [math.cos(self.psi), math.sin(self.psi)]
        ) + self.rng.normal(0.0, self.init.velocity_sigma_mps, size=2)
        self.bias_accel = np.zeros(2)
        self.bias_gyro = 0.0
        self.wind = np.zeros(2)

        # Error covariance.
        self.P = np.diag(
            [
                self.init.position_sigma_m**2,
                self.init.position_sigma_m**2,
                self.init.velocity_sigma_mps**2,
                self.init.velocity_sigma_mps**2,
                self.init.heading_sigma_rad**2,
                self.imu_par.accel_bias_sigma**2,
                self.imu_par.accel_bias_sigma**2,
                self.imu_par.gyro_bias_sigma**2,
                self.aid.wind_sigma_mps**2,
                self.aid.wind_sigma_mps**2,
            ]
        )

        self._t_last_gnss = -math.inf
        self._t_last_air = -math.inf
        self.gnss_available = True
        self.innovation_log: list[tuple[float, str, np.ndarray]] = []

    # ---------------- outage control ----------------

    def set_gnss_available(self, available: bool) -> None:
        """Denies or restores GNSS aiding. Nothing else changes."""
        self.gnss_available = available

    # ---------------- prediction ----------------

    def _mechanise(self, dt: float, f_meas: np.ndarray, omega_meas: float) -> None:
        omega_hat = omega_meas - self.bias_gyro
        f_hat = f_meas - self.bias_accel

        # Rotate the specific force using the MIDPOINT attitude over the step.
        #
        # Using the attitude at the start of the step leaves a systematic
        # velocity error of order |f| omega dt^2 / 2 per step whenever the
        # vehicle is turning. At 250 m/s and 15 deg/s that is roughly 0.4 m/s
        # per second of coherent error -- which the filter cannot distinguish
        # from a heading or bias error, and which destroys consistency. This is
        # the planar analogue of the sculling correction in a full strapdown
        # mechanisation, and reduces the error to O(dt^3).
        psi_mid = self.psi + 0.5 * omega_hat * dt
        a_nav = rotation(psi_mid) @ f_hat

        self.p = self.p + self.v_ground * dt + 0.5 * a_nav * dt**2
        self.v_ground = self.v_ground + a_nav * dt
        self.psi = math.remainder(self.psi + omega_hat * dt, 2.0 * math.pi)

        beta_a = math.exp(-dt / self.imu_par.accel_bias_tau_s)
        beta_g = math.exp(-dt / self.imu_par.gyro_bias_tau_s)
        beta_w = math.exp(-dt / self.aid.wind_tau_s)
        self.bias_accel *= beta_a
        self.bias_gyro *= beta_g
        self.wind *= beta_w

    def _propagate_covariance(self, dt: float, f_hat: np.ndarray) -> None:
        F = np.zeros((N_ERR, N_ERR))
        R = rotation(self.psi)

        F[I_P, I_V] = np.eye(2)
        F[I_V, I_PSI] = d_rotation(self.psi) @ f_hat
        F[I_V, I_BA] = -R
        F[I_PSI, I_BG] = -1.0
        F[I_BA, I_BA] = -np.eye(2) / self.imu_par.accel_bias_tau_s
        F[I_BG, I_BG] = -1.0 / self.imu_par.gyro_bias_tau_s
        F[I_W, I_W] = -np.eye(2) / self.aid.wind_tau_s

        Phi = np.eye(N_ERR) + F * dt + 0.5 * (F @ F) * dt**2

        Qc = np.zeros((N_ERR, N_ERR))
        Qc[I_V, I_V] = R @ (self.imu_par.accel_noise_density**2 * np.eye(2)) @ R.T
        Qc[I_PSI, I_PSI] = self.imu_par.gyro_noise_density**2
        Qc[I_BA, I_BA] = (
            2.0 * self.imu_par.accel_bias_sigma**2 / self.imu_par.accel_bias_tau_s
        ) * np.eye(2)
        Qc[I_BG, I_BG] = (
            2.0 * self.imu_par.gyro_bias_sigma**2 / self.imu_par.gyro_bias_tau_s
        )
        Qc[I_W, I_W] = (
            2.0 * self.aid.wind_sigma_mps**2 / self.aid.wind_tau_s
        ) * np.eye(2)

        self.P = Phi @ self.P @ Phi.T + Qc * dt
        self.P = 0.5 * (self.P + self.P.T)

    # ---------------- correction ----------------

    def _apply(self, dx: np.ndarray) -> None:
        """Inject the estimated error into the nominal state and reset."""
        self.p += dx[I_P]
        self.v_ground += dx[I_V]
        self.psi = math.remainder(self.psi + dx[I_PSI], 2.0 * math.pi)
        self.bias_accel += dx[I_BA]
        self.bias_gyro += dx[I_BG]
        self.wind += dx[I_W]

    def _kalman_update(
        self, H: np.ndarray, residual: np.ndarray, R_meas: np.ndarray, tag: str, t: float
    ) -> None:
        S = H @ self.P @ H.T + R_meas
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ residual
        self._apply(dx)
        A = np.eye(N_ERR) - K @ H
        self.P = A @ self.P @ A.T + K @ R_meas @ K.T      # Joseph form
        self.P = 0.5 * (self.P + self.P.T)
        self.innovation_log.append((t, tag, residual.copy()))

    def _gnss_update(self, t: float, true_state: VehicleState, true_dist: Disturbance):
        a = self.aid
        p_true = np.array([true_state.p_x_m, true_state.p_y_m])
        z_p = p_true + self.rng.normal(0.0, a.gnss_position_sigma_m, size=2)
        H = np.zeros((2, N_ERR))
        H[:, I_P] = np.eye(2)
        self._kalman_update(
            H, z_p - self.p, a.gnss_position_sigma_m**2 * np.eye(2), "gnss_pos", t
        )

        if a.gnss_velocity_enabled:
            v_true = true_state.v_mps * np.array(
                [math.cos(true_state.psi_rad), math.sin(true_state.psi_rad)]
            ) + np.array([true_dist.wind_x_mps, true_dist.wind_y_mps])
            z_v = v_true + self.rng.normal(0.0, a.gnss_velocity_sigma_mps, size=2)
            H = np.zeros((2, N_ERR))
            H[:, I_V] = np.eye(2)
            self._kalman_update(
                H,
                z_v - self.v_ground,
                a.gnss_velocity_sigma_mps**2 * np.eye(2),
                "gnss_vel",
                t,
            )

    def _air_data_update(self, t: float, true_state: VehicleState):
        a = self.aid
        z = true_state.v_mps + float(self.rng.normal(0.0, a.air_data_sigma_mps))
        v_air_vec = self.v_ground - self.wind
        speed = float(np.linalg.norm(v_air_vec))
        if speed < 1.0:
            return
        unit = v_air_vec / speed
        H = np.zeros((1, N_ERR))
        H[0, I_V] = unit
        H[0, I_W] = -unit
        self._kalman_update(
            H, np.array([z - speed]), np.array([[a.air_data_sigma_mps**2]]), "air", t
        )

    # ---------------- the interface method ----------------

    def update(self, t_s, dt_s, true_state, true_command, true_disturbance):
        """Correct at t_s, then predict to t_s + dt.

        Ordering matters and is easy to get wrong. On entry the nominal state
        and the supplied truth both refer to t_s, so residuals formed here are
        time aligned. Mechanising first and then forming residuals against
        truth at t_s misaligns them by one step, which during a turn injects a
        systematic velocity residual of v * omega * dt -- some 3 m/s at 250 m/s
        and 15 deg/s, against a measurement noise of 0.15 m/s. The filter has
        no state that can explain such a residual and absorbs it into heading
        and bias, destroying consistency. Straight flight hides the fault
        entirely, because the velocity vector is then not rotating.
        """
        # 1. Corrections, at t_s, against time-aligned truth.
        if self.gnss_available and t_s - self._t_last_gnss >= 1.0 / self.aid.gnss_rate_hz:
            self._gnss_update(t_s, true_state, true_disturbance)
            self._t_last_gnss = t_s

        if t_s - self._t_last_air >= 1.0 / self.aid.air_data_rate_hz:
            self._air_data_update(t_s, true_state)
            self._t_last_air = t_s

        # 2. The estimate published to the platform refers to t_s.
        estimate = self._estimate(t_s)

        # 3. Prediction to t_s + dt, ready for the next call.
        f_meas, omega_meas = self.imu.measure(
            dt_s, true_state, true_command, true_disturbance
        )
        f_hat = f_meas - self.bias_accel
        self._mechanise(dt_s, f_meas, omega_meas)
        self._propagate_covariance(dt_s, f_hat)

        return estimate

    def _estimate(self, t_s: float) -> OwnStateEstimate:
        v_air_vec = self.v_ground - self.wind
        v_air = float(np.linalg.norm(v_air_vec))

        # Map the error covariance onto [p_x, p_y, psi, v_air].
        J = np.zeros((4, N_ERR))
        J[0, 0] = 1.0
        J[1, 1] = 1.0
        J[2, I_PSI] = 1.0
        if v_air > 1.0:
            unit = v_air_vec / v_air
            J[3, I_V] = unit
            J[3, I_W] = -unit
        cov = J @ self.P @ J.T

        return OwnStateEstimate(
            t_s=t_s,
            p_x_m=float(self.p[0]),
            p_y_m=float(self.p[1]),
            psi_rad=self.psi,
            v_air_mps=v_air,
            ground_velocity_mps=self.v_ground.copy(),
            wind_estimate_mps=self.wind.copy(),
            covariance=cov,
            gnss_available=self.gnss_available,
        )
