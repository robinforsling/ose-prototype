"""
Navigation estimator: a closed-loop error-state Kalman filter over a strapdown
inertial mechanisation, corrected by GNSS position and velocity and by air
data.

Subsystem-layer: purely cyber. This module must not import anything from
ose.equipment, and no public method may take a parameter whose name begins
with true_ -- see test_estimator_cannot_see_truth in
tests/test_navigation_state_estimator.py, which checks both by parsing this
file with ast. The estimator is a pure function of the measurement stream it is
fed through ingest(); see test_replay_determinism for what that buys.

Reproduces the behaviour that matters: error growth during GNSS outage,
covariance that grows with it, heading error observable only under lateral
specific force, and wind estimated from the disagreement between
ground-referenced and air-referenced measurements.

Frame and sign conventions follow ose.frames: p_x north, p_y east, psi
clockwise from north, omega positive right.

Error state ordering (10 states)
    0,1   delta position          north, east          [m]
    2,3   delta ground velocity   north, east          [m/s]
    4     delta heading                                [rad]
    5,6   delta accelerometer bias, body x and y       [m/s^2]
    7     delta gyro bias                              [rad/s]
    8,9   delta wind              north, east          [m/s]

Two things this filter deliberately does NOT get from any sensor's declared
accuracy:

  * The IMU bias's assumed Gauss-Markov behaviour (accel_bias_sigma_mps2,
    accel_bias_tau_s, gyro_bias_sigma_rad_s, gyro_bias_tau_s) and the wind
    process model (wind_sigma_mps, wind_tau_s) are the filter's own priors,
    not a property of any measurement. A real IMU's bias need not actually
    behave this way; the filter is tuned assuming it roughly does.
  * The white-noise part of the IMU's process noise IS taken from the
    incoming ImuMeasurement's declared specific_force_sigma_mps2 and
    angular_rate_sigma_rad_s, exactly the "estimator believes the
    declaration" property the split exists to establish.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar

import numpy as np

from ose.frames import d_rotation, rotation
from ose.interfaces import AirDataMeasurement, GnssFix, ImuMeasurement, OwnStateEstimate

I_P = slice(0, 2)
I_V = slice(2, 4)
I_PSI = 4
I_BA = slice(5, 7)
I_BG = 7
I_W = slice(8, 10)
N_ERR = 10


@dataclass
class EstimatorParameters:
    """The filter's own process-noise model: its prior beliefs about how the
    IMU bias and the wind aloft evolve. Independent of any sensor's declared
    accuracy -- see the module docstring."""

    accel_bias_sigma_mps2: float = 1.0e-3
    accel_bias_tau_s: float = 3600.0
    gyro_bias_sigma_rad_s: float = 5.0e-6
    gyro_bias_tau_s: float = 3600.0
    wind_sigma_mps: float = 25.0             # prior on wind aloft, deliberately loose
    wind_tau_s: float = 1800.0
    gnss_timeout_s: float = 5.0              # aiding considered lost after this long


@dataclass
class InitialUncertainty:
    position_sigma_m: float = 10.0
    velocity_sigma_mps: float = 1.0
    heading_sigma_rad: float = math.radians(1.0)


class InsGnssEstimator:
    """Closed-loop error-state Kalman filter over an inertial mechanisation.

    Constructed with a numeric initial guess -- not a true state. Producing
    that guess (typically truth corrupted by some unmodelled alignment
    process) is the caller's responsibility; nothing in this class ever sees
    truth.

    It publishes on vehicle.state_source.v1, not vehicle.state.v1. The record
    is the same OwnStateEstimate either way; the PORT is not. A source
    estimate is one input to the platform's navigation, and the platform's
    published state is what NavigationManager says it is -- one publisher per
    platform, ADR 0014. Binding a consumer straight to this estimator would
    work and would be wrong, and until the ports were named apart nothing
    could tell. See ADR 0021 and ADR 0022.
    """

    PUBLISHES: ClassVar[dict[str, str]] = {"estimate": "vehicle.state_source.v1"}

    def __init__(
        self,
        initial_p_m,
        initial_psi_rad: float,
        initial_v_ground_mps,
        parameters: EstimatorParameters | None = None,
        initial_uncertainty: InitialUncertainty | None = None,
        t0_s: float = 0.0,
    ) -> None:
        self.par = parameters or EstimatorParameters()
        self.init = initial_uncertainty or InitialUncertainty()

        self.p = np.array(initial_p_m, dtype=float)
        self.psi = float(initial_psi_rad)
        self.v_ground = np.array(initial_v_ground_mps, dtype=float)
        self.bias_accel = np.zeros(2)
        self.bias_gyro = 0.0
        self.wind = np.zeros(2)

        self.P = np.diag(
            [
                self.init.position_sigma_m**2,
                self.init.position_sigma_m**2,
                self.init.velocity_sigma_mps**2,
                self.init.velocity_sigma_mps**2,
                self.init.heading_sigma_rad**2,
                self.par.accel_bias_sigma_mps2**2,
                self.par.accel_bias_sigma_mps2**2,
                self.par.gyro_bias_sigma_rad_s**2,
                self.par.wind_sigma_mps**2,
                self.par.wind_sigma_mps**2,
            ]
        )

        self._t = t0_s
        self._last_ingest_time = -math.inf
        self._t_last_gnss_fix = t0_s
        self.innovation_log: list[tuple[float, str, np.ndarray]] = []

    # ---------------- ingestion ----------------

    def ingest(self, measurement) -> None:
        """Dispatches on measurement type. ImuMeasurement drives prediction;
        GnssFix and AirDataMeasurement drive correction. Measurements must
        arrive in non-decreasing valid_time_s order."""
        if isinstance(measurement, ImuMeasurement):
            self._check_order(measurement.valid_time_s)
            self._ingest_imu(measurement)
        elif isinstance(measurement, GnssFix):
            self._check_order(measurement.valid_time_s)
            self._ingest_gnss(measurement)
        elif isinstance(measurement, AirDataMeasurement):
            self._check_order(measurement.valid_time_s)
            self._ingest_air(measurement)
        else:
            raise TypeError(
                f"InsGnssEstimator cannot ingest {type(measurement).__name__}"
            )

    def _check_order(self, valid_time_s: float) -> None:
        if valid_time_s < self._last_ingest_time:
            raise ValueError(
                f"measurement at t={valid_time_s} arrived after t="
                f"{self._last_ingest_time}; measurements must be ingested in "
                "non-decreasing valid_time_s order"
            )
        self._last_ingest_time = valid_time_s

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

        beta_a = math.exp(-dt / self.par.accel_bias_tau_s)
        beta_g = math.exp(-dt / self.par.gyro_bias_tau_s)
        beta_w = math.exp(-dt / self.par.wind_tau_s)
        self.bias_accel *= beta_a
        self.bias_gyro *= beta_g
        self.wind *= beta_w

    def _propagate_covariance(
        self,
        dt: float,
        f_hat: np.ndarray,
        specific_force_sigma_mps2: float,
        angular_rate_sigma_rad_s: float,
    ) -> None:
        F = np.zeros((N_ERR, N_ERR))
        R = rotation(self.psi)

        F[I_P, I_V] = np.eye(2)
        F[I_V, I_PSI] = d_rotation(self.psi) @ f_hat
        F[I_V, I_BA] = -R
        F[I_PSI, I_BG] = -1.0
        F[I_BA, I_BA] = -np.eye(2) / self.par.accel_bias_tau_s
        F[I_BG, I_BG] = -1.0 / self.par.gyro_bias_tau_s
        F[I_W, I_W] = -np.eye(2) / self.par.wind_tau_s

        Phi = np.eye(N_ERR) + F * dt + 0.5 * (F @ F) * dt**2

        # The white-noise block is driven by the measurement's OWN declared
        # sigma, not a separately configured density. specific_force_sigma is
        # the per-sample stddev (density / sqrt(dt)); squaring and scaling by
        # dt recovers density**2, which is what belongs in a continuous-time
        # spectral density matrix.
        Qc = np.zeros((N_ERR, N_ERR))
        Qc[I_V, I_V] = dt * R @ (specific_force_sigma_mps2**2 * np.eye(2)) @ R.T
        Qc[I_PSI, I_PSI] = dt * angular_rate_sigma_rad_s**2
        Qc[I_BA, I_BA] = (
            2.0 * self.par.accel_bias_sigma_mps2**2 / self.par.accel_bias_tau_s
        ) * np.eye(2)
        Qc[I_BG, I_BG] = (
            2.0 * self.par.gyro_bias_sigma_rad_s**2 / self.par.gyro_bias_tau_s
        )
        Qc[I_W, I_W] = (
            2.0 * self.par.wind_sigma_mps**2 / self.par.wind_tau_s
        ) * np.eye(2)

        self.P = Phi @ self.P @ Phi.T + Qc * dt
        self.P = 0.5 * (self.P + self.P.T)

    def _ingest_imu(self, m: ImuMeasurement) -> None:
        f_hat = m.specific_force_body_mps2 - self.bias_accel
        self._mechanise(m.interval_s, m.specific_force_body_mps2, m.angular_rate_rad_s)
        self._propagate_covariance(
            m.interval_s, f_hat, m.specific_force_sigma_mps2, m.angular_rate_sigma_rad_s
        )
        self._t = m.valid_time_s + m.interval_s

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

    def _ingest_gnss(self, fix: GnssFix) -> None:
        H = np.zeros((2, N_ERR))
        H[:, I_P] = np.eye(2)
        self._kalman_update(
            H, fix.position_m - self.p, fix.position_sigma_m**2 * np.eye(2),
            "gnss_pos", fix.valid_time_s,
        )
        self._t_last_gnss_fix = fix.valid_time_s

        if fix.velocity_mps is not None:
            H = np.zeros((2, N_ERR))
            H[:, I_V] = np.eye(2)
            self._kalman_update(
                H, fix.velocity_mps - self.v_ground,
                fix.velocity_sigma_mps**2 * np.eye(2), "gnss_vel", fix.valid_time_s,
            )

    def _ingest_air(self, m: AirDataMeasurement) -> None:
        v_air_vec = self.v_ground - self.wind
        speed = float(np.linalg.norm(v_air_vec))
        if speed < 1.0:
            return
        unit = v_air_vec / speed
        H = np.zeros((1, N_ERR))
        H[0, I_V] = unit
        H[0, I_W] = -unit
        self._kalman_update(
            H, np.array([m.airspeed_mps - speed]),
            np.array([[m.airspeed_sigma_mps**2]]), "air", m.valid_time_s,
        )

    # ---------------- publication ----------------

    def estimate(self, t_s: float) -> OwnStateEstimate:
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
            # Straight out of the error covariance, no projection: the filter
            # already carries ground velocity as an error state, so a track
            # claim needs nothing computed here that was not computed anyway.
            ground_velocity_covariance=self.P[I_V, I_V].copy(),
            gnss_available=(t_s - self._t_last_gnss_fix) < self.par.gnss_timeout_s,
        )
