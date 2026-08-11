"""
Clock model: a platform's own internal timekeeping (e.g. a crystal or atomic
oscillator).

Equipment-layer: reads true elapsed time directly, privileged access nothing
above this layer has. True time is ground truth owned by the simulation core
(see CLAUDE.md's layer table), no different in kind from the true vehicle
state read by other equipment-layer sensors. Publishes ClockMeasurement,
which carries no truth -- see its docstring for why that record has no true
interval field, unlike ImuMeasurement.

Two error sources, matching the request this component was built against:
a drift term and an additive white-noise term.

  * Drift is a fractional frequency offset (dimensionless, "seconds of
    error per second of true time") modelled as a first-order Gauss-Markov
    process, exactly like Imu's accelerometer/gyro bias -- drawn once at
    construction, then propagated on every sample(). It is the clock's own
    true behaviour and is never declared in the published measurement,
    matching ADR 0009: a future time estimator's assumed model of this
    drift is its own prior, not something read off the sensor.
  * The white-noise term is a fixed per-reading timing jitter -- read-out
    noise on the clock's counter -- not scaled by dt_s the way IMU's noise
    density is. A clock reading's jitter is a property of the read-out
    mechanism, not a continuous-time process sampled at rate 1/dt.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import ClockMeasurement, MeasurementChannel, SensorCapability


@dataclass
class ClockParameters:
    """Shape only, no defaults -- an oscillator grade (crystal, disciplined,
    laboratory, ...) is a choice, not a universal, so it belongs in a named
    reference config (equipment/reference_configs/reference_clock.py), not
    baked in here."""

    drift_sigma: float          # steady-state fractional frequency offset [s/s]
    drift_tau_s: float          # correlation time of the drift
    white_noise_sigma_s: float  # per-reading timing jitter [s]


class Clock:
    """Corrupts true elapsed time with drift plus white noise.

    Keeps its own true drift state, drawn once at construction and then
    propagated as a first-order Gauss-Markov process on every sample() --
    the same pattern as Imu's accelerometer/gyro bias.
    """

    def __init__(
        self,
        parameters: ClockParameters,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters
        self.rng = rng or np.random.default_rng(0)
        self.drift = float(self.rng.normal(0.0, self.par.drift_sigma))

    def capability(self) -> SensorCapability:
        """Like Imu, does not own its rate. Unlike Imu, its white-noise term
        is a fixed per-reading jitter rather than a density, so the sigma
        reported here is directly the one its measurements declare -- the
        drift term is deliberately not reported, being the clock's own true
        behaviour rather than something it declares (ADR 0010).
        """
        return SensorCapability(
            rate_hz=None,
            channels=(
                MeasurementChannel("elapsed", self.par.white_noise_sigma_s, "s"),
            ),
            available=True,
        )

    def sample(self, t_s: float, dt_s: float) -> ClockMeasurement:
        p = self.par

        beta = math.exp(-dt_s / p.drift_tau_s)
        q = p.drift_sigma * math.sqrt(max(1.0 - beta**2, 0.0))
        self.drift = beta * self.drift + float(self.rng.normal(0.0, q))

        elapsed = dt_s * (1.0 + self.drift) + float(
            self.rng.normal(0.0, p.white_noise_sigma_s)
        )

        return ClockMeasurement(
            valid_time_s=t_s,
            elapsed_s=elapsed,
            elapsed_sigma_s=p.white_noise_sigma_s,
        )
