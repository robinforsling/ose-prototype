"""
Time estimator: tracks a platform clock's offset and drift from a stream of
ClockMeasurement readings.

Subsystem-layer: purely cyber, same rule as navigation_state_estimator.py --
this module must not import anything from ose.resource, and no public
method may take a parameter whose name begins with true_. See
test_estimator_cannot_see_truth in tests/test_time_estimator.py.

Dead reckoning only, deliberately. A Kalman filter needs something to
correct against; InsGnssEstimator has GNSS and air data to correct the IMU.
Nothing here corrects the clock yet -- there is no second, independent time
reference in this simulation (a second clock, a time-sync message) to check
it against, and faking one would be dishonest. So TimeEstimator only ever
predicts: it accumulates the clock's own (corrupted) readings into
platform_time_s, unfiltered, and propagates a covariance that honestly grows
over time using its own assumed drift process model. This is exactly what
InsGnssEstimator does during a GNSS outage, except here it is permanent.

ingest() dispatches on measurement type -- today only ClockMeasurement -- so
a future correction source can be added as a new isinstance branch without
changing the protocol, the same extensibility argument made for
NavigationEstimator.

State (2 states)
    0   offset       accumulated timing error, platform minus true   [s]
    1   drift_rate   fractional frequency offset                     [s/s]

Both states are always propagated with zero-mean process noise, so their
mean stays at 0 for as long as nothing corrects them -- offset and drift_rate
only ever move once a correcting measurement exists. What is real today is
the covariance: it is where a future correction would be injected via a
Kalman update, and it is the actual product of this component in the
meantime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import ClockMeasurement, TimeEstimate

I_OFFSET = 0
I_DRIFT = 1
N_ERR = 2


@dataclass
class TimeEstimatorParameters:
    """The filter's own assumed drift process model -- its prior belief
    about how the platform clock's drift behaves, decoupled from whatever
    the true Clock resource actually does. Same split as EstimatorParameters
    vs ImuParameters in navigation_state_estimator.py / ADR 0009."""

    drift_sigma: float = 1.0e-9        # assumed steady-state fractional frequency offset [s/s]
    drift_tau_s: float = 3600.0        # assumed correlation time of the drift
    initial_offset_sigma_s: float = 0.0  # assume the clock was just set at construction


class TimeEstimator:
    """Dead-reckoning two-state (offset, drift) filter over a ClockMeasurement
    stream. See the module docstring for why there is no correction step."""

    def __init__(
        self,
        parameters: TimeEstimatorParameters | None = None,
        t0_s: float = 0.0,
    ) -> None:
        self.par = parameters or TimeEstimatorParameters()

        self.offset_s = 0.0
        self.drift_rate = 0.0
        self.platform_time_s = 0.0

        self.P = np.diag(
            [self.par.initial_offset_sigma_s**2, self.par.drift_sigma**2]
        )

        self._t = t0_s
        self._last_ingest_time = -math.inf

    # ---------------- ingestion ----------------

    def ingest(self, measurement) -> None:
        """Dispatches on measurement type. Measurements must arrive in
        non-decreasing valid_time_s order."""
        if isinstance(measurement, ClockMeasurement):
            self._check_order(measurement.valid_time_s)
            self._ingest_clock(measurement)
        else:
            raise TypeError(
                f"TimeEstimator cannot ingest {type(measurement).__name__}"
            )

    def _check_order(self, valid_time_s: float) -> None:
        if valid_time_s < self._last_ingest_time:
            raise ValueError(
                f"measurement at t={valid_time_s} arrived after t="
                f"{self._last_ingest_time}; measurements must be ingested in "
                "non-decreasing valid_time_s order"
            )
        self._last_ingest_time = valid_time_s

    def _ingest_clock(self, m: ClockMeasurement) -> None:
        # The platform's own reading is its best available estimate of how
        # much time just passed -- there is no true dt to fall back on.
        dt = m.elapsed_s

        F = np.array([[0.0, 1.0], [0.0, -1.0 / self.par.drift_tau_s]])
        Phi = np.eye(N_ERR) + F * dt + 0.5 * (F @ F) * dt**2

        x = Phi @ np.array([self.offset_s, self.drift_rate])
        self.offset_s, self.drift_rate = float(x[0]), float(x[1])

        # Drift's continuous-time Gauss-Markov process noise, exactly the
        # nav-filter formula. The white-noise term is different: it is a
        # fixed per-reading sigma travelling with the measurement (declared,
        # per ADR 0009), not a density -- so it injects its variance
        # directly into P[offset, offset], once per reading, rather than
        # through the *dt scaling that a continuous-time density needs.
        Qc = np.zeros((N_ERR, N_ERR))
        Qc[I_DRIFT, I_DRIFT] = 2.0 * self.par.drift_sigma**2 / self.par.drift_tau_s

        self.P = Phi @ self.P @ Phi.T + Qc * dt
        self.P[I_OFFSET, I_OFFSET] += m.elapsed_sigma_s**2
        self.P = 0.5 * (self.P + self.P.T)

        self.platform_time_s += m.elapsed_s
        self._t = m.valid_time_s

    # ---------------- publication ----------------

    def estimate(self, t_s: float) -> TimeEstimate:
        return TimeEstimate(
            t_s=t_s,
            platform_time_s=self.platform_time_s,
            drift_rate=self.drift_rate,
            covariance=self.P.copy(),
        )
