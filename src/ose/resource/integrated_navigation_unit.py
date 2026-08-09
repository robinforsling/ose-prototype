"""
Black-box integrated navigation unit.

Resource-layer: reads true_state and true_disturbance directly, privileged
access nothing above this layer has. See ADR 0008.

Truth plus zero-mean white noise, no temporal correlation, no drift, no
outage behaviour. Its internals -- IMU, GNSS, filter -- are not simulated;
this component IS the black box. It satisfies OwnStateSource, publishing
vehicle.state.v1 directly, but not NavigationEstimator: there is no
measurement stream to ingest and nothing to replay.

This deliberately collapses the resource and subsystem layers into one
component. It is valid when navigation is not the component under test and
estimation error must simply be present and roughly plausible -- cheap
scaffolding for exercising something else entirely. It is NOT a valid
baseline for any claim about navigation performance; for that, compose
Imu, GnssReceiver and AirDataSensor with InsGnssEstimator instead, where
error growth under outage, covariance consistency and observability
structure are actually modelled rather than assumed away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import OwnStateEstimate
from ose.resource.vehicle import Disturbance, VehicleState


@dataclass
class IntegratedNavParameters:
    """Shape only, no defaults -- an accuracy grade is a choice, not a
    universal, so it belongs in a named reference config
    (resource/reference_configs/reference_integrated_navigation_unit.py),
    not baked in here."""

    position_sigma_m: float
    heading_sigma_rad: float
    airspeed_sigma_mps: float


class IntegratedNavUnit:
    """Truth plus zero-mean white noise. No temporal correlation."""

    def __init__(
        self,
        parameters: IntegratedNavParameters,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters
        self.rng = rng or np.random.default_rng(0)
        self._last: OwnStateEstimate | None = None

    def update(
        self, t_s: float, true_state: VehicleState, true_disturbance: Disturbance
    ) -> OwnStateEstimate:
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
        self._last = OwnStateEstimate(
            t_s=t_s,
            p_x_m=true_state.p_x_m + self.rng.normal(0.0, p.position_sigma_m),
            p_y_m=true_state.p_y_m + self.rng.normal(0.0, p.position_sigma_m),
            psi_rad=psi,
            v_air_mps=v,
            ground_velocity_mps=v * np.array([math.cos(psi), math.sin(psi)]) + wind,
            wind_estimate_mps=wind,
            covariance=cov,
        )
        return self._last

    def estimate(self, t_s: float) -> OwnStateEstimate:
        """Satisfies OwnStateSource. Returns the estimate from the most
        recent update(); does not itself read truth."""
        if self._last is None:
            raise RuntimeError("IntegratedNavUnit.estimate() called before update()")
        return self._last
