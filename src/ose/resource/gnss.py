"""
GNSS receiver model.

Resource-layer: reads true_state and true_disturbance directly, privileged
access nothing above this layer has. Publishes GnssFix, which carries no
truth. See ADR 0008.

Rate-limiting is the caller's responsibility -- sample() is expected to be
called only when due, per the ordering contract in ADR 0009. due() is
offered as a convenience so the rate parameter does not have to be reached
for from outside. Denial is a
property of the receiver itself: set_gnss_available controls whether sample()
returns a fix or None, independent of timing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import GnssFix, MeasurementChannel, SensorCapability
from ose.resource.vehicle import Disturbance, VehicleState


@dataclass
class GnssParameters:
    """Shape only, no defaults -- a receiver grade is a choice, not a
    universal, so it belongs in a named reference config
    (resource/reference_configs/reference_gnss.py), not baked in here."""

    gnss_rate_hz: float
    gnss_position_sigma_m: float
    gnss_velocity_sigma_mps: float
    gnss_velocity_enabled: bool


class GnssReceiver:
    """Synthesises position (and optionally velocity) fixes from truth."""

    def __init__(
        self,
        parameters: GnssParameters,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters
        self.rng = rng or np.random.default_rng(0)
        self._t_last = -math.inf
        self._available = True
        self.n_fixes = 0

    def capability(self) -> SensorCapability:
        """available tracks denial, so a consumer that asks during an outage
        is told the receiver cannot currently deliver -- the one resource
        here whose capability is genuinely dynamic.

        Two channels, and the velocity one is absent entirely when velocity
        aiding is disabled: a receiver that will not publish velocity should
        not claim an accuracy for it.
        """
        channels = [
            MeasurementChannel("position", self.par.gnss_position_sigma_m, "m")
        ]
        if self.par.gnss_velocity_enabled:
            channels.append(
                MeasurementChannel("velocity", self.par.gnss_velocity_sigma_mps, "m/s")
            )
        return SensorCapability(
            rate_hz=self.par.gnss_rate_hz,
            channels=tuple(channels),
            available=self._available,
        )

    def due(self, t_s: float) -> bool:
        return t_s - self._t_last >= 1.0 / self.par.gnss_rate_hz

    def set_gnss_available(self, available: bool) -> None:
        """Denies or restores GNSS aiding. Nothing else changes."""
        self._available = available

    @property
    def available(self) -> bool:
        return self._available

    def sample(
        self, t_s: float, true_state: VehicleState, true_disturbance: Disturbance
    ) -> GnssFix | None:
        self._t_last = t_s
        if not self._available:
            return None

        p = self.par
        p_true = np.array([true_state.p_x_m, true_state.p_y_m])
        position = p_true + self.rng.normal(0.0, p.gnss_position_sigma_m, size=2)

        velocity = None
        velocity_sigma = None
        if p.gnss_velocity_enabled:
            v_true = true_state.v_mps * np.array(
                [math.cos(true_state.psi_rad), math.sin(true_state.psi_rad)]
            ) + np.array([true_disturbance.wind_x_mps, true_disturbance.wind_y_mps])
            velocity = v_true + self.rng.normal(0.0, p.gnss_velocity_sigma_mps, size=2)
            velocity_sigma = p.gnss_velocity_sigma_mps

        self.n_fixes += 1
        return GnssFix(
            valid_time_s=t_s,
            position_m=position,
            position_sigma_m=p.gnss_position_sigma_m,
            velocity_mps=velocity,
            velocity_sigma_mps=velocity_sigma,
        )
