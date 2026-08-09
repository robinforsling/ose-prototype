"""
Air data sensor model.

Resource-layer: reads true_state directly, privileged access nothing above
this layer has. Publishes AirDataMeasurement, which carries no truth. See
ADR 0008.

Rate-limiting is the caller's responsibility -- sample() is expected to be
called only when due, per the ordering contract in ADR 0009. due() is
offered as a convenience so the rate parameter does not have to be reached
for from outside. Unlike GNSS,
this sensor has no denial concept: called when due, it always returns a
measurement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import AirDataMeasurement
from ose.resource.vehicle import VehicleState


@dataclass
class AirDataParameters:
    """Shape only, no defaults -- a sensor grade is a choice, not a
    universal, so it belongs in a named reference config
    (resource/reference_configs/reference_air_data.py), not baked in here."""

    air_data_rate_hz: float
    air_data_sigma_mps: float


class AirDataSensor:
    """Synthesises a scalar airspeed measurement from truth."""

    def __init__(
        self,
        parameters: AirDataParameters,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters
        self.rng = rng or np.random.default_rng(0)
        self._t_last = -math.inf

    def due(self, t_s: float) -> bool:
        return t_s - self._t_last >= 1.0 / self.par.air_data_rate_hz

    def sample(self, t_s: float, true_state: VehicleState) -> AirDataMeasurement:
        self._t_last = t_s
        p = self.par
        z = true_state.v_mps + float(self.rng.normal(0.0, p.air_data_sigma_mps))
        return AirDataMeasurement(
            valid_time_s=t_s,
            airspeed_mps=z,
            airspeed_sigma_mps=p.air_data_sigma_mps,
        )
