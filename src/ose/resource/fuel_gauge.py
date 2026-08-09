"""
Fuel gauge: a direct reading of remaining fuel mass.

Resource-layer: reads true_state directly, privileged access nothing above
this layer has. Publishes FuelMeasurement, which carries no truth. See
ADR 0008.

Unlike Imu or Clock, this sensor has no drift term and needs none: it reads
a quantity (remaining fuel), not a rate that must be integrated to be
useful, so a single additive white-noise term is the whole error model.
mass_dry_kg is a vehicle design constant (a Constraints field), not runtime
truth, so holding it here is no different from Imu holding a Vehicle2D
reference for drag_N.

Rate-limiting is the caller's responsibility -- sample() is expected to be
called only when due, per the ordering contract in ADR 0009. due() is
offered as a convenience so the rate parameter does not have to be reached
for from outside, the same pattern as AirDataSensor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import FuelMeasurement
from ose.resource.vehicle import VehicleState


@dataclass
class FuelGaugeParameters:
    """Shape only, no defaults -- a sensor grade is a choice, not a
    universal, so it belongs in a named reference config
    (resource/reference_configs/reference_fuel_gauge.py), not baked in
    here."""

    fuel_rate_hz: float
    fuel_sigma_kg: float


class FuelGauge:
    """Reads remaining fuel mass (true mass minus dry mass), corrupted by
    additive white noise. No temporal correlation, no bias."""

    def __init__(
        self,
        parameters: FuelGaugeParameters,
        mass_dry_kg: float,
        rng: np.random.Generator | None = None,
    ) -> None:
        self.par = parameters
        self.mass_dry_kg = mass_dry_kg
        self.rng = rng or np.random.default_rng(0)
        self._t_last = -math.inf

    def due(self, t_s: float) -> bool:
        return t_s - self._t_last >= 1.0 / self.par.fuel_rate_hz

    def sample(self, t_s: float, true_state: VehicleState) -> FuelMeasurement:
        self._t_last = t_s
        p = self.par
        true_fuel_kg = true_state.mass_kg - self.mass_dry_kg
        reading = true_fuel_kg + float(self.rng.normal(0.0, p.fuel_sigma_kg))
        return FuelMeasurement(
            valid_time_s=t_s,
            fuel_remaining_kg=reading,
            fuel_remaining_sigma_kg=p.fuel_sigma_kg,
        )
