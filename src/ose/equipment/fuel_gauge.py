"""
Fuel gauge: a direct reading of remaining fuel mass.

Equipment-layer: reads true_state directly, privileged access nothing above
this layer has. Publishes FuelMeasurement, which carries no truth. See
ADR 0008.

Unlike Imu or Clock, this sensor has no drift term and needs none: it reads
a quantity, not a rate that must be integrated to be useful, so a single
additive white-noise term is the whole error model.

WHAT IT ACTUALLY REPORTS
------------------------
Mass above DRY, not fuel, and the field is now named for it. The two differ
by whatever the platform is carrying, and this component has no way to know
that and no business knowing it -- a fuel gauge measures a tank, not a
loadout.

The distinction is not pedantic. VehicleManager decomposes mass as
dry + payload + fuel and used to correct its fuel state on this reading
directly, which put the payload into the fuel state and then added it again
in the mass sum: a platform with 500 kg of stores believed itself 500 kg
heavy at a stated sigma of 1.4 kg. Every fixture left payload at zero, where
the two definitions coincide. The manager now subtracts the payload it
believes in (ADR 0026), and the field is `mass_above_dry_kg` rather than
`fuel_remaining_kg`, which is what made the mistake natural (ADR 0027).

mass_dry_kg is a vehicle design constant (a Constraints field), not runtime
truth. It is a COPY of one, though, taken at construction -- which is the
one way this differs from Imu holding a vehicle reference for drag_N, since
a reference cannot go stale and a copy can. It is also a dependency on the
vehicle that no signature carries, so the architecture diagram and the
composition descriptors both miss it. Both are recorded in ADR 0025.

Rate-limiting is the caller's responsibility -- sample() is expected to be
called only when due, per the ordering contract in ADR 0009. due() is
offered as a convenience so the rate parameter does not have to be reached
for from outside, the same pattern as AirDataSensor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.equipment.vehicle import VehicleState
from ose.interfaces import FuelMeasurement, MeasurementChannel, SensorCapability


@dataclass
class FuelGaugeParameters:
    """Shape only, no defaults -- a sensor grade is a choice, not a
    universal, so it belongs in a named reference config
    (equipment/reference_configs/reference_fuel_gauge.py), not baked in
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

    def capability(self) -> SensorCapability:
        """Always available: this sensor has no denial or failure concept."""
        return SensorCapability(
            rate_hz=self.par.fuel_rate_hz,
            channels=(
                MeasurementChannel("mass_above_dry", self.par.fuel_sigma_kg, "kg"),
            ),
            available=True,
        )

    def due(self, t_s: float) -> bool:
        return t_s - self._t_last >= 1.0 / self.par.fuel_rate_hz

    def sample(self, t_s: float, true_state: VehicleState) -> FuelMeasurement:
        self._t_last = t_s
        p = self.par
        true_fuel_kg = true_state.mass_kg - self.mass_dry_kg
        reading = true_fuel_kg + float(self.rng.normal(0.0, p.fuel_sigma_kg))
        return FuelMeasurement(
            valid_time_s=t_s,
            mass_above_dry_kg=reading,
            mass_above_dry_sigma_kg=p.fuel_sigma_kg,
        )
