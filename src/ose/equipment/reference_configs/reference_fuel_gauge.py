"""Reference configurations for the fuel gauge. See package
docstring."""

from __future__ import annotations

from ose.equipment.fuel_gauge import FuelGaugeParameters

STANDARD = FuelGaugeParameters(
    fuel_rate_hz=1.0,
    fuel_sigma_kg=20.0,
)
