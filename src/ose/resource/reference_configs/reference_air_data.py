"""Reference configurations for the air data resource. See package docstring."""

from __future__ import annotations

from ose.resource.air_data import AirDataParameters

STANDARD = AirDataParameters(
    air_data_rate_hz=10.0,
    air_data_sigma_mps=1.0,
)
