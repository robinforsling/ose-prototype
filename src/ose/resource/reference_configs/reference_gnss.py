"""Reference configurations for the GNSS resource. See package docstring."""

from __future__ import annotations

from ose.resource.gnss import GnssParameters

STANDARD = GnssParameters(
    gnss_rate_hz=1.0,
    gnss_position_sigma_m=3.0,
    gnss_velocity_sigma_mps=0.15,
    gnss_velocity_enabled=True,
)
