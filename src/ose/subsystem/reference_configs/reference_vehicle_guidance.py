"""Reference configurations for vehicle guidance. See package docstring."""

from __future__ import annotations

from ose.subsystem.vehicle_guidance import VehicleGuidanceParameters

STANDARD = VehicleGuidanceParameters(
    heading_gain_per_s=0.3,
    speed_gain_per_s=0.05,
)
