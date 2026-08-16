"""Reference configurations for vehicle guidance. See package docstring."""

from __future__ import annotations

from ose.subsystem.vehicle_guidance import VehicleGuidanceParameters

STANDARD = VehicleGuidanceParameters(
    heading_gain_per_s=0.3,
    speed_gain_per_s=0.05,
    # Same as the heading gain: the track loop has the same structure and the
    # same plant beneath it, so there is no reason for it to differ until a
    # measurement says otherwise.
    track_gain_per_s=0.3,
)
