"""Reference configurations for the action planner. See package docstring."""

from __future__ import annotations

from ose.single_ship.action_planner import WaypointPlannerParameters

STANDARD = WaypointPlannerParameters(
    min_capture_radius_m=500.0,
    # Slightly over one turn radius. Below one the aircraft cannot curve onto
    # the next leg and orbits the waypoint it is trying to capture.
    capture_turn_radii=1.2,
)
