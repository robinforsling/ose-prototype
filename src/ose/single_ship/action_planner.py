"""
Action planner: decides what the platform should do next, and publishes it
as an ActionSet.

Single-ship layer, purely cyber, and two layers above anything entitled to
read truth. This module must not import VehicleState or Disturbance, and no
public method may take a parameter beginning with true_ -- checked by
test_planner_cannot_see_truth. Its only view of the world is the
OwnStateEstimate that the navigation manager publishes.

WaypointPlanner is one implementation of the role, not the role itself. It
follows a route: steer at the active waypoint, capture it, move to the next,
and when the route runs out, say nothing further about motion. Swapping it
for something else -- a threat-reactive planner, or eventually one that
plans motion and sensing together -- is a matter of providing the same ports,
which is what `single_ship.planner.<name>` in the composition spec is for.

Where the layers divide
-----------------------
The planner decides WHERE to go; guidance decides HOW to fly there. So
waypoint steering lives here, converting a position to a heading command,
and guidance stays an inner loop over heading, speed and turn rate. That is
the conventional split, and it is why a waypoint setpoint type was never
needed: the conversion happens a layer up, where the route is known.

It is stateful, unlike guidance, and legitimately so: a plan is a thing you
are partway through. The active waypoint index is that state.

On capability
-------------
The planner is handed the capability of the guidance it commands, and uses
it rather than reimplementing the dynamics. Capture radius is the clearest
case: a waypoint cannot sensibly be captured inside the vehicle's own turn
radius, because the aircraft physically cannot curve tightly enough to pass
through it and would orbit instead. Turn radius is v divided by the
achievable turn rate, both of which move with state, so the radius is
recomputed rather than configured once.

It deliberately does NOT clamp a commanded speed into the achievable band.
Emitting what the route asks for and letting enforcement clip it leaves a
visible Saturation finding, where silently clamping here would hide an
infeasible route behind plausible-looking flight. Same reasoning as ADR
0006. `admits()` is available to anyone who wants to check a route before
flying it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ose.interfaces import (
    ActionSet,
    GuidanceCapability,
    HeadingSpeedSetpoint,
    OwnStateEstimate,
)


@dataclass(frozen=True)
class Waypoint:
    """Somewhere to be, and how fast to be going when you get there.

    Not in interfaces.py: a route is this planner's own parameter today, not
    a record passed between components. It moves there if and when something
    publishes routes -- coord.intent.v1 is the likely source.
    """

    p_x_m: float
    p_y_m: float
    v_cmd_mps: float


@dataclass
class WaypointPlannerParameters:
    """Shape only, no defaults, as everywhere else in this repository."""

    # Floor on the capture radius. The turn-radius rule below usually
    # dominates; this stops the radius collapsing to nothing when slow.
    min_capture_radius_m: float
    # Multiple of the vehicle's own minimum turn radius to capture within.
    # Below about 1 the aircraft cannot curve onto the next leg and orbits.
    capture_turn_radii: float


class WaypointPlanner:
    """Follows a route of waypoints, one leg at a time."""

    def __init__(self, route: list[Waypoint], parameters: WaypointPlannerParameters) -> None:
        self.route = list(route)
        self.par = parameters
        self.index = 0

    @property
    def finished(self) -> bool:
        return self.index >= len(self.route)

    @property
    def active(self) -> Waypoint | None:
        return None if self.finished else self.route[self.index]

    def capture_radius_m(
        self, own_state: OwnStateEstimate, capability: GuidanceCapability
    ) -> float:
        """How close counts as arrived, given what the vehicle can currently
        turn. Recomputed each call because both terms move with state."""
        rate = capability.max_turn_rate_rad_s
        turn_radius = own_state.v_air_mps / rate if rate > 0.0 else math.inf
        return max(self.par.min_capture_radius_m, self.par.capture_turn_radii * turn_radius)

    def range_to_active_m(self, own_state: OwnStateEstimate) -> float:
        wp = self.active
        if wp is None:
            return math.inf
        return math.hypot(wp.p_x_m - own_state.p_x_m, wp.p_y_m - own_state.p_y_m)

    def plan(
        self,
        t_s: float,
        own_state: OwnStateEstimate,
        capability: GuidanceCapability,
    ) -> ActionSet:
        # Capture first, so arriving and departing happen in the same cycle
        # rather than leaving one step of flight aimed at a waypoint already
        # behind the aircraft.
        while (
            not self.finished
            and self.range_to_active_m(own_state)
            <= self.capture_radius_m(own_state, capability)
        ):
            self.index += 1

        wp = self.active
        if wp is None:
            # Route exhausted. motion=None means "no new action, continue as
            # before" -- the vehicle holds whatever guidance was last told,
            # which is straight flight on the final leg's heading. Commanding
            # a stop would be a different decision, and would have to be said
            # rather than implied.
            return ActionSet(t_s=t_s)

        bearing = math.atan2(wp.p_y_m - own_state.p_y_m, wp.p_x_m - own_state.p_x_m)
        return ActionSet(
            t_s=t_s,
            # Rate zero: a straight leg has a bearing that barely moves, so
            # there is nothing to feed forward. A curved leg would set this.
            motion=HeadingSpeedSetpoint(bearing, wp.v_cmd_mps, psi_rate_cmd_rad_s=0.0),
        )
