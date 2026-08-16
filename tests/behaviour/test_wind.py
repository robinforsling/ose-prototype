"""What a crosswind costs a platform that holds a heading.

Guidance holds air-relative quantities — a heading and an airspeed — and wind
enters the dynamics in the position rows only. So the loop sees nothing change
while the platform drifts, and every claim here is about the gap between
pointing and moving.

None of it is a defect in a component. Guidance holds the commanded heading
exactly; the vehicle flies what it is told; navigation, here, is perfect. The
error is emergent, which is why these are behaviour tests: no unit or seam test
could show it, because nothing is behaving incorrectly.

Guidance now holds a ground track when asked for one, and the planner asks
(ADR 0029), so most of the cost is gone: the same 30 km leg that bowed 1 390 m
off the direct line bows 92 m. What remains is the heading tests below, which
still describe what a HEADING setpoint does, because that setpoint still
exists and is still the right thing when what you mean is a heading.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from _platform import fly

from ose.equipment.vehicle import Disturbance, VehicleState
from ose.interfaces import HeadingSpeedSetpoint
from ose.single_ship.action_planner import Waypoint

AIRSPEED_MPS = 250.0
CROSSWIND_MPS = 30.0

# Due north, so a +y wind is a pure crosswind and cross-track error is p_y.
NORTH = 0.0
LEG_M = 30000.0


def _hold_north(t, capability):
    return HeadingSpeedSetpoint(NORTH, AIRSPEED_MPS)


def _initial():
    return VehicleState(0.0, 0.0, NORTH, AIRSPEED_MPS, 16000.0)


@pytest.mark.performance
def test_heading_is_held_exactly_while_the_platform_drifts():
    """The two halves of the claim, in one test, because either alone misleads.

    A test that only checked the drift would look like a guidance fault. A test
    that only checked the heading would look like everything was fine.
    """
    rec, _, _ = fly(
        _hold_north, 200.0, initial=_initial(),
        disturbance=Disturbance(wind_y_mps=CROSSWIND_MPS),
    )
    a = rec.arrays()

    assert abs(a["psi"]).max() < math.radians(0.01), (
        "guidance did not hold the commanded heading -- this test assumes it does"
    )
    assert a["p_y"][-1] > 5000.0, (
        f"only {a['p_y'][-1]:.0f} m of drift; the wind is not reaching the platform"
    )


@pytest.mark.performance
def test_the_track_error_is_the_wind_triangle():
    """Drift is not approximately the wind triangle, it is exactly it.

    Holding heading in a crosswind puts the ground track arctan(w/v) off the
    commanded heading. Pinning the closed form rather than a measured number
    means the test still means something if the reference airspeed changes.
    """
    rec, _, _ = fly(
        _hold_north, 200.0, initial=_initial(),
        disturbance=Disturbance(wind_y_mps=CROSSWIND_MPS),
    )
    a = rec.arrays()

    track = math.atan2(a["p_y"][-1] - a["p_y"][0], a["p_x"][-1] - a["p_x"][0])
    expected = math.atan2(CROSSWIND_MPS, AIRSPEED_MPS)
    assert abs(track - expected) < math.radians(0.05), (
        f"track {math.degrees(track):.2f}° against a wind triangle predicting "
        f"{math.degrees(expected):.2f}°"
    )


@pytest.mark.performance
def test_a_route_in_wind_stays_near_the_direct_line():
    """What the planner commanding a TRACK bought.

    This assertion is the inverse of the one it replaces. Before ADR 0029 the
    planner commanded the bearing as a HEADING, so the platform pointed at the
    waypoint and moved somewhere else, bowing 1 390 m off a 30 km leg in this
    wind; the test asserted the bow was greater than 500 m. It is now 92 m, and
    the threshold is what a reader should compare against that number.

    The residual is not an error to be tuned away. A pursuit law steers at
    where the waypoint is, so a leg begins with a real track error while the
    loop turns onto it, and that transient is most of what is left.
    """
    route = [Waypoint(LEG_M, 0.0, AIRSPEED_MPS)]
    common = dict(route=route, initial=_initial(), stop_when_route_done=True)

    calm, _, planner_calm = fly(None, 400.0, **common)
    windy, _, planner_wind = fly(
        None, 400.0, disturbance=Disturbance(wind_y_mps=CROSSWIND_MPS), **common
    )
    assert planner_calm.finished and planner_wind.finished

    calm_excursion = float(np.abs(calm.arrays()["p_y"]).max())
    wind_excursion = float(np.abs(windy.arrays()["p_y"]).max())

    assert calm_excursion < 1.0, f"the calm leg was not straight: {calm_excursion:.0f} m"
    assert wind_excursion < 250.0, (
        f"{wind_excursion:.0f} m off the direct line -- track hold has stopped "
        "working, or the estimate has stopped carrying the wind in its ground "
        "velocity, which is the same failure wearing a different hat"
    )


@pytest.mark.performance
def test_the_heading_settles_at_the_crab_angle():
    """Holding a track means NOT holding a heading, and the heading says so.

    Guidance never computes arcsin(w/v); the loop leaves the heading wherever
    zero track error requires it, which is the crab angle. Asserting the closed
    form is what distinguishes a loop that found the right answer from one that
    merely stopped moving.
    """
    route = [Waypoint(LEG_M, 0.0, AIRSPEED_MPS)]
    windy, _, _ = fly(
        None, 400.0, route=route, initial=_initial(), stop_when_route_done=True,
        disturbance=Disturbance(wind_y_mps=CROSSWIND_MPS),
    )
    settled = windy.arrays()["psi"][-1]
    crab = -math.asin(CROSSWIND_MPS / AIRSPEED_MPS)

    assert abs(settled - crab) < math.radians(0.5), (
        f"heading settled at {math.degrees(settled):.2f}°, crab angle is "
        f"{math.degrees(crab):.2f}°"
    )


@pytest.mark.performance
def test_head_and_tailwind_change_the_clock_and_nothing_else():
    """Along-track wind costs time, not track.

    The complement of the crosswind case, and it is what shows the drift is a
    geometry effect rather than the loop misbehaving whenever the air moves.
    """
    route = [Waypoint(LEG_M, 0.0, AIRSPEED_MPS)]
    common = dict(route=route, initial=_initial(), stop_when_route_done=True)

    calm, _, _ = fly(None, 400.0, **common)
    head, _, _ = fly(None, 400.0, disturbance=Disturbance(wind_x_mps=-30.0), **common)
    tail, _, _ = fly(None, 400.0, disturbance=Disturbance(wind_x_mps=30.0), **common)

    for label, run in (("headwind", head), ("tailwind", tail)):
        excursion = float(np.abs(run.arrays()["p_y"]).max())
        assert excursion < 1.0, f"{label} produced {excursion:.0f} m of track error"

    assert head.captures[0] > calm.captures[0] > tail.captures[0], (
        f"capture times not ordered head > calm > tail: "
        f"{head.captures[0]:.1f}, {calm.captures[0]:.1f}, {tail.captures[0]:.1f}"
    )
