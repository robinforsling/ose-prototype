"""What a crosswind costs a platform that holds a heading.

Guidance holds air-relative quantities — a heading and an airspeed — and wind
enters the dynamics in the position rows only. So the loop sees nothing change
while the platform drifts, and every claim here is about the gap between
pointing and moving.

None of it is a defect in a component. Guidance holds the commanded heading
exactly; the vehicle flies what it is told; navigation, here, is perfect. The
error is emergent, which is why these are behaviour tests: no unit or seam test
could show it, because nothing is behaving incorrectly.

They exist because the platform publishes everything needed to correct for wind
— `wind_estimate_mps` and `ground_velocity_mps` travel on every
`OwnStateEstimate` — and nothing consumes either. Pinning the cost is what
makes that a known gap rather than a surprise, and what would make a future
track-hold setpoint show up as these numbers improving.
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
def test_a_route_still_captures_in_wind_but_bows_off_the_direct_line():
    """Pursuit converges, which is why the gap is easy to miss.

    The planner recomputes the bearing to the active waypoint every cycle, so a
    crosswind is corrected continuously and the waypoint is reached. It is
    reached along a bowed path, because the platform is always pointing at the
    waypoint and always moving somewhere else.

    A route flown in wind therefore looks correct -- every waypoint captured --
    while flying a track nobody asked for. That is the shape of the omission.
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
    assert wind_excursion > 500.0, (
        f"only {wind_excursion:.0f} m of excursion -- the bow should be hundreds "
        "of metres, and if it is not the wind is not reaching the platform"
    )
    # Bounded as well as present: pursuit does converge, so an unbounded
    # excursion would mean it had stopped converging rather than merely bowing.
    assert wind_excursion < 0.1 * LEG_M, (
        f"{wind_excursion:.0f} m off a {LEG_M:.0f} m leg is not a bow, it is a "
        "failure to converge"
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
