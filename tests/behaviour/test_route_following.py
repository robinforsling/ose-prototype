"""A platform flies a route it was given.

The only test that exercises all four layers at once: an equipment-layer gauge
and vehicle, a subsystem-layer manager and guidance, and a single-ship planner.
No component makes this claim -- the planner decides where and never checks it
arrived, guidance decides how and never sees a route -- so it belongs to the
platform or to nothing.

The route is `demos/demo_live_route.py`'s, restated rather than imported, and
its second half is deliberately hostile: a near-reversal the aircraft cannot
turn onto in one pass, and a waypoint demanding a speed above v_max. A route
that could all be flown cleanly would not test the interesting half of the
planner's contract.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from _platform import fly

from ose.single_ship.action_planner import Waypoint

ROUTE = [
    Waypoint(16000.0, 0.0, 280.0),         # settle on a long northbound leg
    Waypoint(24000.0, 15000.0, 280.0),     # ordinary turn at cruise
    Waypoint(13000.0, 24000.0, 190.0),     # slow down: capture radius shrinks
    Waypoint(15000.0, 22500.0, 190.0),     # infeasible corner, near-reversal
    Waypoint(2000.0, 29000.0, 650.0),      # above v_max, deliberately
    Waypoint(0.0, 8000.0, 250.0),          # home
]

T_END = 420.0


@pytest.fixture(scope="module")
def flight():
    return fly(None, T_END, route=ROUTE)


def test_the_whole_route_is_flown(flight):
    """Every waypoint captured, in order, within the time allowed.

    The claim is completion, not accuracy: a planner that captured four of six
    and then orbited would look healthy in every per-component test, because
    each component would still be doing its own job correctly.
    """
    rec, _, planner = flight
    assert planner.finished, (
        f"only {planner.index} of {len(ROUTE)} waypoints captured in {T_END} s"
    )
    assert len(rec.captures) == len(ROUTE)
    assert rec.captures == sorted(rec.captures), "captures out of order"


def test_the_planner_stops_commanding_once_the_route_is_done(flight):
    """A finished planner publishes no motion, and the last setpoint stays in
    force -- absent means "carry on", not "stop". Nothing should lurch when the
    route runs out."""
    rec, _, planner = flight
    assert planner.active is None

    after = [i for i, t in enumerate(rec.t) if t > rec.captures[-1] + 5.0]
    assert after, "route finished too late to observe what follows"
    tail = rec.arrays()
    turn = tail["omega_delivered"][after]
    assert abs(turn).max() < math.radians(5.0), (
        "the platform is still manoeuvring after the route finished"
    )


@pytest.mark.performance
def test_only_the_infeasible_corner_costs_distance(flight):
    """The near-reversal cannot be turned onto in one pass, so the platform
    overshoots and comes back.

    Measured as flown path over direct distance, which is the honest way to ask
    it. Elapsed time is not: leg duration is dominated by leg length, and the
    near-reversal is the shortest leg on the route -- the first version of this
    test asserted it was the slowest and was simply wrong about the system.

    Every other leg comes in at or under 1.0, capture happening within a radius
    rather than at the point. The near-reversal is the one that cannot.
    """
    rec, _, _ = flight
    a = rec.arrays()

    ratios, previous, t0 = [], (0.0, 0.0), 0.0
    for captured_at, wp in zip(rec.captures, ROUTE):
        leg = (a["t"] >= t0) & (a["t"] <= captured_at)
        flown = float(
            np.hypot(np.diff(a["p_x"][leg]), np.diff(a["p_y"][leg])).sum()
        )
        direct = math.hypot(wp.p_x_m - previous[0], wp.p_y_m - previous[1])
        ratios.append(flown / direct)
        previous, t0 = (wp.p_x_m, wp.p_y_m), captured_at

    reversal = ratios[3]              # the leg ENDING at the near-reversal
    others = ratios[:3] + ratios[4:]
    assert reversal > 1.2, f"the infeasible corner cost nothing: {reversal:.2f}"
    assert reversal == max(ratios), (
        f"a feasible leg cost more than the infeasible one: "
        f"{[round(r, 2) for r in ratios]}"
    )
    assert max(others) <= 1.05, (
        f"a leg that should be flyable was not: {[round(r, 2) for r in others]}"
    )
