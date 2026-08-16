"""Tests for the single-ship action planner.

The one that matters is test_flies_the_whole_route, which closes the loop
through every layer built so far: planner decides, guidance converts to a
command, the vehicle enforces its own sets, the integrator flies it. It is
the first test in the repository that exercises the full stack rather than
one component against stubs.

test_route_end_publishes_no_motion pins the semantics of an absent field --
"no new action, continue as before", never "stop" -- because that reading is
what makes silence safe, and nothing but a test stops someone reinterpreting
it later.
"""

import dataclasses
import math

import numpy as np
import pytest


from ose import interfaces
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
from ose.equipment.vehicle import VehicleState
from ose.integration import step_rk4
from ose.interfaces import ActionSet, TrackSpeedSetpoint, OwnStateEstimate
from ose.single_ship.action_planner import (
    Waypoint,
    WaypointPlanner,
    WaypointPlannerParameters,
)
from ose.single_ship.reference_configs.reference_action_planner import STANDARD
from ose.subsystem.reference_configs.reference_vehicle_guidance import (
    STANDARD as GUIDANCE_STANDARD,
)
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_guidance import VehicleGuidance
from ose.subsystem.vehicle_manager import VehicleManager


def _estimate(state: VehicleState, t_s: float = 0.0) -> OwnStateEstimate:
    v = state.v_mps * np.array([math.cos(state.psi_rad), math.sin(state.psi_rad)])
    return OwnStateEstimate(
        t_s=t_s,
        p_x_m=state.p_x_m,
        p_y_m=state.p_y_m,
        psi_rad=state.psi_rad,
        v_air_mps=state.v_mps,
        ground_velocity_mps=v,
        wind_estimate_mps=np.zeros(2),
        covariance=np.zeros((4, 4)),
    )


@pytest.fixture
def vehicle():
    return reference_fighter()


@pytest.fixture
def guidance(vehicle):
    return VehicleGuidance(
        VehicleManager(vehicle, MANAGER_STANDARD), GUIDANCE_STANDARD
    )


@pytest.fixture
def state():
    return VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_satisfies_the_protocol():
    planner = WaypointPlanner([Waypoint(1000.0, 0.0, 250.0)], STANDARD)
    assert isinstance(planner, interfaces.ActionPlanner)


# --------------------------------------------------------------------------
# The absent-field semantics
# --------------------------------------------------------------------------

def test_route_end_publishes_no_motion(vehicle, guidance, state):
    """An exhausted route says nothing about motion. None means continue as
    before, so the vehicle keeps flying its last commanded heading; it does
    not mean stop, which would have to be commanded explicitly."""
    planner = WaypointPlanner([Waypoint(10.0, 0.0, 250.0)], STANDARD)
    est = _estimate(state)
    cap = guidance.capability(est)

    actions = planner.plan(0.0, est, cap)      # already inside capture radius
    assert planner.finished
    assert actions.motion is None
    assert actions.t_s == 0.0


def test_an_empty_route_is_immediately_finished(guidance, state):
    planner = WaypointPlanner([], STANDARD)
    est = _estimate(state)
    assert planner.plan(0.0, est, guidance.capability(est)).motion is None


# --------------------------------------------------------------------------
# Carrying an absent field over -- the rule, rather than each caller's copy
# --------------------------------------------------------------------------

def _action_fields():
    fields = [f.name for f in dataclasses.fields(ActionSet) if f.name != "t_s"]
    assert fields, "ActionSet has no action fields -- these tests are vacuous"
    return fields


def test_every_action_field_participates_in_the_merge():
    """The guard that has to exist before there is a second field.

    A field added to ActionSet and forgotten in merged_onto would silently
    reset to None every cycle, so a subsystem would lose its last commanded
    action with nothing raising. Walking the dataclass makes that impossible
    to introduce quietly, which is the same failure PromisedEnvelope's
    direction table exists to prevent.

    Distinct sentinel objects rather than real setpoints: this is about
    whether each field is plumbed through at all, not about what it holds.
    """
    fields = _action_fields()
    previous = ActionSet(t_s=0.0, **{name: object() for name in fields})
    silent = ActionSet(t_s=1.0)                      # nothing new to say

    merged = silent.merged_onto(previous)

    for name in fields:
        assert getattr(merged, name) is getattr(previous, name), (
            f"ActionSet.{name} is not carried over by merged_onto(); an absent "
            "value resets it instead of continuing the last committed one"
        )


def test_a_stated_action_overrides_the_previous_one():
    """The other half: present means present. A rule that only ever carried
    forward would freeze the platform on its first action."""
    fields = _action_fields()
    previous = ActionSet(t_s=0.0, **{name: object() for name in fields})
    fresh = ActionSet(t_s=1.0, **{name: object() for name in fields})

    merged = fresh.merged_onto(previous)

    for name in fields:
        assert getattr(merged, name) is getattr(fresh, name), (
            f"ActionSet.{name} kept the previous value over a stated one"
        )


def test_propulsion_carries_over_independently_of_motion():
    """The two fields have different lifetimes, so they must not be coupled:
    a planner restating a heading without mentioning boost should not cancel
    boost, and vice versa. The generic guard above proves every field is
    carried; this proves they are carried separately."""
    previous = ActionSet(
        t_s=0.0, motion=TrackSpeedSetpoint(1.0, 250.0), propulsion="boost"
    )

    new_heading = ActionSet(t_s=1.0, motion=TrackSpeedSetpoint(2.0, 250.0))
    merged = new_heading.merged_onto(previous)
    assert merged.motion.psi_g_cmd_rad == 2.0
    assert merged.propulsion == "boost", "a new heading cancelled the mode"

    new_mode = ActionSet(t_s=2.0, propulsion="nominal")
    merged = new_mode.merged_onto(previous)
    assert merged.propulsion == "nominal"
    assert merged.motion is previous.motion, "a mode change cancelled the heading"


def test_the_merge_takes_the_new_timestamp():
    """t_s labels this cycle, not the cycle the carried-over action came
    from -- otherwise a held action would make time appear to stop."""
    previous = ActionSet(t_s=0.0, motion=TrackSpeedSetpoint(0.0, 250.0))
    assert ActionSet(t_s=7.5).merged_onto(previous).t_s == 7.5


def test_merging_is_the_rule_the_route_end_relies_on(vehicle, guidance, state):
    """End to end on the record alone: once the route is exhausted the
    planner says nothing, and the committed action is unchanged."""
    planner = WaypointPlanner([Waypoint(10.0, 0.0, 250.0)], STANDARD)
    est = _estimate(state)

    committed = ActionSet(t_s=0.0, motion=TrackSpeedSetpoint(1.23, 251.0))
    committed = planner.plan(0.0, est, guidance.capability(est)).merged_onto(committed)

    assert planner.finished
    assert committed.motion.psi_g_cmd_rad == 1.23
    assert committed.motion.v_cmd_mps == 251.0


# --------------------------------------------------------------------------
# Steering
# --------------------------------------------------------------------------

def test_steers_at_the_active_waypoint(vehicle, guidance, state):
    planner = WaypointPlanner([Waypoint(0.0, 20000.0, 300.0)], STANDARD)  # due east
    est = _estimate(state)
    actions = planner.plan(0.0, est, guidance.capability(est))

    assert isinstance(actions.motion, TrackSpeedSetpoint)
    assert actions.motion.psi_g_cmd_rad == pytest.approx(math.radians(90.0))
    assert actions.motion.v_cmd_mps == 300.0


def test_advances_through_the_route_as_each_is_captured(vehicle, guidance, state):
    route = [
        Waypoint(20000.0, 0.0, 250.0),
        Waypoint(20000.0, 20000.0, 250.0),
        Waypoint(0.0, 20000.0, 250.0),
    ]
    planner = WaypointPlanner(route, STANDARD)
    est = _estimate(state)
    cap = guidance.capability(est)

    assert planner.index == 0
    planner.plan(0.0, est, cap)
    assert planner.index == 0                       # far from the first

    at_first = _estimate(VehicleState(20000.0, 0.0, 0.0, 250.0, 16000.0))
    planner.plan(1.0, at_first, cap)
    assert planner.index == 1                       # captured, moved on


# --------------------------------------------------------------------------
# Capability, used rather than reimplemented
# --------------------------------------------------------------------------

def test_capture_radius_grows_with_turn_radius(vehicle, guidance):
    """A waypoint cannot be captured inside the circle the aircraft is
    physically able to fly, so the radius has to follow the vehicle's
    current turn performance rather than be a fixed number."""
    planner = WaypointPlanner([Waypoint(50000.0, 0.0, 250.0)], STANDARD)

    slow = _estimate(VehicleState(0.0, 0.0, 0.0, 150.0, 16000.0))
    fast = _estimate(VehicleState(0.0, 0.0, 0.0, 400.0, 16000.0))
    r_slow = planner.capture_radius_m(slow, guidance.capability(slow))
    r_fast = planner.capture_radius_m(fast, guidance.capability(fast))

    assert r_fast > r_slow
    # And it really is about a turn radius, not the configured floor.
    cap = guidance.capability(fast)
    turn_radius = fast.v_air_mps / cap.max_turn_rate_rad_s
    assert r_fast == pytest.approx(STANDARD.capture_turn_radii * turn_radius)


def test_capture_radius_never_falls_below_the_floor(vehicle, guidance):
    """A vehicle that can turn on the spot would otherwise be asked to hit a
    waypoint exactly, which no closed loop achieves."""
    tight = WaypointPlannerParameters(min_capture_radius_m=800.0, capture_turn_radii=0.0)
    planner = WaypointPlanner([Waypoint(50000.0, 0.0, 250.0)], tight)
    est = _estimate(VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0))
    assert planner.capture_radius_m(est, guidance.capability(est)) == 800.0


def test_planner_does_not_clamp_an_infeasible_speed(vehicle, guidance, state):
    """Silently clamping would hide an infeasible route behind plausible
    flight. The planner emits what the route asks and lets enforcement clip
    it visibly, the same argument as ADR 0006."""
    too_fast = vehicle.lam.v_max_mps + 200.0
    planner = WaypointPlanner([Waypoint(50000.0, 0.0, too_fast)], STANDARD)
    est = _estimate(state)
    cap = guidance.capability(est)

    actions = planner.plan(0.0, est, cap)
    assert actions.motion.v_cmd_mps == too_fast
    assert not cap.admits(actions.motion)      # and the check is available


def test_the_plan_is_time_invariant(guidance, state):
    """t_s reaches no decision: it only stamps the record.

    A Waypoint carries no time, capture is by radius rather than by clock, and
    nothing in plan() reads t_s except the ActionSet it builds. So the same
    estimate at a different time yields the same motion.

    Pinned because the module docstring claims it, and because a
    time-of-arrival constraint would break it -- which is the point at which
    this planner stops being geometric, and the claim should be revisited
    deliberately rather than discovered.
    """
    route = [Waypoint(20000.0, 0.0, 250.0)]
    estimate = _estimate(state)
    capability = guidance.capability(estimate)

    early = WaypointPlanner(route, STANDARD).plan(0.0, estimate, capability)
    late = WaypointPlanner(route, STANDARD).plan(999.0, estimate, capability)

    assert early.motion == late.motion
    assert early.t_s != late.t_s, "the timestamp should still be the one given"


# --------------------------------------------------------------------------
# The whole stack
# --------------------------------------------------------------------------

@pytest.mark.slow
def test_flies_the_whole_route(vehicle, guidance, state):
    """Planner to guidance to vehicle to integrator, closed loop.

    A box route, each leg long enough that the turns are not the whole
    flight. Reaching every waypoint means the layers agree about frames,
    units and signs -- the classic integration failure this environment
    exists to catch, and the first test here to exercise all of them at once.
    """
    route = [
        Waypoint(30000.0, 0.0, 250.0),
        Waypoint(30000.0, 30000.0, 250.0),
        Waypoint(0.0, 30000.0, 250.0),
        Waypoint(0.0, 0.0, 250.0),
    ]
    planner = WaypointPlanner(route, STANDARD)

    committed = ActionSet(t_s=0.0, motion=TrackSpeedSetpoint(0.0, 250.0))
    dt, t = 0.05, 0.0
    while t < 900.0 and not planner.finished:
        est = _estimate(state, t)
        committed = planner.plan(t, est, guidance.capability(est)).merged_onto(committed)
        cmd, _ = guidance.command(t, committed.motion, est)
        state = step_rk4(vehicle, state, cmd, dt)
        t += dt

    assert planner.finished, f"only reached waypoint {planner.index} of {len(route)}"
    assert t < 900.0


def test_holding_pattern_after_the_route_keeps_flying(vehicle, guidance, state):
    """The absent-field semantics, end to end: once the route is done the
    caller has no new motion action, keeps the last command, and the vehicle
    carries on rather than stopping or falling out of the sky."""
    planner = WaypointPlanner([Waypoint(4000.0, 0.0, 250.0)], STANDARD)
    committed = ActionSet(t_s=0.0, motion=TrackSpeedSetpoint(0.0, 250.0))

    dt, t = 0.05, 0.0
    while t < 120.0:
        est = _estimate(state, t)
        # merged_onto is the rule: an absent motion leaves the last one
        # standing, so the loop needs no branch of its own.
        committed = planner.plan(t, est, guidance.capability(est)).merged_onto(committed)
        cmd, _ = guidance.command(t, committed.motion, est)
        state = step_rk4(vehicle, state, cmd, dt)
        t += dt

    assert planner.finished
    assert state.v_mps > 200.0                 # still flying
    assert state.p_x_m > 4000.0                # and still going
