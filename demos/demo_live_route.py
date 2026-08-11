"""
Flies a route through the action planner and visualises the result, live in a
window or written to a video file.

Companion to demo_live_flight.py. That one drives guidance directly from a
scripted sequence of setpoints; this one puts the single-ship action planner
in front of it, so the loop is the full stack:

    WaypointPlanner  ->  ActionSet  ->  VehicleGuidance  ->  PlanarPointMass  ->  RK4
    (single-ship)                       (subsystem)          (equipment)

Nobody tells the aircraft what heading to fly. The planner is given a route of
places to be, decides a bearing each cycle from where it believes it is, and
guidance turns that into an admissible command. Every layer built so far is in
the loop, which is what makes this the demo where an integration error between
layers -- a sign, a frame, a unit -- actually shows up.

What the route is chosen to show
--------------------------------
  legs 1-2      ordinary cruise at 280 m/s. The capture radius sits near
                1070 m, a little over one turn radius at that speed.

  leg 3         same geometry, slower (190 m/s). The capture radius falls to
                about 690 m, because turn radius is v / omega_available and
                both terms move with state. This is why the planner asks
                guidance for its capability instead of reading a configured
                number: a fixed radius would be wrong at one end or the other.

  leg 4         a deliberately infeasible corner. The next waypoint is 2.5 km
                away and almost directly back along the inbound leg, so the
                aircraft cannot curve onto it: it saturates the turn rate for
                43 percent of the leg, overshoots, and has to come back. Watch
                the range trace rise before it falls. The planner is not wrong
                and the vehicle is not wrong -- the route is infeasible, and
                the point is that this is visible rather than quietly smoothed
                away.

  leg 5         commands 650 m/s against a 600 m/s limit. The planner emits it
                unclamped on purpose (ADR 0006, and
                test_planner_does_not_clamp_an_infeasible_speed): thrust pins
                at maximum and the airspeed trace never reaches the dotted
                command. Clamping in the planner would have hidden an
                infeasible route behind plausible-looking flight.

  after the route  the planner publishes ActionSet(motion=None), which means
                "no new action, continue as before" and never "stop". The
                caller holds the last setpoint and the aircraft flies straight
                on, shaded grey on the plots. That semantics is pinned by
                test_route_end_publishes_no_motion; here you can watch it.

The moving circle on the ground track is the capture radius around the active
waypoint, resized every frame. The dashed line is the bearing the planner is
currently steering at.

Run with:
    python demo_live_route.py                 live window, or video if headless
    python demo_live_route.py --video         force writing plots/live_route.mp4
    python demo_live_route.py --speed 1       start at real time

Transport controls are shared with demo_live_flight (see _player.py):

    |<  <<  pause  >>  >|      restart, step back, play/pause, step, jump to end
    - speed / + speed          halve or double the playback rate
    the bar                    drag to scrub anywhere; scrubbing pauses

Mass is believed, not known
---------------------------
The full loop is    FuelGauge -> VehicleManager -> VehicleGuidance,   so the
mass guidance flies on is estimated rather than read from truth. The readout
prints both: true mass beside the manager's belief and its declared sigma.

Over this route the vehicle burns about 420 kg and the belief tracks it to
about 3 kg rms, against a gauge whose individual readings are worth 20 kg.
The filter predicts on the commanded thrust between readings and corrects on
each one, so most of the gauge noise is averaged away and the sigma settles
near 2 kg. It reached 65 kg before that filter existed, when the belief was
the last raw reading.

This demo is NOT evidence about navigation
------------------------------------------
Navigation is perfect here: the estimate handed to the planner and to guidance
is built from truth, so the planner steers at where the aircraft really is.
A real navigation error moves the believed position, which moves the bearing
and the measured range, and capture becomes a decision made on an estimate.
demo_navigation.py is where estimation error is the subject.
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np
from _player import CONTROLS_HELP, Player

from ose.equipment.fuel_gauge import FuelGauge
from ose.equipment.reference_configs.reference_fuel_gauge import (
    STANDARD as FUEL_GAUGE_STANDARD,
)
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
from ose.equipment.vehicle import VehicleState
from ose.integration import step_rk4
from ose.interfaces import ActionSet, HeadingSpeedSetpoint, OwnStateEstimate
from ose.single_ship.action_planner import Waypoint, WaypointPlanner
from ose.single_ship.reference_configs.reference_action_planner import (
    STANDARD as PLANNER_STANDARD,
)
from ose.subsystem.reference_configs.reference_vehicle_guidance import (
    STANDARD as GUIDANCE_STANDARD,
)
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    BELIEVED_TSFC_KG_PER_N_S,
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_guidance import VehicleGuidance
from ose.subsystem.vehicle_manager import VehicleManager

DT = 0.02
T_MAX = 900.0            # backstop: an infeasible route must not hang the demo
HOLD_AFTER_ROUTE_S = 45.0
PLOTS_DIR = Path(__file__).resolve().parent / "plots"

# Positions are (north, east) in metres, per the planar NED convention. The
# per-waypoint speed is what the route asks for on the leg leading to it, and
# is emitted unclamped -- see the module docstring on leg 5.
ROUTE: list[Waypoint] = [
    Waypoint(16000.0, 0.0, 280.0),         # settle on a long northbound leg
    Waypoint(24000.0, 15000.0, 280.0),     # ordinary turn, cruise speed
    Waypoint(13000.0, 24000.0, 190.0),     # slow down: capture radius shrinks
    Waypoint(15000.0, 22500.0, 190.0),     # infeasible corner, near-reversal
    Waypoint(2000.0, 29000.0, 650.0),      # above v_max, deliberately
    Waypoint(0.0, 8000.0, 250.0),          # home
]


@dataclass
class Recording:
    """Everything the renderer needs, sampled on the same clock.

    Kept as plain lists while running and converted once at the end: appending
    to a list is cheap, growing a numpy array per step is not.
    """

    t: list[float] = field(default_factory=list)
    p_x: list[float] = field(default_factory=list)
    p_y: list[float] = field(default_factory=list)
    psi: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    mass: list[float] = field(default_factory=list)
    # True mass and the manager's belief about it, side by side. They
    # differ by the gauge noise plus however stale the last reading is,
    # and a renderer that drew only one of them would hide the component
    # under test.
    mass_believed: list[float] = field(default_factory=list)
    mass_sigma: list[float] = field(default_factory=list)

    psi_cmd: list[float] = field(default_factory=list)
    v_cmd: list[float] = field(default_factory=list)

    # The planner's own view: which waypoint it is chasing, how far away it
    # thinks it is, and how close counts as arrived at this instant.
    wp_index: list[float] = field(default_factory=list)
    range_m: list[float] = field(default_factory=list)
    capture_m: list[float] = field(default_factory=list)
    # True once the route is exhausted and motion=None is being published.
    holding: list[bool] = field(default_factory=list)

    omega_req: list[float] = field(default_factory=list)
    omega_del: list[float] = field(default_factory=list)
    omega_lim: list[float] = field(default_factory=list)
    thrust_req: list[float] = field(default_factory=list)
    thrust_del: list[float] = field(default_factory=list)
    thrust_lim: list[float] = field(default_factory=list)

    omega_clipped: list[bool] = field(default_factory=list)
    thrust_clipped: list[bool] = field(default_factory=list)
    in_violation: list[bool] = field(default_factory=list)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v) for k, v in self.__dict__.items()}


def perfect_estimate(t_s: float, state: VehicleState) -> OwnStateEstimate:
    """Stands in for navigation. Truth and belief coincide here on purpose, so
    that what the plots show is planner and guidance behaviour rather than
    estimation error -- see the module docstring."""
    v_vec = state.v_mps * np.array([math.cos(state.psi_rad), math.sin(state.psi_rad)])
    return OwnStateEstimate(
        t_s=t_s,
        p_x_m=state.p_x_m,
        p_y_m=state.p_y_m,
        psi_rad=state.psi_rad,
        v_air_mps=state.v_mps,
        ground_velocity_mps=v_vec,
        wind_estimate_mps=np.zeros(2),
        covariance=np.zeros((4, 4)),
    )


def fly() -> tuple[dict[str, np.ndarray], list[float]]:
    """The throwaway simulation core: fixed step, one platform, log everything.

    Returns the recording and the times at which a waypoint was captured.
    """
    vehicle = reference_fighter()
    # The fuel gauge is the only measurement source in this demo, and the
    # vehicle manager the only consumer of it. Without the pair, the manager
    # would sit on its initial 4 000 kg guess for the whole run while the
    # vehicle burned through it, and guidance would fly an aircraft it
    # believed to be a tonne heavier than it was.
    gauge = FuelGauge(
        FUEL_GAUGE_STANDARD, vehicle.lam.mass_dry_kg, np.random.default_rng(7)
    )
    manager = VehicleManager(vehicle, MANAGER_STANDARD)
    guidance = VehicleGuidance(manager, GUIDANCE_STANDARD)
    planner = WaypointPlanner(ROUTE, PLANNER_STANDARD)
    state = VehicleState(0.0, 0.0, 0.0, 280.0, 16000.0)
    rec = Recording()

    # Seeded with "hold what you are already doing", so the absent-field
    # semantics are well defined even before the planner has said anything
    # and even if the route is empty.
    committed = ActionSet(
        t_s=0.0, motion=HeadingSpeedSetpoint(state.psi_rad, state.v_mps)
    )

    captures: list[float] = []
    previous_index = 0
    t_route_done: float | None = None
    reported = False

    t = 0.0
    while t < T_MAX:
        # Equipment layer first: the gauge is the one component here entitled
        # to read truth, and the manager consumes only what it publishes.
        if gauge.due(t):
            manager.ingest(gauge.sample(t, state))

        estimate = perfect_estimate(t, state)
        capability = guidance.capability(estimate)
        actions = planner.plan(t, estimate, capability)

        # The absent-field rule, applied by the record that defines it:
        # motion=None means "no new action, continue as before", so the last
        # setpoint stays in force. It does NOT mean stop. Callers used to
        # each write this branch themselves.
        holding = actions.motion is None
        committed = actions.merged_onto(committed)

        if planner.index != previous_index:
            captures.append(t)
            previous_index = planner.index
        if planner.finished and t_route_done is None:
            t_route_done = t
            print(f"  route complete at t={t:.1f} s; holding last action")

        cmd, sat = guidance.command(t, committed.motion, estimate)
        envelope = vehicle.capability(state)

        rec.t.append(t)
        rec.p_x.append(state.p_x_m)
        rec.p_y.append(state.p_y_m)
        rec.psi.append(math.degrees(state.psi_rad))
        rec.v.append(state.v_mps)
        rec.mass.append(state.mass_kg)
        believed = manager.mass(t)
        rec.mass_believed.append(believed.mass_kg)
        rec.mass_sigma.append(believed.mass_sigma_kg)
        rec.psi_cmd.append(
            math.degrees(math.remainder(committed.motion.psi_cmd_rad, 2.0 * math.pi))
        )
        rec.v_cmd.append(committed.motion.v_cmd_mps)

        # NaN once the route is done: there is no active waypoint, so there is
        # no range and no capture radius. Recording the last value instead
        # would draw a flat line suggesting the planner was still chasing
        # something.
        rec.wp_index.append(float(planner.index) if not planner.finished else float("nan"))
        rec.range_m.append(
            planner.range_to_active_m(estimate) if not planner.finished else float("nan")
        )
        rec.capture_m.append(
            planner.capture_radius_m(estimate, capability)
            if not planner.finished
            else float("nan")
        )
        rec.holding.append(holding)

        rec.omega_req.append(math.degrees(sat.requested.omega_rad_s))
        rec.omega_del.append(math.degrees(cmd.omega_rad_s))
        rec.omega_lim.append(math.degrees(envelope.omega_available_rad_s))
        rec.thrust_req.append(sat.requested.thrust_N / 1e3)
        rec.thrust_del.append(cmd.thrust_N / 1e3)
        rec.thrust_lim.append(envelope.thrust_available_N / 1e3)
        rec.omega_clipped.append(sat.omega_clipped)
        rec.thrust_clipped.append(sat.thrust_clipped)

        # Propagate the mass belief over the same interval the vehicle is
        # about to fly, burning at the thrust actually commanded. Without it
        # the filter would grow steadily more confident in a fuel figure that
        # is falling underneath it.
        manager.predict(t + DT, cmd.thrust_N, BELIEVED_TSFC_KG_PER_N_S)
        state = step_rk4(vehicle, state, cmd, DT)
        violations = vehicle.state_violations(state)
        if violations and not reported:
            print(f"  state violation at t={t:.1f} s: {violations[0]}")
            reported = True
        rec.in_violation.append(bool(violations))

        t += DT
        if t_route_done is not None and t > t_route_done + HOLD_AFTER_ROUTE_S:
            break

    if not planner.finished:
        print(
            f"  WARNING: route not completed within {T_MAX:.0f} s "
            f"(stuck on waypoint {planner.index})"
        )
    return rec.as_arrays(), captures


# A unit circle, reused every frame to draw the capture radius rather than
# rebuilding a patch. Closed, so the line joins up.
_UNIT_CIRCLE = np.linspace(0.0, 2.0 * math.pi, 121)


def _break_wraps(deg: np.ndarray) -> np.ndarray:
    """Insert NaN wherever a wrapped angle jumps, so the pen lifts instead of
    drawing a vertical line clean across the panel.

    Both traces on the heading panel live in (-180, 180], and this route
    crosses that seam three times. Without the break each crossing draws a
    full-height stroke that looks like an instantaneous 360 degree reversal.
    Display only: the recording keeps the real values, which the readout and
    the heading arrows need.
    """
    out = np.asarray(deg, dtype=float).copy()
    out[1:][np.abs(np.diff(out)) > 180.0] = np.nan
    return out


def build_figure(log, captures, plt):
    """Static scaffolding: axes, limits, and the lines that never change.
    Everything the animation moves is returned in `art`."""
    fig = plt.figure(figsize=(15.0, 9.0))
    gs = fig.add_gridspec(5, 2, width_ratios=[1.15, 1.0], hspace=0.55, wspace=0.18)
    art: dict[str, object] = {}

    route_y = np.array([wp.p_y_m for wp in ROUTE]) / 1e3
    route_x = np.array([wp.p_x_m for wp in ROUTE]) / 1e3

    # ---- ground track -----------------------------------------------------
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(log["p_y"] / 1e3, log["p_x"] / 1e3, color="0.85", lw=1.0, zorder=1)
    # The route as authored, before anything tried to fly it. The gap between
    # this and the flown track is the whole subject of the demo.
    ax.plot(
        np.r_[0.0, route_y], np.r_[0.0, route_x],
        "-", color="C2", lw=1.0, alpha=0.55, zorder=2, label="route",
    )
    ax.plot(route_y, route_x, "s", color="C2", ms=6, zorder=5)
    for k, (yy, xx) in enumerate(zip(route_y, route_x)):
        ax.annotate(
            f" {k}", (yy, xx), fontsize=8, color="C2",
            va="bottom", ha="left", zorder=5,
        )

    art["capture"] = ax.plot([], [], "-", color="C2", lw=1.2, alpha=0.8, zorder=3)[0]
    # Red, not green: this one belongs to the aircraft, not to the route. In
    # green it was indistinguishable from the leg it runs alongside.
    art["los"] = ax.plot([], [], ":", color="C3", lw=1.0, alpha=0.6, zorder=3)[0]
    art["target"] = ax.plot([], [], "s", color="C2", ms=11, mfc="none", mew=2.0, zorder=6)[0]
    art["trail"] = ax.plot([], [], color="C0", lw=2.0, zorder=4)[0]
    art["ship"] = ax.plot([], [], "o", color="C3", ms=9, zorder=7)[0]
    art["nose"] = ax.plot([], [], "-", color="C3", lw=2.0, zorder=6)[0]
    art["cmd_arrow"] = ax.plot([], [], "--", color="0.45", lw=1.4, zorder=6)[0]

    # A ground track must be equal-aspect or the turns lie about their shape.
    # Rather than let matplotlib shrink the axes box to achieve that, make the
    # data range square first: then equal aspect fills the panel exactly.
    all_y = np.r_[log["p_y"], np.array([wp.p_y_m for wp in ROUTE])]
    all_x = np.r_[log["p_x"], np.array([wp.p_x_m for wp in ROUTE])]
    cx = 0.5 * (all_y.max() + all_y.min()) / 1e3
    cy = 0.5 * (all_x.max() + all_x.min()) / 1e3
    half = 0.5 * max(np.ptp(all_y), np.ptp(all_x)) / 1e3 * 1.12
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")
    ax.set_xlabel("east, $p_y$ [km]")
    ax.set_ylabel("north, $p_x$ [km]")
    ax.set_title("Ground track, route and capture radius", fontsize=10)
    ax.grid(alpha=0.25)
    art["readout"] = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
        family="monospace", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.8"),
    )

    holding = log["holding"]

    # Display copy: the two heading traces get their wraps broken, everything
    # else is passed through untouched.
    disp = dict(log)
    disp["psi"] = _break_wraps(log["psi"])
    disp["psi_cmd"] = _break_wraps(log["psi_cmd"])

    def time_axis(row, ylabel, series, title=None, shade=None):
        a = fig.add_subplot(gs[row, 1])
        for kw in series:
            a.plot(log["t"], disp[kw["key"]], **kw["style"])
        a.set_ylabel(ylabel)
        a.set_xlim(0.0, log["t"][-1])
        a.grid(alpha=0.25)
        a.legend(frameon=False, fontsize=7, ncol=3, loc="upper right")
        if title:
            a.set_title(title, fontsize=9)
        if shade is not None and shade.any():
            a.fill_between(
                log["t"], 0, 1, where=shade, transform=a.get_xaxis_transform(),
                color="C1", alpha=0.15, zorder=0, linewidth=0,
            )
        # The route-exhausted region, on every panel: this is where motion=None
        # is being published and the last action is being held.
        if holding.any():
            a.fill_between(
                log["t"], 0, 1, where=holding, transform=a.get_xaxis_transform(),
                color="0.55", alpha=0.18, zorder=0, linewidth=0,
            )
        # One line per capture, so a feature can be attributed to a leg
        # without counting seconds.
        for tc in captures:
            a.axvline(tc, color="C2", lw=0.8, alpha=0.7, zorder=0)
        cur = a.axvline(0.0, color="C3", lw=1.2)
        return a, cur

    dotted = dict(color="0.4", ls=":", lw=1.3, label="commanded")
    solid = dict(color="C0", lw=1.5, label="true")
    req = dict(color="C3", ls=":", lw=1.1, label="requested")
    dlv = dict(color="C0", lw=1.5, label="delivered")
    lim = dict(color="0.5", ls="--", lw=1.0, label="limit")

    # Range and capture radius on one log axis: range spans 25 km down to a
    # few hundred metres, and the crossing is the event that matters. On a
    # linear axis the capture radius would be pressed flat against zero.
    ax0, art["cur0"] = time_axis(
        0, "range [m]",
        [{"key": "range_m", "style": dict(color="C0", lw=1.5, label="to active wp")},
         {"key": "capture_m", "style": dict(color="C2", ls="--", lw=1.2,
                                            label="capture radius")}],
        "Planner: range to the active waypoint (capture where they meet)",
    )
    ax0.set_yscale("log")

    _, art["cur1"] = time_axis(
        1, "heading [deg]",
        [{"key": "psi_cmd", "style": dotted}, {"key": "psi", "style": solid}],
    )
    _, art["cur2"] = time_axis(
        2, "airspeed [m/s]",
        [{"key": "v_cmd", "style": dotted}, {"key": "v", "style": solid}],
        shade=log["thrust_clipped"],
    )
    _, art["cur3"] = time_axis(
        3, "turn rate [deg/s]",
        [{"key": "omega_lim", "style": lim}, {"key": "omega_req", "style": req},
         {"key": "omega_del", "style": dlv}],
        "Requested vs delivered (orange = saturated, grey = route exhausted)",
        shade=log["omega_clipped"],
    )
    ax4, art["cur4"] = time_axis(
        4, "active waypoint",
        [{"key": "wp_index", "style": dict(color="C2", lw=1.8, drawstyle="steps-post",
                                           label="index")}],
    )
    ax4.set_yticks(range(len(ROUTE)))
    ax4.set_ylim(-0.5, len(ROUTE) - 0.5)
    ax4.set_xlabel("time [s]")

    # tight_layout refuses to reason about a fixed-aspect axes, so the
    # margins are set directly.
    fig.subplots_adjust(left=0.055, right=0.975, top=0.885, bottom=0.14)
    return fig, art


def make_updater(log, art):
    """One frame. Kept separate from the figure so live and video share it."""
    trail_km_x, trail_km_y = log["p_y"] / 1e3, log["p_x"] / 1e3

    # Heading indicators are drawn a fixed fraction of the visible extent so
    # they stay legible whatever the track's size. Constant over the run, so
    # computed once rather than per frame.
    span = max(float(np.ptp(trail_km_x)), float(np.ptp(trail_km_y)), 1.0) * 0.06
    cos_c, sin_c = np.cos(_UNIT_CIRCLE), np.sin(_UNIT_CIRCLE)

    def update(i):
        art["trail"].set_data(trail_km_x[: i + 1], trail_km_y[: i + 1])
        art["ship"].set_data([trail_km_x[i]], [trail_km_y[i]])

        for key, ang_deg, scale in (
            ("nose", log["psi"][i], 1.0),
            ("cmd_arrow", log["psi_cmd"][i], 1.5),
        ):
            a = math.radians(ang_deg)
            art[key].set_data(
                [trail_km_x[i], trail_km_x[i] + span * scale * math.sin(a)],
                [trail_km_y[i], trail_km_y[i] + span * scale * math.cos(a)],
            )

        # The active waypoint, its capture circle at the radius holding right
        # now, and the line of sight the planner is steering along. All three
        # vanish once the route is done, which is exactly what has happened.
        k = log["wp_index"][i]
        if math.isnan(k):
            for key in ("capture", "los", "target"):
                art[key].set_data([], [])
        else:
            wp = ROUTE[int(k)]
            wy, wx = wp.p_y_m / 1e3, wp.p_x_m / 1e3
            r = log["capture_m"][i] / 1e3
            art["capture"].set_data(wy + r * cos_c, wx + r * sin_c)
            art["los"].set_data([trail_km_x[i], wy], [trail_km_y[i], wx])
            art["target"].set_data([wy], [wx])

        flags = []
        if log["omega_clipped"][i]:
            flags.append("TURN RATE SATURATED")
        if log["thrust_clipped"][i]:
            flags.append("THRUST SATURATED")
        if log["in_violation"][i]:
            flags.append("STATE OUTSIDE X(lambda)")
        if log["holding"][i]:
            flags.append("ROUTE DONE - holding last action")

        wp_text = "none" if math.isnan(k) else f"{int(k)} of {len(ROUTE)}"
        rng = log["range_m"][i]
        rng_text = "     -- " if math.isnan(rng) else f"{rng:7.0f} m"
        cap_text = (
            "     -- " if math.isnan(log["capture_m"][i])
            else f"{log['capture_m'][i]:7.0f} m"
        )

        art["readout"].set_text(
            f"t      {log['t'][i]:7.1f} s\n"
            f"p_x    {log['p_x'][i] / 1e3:7.2f} km\n"
            f"p_y    {log['p_y'][i] / 1e3:7.2f} km\n"
            f"psi    {log['psi'][i]:7.1f} deg   (cmd {log['psi_cmd'][i]:6.1f})\n"
            f"v      {log['v'][i]:7.1f} m/s   (cmd {log['v_cmd'][i]:6.1f})\n"
            f"m      {log['mass'][i]:7.0f} kg    (believed "
            f"{log['mass_believed'][i]:.0f} +- {log['mass_sigma'][i]:.0f})\n"
            f"omega  {log['omega_del'][i]:7.2f} deg/s (req {log['omega_req'][i]:7.2f})\n"
            f"thrust {log['thrust_del'][i]:7.1f} kN   (req {log['thrust_req'][i]:7.1f})\n"
            f"target {wp_text}\n"
            f"range  {rng_text}  (capture {cap_text})\n"
            + ("\n" + "\n".join(flags) if flags else "")
        )
        for key in ("cur0", "cur1", "cur2", "cur3", "cur4"):
            art[key].set_xdata([log["t"][i], log["t"][i]])
        return tuple(art.values())

    return update


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--video", action="store_true", help="write mp4 instead of a window")
    # Slow by default: the point of the window is to study the behaviour, and
    # anything faster runs the interesting parts past you. Adjust live with
    # the speed buttons, or pause and step.
    ap.add_argument("--speed", type=float, default=4.0, help="initial playback speed-up")
    ap.add_argument("--fps", type=int, default=25)
    args = ap.parse_args()

    headless = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    to_video = args.video or headless
    matplotlib.use("Agg" if to_video else "TkAgg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation

    print("Route flight: planner -> guidance -> vehicle")
    print("-" * 62)
    print(f"{'wp':>3}  {'north [km]':>11} {'east [km]':>10} {'v_cmd [m/s]':>12}")
    for k, wp in enumerate(ROUTE):
        print(f"{k:>3}  {wp.p_x_m / 1e3:>11.1f} {wp.p_y_m / 1e3:>10.1f} "
              f"{wp.v_cmd_mps:>12.0f}")
    print()

    log, captures = fly()

    # Lesson 4 from demo_live_flight: simulate at the physics rate, render at
    # an eye's rate.
    stride = max(1, int(round((1.0 / args.fps) * args.speed / DT)))
    frames = range(0, len(log["t"]), stride)

    v_max = reference_fighter().lam.v_max_mps
    print(
        f"\n  simulated       : {log['t'][-1]:.0f} s at {1 / DT:.0f} Hz "
        f"({len(log['t'])} steps)"
        f"\n  rendered        : {len(frames)} frames at {args.fps} fps"
        f"\n  captured        : {len(captures)} of {len(ROUTE)} waypoints"
        f"\n  turn saturated  : {100.0 * log['omega_clipped'].mean():.1f} % of the time"
        f"\n  thrust saturated: {100.0 * log['thrust_clipped'].mean():.1f} % of the time"
        f"\n  outside X(lam)  : {100.0 * log['in_violation'].mean():.1f} % of the time"
    )

    # Per-leg, because the interesting numbers are per-leg: the capture radius
    # tracking speed, and the one corner the airframe cannot make.
    print(f"\n{'leg':>4} {'duration':>9} {'capture radius':>20} {'turn saturated':>15}"
          f" {'range rose':>11}")
    edges = [0.0, *captures, log["t"][-1]]
    for k in range(len(ROUTE)):
        m = log["wp_index"] == k
        if not m.any():
            continue
        r = log["range_m"][m]
        rose = int((np.diff(r) > 0).sum())
        print(
            f"{k:>4} {edges[k + 1] - edges[k]:>8.1f}s "
            f"{log['capture_m'][m].min():>9.0f}-{log['capture_m'][m].max():<9.0f} m"
            f" {100.0 * log['omega_clipped'][m].mean():>13.1f} %"
            f" {rose:>10} "
        )
    print(
        "\n  leg 3 is the infeasible corner: the turn saturates and the range to"
        "\n  the waypoint rises before it falls -- the aircraft overshoots and has"
        "\n  to come back. The route is infeasible; nothing hides it."
        f"\n\n  leg 4 commands {ROUTE[4].v_cmd_mps:.0f} m/s against a {v_max:.0f} m/s limit and reaches "
        f"{log['v'].max():.0f} m/s."
        "\n  The planner does not clamp it; thrust saturates and the gap stays"
        "\n  visible, which is what ADR 0006 is for."
    )

    fig, art = build_figure(log, captures, plt)
    update = make_updater(log, art)

    if to_video:
        PLOTS_DIR.mkdir(exist_ok=True)
        out = PLOTS_DIR / "live_route.mp4"
        anim = FuncAnimation(fig, update, frames=frames, blit=False)
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
        plt.close(fig)
        print(f"\nWrote {out}")
    else:
        player = Player(fig, log, update, args.speed, args.fps, DT)  # noqa: F841
        fig.canvas.manager.set_window_title("OSE - route flight")
        print("\n" + CONTROLS_HELP + "\n\nClose the window to exit.")
        plt.show()


if __name__ == "__main__":
    main()
