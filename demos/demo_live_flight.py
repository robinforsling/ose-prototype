"""
Flies a sequence of guidance setpoints and visualises the result, live in a
window or written to a video file.

This is a prototype of the plan-view renderer `docs/50-tooling.md` calls
priority one, and a throwaway stand-in for the simulation core, which does not
exist. Neither is meant to survive as written -- see LESSONS below, and the
notes in that file, for what is worth keeping when the real ones are built.

What it shows
-------------
A mission is a list of segments, each holding a heading or sweeping it at a
fixed rate. Guidance chases the current setpoint; the vehicle flies whatever
guidance commands, after its own admissible sets have been applied.

The manoeuvres are chosen to press against different limits:

  360 at 9 g       a full circle at the structural limit. It cannot be held --
                   sustained rate at 250 m/s is 14.7 deg/s against the 20.1
                   commanded -- so speed bleeds and the circle spirals in.
                   Reaches the full 9 g because the sweep declares its own
                   rate and guidance feeds it forward; without that a
                   proportional law trails a ramp by rate/gain, about 67
                   degrees, and only ever reached 8 g.

  lazy eight       the planar equivalent. The real manoeuvre trades height for
                   speed through opposing 180s; with altitude suppressed the
                   exchange lives in airspeed alone, decelerating into one loop
                   and accelerating out through the other, and the ground track
                   crosses itself as a figure eight.

  corner sweep     flown on a turn-rate setpoint, asking for 60 deg/s that no
                   speed can deliver, so the vehicle stays pinned against
                   omega_max for the whole segment while decelerating from
                   380 m/s. The delivered rate therefore traces the
                   corner-speed curve and peaks where the lift and structural
                   limits meet, landing on v_corner exactly. That is the
                   sharpest check in this repository that the turn-performance
                   model is self-consistent.

  ground track      where it went, with the trail and current heading
  heading, airspeed  commanded against true -- the tracking task itself
  turn rate, thrust  requested against delivered, with the vehicle's limit
  readout            the state vector, load factor, and saturation flags

Requested-against-delivered is the interesting pair. They separate exactly
when guidance asks for more than the airframe can give, and that difference
is what ADR 0006 exists to keep visible rather than silently clipping.

Run with:
    python demo_live_flight.py                 live window, or video if headless
    python demo_live_flight.py --video         force writing plots/live_flight.mp4
    python demo_live_flight.py --video --speed 16   shorter video, quicker render
    python demo_live_flight.py --speed 1       start at real time

The live window has transport controls, because watching a mission go past
once tells you much less than being able to stop on the moment a limit binds
and step through it:

    |<  <<  pause  >>  >|      restart, step back, play/pause, step, jump to end
    - speed / + speed          halve or double the playback rate
    the bar                    drag to scrub anywhere; scrubbing pauses
    space, left/right          play/pause, step a frame
    home/end, up/down          jump to either end, change speed

The clock reads elapsed, total, and remaining, with the current speed-up.

LESSONS, for whoever builds the real core and the real renderer
---------------------------------------------------------------
1. Simulate first, render second. The loop below records everything and
   returns; the renderer reads the recording afterwards. Interleaving them
   couples frame rate to step size and makes the run unrepeatable when the
   display stalls. It also means the same recording can be replayed live,
   written to video, or diffed against another run.

2. Record what was asked for, not only what happened. Enforcement is
   invisible otherwise -- a clipped command looks like a command. This demo
   needed `Saturation.requested`, which did not exist until a previous demo
   worked around its absence by duplicating the control law and then went
   stale.

3. Record the bounds too, not just the signals. The turn-rate limit moves
   with speed and mass, so a fixed line on the plot would be wrong. Anything
   state-dependent that a reader will compare against has to be sampled at
   the same instant as the thing it bounds.

4. Decimate for display, not for simulation. Physics wants 50 Hz; an eye
   wants 25 fps. Conflating them either wastes frames or corrupts dynamics.

5. The renderer reads truth, and must stay write-only. It is a tool, not a
   component, so ADR 0008 does not forbid it truth -- but nothing it computes
   may flow back into the simulation, or the truth boundary is breached
   through the debugging aid, which is the case that ADR warns is hardest to
   detect afterwards. Here the estimate handed to guidance is perfect, so
   truth and belief coincide; a real renderer must draw them as two things,
   because they will not.
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

from ose.integration import step_rk4
from ose.interfaces import (
    HeadingSpeedSetpoint,
    OwnStateEstimate,
    TurnRateSpeedSetpoint,
)
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleState
from ose.subsystem.reference_configs.reference_vehicle_guidance import STANDARD
from ose.subsystem.vehicle_guidance import VehicleGuidance

DT = 0.02
PLOTS_DIR = Path(__file__).resolve().parent / "plots"

# The mission, as segments rather than instants: three of these manoeuvres are
# continuous turns, and a list of held headings cannot express one. A segment
# either holds the heading it inherits or sweeps it at a fixed rate. The
# schedule is built by accumulating, so the commanded heading stays a pure
# function of time and never reads the vehicle's state.
@dataclass(frozen=True)
class Segment:
    name: str
    duration_s: float
    v_cmd_mps: float
    turn_rate_deg_s: float = 0.0      # 0 holds the inherited heading
    # Fly this one on a turn-rate setpoint instead of a heading one. Needed
    # only where the commanded rate is deliberately unreachable: a heading
    # command cannot express that, because the setpoint laps the vehicle and
    # the wrapped error reverses the turn.
    rate_setpoint: bool = False


MISSION: list[Segment] = [
    Segment("settle", 12.0, 250.0),

    # A full circle at the structural limit. 20.1 deg/s is omega_max at
    # 250 m/s, where the 9 g structural limit binds rather than lift.
    # Sustained rate there is only 14.7 deg/s, so this cannot be held: speed
    # bleeds and the circle spirals in. That energy cost is the point.
    Segment("360 at 9 g", 18.0, 250.0, turn_rate_deg_s=20.1),
    Segment("recover", 30.0, 250.0),

    # Lazy eight, planar equivalent. The real manoeuvre trades height for
    # speed through opposing 180s; with altitude suppressed the exchange has
    # to be expressed in airspeed alone -- decelerating into one loop and
    # accelerating out through the other -- and the ground track crosses
    # itself as a figure eight rather than the nose tracing one against the
    # horizon.
    Segment("lazy eight, right loop", 26.0, 190.0, turn_rate_deg_s=14.0),
    Segment("lazy eight, left loop", 26.0, 290.0, turn_rate_deg_s=-14.0),
    Segment("rejoin", 25.0, 250.0),

    # Corner speed. Accelerate, then command a turn rate faster than any
    # speed can deliver together with a low speed setpoint, so thrust falls to
    # idle. The vehicle decelerates through the envelope while pinned against
    # omega_max, and the delivered turn rate traces the corner-speed curve:
    # rising as lift allows more, peaking where the lift and structural limits
    # meet, then falling as g/v shrinks. Sweeping through beats sampling three
    # fixed speeds, because the peak lands wherever it lands.
    #
    # rate_setpoint=True, and this segment is why TurnRateSpeedSetpoint
    # exists. A heading command cannot say "turn as hard as you can": ask for
    # a rate above the achievable one and the setpoint laps the vehicle, the
    # heading error wraps through 180 and changes sign, and guidance reverses.
    # That gave a sawtooth, five reversals, and only 80 percent of the segment
    # saturated. With no heading to chase there is no error to wrap: 60 deg/s
    # now pins against omega_max for the whole segment, zero reversals.
    Segment("accelerate for corner run", 45.0, 380.0, turn_rate_deg_s=4.0),
    Segment("corner sweep", 95.0, 140.0, turn_rate_deg_s=60.0, rate_setpoint=True),
    Segment("recover", 20.0, 220.0),
]
T_END = sum(seg.duration_s for seg in MISSION)


def build_schedule(mission):
    """(start, end, segment, commanded heading at its start)."""
    out, t0, psi = [], 0.0, 0.0
    for seg in mission:
        out.append((t0, t0 + seg.duration_s, seg, psi))
        psi += seg.turn_rate_deg_s * seg.duration_s
        t0 += seg.duration_s
    return out


SCHEDULE = build_schedule(MISSION)


def segment_at(t_s: float):
    for entry in SCHEDULE:
        if entry[0] <= t_s < entry[1]:
            return entry
    return SCHEDULE[-1]


def setpoint_at(t_s: float):
    t0, _, seg, psi0 = segment_at(t_s)
    if seg.rate_setpoint:
        return TurnRateSpeedSetpoint(
            math.radians(seg.turn_rate_deg_s), seg.v_cmd_mps
        )
    return HeadingSpeedSetpoint(
        math.radians(psi0 + seg.turn_rate_deg_s * (t_s - t0)),
        seg.v_cmd_mps,
        # The sweep's own rate, fed forward. Without it a proportional law
        # settles rate/gain behind a moving setpoint -- 67 degrees for the
        # 9 g circle -- and the vehicle never reaches the commanded turn.
        psi_rate_cmd_rad_s=math.radians(seg.turn_rate_deg_s),
    )


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

    psi_cmd: list[float] = field(default_factory=list)
    v_cmd: list[float] = field(default_factory=list)

    omega_req: list[float] = field(default_factory=list)
    omega_del: list[float] = field(default_factory=list)
    omega_lim: list[float] = field(default_factory=list)
    thrust_req: list[float] = field(default_factory=list)
    thrust_del: list[float] = field(default_factory=list)
    thrust_lim: list[float] = field(default_factory=list)

    load_factor: list[float] = field(default_factory=list)
    n_available: list[float] = field(default_factory=list)
    omega_clipped: list[bool] = field(default_factory=list)
    thrust_clipped: list[bool] = field(default_factory=list)
    # X(lambda) violations, checked every step. project_command() keeps the
    # INPUT inside U(x); nothing keeps the STATE inside X, because a state
    # cannot be projected without falsifying the dynamics that produced it.
    # So it is reported instead -- see ADR 0006 and state_violations().
    in_violation: list[bool] = field(default_factory=list)

    def as_arrays(self) -> dict[str, np.ndarray]:
        return {k: np.asarray(v) for k, v in self.__dict__.items()}


def perfect_estimate(t_s: float, state: VehicleState) -> OwnStateEstimate:
    """Stands in for navigation. Truth and belief coincide here on purpose, so
    that what the plots show is guidance behaviour and not estimation error --
    demo_navigation.py is where estimation error is the subject."""
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


def fly() -> dict[str, np.ndarray]:
    """The throwaway simulation core: fixed step, one platform, log everything."""
    vehicle = reference_fighter()
    guidance = VehicleGuidance(vehicle, STANDARD)
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    rec = Recording()
    reported = False

    t = 0.0
    while t < T_END:
        setpoint = setpoint_at(t)
        cmd, sat = guidance.command(t, setpoint, perfect_estimate(t, state), state.mass_kg)
        envelope = vehicle.capability(state)

        rec.t.append(t)
        rec.p_x.append(state.p_x_m)
        rec.p_y.append(state.p_y_m)
        rec.psi.append(math.degrees(state.psi_rad))
        rec.v.append(state.v_mps)
        rec.mass.append(state.mass_kg)
        # Wrapped, because a sweeping setpoint accumulates without bound --
        # the corner run alone commands over 5000 degrees -- while the state's
        # heading wraps to +-180. Plotting the two unwrapped compares a ramp
        # against a sawtooth and shows nothing.
        # NaN while a turn-rate setpoint is active: no heading is being
        # commanded then, and matplotlib leaves a gap rather than drawing a
        # line through a number that does not exist. Inventing one -- the
        # current heading, say -- would suggest guidance was holding it.
        rec.psi_cmd.append(
            math.degrees(math.remainder(setpoint.psi_cmd_rad, 2.0 * math.pi))
            if isinstance(setpoint, HeadingSpeedSetpoint)
            else float("nan")
        )
        rec.v_cmd.append(setpoint.v_cmd_mps)
        rec.omega_req.append(math.degrees(sat.requested.omega_rad_s))
        rec.omega_del.append(math.degrees(cmd.omega_rad_s))
        rec.omega_lim.append(math.degrees(envelope.omega_available_rad_s))
        rec.thrust_req.append(sat.requested.thrust_N / 1e3)
        rec.thrust_del.append(cmd.thrust_N / 1e3)
        rec.thrust_lim.append(envelope.thrust_available_N / 1e3)
        rec.load_factor.append(vehicle.load_factor(state.v_mps, cmd.omega_rad_s))
        rec.n_available.append(envelope.load_factor_available)
        rec.omega_clipped.append(sat.omega_clipped)
        rec.thrust_clipped.append(sat.thrust_clipped)

        state = step_rk4(vehicle, state, cmd, DT)
        violations = vehicle.state_violations(state)
        if violations and not reported:
            print(f"  state violation at t={t:.1f} s: {violations[0]}")
            reported = True
        rec.in_violation.append(bool(violations))
        t += DT

    return rec.as_arrays()


def build_figure(log, plt, vehicle_n_structural=9.0):
    """Static scaffolding: axes, limits, and the lines that never change.
    Everything the animation moves is returned in `art`."""
    fig = plt.figure(figsize=(15.0, 9.0))
    gs = fig.add_gridspec(5, 2, width_ratios=[1.15, 1.0], hspace=0.55, wspace=0.18)
    art: dict[str, object] = {}

    # ---- ground track -----------------------------------------------------
    ax = fig.add_subplot(gs[:, 0])
    ax.plot(log["p_y"] / 1e3, log["p_x"] / 1e3, color="0.85", lw=1.0, zorder=1)
    art["trail"] = ax.plot([], [], color="C0", lw=2.0, zorder=2)[0]
    art["ship"] = ax.plot([], [], "o", color="C3", ms=9, zorder=4)[0]
    art["nose"] = ax.plot([], [], "-", color="C3", lw=2.0, zorder=3)[0]
    art["cmd_arrow"] = ax.plot([], [], "--", color="0.45", lw=1.4, zorder=3)[0]

    # A ground track must be equal-aspect or the turns lie about their shape.
    # Rather than let matplotlib shrink the axes box to achieve that, make the
    # data range square first: then equal aspect fills the panel exactly.
    # (adjustable="datalim" would do it too, but silently discards set_ylim.)
    cx = 0.5 * (log["p_y"].max() + log["p_y"].min()) / 1e3
    cy = 0.5 * (log["p_x"].max() + log["p_x"].min()) / 1e3
    half = 0.5 * max(np.ptp(log["p_y"]), np.ptp(log["p_x"])) / 1e3 * 1.12
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal")
    ax.set_xlabel("east, $p_y$ [km]")
    ax.set_ylabel("north, $p_x$ [km]")
    ax.set_title("Ground track", fontsize=10)
    ax.grid(alpha=0.25)
    art["readout"] = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
        family="monospace", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.8"),
    )

    def time_axis(row, ylabel, series, title=None):
        a = fig.add_subplot(gs[row, 1])
        for kw in series:
            a.plot(log["t"], log[kw["key"]], **kw["style"])
        a.set_ylabel(ylabel)
        a.set_xlim(0.0, log["t"][-1])
        a.grid(alpha=0.25)
        a.legend(frameon=False, fontsize=7, ncol=3, loc="upper right")
        if title:
            a.set_title(title, fontsize=9)
        # Shade every interval in which a command was clipped, so saturation
        # is legible from the static plot as well as from the moving cursor.
        clipped = log["omega_clipped"] if row == 2 else (
            log["thrust_clipped"] if row == 3 else None
        )
        if clipped is not None and clipped.any():
            a.fill_between(log["t"], 0, 1, where=clipped, transform=a.get_xaxis_transform(),
                           color="C1", alpha=0.15, zorder=0, linewidth=0)
        # Segment boundaries, so a reader can tell which manoeuvre a feature
        # belongs to without counting seconds.
        for t0, _, seg, _ in SCHEDULE[1:]:
            a.axvline(t0, color="0.75", lw=0.7, zorder=0)
        if row == 0:
            # Alternating rows: short segments sit close together and their
            # labels ran into each other on one line.
            for k, (t0, t1, seg, _) in enumerate(SCHEDULE):
                a.text(
                    0.5 * (t0 + t1), 1.04 + 0.15 * (k % 2), seg.name,
                    transform=a.get_xaxis_transform(), ha="center", va="bottom",
                    fontsize=6, color="0.35",
                )
        cur = a.axvline(0.0, color="C3", lw=1.2)
        return a, cur

    dotted = dict(color="0.4", ls=":", lw=1.3, label="commanded")
    solid = dict(color="C0", lw=1.5, label="true")
    req = dict(color="C3", ls=":", lw=1.1, label="requested")
    dlv = dict(color="C0", lw=1.5, label="delivered")
    lim = dict(color="0.5", ls="--", lw=1.0, label="limit")

    _, art["cur0"] = time_axis(
        0, "heading [deg]",
        [{"key": "psi_cmd", "style": dotted}, {"key": "psi", "style": solid}],
    )
    _, art["cur1"] = time_axis(
        1, "airspeed [m/s]",
        [{"key": "v_cmd", "style": dotted}, {"key": "v", "style": solid}],
    )
    _, art["cur2"] = time_axis(
        2, "turn rate [deg/s]",
        [{"key": "omega_lim", "style": lim}, {"key": "omega_req", "style": req},
         {"key": "omega_del", "style": dlv}],
        "Control inputs: requested vs delivered (shaded = saturated)",
    )
    _, art["cur3"] = time_axis(
        3, "thrust [kN]",
        [{"key": "thrust_lim", "style": lim}, {"key": "thrust_req", "style": req},
         {"key": "thrust_del", "style": dlv}],
    )
    ax4, art["cur4"] = time_axis(
        4, "load factor [g]",
        [{"key": "n_available", "style": dict(color="0.5", ls="--", lw=1.0,
                                              label="available")},
         {"key": "load_factor", "style": dict(color="C0", lw=1.5, label="flown")}],
    )
    ax4.axhline(
        vehicle_n_structural, color="C3", ls="-.", lw=1.0, label="structural",
    )
    ax4.legend(frameon=False, fontsize=7, ncol=3, loc="upper right")
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

        flags = []
        if log["omega_clipped"][i]:
            flags.append("TURN RATE SATURATED")
        if log["thrust_clipped"][i]:
            flags.append("THRUST SATURATED")
        if log["in_violation"][i]:
            flags.append("STATE OUTSIDE X(lambda)")

        art["readout"].set_text(
            f"t      {log['t'][i]:7.1f} s\n"
            f"p_x    {log['p_x'][i] / 1e3:7.2f} km\n"
            f"p_y    {log['p_y'][i] / 1e3:7.2f} km\n"
            f"psi    {log['psi'][i]:7.1f} deg   (cmd {log['psi_cmd'][i]:6.1f})\n"
            f"v      {log['v'][i]:7.1f} m/s   (cmd {log['v_cmd'][i]:6.1f})\n"
            f"m      {log['mass'][i]:7.0f} kg\n"
            f"n      {log['load_factor'][i]:7.2f} g\n"
            f"omega  {log['omega_del'][i]:7.2f} deg/s (req {log['omega_req'][i]:7.2f})\n"
            f"thrust {log['thrust_del'][i]:7.1f} kN   (req {log['thrust_req'][i]:7.1f})\n"
            + ("\n" + "\n".join(flags) if flags else "")
        )
        for k in ("cur0", "cur1", "cur2", "cur3", "cur4"):
            art[k].set_xdata([log["t"][i], log["t"][i]])
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

    log = fly()

    # Lesson 4: simulate at the physics rate, render at an eye's rate.
    stride = max(1, int(round((1.0 / args.fps) * args.speed / DT)))
    frames = range(0, len(log["t"]), stride)

    print("Live flight")
    print("-" * 62)
    print(f"{'t [s]':>12}  {'manoeuvre':<26} {'turn [deg/s]':>13} {'v [m/s]':>8}")
    for t0, t1, seg, _ in SCHEDULE:
        rate = f"{seg.turn_rate_deg_s:+.1f}" if seg.turn_rate_deg_s else "hold"
        print(f"{t0:5.0f}-{t1:<6.0f}  {seg.name:<26} {rate:>13} {seg.v_cmd_mps:>8.0f}")
    print(
        f"\n  simulated       : {T_END:.0f} s at {1 / DT:.0f} Hz "
        f"({len(log['t'])} steps)"
        f"\n  rendered        : {len(frames)} frames at {args.fps} fps"
        f"\n  turn saturated  : {100.0 * log['omega_clipped'].mean():.1f} % of the time"
        f"\n  thrust saturated: {100.0 * log['thrust_clipped'].mean():.1f} % of the time"
        f"\n  outside X(lam)  : {100.0 * log['in_violation'].mean():.1f} % of the time"
        f"\n  peak load factor: {log['load_factor'].max():.2f} g"
    )

    # The corner-speed run is the one result worth stating numerically: the
    # turn rate should peak exactly where the lift and structural limits
    # cross, and that speed is predictable in closed form from the mass.
    t0, t1, _, _ = next(e for e in SCHEDULE if e[2].name == "corner sweep")
    m = (log["t"] >= t0) & (log["t"] < t1)
    k = int(np.argmax(log["omega_del"][m]))
    v_peak, mass_peak = log["v"][m][k], log["mass"][m][k]
    predicted = reference_fighter().v_corner_mps(mass_peak)
    print(
        f"\n  corner sweep: fastest turn {log['omega_del'][m].max():.2f} deg/s "
        f"at {v_peak:.1f} m/s"
        f"\n                v_corner predicted for {mass_peak:.0f} kg = "
        f"{predicted:.1f} m/s   (error {abs(v_peak - predicted):.1f} m/s)"
        f"\n                swept {log['v'][m].max():.0f} -> {log['v'][m].min():.0f} m/s"
    )

    fig, art = build_figure(log, plt, reference_fighter().lam.n_structural)
    update = make_updater(log, art)

    if to_video:
        PLOTS_DIR.mkdir(exist_ok=True)
        out = PLOTS_DIR / "live_flight.mp4"
        anim = FuncAnimation(fig, update, frames=frames, blit=False)
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
        plt.close(fig)
        print(f"\nWrote {out}")
    else:
        player = Player(fig, log, update, args.speed, args.fps, DT)  # noqa: F841
        fig.canvas.manager.set_window_title("OSE - live flight")
        print(
            "\n" + CONTROLS_HELP + "\n\nClose the window to exit."
        )
        plt.show()


if __name__ == "__main__":
    main()
