"""
Flies a sequence of guidance setpoints and visualises the result, live in a
window or written to a video file.

This is a prototype of the plan-view renderer `docs/50-tooling.md` calls
priority one, and a throwaway stand-in for the simulation core, which does not
exist. Neither is meant to survive as written -- see LESSONS below, and the
notes in that file, for what is worth keeping when the real ones are built.

What it shows
-------------
A mission is a list of setpoints with the time each becomes active. Guidance
chases whichever is current; the vehicle flies whatever guidance commands,
after the vehicle's own admissible sets have been applied to it.

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
import time
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
import numpy as np

from ose.integration import step_rk4
from ose.interfaces import HeadingSpeedSetpoint, OwnStateEstimate
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleState
from ose.subsystem.reference_configs.reference_vehicle_guidance import STANDARD
from ose.subsystem.vehicle_guidance import VehicleGuidance

DT = 0.02
PLOTS_DIR = Path(__file__).resolve().parent / "plots"

# The mission: (activation time, heading in degrees, airspeed). Guidance holds
# whichever entry is current. Chosen to exercise a spread of behaviours --
# gentle capture, a speed change with no turn, a reversal the airframe cannot
# follow at the commanded rate, and a slow-down.
MISSION: list[tuple[float, float, float]] = [
    (0.0, 0.0, 250.0),        # hold: no control action should be needed
    (20.0, 70.0, 250.0),      # moderate turn, comfortably inside the envelope
    (60.0, 70.0, 320.0),      # accelerate on a fixed heading
    (100.0, -110.0, 320.0),   # reversal: saturates the turn rate at first
    (150.0, -110.0, 170.0),   # decelerate: guidance asks for negative thrust
                              # and gets idle, no speedbrake on this airframe
    (190.0, -20.0, 200.0),    # final capture at the lower speed
]
T_END = 240.0


def setpoint_at(t_s: float) -> HeadingSpeedSetpoint:
    psi_deg, v = MISSION[0][1], MISSION[0][2]
    for t_active, psi_d, v_cmd in MISSION:
        if t_s >= t_active:
            psi_deg, v = psi_d, v_cmd
    return HeadingSpeedSetpoint(math.radians(psi_deg), v)


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
        rec.psi_cmd.append(math.degrees(setpoint.psi_cmd_rad))
        rec.v_cmd.append(setpoint.v_cmd_mps)
        rec.omega_req.append(math.degrees(sat.requested.omega_rad_s))
        rec.omega_del.append(math.degrees(cmd.omega_rad_s))
        rec.omega_lim.append(math.degrees(envelope.omega_available_rad_s))
        rec.thrust_req.append(sat.requested.thrust_N / 1e3)
        rec.thrust_del.append(cmd.thrust_N / 1e3)
        rec.thrust_lim.append(envelope.thrust_available_N / 1e3)
        rec.load_factor.append(vehicle.load_factor(state.v_mps, cmd.omega_rad_s))
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


def build_figure(log, plt):
    """Static scaffolding: axes, limits, and the lines that never change.
    Everything the animation moves is returned in `art`."""
    fig = plt.figure(figsize=(14.0, 8.0))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.25, 1.0], hspace=0.45, wspace=0.2)
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
        "Guidance setpoints and the response",
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
    ax3, art["cur3"] = time_axis(
        3, "thrust [kN]",
        [{"key": "thrust_lim", "style": lim}, {"key": "thrust_req", "style": req},
         {"key": "thrust_del", "style": dlv}],
    )
    ax3.set_xlabel("time [s]")

    # tight_layout refuses to reason about a fixed-aspect axes, so the
    # margins are set directly.
    fig.subplots_adjust(left=0.06, right=0.97, top=0.93, bottom=0.15)
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
        for k in ("cur0", "cur1", "cur2", "cur3"):
            art[k].set_xdata([log["t"][i], log["t"][i]])
        return tuple(art.values())

    return update


class Player:
    """Transport controls over a finished recording.

    Playback is a timer stepping an index, not a FuncAnimation, because the
    index has to be settable from a slider and a keypress as well as by the
    clock. Scrubbing an animation that owns its own frame counter means
    fighting it; owning the index outright is simpler and makes pause, step
    and seek the same operation with different arguments.
    """

    def __init__(self, fig, plt, log, update, speed, fps):
        from matplotlib.widgets import Button, Slider

        self.fig, self.log, self.update = fig, log, update
        self.n = len(log["t"])
        self.speed = float(speed)
        self.fps = fps
        self.idx = 0
        self.playing = True
        self._last_tick = time.monotonic()
        self._measured_fps = float(fps)

        bar = dict(color="0.92", hovercolor="0.82")
        self.ax_slider = fig.add_axes([0.06, 0.065, 0.91, 0.022])
        self.slider = Slider(
            self.ax_slider, "", 0.0, float(log["t"][-1]),
            valinit=0.0, color="C0", track_color="0.88",
        )
        self.slider.valtext.set_visible(False)
        self.slider.on_changed(self._seek)

        def button(x, w, label, cb):
            b = Button(fig.add_axes([x, 0.018, w, 0.032]), label, **bar)
            b.label.set_fontsize(9)
            b.on_clicked(cb)
            return b

        self.b_start = button(0.06, 0.05, "|<", lambda _: self._goto(0))
        self.b_back = button(0.115, 0.05, "<<", lambda _: self._nudge(-1))
        self.b_play = button(0.17, 0.07, "pause", self._toggle)
        self.b_fwd = button(0.245, 0.05, ">>", lambda _: self._nudge(+1))
        self.b_end = button(0.30, 0.05, ">|", lambda _: self._goto(self.n - 1))
        self.b_slow = button(0.365, 0.05, "- speed", lambda _: self._scale(0.5))
        self.b_fast = button(0.42, 0.05, "+ speed", lambda _: self._scale(2.0))

        self.clock = fig.text(
            0.50, 0.033, "", family="monospace", fontsize=10, va="center",
        )
        fig.text(
            0.72, 0.033,
            "space play/pause   left/right step   home/end   up/down speed",
            fontsize=8, color="0.4", va="center",
        )
        fig.canvas.mpl_connect("key_press_event", self._key)

        self.timer = fig.canvas.new_timer(interval=int(1000.0 / fps))
        self.timer.add_callback(self._tick)
        self.timer.start()
        self._render()

    # -- state changes ---------------------------------------------------

    def _goto(self, i, from_slider=False):
        self.idx = int(min(max(i, 0), self.n - 1))
        if not from_slider:
            # Setting the slider re-enters _seek; guard so the two cannot
            # chase each other.
            self._syncing = True
            self.slider.set_val(self.log["t"][self.idx])
            self._syncing = False
        self._render()

    def _seek(self, value):
        if getattr(self, "_syncing", False):
            return
        self.playing = False
        self.b_play.label.set_text("play")
        self._goto(int(round(value / DT)), from_slider=True)

    @property
    def _step(self) -> int:
        """Indices in one nominal display frame at the current speed."""
        return max(1, int(round(self.speed / (self.fps * DT))))

    def _nudge(self, direction):
        self.playing = False
        self.b_play.label.set_text("play")
        self._goto(self.idx + direction * self._step)

    def _toggle(self, _event=None):
        # Restart from the beginning if play is pressed at the very end,
        # rather than appearing to do nothing.
        if not self.playing and self.idx >= self.n - 1:
            self.idx = 0
        self.playing = not self.playing
        self.b_play.label.set_text("pause" if self.playing else "play")
        self.fig.canvas.draw_idle()

    def _scale(self, factor):
        self.speed = min(max(self.speed * factor, 0.25), 256.0)
        self._render()

    def _key(self, event):
        if event.key == " ":
            self._toggle()
        elif event.key == "right":
            self._nudge(+1)
        elif event.key == "left":
            self._nudge(-1)
        elif event.key == "home":
            self._goto(0)
        elif event.key == "end":
            self._goto(self.n - 1)
        elif event.key == "up":
            self._scale(2.0)
        elif event.key == "down":
            self._scale(0.5)

    # -- the loop --------------------------------------------------------

    def _tick(self):
        """Advance by wall-clock time, not by a fixed number of steps.

        Stepping a fixed stride per frame silently ties playback speed to
        render cost: this figure takes about 80 ms to draw, so a timer asking
        for 25 fps gets 12.5, and a nominal 4x played at 2x while the readout
        insisted it was 4x. Deriving the advance from elapsed wall time makes
        the speed honest -- when rendering cannot keep up, frames are dropped
        rather than time dilated, which is what a media player does.
        """
        now = time.monotonic()
        elapsed = now - self._last_tick
        self._last_tick = now
        if elapsed > 0.0:
            # Smoothed, only for the readout, so a reader can see when the
            # display is struggling rather than wonder why it looks choppy.
            self._measured_fps += 0.2 * (1.0 / elapsed - self._measured_fps)

        if not self.playing:
            return
        if self.idx >= self.n - 1:
            self.playing = False
            self.b_play.label.set_text("play")
            return

        # Clamp the jump so a stall or a long pause does not skip the mission.
        elapsed = min(elapsed, 0.25)
        self._goto(self.idx + max(1, int(round(self.speed * elapsed / DT))))

    def _render(self):
        self.update(self.idx)
        now, total = self.log["t"][self.idx], self.log["t"][-1]
        self.clock.set_text(
            f"{now:6.1f} / {total:.0f} s   -{total - now:5.1f} s"
            f"   x{self.speed:g}   {self._measured_fps:4.1f} fps"
        )
        self.fig.canvas.draw_idle()


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
    print(f"{'t [s]':>8} {'heading [deg]':>15} {'airspeed [m/s]':>16}")
    for t_active, psi_d, v_cmd in MISSION:
        print(f"{t_active:>8.0f} {psi_d:>15.0f} {v_cmd:>16.0f}")
    print(
        f"\n  simulated       : {T_END:.0f} s at {1 / DT:.0f} Hz "
        f"({len(log['t'])} steps)"
        f"\n  rendered        : {len(frames)} frames at {args.fps} fps"
        f"\n  turn saturated  : {100.0 * log['omega_clipped'].mean():.1f} % of the time"
        f"\n  thrust saturated: {100.0 * log['thrust_clipped'].mean():.1f} % of the time"
        f"\n  outside X(lam)  : {100.0 * log['in_violation'].mean():.1f} % of the time"
    )

    fig, art = build_figure(log, plt)
    update = make_updater(log, art)

    if to_video:
        PLOTS_DIR.mkdir(exist_ok=True)
        out = PLOTS_DIR / "live_flight.mp4"
        anim = FuncAnimation(fig, update, frames=frames, blit=False)
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
        plt.close(fig)
        print(f"\nWrote {out}")
    else:
        player = Player(fig, plt, log, update, args.speed, args.fps)  # noqa: F841
        fig.canvas.manager.set_window_title("OSE - live flight")
        print(
            "\n  controls: space play/pause, left/right step, home/end,"
            "\n            up/down speed, or drag the time bar"
            "\n\nClose the window to exit."
        )
        plt.show()


if __name__ == "__main__":
    main()
