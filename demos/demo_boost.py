"""
What an afterburner actually buys, and what it costs.

The two-mode vehicle model of section 5 of docs/vehicle/vehicle_model.pdf,
flown so that the counter-intuitive parts are unmissable.

The claim worth testing
-----------------------
Boost does NOT let the aircraft turn tighter. Remark 5.1 of the model document
records that an earlier formulation making the instantaneous turn rate mode
dependent was inconsistent with the aerodynamics: that bound comes from
available lift and structural strength, and an afterburner changes neither.
What boost changes is the SUSTAINED rate -- the turn that can be held without
bleeding airspeed -- because that depends on thrust.

At 250 m/s and 16 t:

    instantaneous limit    20.1 deg/s     identical in both modes
    sustained, nominal     14.7 deg/s
    sustained, boost       18.1 deg/s

So a turn commanded at 16.5 deg/s is flyable in both modes and holdable in
only one.

The mission, and two policies
-----------------------------
  1. cruise at 250 m/s
  2. turn at 16.5 deg/s in NOMINAL            speed bleeds, 250 -> 189 m/s
  3. straight, recover to 250 m/s
  4. the same turn, this time asking for boost

Leg 4 is flown twice, by two platforms differing only in WHEN they ask for
boost -- a planning decision, not the model's:

    naive        ask for the whole leg and let the switching set decide
    hysteresis   stop asking at s = 0.85, do not ask again until s < 0.25

What the demo found
-------------------
The naive policy produces a limit cycle. Boost is granted, the thermal state
fills in tau_h = 30 s, the switching set withdraws it, the dwell time forces a
five-second wait, s decays by only 0.05 in that time, boost is granted again
and lasts under two seconds. Twenty mode changes in one turn, with the
aircraft pinned on its own thermal limit.

It also overshoots that limit, to s = 1.0016. That is the discretisation
showing: the mode is re-evaluated at step boundaries, so s crosses s_max
before anything notices, and with tau_c = 90 s a 0.2 per cent overshoot takes
a minute to decay back under. The overshoot is reported as a state violation
for the whole of that minute, which is correct -- the state really is outside
X_q.

The naive policy is not simply worse, which is what makes it worth showing.
It extracts MORE boost -- 45 s against 25 s -- precisely by living on the
limit. It pays with twenty mode changes, a violated thermal limit and 133 kg
more fuel. Neither the vehicle nor the switching set misbehaves anywhere in
this: the constraints do exactly what they declare, and the policy is bad.
Finding that out is what a demo is for.

What to look at
---------------
  the turns        all three overlaid, translated and rotated onto a common
                   start. Nominal spirals inward as speed decays; the boosted
                   ones hold a wider radius.

  airspeed         bleeds, recovers, holds. The naive trace's sawtooth is the
                   limit cycle.

  turn rate        the instantaneous limit is one line, not two.

  thermal state    the naive policy pinned against s_max; hysteresis
                   sawtoothing well below it.

  boost engaged    requested against delivered. Twenty transitions against
                   two.

  fuel             boost costs 6.0e-5 kg/(N s) against 2.5e-5 nominal.

Live, with the transport controls shared with the other live demos (see
_player.py) -- pause on the moment boost is withdrawn and step through it.
The plan view follows the pair rather than showing the whole track: the
mission spans about 25 km while the turn radii that matter are one to two
kilometres, and a fixed view renders both turns as dots.

Run with:
    python demo_boost.py                live window, or video if headless
    python demo_boost.py --video        force writing plots/boost.mp4
    python demo_boost.py --speed 1      start at real time
"""

from __future__ import annotations

import math
from pathlib import Path

import argparse
import os

import matplotlib
import numpy as np
from _player import CONTROLS_HELP, Player

from ose.equipment.reference_configs.vehicle.planar_point_mass_with_booster import (
    FIGHTER_BOOST_LIMITS,
    reference_boosted_fighter,
)
from ose.equipment.vehicle import BoostState, Mode, VehicleCommand
from ose.integration import rk4_step

DT = 0.05
T_END = 300.0
PLOTS_DIR = Path(__file__).resolve().parent / "plots"

CRUISE_MPS = 250.0
# Between the nominal sustained rate (14.7 deg/s) and the boosted one
# (18.1 deg/s) at the cruise condition, and below the instantaneous limit
# (20.1 deg/s) which is the same in both. Chosen so the two legs differ for
# the reason the demo is about, and not because one of them was unflyable.
TURN_DEG_S = 16.5
# Proportional speed hold on the straight legs, so both turns start
# from the same condition.
SPEED_GAIN_PER_S = 0.08

NOMINAL_TURN = (20.0, 80.0)
BOOSTED_TURN = (170.0, 260.0)       # longer than tau_h = 30 s, deliberately


# The two mode policies. Both fly the identical mission and differ only in
# when they ask for boost, which is a planning decision and not the model's.
DISENGAGE_AT = 0.85         # hysteresis: stop asking before the limit
RE_ENGAGE_BELOW = 0.25      # and do not ask again until well recovered


def ask_always(thermal: float, asking: bool) -> bool:
    """Naive: ask for boost for the whole leg and let S_q sort it out."""
    return True


def ask_with_hysteresis(thermal: float, asking: bool) -> bool:
    """Stop asking before the limit, and stay quiet until properly cool."""
    if asking:
        return thermal < DISENGAGE_AT
    return thermal < RE_ENGAGE_BELOW


def _turning(t_s: float) -> tuple[float, bool]:
    """The mission: commanded turn rate, and whether this leg wants boost."""
    if NOMINAL_TURN[0] <= t_s < NOMINAL_TURN[1]:
        return math.radians(TURN_DEG_S), False
    if BOOSTED_TURN[0] <= t_s < BOOSTED_TURN[1]:
        return math.radians(TURN_DEG_S), True
    return 0.0, False


def fly(policy) -> dict[str, np.ndarray]:
    vehicle = reference_boosted_fighter()
    state = BoostState(0.0, 0.0, 0.0, CRUISE_MPS, 16000.0, 0.0, Mode.NOMINAL)

    log: dict[str, list] = {k: [] for k in (
        "t", "p_x", "p_y", "psi", "v", "thermal", "fuel", "omega_cmd", "omega_av",
        "omega_sus", "thrust", "mode_boost", "wanted_boost", "denied",
        "violations",
    )}
    last_change_s = -math.inf
    asking = False
    t = 0.0

    while t < T_END:
        omega_cmd, leg_wants_boost = _turning(t)
        asking = leg_wants_boost and policy(state.thermal, asking)
        wanted = Mode.BOOST if asking else Mode.NOMINAL

        # Straight legs drive the speed back to cruise; turns use everything
        # the current mode has. Both turns must START from the same speed or
        # the comparison means nothing -- an earlier version held whatever
        # speed the straight leg inherited, so turn 2 began 60 m/s slower than
        # turn 1 and the two were not comparable at all.
        if omega_cmd == 0.0:
            error = CRUISE_MPS - state.v_mps
            thrust = (
                vehicle.thrust_required_N(state.v_mps, state.mass_kg, 0.0)
                + state.mass_kg * SPEED_GAIN_PER_S * error
            )
            thrust = min(max(thrust, FIGHTER_BOOST_LIMITS.nominal.thrust_min_N),
                         vehicle.capability(state).thrust_available_N)
        else:
            thrust = vehicle.capability(state).thrust_available_N

        cmd, saturation = vehicle.project_command(
            state,
            VehicleCommand(thrust, omega_cmd, mode=wanted),
            since_transition_s=t - last_change_s,
        )
        if cmd.mode is not state.mode:
            last_change_s = t
            state = BoostState(state.p_x_m, state.p_y_m, state.psi_rad,
                               state.v_mps, state.mass_kg, state.thermal,
                               cmd.mode)
            # The thrust ceiling moved with the mode, so re-ask.
            if omega_cmd != 0.0:
                cmd = VehicleCommand(
                    vehicle.capability(state).thrust_available_N,
                    cmd.omega_rad_s, mode=state.mode,
                )

        capability = vehicle.capability(state)
        log["t"].append(t)
        log["p_x"].append(state.p_x_m)
        log["p_y"].append(state.p_y_m)
        log["psi"].append(state.psi_rad)
        log["v"].append(state.v_mps)
        log["thermal"].append(state.thermal)
        log["fuel"].append(state.mass_kg - vehicle.dry_mass_kg)
        log["omega_cmd"].append(math.degrees(omega_cmd))
        log["omega_av"].append(math.degrees(capability.omega_available_rad_s))
        log["omega_sus"].append(math.degrees(capability.omega_sustained_rad_s))
        log["thrust"].append(cmd.thrust_N / 1e3)
        log["mode_boost"].append(state.mode is Mode.BOOST)
        log["wanted_boost"].append(wanted is Mode.BOOST)
        log["denied"].append(wanted is Mode.BOOST and state.mode is not Mode.BOOST)
        log["violations"].append(bool(vehicle.state_violations(state)))

        def f(x, _mode=state.mode, _cmd=cmd):
            return vehicle.derivative(BoostState.from_array(x, _mode), _cmd)

        state = BoostState.from_array(
            rk4_step(f, state.to_array(), DT, normalise=vehicle.normalise_state),
            state.mode,
        )
        t += DT

    return {k: np.asarray(v) for k, v in log.items()}


RUNS = (("naive", ask_always, "C3"), ("hysteresis", ask_with_hysteresis, "C0"))

# The plan view follows the pair rather than showing the whole track. The
# mission spans about 25 km while the features that matter -- turn radii --
# are one to two kilometres, and a fixed view of the lot renders both turns
# as dots. A window this wide keeps them legible throughout.
VIEW_SPAN_KM = 7.0


def build_figure(runs, plt):
    """Static scaffolding. Everything the animation moves is returned in
    `art`, exactly as in demo_live_flight and demo_live_route."""
    t = runs["naive"]["t"]
    fig = plt.figure(figsize=(15.0, 10.0))
    gs = fig.add_gridspec(5, 2, width_ratios=[1.0, 1.15], hspace=0.5, wspace=0.2)
    art: dict[str, object] = {}

    # ---- plan view, following the aircraft -------------------------------
    ax = fig.add_subplot(gs[:, 0])
    for name, _, colour in RUNS:
        log = runs[name]
        ax.plot(log["p_y"] / 1e3, log["p_x"] / 1e3, color=colour, lw=0.8,
                alpha=0.25, zorder=1)
        art[f"trail_{name}"] = ax.plot([], [], color=colour, lw=2.0, zorder=3)[0]
        art[f"ship_{name}"] = ax.plot([], [], "o", color=colour, ms=9, zorder=5,
                                      label=f"{name} policy")[0]
        art[f"nose_{name}"] = ax.plot([], [], "-", color=colour, lw=2.0, zorder=4)[0]
    ax.set_aspect("equal")
    ax.set_xlabel("east, $p_y$ [km]")
    ax.set_ylabel("north, $p_x$ [km]")
    ax.set_title("One mission, two boost policies\n"
                 "the view follows the pair; faint lines are the whole track",
                 fontsize=10)
    ax.legend(frameon=False, fontsize=8, loc="upper right")
    ax.grid(alpha=0.25)
    art["view"] = ax
    art["readout"] = ax.text(
        0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
        family="monospace", fontsize=9,
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.85, edgecolor="0.8"),
    )

    def panel(row, ylabel, title=None):
        a = fig.add_subplot(gs[row, 1])
        a.axvspan(*NOMINAL_TURN, color="0.85", alpha=0.6, linewidth=0, zorder=0)
        a.axvspan(*BOOSTED_TURN, color="0.85", alpha=0.6, linewidth=0, zorder=0)
        a.set_ylabel(ylabel); a.set_xlim(0.0, t[-1]); a.grid(alpha=0.25)
        if title:
            a.set_title(title, fontsize=9)
        cursor = a.axvline(0.0, color="k", lw=1.2)
        return a, cursor

    a, art["cur0"] = panel(
        0, "airspeed [m/s]",
        "The payoff: the same turn bleeds speed in nominal and holds it in boost")
    for name, _, colour in RUNS:
        a.plot(t, runs[name]["v"], color=colour, lw=1.4, label=name)
    a.axhline(CRUISE_MPS, color="0.5", ls="--", lw=1.0)
    a.legend(frameon=False, fontsize=7, ncol=2, loc="lower right")

    a, art["cur1"] = panel(
        1, "turn rate [deg/s]",
        "The instantaneous limit is ONE line: it does not move with mode "
        "(remark 5.1)")
    log = runs["hysteresis"]
    a.plot(t, log["omega_av"], color="0.45", ls="--", lw=1.2,
           label="instantaneous limit")
    a.plot(t, log["omega_sus"], color="C2", lw=1.4, label="sustained, this mode")
    a.plot(t, log["omega_cmd"], color="C3", ls=":", lw=1.6, label="commanded")
    a.legend(frameon=False, fontsize=7, ncol=3, loc="lower left")

    a, art["cur2"] = panel(
        2, "thermal state $s$",
        "Asking for boost whenever it is allowed pins the aircraft on its own "
        "limit")
    for name, _, colour in RUNS:
        a.plot(t, runs[name]["thermal"], color=colour, lw=1.5)
    a.axhline(FIGHTER_BOOST_LIMITS.thermal_max, color="k", ls="-.", lw=1.2)
    a.set_ylim(-0.05, 1.15)

    a, art["cur3"] = panel(3, "boost engaged")
    for k, (name, _, colour) in enumerate(RUNS):
        a.plot(t, runs[name]["mode_boost"].astype(float) * (1 - 0.06 * k),
               color=colour, lw=1.6)
    a.set_yticks([0, 1]); a.set_yticklabels(["nom", "boost"]); a.set_ylim(-0.15, 1.3)

    a, art["cur4"] = panel(4, "fuel [kg]")
    for name, _, colour in RUNS:
        a.plot(t, runs[name]["fuel"], color=colour, lw=1.4)
    a.set_xlabel("time [s]")

    fig.subplots_adjust(left=0.05, right=0.975, top=0.92, bottom=0.11)
    return fig, art


def make_updater(runs, art):
    """One frame. Kept separate from the figure so live and video share it."""
    t = runs["naive"]["t"]
    span = VIEW_SPAN_KM / 2.0
    nose = VIEW_SPAN_KM * 0.05

    def update(i):
        ys, xs = [], []
        for name, _, _ in RUNS:
            log = runs[name]
            y, x = log["p_y"] / 1e3, log["p_x"] / 1e3
            art[f"trail_{name}"].set_data(y[: i + 1], x[: i + 1])
            art[f"ship_{name}"].set_data([y[i]], [x[i]])
            psi = log["psi"][i]
            art[f"nose_{name}"].set_data(
                [y[i], y[i] + nose * math.sin(psi)],
                [x[i], x[i] + nose * math.cos(psi)],
            )
            ys.append(y[i]); xs.append(x[i])

        # Follow the pair. A fixed view of the whole 25 km track renders both
        # turns as dots; the features worth watching are a kilometre across.
        cy, cx = sum(ys) / len(ys), sum(xs) / len(xs)
        art["view"].set_xlim(cy - span, cy + span)
        art["view"].set_ylim(cx - span, cx + span)

        lines = [f"t      {t[i]:7.1f} s"]
        for name, _, _ in RUNS:
            log = runs[name]
            flag = "BOOST" if log["mode_boost"][i] else "nom  "
            if log["denied"][i]:
                flag = "DENIED"
            lines.append(
                f"{name[:10]:<10} {log['v'][i]:5.0f} m/s  s={log['thermal'][i]:4.2f}  "
                f"{flag}"
            )
        if runs["naive"]["violations"][i]:
            lines.append("naive: s ABOVE s_max")
        art["readout"].set_text("\n".join(lines))

        for k in range(5):
            art[f"cur{k}"].set_xdata([t[i], t[i]])
        return tuple(art.values())

    return update


def _summary(runs):
    t = runs["naive"]["t"]
    turn = (t >= BOOSTED_TURN[0]) & (t < BOOSTED_TURN[1])
    nom_turn = (t >= NOMINAL_TURN[0]) & (t < NOMINAL_TURN[1])

    vehicle = reference_boosted_fighter()
    inst = math.degrees(vehicle.omega_max_rad_s(CRUISE_MPS, 16000.0))
    sus = [math.degrees(vehicle.omega_sustained_rad_s(CRUISE_MPS, 16000.0, m))
           for m in (Mode.NOMINAL, Mode.BOOST)]

    print("Boost: what it buys and what it costs")
    print("-" * 70)
    print(f"  commanded turn rate : {TURN_DEG_S:.1f} deg/s, in both turns")
    print(f"  at {CRUISE_MPS:.0f} m/s and 16 t, nominal / boost:")
    print(f"    instantaneous limit : {inst:5.2f} / {inst:5.2f} deg/s   <- identical,")
    print("                          and it is one call: the model takes no mode here")
    print(f"    sustained rate      : {sus[0]:5.2f} / {sus[1]:5.2f} deg/s   <- the channel")
    print("                          boost actually moves")

    v = runs["naive"]["v"]
    print(f"\n  turn 1, nominal throughout: {v[nom_turn][0]:.0f} -> "
          f"{v[nom_turn][-1]:.0f} m/s, {v[nom_turn][0] - v[nom_turn][-1]:+.0f} bled")

    print(f"\n{'policy':<12} {'boost held':>11} {'switches':>9} {'peak s':>8} "
          f"{'violations':>11} {'fuel':>8} {'exit speed':>11}")
    for name, _, _ in RUNS:
        log = runs[name]
        b = log["mode_boost"]
        print(f"{name:<12} {b[turn].sum() * DT:>9.1f} s "
              f"{int(np.count_nonzero(np.diff(b.astype(int)))):>9} "
              f"{log['thermal'].max():>8.4f} "
              f"{int(log['violations'].sum()):>10} s "
              f"{log['fuel'][0] - log['fuel'][-1]:>6.0f} kg "
              f"{log['v'][turn][-1]:>8.0f} m/s")

    print("""
  Asking for boost whenever the switching set allows it is not free. It
  extracts more boost -- by living on the thermal limit -- and pays for it
  with twenty mode changes, a thermal state that overshoots s_max, and
  noticeably more fuel. The overshoot is the discretisation: the mode is
  re-evaluated at step boundaries, so s passes the limit before anything
  notices, and with tau_c = 90 s that 0.2 per cent takes a minute to decay
  back under. None of this is the model misbehaving. The constraints are
  doing exactly what they declare; the naive policy is simply a bad one, and
  a demo is how you find that out.""")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--video", action="store_true", help="write mp4 instead of a window")
    ap.add_argument("--speed", type=float, default=8.0, help="initial playback speed-up")
    ap.add_argument("--fps", type=int, default=25)
    args = ap.parse_args()

    headless = not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    to_video = args.video or headless
    matplotlib.use("Agg" if to_video else "TkAgg", force=True)
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter, FuncAnimation

    runs = {name: fly(policy) for name, policy, _ in RUNS}
    _summary(runs)

    stride = max(1, int(round((1.0 / args.fps) * args.speed / DT)))
    frames = range(0, len(runs["naive"]["t"]), stride)

    fig, art = build_figure(runs, plt)
    update = make_updater(runs, art)

    PLOTS_DIR.mkdir(exist_ok=True)
    if to_video:
        out = PLOTS_DIR / "boost.mp4"
        anim = FuncAnimation(fig, update, frames=frames, blit=False)
        anim.save(out, writer=FFMpegWriter(fps=args.fps, bitrate=2400))
        plt.close(fig)
        print(f"\nWrote {out}")
    else:
        player = Player(fig, runs["naive"], update, args.speed, args.fps, DT)  # noqa: F841
        fig.canvas.manager.set_window_title("OSE - boost")
        print("\n" + CONTROLS_HELP + "\n\nClose the window to exit.")
        plt.show()


if __name__ == "__main__":
    main()
