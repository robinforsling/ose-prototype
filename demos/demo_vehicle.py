"""
Exercises the baseline vehicle model and produces two figures in plots/,
alongside this script:

  1. vehicle_envelope.png -- the turn performance envelope (a doghouse plot),
     instantaneous and sustained turn rate against airspeed.
  2. vehicle_trajectory.png -- an open-loop manoeuvre: accelerate, then a
     sustained-rate turn, then a maximum-rate turn, showing the energy cost.

Run with:  python demo_vehicle.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ose.integration import step_rk4
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import VehicleCommand, VehicleState

PLOTS_DIR = Path(__file__).resolve().parent / "plots"


def print_capability_report(vehicle, mass_kg: float) -> None:
    """What the vehicle says about itself at a few speeds."""
    print(f"\nCapability at m = {mass_kg:.0f} kg\n" + "-" * 72)
    print(
        f"{'v [m/s]':>8} {'n_avail':>8} {'w_inst':>9} {'w_sust':>9} "
        f"{'R_min':>9} {'T_req':>9} {'a_max':>8}"
    )
    print(
        f"{'':>8} {'[-]':>8} {'[deg/s]':>9} {'[deg/s]':>9} "
        f"{'[m]':>9} {'[kN]':>9} {'[m/s2]':>8}"
    )
    for v in (120, 160, 210, 250, 300, 400, 500):
        state = VehicleState(0.0, 0.0, 0.0, float(v), mass_kg)
        c = vehicle.capability(state)
        print(
            f"{v:>8.0f} {c.load_factor_available:>8.2f} "
            f"{math.degrees(c.omega_available_rad_s):>9.2f} "
            f"{math.degrees(c.omega_sustained_rad_s):>9.2f} "
            f"{c.turn_radius_min_m:>9.0f} "
            f"{c.thrust_required_N / 1e3:>9.1f} "
            f"{c.accel_max_mps2:>8.2f}"
        )

    state = VehicleState(0.0, 0.0, 0.0, 250.0, mass_kg)
    c = vehicle.capability(state)
    print(
        f"\n  stall speed (1 g) : {c.v_stall_mps:6.1f} m/s"
        f"\n  stall speed (2 g) : {vehicle.v_stall_mps(mass_kg, 2.0):6.1f} m/s"
        f"\n  stall speed (4 g) : {vehicle.v_stall_mps(mass_kg, 4.0):6.1f} m/s"
        f"\n  corner speed      : {c.v_corner_mps:6.1f} m/s"
        f"\n  fuel remaining    : {c.fuel_mass_kg:6.0f} kg"
        f"\n  endurance at 250  : {c.endurance_s / 60.0:6.1f} min"
    )


def plot_envelope(vehicle, mass_kg: float, path: Path) -> None:
    speeds = np.linspace(60.0, vehicle.lam.v_max_mps, 500)
    inst, sust, stall_mask = [], [], []

    for v in speeds:
        v_stall = vehicle.v_stall_mps(mass_kg)
        stall_mask.append(v < v_stall)
        inst.append(math.degrees(vehicle.omega_max_rad_s(v, mass_kg)))
        sust.append(math.degrees(vehicle.omega_sustained_rad_s(v, mass_kg)))

    inst = np.array(inst)
    sust = np.array(sust)
    valid = ~np.array(stall_mask)

    v_stall = vehicle.v_stall_mps(mass_kg)
    v_corner = vehicle.v_corner_mps(mass_kg)

    fig, ax = plt.subplots(figsize=(8.5, 5.5))
    ax.plot(speeds[valid], inst[valid], lw=2.0, label="instantaneous (lift / structural)")
    ax.plot(speeds[valid], sust[valid], lw=2.0, ls="--", label="sustained (thrust = drag)")
    ax.fill_between(speeds[valid], 0, sust[valid], alpha=0.12)

    ax.axvline(v_stall, color="0.4", lw=1.0, ls=":")
    ax.axvline(v_corner, color="0.4", lw=1.0, ls=":")
    ax.annotate(
        f"stall {v_stall:.0f} m/s",
        (v_stall, ax.get_ylim()[1] * 0.92),
        rotation=90, va="top", ha="right", fontsize=9, color="0.35",
    )
    ax.annotate(
        f"corner {v_corner:.0f} m/s",
        (v_corner, ax.get_ylim()[1] * 0.92),
        rotation=90, va="top", ha="right", fontsize=9, color="0.35",
    )

    ax.set_xlabel("airspeed [m/s]")
    ax.set_ylabel("turn rate [deg/s]")
    ax.set_title(f"Turn performance envelope, m = {mass_kg:.0f} kg, sea level")
    ax.legend(frameon=False)
    ax.grid(alpha=0.25)
    ax.set_xlim(60, vehicle.lam.v_max_mps)
    ax.set_ylim(0, None)
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def simulate(vehicle, dt: float = 0.02):
    """Open loop: accelerate, sustained turn, then maximum-rate turn."""
    state = VehicleState(p_x_m=0.0, p_y_m=0.0, psi_rad=0.0, v_mps=200.0, mass_kg=16000.0)
    log = {k: [] for k in ("t", "x", "y", "v", "n", "omega", "mass", "psi")}
    violations: list[str] = []

    t, t_end = 0.0, 180.0
    while t < t_end:
        if t < 40.0:
            # accelerate wings level at full thrust
            cmd = VehicleCommand(vehicle.lam.thrust_max_N, 0.0)
        elif t < 110.0:
            # sustained-rate turn: hold speed
            omega = vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg)
            cmd = VehicleCommand(vehicle.lam.thrust_max_N, omega)
        else:
            # maximum instantaneous rate: bleeds energy
            omega = vehicle.omega_max_rad_s(state.v_mps, state.mass_kg)
            cmd = VehicleCommand(vehicle.lam.thrust_max_N, omega)

        # The guidance layer -- here, this open-loop script -- is responsible
        # for keeping the command admissible. The vehicle will not do it.
        applied, sat = vehicle.project_command(state, cmd)
        if sat.any:
            violations.extend(sat.notes)
        log["t"].append(t)
        log["x"].append(state.p_x_m)
        log["y"].append(state.p_y_m)
        log["v"].append(state.v_mps)
        log["psi"].append(state.psi_rad)
        log["mass"].append(state.mass_kg)
        log["omega"].append(math.degrees(applied.omega_rad_s))
        log["n"].append(vehicle.load_factor(state.v_mps, applied.omega_rad_s))

        state = step_rk4(vehicle, state, applied, dt)
        violations.extend(vehicle.state_violations(state))
        t += dt

    out = {k: np.array(v) for k, v in log.items()}
    out["_violations"] = violations
    return out


def plot_trajectory(log, path: Path) -> None:
    fig = plt.figure(figsize=(11.5, 5.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1.0], hspace=0.35, wspace=0.25)

    ax0 = fig.add_subplot(gs[:, 0])
    ax0.plot(log["y"] / 1000.0, log["x"] / 1000.0, lw=1.8)
    ax0.plot(log["y"][0] / 1000.0, log["x"][0] / 1000.0, "o", ms=6, label="start")
    ax0.set_xlabel("east, $p_y$ [km]")
    ax0.set_ylabel("north, $p_x$ [km]")
    ax0.set_title("Ground track")
    ax0.set_aspect("equal", adjustable="datalim")
    ax0.grid(alpha=0.25)
    ax0.legend(frameon=False, loc="best")

    ax1 = fig.add_subplot(gs[0, 1])
    ax1.plot(log["t"], log["v"], lw=1.6)
    ax1.set_ylabel("v [m/s]")
    ax1.grid(alpha=0.25)
    ax1.set_title("Airspeed, turn rate, load factor")

    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax2.plot(log["t"], log["omega"], lw=1.6, color="C1")
    ax2.set_ylabel(r"$\omega$ [deg/s]")
    ax2.grid(alpha=0.25)

    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    ax3.plot(log["t"], log["n"], lw=1.6, color="C2")
    ax3.set_ylabel("n [-]")
    ax3.set_xlabel("time [s]")
    ax3.grid(alpha=0.25)

    for ax in (ax1, ax2, ax3):
        ax.axvline(40.0, color="0.6", lw=0.9, ls=":")
        ax.axvline(110.0, color="0.6", lw=0.9, ls=":")

    fig.savefig(path, dpi=140, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    vehicle = reference_fighter()

    print("Reference configuration")
    print("-" * 72)
    print(f"  c_p    = {vehicle.theta.c_p:9.4f}  m^2")
    print(f"  c_i    = {vehicle.theta.c_i:9.4f}  1/s^4")
    print(f"  c_l    = {vehicle.theta.c_l:9.4f}  m^2")
    print(f"  T_max  = {vehicle.lam.thrust_max_N / 1e3:9.1f}  kN")
    print(f"  n_max  = {vehicle.lam.n_structural:9.1f}  -")

    print_capability_report(vehicle, mass_kg=16000.0)

    PLOTS_DIR.mkdir(exist_ok=True)
    envelope_path = PLOTS_DIR / "vehicle_envelope.png"
    trajectory_path = PLOTS_DIR / "vehicle_trajectory.png"

    plot_envelope(vehicle, 16000.0, envelope_path)
    log = simulate(vehicle)
    plot_trajectory(log, trajectory_path)

    print(
        f"\nManoeuvre summary"
        f"\n  speed at t=40 s   : {log['v'][int(40 / 0.02)]:6.1f} m/s"
        f"\n  speed at t=110 s  : {log['v'][int(110 / 0.02)]:6.1f} m/s"
        f"\n  speed at t=180 s  : {log['v'][-1]:6.1f} m/s"
        f"\n  fuel burned       : {log['mass'][0] - log['mass'][-1]:6.0f} kg"
        f"\n  envelope events   : {len(log['_violations'])}"
    )
    print(f"\nWrote {envelope_path} and {trajectory_path}")


if __name__ == "__main__":
    main()
