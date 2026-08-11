"""
Exercises VehicleGuidance in closed loop against the vehicle model.

Feeds guidance a perfect OwnStateEstimate constructed directly from truth --
this demo isolates guidance's own behaviour, not navigation error. For the
navigation estimator's own error growth and outage behaviour, see
demo_navigation.py; composing the two (real Imu/GnssReceiver/InsGnssEstimator
feeding real guidance) is future work once the single-ship layer exists to
supply real setpoints.

The setpoint schedule is deliberately three segments:

  1. hold the initial heading and speed -- guidance should command close to
     zero turn rate and steady thrust, no unnecessary control action.
  2. a moderate turn and speed increase -- converges without saturating.
  3. a much larger turn -- saturates the turn rate at first, a visible
     Saturation finding rather than a silent clip (ADR 0006), then recovers
     as the heading error shrinks back within capability.

Produces, in plots/ alongside this script:

  vehicle_guidance.png   heading and airspeed tracking against their
                         setpoints, and commanded vs delivered turn rate
                         and thrust against the vehicle's own bounds.

Run with:  python demo_vehicle_guidance.py
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ose.equipment.fuel_gauge import FuelGauge
from ose.equipment.reference_configs.reference_fuel_gauge import (
    STANDARD as FUEL_GAUGE_STANDARD,
)
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
from ose.equipment.vehicle import VehicleState
from ose.integration import step_rk4
from ose.interfaces import HeadingSpeedSetpoint, OwnStateEstimate
from ose.subsystem.reference_configs.reference_vehicle_guidance import STANDARD
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    BELIEVED_TSFC_KG_PER_N_S,
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_guidance import VehicleGuidance
from ose.subsystem.vehicle_manager import VehicleManager

DT = 0.02
T_END = 150.0
TURN_1 = (30.0, 70.0)     # moderate turn + speed change, no saturation expected
TURN_2 = (70.0, 110.0)    # large turn, saturates the turn rate at first
PLOTS_DIR = Path(__file__).resolve().parent / "plots"


def setpoint_at(t: float, psi0: float, v0: float) -> HeadingSpeedSetpoint:
    if t < TURN_1[0]:
        return HeadingSpeedSetpoint(psi0, v0)
    if t < TURN_2[0]:
        return HeadingSpeedSetpoint(math.radians(60.0), 300.0)
    return HeadingSpeedSetpoint(math.radians(-100.0), 300.0)


def perfect_estimate(t_s: float, state: VehicleState) -> OwnStateEstimate:
    """Stands in for a real navigation solution. See module docstring."""
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


def run():
    vehicle = reference_fighter()
    gauge = FuelGauge(
        FUEL_GAUGE_STANDARD, vehicle.lam.mass_dry_kg, np.random.default_rng(7)
    )
    manager = VehicleManager(vehicle, MANAGER_STANDARD)
    guidance = VehicleGuidance(manager, STANDARD)

    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    psi0, v0 = state.psi_rad, state.v_mps
    violations: set[str] = set()

    log = {k: [] for k in (
        "t", "psi", "psi_cmd", "v", "v_cmd",
        "omega_raw_deg_s", "omega_applied_deg_s", "omega_max_deg_s",
        "thrust_raw_kN", "thrust_applied_kN", "thrust_avail_kN",
        "omega_clipped", "thrust_clipped",
    )}

    t = 0.0
    while t < T_END:
        # The fuel gauge is the only truth-reading component in this loop; the
        # manager consumes what it publishes and guidance flies on the
        # believed mass that results.
        if gauge.due(t):
            manager.ingest(gauge.sample(t, state))

        setpoint = setpoint_at(t, psi0, v0)
        own_state = perfect_estimate(t, state)
        cmd, sat = guidance.command(t, setpoint, own_state)

        # What guidance asked for, before enforcement. Reported by Saturation
        # rather than recomputed here: this used to be a copy of guidance's
        # control law, and it went stale the first time that law changed.
        requested = sat.requested

        log["t"].append(t)
        log["psi"].append(math.degrees(state.psi_rad))
        log["psi_cmd"].append(math.degrees(setpoint.psi_cmd_rad))
        log["v"].append(state.v_mps)
        log["v_cmd"].append(setpoint.v_cmd_mps)
        log["omega_raw_deg_s"].append(math.degrees(requested.omega_rad_s))
        log["omega_applied_deg_s"].append(math.degrees(cmd.omega_rad_s))
        log["omega_max_deg_s"].append(
            math.degrees(vehicle.omega_max_rad_s(state.v_mps, state.mass_kg))
        )
        log["thrust_raw_kN"].append(requested.thrust_N / 1e3)
        log["thrust_applied_kN"].append(cmd.thrust_N / 1e3)
        log["thrust_avail_kN"].append(vehicle.thrust_available_N(state) / 1e3)
        log["omega_clipped"].append(sat.omega_clipped)
        log["thrust_clipped"].append(sat.thrust_clipped)

        # Propagate the mass belief over the same interval the vehicle is
        # about to fly, burning at the thrust actually commanded. Without it
        # the filter would grow steadily more confident in a fuel figure that
        # is falling underneath it.
        manager.predict(t + DT, cmd.thrust_N, BELIEVED_TSFC_KG_PER_N_S)
        state = step_rk4(vehicle, state, cmd, DT)
        # X(lambda) is not enforced by anything -- project_command() keeps the
        # input admissible, nothing keeps the state so. Report rather than hide.
        for note in vehicle.state_violations(state):
            violations.add(note.split(" ")[0])
        t += DT

    out = {k: np.array(v) for k, v in log.items()}
    out['_violations'] = violations
    return out


def plot(log, path: Path) -> None:
    t = log["t"]
    fig, axes = plt.subplots(4, 1, figsize=(10.0, 11.0), sharex=True)

    def shade_clipped(ax, clipped, color):
        spans = t[clipped]
        if spans.size:
            ax.axvspan(spans.min(), spans.max(), color=color, alpha=0.12, zorder=0)

    ax = axes[0]
    shade_clipped(ax, log["omega_clipped"], "C1")
    ax.plot(t, log["psi_cmd"], color="0.4", ls=":", lw=1.2, label="commanded")
    ax.plot(t, log["psi"], color="C0", lw=1.4, label="true")
    ax.set_ylabel("heading [deg]")
    ax.legend(frameon=False, loc="upper left", ncol=2)
    ax.grid(alpha=0.25)
    ax.set_title("Vehicle guidance: tracking and enforcement, shaded region is turn-rate saturation")

    ax = axes[1]
    ax.plot(t, log["v_cmd"], color="0.4", ls=":", lw=1.2, label="commanded")
    ax.plot(t, log["v"], color="C0", lw=1.4, label="true")
    ax.set_ylabel("airspeed [m/s]")
    ax.legend(frameon=False, loc="upper left", ncol=2)
    ax.grid(alpha=0.25)

    ax = axes[2]
    shade_clipped(ax, log["omega_clipped"], "C1")
    ax.plot(t, log["omega_max_deg_s"], color="0.5", lw=1.0, ls="--")
    ax.plot(t, -log["omega_max_deg_s"], color="0.5", lw=1.0, ls="--", label="vehicle limit")
    ax.plot(t, log["omega_raw_deg_s"], color="C3", lw=1.0, ls=":", label="requested")
    ax.plot(t, log["omega_applied_deg_s"], color="C0", lw=1.4, label="delivered")
    ax.set_ylabel("turn rate [deg/s]")
    ax.legend(frameon=False, loc="upper left", ncol=3, fontsize=8)
    ax.grid(alpha=0.25)

    ax = axes[3]
    ax.plot(t, log["thrust_avail_kN"], color="0.5", lw=1.0, ls="--", label="vehicle limit")
    ax.plot(t, log["thrust_raw_kN"], color="C3", lw=1.0, ls=":", label="requested")
    ax.plot(t, log["thrust_applied_kN"], color="C0", lw=1.4, label="delivered")
    ax.set_ylabel("thrust [kN]")
    ax.set_xlabel("time [s]")
    ax.legend(frameon=False, loc="upper left", ncol=3, fontsize=8)
    ax.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    log = run()

    def settled_error(t_start, t_end):
        mask = (log["t"] >= t_start) & (log["t"] < t_end)
        return log["psi"][mask][-1] - log["psi_cmd"][mask][-1]

    print("Vehicle guidance")
    print("-" * 62)
    print(f"{'segment':>22} {'ends at heading err [deg]':>28}")
    print(f"{'hold':>22} {settled_error(0.0, TURN_1[0]):>28.2f}")
    print(f"{'moderate turn':>22} {settled_error(*TURN_1):>28.2f}")
    print(f"{'large turn':>22} {settled_error(TURN_2[0], T_END):>28.2f}")

    frac_omega_clipped = 100.0 * log["omega_clipped"].mean()
    frac_thrust_clipped = 100.0 * log["thrust_clipped"].mean()
    print(
        f"\n  time with turn-rate saturated : {frac_omega_clipped:6.1f} %"
        f"\n  time with thrust saturated    : {frac_thrust_clipped:6.1f} %"
        f"\n  final heading                 : {log['psi'][-1]:6.1f} deg"
        f"\n  final airspeed                : {log['v'][-1]:6.1f} m/s"
        f"\n  state left X(lambda)          : "
        f"{', '.join(sorted(log['_violations'])) if log['_violations'] else 'never'}"
    )

    PLOTS_DIR.mkdir(exist_ok=True)
    out_path = PLOTS_DIR / "vehicle_guidance.png"
    plot(log, out_path)
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
