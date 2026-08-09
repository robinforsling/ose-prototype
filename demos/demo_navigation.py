"""
Exercises the INS/GNSS navigation estimator against the vehicle model.

Constructs the four components explicitly -- Imu, GnssReceiver, AirDataSensor,
InsGnssEstimator -- and drives them through the ordering contract described in
docs/refactor-navigation-split.md: corrections at t, then the published
estimate for t, then prediction to t + dt.

The vehicle flies a profile with straight and turning segments in a steady
wind. GNSS is denied between t = 150 s and t = 300 s. Produces:

  navigation_errors.png   estimation errors against their 3-sigma bounds,
                          and the wind estimate converging on truth.

Run with:  python demo_navigation.py
"""

from __future__ import annotations

import math

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ose.resource.air_data import AirDataParameters
from ose.resource.air_data import AirDataSensor as AirDataSensorImpl
from ose.resource.gnss import GnssParameters, GnssReceiver
from ose.resource.imu import Imu, ImuParameters
from ose.resource.vehicle import (
    Disturbance,
    VehicleCommand,
    VehicleState,
    reference_fighter,
    step_rk4,
)
from ose.subsystem.navigation_state_estimator import InitialUncertainty, InsGnssEstimator

DT = 0.02
T_END = 480.0
OUTAGE = (150.0, 300.0)
TRUE_WIND = np.array([12.0, -18.0])          # north, east [m/s]
RUN_SEED = 20260808


def guidance(t: float, vehicle, state: VehicleState) -> VehicleCommand:
    """Open-loop profile: straight, right turn, straight, left turn, straight."""
    omega_cmd = 0.0
    if 60.0 <= t < 120.0:
        omega_cmd = vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg)
    elif 200.0 <= t < 260.0:
        omega_cmd = -0.6 * vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg)
    elif 340.0 <= t < 400.0:
        omega_cmd = 0.8 * vehicle.omega_sustained_rad_s(state.v_mps, state.mass_kg)

    thrust = vehicle.thrust_required_N(state.v_mps, state.mass_kg, omega_cmd)
    cmd = VehicleCommand(thrust, omega_cmd)
    applied, _ = vehicle.project_command(state, cmd)
    return applied


def run():
    # Each component derives its own stream from one run seed, per ADR 0005.
    imu_seed, gnss_seed, air_seed, init_seed = np.random.SeedSequence(RUN_SEED).spawn(4)
    vehicle = reference_fighter()

    state = VehicleState(0.0, 0.0, math.radians(30.0), 250.0, 16000.0)
    disturbance = Disturbance(
        wind_x_mps=float(TRUE_WIND[0]), wind_y_mps=float(TRUE_WIND[1])
    )

    imu = Imu(ImuParameters(), np.random.default_rng(imu_seed), vehicle)
    gnss = GnssReceiver(GnssParameters(), rng=np.random.default_rng(gnss_seed))
    air = AirDataSensorImpl(AirDataParameters(), rng=np.random.default_rng(air_seed))

    # The initial guess handed to the estimator: truth corrupted by an
    # unmodelled alignment process. Producing it is scenario setup, not
    # something the estimator does -- it never sees truth. See ADR 0009.
    initial = InitialUncertainty()
    init_rng = np.random.default_rng(init_seed)
    psi0 = state.psi_rad + float(init_rng.normal(0.0, initial.heading_sigma_rad))
    p0 = np.array([state.p_x_m, state.p_y_m]) + init_rng.normal(
        0.0, initial.position_sigma_m, size=2
    )
    v0 = state.v_mps * np.array([math.cos(psi0), math.sin(psi0)]) + init_rng.normal(
        0.0, initial.velocity_sigma_mps, size=2
    )
    estimator = InsGnssEstimator(p0, psi0, v0, initial_uncertainty=initial)

    log = {k: [] for k in (
        "t", "e_pos", "s_pos", "e_psi", "s_psi", "e_v", "s_v",
        "wx", "wy", "gnss", "true_x", "true_y", "est_x", "est_y",
    )}

    t = 0.0
    while t < T_END:
        gnss.set_gnss_available(not (OUTAGE[0] <= t < OUTAGE[1]))

        cmd = guidance(t, vehicle, state)

        imu_m = imu.sample(t, DT, state, cmd, disturbance)
        fix = gnss.sample(t, state, disturbance) if gnss.due(t) else None
        air_m = air.sample(t, state) if air.due(t) else None

        if fix is not None:
            estimator.ingest(fix)
        if air_m is not None:
            estimator.ingest(air_m)
        est = estimator.estimate(t)                # the published estimate refers to t
        estimator.ingest(imu_m)                     # prediction to t + dt

        e_pos = math.hypot(est.p_x_m - state.p_x_m, est.p_y_m - state.p_y_m)
        e_psi = math.remainder(est.psi_rad - state.psi_rad, 2.0 * math.pi)
        e_v = est.v_air_mps - state.v_mps

        log["t"].append(t)
        log["e_pos"].append(e_pos)
        log["s_pos"].append(est.position_sigma_m)
        log["e_psi"].append(math.degrees(e_psi))
        log["s_psi"].append(math.degrees(math.sqrt(max(est.covariance[2, 2], 0.0))))
        log["e_v"].append(e_v)
        log["s_v"].append(math.sqrt(max(est.covariance[3, 3], 0.0)))
        log["wx"].append(est.wind_estimate_mps[0])
        log["wy"].append(est.wind_estimate_mps[1])
        log["gnss"].append(est.gnss_available)
        log["true_x"].append(state.p_x_m)
        log["true_y"].append(state.p_y_m)
        log["est_x"].append(est.p_x_m)
        log["est_y"].append(est.p_y_m)

        state = step_rk4(vehicle, state, cmd, DT, disturbance)
        t += DT

    return {k: np.array(v) for k, v in log.items()}, gnss.n_fixes


def plot(log, path: str) -> None:
    t = log["t"]
    fig, axes = plt.subplots(4, 1, figsize=(10.0, 10.5), sharex=True)

    def shade(ax):
        ax.axvspan(*OUTAGE, color="0.85", zorder=0)

    ax = axes[0]
    shade(ax)
    ax.plot(t, 3.0 * log["s_pos"], color="C1", lw=1.2, label=r"3$\sigma$ bound")
    ax.plot(t, log["e_pos"], color="C0", lw=1.2, label="position error")
    ax.set_ylabel("position [m]")
    ax.set_yscale("log")
    ax.legend(frameon=False, loc="upper left", ncol=2)
    ax.grid(alpha=0.25, which="both")
    ax.set_title("INS/GNSS navigation error, shaded region is GNSS denied")

    ax = axes[1]
    shade(ax)
    ax.plot(t, 3.0 * log["s_psi"], color="C1", lw=1.2)
    ax.plot(t, -3.0 * log["s_psi"], color="C1", lw=1.2)
    ax.plot(t, log["e_psi"], color="C0", lw=1.2)
    ax.set_ylabel("heading [deg]")
    ax.grid(alpha=0.25)

    ax = axes[2]
    shade(ax)
    ax.plot(t, 3.0 * log["s_v"], color="C1", lw=1.2)
    ax.plot(t, -3.0 * log["s_v"], color="C1", lw=1.2)
    ax.plot(t, log["e_v"], color="C0", lw=1.2)
    ax.set_ylabel("airspeed [m/s]")
    ax.grid(alpha=0.25)

    ax = axes[3]
    shade(ax)
    ax.axhline(TRUE_WIND[0], color="0.4", ls=":", lw=1.0)
    ax.axhline(TRUE_WIND[1], color="0.4", ls=":", lw=1.0)
    ax.plot(t, log["wx"], color="C2", lw=1.3, label="north")
    ax.plot(t, log["wy"], color="C3", lw=1.3, label="east")
    ax.set_ylabel("wind estimate [m/s]")
    ax.set_xlabel("time [s]")
    ax.legend(frameon=False, ncol=2)
    ax.grid(alpha=0.25)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    log, n_gnss_fixes = run()

    inside = np.abs(log["e_psi"]) <= 3.0 * log["s_psi"]
    pre = log["t"] < OUTAGE[0]
    during = (log["t"] >= OUTAGE[0]) & (log["t"] < OUTAGE[1])
    post = log["t"] >= OUTAGE[1]

    print("INS/GNSS navigation")
    print("-" * 62)
    print(f"{'segment':>22} {'RMS pos err [m]':>18} {'final 3-sigma [m]':>19}")
    for name, mask in (("aided", pre), ("GNSS denied", during), ("reacquired", post)):
        rms = math.sqrt(float(np.mean(log["e_pos"][mask] ** 2)))
        print(f"{name:>22} {rms:>18.2f} {3 * log['s_pos'][mask][-1]:>19.2f}")

    print(
        f"\n  peak position error during outage : "
        f"{log['e_pos'][during].max():8.1f} m"
        f"\n  heading error within 3-sigma      : {100.0 * inside.mean():8.1f} %"
        f"\n  GNSS fixes received               : {n_gnss_fixes:8d}"
        f"\n  final wind estimate               : "
        f"[{log['wx'][-1]:6.2f}, {log['wy'][-1]:6.2f}] m/s"
        f"\n  true wind                         : "
        f"[{TRUE_WIND[0]:6.2f}, {TRUE_WIND[1]:6.2f}] m/s"
    )

    plot(log, "navigation_errors.png")
    print("\nWrote navigation_errors.png")


if __name__ == "__main__":
    main()
