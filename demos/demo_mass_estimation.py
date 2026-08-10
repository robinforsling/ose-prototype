"""
The platform's belief about its own mass, and what it promises on the
strength of it.

The mass analogue of demo_navigation.py. That one flies one trajectory and
shows an INS/GNSS solution degrading through a GNSS outage; this one flies one
trajectory and shows three differently-equipped platforms estimating the same
fuel load, so the question is not "does the filter work" but "what is the
belief worth, and what should be promised on it".

Three platforms, one truth
--------------------------
All three fly the identical trajectory -- fixed thrust, straight and level at
about 170 m/s -- and differ only in what they are told about their own fuel:

  gauge on     a working 1 Hz fuel gauge throughout.
  outage       the same gauge, dead from t = 120 s to t = 520 s. The filter
               keeps dead-reckoning on commanded thrust with nothing to
               correct against, exactly as the navigation filter coasts
               through a GNSS outage.
  no gauge     no fuel gauge at all, which is a legitimate configuration and
               not a failure. The platform knows only what it was loaded
               with, to the tolerance it declared.

The truth is loaded 260 kg ABOVE the nominal 4 000 kg and nobody tells the
platform. Without that offset the no-gauge case would start out exactly right
by accident, its 200 kg sigma would look like pure pessimism, and the plots
would flatter it. An estimator demonstrated on an initial condition equal to
its own prior mean is demonstrating nothing.

What to look at
---------------
  fuel            truth against each belief, with its own three-sigma band.
                  The band is the claim; the black line staying inside it is
                  the claim being kept.

  sigma           the sawtooth -- growing while predicting, dropping at each
                  reading -- then unbounded growth through the outage, then
                  the collapse when the gauge returns. Log scale, because the
                  three platforms are two orders of magnitude apart.

  error / sigma   the honesty check, and the only panel that can fail. If a
                  trace leaves the three-sigma lines the filter is claiming
                  more than it knows. Note the no-gauge trace sits near minus
                  one and stays there: it is wrong by 260 kg and says so.

  margin          what the promise costs, as the percentage by which
                  capability_bound() narrows the reported turn rate against
                  capability(). This is ADR 0016 made visible.

The honest headline
-------------------
The margin is worth 0.03 per cent with a working gauge, 0.2 per cent at the
end of a 400-second outage, and 3.8 per cent with no gauge at all.

That is a small effect and it should be reported as one. Twenty kilograms of
doubt on a fifteen-tonne aircraft is nothing, so a gauge failure barely dents
what this platform can promise -- the aeroplane is simply heavy compared with
how wrong it can be about itself. The margin is not decoration either: with no
gauge it costs four per cent of the turn rate permanently, which a planner
sizing a turn should be told about.

The general lesson is the ratio, not the number. A margin that scales with the
live uncertainty stays out of the way when the platform knows itself and bites
when it does not, without anyone choosing when. Expect it to matter far more
for a light platform, a large fuel fraction, or a released store whose mass is
unknown until it is confirmed gone.

Produces, in plots/ alongside this script:

  mass_estimation.png

Run with:  python demo_mass_estimation.py
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ose.integration import step_rk4
from ose.interfaces import OwnStateEstimate
from ose.resource.fuel_gauge import FuelGauge
from ose.resource.reference_configs.reference_fuel_gauge import (
    STANDARD as FUEL_GAUGE_STANDARD,
)
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import Vehicle2D, VehicleCommand, VehicleState
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_manager import VehicleManager

DT = 0.1
T_END = 640.0
OUTAGE = (120.0, 520.0)
PLOTS_DIR = Path(__file__).resolve().parent / "plots"

# Held roughly constant, and chosen deliberately. At 250 m/s the turn rate is
# limited by the 9 g structural bound, which does not move with mass at all,
# so the margin panel would read zero everywhere and show nothing. Below about
# 220 m/s the limit is lift, which does move with mass. Picking a flight
# condition where the effect can exist at all is part of demonstrating it.
CRUISE_THRUST_N = 19.4e3

# Loaded over nominal, and the platform is not told. See the module docstring.
TRUE_FUEL_EXCESS_KG = 260.0

# The vehicle's real burn coefficient is four per cent off the one the manager
# predicts with. The manager declares five per cent of doubt about it
# (tsfc_sigma_fraction), so this is a plausible draw rather than a worst case,
# and it is what makes dead reckoning through the outage drift at all.
TRUE_TSFC_ERROR = 0.04

PLATFORMS = ("gauge on", "outage", "no gauge")


def _gauge_live(platform: str, t_s: float) -> bool:
    if platform == "gauge on":
        return True
    if platform == "no gauge":
        return False
    return not (OUTAGE[0] <= t_s < OUTAGE[1])


def _estimate(t_s: float, state: VehicleState) -> OwnStateEstimate:
    """Perfect navigation. This demo is about the mass belief, so the state
    belief is made exact to keep one source of error in the picture at a
    time -- demo_navigation.py is where the other one lives."""
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


def fly() -> dict:
    nominal = reference_fighter()
    vehicle = Vehicle2D(
        dataclasses.replace(
            nominal.theta,
            c_tsfc=MANAGER_STANDARD.tsfc_kg_per_N_s * (1.0 + TRUE_TSFC_ERROR),
        ),
        nominal.lam,
        nominal.eta,
    )

    # One truth, three estimators. The platforms cannot influence the
    # trajectory -- the thrust command is fixed -- so any difference between
    # them is a difference in belief and nothing else. A closed loop would
    # confound the two.
    state = VehicleState(
        0.0, 0.0, 0.0, 170.0,
        vehicle.lam.mass_dry_kg + MANAGER_STANDARD.initial_fuel_kg
        + TRUE_FUEL_EXCESS_KG,
    )
    command = VehicleCommand(CRUISE_THRUST_N, 0.0)

    managers = {p: VehicleManager(vehicle, MANAGER_STANDARD) for p in PLATFORMS}
    # Each gauge gets its own stream, per ADR 0005: the two platforms that
    # have a gauge must not share one, or adding the third would perturb them.
    seeds = np.random.SeedSequence(11).spawn(len(PLATFORMS))
    gauges = {
        p: FuelGauge(FUEL_GAUGE_STANDARD, vehicle.lam.mass_dry_kg,
                     np.random.default_rng(s))
        for p, s in zip(PLATFORMS, seeds)
    }

    log = {"t": [], "true_fuel": [], "v": []}
    for p in PLATFORMS:
        log[p] = {"fuel": [], "sigma": [], "error": [], "margin": [], "tsfc": []}

    t = 0.0
    while t < T_END:
        estimate = _estimate(t, state)
        true_fuel = state.mass_kg - vehicle.lam.mass_dry_kg

        log["t"].append(t)
        log["true_fuel"].append(true_fuel)
        log["v"].append(state.v_mps)

        for p in PLATFORMS:
            manager, gauge = managers[p], gauges[p]
            if _gauge_live(p, t) and gauge.due(t):
                manager.ingest(gauge.sample(t, state))

            belief = manager.mass(t)
            point = manager.capability(estimate)
            promise = manager.capability_bound(estimate)

            row = log[p]
            row["fuel"].append(belief.fuel_mass_kg)
            row["sigma"].append(belief.mass_sigma_kg)
            row["error"].append(belief.fuel_mass_kg - true_fuel)
            row["margin"].append(
                100.0 * (1.0 - promise.max_turn_rate_rad_s
                         / point.omega_available_rad_s)
            )
            row["tsfc"].append(belief.tsfc_error)

            manager.predict(t + DT, command.thrust_N)

        state = step_rk4(vehicle, state, command, DT)
        t += DT

    out = {"t": np.asarray(log["t"]),
           "true_fuel": np.asarray(log["true_fuel"]),
           "v": np.asarray(log["v"])}
    for p in PLATFORMS:
        out[p] = {k: np.asarray(v) for k, v in log[p].items()}
    return out


def plot(log, path: Path) -> None:
    t = log["t"]
    colours = {"gauge on": "C0", "outage": "C1", "no gauge": "C3"}
    fig, axes = plt.subplots(4, 1, figsize=(11.0, 12.5), sharex=True)

    def shade_outage(ax):
        ax.axvspan(*OUTAGE, color="0.85", alpha=0.55, zorder=0, linewidth=0)

    # -- fuel, with each platform's own three-sigma band -------------------
    ax = axes[0]
    shade_outage(ax)
    for p in PLATFORMS:
        r = log[p]
        # Bands for the gauged platforms only. The no-gauge band is 1200 kg
        # tall and drawing it here compresses everything else into a ribbon,
        # hiding the outage divergence this panel exists to show. Its width is
        # the whole subject of the panel below, where a log axis can hold it.
        if p != "no gauge":
            ax.fill_between(t, r["fuel"] - 3 * r["sigma"], r["fuel"] + 3 * r["sigma"],
                            color=colours[p], alpha=0.2, linewidth=0)
        ax.plot(t, r["fuel"], color=colours[p], lw=1.3, label=f"{p} (belief)")
    ax.plot(t, log["true_fuel"], color="k", lw=1.6, ls="--", label="truth")

    low = min(log["no gauge"]["fuel"].min(), log["true_fuel"].min())
    high = max(log["true_fuel"].max(), log["no gauge"]["fuel"].max())
    pad = 0.12 * (high - low)
    ax.set_ylim(low - pad, high + pad)
    ax.annotate(
        "no-gauge 3-sigma band is +-600 kg, off scale",
        xy=(0.015, 0.06), xycoords="axes fraction", fontsize=8, color=colours["no gauge"],
    )
    ax.set_ylabel("fuel remaining [kg]")
    ax.legend(frameon=False, fontsize=8, ncol=2, loc="upper right")
    ax.grid(alpha=0.25)
    ax.set_title(
        "Mass estimation: one trajectory, three differently-equipped platforms\n"
        "shaded band = gauge outage; coloured bands = each platform's own 3 sigma",
        fontsize=10,
    )

    # -- the uncertainty itself --------------------------------------------
    ax = axes[1]
    shade_outage(ax)
    for p in PLATFORMS:
        ax.plot(t, log[p]["sigma"], color=colours[p], lw=1.3, label=p)
    ax.set_yscale("log")
    ax.set_ylabel("mass sigma [kg]")
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="upper right")
    ax.grid(alpha=0.25, which="both")

    # -- the honesty check --------------------------------------------------
    ax = axes[2]
    shade_outage(ax)
    for p in PLATFORMS:
        r = log[p]
        ax.plot(t, r["error"] / np.maximum(r["sigma"], 1e-9),
                color=colours[p], lw=1.2, label=p)
    for k in (-3.0, 3.0):
        ax.axhline(k, color="0.4", ls="--", lw=1.0)
    ax.set_ylim(-4.0, 4.0)
    ax.set_ylabel("error / sigma")
    ax.grid(alpha=0.25)
    ax.set_title(
        "The only panel that can fail: outside the dashed lines the filter is "
        "claiming more than it knows",
        fontsize=9,
    )

    # -- what the promise costs --------------------------------------------
    ax = axes[3]
    shade_outage(ax)
    for p in PLATFORMS:
        ax.plot(t, log[p]["margin"], color=colours[p], lw=1.3, label=p)
    ax.set_ylabel("turn rate given up [%]")
    ax.set_xlabel("time [s]")
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="center right")
    ax.grid(alpha=0.25)
    ax.set_title(
        "capability_bound() against capability(): the margin scales with what "
        "the platform actually knows (ADR 0016)",
        fontsize=9,
    )

    for a in axes:
        a.set_xlim(0.0, t[-1])
    fig.tight_layout()
    PLOTS_DIR.mkdir(exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main() -> None:
    log = fly()
    t = log["t"]

    def at(p, key, t_s):
        return log[p][key][int(round(t_s / DT))]

    print("Mass estimation: one truth, three platforms")
    print("-" * 74)
    print(f"  true fuel loaded  : "
          f"{MANAGER_STANDARD.initial_fuel_kg + TRUE_FUEL_EXCESS_KG:.0f} kg "
          f"({TRUE_FUEL_EXCESS_KG:+.0f} kg on nominal, and nobody tells the platform)")
    print(f"  true burn coeff   : {100 * TRUE_TSFC_ERROR:+.0f} % on the one the "
          f"filter predicts with")
    print(f"  flown             : {t[-1]:.0f} s at {log['v'].mean():.0f} m/s, "
          f"{log['true_fuel'][0] - log['true_fuel'][-1]:.0f} kg burned")
    print(f"  gauge outage      : {OUTAGE[0]:.0f}-{OUTAGE[1]:.0f} s\n")

    print(f"{'':<10} {'':<7} {'sigma':>9} {'error':>9} {'err/sig':>8} {'margin':>8}")
    for p in PLATFORMS:
        for label, t_s in (("settled", 119.0), ("outage end", 519.0),
                           ("recovered", 639.0)):
            name = p if label == "settled" else ""
            print(f"{name:<10} {label:<11} {at(p,'sigma',t_s):7.1f}kg "
                  f"{at(p,'error',t_s):+7.1f}kg {at(p,'error',t_s)/at(p,'sigma',t_s):+8.2f} "
                  f"{at(p,'margin',t_s):7.2f}%")
        print()

    worst = max(
        np.abs(log[p]["error"] / np.maximum(log[p]["sigma"], 1e-9)).max()
        for p in PLATFORMS
    )
    print(f"  worst |error|/sigma over all platforms and all time: {worst:.2f}")
    print("  (a single run, so this is an illustration -- the calibrated claim is\n"
          "   test_fuel_estimate_is_consistent_through_the_run, over an ensemble)")

    print(
        "\n  The margin is worth "
        f"{at('gauge on','margin',639.0):.2f} % with a working gauge, "
        f"{at('outage','margin',519.0):.2f} % at the end of a 400 s outage,"
        f"\n  and {at('no gauge','margin',639.0):.2f} % with no gauge at all. Small, and "
        "reported as small: twenty\n  kilograms of doubt on a fifteen-tonne aircraft is "
        "nothing. It scales with what\n  the platform knows, which is the property worth "
        "having."
    )

    out = PLOTS_DIR / "mass_estimation.png"
    plot(log, out)
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
