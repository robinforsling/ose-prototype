#!/usr/bin/env python3
"""
Regenerate the computed tables in docs/models/vehicle/.

    python tools/generate_model_docs.py            rewrite them in place
    python tools/generate_model_docs.py --check    exit non-zero if stale

What this does and does not do
------------------------------
It generates the NUMBERS: parameter tables, limit tables, turn-performance
tables. Those are mechanical, they are the parts most likely to drift when a
model or a reference configuration changes, and nobody should be transcribing
them by hand.

It does not generate the prose, and no version of this should try. The value
of those pages is the behaviour -- which limit binds where, what is
counter-intuitive, what a naive policy does -- and that is a judgement about
what a reader would otherwise get wrong. A generator cannot have that
judgement, and a page assembled entirely from field names would be a worse
version of the source code.

So the pages are part generated and part written, and the split is marked:

    <!-- generated: NAME -->
    ... anything here is overwritten ...
    <!-- end generated: NAME -->

test_model_docs.py runs this with --check, so a model change that moves a
number fails the suite until the tables are regenerated. That is the enforcing
mechanism; the rule in CLAUDE.md is only the explanation.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import re
import sys
from pathlib import Path

from ose.equipment.reference_configs.vehicle.planar_point_mass import (
    FIGHTER_GEOMETRY,
    FIGHTER_LIMITS,
    reference_fighter,
)
from ose.equipment.reference_configs.vehicle.planar_point_mass_with_booster import (
    FIGHTER_BOOST_LIMITS,
    reference_boosted_fighter,
)
from ose.equipment.vehicle import BoostState, Mode, VehicleState

DOCS = Path(__file__).resolve().parents[1] / "docs" / "models" / "vehicle"
MASS_KG = 16000.0
SPEEDS = (100.0, 150.0, 200.0, 225.0, 250.0, 300.0, 400.0, 500.0, 600.0)
BOOST_SPEEDS = (150.0, 200.0, 225.0, 250.0, 300.0, 400.0, 500.0)


def _level_flight_ceiling(vehicle, mass_kg: float, thrust_max_N: float) -> float:
    """Where the thrust required for straight and level flight crosses what is
    available. Bisected rather than solved: the drag polar is quartic in v and
    the root has no form worth writing down."""
    lo, hi = 100.0, 900.0
    for _ in range(80):
        mid = 0.5 * (lo + hi)
        if vehicle.thrust_required_N(mid, mass_kg, 0.0) < thrust_max_N:
            lo = mid
        else:
            hi = mid
    return lo


# --------------------------------------------------------------------------
# Baseline
# --------------------------------------------------------------------------

def theta_table() -> str:
    v = reference_fighter()
    g, t = FIGHTER_GEOMETRY, v.theta
    return "\n".join([
        "| authored | reference value | | lumped | reference value |",
        "|---|---|---|---|---|",
        f"| $S$ | {g.wing_area_m2} m² | | $c_p$ | {t.c_p:.3f} m² |",
        f"| $C_{{D0}}$ | {g.cd0} | | $c_i$ | {t.c_i:.3f} s⁻⁴ |",
        f"| $e$ | {g.oswald_e} | | $c_\\ell$ | {t.c_l:.1f} m² |",
        f"| $A\\!R$ | {g.aspect_ratio} | | $c_{{\\mathrm{{TSFC}}}}$ | "
        f"${t.c_tsfc:.1e}".replace("e-05", r"\times10^{-5}$") + " kg/(N s) |",
        f"| $C_{{L\\max}}$ | {g.cl_max} | | | |",
    ])


def lambda_table() -> str:
    lam = FIGHTER_LIMITS
    notes = {
        "thrust_min_N": ("$T_{\\min}$", f"{lam.thrust_min_N / 1e3:.0f} kN", "idle"),
        "thrust_max_N": ("$T_{\\max}$", f"{lam.thrust_max_N / 1e3:.0f} kN", ""),
        "n_structural": ("$n_{\\max}$", f"{lam.n_structural:.0f}", "structural"),
        "omega_max_rad_s": ("$\\omega_{\\mathrm{cap}}$",
                            f"{math.degrees(lam.omega_max_rad_s):.0f} °/s",
                            "roll and control authority"),
        "v_min_mps": ("$v_{\\min}$", f"{lam.v_min_mps:.0f} m/s",
                      "hard floor, **independent of stall**"),
        "v_max_mps": ("$v_{\\max}$", f"{lam.v_max_mps:.0f} m/s", "airframe"),
        "mass_dry_kg": ("$m_{\\mathrm{dry}}$", f"{lam.mass_dry_kg:,.0f} kg", ""),
        "mass_max_kg": ("$m_{\\max}$", f"{lam.mass_max_kg:,.0f} kg", ""),
    }
    missing = {f.name for f in dataclasses.fields(lam)} - set(notes)
    if missing:
        raise SystemExit(
            f"Constraints gained {sorted(missing)}; add them to lambda_table() "
            "with a symbol and a note, or the documented lambda is incomplete"
        )
    rows = ["| | reference | note |", "|---|---|---|"]
    for sym, val, note in notes.values():
        # Thin the thousands separator in the VALUE only. Doing it to the whole
        # row once ate the comma out of "hard floor, independent of stall".
        rows.append(f"| {sym} | {val.replace(',', ' ')} |{' ' + note if note else ''} |")
    return "\n".join(rows)


def turn_performance_table() -> str:
    v = reference_fighter()
    rows = [
        "| $v$ [m/s] | $n_\\ell$ | $n_{\\mathrm{av}}$ | $\\omega_{\\mathrm{av}}$ "
        "[°/s] | $\\omega_{\\mathrm{sus}}$ [°/s] | $R_{\\min}$ [m] | "
        "$T_{\\mathrm{req}}$ [kN] |",
        "|---|---|---|---|---|---|---|",
    ]
    corner = v.v_corner_mps(MASS_KG)
    for spd in SPEEDS:
        c = v.capability(VehicleState(0.0, 0.0, 0.0, spd, MASS_KG))
        at_corner = abs(spd - corner) < 1.0
        b = (lambda x: f"**{x}**") if at_corner else (lambda x: x)
        rows.append(
            f"| {b(f'{spd:.0f}')} | {b(f'{v.lift_limited_load_factor(spd, MASS_KG):.2f}')} "
            f"| {b(f'{c.load_factor_available:.2f}')} "
            f"| {b(f'{math.degrees(c.omega_available_rad_s):.2f}')} "
            f"| {math.degrees(c.omega_sustained_rad_s):.2f} "
            f"| {b(f'{c.turn_radius_min_m:.0f}')} "
            f"| {c.thrust_required_N / 1e3:.1f} |"
        )
    return "\n".join(rows)


# --------------------------------------------------------------------------
# Booster
# --------------------------------------------------------------------------

def boost_parameter_table() -> str:
    b, lam = reference_boosted_fighter(), FIGHTER_BOOST_LIMITS
    ratio = b.theta.c_tsfc_boost / b.theta.nominal.c_tsfc
    return "\n".join([
        "| | reference | against baseline |",
        "|---|---|---|",
        f"| $c^{{\\mathrm{{boost}}}}_{{\\mathrm{{TSFC}}}}$ | "
        f"${b.theta.c_tsfc_boost:.1e}".replace("e-05", r"\times10^{-5}$")
        + f" kg/(N s) | **{ratio:.1f}×** the nominal rate |",
        f"| $T^{{\\mathrm{{boost}}}}_{{\\max}}$ | {lam.thrust_max_boost_N / 1e3:.0f} kN "
        f"| +{100 * (lam.thrust_max_boost_N / lam.nominal.thrust_max_N - 1):.0f}% |",
        f"| $v^{{\\mathrm{{boost}}}}_{{\\max}}$ | {lam.v_max_boost_mps:.0f} m/s "
        f"| +{lam.v_max_boost_mps - lam.nominal.v_max_mps:.0f} m/s |",
        f"| $\\tau_{{\\mathrm{{h}}}}$ | {b.theta.tau_h_s:.0f} s | cold to limit |",
        f"| $\\tau_{{\\mathrm{{c}}}}$ | {b.theta.tau_c_s:.0f} s "
        f"| ${b.theta.tau_c_s / b.theta.tau_h_s:.0f}\\tau_{{\\mathrm{{h}}}}$ |",
        f"| $m_{{\\mathrm{{res}}}}$ | {lam.mass_reserve_kg:.0f} kg "
        "| fuel reserve — **policy** |",
        f"| $s_{{\\max}}$ | {lam.thermal_max:.0f} | thermal limit — **physics** |",
        f"| $\\Delta t_{{\\mathrm{{dwell}}}}$ | {lam.dwell_s:.0f} s "
        "| anti-chatter — **policy** |",
    ])


def boost_turn_performance_table() -> str:
    b = reference_boosted_fighter()
    rows = [
        "| $v$ [m/s] | $\\omega_{\\mathrm{av}}$ [°/s] | $\\omega_{\\mathrm{sus}}$ "
        "nom | $\\omega_{\\mathrm{sus}}$ boost | gain | $t_{\\mathrm{end}}$ nom "
        "| $t_{\\mathrm{end}}$ boost |",
        "|---|---|---|---|---|---|---|",
    ]
    for spd in BOOST_SPEEDS:
        cn = b.capability(BoostState(0, 0, 0, spd, MASS_KG, 0.0, Mode.NOMINAL))
        cb = b.capability(BoostState(0, 0, 0, spd, MASS_KG, 0.0, Mode.BOOST))
        n = math.degrees(cn.omega_sustained_rad_s)
        bo = math.degrees(cb.omega_sustained_rad_s)
        rows.append(
            f"| {spd:.0f} | {math.degrees(b.omega_max_rad_s(spd, MASS_KG)):.2f} "
            f"| {n:.2f} | {bo:.2f} | {bo - n:+.2f} "
            f"| {cn.endurance_s / 60:.0f} min | {cb.endurance_s / 60:.0f} min |"
        )
    return "\n".join(rows)


# --------------------------------------------------------------------------
# Scalars quoted in prose. Not substituted -- checked.
# --------------------------------------------------------------------------

def prose_figures() -> dict[str, list[str]]:
    """Numbers that appear in hand-written sentences, and the file they appear
    in. Wrapping every sentence in markers would make the pages unreadable, so
    these are verified to still be present instead of being generated.
    """
    v, b = reference_fighter(), reference_boosted_fighter()
    base = _level_flight_ceiling(v, MASS_KG, FIGHTER_LIMITS.thrust_max_N)
    boost = _level_flight_ceiling(
        b, MASS_KG, FIGHTER_BOOST_LIMITS.thrust_max_boost_N
    )
    return {
        "planar_point_mass.md": [
            f"{v.v_stall_mps(MASS_KG):.1f}",              # stall at 16 t
            f"{v.v_corner_mps(MASS_KG):.1f}",             # corner speed
            f"{v.v_min_achievable_mps(MASS_KG):.0f}",     # usable floor
            f"{base:.0f}",                                # level-flight ceiling
            f"{v.thrust_required_N(600.0, MASS_KG, 0.0) / 1e3:.1f}",
        ],
        "planar_point_mass_with_booster.md": [
            f"{base:.0f}",
            f"{boost:.0f}",
            f"{b.theta.tau_h_s:.0f}",
            f"{b.theta.tau_c_s:.0f}",
            f"{b.theta.c_tsfc_boost / b.theta.nominal.c_tsfc:.1f}",
        ],
    }


BLOCKS = {
    "planar_point_mass.md": {
        "theta": theta_table,
        "lambda": lambda_table,
        "turn-performance": turn_performance_table,
    },
    "planar_point_mass_with_booster.md": {
        "boost-parameters": boost_parameter_table,
        "boost-turn-performance": boost_turn_performance_table,
    },
}


def render(path: Path, blocks) -> str:
    text = path.read_text()
    for name, build in blocks.items():
        pattern = re.compile(
            rf"(<!-- generated: {re.escape(name)} -->\n).*?(\n<!-- end generated: "
            rf"{re.escape(name)} -->)",
            re.S,
        )
        if not pattern.search(text):
            raise SystemExit(f"{path.name}: no marker pair for {name!r}")
        text = pattern.sub(lambda m: m.group(1) + build() + m.group(2), text)
    return text


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--check", action="store_true",
                    help="report staleness instead of rewriting")
    args = ap.parse_args()

    stale = []
    for name, blocks in BLOCKS.items():
        path = DOCS / name
        rendered = render(path, blocks)
        if rendered == path.read_text():
            continue
        if args.check:
            stale.append(name)
        else:
            path.write_text(rendered)
            print(f"  rewrote {name}")

    missing = []
    for name, figures in prose_figures().items():
        text = (DOCS / name).read_text()
        missing += [f"{name}: {f}" for f in figures if f not in text]

    if args.check and (stale or missing):
        if stale:
            print("Stale generated tables in: " + ", ".join(stale))
        for m in missing:
            print(f"Figure quoted in prose no longer matches the code -- {m}")
        print("\nRun: python tools/generate_model_docs.py")
        return 1
    if missing:
        for m in missing:
            print(f"  WARNING prose figure not found -- {m}")
    if not args.check and not stale:
        print("  tables up to date")
    return 0


if __name__ == "__main__":
    sys.exit(main())
