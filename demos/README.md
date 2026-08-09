# Demonstrations

Run from the repository root with the package installed (`pip install -e .`).

| Demo | Produces | Runtime |
|---|---|---|
| `demo_vehicle.py` | `plots/vehicle_envelope.png`, `plots/vehicle_trajectory.png` | ~5 s |
| `demo_navigation.py` | `plots/navigation_errors.png` | ~30–60 s |
| `demo_discretization.py` | printed tables only | ~2 s |

Figures are written to `demos/plots/`, alongside the scripts, regardless of
which directory you run them from. `demos/plots/` is gitignored.

## What each one shows

**`demo_vehicle.py`** — the turn performance envelope, a doghouse plot with the
lift limit binding below corner speed and the structural limit above, and the gap
between instantaneous and sustained turn rate. Then an open-loop manoeuvre in
which a sustained-rate turn holds airspeed to a tenth of a m/s over 70 seconds
while a maximum-rate turn bleeds 100 m/s.

**`demo_navigation.py`** — INS/GNSS estimation error against its own 3-sigma
bounds through a 150 s GNSS outage. Two properties visible in the output are
physics rather than implementation: heading uncertainty stays flat until the
first turn, because heading is unobservable from GNSS position without lateral
specific force; and the wind estimate does the same, because separating its two
components requires heading diversity.

**`demo_discretization.py`** — one untouched `derivative()` consumed by
fixed-step RK4, an adaptive solver, and a numerically-linearised zero-order-hold
discrete model. Demonstrates ADR 0004: the component publishes continuous
dynamics and the consumer chooses the discretisation.

## Changing the seed

`demo_navigation.py` is seeded for reproducibility. Change it and re-run several
times before believing any consistency claim — a single trajectory can flatter a
filter that is quietly wrong.
