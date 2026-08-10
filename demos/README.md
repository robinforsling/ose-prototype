# Demonstrations

Run from the repository root with the package installed (`pip install -e .`).

| Demo | Produces | Runtime |
|---|---|---|
| `demo_vehicle.py` | `plots/vehicle_envelope.png`, `plots/vehicle_trajectory.png` | ~5 s |
| `demo_navigation.py` | `plots/navigation_errors.png` | ~30–60 s |
| `demo_vehicle_guidance.py` | `plots/vehicle_guidance.png` | ~5 s |
| `demo_live_flight.py` | live window, or `plots/live_flight.mp4` with `--video` | instant live, ~5 min to render video |
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

**`demo_vehicle_guidance.py`** — closed-loop heading/speed-hold guidance
against a perfect state estimate (isolating guidance's own behaviour from
navigation error). A moderate setpoint change converges cleanly; a much
larger heading change saturates the turn rate at first -- visibly, both in
the shaded region and in requested-vs-delivered traces -- rather than being
silently clipped, then recovers as the heading error shrinks back within
capability.

**`demo_live_flight.py`** — a sequence of guidance setpoints flown and
animated: ground track with heading, commanded against true heading and
airspeed, and requested against delivered turn rate and thrust with the
vehicle's own moving limits. Opens a window if there is a display, writes mp4
otherwise or with `--video`. Doubles as a throwaway stand-in for the
simulation core; `docs/50-tooling.md` records what building it taught.

The window has transport controls -- play/pause, step, scrub, speed -- because
watching a mission go past once tells you much less than stopping on the
moment a limit binds and stepping through it. Space plays and pauses, the
arrow keys step, and dragging the bar seeks.

The mission presses on different limits in turn: a 360 at the structural
load-factor limit that cannot be held and spirals in as speed bleeds; a planar
lazy eight, trading airspeed rather than height between opposing loops, whose
ground track crosses itself; and a corner-speed sweep, decelerating from
380 m/s pinned against the turn-rate limit so the delivered rate traces the
corner curve and peaks at `v_corner` -- within 0.1 m/s of the closed form.

Saturation shows in both directions: the turn rate against the 9 g structural
limit, and thrust the other way during deceleration, where guidance asks for
negative thrust and gets idle because this airframe has no speedbrake. Both
appear as requested and delivered separating.

**`demo_discretization.py`** — one untouched `derivative()` consumed by
fixed-step RK4, an adaptive solver, and a numerically-linearised zero-order-hold
discrete model. Demonstrates ADR 0004: the component publishes continuous
dynamics and the consumer chooses the discretisation.

## Changing the seed

`demo_navigation.py` is seeded for reproducibility. Change it and re-run several
times before believing any consistency claim — a single trajectory can flatter a
filter that is quietly wrong.
