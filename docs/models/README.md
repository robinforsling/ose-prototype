# Model reference

One page per implemented model: state, parameters, constraints, and — the part
worth writing down — what the thing actually *does*.

**Derived from the code, not from the mathematics.** Every table and every
figure here is computed by importing `ose.equipment.*` and asking the model;
nothing in that path reads a `.tex` file. Writing a page from the preliminary
modelling instead would describe what was intended, which is not the same
thing — it would agree with the theory while quietly disagreeing with the
software.

Derivations, sign conventions and the reasoning behind each formulation live
in [`docs/preliminary_models/`](../preliminary_models/), and these pages use
its notation so the two read together. What they add is the behaviour: which
limit binds where, what is counter-intuitive, what the numbers come out as for
the bundled reference configuration, and which tests pin it.

The tables sit between `<!-- generated: NAME -->` markers and are written by

```bash
python tools/generate_model_docs.py
```

with `pytest` failing while they are stale, so a model change cannot quietly
leave its page behind. The prose between the markers is not generated and no
version of that tool should try — see its docstring. It is still derived from
the code, by running it.

All values are fictional and plausible, never claims about a real system.

## Vehicle

| Model | Module |
|---|---|
| [Planar point mass](vehicle/planar_point_mass.md) | `ose.equipment.vehicle.planar_point_mass` |
| [Planar point mass with booster](vehicle/planar_point_mass_with_booster.md) | `ose.equipment.vehicle.planar_point_mass_with_booster` |

## Guidance

| Model | Module |
|---|---|
| [Vehicle guidance](guidance/vehicle_guidance.md) | `ose.subsystem.vehicle_guidance` |

The first page here for something that is not a physical model, and the first
with no preliminary modelling behind it — a proportional law needs no
derivation. What it needs is the three things it does that reading it does not
reveal: which turn rate the thrust feedforward is evaluated at, what a moving
setpoint costs without its rate declared, and why the same believed mass is
asked for in two different forms.

## Where this sits

```
docs/preliminary_models/   notation and the basic formulation
        ↓                  the modelling the code is written FROM
src/ose/                   THE MODEL. This is what runs
        ↓
docs/models/               these pages, derived from the code
```

The developed model is the code. [`docs/preliminary_models/`](../preliminary_models/)
is an input to it and stays authoritative on notation and on why a formulation
is what it is; see its README for what happens when the three disagree.

## Everything else

Navigation sensors, the INS/GNSS estimator, the clock, the fuel gauge and the
vehicle manager have no page here yet. Their behaviour is currently described
in their module docstrings, which is where a reader should look until this
directory catches up. A page here is worth writing when a model's behaviour is
surprising enough that a reader would get it wrong from the code alone — which
is the test the vehicle and guidance pages meet.

The vehicle manager is the strongest remaining candidate: a two-state filter
whose burn-coefficient state is weakly observable by design, and which
reconciles a gauge reading that is not what it is named after (ADR 0026).
