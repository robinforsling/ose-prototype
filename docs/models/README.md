# Model reference

One page per implemented model: state, parameters, constraints, and — the part
worth writing down — what the thing actually *does*.

These are not the mathematics. Derivations, sign conventions and the reasoning
behind each formulation live in the model documents under `docs/vehicle/`, and
these pages use the same notation so the two read together. What they add is
the behaviour: which limit binds where, what is counter-intuitive, what the
numbers come out as for the bundled reference configuration, and which tests
pin it.

Every number on these pages is computed from the reference configuration
rather than remembered. The tables are generated between
`<!-- generated: NAME -->` markers by

```bash
python tools/generate_model_docs.py
```

and `pytest` fails while they are stale, so a model change cannot quietly
leave its page behind. The prose between the markers is written by hand and
stays that way — see the tool's docstring for why. All values are fictional
and plausible, never claims about a real system.

## Vehicle

| Model | Module |
|---|---|
| [Planar point mass](vehicle/planar_point_mass.md) | `ose.equipment.vehicle.planar_point_mass` |
| [Planar point mass with booster](vehicle/planar_point_mass_with_booster.md) | `ose.equipment.vehicle.planar_point_mass_with_booster` |

## Everything else

Navigation sensors, the INS/GNSS estimator, the clock, the fuel gauge, the
vehicle manager and vehicle guidance have no page here yet. Their behaviour is
currently described in their module docstrings, which is where a reader should
look until this directory catches up. A page here is worth writing when a
model's behaviour is surprising enough that a reader would get it wrong from
the code alone — which is the test both vehicle pages meet.
