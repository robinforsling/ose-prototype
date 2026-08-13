# 0025 — Descriptors describe components that exist

**Status:** Accepted.

## Context

There were two models of "a component" in this repository and they had never
met.

`ose/composition/descriptor.py` defined `ComponentDescriptor(type, layer,
category, consumes, supplies)`, and `load_check.py` ran station, mass and power
checks over it. **No descriptor instance existed anywhere in `src/`.** The only
ones were five fixtures in `tests/test_load_check.py` — `FIGHTER`, `RADAR`,
`MISSILE`, `INS`, `GENERATOR` — and none of them named a class that exists.
`sensor.radar.pulse_doppler` has no implementation. `Imu` had no descriptor.

So the composition-time checks were exercised entirely against fiction, and
nothing connected them to the thirteen components that do exist. Whatever
built the binder would have had to bridge the two worlds, and neither had been
built with the other in mind.

`docs/40-composition-spec.md` §4 already specified the bridge:
`implementation: importable Python path to the factory`, plus `provides` and
`requires` port lists. None of it was modelled, because `load_check.py` needed
a port graph it did not have.

The port graph now exists. `tools/generate_architecture_diagram.py` derives,
per component class, what it publishes, what it consumes, and which layer it
lives in.

## Decision

**A descriptor is authored data, cross-checked against the code.** It has to be
readable without constructing anything — that is what lets a binder validate a
platform before it is built, and what lets this be YAML later. Hand-written
data drifts, so `tests/test_descriptor_catalogue.py` resolves every
`implementation` and asserts the declared layer and ports match what the
generator derives. Authored, and checked: the same split as the generated
tables in the model pages.

**A `type` is a class paired with a configuration**, not a class.
`nav_sensor.imu.tactical` is `Imu` with `TACTICAL_GRADE`; the same class with a
navigation-grade configuration would be a different type weighing a different
amount. This is what the specification's own names imply, what `implementation`
is for, and the only reading under which `consumes.mass_kg` — the number the
mass budget check is built on — is honest.

**`ose/composition/catalogue.py`** holds one descriptor per implemented type.
It imports nothing from the layers: `implementation` is a string, because a
descriptor is data.

**`PlatformSpec` grows `subsystems` and `single_ship`**, as type names. Cyber
components have no physical part, so they need no station and no attachment
record. The specification's worked example has had those sections all along;
only `equipment` was modelled, because only the load checks existed.

**Port satisfaction — check 4 — is implemented.** A required port with no
provider is a finding. A required port with *several* providers is also a
finding: not obviously an error, but nothing says which one a consumer binds,
and a binder that picked either would be choosing an architecture by accident.
That second case is the shape of the defect that had `InsGnssEstimator` and
`NavigationManager` both publishing `vehicle.state.v1` (ADR 0021) — visible in
a rendered diagram, and now visible at composition time.

## Consequences

A real platform can be validated before anything is constructed. The aircraft
`demo_live_route.py` and `demo_navigation.py` fly between them — fighter, five
navigation sensors, five subsystems, a planner — passes `check_load` with no
findings, which is the first time any of these checks has run against
components that exist.

The specification goes from three checks implemented to four. Checks 5 and 6,
the truth boundary and layer discipline, are deliberately **not** restated
here: they are enforced at import time and by the architecture generator (ADR
0008, ADR 0024), which is earlier and stricter, because they are properties of
the code rather than of a platform.

**Two dependencies cannot be expressed, and are recorded rather than worked
around.** `VehicleGuidance` binds `VehicleManager` by concrete class, so it is
not a port, cannot appear in `requires`, and port satisfaction cannot check it.
`FuelGauge` takes `mass_dry_kg: float` from the vehicle, invisible here exactly
as it is to the diagram. Both are now visible from two directions, which is the
argument for turning the first into a port.

**The catalogue is a second place to update when a component changes.** Adding
a port means adding it in the code and in the descriptor; forgetting fails the
suite, which is the intended cost rather than an unintended one. The
alternative — deriving descriptors — would have made them unreadable without
running Python, defeating the reason a binder has them.

**Masses and power draws are invented.** They are fictional and plausible per
CLAUDE.md, and nothing validates them against anything, because there is
nothing to validate them against. The one exception is the vehicle's
`max_mass_kg`, which is its own `mass_max_kg` and is cross-checked: a ceiling
disagreeing with the model it describes would be worse than no ceiling, since
the mass budget check is built on it.

**Two catalogues of fixtures now exist in the tests.** The five fictional ones
stay, because they are the specification's worked example and exercise a
platform richer than anything implemented — a radar, a missile, a generator.
A reader has to know which is which, and the module docstrings say so.

## References

- ADR 0020 — the derivation the cross-check compares against
- ADR 0021 — the two-publishers defect this check would have caught
- ADR 0024 — layer discipline, enforced earlier and not restated here
- `docs/40-composition-spec.md` §4, §5, §6.1
