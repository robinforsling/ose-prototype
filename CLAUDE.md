# CLAUDE.md

Context for agentic work in this repository. Read this first; follow the
pointers rather than inferring structure from the file tree.

## What this is

An open simulation environment for research and teaching in autonomous combat
aircraft systems. A student or researcher should be able to test one component
inside a complete integrated system without implementing everything around it.

**The emphasis is integration, not fidelity.** Given a choice between a faithful
model and a simple one that preserves the integration problem, choose the simple
one.

Planar (2D). Python. Low fidelity by design. All parameter values in this
repository are fictional and plausible, never claims about any real system.

## Layer structure

Composition is bottom-up. A component may bind to the layer below it and to
peers in the same layer on the same platform. **Nothing binds upward.**

| Layer | Contains | Physical? | Package |
|---|---|---|---|
| Multi-ship | Mission objectives, coordination, tasking | No | `ose.multi_ship` |
| Single-ship | Tracker, situation awareness, action planner | No | `ose.single_ship` |
| Subsystem | Vehicle system, sensor system, navigation estimator | No | `ose.subsystem` |
| Resource | Vehicle, IMU, GNSS, sensors, communicators, effectors | Yes | `ose.resource` |

Below the resource layer sits the simulation core, which owns ground truth: the
clock, kinematics, detection, and engagement resolution.

Layer packages are created when they acquire their first component, not before.

**Deciding where a new component goes.** Does it have a physical part, or does
it read ground truth? Resource. Does it integrate resources on one platform?
Subsystem. Does it decide for one ship? Single-ship. Does it coordinate across
ships? Multi-ship. If a component seems to span two layers, it is two
components.

Authoritative detail: `docs/10-concepts.md`. Diagram and principles:
`docs/20-architecture.md`.

## Invariants — do not break these

**1. Only resource-layer components may read ground truth.** Everything above
consumes published estimates. This holds for debugging aids and visualisation
too: one leak invalidates every result produced afterwards and is nearly
impossible to detect later. A cyber-layer component whose signature contains a
truth-carrying type is wrong regardless of what it does with it. See ADR 0008.

**2. Components publish continuous dynamics, never discrete updates.** A model
publishes `f(x, u, ...)`; the consumer chooses the discretisation. That function
must be pure: no hidden state, no internal RNG, no wall-clock dependence, no
logging. Purity is load-bearing — violating it breaks adaptive solvers,
Jacobians, parallel Monte Carlo and reproducibility simultaneously. See ADR 0004.

**3. Components declare constraints; they do not enforce them.** The vehicle
states its admissible sets and integrates whatever command it is given.
Enforcement belongs to guidance or a runtime assurance layer, so that a control
law commanding outside the envelope produces a visible finding rather than a
silently clipped command. Two tests assert this; if they fail, the separation
has been broken. See ADR 0006.

**4. Every measurement record carries its own valid-time and its own declared
uncertainty.** Valid-time is the time the measurement refers to, not the time it
was delivered. The consumer uses the sigma travelling with the measurement,
never a separately configured value.

**5. Each component owns its own RNG stream.** Never share a generator across
components to keep results bit-identical. Adding a component must not perturb
any other component's stream. See ADR 0005.

## Conventions

- **Frame**: planar NED. `p_x` north, `p_y` east, metres.
- **Heading**: `psi` clockwise from north, radians. `omega` positive right.
- **Airspeed vs ground speed**: vehicle state carries airspeed; wind enters
  position kinematics only. Heading is air-relative and ground track differs
  whenever wind is non-zero. Conflating them is a recurring source of error.
- **Units**: SI, angles in radians internally, degrees in authored files with the
  unit in the field name (`fov_deg`).
- **Interfaces** live in `src/ose/interfaces.py` and contain no implementations.
  Components depend on that module, never on each other.

## Working in this repository

Run the tests after every change:

```bash
pytest
```

**Never weaken a test threshold to make a test pass.** The NEES bound, the
three-sigma containment fractions, and the observability ratios are calibrated.
If one starts failing, the change is wrong.

**Record architectural decisions as ADRs** in `docs/adr/`, numbered, immutable
once accepted, stating consequences you dislike as well as benefits. Superseding
or extending means writing a new record that references the old one. The only
edit an accepted record takes is a forward pointer in its status line, so a
reader cannot mistake it for current state; its reasoning stays as written. An
ADR is a dated record of why a choice was made — for how things are now, read
`docs/10-concepts.md`, `docs/20-architecture.md` and `docs/interfaces/`.

**Prefer adding fields to published records over changing them.** Adding is
backward compatible; removing or renaming requires a version increment on the
interface.

## Testing philosophy

Test properties, not appearances. Any component that publishes an uncertainty
must have a test that its stated uncertainty is honest.

This is not boilerplate. The INS/GNSS filter shipped with a one-step
misalignment between the mechanised state and the truth used to form measurement
residuals. In straight flight the velocity vector is not rotating, so the
residual vanished and every plot looked correct. Under turn it injected a
systematic 3 m/s residual against 0.15 m/s of measurement noise, and the filter
absorbed it into heading and bias, finishing thirty times overconfident. Four
wrong hypotheses were investigated before the cause was found. A NEES test would
have pointed at it in seconds.

Related: prefer testing a property directly over testing a downstream
consequence. A test that an inadmissible command was not clipped should check
that heading advanced by exactly `omega * dt`, not that speed increased — speed
depends on thrust and turn rate together and cannot isolate either.

## Current state

Implemented: the baseline vehicle model, navigation sensors and estimator, with
tests.

Not implemented: the simulation core, the service registry, the composition
binder, the descriptor validator, and every component type other than vehicle
and navigation. `docs/40-composition-spec.md` describes the intended
specification format; nothing consumes it yet.

Do not describe unimplemented parts as working, in code comments or in
documentation.

## Where to look

| For | See |
|---|---|
| Scope, and what is deliberately excluded | `docs/00-scope.md` |
| Vocabulary, frames, conventions | `docs/10-concepts.md` |
| Structure and principles | `docs/20-architecture.md` |
| Why something is the way it is | `docs/adr/` |
| Interface catalogue | `docs/interfaces/README.md` |
| Composition specification format | `docs/40-composition-spec.md` |
| Vehicle model mathematics | `docs/vehicle/vehicle_model.pdf` |
| Planned tooling | `docs/50-tooling.md` |
