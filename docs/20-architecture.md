# Architecture

Status: draft

## Shape

```
              +------------------------------------+
              |         Multi-ship layer           |   cyber
              |  objectives, coordination, tasking |
              +------------------------------------+
                              |
              +------------------------------------+
              |        Single-ship layer           |   cyber
              |   tracker, SA, action planner      |
              +------------------------------------+
                              |
              +------------------------------------+
              |         Subsystem layer            |   cyber
              |  vehicle, sensor, effector systems |
              +------------------------------------+
- - - - - - - - - - - - - - - | - - - - - - - - - - - -  truth boundary
              +------------------------------------+
              |          Resource layer            |   physical
              | vehicle, sensors, comms, effectors |
              +------------------------------------+
                              |
     +--------------------------------------------------+
     |        Simulation core -- ground truth            |
     |  clock, kinematics, detection, engagement         |
     +--------------------------------------------------+
```

## Principles

**Composition is declarative.** A YAML specification is the single source of
truth. The GUI edits it, the scenario builder generates it, Monte Carlo
transforms it, a lab is one with stubs substituted. Nothing in the toolchain
holds state that is not expressible in it. See `40-composition-spec.md`.

**Service orientation as a pattern, not a deployment.** Components register
against a typed in-process registry and are resolved synchronously. Network
transports, brokers, and runtime discovery are excluded: they introduce
nondeterminism, which destroys reproducible Monte Carlo, debuggability, and a
sane fixed-step clock. The registry interface is designed so a remote transport
could be slotted behind it later, and no component may assume in-process
semantics -- no reaching into another component's internals, no shared mutable
state, everything through declared ports.

**Binding happens once, at composition time, then freezes.** Load, discover,
match capabilities against requirements, validate, bind, freeze, run. Continuous
runtime discovery would make a scenario's topology depend on execution order and
would surface mis-wiring mid-run rather than at load. Composition-time validation
is also what lets the GUI grey out pieces that do not fit.

**Interfaces are a package containing no implementations.** Components depend on
`ose.interfaces`; no component depends on another component.

**Only the resource layer reads truth.** Enforced by port type at import time.
See ADR 0008.

## Current state

Implemented: the baseline vehicle model and two navigation systems, with tests.

Not yet implemented: the registry, the binder, the descriptor validator, the
simulation core itself, and every component type other than vehicle and
navigation. The composition specification describes the intended format; nothing
consumes it yet.

## Repository layout

```
src/ose/interfaces.py        contracts only
src/ose/resource/            resource-layer components
docs/                        scope, concepts, architecture, interfaces
docs/adr/                    architecture decision records
docs/vehicle/                the vehicle model document, LaTeX and PDF
demos/                       runnable demonstrations
tests/                       pinning and consistency tests
```

Layer packages `subsystem/`, `single_ship/`, and `multi_ship/` are added as
siblings of `resource/` when they acquire their first component.
