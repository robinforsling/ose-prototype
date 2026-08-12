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
              |          Equipment layer            |   physical
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

**Code declares shape; data supplies values.** Component logic is Python (ADR
0002); parameter values are data, authored in YAML -- the capability
descriptor for defaults and bounds shipped with the component, the
composition spec for what a specific scenario overrides them to. See
`40-composition-spec.md` §4. The registry, the binder, and the descriptor
validator do not exist yet, so today every component's Python dataclass
defaults (`VehicleParameters`, `ImuParameters`, ...) stand in as that data;
`reference_fighter()`-style helpers are test and demo fixtures, not evidence
that a real configuration belongs in source. The moment a value needs to
differ per scenario rather than per component type, it belongs in a
descriptor or a spec, not in a Python default.

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

**Only the equipment layer reads truth.** Enforced by port type at import time.
See ADR 0008.

## Current state

Implemented: the baseline vehicle model; three equipment-layer navigation
sensors (`Imu`, `GnssReceiver`, `AirDataSensor`) and the equipment-layer
black-box `IntegratedNavUnit`; the subsystem-layer `InsGnssEstimator`, an
error-state Kalman filter fed by the sensors' published measurements (ADR
0009), published to the rest of the platform through a subsystem-layer
`NavigationManager` -- one own-state publisher per platform, which selects a
source but deliberately does not fuse alternatives (ADR 0014); a equipment-layer
`Clock`; the subsystem-layer `TimeEstimator`, a
dead-reckoning-only estimator of the platform clock's offset and drift, with
no correction source yet (ADR 0010); and the subsystem-layer
`VehicleGuidance`, a heading/speed-hold controller that enforces the
vehicle's admissible sets before publishing a command, the first real
consumer of `PlanarPointMass.project_command()` and the first real producer of
`vehicle.command.v1` (ADR 0011).

Every equipment-layer component answers `capability()`, a self-assessment of
what it can currently achieve, checked by tests that integrate the dynamics
forward and confirm it delivers what it claimed (ADR 0012). `VehicleGuidance`
both consumes capability -- feedforwarding thrust for the turn the vehicle can
actually fly rather than the one its error term asked for -- and publishes one
of its own, composed from the vehicle's envelope and the navigation covariance
it steers on (ADR 0013). The two estimators publish none; nothing asks them.

The first single-ship component exists: `WaypointPlanner`, publishing
`planning.action.v1` as an `ActionSet` bundle with one field per subsystem,
so a planner that later decides motion and sensing together can publish
through the same record. It steers at waypoints and converts position to a
heading command, which is why guidance never needed a waypoint setpoint --
the planner decides where, guidance decides how.

Not yet implemented: the registry, the binder, the descriptor validator, the
simulation core itself, and every component type other than vehicle,
navigation, the platform clock, the fuel gauge, the vehicle manager, vehicle
guidance, and the action planner. The composition specification describes
the intended format; nothing consumes it yet.

## Repository layout

```
src/ose/interfaces.py        contracts only
src/ose/frames.py            rotation utilities, no dependencies
src/ose/integration.py       integrators, external to components (ADR 0004)
src/ose/environment.py       environmental parameters (g, rho), no dependencies
src/ose/equipment/            equipment-layer components
src/ose/subsystem/           subsystem-layer components
docs/                        scope, concepts, architecture, interfaces
docs/adr/                    architecture decision records
docs/models/                 per-model reference, and the modelling documents
                             the code was derived from
demos/                       runnable demonstrations
tests/                       pinning and consistency tests
```

Layer packages `single_ship/` and `multi_ship/` are added as siblings of
`equipment/` when they acquire their first component.
