# OSE — Open Simulation Environment

A simulation environment for research and teaching in autonomous combat aircraft
systems. It exists so that a student or researcher can test one component,
algorithm, or subsystem inside a complete integrated system of interest without
first implementing everything around it.

The emphasis is integration, not fidelity.

**Status: early.** The baseline vehicle model; three equipment-layer navigation
sensors (IMU, GNSS, air data), a equipment-layer black-box integrated nav unit,
a subsystem-layer INS/GNSS estimator, and a navigation manager that is the
platform's single publisher of own-state; a equipment-layer clock with a
dead-reckoning-only subsystem-layer time estimator (no correction source
exists yet); a equipment-layer fuel gauge feeding a subsystem-layer vehicle
manager that owns the platform's believed mass and is the only consumer of
the vehicle model; a subsystem-layer vehicle guidance component that
enforces the vehicle's admissible sets before publishing a command; and a
single-ship action planner that follows a route of waypoints, are
implemented and tested. Every equipment component also answers
`capability()`, a self-assessment checked against integrated dynamics. The
simulation core, the service registry, the composition binder, and every other
component type are described in `docs/` but not yet built.

All parameter values in this repository are fictional and plausible. They are not
claims about any real system.

## Quick start

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

pytest                                 # run the test suite
python demos/demo_vehicle.py           # turn envelope and an open-loop manoeuvre
python demos/demo_navigation.py        # INS/GNSS through a GNSS outage
python demos/demo_mass_estimation.py   # mass belief through a fuel-gauge outage
python demos/demo_vehicle_guidance.py  # closed-loop heading/speed-hold, enforced
python demos/demo_live_flight.py       # a setpoint mission, animated live
python demos/demo_live_route.py        # a route through the planner, animated live
python demos/demo_discretization.py    # one model, several discretisations
```

Full setup notes, including direnv and troubleshooting, are in
[SETUP.md](SETUP.md).

## Layout

```
src/ose/interfaces.py                             contracts only, no implementations
src/ose/frames.py                                 rotation utilities, no dependencies
src/ose/integration.py                            integrators, external to the components they step
src/ose/environment.py                            environmental parameters (g, rho), no dependencies
src/ose/reference_configs/                        reference configs for cross-layer shapes, e.g. environment
src/ose/equipment/vehicle.py                       baseline 2D vehicle model
src/ose/equipment/reference_configs/               reference configs for equipment components, e.g. reference_vehicle.py
src/ose/equipment/imu.py                           IMU sensor model
src/ose/equipment/gnss.py                          GNSS receiver model
src/ose/equipment/air_data.py                      air data sensor model
src/ose/equipment/integrated_navigation_unit.py    black-box integrated nav unit
src/ose/equipment/clock.py                         platform clock model
src/ose/equipment/fuel_gauge.py                    remaining-fuel sensor
src/ose/subsystem/navigation_manager.py           one own-state publisher per platform
src/ose/subsystem/navigation_state_estimator.py   INS/GNSS error-state Kalman filter
src/ose/subsystem/time_state_estimator.py         dead-reckoning platform clock estimator
src/ose/subsystem/vehicle_manager.py              believed mass; the only consumer of the vehicle model
src/ose/subsystem/vehicle_guidance.py             heading/speed-hold guidance, enforces admissibility
src/ose/subsystem/reference_configs/              reference configs for subsystem components
src/ose/single_ship/action_planner.py             waypoint-following action planner
src/ose/single_ship/reference_configs/            reference configs for single-ship components
docs/                                             scope, concepts, architecture, tooling
docs/adr/                                         architecture decision records
docs/interfaces/                                  interface catalogue
docs/vehicle/                                     vehicle model document, LaTeX and PDF
demos/                                            runnable demonstrations
tests/                                            pinning and consistency tests
```

## Where to start reading

| If you want to | Read |
|---|---|
| Know what this is and is not | `docs/00-scope.md` |
| Understand the vocabulary and conventions | `docs/10-concepts.md` |
| See the structure | `docs/20-architecture.md` |
| Know why something is the way it is | `docs/adr/` |
| Understand the vehicle model mathematically | `docs/vehicle/vehicle_model.pdf` |
| Add a component | `docs/interfaces/README.md`, then `docs/40-composition-spec.md` |

## The four things that constrain everything else

**Only the equipment layer reads ground truth.** The simulation core owns true
world state. Equipment-layer components read it, corrupt it according to a sensor
error model, and publish estimates. Everything above consumes only estimates.
This cannot be relaxed for convenience, including in debugging aids: one leak
invalidates every result produced afterwards and is nearly impossible to detect
later. See ADR 0008.

**Components publish continuous dynamics, never discrete updates.** A model
publishes `f(x, u, ...)`; the consumer chooses the discretisation. That function
must be pure — no hidden state, no internal RNG, no wall-clock dependence.
Purity is load-bearing: violating it breaks adaptive solvers, Jacobians,
parallel Monte Carlo and reproducibility at once. See ADR 0004.

**Components declare constraints; they do not enforce them.** The vehicle states
its admissible sets and integrates whatever command it is given. Enforcement
belongs to guidance or to a runtime assurance layer, so that a control law
commanding outside the envelope produces a visible finding rather than a silently
clipped command. See ADR 0006.

**Composition is declarative and the specification is the single source of
truth.** The GUI edits it, Monte Carlo transforms it, a lab is one with stubs
substituted. See `docs/40-composition-spec.md`.

## Contributing a component

1. Read `docs/10-concepts.md` for frames, units, and the truth boundary.
2. Define or reuse an interface in `docs/interfaces/`.
3. Implement against the protocol in `src/ose/interfaces.py`. Depend on that
   module, never on another component.
4. Publish a capability model: what your component can currently achieve,
   answerable without simulating it forward.
5. Write consistency tests, not just plots. If your component estimates
   something, test that its stated uncertainty is honest.

That last point is not boilerplate. The INS/GNSS filter here shipped with a
one-step misalignment between the mechanised state and the truth used to form
measurement residuals. Straight flight looked perfect; under turn the filter was
overconfident by a factor of thirty, which would have quietly corrupted every
tracker and planner downstream. A NEES test catches it in seconds. A plot of a
single seed does not.

## Licence

Apache License 2.0. Copyright 2026 Saab AB. Full text in [LICENSE](LICENSE),
attribution in [NOTICE](NOTICE).

Apache 2.0 rather than a permissive alternative because it grants patent rights
explicitly and requires contributors to do the same, which matters more than
usual for a defence-adjacent project. Contributions are accepted under the same
terms, per section 5 of the licence — there is no separate CLA.

All parameter values in this repository are fictional and plausible. They are
not claims about, and are not derived from, any real system.
