# Concepts and shared vocabulary

Status: draft

Terms are defined once here and used consistently everywhere. Where a term is
ambiguous in the wider literature, the meaning adopted here is stated explicitly.

## Frames and units

**Frame.** Planar projection of North-East-Down. `p_x` towards north, `p_y`
towards east, metres. Altitude suppressed.

**Heading.** `psi` measured clockwise from north, that is from the positive
`p_x` axis towards the positive `p_y` axis. Aviation convention, deliberately
not the mathematical one.

**Turn rate.** `omega` positive for a right turn.

**Units.** SI throughout. Angles in radians internally, degrees in
human-authored files with the unit in the field name (`fov_deg`).

**Airspeed versus ground speed.** Vehicle state carries airspeed. Wind enters
position kinematics only. Heading is therefore air-relative and ground track
differs from heading whenever wind is non-zero. Any external observer of a
platform measures ground-referenced motion; the platform's own aerodynamics
depend on airspeed. Conflating the two is a recurring source of error.

## Vehicle geometry

**Point-mass dynamics.** Position, heading and speed are propagated as a
point: the equations of motion carry no physical extent. The vehicle still
has an orientation, `psi` -- point-mass is not the same claim as
orientation-less. Airspeed direction relative to `psi` drives drag and lift,
and `psi` is what a mounted sensor's or communicator's own pointing is
expressed relative to.

**Circular extent, for anything that needs one.** Collision avoidance and
similar proximity checks need a footprint the point-mass dynamics do not
provide. For that, and only for that, the vehicle is represented as a circle
of a fixed radius centred on the point-mass position. A circle needs no
orientation of its own -- unlike a real airframe outline, it looks the same
from every angle, so no assumption beyond `psi` is required to place it.
This is a placeholder for eventual proximity and collision-avoidance work,
not a claim about actual airframe shape; see `docs/00-scope.md` for the
surrounding fidelity exclusions (high-fidelity aerodynamics, 6-DoF dynamics).

**Mounting orientation.** Sensors, communicators, and effectors attach to
the vehicle at a station (`docs/40-composition-spec.md`) with their own
fixed orientation relative to the vehicle body frame. A component's absolute
pointing is therefore `psi` plus its mounting offset; nothing in the
resource layer computes pointing independent of the vehicle's own
orientation. The composition-spec station schema does not yet carry a
mounting-orientation field -- this states the assumption ahead of that
implementation, not a description of it.

## Layers

Composition proceeds bottom-up. A layer may bind to the layer below it and to
peers within the same layer on the same platform. Nothing binds upward.

| Layer | Contains | Physical? |
|---|---|---|
| Resource | Vehicle, navigation sensors, sensors, communicators, effectors | Yes |
| Subsystem | Vehicle system, sensor system, effector system, comms system | No |
| Single-ship | Tracker, situation awareness, action planner | No |
| Multi-ship | Mission objective management, coordination, tasking | No |

Only the resource layer has a physical part. Everything above is purely cyber.

## Truth and perception

The simulation core owns true world state and the true forces acting on every
platform. Only resource-layer components may read it. Everything above consumes
published estimates.

This applies to capability as well as to state. The capability model is a
function of the disturbance, which the platform cannot observe. Evaluated with
the true disturbance it gives true capability, available only to the simulation;
evaluated with an estimated disturbance it gives believed capability, which is
what guidance and planning must use. The discrepancy is preserved deliberately.

See ADR 0008.

## Measurement records

Every record a sensor publishes -- `ImuMeasurement`, `GnssFix`,
`AirDataMeasurement`, `ClockMeasurement`, and any aiding source added later --
carries two things non-negotiably:

**Its own `valid_time_s`.** The time the measurement refers to, not the time
it was delivered. A consumer that mechanises or forms a residual against the
wrong timestamp gets a systematic, correlated error that is invisible in the
common case and only appears under manoeuvre. This is not hypothetical: a
one-step misalignment of exactly this kind produced a filter thirty times
overconfident while looking perfect in straight flight.

**Its own declared uncertainty.** A consumer uses the sigma travelling with
the measurement, never a separately configured value describing the same
sensor. This is what lets a sensor's declared accuracy and its true error
statistics diverge -- realistic when a sensor is miscalibrated or degraded,
and impossible to express if the same number configures both the corruption
and the correction.

See ADR 0009 for the navigation split that established this pattern and ADR
0010 for its second application to the platform clock, and
`docs/interfaces/README.md` for the current catalogue of measurement records.

## Capability model

Every component publishes a machine-readable statement of what it can currently
achieve, answerable without simulating it forward. For a vehicle this includes
available thrust, required thrust, acceleration bounds, instantaneous and
sustained turn rates, minimum turn radius, characteristic speeds, fuel and
endurance.

The capability model is the component's external interface. It is what makes
composition-time validation possible, what a planner queries instead of
reimplementing the dynamics, and what the composition GUI reads to know the
shape of each puzzle piece.

## Time

Fixed-step core with named rate groups: fast (typically 50 Hz), medium (10 Hz),
slow (1 Hz). Components declare which group they belong to. Seeds are derived
hierarchically so that any single run is reproducible in isolation.

See ADR 0005.

## Component contract

A component declares its ports, the resources it consumes, the resources it
supplies, its capability model, and its parameters. Components with continuous
dynamics publish a derivative function, never a discrete update. That function
must be pure.

See ADR 0004.
