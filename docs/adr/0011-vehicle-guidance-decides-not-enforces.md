# 0011. Vehicle guidance decides what to command, not what is admissible

Status: accepted
Date: 2026-08-10

## Context

ADR 0006 declared that the vehicle states its admissible sets but does not
enforce them, and that enforcement belongs to guidance or a runtime
assurance layer. Until now nothing occupied that role: both demos that fly
the vehicle model played it inline, with a comment saying so
(`demo_vehicle.py`'s `simulate()`: "The guidance layer -- here, this
open-loop script -- is responsible for keeping the command admissible"), and
`Vehicle2D.project_command()` / `Saturation` existed but had no real caller.

Guidance also needs a state to steer from. It sits in the subsystem layer,
so per ADR 0008 it may not read `VehicleState` or `Disturbance` directly --
but `OwnStateEstimate.as_vehicle_state(mass_kg)` already existed precisely
to let a cyber-layer component construct a believed state for calling
vehicle-shaped APIs. Nothing had used it yet either.

No single-ship layer exists yet to supply a real desired trajectory
(`planning.action.v1` is still `planned` in the interface catalogue), and no
component estimates the platform's own mass -- no vehicle-system or
fuel-accounting component exists.

## Decision

`VehicleGuidance` (`subsystem/vehicle_guidance.py`) takes a
`HeadingSpeedSetpoint` -- a stand-in for `planning.action.v1` -- and the
platform's own `OwnStateEstimate`, converts the estimate to a believed
`VehicleState` via `as_vehicle_state()`, computes a raw command by
proportional control on heading and speed error (drag feedforward on the
thrust channel), and hands that raw command to
`Vehicle2D.project_command()`. The `Saturation` it returns is passed back to
the caller rather than inspected and discarded -- the whole point of ADR
0006 was that clipping should be a visible finding, and this is the first
component to make that finding visible to anyone.

`command()` dispatches on setpoint type, the same reasoning
`NavigationEstimator.ingest()` and `TimeEstimator.ingest()` already use: a
later waypoint-pursuit mode is a new type and a new branch, not a change to
the protocol or a redesign of existing callers.

`mass_kg` is a plain parameter to `command()`, supplied by the caller, not
derived from `own_state`. This is stated as a simplification, not hidden:
nothing in this repository estimates mass yet, and inventing an estimate
would be worse than being honest that today's callers must supply the true
value directly.

## Consequences

The truth boundary is exercised by something other than an estimator for
the first time: guidance never imports `VehicleState` or `Disturbance`,
checked by `test_guidance_cannot_see_truth` the same way as the two
estimators. `Vehicle2D.project_command()` and `Saturation` have their first
real caller, and `test_reports_saturation_when_setpoint_exceeds_envelope` is
the first test in the repository that exercises a command actually being
clipped and that clipping being reported rather than silently applied.

That first caller also showed what `Saturation` was missing. It reported
*that* a command had been clipped but not *what had been asked for*, except
inside its note strings, so the pre-enforcement command was unobservable from
outside. The first consumer that wanted to show the difference duplicated the
control law to recover it, and that duplicate went stale as soon as the law
changed. `Saturation` now carries the requested command as numbers.

Guidance's quality is bounded by what it is given. A mass supplied directly
by the caller is exactly the kind of implicit truth-reading ADR 0008 exists
to prevent, once a real fuel-accounting component exists to supply a
believed mass instead. Until then, any demo or test using `VehicleGuidance`
is implicitly assuming perfect mass knowledge and should not be read as
evidence that guidance works under mass uncertainty.

The control law has no integral term, so a persistent disturbance would
leave a steady-state error uncorrected. Accepted for now: the setpoint
stand-in and the missing mass estimate are both bigger gaps than controller
structure, and revisiting the law is cheap once those exist.
