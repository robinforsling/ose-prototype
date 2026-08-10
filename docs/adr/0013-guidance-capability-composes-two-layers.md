# 0013. Guidance capability composes the vehicle's and navigation's

Status: accepted, amended by ADR 0015
Date: 2026-08-10

Extends ADR 0012, which established the capability contract for resource-layer
components. This record covers the first subsystem-layer capability model and
the composition rule it required. ADR 0012's other decisions -- the unfixed
return type, per-channel accuracy, capability as a tested claim -- stand
unchanged and apply here too.

## Context

Capability began as a resource-layer idea, on the reasoning that the categories
`docs/40-composition-spec.md` defines envelopes for are all resource-layer, and
that nothing had yet needed to ask a filter or a guidance law what it could
achieve.

That reasoning does not survive contact with the first real consumer.
`VehicleGuidance` is asked to hold a heading and a speed, and whether it can
depends on two components, neither of which can answer alone. The vehicle
decides which setpoints are reachable: a speed below stall cannot be held by
any control law, however good. Navigation decides how tightly a reachable one
can be held: guidance steers on an estimate, so its own accuracy is bounded by
that estimate's.

The second half is not a heuristic. Guidance drives the *believed* state onto
the setpoint, so at steady state the true error is the navigation error, one
for one. Flown closed loop against an estimate carrying a five-degree heading
error, the vehicle settles five degrees off the commanded heading -- exactly,
not approximately. A guidance component that published only the vehicle's
envelope would be claiming a precision the installed navigation cannot support.

## Decision

`VehicleGuidance.capability(own_state)` returns a `GuidanceCapability`
composed from both layers: `max_turn_rate_rad_s`, `min_speed_mps` and
`max_speed_mps` from the vehicle's capability model, and
`heading_hold_sigma_rad` and `speed_hold_sigma_mps` from the navigation
uncertainty. (ADR 0015 removed the `mass_kg` argument: the vehicle half now
reaches guidance through the vehicle manager, already evaluated at the
platform's believed mass. The composition reasoning below is unchanged.) It carries an `admits(setpoint)` predicate answering the "can I
do this?" question `docs/40-composition-spec.md` section 4.1 requires an
envelope to answer.

The navigation half is read from the covariance travelling with
`OwnStateEstimate`, not by querying a navigation component. This follows the
rule ADR 0009 set for measurements -- the consumer uses the uncertainty that
arrives with the data -- and avoids coupling guidance to whichever estimator
is installed. It is also the more useful number: the covariance is what
navigation's uncertainty *is right now*, widened by a GNSS outage or not,
where a static claim from the estimator would not be.

Subsystem-layer components therefore may publish capability models, and should
when a consumer needs one. This is not a blanket requirement: the two
estimators still publish none, because nothing asks them.

## Consequences

Capability composes across the layer boundary, which is the property that
makes the modularity claim real rather than decorative. Swapping a
tactical-grade IMU for a worse one widens the navigation covariance, which
widens the guidance hold sigma, without a line changing in guidance. A planner
reading `GuidanceCapability` sees the consequence of a resource-layer
substitution three layers down.

The hold sigmas are floors, not guarantees, and this is the most likely way
for the record to be misread. They say the loop cannot do better once settled;
during a transient, or while the command is saturated, the true error is
larger. A consumer wanting "is it within X right now" must look at the
tracking error, not at this claim. The docstring says so; nothing enforces it.

`admits()` only tests speed, because any heading is reachable given time --
`max_turn_rate_rad_s` says how long, not whether. That asymmetry is honest for
a heading-and-speed setpoint but will not survive a waypoint mode, where
reachability becomes a question about time and distance and the vehicle's
existing `can_reach()` becomes the right tool.

Guidance initially read `self.vehicle.lam` directly for the hard speed limits,
because `Capability` reported `v_stall_mps` but not the airframe's `v_min_mps`
and `v_max_mps`. That gap has since been closed: `Capability` carries
`v_min_achievable_mps` and `v_max_achievable_mps`, and the floor is composed by
the vehicle rather than by each consumer. No subsystem component reaches into
`Constraints` now.
