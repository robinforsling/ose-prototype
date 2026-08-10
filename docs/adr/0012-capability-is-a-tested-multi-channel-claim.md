# 0012. Capability is a tested, multi-channel claim

Status: accepted, extended by ADR 0013
Date: 2026-08-10

The decisions below stand. ADR 0013 answers the one question this record
deliberately left open -- whether subsystem-layer components publish capability
models -- and the answer changed: `VehicleGuidance` publishes one, composed
from the vehicle's envelope and navigation's uncertainty. Read the scope
paragraph under Decision as the position at the time of writing, not as
current state; `docs/10-concepts.md` has that.

## Context

Self-contained capability assessment is the mechanism that is supposed to make
components swappable: a planner asks a component what it can achieve rather
than reimplementing its internals, and the binder validates a composition
without executing it. `docs/10-concepts.md` claimed every component published
one.

An audit found the claim was largely aspirational. One of seven resources had
a capability model at all (`Vehicle2D`), it was exposed through no interface --
`Capability` was defined inside `vehicle.py`, making it a vehicle-specific type
rather than a contract -- it had no tests, and its only consumer was a demo
printing a report. Nothing could ask an arbitrary component what it could do,
so a binder or planner would have had to special-case `Vehicle2D`, which is
precisely the coupling capability exists to remove.

A first attempt gave sensors a `SensorCapability` carrying one
`declared_sigma`. That shape was wrong, and wrong in a way worth recording:
`GnssReceiver` declares two accuracies with different units (position in
metres, velocity in metres per second), so the single-valued record silently
reported only position. A planner asking GNSS about its accuracy got a partial
answer and no indication that anything was missing.

## Decision

**A `CapabilityModel` protocol, with a deliberately unfixed return type.**
`docs/40-composition-spec.md` section 4.1 already establishes that envelope
structure varies by category and that the binder treats it as opaque. Forcing
one record onto vehicle, sensor, communicator and effector would either bloat
it with fields inapplicable to most, or flatten away what each category needs
to declare. The protocol fixes the question -- every component can be asked --
not the answer. Its arguments are loose for the same reason: a vehicle's
capability depends on its state, a sensor's does not, and an effector's will
depend on engagement geometry.

**Accuracy claims are multi-channel.** `SensorCapability` carries a tuple of
`MeasurementChannel(name, sigma, units)` rather than a single sigma. Units are
carried explicitly rather than encoded in the field name, the one place this
repository departs from its `position_sigma_m` convention: the field must stay
generic enough for a consumer to iterate channels it was not written against.
A channel a component will not deliver -- GNSS velocity when velocity aiding is
disabled -- is omitted entirely rather than reported with a meaningless value.

**`rate_hz` is optional.** `Imu` and `Clock` are sampled at whatever interval
the caller chooses, so claiming a rate for them would be an invention;
`rate_hz` is `None` and `interval_s` returns `None` to match. Such sensors also
declare noise *densities* rather than per-sample sigmas, because their
per-sample accuracy is undefined until an interval is known
(`sigma = density / sqrt(dt)`); the channel's `units` field says which is being
reported.

**A capability is a claim, and claims are tested.** Every published capability
must have a test that it is honest, the same obligation ADR 0009 established
for declared uncertainty. The tests integrate the dynamics forward and check
the component delivers what it claimed.

Capability is implemented for resource-layer components only. Whether
subsystem-layer components should publish one is left open: the categories the
composition specification defines envelopes for are all resource-layer, and no
consumer has yet needed to ask a filter or a guidance law what it can achieve.

## Consequences

Every resource can now be asked the same question, and the test that says so
discovers resource modules by walking the package rather than listing them.
That was not the first attempt: a hand-written list under the name "every
resource" quietly omitted `IntegratedNavUnit`, which had no capability model at
all, and passed. A test whose name claims completeness and whose body encodes a
snapshot is worse than an obviously partial one, because it stops anyone
looking. Adding a resource without a capability model now fails.

The honesty tests were verified by deliberately breaking five capability
claims (inflated sustained turn rate, overstated endurance, understated turn
radius, inflated maximum acceleration, mass-blind capability); all were caught.
One of them only after a fix: the first endurance test missed a ten percent
overstatement entirely, because fuel flow stops once the tanks are dry, so
flying past an inflated claim leaves the remaining fuel at zero and looks
healthy. Overstating endurance is the dangerous direction, and catching it
needs an assertion that fuel still remains *shortly before* the claim. A
capability test that only checks one side of a claim can be worse than none,
because it looks like coverage.

Nothing consumes capability yet. `VehicleGuidance` enforces admissibility
through `project_command()`, not through `capability()`, so the "planner
queries instead of reimplementing" claim remains unexercised by anything but
tests and a demo print. Until a real consumer exists the interface is shaped by
what seemed useful rather than by what was needed, and should be expected to
change when one arrives.

Explicit `units` strings are unvalidated. Nothing stops a component declaring
`"m"` where it means `"m/s"`, and no consumer currently checks. This is weaker
than the field-name convention it replaces, and is accepted because the
alternative -- silently dropping channels -- was worse.
