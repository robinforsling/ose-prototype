# 0016. Only the promised envelope carries the mass margin

Status: accepted
Date: 2026-08-11

Extends ADR 0015, which made the vehicle manager the owner of the platform's
believed mass, and completes the dead-reckoning work that record listed as its
intended next step.

## Context

ADR 0015 shipped the manager with a sum rather than a filter and stated the
cost: the belief was stale between gauge readings, and the published sigma
described the measurement rather than the belief, so it was a floor and not a
bound. Nothing could safely consume it. The obvious consumer — reporting a
capability envelope that accounts for how well the mass is actually known —
was therefore deferred rather than built on a number that did not yet mean
anything.

Fuel is now tracked by a two-state filter over `[fuel_kg, tsfc_error]` that
predicts on commanded thrust and corrects on each measurement, checked by an
ensemble ANEES test through the run. The sigma is a bound. The question of
what should consume it comes back.

The naive answer — evaluate everything at `mass + k*sigma` — is wrong, and
wrong in a way worth recording, because it is wrong differently in each of the
three places the manager is asked about the vehicle:

  - the **envelope published upward**, which a planner uses to decide whether
    a leg is flyable at all;
  - the **thrust feedforward**, which guidance uses to hold speed through a
    turn;
  - **enforcement**, which clips a command against the vehicle's admissible
    sets and reports the clipping.

Only the first is a promise. Feedforward is a computation: thrust worked out
for an aircraft heavier than the real one is not cautious, it simply
accelerates, and the speed setpoint is missed in the direction nobody asked
for. Enforcement is a statement about the airframe: clipping against a margin
would make a `Saturation` finding mean either "the vehicle cannot do this" or
"the estimator is unsure", and ADR 0006 exists precisely so that finding means
one thing.

## Decision

`VehicleManager.capability_bound(own_state)` reports the
envelope at `mass + capability_margin_sigma * sigma`, where sigma is the live
fuel uncertainty from the filter. `capability()` continues to report at the
believed mass, unchanged.

`VehicleGuidance.capability()` — the envelope it publishes to the planner —
uses `capability_bound()`. Its control law and the manager's
`project_command()` both continue to use the point estimate.

The rule in one line: **`capability()` for what you compute with,
`capability_bound()` for what you promise.**

`capability_bound()` returns a `PromisedEnvelope`, not the vehicle's
`Capability`, and the narrowing of the record is as much the decision as the
margin itself.

Adding mass is conservative for a *manoeuvre* limit: a heavier aircraft turns
no faster, pulls no more g, needs more room and stalls no slower. It is not
conservative for everything the vehicle reports, and the exceptions divide
into two kinds.

**Anti-conservative.** Mass uncertainty here *is* fuel uncertainty, and fuel
enters the capability model twice with opposite senses. A heavier aircraft
manoeuvres worse but is carrying more fuel, so evaluating at the margined mass
reports a *longer* endurance and a *larger* fuel quantity than the point
estimate. Those two channels must not appear in a promise.

**Non-monotone.** `accel_max_mps2` and `accel_min_mps2` move both ways across
the speed range, because mass enters the induced drag and the division by
mass. No single signed margin can be conservative for them.

`thrust_required_N` and `v_corner_mps` are excluded for a third reason: they
are not capabilities. A required thrust is an input to a control law, which
must use the point estimate, and a characteristic speed is neither better nor
worse when it moves.

The remaining six channels, plus the mass and margin they were evaluated at,
are what `PromisedEnvelope` carries. A table in the test module names the
required direction of every field and asserts that no field lacks one, so
adding a channel without deciding which way the margin should move it fails
rather than silently joining the promise. A second test sweeps the speed range
— the binding limit changes from structural to lift-limited — and asserts each
direction holds in both regimes.

`GuidanceCapability` gains `mass_margin_sigma`, and `PromisedEnvelope` carries
both that and the mass it was evaluated at, so a consumer can tell a promise
from a point estimate without knowing how the platform was configured. The two
coincide numerically once a filter has converged, which is exactly when a
planner would be unable to tell them apart by inspection.

## Consequences

The margin is scaled by the live uncertainty, so it is neither decoration nor
a permanent tax. Before the first gauge reading the sigma is the configured
200 kg and three of them cost about four per cent of the reported turn rate;
after thirty readings the sigma is a few kilograms and the cost falls below
half a per cent. The platform stops promising less than it can do as soon as
it has grounds to. Both ends are pinned by a test, because a margin that never
bites and a margin that always bites are both defects and neither is visible
from the code.

**The margin is almost invisible in normal flight, and that is the honest
result rather than a disappointment.** Three sigma of a 2 kg uncertainty
against 15 tonnes changes nothing measurable, so no demo in this repository
shows it doing anything dramatic. It earns its place in the cases the demos do
not currently cover: the startup window before the gauge has spoken, a gauge
failure where sigma grows without bound, and eventually a payload whose mass
is poorly known or stores released mid-run. A reader should not conclude from
the demos that the mechanism is doing work; they should conclude that the
platform currently knows its mass well.

**Three call sites still have to be right about which question they are
asking**, though the return types now differ, which helps.

This record initially shipped with `capability_bound()` returning the
vehicle's full `Capability`, on the reasoning that a distinct type would
merely restate the vehicle's own fields with nothing added. That reasoning was
wrong, and the way it was wrong is the most useful thing here. Every field
returned *was* a true statement about the vehicle at the margined mass, so
nothing was fabricated — and `endurance_s` came back 674 seconds longer than
the point estimate while wearing the name of a bound. A consumer planning fuel
against it would have planned a mission it could not fly. Truthful and
dangerously misleading are not exclusive, and the information a narrower type
adds is not the fields it carries but *which fields the margin is valid for*.

It was found by asking what a demo of the margin would plot, before writing
the demo. The uniform-conservatism test that was supposed to protect this
covered only the three channels `GuidanceCapability` republishes, so it passed
throughout — a test that checked the property on the fields that happened to
be consumed rather than on the record actually returned.

**A conservative envelope is not a safety argument.** It narrows what the
platform promises; it does nothing to stop a control law commanding outside
the envelope, which remains enforcement's job and remains evaluated against
the airframe. Nothing here should be read as runtime assurance.
