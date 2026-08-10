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

`VehicleManager.capability_bound(own_state, omega_rad_s=0.0)` reports the
envelope at `mass + capability_margin_sigma * sigma`, where sigma is the live
fuel uncertainty from the filter. `capability()` continues to report at the
believed mass, unchanged.

`VehicleGuidance.capability()` — the envelope it publishes to the planner —
uses `capability_bound()`. Its control law and the manager's
`project_command()` both continue to use the point estimate.

The rule in one line: **`capability()` for what you compute with,
`capability_bound()` for what you promise.**

Adding mass, rather than applying a per-channel rule, is sound only because
heavier is uniformly worse across every channel an envelope publishes: a
heavier aircraft turns no faster, stalls no slower, and the airframe speed
limit does not move with mass at all. A single signed margin therefore narrows
the claim in every direction at once and can never widen it.
`test_the_bound_is_never_wider_than_the_estimate` sweeps the speed range and
asserts it, because the binding limit changes with speed — structural at high
speed, lift-limited at low — and the property has to hold in both regimes. If
a future channel is anti-conservative in mass, that test fails and this
decision needs revisiting rather than patching.

`GuidanceCapability` gains `mass_margin_sigma`, so a consumer can tell a
promised envelope from a point estimate without knowing how the platform was
configured. The two coincide numerically once a filter has converged, which is
exactly when a planner would be unable to tell them apart by inspection.

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

**Three call sites now have to be right about which question they are
asking**, and the compiler cannot help. The two methods differ by one word and
return the same type. This was weighed against introducing a distinct
`PromisedCapability` record, which would have made the distinction
type-checkable; that was rejected because every field would have restated the
vehicle's own `Capability` with no added information, and the type boundary
that actually matters already exists one layer up in `GuidanceCapability`,
which is what a planner receives. The mitigation is four tests that pin each
call site to the right side of the split, all four verified by sabotage.

**A conservative envelope is not a safety argument.** It narrows what the
platform promises; it does nothing to stop a control law commanding outside
the envelope, which remains enforcement's job and remains evaluated against
the airframe. Nothing here should be read as runtime assurance.
