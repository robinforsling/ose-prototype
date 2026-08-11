# 0015. The vehicle manager owns the platform's believed mass

Status: accepted, extended by ADR 0016
Date: 2026-08-10

Amends ADR 0011 (the mass simplification it recorded is resolved) and ADR 0013
(guidance now composes the manager's capability, not the vehicle's).

## Context

ADR 0011 gave `VehicleGuidance.command()` a plain `mass_kg` parameter and said
plainly why: nothing in the repository estimated mass, and inventing an
estimate would have been worse than admitting the gap. It also named the
consequence — that a caller-supplied mass "is exactly the kind of implicit
truth-reading ADR 0008 exists to prevent, once a real fuel-accounting
component exists to supply a believed mass instead."

That gap turned out to be wider than one parameter. Guidance itself was clean:
`test_guidance_cannot_see_truth` passed, because guidance never imported
`VehicleState` and never took a `true_` argument. But every *composition* of
guidance leaked. Nineteen of twenty call sites across tests and demos passed
`state.mass_kg` — the true mass, read straight off the simulation's own state.
The boundary held inside the component and failed everywhere the component was
used, which is the failure mode ADR 0008 warns is hardest to notice: each
individual call looks reasonable, and the component's own test suite is green.

Meanwhile `FuelGauge` had existed since the capability sweep with no consumer
at all. It published a `FuelMeasurement` carrying a declared sigma, and nothing
read it.

A first sketch had the new component publish a mass and nothing more, with
guidance keeping its `PlanarPointMass` reference. That does not work. Guidance asks
the vehicle three different things, and only one of them is a capability read:
it needs the envelope, a *parametrised* query for the thrust that holds a given
turn rate (the feedforward that fixed the 1330 kN command), and
`project_command()` for enforcement. A component that republished only a
`Capability` record would have left guidance holding the vehicle anyway — and
holding the vehicle is what forces a mass argument, because every one of those
queries takes a state whose `mass_kg` field is the unestimated quantity.

## Decision

`VehicleManager` (`subsystem/vehicle_manager.py`) owns the platform's believed
mass and is the only component permitted to bind `PlanarPointMass`.

It is best understood as **`PlanarPointMass` with the mass argument closed over by a
believed value** — a partial application, not a pass-through. It consumes
`FuelMeasurement`, publishes `vehicle.mass.v1` as a `MassEstimate`, owns the
`as_vehicle_state()` conversion that guidance used to perform, and answers the
three vehicle questions above at the mass it believes.

Mass is `dry + payload + fuel`. Dry mass is a vehicle design constant and
payload is a configuration decision, so both are exact; only the fuel term is
measured. The published `mass_sigma_kg` is therefore the fuel term's alone.
(As shipped here it was exactly the sigma that travelled with the
`FuelMeasurement`. Under ADR 0016 it comes from the filter's covariance, and
the measurement's declared sigma sets the gain rather than the output; the
manager still substitutes no configured number of its own, per invariant 4.)

`VehicleGuidance` binds to the manager — a peer in the same layer on the same
platform — instead of to `PlanarPointMass`. It no longer takes `mass_kg`, no longer
constructs a believed `VehicleState`, and contributes only what it alone knows:
how tightly navigation can hold what the vehicle can reach.

    PlanarPointMass      physics, needs a mass
      +- VehicleManager      binds believed mass
           +- VehicleGuidance    adds navigation uncertainty
                +- WaypointPlanner

Each layer adds exactly what it knows.

The sole-consumer rule is enforced as an import rule, because "only the manager
consumes vehicle capability" is not decidable at a call site without type
inference, while "only the manager holds a `PlanarPointMass`" is, and it is the same
rule with a checkable edge. Three exemptions, each for its own reason: the
equipment layer owns the vehicle and `Imu` is a peer rather than a consumer
above it; `ose/integration.py` steps the model instead of asking it what it can
do, which is the simulation core's job living outside the components per ADR
0004; and the manager itself.

The estimator is a sum, not a filter. The last fuel reading is used as it
stands. *(Superseded by ADR 0016: a two-state filter now predicts on commanded
thrust and corrects on each reading.)*

## Consequences

The truth leak is closed at the composition level, not merely inside one
component. There is no mass parameter for a caller to supply, so there is
nothing to reach for. `FuelGauge` has a consumer for the first time, and the
demos now run the full chain `FuelGauge -> VehicleManager -> VehicleGuidance`;
over the route demo the vehicle burns about 420 kg and the belief stays within
roughly 65 kg of truth, which is gauge noise rather than drift.

**Superseded by ADR 0016 as of 2026-08-11.** Fuel is now tracked by a
two-state filter that predicts on commanded thrust and corrects on each
measurement, and the sigma is a bound rather than a floor. The paragraph below
describes the sum-only version this record shipped, and the reasoning that
made a filter the next step; it is left as written because the staleness it
predicted is exactly what showed up.

**The belief is stale between measurements, and the published sigma does not
say so.** Mass falls continuously at a rate the platform could predict, because
`mdot` follows from the commanded thrust, and nothing here uses that. The sigma
describes the measurement rather than the belief, so it is a floor and not a
bound. Do not build a consistency test against it and conclude the belief is
calibrated. The dead-reckoning version — predict on commanded thrust, correct
on each measurement, grow the covariance in between — is the same structure
`TimeEstimator` already uses for the clock (ADR 0010) and is the intended next
step. This was chosen deliberately: the plumbing change and the honesty
question are separable, and doing them together would have made it impossible
to tell which one broke a number.

That staleness became visible immediately, which is the argument for having
made it explicit. `test_unreachable_turn_rate_saturates_and_stays_saturated`
flies for sixty seconds while nothing feeds the manager, and started failing
because the vehicle burned several hundred kilograms the manager never heard
about. The assertion was rewritten against the believed mass, because asserting
against the true one would have been asserting that guidance can read truth.
A reader should take the lesson generally: any test that flies long enough for
mass to matter, and does not wire up a gauge, is testing a stale belief.

**The manager is an indirection that must forward every future vehicle
query.** Adding a channel to `PlanarPointMass` now means editing two files, and a
consumer wanting something the manager does not forward has to change the
manager rather than reach past it. That is the price of the boundary, and it is
deliberately not softened by leaving a public accessor for the underlying
model — one would be used, and the rule would decay.

**A second fuel sink would not announce itself.** The prediction assumes
thrust is the only thing burning fuel. A power generator, which is planned,
would be a second sink — and measured over 150 s of steady cruise an
unmodelled drain of one to three per cent of the thrust burn leaves ANEES at
about 1.1. The `tsfc_error` state absorbs it, because at constant thrust a
constant drain looks exactly like a slightly wrong burn coefficient. The
filter stays calibrated while its coefficient estimate quietly goes wrong,
which is worse than a clean failure, and the absorption only holds while
thrust is constant. Whoever adds a generator must give it its own term in the
prediction and re-check consistency at varying thrust; the module docstring
carries the numbers.

Payload is a single configured scalar and nothing sets it non-zero yet.
Effectors and stores released during a run would each contribute a term and
change the sum at discrete instants; the record already publishes the
contributions separately, so that arrives as a field rather than a version
increment.

Guidance's capability is still genuinely composed, but from a different pair:
the vehicle half now arrives already evaluated at the believed mass. ADR 0013's
reasoning is unchanged and its wording is amended to name the manager.
