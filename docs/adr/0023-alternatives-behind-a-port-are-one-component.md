# 0023 — Alternative implementations of a port are one component

**Status:** Accepted.

## Context

The generated diagram drew `PlanarPointMass` and `PlanarPointMassWithBooster`
as two nodes, with `Imu` and `VehicleManager` connected to the first and
nothing connected to the second.

That was faithful to the annotations and wrong about the code.
`VehicleManager.__init__` said `vehicle: PlanarPointMass`, while
`tests/test_vehicle_manager.py` has been building a manager on
`reference_boosted_fighter()` — a `PlanarPointMassWithBooster` — for as long as
that model has existed. `Imu` said the same and touches only `drag_N`. Both
were duck-typed all along, the two models share no base class and no protocol,
and nothing could tell.

It is also wrong about the architecture in a second way. A platform carries one
vehicle. The two models are alternatives selected at composition time, not
collaborators, and a picture showing both invites the reading that a platform
has both.

## Decision

**A `Vehicle` protocol** in `ose/interfaces.py`, stating what its consumers
actually use — `dry_mass_kg`, `drag_N`, `capability`, `project_command`,
`state_from` — and nothing else. `Imu` and `VehicleManager` are annotated
against it.

**It is not `runtime_checkable`, and cannot be.** `dry_mass_kg` is a property,
and `issubclass` against a protocol with a non-method member raises
`Protocols with non-method members don't support issubclass()`. Dropping it to
keep `issubclass` working would mean a port that cannot state something its
consumer reads, which is the wrong way round. Conformance is checked
structurally instead, by member name.

**When a protocol port resolves to more than one provider, the diagram draws
one node named for the port**, and retargets the members' edges onto it. The
graph records what it collapsed, so nothing is lost to a reader or to a test. A
port with a single provider does not collapse: naming that node after the port
would hide which implementation is in the tree.

## Consequences

The annotations now say what the code does, which is the substantive half of
this. A model that stopped providing the port would break a consumer that never
named it, and a test asserts both models still provide it.

The diagram shows one `Vehicle`, which is what a platform has.

**A port states less than an implementation offers.** `Vehicle` names five
members; `PlanarPointMass` has around twenty, including `derivative` and
`admissible`. That is deliberate — a port says what a consumer may rely on —
but it means the protocol has to be widened whenever a consumer starts using
something new, and nothing detects the omission until it is written.

**The collapse hides which model is composed.** The diagram can no longer
answer "is this the boosted one", because the answer is a composition decision
and the diagram describes the architecture. Anyone wanting that will need the
composition spec, which does not exist yet.

**`issubclass` is no longer usable for protocol conformance here**, so the
generator and the tests each do their own structural check. Two structural
checks are two things that can disagree; they are in the same repository and
tested against the same models, which is the mitigation rather than a solution.

**A property in a protocol is invisible to `isinstance` too** for subclass
checks, so `tests/test_capability.py`'s `runtime_checkable` pattern does not
extend to `Vehicle`. The conformance test walks members by name instead, and
asserts it found some — a protocol that declared nothing would otherwise pass.

## References

- ADR 0015 — why `VehicleManager` is the only component binding a vehicle model
- ADR 0020 — the diagram that surfaced the mismatch
- ADR 0021 — the protocol-versus-concrete distinction this relies on
