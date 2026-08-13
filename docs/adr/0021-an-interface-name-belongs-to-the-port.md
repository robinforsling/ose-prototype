# 0021 — An interface name belongs to the port, not the record

**Status:** Accepted. Amends ADR 0020.

## Context

ADR 0020 put interface names in the code, as an `INTERFACE` class variable on
the record that carries them. That fixed three documented drifts and made the
architecture diagram possible. It also asserted something that turns out not to
be true in general: that a record determines its interface.

The generated diagram showed `vehicle.state.v1` with two publishers,
`InsGnssEstimator` and `NavigationManager`, feeding three consumers — six
edges, where ADR 0014 says a platform has exactly one publisher of its own
state. The code really did work that way: a consumer could bind either, and
both were correct as far as any check could tell.

The intended architecture is that a navigation *source* feeds the manager, and
the manager alone publishes the platform's state. Expressing that requires the
two to be distinguishable. They were not, because both return an
`OwnStateEstimate` and the interface name hung off the record.

That is a category error rather than an oversight. **A topic and a message type
are different things.** The same message type routinely travels on several
topics, meaning different things on each; "the estimate this source produced"
and "the state this platform publishes" are different claims about the same
numbers. Tying the name to the record made the second claim unsayable.

The same applies to timing. `TimeEstimator` and `NavigationManager` both
publish a `TimeEstimate` (ADR 0022), for the same reason and with the same
collision.

## Decision

**A record's `INTERFACE` is the default, not the definition.** Most records
serve exactly one port and nothing changes for them.

**A component may name the port a method publishes on**, overriding the
default:

```python
class InsGnssEstimator:
    PUBLISHES: ClassVar[dict[str, str]] = {"estimate": "vehicle.state_source.v1"}
```

Two ports are added, reusing existing families so the pairing with what the
manager republishes is visible in the name rather than needing to be looked up:

| Port | Interface | Record |
|---|---|---|
| what a navigation source supplies | `vehicle.state_source.v1` | `OwnStateEstimate` |
| what a time source supplies | `platform.time_source.v1` | `TimeEstimate` |

**Only the publisher side needs it.** Every consumer annotates the record and
wants the default; the manager receives its sources through bindings rather
than parameters.

**The full port catalogue is composed by the generator**, not by
`ose.interfaces`. The module holds contracts and no implementations, and
components import it — so it cannot import them to discover their overrides.
The record defaults live there; the generator merges the overrides and writes
the catalogue table.

**A constructor parameter typed by a protocol is a declared port**, and draws a
publication edge from provider to consumer labelled with the provider's port. A
parameter typed by a concrete class is composition, and draws an unlabelled
one. This is what makes the source-to-manager edge derivable without inventing
others: `VehicleGuidance` binds `VehicleManager` concretely, so nothing claims
guidance consumes `vehicle.mass.v1` — it calls `capability_bound()` and never
takes a `MassEstimate`.

## Consequences

The manager is now the platform's only publisher of `vehicle.state.v1`, and
that is enforced by what the code says rather than by a rule in an ADR. Binding
a consumer straight to the estimator is still possible, but it now means
binding a different interface, which a composition-time check can refuse.

**Two mechanisms where there was one.** A reader asking what interface a record
travels on must check the record *and* whether its publisher overrides. That is
a real cost, accepted because the alternative — a record per port — would have
duplicated a ten-field dataclass to express a routing distinction.

**`ose.interfaces.catalogue()` is no longer the whole catalogue.** It is the
record defaults. Anything wanting every port name has to ask the generator,
which is a worse place for it to live and the only place it can live while
`interfaces.py` holds no implementations.

**The protocol-versus-concrete distinction is load-bearing and undeclared.**
Nothing states that a concrete-typed binding is deliberately not a port. Today
`VehicleGuidance ==> VehicleManager` is the only one, and whether it *should*
be a port is an open question this ADR does not answer — the diagram now makes
it visible, which is the point.

**Protocol resolution had to be rewritten.** It compared interface names, which
this change breaks by construction, and used `issubclass`, which compares
method names only and raises outright on a protocol with a property member. It
now matches structurally on member names plus the *record* a method returns.
See ADR 0023 for the property member that forced the second half.

## References

- ADR 0020 — the registry this amends
- ADR 0014 — one navigation publisher per platform, now structural
- ADR 0022 — the same collision, for timing
