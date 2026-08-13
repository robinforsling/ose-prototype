# 0024 — Layer discipline is enforced, not documented

**Status:** Accepted.

## Context

"Composition proceeds bottom-up. A layer may bind to the layer below it and to
peers within the same layer on the same platform. Nothing binds upward." That
sentence is in `docs/10-concepts.md`, in `CLAUDE.md`, and in the layer table in
`docs/20-architecture.md`.

Nothing checked it.

What existed was two calls to `assert_no_equipment_imports`, one in
`tests/test_time_estimator.py` and one in `tests/test_action_planner.py`, each
naming its own component by hand at the call site. Four of the six cyber
components were covered by nothing, and a seventh would have been covered by
nothing unless whoever added it remembered to add a third call.

This is the shape of failure this repository has recorded three times already:
a hand-written list that omitted the integrated navigation unit, a glob that
stopped covering the vehicle when `vehicle.py` became `vehicle/`, and six
truth-boundary guards naming their subject with a literal module string. In
each case the check went on passing while covering less, and in each case the
fix was to discover rather than to list.

The truth boundary — the other invariant of this kind — is enforced three ways:
import guards, parameter-name guards, and the architecture generator. Layer
discipline had one and a half.

## Decision

**The rule becomes executable**, as `binding_is_allowed(consumer, provider)` in
`ose/topology.py`, next to the layer table it depends on. "The layer below" is
read as the layer *directly* below: a gap of 0 (peers) or 1. A single-ship
component binding equipment is refused, because it would skip the subsystem
that exists to integrate that equipment.

**It is checked in two places, because they catch different things.**

`tests/test_layer_discipline.py` walks every component module — discovered, not
listed — and refuses an import from a layer the module may not bind.

`tools/generate_architecture_diagram.py` checks the *derived bindings* and
exits non-zero on a violation, exactly as it does for the truth boundary.

The second is not redundant. A component bound through a protocol in
`ose.interfaces` has **no import naming what it binds**, so an upward binding
can arrive with nothing in the import check to see it. That was verified by
planting one: giving `VehicleGuidance` an `ActionPlanner` parameter creates a
subsystem component binding a single-ship component, the import check passes,
and the generator reports `VehicleGuidance (subsystem) binds WaypointPlanner
(single_ship) -- upward`.

Since a protocol *is* the intended way to declare a port, that is the path an
upward binding would most plausibly take.

## Consequences

The rule now fails the suite instead of being remembered, and a component added
tomorrow is covered without anyone choosing to cover it.

`assert_no_equipment_imports` on `WaypointPlanner` is now subsumed by the global
check. It is kept: it costs nothing, and a redundant assertion about a specific
component is not the same claim as a general one.

**The call on `TimeEstimator` is *stricter* than this rule and stays for its own
reasons.** A subsystem component may bind equipment; that one specifically must
not, because it is a pure filter over a measurement stream. Nothing here
distinguishes an architectural rule from a per-component choice, and a reader
could mistake the second for the first.

**An import is not a binding.** The import half refuses a module *naming* a type
from a layer it may not bind, which is stricter than the stated rule and simpler
to check. A module that imported something purely for an annotation it never
holds would be refused. That has not happened, and the stricter reading is the
one worth having.

**The adjacent-only reading is an interpretation.** The documents say "the layer
below", singular, and that is how it is now enforced. If a component ever has a
legitimate reason to reach two layers down, this will be what refuses it, and
the argument will have to be had explicitly — which is the point, but it is
still a constraint that did not bite before.

**A discovered check has a vacuity failure mode.** If the walk found no modules
every assertion would pass, so the walk is asserted non-empty and the rule is
pinned against the documented table independently of the components it is
applied to.

## References

- ADR 0008 — the truth boundary, enforced the same way and for the same reason
- ADR 0017 — the rename that nearly invalidated the truth guards, and why
  vocabulary lives in `ose/topology.py`
- ADR 0020 — the generator that already knew every component's layer
- ADR 0021 — protocol-typed bindings, which is why the import check is not enough
