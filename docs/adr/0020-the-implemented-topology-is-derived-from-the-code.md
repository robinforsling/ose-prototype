# 0020 — The implemented topology is derived from the code

**Status:** Accepted. Amended by ADR 0021, which moves an interface name off
the record and onto the port.

## Context

Three documents described how the components in this repository connect, and
none of them was checked against the code.

`docs/20-architecture.md` opened with an ASCII drawing of the five layers. It
is correct about the layers and silent about the wiring — which component
publishes what, and what consumes it — and being hand-drawn it would have gone
stale the first time a component was added, while continuing to look
authoritative.

`docs/interfaces/README.md` carried a catalogue of nineteen interfaces named
`family.name.vN`, with a direction and a status for each. `docs/40-composition-spec.md`
carried a second copy of the same table. Interface names appeared in `src/`
only inside docstrings, never as constants, so nothing could compare either
table with anything.

They had already drifted, in three separate ways, and each one had been sitting
there unnoticed:

- `vehicle.mass.v1` had a prose section in the catalogue and no row in the
  table.
- `FuelMeasurement` had no interface name at all — the other four sensing
  records were catalogued and this one was simply missed.
- The two copies of the table disagreed about `vehicle.state.v1`: the
  composition spec said `equipment → subsystem`, the catalogue said
  `subsystem to above`. The catalogue was right; `NavigationManager` is a
  subsystem component, which is the whole subject of ADR 0014.

That is the same failure this repository has already recorded twice at the code
level — a hand-written list that omitted the integrated navigation unit, and a
glob that stopped covering the vehicle when `vehicle.py` became `vehicle/`. In
both cases the fix was to discover rather than to list, and in both cases the
list had gone on passing while checking less and less.

A diagram is the worst possible place for that failure, because a picture is
trusted in proportion to how specific it looks.

## Decision

**Interface names live in the code, on the record that carries them**, as an
`INTERFACE` class variable. A record either has one or is declared in
`NOT_A_PORT` with the reason; `catalogue()` raises on any record that is
neither. The negative declaration is load-bearing — an unregistered record is
otherwise indistinguishable from one that deliberately is not a port, and
several records legitimately are not.

**The topology is derived by walking the layer packages**, not listed.
Components, their layer, their publications and their consumptions all come
from the source. `tools/generate_architecture_diagram.py` does the walking and
writes two generated blocks: a Mermaid diagram in `docs/20-architecture.md`,
and the implemented half of the catalogue table in
`docs/interfaces/README.md`. `pytest` runs it with `--check` and fails while
either is stale.

**The diagram shows only what is implemented.** Layers with no components do
not appear, and neither does the unbuilt simulation core. The ASCII drawing
stays, because it shows the intended shape, which is a different claim.

**The duplicate catalogue in `docs/40-composition-spec.md` is deleted**, and
replaced by a pointer. Two tables that must agree and cannot be checked against
each other will disagree.

**Shared presentation values live in `prefs/`**, as data. `prefs/palette.json`
maps semantic roles — `equipment`, `subsystem`, `truth` — to fill, stroke and
text colours, so that the diagram and, later, the plots and animations, colour
the same concept the same way without either author deciding it separately.

## Consequences

The diagram cannot drift. Adding a component or an interface fails the suite
until it is regenerated, and what appears is what the code does, not what
someone believed it did.

The generator became a second enforcement of invariant 1. It collects the
components that read truth and exits non-zero if any sits outside a physical
layer, so the picture and the boundary cannot disagree. That was not the reason
for building it, and it is the most useful thing it does.

**It understates coupling, and cannot do otherwise.** `FuelGauge` depends on
the vehicle through `mass_dry_kg: float`. No signature carries where that float
came from, and inferring a component dependency from a scalar's name would
invent edges elsewhere. The diagram therefore omits a real dependency. It
reports the omission rather than hiding it, which is the best available
outcome, not a good one.

**`Clock` reads truth and is not shown doing so.** Every other sensor takes a
`true_`-prefixed argument. A clock's truth input is the elapsed interval,
deliberately unprefixed because for a clock the interval *is* the corrupted
quantity (ADR 0010). The rule that finds truth readers cannot see it.

**Consumption recovery is fragile by construction.** `ingest()` and `command()`
take unannotated parameters and dispatch on type, so the consumer side is
recovered by walking `isinstance` calls in the method body. Rewriting such a
dispatch as a `match` statement or a dict lookup would silently drop edges. The
symptom is a staleness failure plus a warning line, which is visible but
indirect. `NavigationManager.ingest` already produces that warning legitimately
— it forwards rather than dispatching.

**A protocol does not identify a publisher on its own.** `issubclass` against a
`runtime_checkable` Protocol compares method *names* only, not signatures, so
`TimeEstimator` satisfies `OwnStateSource` despite returning the wrong record —
and the first version of the generator bound the navigation manager to the
clock estimator on that basis. This is worth knowing about beyond this tool:
nothing in the type system stops that substitution. *(The fix recorded here
was to require the candidate to publish what the protocol publishes. ADR 0021
replaced it with a structural match on member names plus the record a method
returns, because comparing interface names breaks as soon as a component
publishes on its own port.)*

**Both `vehicle.state.v1` publishers were drawn**, giving six edges where ADR
0014 says a platform has one publisher. The alternative considered was a rule
suppressing the wrapped publisher, which is an interpretation rather than a
derivation and would have fired exactly once; the diagram showed what the code
offered and left ADR 0014 to say which a consumer should bind. *(That picture
is what prompted ADR 0021 and ADR 0022: the code really did have two
publishers, and the right fix was to the code rather than to the drawing.
There is now one.)*

**The palette is hardcoded hex, and the contrast tests are a proxy.** They
check WCAG luminance ratios, pairwise fill separation, and that fills stay
bright against GitHub's dark canvas. Nothing here renders a GitHub page, so
passing them makes legibility likely rather than certain. Mermaid rendering
could not be verified programmatically either: mermaid 11 needs Node 18 and the
environment has Node 12, so the emitted syntax is checked structurally —
dangling endpoints, undefined classes, unbalanced subgraphs — and confirmed by
eye.

**A second generator to keep working**, alongside `generate_model_docs.py`.
They share conventions and no code; `render()` is duplicated deliberately,
since `tools/` is not a package and the two are otherwise independent.

## References

- ADR 0008 — the truth boundary this diagram draws and the generator enforces
- ADR 0012, 0013, 0016 — why capability is not a port, and therefore why
  `capability()` produces no edges
- ADR 0014 — one navigation publisher per platform
- ADR 0017 — the rename that nearly invalidated the truth guards, and the
  reason the layer vocabulary now lives in `src/ose/topology.py`
