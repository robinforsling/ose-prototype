# 0027 — `sensing.fuel.v2` names the quantity actually measured

**Status:** Accepted. Amends ADR 0026.

## Context

ADR 0026 fixed a payload double-count: the fuel gauge reports mass above dry,
the vehicle manager decomposes mass as dry + payload + fuel, and correcting on
the raw reading put the payload into the fuel state and added it again in the
sum. A platform with 500 kg of stores believed itself 500 kg heavy at a stated
sigma of 1.4 kg.

That ADR left the field called `fuel_remaining_kg` and argued the docstrings
could carry the clarification, since renaming a published field costs a major
version increment.

That was the wrong call, and the ADR's own consequences section said why
without following the thought through: *"a reader who trusts the field name and
not the docstring can still make the original mistake."*

The name is not incidental to the defect. It **is** the defect. A consumer
reading `measurement.fuel_remaining_kg` has no reason to suspect it is not fuel
remaining, and therefore no reason to read the docstring that says so. The
original mistake was made by someone doing exactly that, and leaving the name
in place leaves the trap armed for the next one. Documentation that exists to
warn against a name is weaker than a name that needs no warning.

## Decision

**The fields are renamed to say what they carry:**

| v1 | v2 |
|---|---|
| `fuel_remaining_kg` | `mass_above_dry_kg` |
| `fuel_remaining_sigma_kg` | `mass_above_dry_sigma_kg` |

The `MeasurementChannel` the gauge declares is renamed to `mass_above_dry` to
match, so the capability and the record agree.

**The interface goes to `sensing.fuel.v2`.** Renaming a field on a published
record is a breaking change, and `docs/interfaces/README.md` states the rule:
two components bind only if the interface names match and the major versions
are equal. Nothing built against v1 can bind v2 by accident.

**The family stays `sensing.fuel`.** It names the sensor, which really is a
fuel gauge; the field names the quantity, which is not fuel. Renaming the
family as well would say the component had changed, and it has not.

## Consequences

The trap is disarmed at the point a consumer meets it. Reading
`mass_above_dry_kg` and treating it as fuel is now a visible mistake rather
than an invisible one.

**This is the first interface version increment in the repository**, and the
first exercise of a rule that had been written down since the interface
catalogue existed. Worth noting what it cost: two field renames, one channel
name, one `INTERFACE` constant, four call sites, and three documentation
updates. The rule was cheap to follow because the registry, the catalogue and
the generated tables all knew where the interface was used — before ADR 0020
this would have been a grep and a hope.

**Three guards caught the change independently**, which is the machinery
working as intended: the diagram went stale, the edge-label test found a name
not in the port catalogue, and `test_every_implemented_interface_is_written_up`
found a documented section that no longer matched. None of them had to be
looked for.

**Nothing external is broken, because nothing external exists.** This
repository has no downstream consumers, so a major increment costs nothing here
that it would cost in a released library — and that is exactly why it was worth
doing now rather than later. The next such rename will not be free.

**ADR 0026's consequence about keeping the name is now false** and has been
revised. Its Context and Decision stand: the manager still reconciles, and the
gauge still knows nothing about payload.

## References

- ADR 0026 — the double-count, and the decision this amends
- ADR 0020 — the registry that made the rename traceable
- `docs/interfaces/README.md` — the versioning rule this is the first use of
