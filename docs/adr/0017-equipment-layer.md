# 0017. The resource layer is renamed the equipment layer

Status: accepted
Date: 2026-08-11

Renames the layer named in ADR 0008 and referenced throughout. No behaviour
changes; every earlier record has been updated, so a reader will not meet the
old name.

## Context

"Resource" was already doing two jobs. The composition specification uses it
in its ordinary sense for the consumables a component draws when mounted —
electrical power and cooling — in the same document, and at one point in the
same line, as the layer name:

```
# Physical resources this component consumes when mounted. Resource layer only.
```

A reader has to work out which sense is meant from context, and there is no
context that settles it. The collision would only get worse: a power model is
in scope, and it would have to talk about resources constantly.

"Equipment" carries the meaning the layer actually has — the parts of a
platform with a physical existence, which is exactly what ADR 0008 grants
privileged access to truth.

The rename is cheap now and gets steadily less so. The descriptor format
declares `layer: resource` as an enum value, keys each platform's components
under a `resource:` block, and addresses them in Monte Carlo selector paths.
Nothing consumes that format yet — no registry, no binder, no descriptor
validator, no authored descriptors — so there is nothing to migrate. Once
those exist, and once a lab has scenario files on disk, this becomes a
migration rather than a substitution.

## Decision

`ose.resource` becomes `ose.equipment`, and the layer is called the equipment
layer everywhere: code, tests, demos, documentation, the descriptor format,
and earlier ADRs.

Earlier records are updated rather than left as written. This does not
conflict with the rule that an ADR's Context stays as it was: a terminology
substitution changes no reasoning and no decision, and leaving "resource
layer" in ADR 0008 would mean a reader could not tell whether it described
today's equipment layer or something since removed. Where the old name
appeared as *quoted history* — a former test name, for instance — the
sentence was rewritten to describe what the thing did rather than what it was
called, because the name no longer means anything to a reader.

Three lines keep "resources" in the ordinary sense, and are the point:

```
A component declares its ports, the resources it consumes, the resources it
# Physical resources this component consumes when mounted. Equipment layer only.
# Physical resources this component makes available. Vehicles only.
```

## Consequences

The word now means one thing in each place it appears.

**A blind substitution would have been wrong twice over, and the second way
was dangerous.**

The harmless one is grammar. "Equipment" is a mass noun, so a countable "a
resource" has no direct replacement: those became "an equipment component", or
simply the device's own name — "the clock resource" is now "the clock".

The serious one is that six truth-boundary tests compared imports against the
literal string `"ose.resource.vehicle"`. A string that no longer names
anything matches nothing, so the loops would have found no leak and every one
of those tests would have passed while checking nothing at all. This was
verified before the rename began: renaming the literal in one test and
planting a real `VehicleState` import in guidance left
`test_guidance_cannot_see_truth` green. Invariant 1 — the rule the layer
structure rests on — would have gone silently unenforced across five files.

The guard now lives in `tests/_truth_boundary.py`, which names the layer once
and, more importantly, imports the module it names before using it. A stale
name raises `ModuleNotFoundError` instead of matching nothing. Centralising
alone would not have caught the failure; the liveness check does.

**The general lesson is worth more than the rename.** A test that identifies
its subject by a string, rather than by something the runtime resolves, does
not fail when the subject moves — it stops having a subject. Any assertion
built on `ast`, on file paths, or on module names has this shape, and the
repository has several. Each should be able to prove it is still looking at
something.

**Git history is bisectable but the rename spans five commits.** Anyone
bisecting through this range will find the package under one name or the other
depending on where they land. The split was deliberate — Python, then the
guards, then documentation, then the descriptor format, then these records —
so that a regression in any one of those is attributable.
