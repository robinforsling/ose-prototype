"""
Layer vocabulary: which package is which layer, and which types carry truth.

Names only, no logic. The same role as frames.py and environment.py -- a fact
several layers need to agree on, which therefore cannot live inside any of
them.

Why this is in src/ and not in tests/
-------------------------------------
`tests/_truth_boundary.py` owned TRUTH_TYPES first, and its docstring records
what happens when such a literal goes stale: it stops naming anything, matches
nothing, and every guard built on it passes while checking nothing. Renaming
the resource layer to the equipment layer (ADR 0017) came within one edit of
exactly that.

A tool outside tests/ now needs the same vocabulary, and copying the literal
into it would recreate that failure in a second place. So the names moved down
here, where both the test guards and tools/generate_architecture_diagram.py
read them. `_truth_boundary.py` re-exports them and keeps its own liveness
guard, which now protects the tool as well.

The layer table in CLAUDE.md and docs/20-architecture.md is prose. This is the
executable copy of the one column a program can act on: layer name to package.
"""

from __future__ import annotations

# Bottom-up. A layer package is created when it acquires its first component,
# so an entry here may name a package that does not exist yet -- multi_ship
# does not today. Consumers skip what is absent rather than treating the
# layer as unknown; listing it is what makes the eventual arrival need no
# change here.
LAYER_PACKAGES = {
    "equipment": "ose.equipment",
    "subsystem": "ose.subsystem",
    "single_ship": "ose.single_ship",
    "multi_ship": "ose.multi_ship",
}

# Bottom-up, and the order is load-bearing rather than presentational: it is
# what "the layer below" means. Read through layer_index() rather than by
# indexing LAYER_PACKAGES directly, so the dependency on ordering is stated
# where it is relied on.
LAYER_ORDER = tuple(LAYER_PACKAGES)


def layer_index(layer: str) -> int:
    """How far up the stack a layer sits. Equipment is 0."""
    return LAYER_ORDER.index(layer)


def binding_is_allowed(consumer_layer: str, provider_layer: str) -> bool:
    """Whether a component in one layer may hold a reference to one in another.

    A component may bind to the layer directly below it and to peers in the
    same layer. Nothing binds upward, and nothing reaches past a layer -- a
    single-ship component binding equipment would skip the subsystem that
    exists to integrate it, which is the coupling the layering is for.

    Stated in docs/10-concepts.md and CLAUDE.md, and enforced in two places
    that catch different things: tests/test_layer_discipline.py over imports,
    which sees a component naming another directly, and the architecture
    generator over derived bindings, which sees one bound through a protocol
    with no import to give it away.
    """
    gap = layer_index(consumer_layer) - layer_index(provider_layer)
    return gap in (0, 1)


# The layers whose components have a physical part, and therefore the only
# ones that may read ground truth (invariant 1, ADR 0008). A frozenset rather
# than a single name because the boundary is a property of the layer, not of
# the one layer that happens to sit below it today.
PHYSICAL_LAYERS = frozenset({"equipment"})

EQUIPMENT_PACKAGE = LAYER_PACKAGES["equipment"]

# A package since the second vehicle model was planned, so any check has to
# cover its submodules too: importing VehicleState from
# ose.equipment.vehicle.planar_point_mass is the same leak as importing it
# from ose.equipment.vehicle, and an equality test would have missed it.
TRUTH_PACKAGE = f"{EQUIPMENT_PACKAGE}.vehicle"

# The types that carry ground truth. A cyber-layer component holding one of
# these is wrong regardless of what it does with it (ADR 0008). VehicleCommand
# and Saturation live in the same module and are not truth -- they are records
# a component may legitimately construct and receive.
TRUTH_TYPES = frozenset({"VehicleState", "Disturbance"})
