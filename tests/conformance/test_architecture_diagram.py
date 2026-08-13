"""The architecture diagram must not go stale, and must not lie.

docs/20-architecture.md carries a generated picture of the implemented
topology: every component, its layer, and the interfaces between them. It is
derived from the source (ADR 0020), so adding a component changes it, and a
document that quietly stops matching the system it draws is worse than no
document -- a reader trusts a diagram precisely because it looks specific.

Two different things are checked here. That it is CURRENT, which is the same
staleness contract the model reference pages have. And that the derivation is
HONEST: that every component reached the page, and that no component above the
truth boundary is shown reading truth.

The prose beneath the block is written, not generated, and nothing here checks
it. Whether it still explains the picture correctly is a judgement.
"""

from __future__ import annotations

import importlib.util
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GENERATOR = ROOT / "tools" / "generate_architecture_diagram.py"
PAGE = ROOT / "docs" / "20-architecture.md"

BLOCK = re.compile(
    r"<!-- generated: topology -->\n(.*?)\n<!-- end generated: topology -->", re.S
)
# `    Name["Name"]` -- a node declaration inside a subgraph, distinguished
# from a class name or a mention in the prose by the bracket that follows it.
NODE = re.compile(r"^\s+(\w+)\[", re.M)


def _load_generator():
    """Import the tool by path.

    It must be registered in sys.modules before exec: the tool defines a
    dataclass under `from __future__ import annotations`, and dataclasses
    resolves such annotations through sys.modules[cls.__module__]. Without the
    registration the import fails with an AttributeError on None.
    """
    spec = importlib.util.spec_from_file_location("generate_architecture_diagram",
                                                  GENERATOR)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def generated_block() -> str:
    match = BLOCK.search(PAGE.read_text())
    assert match, "no generated topology block in docs/20-architecture.md"
    return match.group(1)


def test_the_generated_diagram_is_current():
    """Run as a subprocess, so the test exercises the entry point a
    contributor is told to run rather than a private path into it."""
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        capture_output=True, text=True, cwd=ROOT,
    )
    assert result.returncode == 0, (
        "the architecture diagram is out of date with the code:\n"
        + result.stdout + result.stderr
    )


def test_the_walk_is_not_vacuous():
    """Every test below asks what the walk found, so all of them pass on a
    walk that finds nothing.

    The subpackage assertion is deliberate repetition of the regression
    tests/test_capability.py exists for: a glob over `*.py` stopped covering
    the vehicle the moment vehicle.py became vehicle/, and kept passing. This
    is a different walk and can fail the same way.
    """
    graph = _load_generator().build_graph()

    assert len(graph.components) >= 10, (
        f"only {len(graph.components)} components discovered"
    )
    by_layer: dict[str, int] = {}
    for layer in graph.components.values():
        by_layer[layer] = by_layer.get(layer, 0) + 1
    assert by_layer, "no layers populated"
    for layer, count in by_layer.items():
        assert count >= 1, f"layer {layer} discovered with no components"

    # Discovered, whether or not they survive as their own nodes: the vehicle
    # models collapse into the Vehicle port. The regression this guards is the
    # walk failing to descend into subpackages, which would lose them from
    # both sets at once.
    reached = set(graph.components)
    for members in graph.collapsed.values():
        reached |= members
    assert {"PlanarPointMass", "PlanarPointMassWithBooster"} <= reached, (
        "the vehicle models were not found -- the walk is not descending into "
        "subpackages"
    )
    assert graph.edges(), "no edges derived; the interface registry is not being read"


def test_every_component_reaches_the_page():
    """Set equality, not containment.

    A subset assertion passes while the renderer silently drops a node, and a
    substring test passes on a class name that merely appears in the prose two
    paragraphs further down. Both were available and both are wrong.
    """
    graph = _load_generator().build_graph()
    drawn = set(NODE.findall(generated_block()))
    assert drawn == set(graph.components), (
        f"nodes on the page and components in the code differ: "
        f"only in code {sorted(set(graph.components) - drawn)}, "
        f"only on page {sorted(drawn - set(graph.components))}"
    )


def test_the_walk_agrees_with_the_capability_walk():
    """An independent second derivation for one layer.

    tests/test_capability.py walks the equipment package for its own reasons.
    If the two disagree, one of them is wrong, and neither would notice alone.
    """
    from test_capability import _discover_equipment_components

    graph = _load_generator().build_graph()
    equipment = {name for name, layer in graph.components.items()
                 if layer == "equipment"}
    # A collapsed node stands for its members, so they count as reached. This
    # is why the graph records what it collapsed rather than letting the
    # classes just vanish from the component set.
    for port, members in graph.collapsed.items():
        if port in equipment:
            equipment |= members

    other = {name for name, _, _ in _discover_equipment_components()}
    assert other, "the capability walk found nothing -- this comparison is vacuous"
    assert other <= equipment, (
        f"the capability walk found components this one missed: "
        f"{sorted(other - equipment)}"
    )


def test_no_truth_reader_sits_above_the_boundary():
    """Invariant 1, asserted over the derived graph rather than the drawing.

    Checking the rendered string would test the renderer. The claim is about
    the topology: a component that takes a truth-carrying argument must be in
    a physical layer.
    """
    generator = _load_generator()
    graph = generator.build_graph()
    assert graph.truth_readers, "no truth readers found -- the check is vacuous"
    assert not generator.truth_violations(graph)


def test_a_planted_truth_leak_is_caught():
    """The sabotage for the test above.

    Without it, that test asserts only that currently clean code is clean --
    it would pass just as happily against a truth_violations() that returned
    an empty list unconditionally. ADR 0019's standard: every claim fails
    under a matching sabotage.
    """
    generator = _load_generator()
    graph = generator.build_graph()

    subsystem = next(
        name for name, layer in graph.components.items() if layer == "subsystem"
    )
    graph.truth_readers.add(subsystem)

    violations = generator.truth_violations(graph)
    assert violations, "a truth reader in the subsystem layer went unreported"
    assert any(subsystem in v for v in violations)


def test_the_diagram_is_internally_consistent():
    """Structural properties of the emitted mermaid, with no renderer needed.

    Nothing in this repository can render a GitHub page, so this checks what a
    parser would object to: dangling edge endpoints, a class applied without a
    classDef to apply, and unbalanced subgraph blocks.
    """
    block = generated_block()
    nodes = set(NODE.findall(block))
    subgraphs = set(re.findall(r"subgraph (\w+)\[", block))

    endpoints = set()
    for line in block.splitlines():
        edge = re.match(r"\s+(\w+) [-=]+>(?:\|[^|]*\|)? (\w+)\s*$", line)
        if edge:
            endpoints |= {edge.group(1), edge.group(2)}
    assert endpoints, "no edges parsed out of the diagram"
    assert endpoints <= nodes, (
        f"edge endpoints that are not declared nodes: {sorted(endpoints - nodes)}"
    )

    defined = set(re.findall(r"classDef (\w+) ", block))
    applied = {line.split()[-1]
               for line in block.splitlines() if line.strip().startswith("class ")}
    assert applied, "no classes applied; the diagram is unstyled"
    assert applied <= defined, (
        f"classes applied with no classDef: {sorted(applied - defined)}"
    )

    styled = set(re.findall(r"style (\w+) ", block))
    assert styled <= subgraphs | nodes, (
        f"style targets that exist neither as subgraph nor node: "
        f"{sorted(styled - subgraphs - nodes)}"
    )

    assert len(re.findall(r"^\s+subgraph ", block, re.M)) == \
        len(re.findall(r"^\s+end\s*$", block, re.M)), "unbalanced subgraph/end"

    # A classDef name colliding with a subgraph id renders, but leaves a bare
    # identifier whose meaning depends on which statement it is in.
    assert not (defined & subgraphs), (
        f"classDef names collide with subgraph ids: {sorted(defined & subgraphs)}"
    )


def test_every_edge_is_labelled_with_a_registered_interface():
    """A publication edge carries an interface name, and that name is one the
    catalogue actually knows -- not a record class name that leaked through.

    Checked against the PORT catalogue, not the record one. A record's
    INTERFACE is only the default: a component may publish the same record on
    its own port, which is how a source estimate is told apart from the
    platform's published state (ADR 0021). ose.interfaces cannot know those
    names without importing components, so the generator composes them.
    """
    known = set(_load_generator().build_graph().port_records)
    labels = set(re.findall(r"-->\|\"([^\"]+)\"\|", generated_block()))
    assert labels, "no labelled publication edges"
    assert labels <= known, f"edge labels not in the catalogue: {sorted(labels - known)}"
