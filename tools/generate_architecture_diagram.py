#!/usr/bin/env python3
"""
Regenerate the implemented-topology diagram in docs/20-architecture.md.

    python tools/generate_architecture_diagram.py            rewrite in place
    python tools/generate_architecture_diagram.py --check    exit non-zero if stale
    python tools/generate_architecture_diagram.py --dump     print the graph, write nothing

What this does and does not do
------------------------------
It derives the topology from the source: which components exist, which layer
each lives in, what each publishes, and what consumes it. Nothing here is
hand-listed, because a hand-listed diagram drifts the moment a component is
added and looks authoritative while doing it.

It shows only what is IMPLEMENTED. The layers with no components -- multi-ship
-- and the unbuilt simulation core are absent, deliberately. The ASCII drawing
above the generated block in that page is what shows the intended five-layer
shape; this shows what is actually wired.

The prose beneath the block is written, not generated, and states the blind
spots below. No version of this should try to produce it.

How an edge is derived
----------------------
A record is a port when it carries an INTERFACE class variable (ADR 0020),
which is the DEFAULT name; a component overrides it per method with PUBLISHES
where one record serves two ports (ADR 0021). A component PUBLISHES the
interface of any record a public method returns, and CONSUMES the interface of
any record a public method takes -- by annotation, or by `isinstance` dispatch
when the parameter is unannotated, which is how ingest() and command() accept
several record types on one port.

A constructor parameter typed by a PROTOCOL is a declared port: what the
provider publishes crosses it, drawn as a publication in the data direction. A
parameter typed by a concrete class is composition, drawn as a plain binding.
Claiming a publication there would invent one -- VehicleGuidance holds a
VehicleManager and calls capability_bound(), but never receives a MassEstimate.

Where a port has several providers they are alternatives, and the diagram draws
one node named for the port with their edges retargeted onto it (ADR 0023).

Three rules keep false edges out. Each exists for a component in the tree now,
and removing any one of them produces a wrong picture rather than a noisy one:

  T  truth reads are not port reads. A parameter is truth if its type is a
     truth type, OR its name starts with true_, OR the METHOD's name does.
     Without the second clause Imu.sample(true_command: VehicleCommand) draws
     guidance into an IMU; without the third, Imu.true_specific_force(state,
     command, dist) does the same with no marker on the parameters at all.
     The rule drops PARAMETERS, not whole methods: derivative(state:
     VehicleState, command: VehicleCommand) must keep `command`, or the one
     command edge in the system disappears.

  X  a method taking and returning the same interface is a transform, not a
     port pair. project_command(command: VehicleCommand) ->
     tuple[VehicleCommand, Saturation] would otherwise make the vehicle both
     publisher and consumer of the command it is being handed.

  N  nested carriage. A publisher of a record also publishes any registered
     record that record carries in a field. ActionSet.motion is how a setpoint
     reaches guidance, and without this the planner has a dangling publication
     and guidance a dangling consumption.

Capability records are declared NOT_A_PORT in ose.interfaces, so the twelve
capability() methods produce no edges. That exclusion is an architectural fact
stated where it belongs rather than a special case here.

What it cannot see
------------------
  * FuelGauge depends on the vehicle through `mass_dry_kg: float`. No
    signature carries it, and guessing from a scalar's name would invent
    edges. Counted in the generated output instead.
  * Clock reads truth through `dt_s`, deliberately not named `true_*` because
    for a clock the elapsed interval IS the corrupted quantity. So its truth
    read is underivable and it is not marked as a truth reader.
  * The `isinstance` recovery is fragile by nature. Rewriting a dispatch as a
    match statement or a dict lookup drops edges silently -- visible only as a
    staleness failure here plus a warning line.
  * Which of a port's providers a platform composes. The Vehicle node stands
    for both models; choosing between them is a composition decision, and the
    diagram describes the architecture.

tests/test_architecture_diagram.py runs this with --check, so adding a
component fails the suite until the diagram is regenerated.
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import importlib
import importlib.util
import inspect
import json
import pkgutil
import re
import sys
import textwrap
from enum import Enum
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# Importable from a bare checkout, without `pip install -e .`. CLAUDE.md tells
# a contributor to run this as `python tools/...`, which puts tools/ on the
# path and nothing else; pytest gets `ose` from pythonpath in pyproject.toml,
# but a subprocess does not.
sys.path.insert(0, str(ROOT / "src"))

from ose.interfaces import catalogue                      # noqa: E402
from ose.topology import (                                # noqa: E402
    LAYER_PACKAGES,
    PHYSICAL_LAYERS,
    TRUTH_TYPES,
)

PAGE = ROOT / "docs" / "20-architecture.md"
CATALOGUE_PAGE = ROOT / "docs" / "interfaces" / "README.md"
PALETTE = ROOT / "prefs" / "palette.json"

# Layer packages hold reference configurations alongside components. They are
# data, not components, and walking them would add a node per fixture.
SKIP_MODULE_PARTS = ("reference_configs",)

# A protocol resolving to more than this many components is too loose to mean
# a binding -- CapabilityModel matches almost everything. Warn rather than
# fan out.
MAX_PROTOCOL_FANOUT = 3


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def discover_components() -> dict[str, tuple[str, type]]:
    """Component class name -> (layer, class).

    A component is a class defined in a layer package that is not a record, not
    a parameter block and not an enumeration. This is the predicate
    tests/test_capability.py already applies to the equipment layer, widened to
    every layer package.

    A layer package that does not exist yet is skipped rather than treated as
    empty -- multi_ship is listed in ose.topology precisely so its arrival
    needs no change here.
    """
    found: dict[str, tuple[str, type]] = {}
    for layer, package_name in LAYER_PACKAGES.items():
        if importlib.util.find_spec(package_name) is None:
            continue
        package = importlib.import_module(package_name)
        for info in pkgutil.walk_packages(package.__path__, prefix=f"{package_name}."):
            if any(part in info.name for part in SKIP_MODULE_PARTS):
                continue
            module = importlib.import_module(info.name)
            for attr, obj in vars(module).items():
                if (
                    inspect.isclass(obj)
                    and obj.__module__ == module.__name__
                    and not dataclasses.is_dataclass(obj)
                    and not attr.endswith("Parameters")
                    and not (isinstance(obj, type) and issubclass(obj, Enum))
                ):
                    found[attr] = (layer, obj)
    return found


def record_interfaces() -> dict[str, str]:
    """Record class name -> interface name, from the catalogue."""
    return {
        record.__name__: name
        for name, records in catalogue().items()
        for record in records
    }


# ---------------------------------------------------------------------------
# Annotation reading
#
# ast for structure, the live module for identity. Not typing.get_type_hints:
# every module here uses `from __future__ import annotations`, so annotations
# are strings at runtime anyway, the isinstance dispatch needs ast regardless,
# and ActionSet.motion is a quoted string that needs re-parsing either way.
#
# Resolving each leaf against the module it was written in -- rather than
# string-matching a name -- is what stops a renamed record from becoming a
# literal that silently matches nothing.
# ---------------------------------------------------------------------------

def annotation_leaves(node: ast.AST | None) -> list[str]:
    """Every bare type name in an annotation, however nested."""
    if node is None:
        return []
    if isinstance(node, ast.Name):
        return [node.id]
    if isinstance(node, ast.Attribute):
        return [node.attr]
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.BitOr):
        return annotation_leaves(node.left) + annotation_leaves(node.right)
    if isinstance(node, ast.Subscript):
        return annotation_leaves(node.value) + annotation_leaves(node.slice)
    if isinstance(node, ast.Tuple):
        return [leaf for e in node.elts for leaf in annotation_leaves(e)]
    if isinstance(node, ast.Constant):
        if isinstance(node.value, str):
            try:
                return annotation_leaves(ast.parse(node.value, mode="eval").body)
            except SyntaxError:
                return []
        return []          # None in `X | None`
    return []


def resolve(name: str, module_name: str) -> type | None:
    """A leaf name, resolved against the module that wrote it."""
    module = sys.modules.get(module_name)
    obj = getattr(module, name, None) if module else None
    return obj if isinstance(obj, type) else None


def class_tree(cls: type) -> ast.ClassDef | None:
    try:
        source = inspect.getsource(cls)
    except (OSError, TypeError):
        return None
    # Dedented because a class defined inside a function -- which the tests do
    # -- comes back indented and will not parse on its own.
    tree = ast.parse(textwrap.dedent(source))
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            return node
    return None


# ---------------------------------------------------------------------------
# Extraction
# ---------------------------------------------------------------------------

@dataclasses.dataclass
class Graph:
    """The derived topology. Rendered afterwards; never during."""

    components: dict[str, str] = dataclasses.field(default_factory=dict)
    publishes: set[tuple[str, str]] = dataclasses.field(default_factory=set)
    consumes: set[tuple[str, str]] = dataclasses.field(default_factory=set)
    bindings: set[tuple[str, str, str]] = dataclasses.field(default_factory=set)
    truth_readers: set[str] = dataclasses.field(default_factory=set)
    # Interface name -> the record class names carried on it. Built while
    # walking, because that is the only place a PUBLISHES override and the
    # record it returns are both in hand. ose.interfaces knows the record
    # defaults; it cannot know the overrides without importing components.
    port_records: dict[str, set[str]] = dataclasses.field(default_factory=dict)
    # Port name -> the component classes it stands for. A collapsed node is
    # the only place a real class stops being a node, so it is recorded
    # rather than left for a reader to infer from an absence.
    collapsed: dict[str, set[str]] = dataclasses.field(default_factory=dict)
    warnings: list[str] = dataclasses.field(default_factory=list)

    def edges(self) -> list[tuple[str, str, str]]:
        """(publisher, consumer, interface), ordered for a stable file."""
        out = [
            (publisher, consumer, interface)
            for publisher, interface in sorted(self.publishes)
            for consumer, consumed in sorted(self.consumes)
            if consumed == interface and consumer != publisher
        ]
        return sorted(set(out + self.port_edges()))

    def port_edges(self) -> list[tuple[str, str, str]]:
        """Edges across a protocol-typed binding, in the data direction.

        A constructor parameter annotated with a PROTOCOL is a declared port,
        so what the provider publishes reaches the binder across it. A
        parameter annotated with a concrete class is not a port -- it is a
        component reaching into one implementation -- and stays a plain
        composition edge.

        That distinction is what keeps false edges out. VehicleGuidance binds
        VehicleManager by its concrete type, so nothing here claims guidance
        consumes vehicle.mass.v1: it calls capability_bound(), and never takes
        a MassEstimate.
        """
        out = []
        for consumer, provider, protocol in self.bindings:
            if not protocol:
                continue
            for publisher, interface in self.publishes:
                if publisher == provider:
                    out.append((provider, consumer, interface))
        return sorted(set(out))

    def composition_bindings(self) -> list[tuple[str, str, str]]:
        """Bindings that are not already drawn as a port edge.

        A protocol binding whose provider publishes something is shown by that
        publication instead, in the data direction; drawing both would put two
        edges between the same pair saying the same thing. A provider that
        publishes nothing -- Vehicle -- keeps its composition edge, which is
        the only thing that would otherwise connect it.
        """
        drawn = {(consumer, provider) for provider, consumer, _ in self.port_edges()}
        return sorted(
            (consumer, provider, protocol)
            for consumer, provider, protocol in self.bindings
            if (consumer, provider) not in drawn
        )

    def consumed_interfaces(self) -> set[str]:
        return {i for _, i in self.consumes} | {i for _, _, i in self.port_edges()}

    def unconsumed(self) -> list[tuple[str, str]]:
        wanted = self.consumed_interfaces()
        return sorted(
            (publisher, interface)
            for publisher, interface in self.publishes
            if interface not in wanted
        )


def collapse_alternatives(graph: Graph) -> None:
    """Components providing one port are alternatives, and draw as one node.

    A platform composes one vehicle model, not both. When a protocol-typed
    binding resolves to several providers they are alternatives rather than
    collaborators, so the architecture has one component there and the choice
    between implementations is a configuration decision.

    Drawn as one node named for the port, with every edge of the collapsed
    members retargeted onto it. Nothing is hidden: the members are alternatives
    for the same role, so an edge to one of them is an edge to whichever is
    composed.

    A port with a single provider does not collapse -- OwnStateSource resolves
    to InsGnssEstimator alone, and naming that node after the port would hide
    which implementation is in the tree.
    """
    providers: dict[str, set[str]] = {}
    for _, target, protocol in graph.bindings:
        if protocol:
            providers.setdefault(protocol, set()).add(target)

    for protocol, members in sorted(providers.items()):
        if len(members) < 2:
            continue
        layers = {graph.components[m] for m in members if m in graph.components}
        if len(layers) != 1:
            graph.warnings.append(
                f"{protocol} is provided from more than one layer ({sorted(layers)}); "
                "not collapsed"
            )
            continue

        for member in members:
            graph.components.pop(member, None)
        graph.components[protocol] = layers.pop()
        graph.collapsed[protocol] = set(members)

        def rename(name: str) -> str:
            return protocol if name in members else name

        graph.publishes = {(rename(c), i) for c, i in graph.publishes}
        graph.consumes = {(rename(c), i) for c, i in graph.consumes}
        graph.truth_readers = {rename(n) for n in graph.truth_readers}
        graph.bindings = {
            (rename(c), rename(t), p)
            for c, t, p in graph.bindings
            if rename(c) != rename(t)
        }


def nested_records(record: type, interfaces: dict[str, str]) -> set[str]:
    """Interfaces carried inside a record's own fields -- rule N."""
    out: set[str] = set()
    if not dataclasses.is_dataclass(record):
        return out
    for field in dataclasses.fields(record):
        annotation = field.type if isinstance(field.type, str) else ""
        try:
            leaves = annotation_leaves(ast.parse(annotation, mode="eval").body)
        except SyntaxError:
            continue
        for leaf in leaves:
            if leaf in interfaces and leaf != record.__name__:
                out.add(interfaces[leaf])
    return out


def extract(component: type, layer: str, interfaces: dict[str, str], graph: Graph):
    """Walk one component's public methods into the graph."""
    tree = class_tree(component)
    if tree is None:
        graph.warnings.append(f"{component.__name__}: source unavailable")
        return
    module_name = component.__module__

    def interface_of(leaf: str) -> str | None:
        resolved = resolve(leaf, module_name)
        return interfaces.get(resolved.__name__) if resolved else None

    for node in tree.body:
        if not isinstance(node, ast.FunctionDef) or node.name.startswith("_"):
            continue

        # A component may name the port a method publishes on, overriding the
        # record's default. Needed wherever one record serves two ports: an
        # InsGnssEstimator and a NavigationManager both return an
        # OwnStateEstimate, but a source estimate and the platform's published
        # state are different ports. See ADR 0021.
        override = getattr(component, "PUBLISHES", {}).get(node.name)

        published = set()
        for leaf in annotation_leaves(node.returns):
            interface = override or interface_of(leaf)
            if interface:
                published.add(interface)
                resolved = resolve(leaf, module_name)
                if resolved:
                    # An override names a port the record catalogue does not
                    # know about, so the record has to be recorded here. The
                    # default names are seeded from the catalogue instead,
                    # which already handles one interface carrying several
                    # records -- guidance.setpoint.v1 has two forms.
                    if override:
                        graph.port_records.setdefault(
                            interface, set()
                        ).add(resolved.__name__)
                    published |= nested_records(resolved, interfaces)

        consumed = set()
        # Rule T, third clause: a method whose own name says it deals in truth.
        method_is_truth = node.name.startswith("true_")
        params = node.args.args[1:] + node.args.kwonlyargs
        for param in params:
            leaves = annotation_leaves(param.annotation)
            # Rule T, first and second clauses.
            if param.arg.startswith("true_") or method_is_truth:
                if any(leaf in TRUTH_TYPES for leaf in leaves) or not leaves:
                    graph.truth_readers.add(component.__name__)
                continue
            if any(leaf in TRUTH_TYPES for leaf in leaves):
                graph.truth_readers.add(component.__name__)
                continue
            for leaf in leaves:
                interface = interface_of(leaf)
                if interface:
                    consumed.add(interface)
            if not leaves:
                consumed |= dispatched_interfaces(node, param.arg, interface_of, graph,
                                                  component.__name__)

        # Rule X: same interface in and out is a transform, not a port pair.
        for interface in published & consumed:
            published.discard(interface)
            consumed.discard(interface)

        graph.publishes |= {(component.__name__, i) for i in published}
        graph.consumes |= {(component.__name__, i) for i in consumed}


def dispatched_interfaces(node: ast.FunctionDef, param: str, interface_of,
                          graph: Graph, owner: str) -> set[str]:
    """Interfaces an unannotated parameter is isinstance-dispatched on.

    Restricted to `isinstance(<that bare parameter>, ...)`. The restriction is
    what makes this safe: NavigationManager.consumes_measurements calls
    isinstance(self.source, ...), whose first argument is an attribute, and is
    correctly ignored.
    """
    found = set()
    for call in ast.walk(node):
        if (
            isinstance(call, ast.Call)
            and isinstance(call.func, ast.Name)
            and call.func.id == "isinstance"
            and len(call.args) == 2
            and isinstance(call.args[0], ast.Name)
            and call.args[0].id == param
        ):
            for leaf in annotation_leaves(call.args[1]):
                interface = interface_of(leaf)
                if interface:
                    found.add(interface)
    if not found:
        # One warning per method, not per parameter: state_from(estimate,
        # beliefs) has two unannotated arguments and one derivation gap.
        warning = (
            f"{owner}.{node.name} has unannotated parameters and no isinstance "
            "dispatch -- no consumption edge derived"
        )
        if warning not in graph.warnings:
            graph.warnings.append(warning)
    return found


def extract_bindings(component: type, interfaces: dict[str, str], graph: Graph):
    """Constructor parameters that are other components, or protocols they
    satisfy."""
    tree = class_tree(component)
    if tree is None:
        return
    init = next(
        (n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "__init__"),
        None,
    )
    if init is None:
        return

    for param in init.args.args[1:] + init.args.kwonlyargs:
        for leaf in annotation_leaves(param.annotation):
            # Resolve the name against the module that wrote it BEFORE asking
            # whether it looks like a component. Three protocol names collide
            # with implementation class names -- AirDataSensor, TimeEstimator
            # and VehicleGuidance -- so a bare name check answers the wrong
            # question. navigation_manager.py imports TimeEstimator from
            # ose.interfaces, meaning the protocol; matching the component of
            # that name instead turned a declared port into a concrete
            # composition and silently dropped the edge across it.
            resolved = resolve(leaf, component.__module__)
            if resolved is not None and _is_protocol(resolved):
                pass
            elif leaf in graph.components:
                graph.bindings.add((component.__name__, leaf, ""))
                continue
            else:
                continue
            matches = sorted(
                name for name, cls in _component_classes.items()
                if name != component.__name__ and satisfies(cls, resolved, interfaces)
            )
            if not matches:
                continue
            if len(matches) > MAX_PROTOCOL_FANOUT:
                graph.warnings.append(
                    f"{component.__name__}.__init__({param.arg}: {leaf}) matches "
                    f"{len(matches)} components -- too loose to draw as a binding"
                )
                continue
            for match in matches:
                graph.bindings.add((component.__name__, match, leaf))


def protocol_members(protocol: type) -> set[str]:
    """Every name a protocol requires -- methods, properties and annotations."""
    tree = class_tree(protocol)
    if tree is None:
        return set()
    names = set()
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            names.add(node.name)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if not node.target.id.startswith("_"):
                names.add(node.target.id)
    return names


def protocol_returns(protocol: type, interfaces: dict[str, str]) -> dict[str, str]:
    """Method name -> the registered record it returns, for the methods that
    return one."""
    tree = class_tree(protocol)
    if tree is None:
        return {}
    out = {}
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            for leaf in annotation_leaves(node.returns):
                resolved = resolve(leaf, protocol.__module__)
                if resolved and resolved.__name__ in interfaces:
                    out[node.name] = resolved.__name__
    return out


def satisfies(cls: type, protocol: type, interfaces: dict[str, str]) -> bool:
    """Whether a component provides a protocol's port.

    Structural, in two parts, and `issubclass` does neither of them properly.

    It compares METHOD NAMES ONLY, which is how the first version of this tool
    bound the navigation manager to the clock estimator: TimeEstimator.estimate
    returns a TimeEstimate and satisfies OwnStateSource, whose estimate returns
    an OwnStateEstimate. So the record a method returns is compared too.

    And `issubclass` raises outright on a protocol with a non-method member --
    `Protocols with non-method members don't support issubclass()`. The Vehicle
    protocol has to declare `dry_mass_kg`, which is a property on the models,
    because VehicleManager reads it. A protocol that cannot state what its
    consumers use would be the wrong shape of protocol.

    Comparison is by RECORD, not by interface name, so it survives a port being
    renamed -- which is exactly what happens when a source publishes under its
    own interface rather than the platform's.
    """
    members = protocol_members(protocol)
    if not members or not all(hasattr(cls, name) for name in members):
        return False

    tree = class_tree(cls)
    if tree is None:
        return False
    returns = {
        node.name: annotation_leaves(node.returns)
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
    }
    for method, record in protocol_returns(protocol, interfaces).items():
        if record not in returns.get(method, []):
            return False
    return True


def _is_protocol(cls: type) -> bool:
    return getattr(cls, "_is_protocol", False)


_component_classes: dict[str, type] = {}


def build_graph() -> Graph:
    discovered = discover_components()
    graph = Graph(components={name: layer for name, (layer, _) in discovered.items()})
    interfaces = record_interfaces()
    graph.port_records = {
        name: {r.__name__ for r in records} for name, records in catalogue().items()
    }

    _component_classes.clear()
    _component_classes.update({name: cls for name, (_, cls) in discovered.items()})

    for name, cls in sorted(_component_classes.items()):
        extract(cls, graph.components[name], interfaces, graph)
    # Bindings run in a second pass because a protocol binding is only drawn
    # when the candidate publishes what the protocol publishes, and that is not
    # known until every component's publications have been extracted.
    for name, cls in sorted(_component_classes.items()):
        extract_bindings(cls, interfaces, graph)
    collapse_alternatives(graph)
    return graph


def truth_violations(graph: Graph) -> list[str]:
    """Truth readers outside a physical layer. Invariant 1, checked here as
    well as by the import guards in tests/, because a diagram that draws the
    boundary must be able to tell when it has been crossed."""
    return sorted(
        f"{name} reads ground truth but sits in the {graph.components[name]} layer"
        for name in graph.truth_readers
        if graph.components.get(name) not in PHYSICAL_LAYERS
    )


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------

def palette() -> dict[str, dict[str, str]]:
    if not PALETTE.exists():
        raise SystemExit(
            f"no palette at {PALETTE}. It is the shared source of colour for "
            "diagrams and plots; see prefs/README.md."
        )
    return json.loads(PALETTE.read_text())["roles"]


def role(roles: dict[str, dict[str, str]], name: str) -> dict[str, str]:
    if name not in roles:
        raise SystemExit(
            f"prefs/palette.json has no role {name!r}; add one rather than "
            "letting the diagram fall back to a default colour."
        )
    return roles[name]


def subgraph_id(layer: str) -> str:
    return f"layer_{layer}"


LAYER_TITLES = {
    "equipment": "equipment layer -- physical, reads ground truth",
    "subsystem": "subsystem layer",
    "single_ship": "single-ship layer",
    "multi_ship": "multi-ship layer",
}


def mermaid(graph: Graph) -> str:
    roles = palette()
    lines = [
        "```mermaid",
        "%% Generated by tools/generate_architecture_diagram.py -- "
        "do not edit by hand.",
        "flowchart LR",
    ]

    for layer in LAYER_PACKAGES:
        members = sorted(n for n, l in graph.components.items() if l == layer)
        if not members:
            continue
        # The subgraph id is prefixed so it cannot collide with the classDef
        # of the same layer. `style equipment ...` and `classDef equipment ...`
        # sit in different namespaces and mermaid accepts both, but a reader
        # cannot tell which one a bare `equipment` refers to.
        lines.append(f'  subgraph {subgraph_id(layer)}["{LAYER_TITLES[layer]}"]')
        lines += [f'    {name}["{name}"]' for name in members]
        lines.append("  end")

    lines.append("")
    for publisher, consumer, interface in graph.edges():
        lines.append(f'  {publisher} -->|"{interface}"| {consumer}')

    lines.append("")
    for consumer, target, protocol in sorted(graph.bindings):
        # `A ==> B`, or `A ==>|"label"| B`. The label goes AFTER the arrow, not
        # inside it -- `===|"x"|>` is not an arrow at all.
        # A label repeating the node it points at says nothing: after a
        # collapse the target IS the port.
        label = f'|"{protocol}"|' if protocol and protocol != target else ""
        lines.append(f"  {consumer} ==>{label} {target}")

    lines.append("")
    for layer in LAYER_PACKAGES:
        if not any(l == layer for l in graph.components.values()):
            continue
        colour = role(roles, layer)
        lines.append(
            f"  classDef {layer} fill:{colour['fill']},stroke:{colour['stroke']},"
            f"color:{colour['text']},stroke-width:1.5px"
        )
    truth = role(roles, "truth")
    lines.append(
        f"  classDef readsTruth stroke:{truth['stroke']},stroke-width:2.5px,"
        "stroke-dasharray:5 3"
    )

    for layer in LAYER_PACKAGES:
        members = sorted(n for n, l in graph.components.items() if l == layer)
        if members:
            lines.append(f"  class {','.join(members)} {layer}")
    readers = sorted(graph.truth_readers)
    if readers:
        lines.append(f"  class {','.join(readers)} readsTruth")

    neutral = role(roles, "neutral")
    for layer in LAYER_PACKAGES:
        if not any(l == layer for l in graph.components.values()):
            continue
        stroke = truth["stroke"] if layer in PHYSICAL_LAYERS else neutral["stroke"]
        dashes = ",stroke-dasharray:5 4" if layer in PHYSICAL_LAYERS else ""
        lines.append(
            f"  style {subgraph_id(layer)} fill:none,stroke:{stroke}{dashes}"
        )

    lines.append("```")

    unconsumed = graph.unconsumed()
    if unconsumed:
        published = ", ".join(
            f"`{interface}` ({publisher})" for publisher, interface in unconsumed
        )
        lines += ["", f"Published with no consumer: {published}."]
    return "\n".join(lines)


def interface_table(graph: Graph) -> str:
    """The implemented half of docs/interfaces/README.md.

    Publisher, consumer and records all come from the same derivation the
    diagram uses, so the table and the picture cannot disagree. An interface
    with no consumer prints a dash: nothing asks for it yet, which is a fact
    about the system rather than a gap in the table.
    """
    rows = [
        "| Interface | Record(s) | Published by | Consumed by |",
        "|---|---|---|---|",
    ]
    consumers_of: dict[str, set[str]] = {}
    for consumer, interface in graph.consumes:
        consumers_of.setdefault(interface, set()).add(consumer)
    for _, consumer, interface in graph.port_edges():
        consumers_of.setdefault(interface, set()).add(consumer)

    for name in sorted(graph.port_records):
        publishers = sorted(c for c, i in graph.publishes if i == name)
        consumers = sorted(consumers_of.get(name, ()))
        if not publishers and not consumers:
            continue
        records = ", ".join(f"`{r}`" for r in sorted(graph.port_records[name]))
        rows.append(
            f"| `{name}` | {records} "
            f"| {', '.join(f'`{p}`' for p in publishers) or '--'} "
            f"| {', '.join(f'`{c}`' for c in consumers) or '--'} |"
        )
    return "\n".join(rows)


def render(path: Path, blocks) -> str:
    """Splice generated bodies into their marker pairs, leaving prose alone.

    The same mechanism as tools/generate_model_docs.py. Duplicated rather than
    imported: tools/ is not a package, and the two generators are otherwise
    independent.
    """
    text = path.read_text()
    for name, build in blocks.items():
        pattern = re.compile(
            rf"(<!-- generated: {re.escape(name)} -->\n).*?(\n<!-- end generated: "
            rf"{re.escape(name)} -->)",
            re.S,
        )
        if not pattern.search(text):
            raise SystemExit(f"{path.name}: no marker pair for {name!r}")
        text = pattern.sub(lambda m: m.group(1) + build() + m.group(2), text)
    return text


def dump(graph: Graph) -> None:
    print("Components")
    for layer in LAYER_PACKAGES:
        members = sorted(n for n, l in graph.components.items() if l == layer)
        if members:
            print(f"  {layer:12s} {', '.join(members)}")
    print(f"\nPublications ({len(graph.publishes)})")
    for publisher, interface in sorted(graph.publishes):
        print(f"  {publisher:28s} -> {interface}")
    print(f"\nConsumptions ({len(graph.consumes)})")
    for consumer, interface in sorted(graph.consumes):
        print(f"  {consumer:28s} <- {interface}")
    print(f"\nEdges ({len(graph.edges())})")
    for publisher, consumer, interface in graph.edges():
        print(f"  {publisher:28s} -> {consumer:24s} {interface}")
    print(f"\nBindings ({len(graph.composition_bindings())} drawn, {len(graph.bindings)} total)")
    for consumer, target, protocol in graph.composition_bindings():
        print(f"  {consumer:28s} => {target:24s} {protocol}")
    print(f"\nTruth readers: {', '.join(sorted(graph.truth_readers)) or 'none'}")
    unconsumed = graph.unconsumed()
    print(f"Published with no consumer: "
          f"{', '.join(f'{i} ({p})' for p, i in unconsumed) or 'none'}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("--check", action="store_true",
                    help="report staleness instead of rewriting")
    ap.add_argument("--dump", action="store_true",
                    help="print the derived graph and write nothing")
    args = ap.parse_args()

    graph = build_graph()

    if args.dump:
        dump(graph)
        for warning in graph.warnings:
            print(f"  WARNING {warning}")
        return 0

    violations = truth_violations(graph)
    if violations:
        for violation in violations:
            print(f"  TRUTH BOUNDARY {violation}")
        print("\nOnly equipment-layer components may read ground truth "
              "(invariant 1, ADR 0008).")
        return 1

    blocks = {
        PAGE: {"topology": lambda: mermaid(graph)},
        CATALOGUE_PAGE: {"implemented-interfaces": lambda: interface_table(graph)},
    }

    stale = []
    for path, marked in blocks.items():
        rendered = render(path, marked)
        if rendered == path.read_text():
            continue
        if args.check:
            stale.append(path.name)
        else:
            path.write_text(rendered)
            print(f"  rewrote {path.name}")

    if args.check:
        if stale:
            print("Stale generated blocks in: " + ", ".join(stale))
            print("\nRun: python tools/generate_architecture_diagram.py")
            return 1
        print("  generated blocks up to date")
        return 0

    if not stale:
        print("  generated blocks up to date")
    for warning in graph.warnings:
        print(f"  WARNING {warning}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
