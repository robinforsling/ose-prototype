"""Nothing binds upward, and nothing reaches past a layer.

Composition is bottom-up: a component may bind the layer directly below it and
peers in its own layer. A single-ship component binding equipment would skip
the subsystem that exists to integrate it, which is the coupling the layering
is there to prevent. See docs/10-concepts.md.

That rule was stated in three documents and checked in two places -- one call
to `assert_no_equipment_imports` in test_time_estimator.py and one in
test_action_planner.py, each naming its own component by hand. Four of the six
cyber components were covered by nothing at all, and a seventh would have been
covered by nothing unless whoever added it remembered.

So this walks every component module instead of listing any. The failure it
exists for is not a component that breaks the rule; it is a component nobody
thought to check.

Imports are only half of it. A component bound through a protocol in
ose.interfaces has no import naming the component it binds, so an upward
binding could arrive with nothing here to see it. The architecture generator
checks the derived bindings for the same rule and fails the same way --
tools/generate_architecture_diagram.py, `layer_violations`.
"""

from __future__ import annotations

import ast
import importlib
import importlib.util
import pkgutil
from pathlib import Path

from ose.topology import LAYER_PACKAGES, binding_is_allowed

SRC = Path(__file__).resolve().parents[2] / "src"

# Reference configurations are data, not components. They may name whatever
# they configure.
SKIP_PARTS = ("reference_configs",)


def component_modules() -> list[tuple[str, str, Path]]:
    """(layer, module name, path) for every component module, discovered."""
    out = []
    for layer, package_name in LAYER_PACKAGES.items():
        if importlib.util.find_spec(package_name) is None:
            continue
        package = importlib.import_module(package_name)
        for info in pkgutil.walk_packages(package.__path__, prefix=f"{package_name}."):
            if any(part in info.name for part in SKIP_PARTS):
                continue
            path = SRC.joinpath(*info.name.split(".")).with_suffix(".py")
            if not path.is_file():                     # a package, not a module
                path = SRC.joinpath(*info.name.split(".")) / "__init__.py"
            if path.is_file():
                out.append((layer, info.name, path))
    return out


def imported_layers(path: Path) -> set[str]:
    """The layer packages a module imports from."""
    found = set()
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        modules = []
        if isinstance(node, ast.ImportFrom) and node.module:
            modules.append(node.module)
        elif isinstance(node, ast.Import):
            modules += [a.name for a in node.names]
        for module in modules:
            for layer, package in LAYER_PACKAGES.items():
                if module == package or module.startswith(package + "."):
                    found.add(layer)
    return found


def test_the_walk_is_not_vacuous():
    """Every test below iterates the walk, so all of them pass on an empty
    one. This is the failure the module docstring is about, applied to the
    module itself."""
    modules = component_modules()
    assert len(modules) >= 8, f"only {len(modules)} component modules found"
    assert {layer for layer, _, _ in modules} >= {"equipment", "subsystem"}


def test_no_component_imports_from_a_layer_it_may_not_bind():
    """The rule, applied to every component module rather than to two.

    An import is not proof of a binding -- a module could import a type it
    never holds -- but the layering forbids naming it at all, which is the
    stricter and simpler thing to check.
    """
    offences = []
    for layer, name, path in component_modules():
        for imported in imported_layers(path):
            if not binding_is_allowed(layer, imported):
                offences.append(f"{name} ({layer}) imports from {imported}")
    assert not offences, "layer discipline broken:\n  " + "\n  ".join(sorted(offences))


def test_a_planted_upward_import_is_caught(tmp_path):
    """The sabotage.

    Without it the test above asserts only that currently clean code is clean,
    and would pass just as happily against an `imported_layers` that always
    returned nothing -- which is precisely how the hand-placed guards this
    replaces could have failed.
    """
    module = tmp_path / "planted.py"
    module.write_text(
        "from ose.single_ship.action_planner import WaypointPlanner\n"
        "from ose.equipment.vehicle import VehicleState\n"
    )
    layers = imported_layers(module)
    assert layers == {"single_ship", "equipment"}

    # A subsystem component may bind equipment, but never single-ship.
    assert binding_is_allowed("subsystem", "equipment")
    assert not binding_is_allowed("subsystem", "single_ship")
    # And a single-ship component may not reach past the subsystem layer.
    assert not binding_is_allowed("single_ship", "equipment")
    assert binding_is_allowed("single_ship", "subsystem")
    assert binding_is_allowed("single_ship", "single_ship")


def test_the_rule_matches_the_documented_one():
    """`binding_is_allowed` is the executable copy of a sentence in
    docs/10-concepts.md: the layer below, and peers. Pinned so that widening it
    to 'any lower layer' has to be a deliberate edit here rather than a quiet
    one there."""
    order = list(LAYER_PACKAGES)
    for i, consumer in enumerate(order):
        for j, provider in enumerate(order):
            expected = (i - j) in (0, 1)
            assert binding_is_allowed(consumer, provider) is expected, (
                f"{consumer} binding {provider} should be "
                f"{'allowed' if expected else 'refused'}"
            )


def test_the_generator_agrees_over_derived_bindings():
    """The other half: a binding through a protocol has no import to catch.

    An independent second check of the same rule, over the graph the
    architecture generator derives rather than over source text.
    """
    import importlib.util as iu
    import sys

    generator_path = SRC.parent / "tools" / "generate_architecture_diagram.py"
    spec = iu.spec_from_file_location("generate_architecture_diagram", generator_path)
    generator = iu.module_from_spec(spec)
    sys.modules[spec.name] = generator
    spec.loader.exec_module(generator)

    graph = generator.build_graph()
    assert graph.bindings, "no bindings derived -- this check is vacuous"
    assert not generator.layer_violations(graph)


def test_the_generator_catches_a_planted_upward_binding():
    """Sabotage for the check above, over a graph rather than over source."""
    import importlib.util as iu
    import sys

    generator_path = SRC.parent / "tools" / "generate_architecture_diagram.py"
    spec = iu.spec_from_file_location("generate_architecture_diagram", generator_path)
    generator = iu.module_from_spec(spec)
    sys.modules[spec.name] = generator
    spec.loader.exec_module(generator)

    graph = generator.build_graph()
    equipment = next(n for n, l in graph.components.items() if l == "equipment")
    upper = next(n for n, l in graph.components.items() if l == "single_ship")

    graph.bindings.add((equipment, upper, ""))
    violations = generator.layer_violations(graph)
    assert violations, "an equipment component binding single-ship went unreported"
    assert any("upward" in v for v in violations)
