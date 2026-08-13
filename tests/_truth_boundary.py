"""Shared checks for invariant 1: only equipment-layer components read truth.

Not a test module. It holds the guard that six test modules apply to their own
component, so that the layer is named in one place instead of seven.

Why this exists
---------------
Each of those tests parses its component with `ast` and asserts that no import
brings in a truth-carrying type. The comparison is against a module path
written as a string, and a string that no longer names anything matches
nothing -- so the loop finds no leak, and the test passes while checking
nothing at all.

That is not hypothetical. Renaming the resource layer to the equipment layer
(ADR 0017) moved `ose.resource.vehicle` to `ose.equipment.vehicle`, and had
those seven literals not been updated, every truth-boundary test in the
repository would have gone green over a component importing `VehicleState`
outright. It was checked by planting exactly that leak, and the test passed.

Two defences, because naming it once is not enough on its own:

  1. The layer is named once, so a future rename has one place to touch. That
     place is now `ose.topology`, not this module -- a tool outside tests/
     needs the same vocabulary, and a second copy of a literal whose failure
     mode is "matches nothing" is the last thing this guard should acquire.
     The names are re-exported below so the six test modules importing them
     from here do not care where they live.
  2. `_assert_guard_is_live()` imports the module the guard names before using
     it. A stale name now raises ImportError instead of quietly matching
     nothing. Centralising alone would not have caught the failure; this does.
     It now covers the tool as well, since the tool reads the same names.
"""

from __future__ import annotations

import ast
import importlib
from pathlib import Path

from ose.topology import (  # noqa: F401  -- re-exported for the six guards
    EQUIPMENT_PACKAGE,
    TRUTH_PACKAGE,
    TRUTH_TYPES,
)

SRC = Path(__file__).resolve().parents[1] / "src"


def _assert_guard_is_live() -> None:
    """Fail loudly if this guard has gone stale.

    The whole failure mode above is a string that names nothing. Importing it
    turns that into an error at the point of use.
    """
    module = importlib.import_module(TRUTH_PACKAGE)
    missing = TRUTH_TYPES - set(vars(module))
    assert not missing, (
        f"{TRUTH_PACKAGE} no longer exposes {sorted(missing)}; this guard is "
        "looking for names that have moved and would match nothing"
    )


def is_truth_package(module: str | None) -> bool:
    """The package or anything under it."""
    return module is not None and (
        module == TRUTH_PACKAGE or module.startswith(TRUTH_PACKAGE + ".")
    )


def component_path(*parts: str) -> Path:
    """Locate a component under src/ose/, e.g. component_path("subsystem",
    "vehicle_guidance.py")."""
    path = SRC.joinpath("ose", *parts)
    assert path.is_file(), f"no such component: {path}"
    return path


def _tree(path: Path) -> ast.AST:
    return ast.parse(path.read_text())


def assert_no_truth_types(path: Path) -> None:
    """For a component that may bind the equipment layer but must not read
    truth -- a subsystem component holding a model reference, for instance."""
    _assert_guard_is_live()
    for node in ast.walk(_tree(path)):
        if isinstance(node, ast.ImportFrom) and is_truth_package(node.module):
            leaked = {a.name for a in node.names} & TRUTH_TYPES
            assert not leaked, (
                f"{path.name} imports truth-carrying types from {node.module}: "
                f"{leaked}"
            )


def assert_no_equipment_imports(path: Path) -> None:
    """For a component two or more layers above the equipment layer, which has
    no business referencing it at all."""
    _assert_guard_is_live()
    for node in ast.walk(_tree(path)):
        if isinstance(node, ast.ImportFrom) and node.module is not None:
            assert not node.module.startswith(EQUIPMENT_PACKAGE), (
                f"{path.name} imports from the equipment layer: {node.module}"
            )


def assert_no_truth_parameters(path: Path) -> None:
    """No public method may take a parameter named true_something. A signature
    carrying truth is wrong whatever the body does with it."""
    for node in ast.walk(_tree(path)):
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            params = [a.arg for a in node.args.args + node.args.kwonlyargs]
            leaked = [p for p in params if p.startswith("true_")]
            assert not leaked, f"public method {node.name} takes truth: {leaked}"


def vehicle_model_names() -> frozenset[str]:
    """Every vehicle model class, discovered rather than listed.

    A model is a class defined in a module under ose.equipment.vehicle that is
    not a dataclass -- the dataclasses there are records (state, command,
    parameters, capability), the plain classes are the models themselves.

    Discovered because the alternative is a hand-written list, and the rule
    this feeds is "no component above the equipment layer binds a vehicle
    model". Naming one model would let the second one through silently, which
    is the same failure as a test that checks the channels a consumer happens
    to read. test_every_equipment_module_defines_a_component_with_capability
    was rewritten for exactly this reason after a hand-written list omitted
    the integrated navigation unit (since removed, ADR 0019).
    """
    import dataclasses
    import inspect
    import pkgutil
    from enum import Enum

    package = importlib.import_module(f"{EQUIPMENT_PACKAGE}.vehicle")
    names = set()
    for info in pkgutil.iter_modules(package.__path__):
        module = importlib.import_module(f"{package.__name__}.{info.name}")
        for attr, obj in vars(module).items():
            if (
                inspect.isclass(obj)
                and obj.__module__ == module.__name__
                and not dataclasses.is_dataclass(obj)
                # An enumeration is neither a record nor a model. Mode, the
                # booster's propulsion setting, was being returned as a vehicle
                # model -- harmless for the page check, which only wants the
                # module a model lives in, but wrong for anything asking what a
                # model must provide.
                and not issubclass(obj, Enum)
            ):
                names.add(attr)
    assert names, "no vehicle models discovered -- the walk is looking in the wrong place"
    return frozenset(names)
