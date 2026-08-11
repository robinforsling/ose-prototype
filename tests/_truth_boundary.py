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

  1. The layer is named once, here, so a future rename has one place to touch.
  2. `_assert_guard_is_live()` imports the module the guard names before using
     it. A stale name now raises ImportError instead of quietly matching
     nothing. Centralising alone would not have caught the failure; this does.
"""

from __future__ import annotations

import ast
import importlib
from pathlib import Path

EQUIPMENT_PACKAGE = "ose.equipment"
TRUTH_MODULE = f"{EQUIPMENT_PACKAGE}.vehicle"

# The types that carry ground truth. A cyber-layer component holding one of
# these is wrong regardless of what it does with it (ADR 0008). VehicleCommand
# and Saturation live in the same module and are not truth -- they are records
# a component may legitimately construct and receive.
TRUTH_TYPES = frozenset({"VehicleState", "Disturbance"})

SRC = Path(__file__).resolve().parents[1] / "src"


def _assert_guard_is_live() -> None:
    """Fail loudly if this guard has gone stale.

    The whole failure mode above is a string that names nothing. Importing it
    turns that into an error at the point of use.
    """
    importlib.import_module(TRUTH_MODULE)


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
        if isinstance(node, ast.ImportFrom) and node.module == TRUTH_MODULE:
            leaked = {a.name for a in node.names} & TRUTH_TYPES
            assert not leaked, f"{path.name} imports truth-carrying types: {leaked}"


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
