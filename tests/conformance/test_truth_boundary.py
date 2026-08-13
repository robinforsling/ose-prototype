"""Invariant 1: only equipment-layer components read ground truth.

These checks used to live one per component test file -- six calls to
`assert_no_truth_types` and `assert_no_truth_parameters`, each naming its own
module by hand. They are conformance tests wherever they sit: they ast-parse a
module and never run anything, and a failure indicts the codebase rather than
any model. Gathering them here is what the category is for.

Gathering also made them a walk. Every cyber component is checked, discovered
rather than listed, so a component added tomorrow is covered without anyone
choosing to cover it. The six hand-placed calls happened to cover all six cyber
components, but only because someone remembered each time -- the same fragility
ADR 0024 removed from layer discipline, and the same shape as the four other
hand-written lists this repository has been bitten by.

The stricter per-component rules stay per-component: a subsystem component MAY
bind equipment, and the two that must not are a deliberate choice about those
components rather than a property of their layer.
"""

from __future__ import annotations

import ast
import importlib
import importlib.util
import pkgutil
from pathlib import Path

import pytest
from _truth_boundary import (
    assert_no_equipment_imports,
    assert_no_truth_parameters,
    assert_no_truth_types,
)

from ose.topology import LAYER_PACKAGES, PHYSICAL_LAYERS

SRC = Path(__file__).resolve().parents[2] / "src"

# Components that additionally must not reference the equipment layer at all.
# Stricter than the layer rule, which permits a subsystem component to bind
# equipment: these two are pure functions of a measurement stream, and reaching
# for a model would be a design change rather than a slip.
NO_EQUIPMENT_AT_ALL = {
    "ose.subsystem.time_state_estimator",
    "ose.single_ship.action_planner",
}


def cyber_modules() -> list[tuple[str, Path]]:
    """(module name, path) for every component module above the truth boundary."""
    out = []
    for layer, package_name in LAYER_PACKAGES.items():
        if layer in PHYSICAL_LAYERS:
            continue
        if importlib.util.find_spec(package_name) is None:
            continue
        package = importlib.import_module(package_name)
        for info in pkgutil.walk_packages(package.__path__, prefix=f"{package_name}."):
            if "reference_configs" in info.name:
                continue
            path = SRC.joinpath(*info.name.split(".")).with_suffix(".py")
            if path.is_file():
                out.append((info.name, path))
    return out


def test_the_walk_is_not_vacuous():
    """Every test below iterates it."""
    modules = cyber_modules()
    assert len(modules) >= 5, f"only {len(modules)} cyber modules found"
    assert any("single_ship" in name for name, _ in modules)
    assert any("subsystem" in name for name, _ in modules)


def test_no_cyber_component_imports_a_truth_carrying_type():
    """VehicleState or Disturbance held above the boundary is wrong regardless
    of what is done with it (ADR 0008)."""
    for name, path in cyber_modules():
        assert_no_truth_types(path)


def test_no_cyber_component_takes_truth_as_a_parameter():
    """A signature carrying truth is wrong whatever the body does with it, and
    it is how a composition leaks while every component stays clean."""
    for name, path in cyber_modules():
        assert_no_truth_parameters(path)


@pytest.mark.parametrize("module", sorted(NO_EQUIPMENT_AT_ALL))
def test_the_pure_filters_do_not_reference_equipment_at_all(module):
    """Stricter than the layer rule, and deliberately per-component."""
    path = SRC.joinpath(*module.split(".")).with_suffix(".py")
    assert path.is_file(), f"{module} has moved; this guard names nothing"
    assert_no_equipment_imports(path)


def test_no_cyber_component_reads_the_true_burn_coefficient():
    """The burn coefficient a filter predicts with must be the platform's
    BELIEF, never the coefficient the vehicle actually burns at.

    Predicting with the true one makes the prediction exact by construction:
    tsfc_error is pinned at zero, the filter looks excellent for a reason that
    never holds on a real platform, and every mass consistency test becomes
    vacuous.

    Enforced across the repository rather than on vehicle_manager.py alone,
    because predict() takes the coefficient as an argument. A guard on that one
    file would keep passing while a caller did
    predict(t, T, vehicle.theta.c_tsfc) -- component clean, composition
    leaking, which is precisely how the mass parameter went wrong before
    ADR 0015.

    The equipment layer is exempt: the vehicle owns the coefficient, and Imu
    already reaches into the model for drag_N as a peer. Demos and tests are
    out of scope for the same reason renderers may read truth -- they are
    tools, not components -- and today they are the only callers.
    """
    root = SRC / "ose"
    offenders = []
    for path in sorted(root.rglob("*.py")):
        if "equipment" in path.relative_to(root).parts:
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Attribute) and node.attr == "c_tsfc":
                offenders.append(str(path.relative_to(root)))
                break
    assert not offenders, (
        "cyber-layer code reading the vehicle's true burn coefficient: "
        f"{offenders}"
    )
