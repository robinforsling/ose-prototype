"""Component discovery shared by tests in different directories.

Not a test module. It holds the equipment-layer walk that two tests need from
opposite ends of the suite: a unit test asserting every equipment component
answers `capability()`, and a conformance test cross-checking the architecture
generator's own walk against an independent one.

It lives here rather than in either of them because the categories now sit in
separate directories, and a test importing another test module across a
directory boundary is a coupling that would break the next time a file moved.

The independence the conformance test relies on is unaffected: what makes that
comparison meaningful is that the generator derives its components by a
different route, not that the two walks live in different files.
"""

from __future__ import annotations

import dataclasses
import importlib
import inspect
import pkgutil
from enum import Enum


def equipment_components() -> list[tuple[str, type, str]]:
    """(class name, class, module) for every component in the equipment layer.

    Walks rather than globs. The glob version looked at `*.py` directly under
    ose/equipment/, which silently stopped covering the vehicle the moment
    vehicle.py became vehicle/ -- the test kept passing, kept claiming "every
    equipment module", and had quietly dropped two models. That is the same
    failure its predecessor had when a hand-written list omitted the
    integrated navigation unit, one level up (since removed, ADR 0019).

    reference_configs is skipped: it holds data, not components.
    """
    package = importlib.import_module("ose.equipment")
    found = []
    for info in pkgutil.walk_packages(package.__path__, prefix="ose.equipment."):
        if "reference_configs" in info.name:
            continue
        module = importlib.import_module(info.name)
        for attr, obj in vars(module).items():
            if (
                inspect.isclass(obj)
                and obj.__module__ == module.__name__
                and not attr.endswith("Parameters")
                and not dataclasses.is_dataclass(obj)
                and not issubclass(obj, Enum)
            ):
                found.append((attr, obj, info.name))
    return found
