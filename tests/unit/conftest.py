"""Everything in this directory is a unit test, so the marker is
applied by location.

A failure indicts ONE component. Building a second one does not change
that: test_fuel_gauge.py needs a vehicle to supply truth, and the vehicle
is a fixture rather than a collaborator under test.

The path filter is not optional: pytest_collection_modifyitems is a global
hook, and a conftest in a subdirectory is still called with the ENTIRE
collection rather than its own subset.
"""

from __future__ import annotations

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(pytest.mark.unit)
