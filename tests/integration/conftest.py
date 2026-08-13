"""Everything in this directory is an integration test, so the marker is
applied by location.

A failure indicts a SEAM between components. These are the components
defined by what they combine -- an estimator by the measurement stream it
consumes, a manager by what it wraps, guidance by the manager and vehicle
beneath it.

The payload double-count (ADR 0026) is why the category is drawn this way:
it lived entirely between FuelGauge and VehicleManager, neither was wrong
alone, and no unit test could have seen it.

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
            item.add_marker(pytest.mark.integration)
