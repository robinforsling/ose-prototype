"""Everything in this directory is a behaviour test, marked by location.

A behaviour test composes a whole platform and asserts something about what it
does -- a claim no single component makes and no seam between two of them
explains. It is the category the demos were standing in for: they compose the
same stacks and compute the same numbers, and print them instead of asserting
them.

These are the slowest tests here, because emergent behaviour needs the system
run forward. Keep the simulated durations short -- tens of seconds, not the
demos' hundreds -- and mark anything past about half a second `slow`.

The path filter matters: pytest_collection_modifyitems is a global hook and a
subdirectory conftest is called with the entire collection. See the note in
tests/conformance/conftest.py.
"""

from __future__ import annotations

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(pytest.mark.behaviour)
