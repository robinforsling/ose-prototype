"""Shared test configuration.

Four categories, four directories
---------------------------------
Every test lives in exactly one of `unit/`, `integration/`, `behaviour/` or
`conformance/`, and each directory's conftest applies its marker by location.
The axis is what a FAILURE INDICTS -- one component, a seam between
components, the platform's emergent behaviour, or the codebase itself -- not
how many components a test constructs. See tests/README.md and ADR 0028.

A test at this level, in no category directory, is a collection error. There is
nowhere for it to belong, and a taxonomy nobody is obliged to apply decays into
one nobody applies (ADR 0024).

`tests/` on sys.path
--------------------
pytest's default `prepend` import mode inserts each test file's OWN directory
on sys.path, not the test root. So `tests/unit/test_clock.py` gets
`tests/unit/` and nothing else, and the shared helpers here --
`_truth_boundary.py` and `_discovery.py` -- would not be importable at all.

Adding the test root once here is what makes them shared rather than
duplicated per directory, which is the whole reason they exist.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

TESTS = Path(__file__).resolve().parent

if str(TESTS) not in sys.path:
    sys.path.insert(0, str(TESTS))

CATEGORIES = ("unit", "integration", "behaviour", "conformance")


def pytest_collection_modifyitems(items):
    """Check the categories partition the suite.

    Runs after the directory conftests, which is where the markers are applied.
    """
    unclassified, ambiguous = [], []
    for item in items:
        marked = [k for k in CATEGORIES if item.get_closest_marker(k)]
        if not marked:
            unclassified.append(item.nodeid)
        elif len(marked) > 1:
            ambiguous.append(f"{item.nodeid}: {marked}")

    problems = []
    if unclassified:
        problems.append(
            "tests outside every category directory -- move them into unit/, "
            "integration/, behaviour/ or conformance/:\n  "
            + "\n  ".join(unclassified)
        )
    if ambiguous:
        problems.append(
            "tests in more than one category:\n  " + "\n  ".join(ambiguous)
        )
    if problems:
        raise pytest.UsageError("\n\n".join(problems))
