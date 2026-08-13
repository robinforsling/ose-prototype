"""Shared test configuration.

`tests/` on sys.path
--------------------
pytest's default `prepend` import mode inserts each test file's OWN directory
on sys.path, not the test root. So `tests/conformance/test_interfaces.py` gets
`tests/conformance/` and nothing else, and the shared helpers at `tests/` --
`_truth_boundary.py`, and the component walk in `test_capability.py` that a
conformance test cross-checks against -- stop being importable the moment a
file moves into a subdirectory.

Adding the test root here fixes that once, rather than through an __init__.py
per directory (which changes how pytest names modules) or by duplicating the
helpers (which is what they exist to avoid).

Categories
----------
Four, and they partition the suite: every test is exactly one of `unit`,
`integration`, `behaviour`, `conformance`. See tests/README.md and ADR 0028.

A file states its usual kind once, as a module-level string:

    TEST_KIND = "unit"

and a test that differs says so with a marker, which wins. `pytestmark` would
not work for this: a module-level marker and a function-level one both apply,
and the test would be in two categories at once.

That matters because the majority of a file is usually one kind and the
exceptions are what a reader needs told. Every `test_*_cannot_see_truth` is a
conformance test -- it ast-parses a module and never runs anything -- but it
belongs beside the component it guards, not in conformance/. The marker is the
category; the directory is a convenience that applies one automatically.
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
    """Apply each module's default kind, then check the partition holds.

    Runs after the directory conftests, so a test in behaviour/ or
    conformance/ already carries its marker and is left alone.
    """
    for item in items:
        if not any(item.get_closest_marker(k) for k in CATEGORIES):
            kind = getattr(item.module, "TEST_KIND", None)
            if kind in CATEGORIES:
                item.add_marker(getattr(pytest.mark, kind))

    unclassified, ambiguous = [], []
    for item in items:
        marked = [k for k in CATEGORIES if item.get_closest_marker(k)]
        if not marked:
            unclassified.append(item.nodeid)
        elif len(marked) > 1:
            ambiguous.append(f"{item.nodeid}: {marked}")

    # A taxonomy nobody is obliged to apply decays into one nobody applies.
    # This repository's rule is enforce-don't-remember (ADR 0024), so an
    # unclassified test is a collection error rather than a silent gap.
    problems = []
    if unclassified:
        problems.append(
            "tests with no category (set TEST_KIND in the module, or mark the "
            "test):\n  " + "\n  ".join(unclassified)
        )
    if ambiguous:
        problems.append(
            "tests in more than one category:\n  " + "\n  ".join(ambiguous)
        )
    if problems:
        raise pytest.UsageError("\n\n".join(problems))
