"""Everything in this directory is a conformance test, so the marker is
applied by location rather than by hand.

A conformance test checks the CODEBASE rather than the simulated system: the
truth boundary, layer discipline, the interface registry, the descriptor
cross-check, the generated documents, the palette. A failure indicts the
repository, not a model -- which is why these are a category of their own
rather than unit tests of something.

They need no simulation and no ensemble, and are the fastest thing here by an
order of magnitude. `pytest -m conformance` is the check worth running on
every save.

Note the path filter below, which is not optional.
"""

from __future__ import annotations

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


def pytest_collection_modifyitems(items):
    """Mark the items in THIS directory.

    pytest_collection_modifyitems is a global hook: a conftest in a
    subdirectory is still called once with the ENTIRE collection, not with its
    own subset. Marking `items` wholesale therefore marks every test in the
    repository, and the first version of this file did exactly that -- 318
    tests answered to `-m conformance`, which looks like a working filter
    until you count them.
    """
    for item in items:
        if HERE in Path(str(item.fspath)).resolve().parents:
            item.add_marker(pytest.mark.conformance)
