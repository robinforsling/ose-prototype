"""The model reference pages must not go stale.

docs/models/vehicle/ carries parameter tables, limit tables and
turn-performance tables for each vehicle model. Those numbers come from the
code, and a document that quietly stops matching the thing it documents is
worse than no document -- a reader trusts it precisely because it looks
specific.

So the numbers are generated and this checks they are current. Change a
reference configuration or a model and the suite fails here until
`python tools/generate_model_docs.py` has been run.

The prose is NOT generated and this does not check it. Whether a page still
describes the behaviour correctly is a judgement, and pretending a test can
make it is worse than admitting it cannot. What the second test below does
catch is the narrower case of a hand-written sentence quoting a number that
has since moved, which is the failure that actually happened while the pages
were being written.
"""

import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
GENERATOR = ROOT / "tools" / "generate_model_docs.py"
DOCS = ROOT / "docs" / "models" / "vehicle"


def _load_generator():
    spec = importlib.util.spec_from_file_location("generate_model_docs", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_generated_tables_are_current():
    """Run the generator's own --check in a subprocess, so the test exercises
    the entry point a contributor is told to run rather than a private path
    into it."""
    result = subprocess.run(
        [sys.executable, str(GENERATOR), "--check"],
        capture_output=True, text=True, cwd=ROOT,
    )
    assert result.returncode == 0, (
        "the model reference pages are out of date with the code:\n"
        + result.stdout + result.stderr
    )


def test_figures_quoted_in_prose_still_match_the_code():
    """Sentences quote numbers too, and wrapping every sentence in generator
    markers would make the pages unreadable. These are checked instead.

    This is the failure that actually occurred: an early draft cited a stall
    floor and a level-flight ceiling worked out by hand, and one of them was
    wrong by the time the page was finished.
    """
    generator = _load_generator()
    missing = []
    for name, figures in generator.prose_figures().items():
        text = (DOCS / name).read_text()
        missing += [f"{name} no longer contains {f!r}" for f in figures
                    if f not in text]
    assert not missing, (
        "a figure quoted in prose has moved:\n  " + "\n  ".join(missing)
    )


def test_every_vehicle_model_has_a_page():
    """A model without a page is the drift this whole arrangement exists to
    prevent, and it is the easy one to catch: the pages are named after the
    modules."""
    import _truth_boundary

    documented = {p.stem for p in DOCS.glob("*.md")}
    modules = {
        m.rsplit(".", 1)[-1]
        for _, _, m in _equipment_vehicle_modules(_truth_boundary)
    }
    assert modules <= documented, (
        f"vehicle models with no page in docs/models/vehicle/: "
        f"{sorted(modules - documented)}"
    )


def _equipment_vehicle_modules(truth_boundary):
    """Every module under ose.equipment.vehicle that defines a model, reusing
    the discovery the truth-boundary guard already does rather than keeping a
    second list."""
    import importlib
    import inspect
    import pkgutil

    package = importlib.import_module("ose.equipment.vehicle")
    models = truth_boundary.vehicle_model_names()
    out = []
    for info in pkgutil.iter_modules(package.__path__):
        module = importlib.import_module(f"{package.__name__}.{info.name}")
        for attr, obj in vars(module).items():
            if inspect.isclass(obj) and attr in models and obj.__module__ == module.__name__:
                out.append((attr, obj, module.__name__))
    assert out, "no vehicle models discovered -- this test is vacuous"
    return out


def test_the_generator_notices_a_new_constraint():
    """The lambda table is hand-mapped from field to symbol, so a new field
    would otherwise be silently absent from the documented lambda. The
    generator raises instead."""
    import dataclasses

    generator = _load_generator()
    from ose.equipment.vehicle import Constraints

    documented = dataclasses.replace(generator.FIGHTER_LIMITS)
    assert generator.lambda_table()          # the real one works

    @dataclasses.dataclass(frozen=True)
    class Extended(Constraints):
        something_new: float = 0.0

    original = generator.FIGHTER_LIMITS
    generator.FIGHTER_LIMITS = Extended(
        **dataclasses.asdict(documented), something_new=1.0
    )
    try:
        with pytest.raises(SystemExit, match="something_new"):
            generator.lambda_table()
    finally:
        generator.FIGHTER_LIMITS = original
