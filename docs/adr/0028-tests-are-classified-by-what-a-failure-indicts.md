# 0028 — Tests are classified by what a failure indicts

**Status:** Accepted.

## Context

318 tests across 21 files, with no stated organising principle and no way to
run a subset by kind. `tests/README.md` described what several files pinned,
file by file, but nothing said what *kind* of test anything was.

Three consequences, all measured rather than supposed:

- **The whole suite was the only granularity**, at 32 s, of which 11 s was five
  Monte Carlo ensembles. There was no fast inner loop.
- **One category was empty.** Nothing tested the platform's emergent
  behaviour. The nearest things were the demos, which are not tests:
  `demo_live_flight.py` computed that peak turn rate lands within 0.2 m/s of
  `v_corner` and *printed* it.
- **A whole category had no name.** 65 tests checked the codebase rather than
  the simulated system — truth boundary, layer discipline, the interface
  registry, the descriptor cross-check, generated-document staleness, palette
  contrast — and ran in 5.5 s with no simulation involved.

The obvious axis is how many components a test constructs. It does not work:
**ten of the twenty-one files build two or more**, so counting them would put
half the suite in "integration" and the label would stop meaning anything.
`test_fuel_gauge.py` builds a vehicle and a gauge and is plainly a unit test —
the vehicle is a fixture supplying truth, not a collaborator under test.

## Decision

**Classify by what a failure indicts.**

| Category | A failure indicts |
|---|---|
| `unit/` | one component |
| `integration/` | a seam between components |
| `behaviour/` | the platform's emergent behaviour |
| `conformance/` | the codebase itself, not the simulated system |

The four **partition** the suite: every test is exactly one, checked at
collection.

The seam definition is not a re-labelling. The payload double-count (ADR 0026)
lived entirely between `FuelGauge` and `VehicleManager` — neither component was
wrong on its own, and no unit test could have seen it. Under this definition
that is an integration test and would have been written as one.

**Two orthogonal markers.** `performance` for a claim about *system*
performance — accuracy, envelope, endurance; the thresholds CLAUDE.md forbids
weakening. `slow` for anything over about half a second. Software speed and
memory are a different question and are deliberately out of scope: a filter can
be accurate and slow, or fast and overconfident, and one marker for both would
say neither.

**One directory per category.** `unit/`, `integration/`, `behaviour/` and
`conformance/`, each with a conftest applying its marker by location, so
`pytest -m unit` and `pytest tests/unit` select the same tests. Every test file
moved into one of the four.

**A test file outside them is a collection error**, as is a test in two
categories. There is nowhere for it to belong, and a taxonomy nobody is obliged
to apply decays into one nobody applies; the argument is ADR 0024's.

An earlier version of this decision kept component tests at `tests/` root and
distinguished `unit` from `integration` with a module-level `TEST_KIND` string,
reasoning that a file might hold both kinds and that fourteen files need not
move. Neither held up. The classification turned out to be per-file anyway --
no component test file mixed the two -- so the string bought nothing a
directory does not, and the split it left was harder to see than the one it
replaced.

## Consequences

`pytest -m "not slow"` is 17 s against 32 s, and `pytest -m conformance` is
5.7 s. There is now an inner loop.

The behaviour category has ten members where it had none, all composed the way
a demo composes and asserting what the demos printed. Writing them was worth
more than the classification: **two of the three assumed a behaviour the system
does not have.** The route test first asserted that the infeasible corner was
the slowest leg — it is the *shortest*, because leg time is dominated by
distance, and the real signature is flown path over direct distance, 1.33
against 1.00 elsewhere. The envelope test first demanded the delivered turn
rate equal the true limit exactly; it does not, because guidance clips at the
*believed* mass, and the residual is the fuel gauge's error (ADR 0015). Both
were my error rather than the code's, and both are now asserted as the property
that actually holds.

**Classification is a judgement, and some of it is coarse.**
`integration/test_action_planner.py` sits there because it composes a guidance
stack to obtain a capability, though several of its tests are pure waypoint
geometry. Moving one test is a `git mv` and nothing depends on the current
split being final.

**`test_integration.py` was renamed `test_integrators.py`.** It tests the RK4
integrators in `ose/integration.py` and had nothing to do with integration
testing. The collision would have been permanent confusion.

**Moving every file forced two improvements that were not the point.** The six
`test_*_cannot_see_truth` calls, one per component file, became a single walk
over every cyber component in `conformance/test_truth_boundary.py` -- they were
conformance tests wherever they sat, and hand-listing six modules was the
fragility ADR 0024 removed from layer discipline. And the equipment-layer walk
that a unit test and a conformance test both need moved to
`tests/_discovery.py`, replacing a test module importing another test module
across what is now a directory boundary.

**Two mechanical traps were hit and are worth recording.**
`pytest_collection_modifyitems` in a subdirectory conftest is called with the
*entire* collection, not that directory's subset — the first version marked all
318 tests `conformance`, which looks like a working filter until you count
them. And pytest's `prepend` import mode puts each test file's own directory on
`sys.path`, not the test root, so moving files broke `from _truth_boundary
import …`; a root conftest fixes it once.

**The category counts are in `tests/README.md` and will drift.** They are
useful enough to state and not worth generating.

## References

- ADR 0024 — enforce-don't-remember, the argument for the collection guard
- ADR 0026 — the seam defect that motivates the integration definition
- ADR 0015 — the believed-mass residual the envelope test measures
