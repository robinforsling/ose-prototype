# Tests

```bash
pytest                       # all 328
pytest -m "not slow"         # the inner loop, about half the time
pytest -m conformance        # the codebase checks, ~6 s, no simulation
pytest -m behaviour          # whole-platform runs
pytest -m performance        # every accuracy, envelope and endurance claim
pytest -k consistent
```

## Four categories

Every test is exactly one of these, and the split is **what a failure
indicts** -- not how many components a test constructs. Ten of the component
files build two or more; `test_fuel_gauge.py` builds a vehicle *and* a gauge,
and is still a unit test, because the vehicle is a fixture supplying truth
rather than a collaborator under test. See ADR 0028.

| Category | A failure indicts | Count |
|---|---|---|
| `unit` | one component | 111 |
| `integration` | a seam between components | 135 |
| `behaviour` | the platform's emergent behaviour | 10 |
| `conformance` | the codebase itself, not the simulated system | 72 |

The seam definition earns its keep. The payload double-count (ADR 0026) lived
entirely between `FuelGauge` and `VehicleManager`: neither component was wrong
on its own, and no unit test could have seen it.

Two orthogonal markers apply to any category:

- **`performance`** -- asserts a claim about *system* performance: accuracy,
  envelope, endurance. NEES bands, three-sigma containment, corner speed.
  These are the thresholds CLAUDE.md forbids weakening. Not software speed.
- **`slow`** -- over about half a second, in practice the Monte Carlo
  ensembles.

### How a test gets its category

A file states its usual kind once, near the top:

```python
TEST_KIND = "unit"
```

and a test that differs carries a marker, which wins. `pytestmark` would not
work: a module-level marker and a function-level one both apply, and the test
would land in two categories at once.

Tests under `behaviour/` and `conformance/` are marked by their directory, so
those files need no `TEST_KIND`. The marker is the category; the directory is
a convenience that applies one automatically. That is why every
`test_*_cannot_see_truth` is marked `conformance` while living beside the
component it guards -- it ast-parses a module and runs nothing, but it belongs
next to its subject.

An unclassified test, or one in two categories, is a **collection error**.
A taxonomy nobody is obliged to apply decays into one nobody applies; the same
argument as ADR 0024.

### Where behaviour tests come from

`behaviour/` composes what a demo composes and asserts what the demos print.
`demo_live_flight.py` already computed that peak turn rate lands within
0.2 m/s of `v_corner`; `test_envelope.py` asserts it. The platform assembly is
shared in `behaviour/_platform.py` and deliberately not imported from `demos/`
-- those are throwaway prototypes of the simulation core, take command-line
arguments, and open windows.

## What is being pinned

`test_vehicle.py` checks identities that follow from the model document rather
than numbers that happened to come out of a run: the coordinated-turn relation,
induced drag scaling with load factor squared, the stall speed closed form,
corner speed maximising instantaneous turn rate, thrust required equalling drag,
sustained turn rate balancing thrust and drag, fourth-order convergence of the
integrator, and the frame conventions.

Two tests assert what the vehicle deliberately does *not* do: it integrates an
inadmissible command as given rather than clipping it. If those start failing,
the separation in ADR 0006 has been broken.

Navigation is split across four files, one per component (ADR 0009):

`test_navigation_sensors.py` checks that each equipment-layer sensor's
declared sigma is honest — sample mean and standard deviation against many
draws — plus the IMU bias's Gauss-Markov steady state and GNSS
denial/restoration.

`test_navigation_state_estimator.py` checks the subsystem-layer filter: NEES
consistency across several seeds, that the published covariance is positive
semi-definite, the observability structure (heading variance must not shrink
before the first turn, and must collapse during it), `ast`-parses the module
to confirm it cannot see truth, and replays a recorded measurement stream
into a fresh estimator to confirm it is a pure function of that stream.

Consistency is checked on the **whole** object, not one channel: all four
published channels against the full covariance, and separately the IMU biases
and wind — error states no consumer ever reads, where a defect can land
without a published channel moving. It also checks that air data holds
airspeed through a GNSS outage and that a fifteen-degree initial heading error
converges. That last group replaced the reassurance previously taken from
comparing against a black-box stand-in, which ADR 0019 removed.

`test_clock.py` and `test_time_estimator.py` are the same pattern applied to
the platform clock (ADR 0010): declared sigma honesty and the drift's
Gauss-Markov steady state for the sensor; NEES consistency, the truth
boundary, and replay determinism for the estimator, plus that
`platform_time_s` is exactly the running sum of readings and its uncertainty
never decreases — there is no correction source yet, so nothing should ever
look more confident than dead reckoning warrants.

`test_vehicle_guidance.py` (ADR 0011) is where enforcement is finally
exercised: `test_vehicle.py`'s two tests above check that the vehicle
itself does *not* clip an inadmissible command; `test_reports_saturation_
when_setpoint_exceeds_envelope` checks that guidance does, and that the
clipping comes back as a visible `Saturation` finding rather than being
absorbed silently. Also checks the truth boundary (guidance only ever
touches `OwnStateEstimate`) and closed-loop convergence to a commanded
heading and speed.

`test_capability.py` (ADR 0012) applies the same honesty argument to
capability that the NEES tests apply to covariance. It does not check that
`capability()` returns plausible numbers; it integrates the dynamics forward
and checks the vehicle delivers what it claimed — the one thing capability
promises to answer *without* integrating. It also checks that every equipment component
can be asked at all, and that each sensor's declared accuracy agrees with what
its own measurements carry.

The endurance test is worth reading before writing another capability test.
Its first version missed a ten percent overstatement completely: fuel flow
stops once the tanks are dry, so flying past an inflated claim leaves the
remaining fuel at zero and looks perfectly healthy. Overstating endurance is
the dangerous direction, and catching it needs an assertion that fuel still
remains *shortly before* the claim. A capability test that checks only one side
of a claim can be worse than no test, because it looks like coverage.

## Why consistency is tested rather than eyeballed

The INS/GNSS filter shipped with a one-step misalignment between the mechanised
state and the truth used to form measurement residuals. In straight flight the
velocity vector is not rotating, so the residual vanished and everything looked
correct. Under turn it injected a systematic 3 m/s velocity residual against
0.15 m/s of measurement noise, and the filter absorbed it into heading and bias,
finishing thirty times overconfident.

Four wrong hypotheses were investigated before the real cause was found. A NEES
test would have pointed at it immediately. Any component that publishes an
uncertainty should carry one.
