# Refactor specification: split navigation across the resource and subsystem layers

Intended reader: an agent or contributor executing this refactor against the OSE
repository. Decisions here have already been made; this document is a brief, not
an invitation to redesign.

## Motivation

`InsGnssNavigation` currently combines sensor error models with the estimation
filter in one resource-layer component. The filter is purely cyber and by the
layer definitions in `docs/10-concepts.md` does not belong in the resource layer.

The decisive symptom is the signature:

```python
def update(self, t_s, dt_s, true_state, true_command, true_disturbance)
```

A component that is predominantly an estimator takes ground truth as an
argument. It happens to use truth only to synthesise measurements, but nothing
structural prevents it from doing otherwise. ADR 0008 is therefore currently a
convention rather than an invariant.

After the split, the estimator's public interface contains no truth-carrying
type at all, and the invariant is checkable from the signature.

## Target structure

```
src/ose/
  frames.py                       NEW  rotation utilities, no dependencies
  interfaces.py                   MODIFIED  measurement records and protocols
  resource/
    vehicle.py                    unchanged
    imu.py                        NEW  inertial sensor model
    gnss.py                       NEW  GNSS receiver model
    air_data.py                   NEW  air data sensor model
    integrated_nav.py             NEW  black-box nav unit (was AdditiveNoiseNavigation)
    navigation.py                 DELETED
  subsystem/
    __init__.py                   NEW
    navigation.py                 NEW  the error-state Kalman filter
```

## What moves where

| From `resource/navigation.py` | To | Notes |
|---|---|---|
| `rotation`, `d_rotation` | `ose/frames.py` | Unchanged. |
| `ImuParameters`, `ImuModel` | `resource/imu.py` | `ImuModel` becomes `Imu`; keeps its true bias state. |
| GNSS measurement synthesis inside `_gnss_update` | `resource/gnss.py` as `GnssReceiver` | Also takes over `set_gnss_available`. |
| Air data synthesis inside `_air_data_update` | `resource/air_data.py` as `AirDataSensor` | |
| `AidingParameters` | split, see below | |
| `InitialUncertainty` | `subsystem/navigation.py` | Unchanged. |
| `InsGnssNavigation` | `subsystem/navigation.py` as `InsGnssEstimator` | Filter internals unchanged. |
| `AdditiveNoiseNavigation`, `AdditiveNoiseParameters` | `resource/integrated_nav.py` as `IntegratedNavUnit`, `IntegratedNavParameters` | |

`AidingParameters` splits by ownership:

- `gnss_rate_hz`, `gnss_position_sigma_m`, `gnss_velocity_sigma_mps`,
  `gnss_velocity_enabled` become `GnssParameters` in `resource/gnss.py`.
- `air_data_rate_hz`, `air_data_sigma_mps` become `AirDataParameters` in
  `resource/air_data.py`.
- `wind_sigma_mps`, `wind_tau_s` become `EstimatorParameters` in
  `subsystem/navigation.py`; they describe the filter's wind process model, not
  any sensor.

## Interface definitions

All of the following go in `src/ose/interfaces.py`. Records are frozen
dataclasses. `OwnStateEstimate` stays as it is.

```python
@dataclass(frozen=True)
class ImuMeasurement:
    valid_time_s: float
    interval_s: float                       # interval this sample is held over
    specific_force_body_mps2: np.ndarray    # [x forward, y right]
    angular_rate_rad_s: float
    specific_force_sigma_mps2: float        # declared, per axis
    angular_rate_sigma_rad_s: float         # declared


@dataclass(frozen=True)
class GnssFix:
    valid_time_s: float
    position_m: np.ndarray                  # [north, east]
    position_sigma_m: float
    velocity_mps: np.ndarray | None         # [north, east], None if not provided
    velocity_sigma_mps: float | None


@dataclass(frozen=True)
class AirDataMeasurement:
    valid_time_s: float
    airspeed_mps: float
    airspeed_sigma_mps: float
```

Two properties of these records are deliberate and must not be optimised away.

**Every record carries `valid_time_s`** — the time the measurement refers to,
not the time it was delivered. This exists because a one-step misalignment
between the mechanised state and the truth used to form residuals previously
produced a filter thirty times overconfident, invisible in straight flight. A
measurement that cannot be constructed without its timestamp makes that class of
fault hard to write.

**Every record carries its own declared uncertainty.** The estimator uses the
sigma travelling with the measurement, never a separately configured value. At
present the same `AidingParameters` object supplies both the noise used to
corrupt truth and the noise used in the filter's `R`, so the filter silently
knows the true error statistics. After the split the sensor *declares* its
accuracy and the estimator believes the declaration — which additionally makes
a sensor that misreports its own accuracy expressible, and that is a realistic
failure mode worth being able to simulate.

Protocols, also in `interfaces.py`, all `@runtime_checkable`:

```python
class InertialSensor(Protocol):
    def sample(self, t_s, dt_s, true_state, true_command,
               true_disturbance) -> ImuMeasurement: ...


class PositioningSensor(Protocol):
    def sample(self, t_s, true_state,
               true_disturbance) -> GnssFix | None: ...      # None when denied


class AirDataSensor(Protocol):
    def sample(self, t_s, true_state) -> AirDataMeasurement: ...


class NavigationEstimator(Protocol):
    def ingest(self, measurement) -> None: ...
    def estimate(self, t_s: float) -> OwnStateEstimate: ...


class OwnStateSource(Protocol):
    """Anything publishing vehicle.state.v1, whatever layer it sits in."""
    def estimate(self, t_s: float) -> OwnStateEstimate: ...
```

`ingest` dispatches on measurement type. An `ImuMeasurement` drives prediction;
the others drive correction. A single dispatching method is chosen over named
per-type methods so that a further aiding source — magnetometer, terrain
referenced navigation, visual odometry — can be added without changing the
protocol. Unknown measurement types raise `TypeError`.

## Ordering contract

The estimator maintains an internal current time. Measurements must be ingested
in non-decreasing `valid_time_s` order, and the estimator raises `ValueError` on
an out-of-order measurement. Corrections at time `t` must be applied before any
prediction past `t`.

The calling sequence per step is therefore:

```python
imu_m  = imu.sample(t, dt, state, cmd, dist)
fix    = gnss.sample(t, state, dist) if gnss_due else None
air_m  = air.sample(t, state)        if air_due  else None

if fix   is not None: estimator.ingest(fix)      # corrections at t
if air_m is not None: estimator.ingest(air_m)
own_state = estimator.estimate(t)                # published estimate refers to t
estimator.ingest(imu_m)                          # prediction to t + dt
```

Correct-then-predict, with the published estimate taken between the two. This is
the ordering that fixed the consistency failure and it is now the contract rather
than an implementation detail.

## Where `IntegratedNavUnit` sits

It stays in the resource layer and publishes `vehicle.state.v1` directly,
satisfying `OwnStateSource` but not `NavigationEstimator`. It models a black-box
integrated navigation unit whose internals are not simulated.

This deliberately collapses the two layers, and the docstring must say so
explicitly: it is valid when navigation is not the component under test, and
invalid as a baseline for any claim about navigation performance.

## Random number streams

Each component takes its own `numpy.random.Generator`. Do **not** attempt to
preserve the existing draw order, and do not share one generator across
components to keep results bit-identical with the current implementation.

Per-component streams derived from a run seed are what ADR 0005 already
specifies, and independence is the property that matters: adding a sensor to a
platform must not perturb the random stream of any other component.

Numerical results will therefore change. That is expected and acceptable.

## What must not change

The filter internals are correct and were expensive to make so. Move them
verbatim:

- the ten-element error state and its ordering,
- the `F` matrix, including `F[v, psi] = dR/dpsi @ f_hat` and `F[v, b_a] = -R`,
- the process noise construction,
- the Joseph-form covariance update,
- the midpoint-attitude rotation in the mechanisation,
- the measurement Jacobians, including the air data unit-vector row.

Any change to these is out of scope for this refactor.

## Tests

Split `tests/test_navigation.py` into `tests/test_nav_sensors.py` and
`tests/test_navigation_estimator.py`.

**Existing assertions must not be weakened.** In particular the NEES threshold
(`< 6.0`), the three-sigma containment thresholds (`> 0.95`), the observability
tests, and the outage tests keep their current values. Only construction and
imports change. If one of these now fails, the refactor is wrong — do not adjust
the threshold.

New tests to add:

*Sensors*

- IMU sample mean over many draws approaches true specific force plus true bias.
- IMU sample standard deviation matches the declared sigma to within 10 percent.
- IMU true bias behaves as a first-order Gauss-Markov process: steady-state
  variance matches `bias_sigma**2` over a long run.
- `valid_time_s` on every record equals the time requested.
- `GnssReceiver.sample` returns `None` while denied and a fix once restored.
- Declared sigma on a `GnssFix` equals the receiver's configured sigma.

*Estimator*

- **The estimator cannot see truth.** Parse `src/ose/subsystem/navigation.py`
  with `ast` and assert that it imports neither `Disturbance` nor `VehicleState`
  from `ose.resource.vehicle`, and that no public method has a parameter whose
  name begins with `true_`. This is blunt, and that is the point: it fails
  loudly if truth is reintroduced for convenience.
- **Replay determinism.** Record a measurement stream from one run into a list,
  then replay that list into a freshly constructed estimator. The resulting
  `OwnStateEstimate` sequence must be identical to floating-point equality. This
  proves the estimator is a pure function of its measurement stream, which is
  the property the whole split exists to establish.
- Out-of-order ingestion raises `ValueError`.
- Ingesting an unknown type raises `TypeError`.

## Demos

`demos/demo_navigation.py` constructs the four components explicitly and drives
them with the sequence above. The plot layout and the printed summary keep their
current form. Add one line to the printed summary reporting the number of GNSS
fixes actually received, since that is now a property of the receiver rather than
of the filter.

## Documentation

- **New ADR `0009-navigation-split-across-layers.md`.** Context: the estimator is
  cyber and the truth-carrying signature made ADR 0008 unenforceable. Decision:
  sensors in the resource layer, estimator in the subsystem layer, measurements
  carrying valid-time and declared uncertainty. Consequences: filters become
  interchangeable against a common measurement stream and testable by replay;
  `IntegratedNavUnit` remains as a deliberate layer collapse; results are no
  longer bit-comparable with earlier runs. Add the row to `docs/adr/README.md`.
- **`docs/interfaces/README.md`**: add `sensing.imu.v1`, `sensing.gnss.v1`,
  `sensing.airdata.v1` as implemented, with their record fields, rates, frames
  and units. Update the `vehicle.state.v1` entry to note it may be published
  either by a subsystem estimator or by a resource-layer integrated unit.
- **`docs/20-architecture.md`**: update the current-state paragraph and the
  repository layout block.
- **`README.md`**: update the layout block and the component list.
- **`docs/10-concepts.md`**: add a short subsection on measurement records
  stating the valid-time and declared-uncertainty rules.

## Acceptance criteria

1. `pytest` passes with no assertion thresholds weakened.
2. All three demos run and produce their figures.
3. `test_estimator_cannot_see_truth` passes.
4. `test_replay_determinism` passes.
5. `ose.subsystem.navigation` imports nothing from `ose.resource` except
   `ose.frames` and `ose.interfaces`.
6. Bit-identical reproduction of previous numerical results is explicitly **not**
   required.

## Suggested order of work

1. `ose/frames.py`, then the records and protocols in `interfaces.py`.
2. The three sensors, with their tests. Verify against the old implementation by
   comparing measurement statistics before wiring the estimator.
3. The estimator, moving filter internals verbatim.
4. `IntegratedNavUnit`.
5. Demos, then documentation and the ADR.
6. Delete `resource/navigation.py` only once everything passes.

Commit after each step. Step 3 is the one where a subtle error will be expensive
to localise later, so run the consistency tests immediately after it rather than
at the end.
