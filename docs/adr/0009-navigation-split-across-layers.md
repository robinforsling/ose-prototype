# 0009. Split navigation across the equipment and subsystem layers

Status: accepted
Date: 2026-08-09

## Context

`InsGnssNavigation` combined sensor error models with the estimation filter in
one equipment-layer component. The filter is predominantly cyber and, by the
layer definitions in `docs/10-concepts.md`, does not belong in the equipment
layer.

The decisive symptom was the signature:

```python
def update(self, t_s, dt_s, true_state, true_command, true_disturbance)
```

A component that is predominantly an estimator took ground truth as an
argument. It happened to use truth only to synthesise measurements, but
nothing structural prevented it from doing otherwise. ADR 0008 was therefore a
convention for this component rather than an invariant checkable from its
signature.

The same object, `AidingParameters`, also supplied both the noise used to
corrupt truth and the noise used in the filter's measurement covariance,
so the filter silently knew the true error statistics of its own sensors.

## Decision

Split into sensors (equipment layer: `Imu`, `GnssReceiver`, `AirDataSensor`)
and an estimator (subsystem layer: `InsGnssEstimator`), communicating only
through measurement records (`ImuMeasurement`, `GnssFix`,
`AirDataMeasurement`) defined in `interfaces.py`. Every record carries its own
`valid_time_s` -- the time it refers to, not the time it was delivered -- and
its own declared uncertainty. The estimator uses the sigma travelling with
the measurement, never a separately configured value.

The estimator's public interface (`ingest`/`estimate`, the
`NavigationEstimator` protocol) contains no truth-carrying type. This is
enforced by `test_estimator_cannot_see_truth`, which parses
`subsystem/navigation_state_estimator.py` with `ast` and fails if it imports
`VehicleState` or `Disturbance` from `ose.equipment.vehicle`, or if any public
method takes a parameter named `true_*`.

`ingest` dispatches on measurement type rather than exposing named per-type
methods, so a further aiding source can be added without changing the
protocol. Measurements must arrive in non-decreasing `valid_time_s` order;
the estimator raises `ValueError` otherwise, and `TypeError` on an
unrecognised measurement type.

One consequence of the split turned out not to be anticipated going in: the
filter's mechanisation, covariance propagation and Kalman correction have no
random draws of their own once sensor corruption moves out. `InsGnssEstimator`
therefore takes no RNG and is a pure function of its measurement stream --
`test_replay_determinism` replays a recorded stream into a fresh estimator and
checks the resulting estimates are identical to floating-point equality.

The filter's assumed IMU-bias behaviour and its wind process model
(`EstimatorParameters`) are now the estimator's own prior, separate from
whatever the true sensor in `equipment/imu.py` actually does; only the
white-noise part of the process model is taken from the incoming
measurement's declared sigma.

`IntegratedNavUnit` (`equipment/integrated_nav.py`) replaces
`AdditiveNoiseNavigation` as a deliberate collapse of both layers into one
black-box component: valid scaffolding when navigation is not the component
under test, not a baseline for any claim about navigation performance. It
satisfies `OwnStateSource` (publishes `vehicle.state.v1`) but not
`NavigationEstimator`, since it has no measurement stream to ingest or
replay.

## Consequences

The truth/perception boundary is checkable from the estimator's signature and
imports, not just from convention. A navigation filter is now testable by
replay in isolation from the vehicle model and from any particular sensor
implementation, and a different estimator can be substituted against the same
measurement stream.

Each component draws from its own RNG stream (ADR 0005), so results are no
longer bit-comparable with runs from before this split. This is expected and
was accepted going in.

A sensor that misreports its own accuracy is now representable and will
mislead the estimator exactly as a real miscalibrated sensor would -- a
realistic failure mode that the previous shared-parameter design could not
express.
