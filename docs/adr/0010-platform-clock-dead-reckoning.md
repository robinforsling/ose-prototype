# 0010. Platform clock, estimated by dead reckoning only

Status: accepted
Date: 2026-08-09

## Context

A platform's own sense of elapsed time is not free. An onboard oscillator
drifts, and every timestamp used to align measurements ultimately depends on
that oscillator being roughly right. Until now, `valid_time_s` on every
measurement record has been the simulation's ambient true time -- a
convenience, not a claim that any platform actually has perfect access to it.
Introducing a `Clock` is the first component to model that
this access is imperfect, following the same equipment/subsystem split as
navigation (ADR 0009): a equipment-layer sensor corrupts truth into a
measurement record carrying its own declared uncertainty; a subsystem-layer
estimator consumes that record and never sees truth.

Unlike navigation, there is nothing today for a time estimator to correct
against. `InsGnssEstimator` predicts with the IMU and corrects with GNSS and
air data; GNSS supplies an absolute, truth-referenced fix that bounds the
otherwise unbounded growth of dead-reckoning error. A clock has no analogous
second, independent reference in this simulation -- no second clock to
cross-check against, no external time-sync signal. A single clock's own
readings cannot reveal its own bias or drift; that is a property of clock
error, not a limitation of this implementation.

## Decision

`Clock` (`equipment/clock.py`) corrupts true elapsed time with two error
sources: `drift`, a fractional frequency offset modelled as a first-order
Gauss-Markov process (`drift_sigma`/`drift_tau_s`), the same structure as
`Imu`'s accelerometer and gyro bias; and a fixed per-reading white-noise
term (`white_noise_sigma_s`), which is *not* scaled by the sample interval
the way IMU's noise density is, because a clock reading's jitter is a
property of the read-out mechanism, not a continuous-time process sampled at
rate `1/dt`. It publishes `ClockMeasurement`, which deliberately carries no
true-interval field (unlike `ImuMeasurement.interval_s`): for every other
sensor the interval is sampling metadata alongside separately-corrupted
quantities, but for a clock, elapsed time *is* the corrupted quantity, so
publishing the true interval would leak exactly the truth this component
exists to hide.

`TimeEstimator` (`subsystem/time_state_estimator.py`) is dead reckoning
only. Given no correction source exists, building one that appeared to
correct the estimate -- by any means not traceable to an actual second
reference -- would misrepresent the estimate's basis. So `TimeEstimator`
only ever predicts: it accumulates the clock's own corrupted readings into
`platform_time_s`, unfiltered, and propagates a two-state (`offset`,
`drift_rate`) covariance that honestly grows over time, using its own
assumed drift process model (`TimeEstimatorParameters`) rather than the true
`Clock`'s parameters -- the same decoupling as `EstimatorParameters` versus
`ImuParameters` in ADR 0009. The white-noise contribution instead comes
directly from each measurement's own declared `elapsed_sigma_s`, injected
once per reading rather than through a `*dt`-scaled continuous-time formula,
matching how `Clock` generates it. This is exactly what `InsGnssEstimator`
does during a GNSS outage, except here it is permanent.

`ingest()` dispatches on measurement type, mirroring `NavigationEstimator`,
so a future correction source -- a second clock, a time-sync message -- can
be added as a new `isinstance` branch without changing the protocol. The
concrete class shares its name with the `TimeEstimator` protocol in
`interfaces.py`, the same pattern `AirDataSensor` already uses for its
protocol/implementation pair.

## Consequences

`platform_time_s` is always exactly what the clock has read, never adjusted
-- there is nothing to adjust it with. The value this component adds today
is entirely in the covariance: a calibrated, honestly growing bound on how
far `platform_time_s` may have diverged from true elapsed time, checked by
`test_offset_uncertainty_is_consistent` the same way `test_filter_is_
consistent` checks the navigation filter.

Consumers must not read a shrinking `offset_sigma_s` as a signal of anything
-- it never shrinks, by construction, until a correction source exists.
`OwnStateEstimate.gnss_available`-style outage recovery has no analogue here
yet.

Adding a correction source later is expected to be additive: a new
measurement record in `interfaces.py`, a new `ingest()` branch, and an
actual Kalman update against the existing `offset`/`drift_rate` state and
covariance -- no restructuring of what exists today.
