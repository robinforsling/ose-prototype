# Interface catalogue

Status: draft

Ports are typed by interface, named `family.name.vN`. Two components bind only
if the interface names match and the major versions are equal.

Adding a field to a published record is backward compatible. Removing or
renaming one is not, and requires a version increment.

## Catalogue

| Interface | Direction | Carries | Status |
|---|---|---|---|
| `truth.query.v1` | core to resource | Privileged read of ground-truth state. Grantable only to `layer: resource`. | planned |
| `power.bus.v1` | vehicle to resource | Abstract power draw negotiation, electrical and cooling combined. | planned |
| `vehicle.command.v1` | subsystem to resource | Commanded thrust and turn rate. | **implemented** |
| `vehicle.state.v1` | resource or subsystem to above | Own-ship state as the platform believes it, with covariance. | **implemented** |
| `sensing.imu.v1` | resource to subsystem | Specific force and angular rate, with declared uncertainty. | **implemented** |
| `sensing.gnss.v1` | resource to subsystem | Position and optional velocity fix, with declared uncertainty. | **implemented** |
| `sensing.airdata.v1` | resource to subsystem | Airspeed, with declared uncertainty. | **implemented** |
| `sensing.clock.v1` | resource to subsystem | The platform clock's own elapsed-time reading, with declared uncertainty. | **implemented** |
| `platform.time.v1` | subsystem to above | Platform's belief about its own clock: accumulated reading, drift, covariance. | **implemented** |
| `sensing.detections.v1` | resource to subsystem | Time-stamped detections with measurement uncertainty. | planned |
| `sensing.control.v1` | subsystem to resource | Sensor tasking: pointing, mode, priority. | planned |
| `comms.message.v1` | bidirectional | Addressed transport with loss and latency applied. | planned |
| `effect.request.v1` | subsystem to resource | Employment request against a designated track. | planned |
| `effect.status.v1` | resource to subsystem | Inventory, readiness, in-flight effector state. | planned |
| `tracking.tracks.v1` | subsystem to single-ship | Fused track picture. | planned |
| `sa.picture.v1` | within single-ship | Assessed situation, threat evaluation. | planned |
| `planning.action.v1` | single-ship to subsystem | Committed actions for execution. | planned |
| `coord.intent.v1` | multi-ship to single-ship | Assigned role, tasking, constraints. | planned |

## Implemented interfaces

### `vehicle.command.v1`

`VehicleCommand(thrust_N, omega_rad_s)`. Thrust in newtons, turn rate in radians
per second, positive right.

The vehicle declares admissible sets but does not enforce them. A command
outside `U(x, lambda)` is integrated as given. See ADR 0006.

### `vehicle.state.v1`

`OwnStateEstimate`, published by any component satisfying the
`OwnStateSource` protocol (`estimate(t_s) -> OwnStateEstimate`). Carries
position, heading, airspeed, ground velocity, wind estimate, a 4x4 covariance
over `[p_x, p_y, psi, v_air]`, and a GNSS availability flag.

Two kinds of publisher exist, and a consumer cannot tell which produced a
given estimate from its shape alone:

- `InsGnssEstimator` (subsystem layer), which additionally satisfies
  `NavigationEstimator` -- it is fed a measurement stream via `ingest()`
  rather than reading truth, and is a pure, replayable function of that
  stream. See ADR 0009.
- `IntegratedNavUnit` (resource layer), a deliberate collapse of the
  resource and subsystem layers into one black-box component that reads
  truth directly. Valid scaffolding when navigation is not the component
  under test; not a baseline for any claim about navigation performance.

The covariance is part of the contract, not an optional extra. A consumer that
ignores it is choosing to, and a producer that supplies an inconsistent one
corrupts everything downstream. `tests/test_navigation_estimator.py` checks
consistency by NEES for exactly this reason.

### `sensing.imu.v1`

`ImuMeasurement`, published by `Imu.sample()` at whatever rate it is called
(no rate limiting of its own). Carries `specific_force_body_mps2` (`[x
forward, y right]`, m/s^2) and `angular_rate_rad_s` (positive right), each
with its own declared sigma, plus `valid_time_s` and `interval_s` -- the
duration the sample is held to represent, used by the consumer to construct
process noise rather than any separately configured density.

### `sensing.gnss.v1`

`GnssFix`, published by `GnssReceiver.sample()`, or `None` if denied (see
`set_gnss_available`) or not yet due (see `due()`, at `gnss_rate_hz`).
Carries `position_m` (`[north, east]`, m) with its declared
`position_sigma_m`, and optionally `velocity_mps` with `velocity_sigma_mps`
(`None` for both if velocity aiding is disabled on this receiver).

### `sensing.airdata.v1`

`AirDataMeasurement`, published by `AirDataSensor.sample()` (no denial
concept; rate-limited externally via `due()`, at `air_data_rate_hz`). Carries
scalar `airspeed_mps` with its declared `airspeed_sigma_mps`.

### `sensing.clock.v1`

`ClockMeasurement`, published by `Clock.sample()` at whatever rate it is
called (no rate limiting of its own, like IMU). Carries `elapsed_s` -- the
platform clock's own reading of how much time just passed, corrupted by
drift and white noise -- with its declared `elapsed_sigma_s`. Unlike every
other measurement record, this one has no true-interval field: for a clock,
elapsed time *is* the corrupted quantity, so publishing the true interval
alongside it would leak exactly the truth this component exists to hide.
See ADR 0010.

### `platform.time.v1`

`TimeEstimate`, published by any component satisfying `TimeEstimator`
(`ingest`/`estimate`, mirroring `NavigationEstimator`). Today that is only
`TimeEstimator` (subsystem layer, `subsystem/time_state_estimator.py`),
dead-reckoning `ClockMeasurement` readings with no correction source. Carries
`platform_time_s` (the running, unfiltered sum of the clock's own readings),
`drift_rate`, and a 2x2 covariance over `[offset_s, drift_rate]`.

`platform_time_s` is exactly what the clock has read -- there is nothing to
correct it with yet. The covariance is the actual product of this component:
a calibrated, honestly growing bound on how far it may have diverged from
true elapsed time. It never shrinks, by construction, until a correction
source exists. See ADR 0010.

## What every interface file must state

For each new interface, record: what it carries, at what rate, in what frame and
units, and what either side may assume about the other. Frames and units are the
classic integration killer; time semantics is the classic modularity killer.
