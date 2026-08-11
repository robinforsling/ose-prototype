# Interface catalogue

Status: draft

Ports are typed by interface, named `family.name.vN`. Two components bind only
if the interface names match and the major versions are equal.

Adding a field to a published record is backward compatible. Removing or
renaming one is not, and requires a version increment.

## Catalogue

| Interface | Direction | Carries | Status |
|---|---|---|---|
| `truth.query.v1` | core to equipment | Privileged read of ground-truth state. Grantable only to `layer: equipment`. | planned |
| `power.bus.v1` | vehicle to equipment | Abstract power draw negotiation, electrical and cooling combined. | planned |
| `vehicle.command.v1` | subsystem to equipment | Commanded thrust and turn rate. | **implemented** |
| `vehicle.state.v1` | subsystem to above | Own-ship state as the platform believes it, with covariance. Published by `NavigationManager`, one per platform. | **implemented** |
| `sensing.imu.v1` | equipment to subsystem | Specific force and angular rate, with declared uncertainty. | **implemented** |
| `sensing.gnss.v1` | equipment to subsystem | Position and optional velocity fix, with declared uncertainty. | **implemented** |
| `sensing.airdata.v1` | equipment to subsystem | Airspeed, with declared uncertainty. | **implemented** |
| `sensing.clock.v1` | equipment to subsystem | The platform clock's own elapsed-time reading, with declared uncertainty. | **implemented** |
| `platform.time.v1` | subsystem to above | Platform's belief about its own clock: accumulated reading, drift, covariance. | **implemented** |
| `guidance.setpoint.v1` | within a platform | Commanded heading and speed, or turn rate and speed. Carried inside `planning.action.v1`'s motion field. | **implemented** |
| `sensing.detections.v1` | equipment to subsystem | Time-stamped detections with measurement uncertainty. | planned |
| `sensing.control.v1` | subsystem to equipment | Sensor tasking: pointing, mode, priority. | planned |
| `comms.message.v1` | bidirectional | Addressed transport with loss and latency applied. | planned |
| `effect.request.v1` | subsystem to equipment | Employment request against a designated track. | planned |
| `effect.status.v1` | equipment to subsystem | Inventory, readiness, in-flight effector state. | planned |
| `tracking.tracks.v1` | subsystem to single-ship | Fused track picture. | planned |
| `sa.picture.v1` | within single-ship | Assessed situation, threat evaluation. | planned |
| `planning.action.v1` | single-ship to subsystem | Committed actions for execution, one field per subsystem. | **implemented** |
| `coord.intent.v1` | multi-ship to single-ship | Assigned role, tasking, constraints. | planned |

## Implemented interfaces

### `vehicle.command.v1`

`VehicleCommand(thrust_N, omega_rad_s)`. Thrust in newtons, turn rate in radians
per second, positive right.

The vehicle declares admissible sets but does not enforce them. A command
outside `U(x, lambda)` is integrated as given. See ADR 0006.

`VehicleGuidance` (subsystem layer) is the first real producer of this
interface: it projects its raw command onto the vehicle's admissible sets
via `project_command()` before publishing it, and reports any clipping via
`Saturation` rather than swallowing it. See ADR 0011.

`Saturation` carries `thrust_clipped`, `omega_clipped`, human-readable
`notes`, and `requested` -- the command as it arrived, before enforcement,
always populated. The numbers matter because `project_command()` returns only
what survived, so without them the pre-enforcement command is unobservable and
a caller wanting to show or log it has to duplicate the control law.

### `vehicle.state.v1`

`OwnStateEstimate`, published by any component satisfying the
`OwnStateSource` protocol (`estimate(t_s) -> OwnStateEstimate`). Carries
position, heading, airspeed, ground velocity, wind estimate, a 4x4 covariance
over `[p_x, p_y, psi, v_air]`, and a GNSS availability flag.

Consumers bind to `NavigationManager` (subsystem layer), the platform's
single publisher of this interface, and to nothing below it. A *navigation
system* is the manager plus whatever produces the estimate underneath:

- `InsGnssEstimator` (subsystem layer), which additionally satisfies
  `NavigationEstimator` -- it is fed a measurement stream via `ingest()`
  rather than reading truth, and is a pure, replayable function of that
  stream. See ADR 0009.
- `IntegratedNavUnit` (equipment layer), a deliberate collapse of the
  equipment and subsystem layers into one black-box component that reads
  truth directly. Valid scaffolding when navigation is not the component
  under test; not a baseline for any claim about navigation performance.

Exactly one of those, never both. The manager does not fuse them: they are
alternatives, and merging a fiction with a model would report an estimate
better than either while looking self-consistent. See ADR 0014.

The covariance is part of the contract, not an optional extra. A consumer that
ignores it is choosing to, and a producer that supplies an inconsistent one
corrupts everything downstream. `tests/test_navigation_state_estimator.py`
checks consistency by NEES for exactly this reason.

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

### `vehicle.mass.v1`

`MassEstimate(t_s, mass_kg, dry_mass_kg, payload_mass_kg, fuel_mass_kg,
tsfc_error, covariance)`, published by `VehicleManager` (subsystem layer,
`subsystem/vehicle_manager.py`) from the `FuelMeasurement` the fuel gauge
publishes. **Implemented.**

The platform's belief about its own mass, and the only sanctioned answer to
"what does this aircraft weigh?" above the equipment layer. Broken out by
contribution because the contributions differ in kind: dry mass is a design
constant and payload is a configuration decision, both exact, while only fuel
is estimated. `mass_sigma_kg` is a property derived from the covariance rather
than a field, since the uncertainty in the mass IS the uncertainty in the fuel.

Fuel is tracked by a two-state filter over `[fuel_kg, tsfc_error]` that
predicts on the commanded thrust and corrects on each measurement, so the
sigma sawtooths -- growing with staleness, dropping at each reading -- rather
than restating the gauge's own figure. `tsfc_error` is the estimated
fractional error in the burn coefficient, carried as a state because a
miscalibrated coefficient is a bias and modelling a bias as process noise
makes a filter overconfident. It is weakly observable by design. The stated
uncertainty is checked by an ensemble ANEES test through the run, two-sided.

The gauge's role changed when the filter arrived, and it is worth being clear
about: it used to *be* the belief, so its declared sigma was the platform's
mass sigma outright and its rate only controlled staleness. It is now a
correction source, so rate and noise trade off -- the same 20 kg reading at
0.05 Hz instead of 1 Hz leaves a belief three times worse. Consistency is
checked at both the reference gauge and a twenty-times-slower, three-times-
noisier one, because swapping a component out is the point of this
environment.

`predict(t_s, thrust_N)` is separate from `project_command()` on purpose:
asking whether a command is admissible must not commit the platform to having
flown it.

`capability_bound()` reports a `PromisedEnvelope` at `mass + margin * sigma`
-- what the platform will promise -- while `capability()` reports the vehicle's
full `Capability` at the believed mass, which is what feedforward and
enforcement must use.

The promise is a narrower record on purpose. Adding mass is conservative for a
manoeuvre limit but not for everything the vehicle reports: `endurance_s` and
`fuel_mass_kg` move the *wrong* way, because mass uncertainty here is fuel
uncertainty and a heavier aircraft carries more of it, and the accelerations
are not monotone in mass at all. Those channels are absent rather than unset.
A table in the tests names the required direction of every field that is
present and fails if one is added without a direction. See ADR 0016.

The manager also answers vehicle questions at that mass -- `capability()`,
including the parametrised turn-rate form guidance feeds thrust forward on,
and `project_command()` -- and is the only component permitted to bind
`PlanarPointMass`. That rule is enforced as an import check; see ADR 0015 for why it
is phrased that way and for the three exemptions.

### `guidance.setpoint.v1`

Two setpoint records, consumed by any component satisfying
`VehicleGuidance`. They began as a stand-in for `planning.action.v1` before
a single-ship layer existed; now that one does, they are what its `motion`
field carries rather than a substitute for it.
(`command(t_s, setpoint, own_state) -> (VehicleCommand, Saturation)`). Today the only implementation is
`VehicleGuidance` (subsystem layer, `subsystem/vehicle_guidance.py`), whose
raw command is projected onto the vehicle's admissible sets before
publication.

`HeadingSpeedSetpoint(psi_cmd_rad, v_cmd_mps, psi_rate_cmd_rad_s=0.0)` holds a
heading and a speed. The rate field is how fast the commanded heading is
itself moving, and it exists because a proportional law chasing a ramp settles
at an error of rate/gain rather than zero -- 67 degrees for a 20 deg/s sweep
at this gain. Guidance is memoryless by design and the signal steps between
commands, so it cannot differentiate the heading itself; the commander, which
knows the rate exactly, declares it.

`TurnRateSpeedSetpoint(omega_cmd_rad_s, v_cmd_mps)` turns at a rate with no
heading to aim at. It exists because a heading command cannot express "turn as
hard as you can": ask for more than the airframe can give and the setpoint
laps the vehicle, the error wraps through 180 degrees and flips sign, and
guidance reverses. With no error to wrap, an unreachable rate simply pins
against the limit. The right tool for flying the envelope, the wrong one for
holding a bearing.

`command()` dispatches on setpoint type, mirroring `NavigationEstimator.
ingest()`: a further mode is a new type and a new branch, not a protocol
change. There is no mass parameter -- guidance binds to a vehicle manager,
which owns the platform's believed mass. See ADR 0015.

### `planning.action.v1`

`ActionSet(t_s, motion=None)`, published by any component satisfying
`ActionPlanner` (`plan(t_s, own_state, capability) -> ActionSet`). Today the
only implementation is `WaypointPlanner` (single-ship layer,
`single_ship/action_planner.py`), which follows a route of waypoints.

A bundle with one field per subsystem, not a bare motion setpoint, because
`docs/40-composition-spec.md` binds one planner's `action_out` to several
subsystems at once. Only `motion` exists today; `sensor`, `effect` and
`comms` arrive as new fields when those subsystems do, which is backward
compatible where changing the record's type would not be. A planner that
eventually decides motion and sensing *together* publishes through this
record unchanged.

**A field set to None means "no new action, continue as before", not
"stop".** A planner with nothing new to say about motion leaves the vehicle
doing what it was already doing. Stopping is an action in its own right and
must be commanded as one, because omission is what silence looks like and
silence has to be safe.

`merged_onto(previous)` is that rule made executable: it returns this
cycle's action with absent fields carried over from the last committed one.
It lives on the record rather than in a consumer because the rule is
per-field, not per-consumer -- a vehicle system would latch `motion`, a
sensor system `sensing`, and each would otherwise reimplement it. The
consumer still holds the state, one `ActionSet` however many fields it
grows.

A test walks the dataclass and fails if any field is missing from the merge,
because a field added to the record and forgotten there would silently reset
to None every cycle: a subsystem losing its last commanded action with
nothing raising.

## Capability, which is not a port

`CapabilityModel`, `SensorCapability` and `MeasurementChannel` live in
`ose.interfaces` alongside the port records, but they are deliberately absent
from the catalogue above. That table describes ports: two components bind when
their interface names match and their major versions are equal. Capability is
not something two components bind over -- it is a query surface every component
exposes, to a planner, to the binder, or to the composition GUI, none of which
are the component's peer.

`CapabilityModel` fixes only that a component can be asked
(`capability(...)`), not what comes back. Envelope structure varies by category
and the binder treats it as opaque; see `docs/40-composition-spec.md` section
4.1 and ADR 0012.

Implemented today by every equipment component, and by one subsystem component:

- `PlanarPointMass.capability(state, omega_rad_s, disturbance)` returns `Capability`,
  a fourteen-field record covering thrust, acceleration bounds, turn performance,
  characteristic speeds, fuel and endurance. It is a function of state, so it
  changes as fuel burns.
- `Imu`, `GnssReceiver`, `AirDataSensor`, `Clock` and `FuelGauge` return
  `SensorCapability`: a `rate_hz` (`None` when the sensor does not own its
  rate), a tuple of `MeasurementChannel(name, sigma, units)` -- one per quantity
  measured -- and an `available` flag that tracks denial.

- `VehicleGuidance.capability(own_state)` returns the *promised* envelope --
  the manager's bound, not its point estimate, with `mass_margin_sigma`
  saying so. It returns
  `GuidanceCapability`, the one capability model here that is *composed*
  rather than reported: reachable setpoints come from the vehicle's
  envelope, hold accuracy from the navigation covariance travelling with
  `own_state`. It carries an `admits(setpoint)` predicate. Swapping in a
  worse IMU widens the hold sigma without a line changing in guidance.
  See ADR 0013.

Accuracy is per channel because sensors are routinely multi-channel with
different units per channel, and a single number silently drops part of the
claim. `Imu` reports noise densities rather than per-sample sigmas, since its
per-sample accuracy is undefined until the caller picks an interval; its
channel `units` say so.

## What every interface file must state

For each new interface, record: what it carries, at what rate, in what frame and
units, and what either side may assume about the other. Frames and units are the
classic integration killer; time semantics is the classic modularity killer.
