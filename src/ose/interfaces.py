"""
Interface definitions. Contracts only -- no implementations.

Every component depends on this module; no component depends on another
component. Adding a field to a published record is backward compatible;
removing or renaming one is not.

Interface names
---------------
A port is typed by an interface named `family.name.vN`, and that name lives on
the record as an `INTERFACE` class variable -- here and nowhere else. It
travels with the object, so a consumer holding a record can ask what it is
(`type(record).INTERFACE`) without importing a lookup table.

They used to live only in docstrings and in a hand-written table in
docs/interfaces/README.md, which drifted three ways before anything could
check it: `vehicle.mass.v1` had a prose section and no table row,
FuelMeasurement had no name at all, and docs/40-composition-spec.md carried a
second copy stating the opposite direction for `vehicle.state.v1`. See ADR
0020.

`NOT_A_PORT` below is the other half. A record with no INTERFACE is
indistinguishable from one that deliberately is not a port, so the negative is
declared too, and `catalogue()` raises on any record in neither set. Without
that, a new port record added to this module would be exactly as invisible as
the three above.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass
from typing import ClassVar, Protocol, runtime_checkable

import numpy as np

from ose.equipment.vehicle import (
    Capability,
    Disturbance,
    Saturation,
    VehicleCommand,
    VehicleState,
)


@dataclass
class OwnStateEstimate:
    """The platform's belief about its own state.

    Expressed in the same coordinates as VehicleState so that guidance and
    planning need not know which navigation implementation produced it. The
    covariance refers to [p_x, p_y, psi, v_air].
    """

    INTERFACE: ClassVar[str] = "vehicle.state.v1"

    t_s: float
    p_x_m: float
    p_y_m: float
    psi_rad: float
    v_air_mps: float

    ground_velocity_mps: np.ndarray          # [north, east]
    wind_estimate_mps: np.ndarray            # [north, east]
    covariance: np.ndarray                   # 4x4, over [p_x, p_y, psi, v_air]
    gnss_available: bool = True

    @property
    def position_sigma_m(self) -> float:
        return math.sqrt(max(self.covariance[0, 0] + self.covariance[1, 1], 0.0))


@dataclass
class TimeEstimate:
    """The platform's belief about its own clock.

    platform_time_s is the running total of the clock's own (corrupted)
    readings -- there is no correction source yet to pull it toward true
    elapsed time, so it is reported as-is. The covariance is the actual
    product of this component today: a calibrated, honestly growing bound
    on how far platform_time_s may have diverged from true elapsed time.
    """

    INTERFACE: ClassVar[str] = "platform.time.v1"

    t_s: float
    platform_time_s: float
    drift_rate: float                # estimated fractional frequency offset
    covariance: np.ndarray           # 2x2, over [offset_s, drift_rate]

    @property
    def offset_sigma_s(self) -> float:
        return math.sqrt(max(self.covariance[0, 0], 0.0))


@dataclass(frozen=True)
class PlatformBeliefs:
    """What the platform believes about itself that navigation does not
    estimate.

    The vehicle manager's product, and the argument a vehicle model needs to
    dress an OwnStateEstimate as one of its own states. Navigation supplies
    position, heading and airspeed; everything else in a state vector comes
    from here.

    A model reads the fields it has and ignores the rest -- the baseline never
    looks at `thermal`, the two-mode model does. That is cheap in a way a
    shared state vector would not be: an unused belief costs nothing, whereas
    an unused STATE would sit in the baseline's Jacobian and integrator with
    nothing maintaining it. Which is why states are per-model and beliefs are
    not.

    Grows by adding fields as the platform learns to believe more things.
    """

    mass_kg: float
    thermal: float = 0.0
    mode: object | None = None      # the model's own mode type, or None


@dataclass(frozen=True)
class MassEstimate:
    """The platform's belief about its own mass -- vehicle.mass.v1.

    Published by the vehicle manager, which is the only component entitled to
    say what the platform weighs. Everything above it consumes this rather
    than being handed a true mass, which is what ADR 0011 recorded as the
    outstanding simplification in guidance.

    Broken out by contribution rather than published as one number, because
    the contributions differ in kind and a consumer may care which is which:
    dry mass is a design constant, payload is a configuration decision, and
    only fuel is measured and therefore uncertain. Summing them here and
    publishing the parts as well costs nothing and keeps the reasoning
    visible.

    tsfc_error is the filter's estimate of the fractional error in the burn
    coefficient it predicts with -- the mass analogue of a clock's drift rate,
    and carried for the same reason (ADR 0010): a miscalibrated coefficient is
    a bias, so it has to be estimated rather than absorbed into process noise.

    mass_sigma_kg is a property rather than a field, derived from the
    covariance. Dry mass and payload are exact, so the uncertainty in the mass
    IS the uncertainty in the fuel; deriving it makes that structurally true
    instead of a comment two fields could drift apart from.
    """

    INTERFACE: ClassVar[str] = "vehicle.mass.v1"

    t_s: float
    mass_kg: float                   # dry + payload + fuel
    dry_mass_kg: float               # design constant, exact
    payload_mass_kg: float           # configuration, exact
    fuel_mass_kg: float              # estimated, uncertain
    tsfc_error: float                # fractional error on the burn coefficient
    covariance: np.ndarray           # 2x2, over [fuel_kg, tsfc_error]

    @property
    def mass_sigma_kg(self) -> float:
        return math.sqrt(max(self.covariance[0, 0], 0.0))


# ---------------------------------------------------------------------------
# Measurement records
#
# Every record carries valid_time_s -- the time the measurement refers to,
# not the time it was delivered -- and its own declared uncertainty. The
# estimator uses the sigma travelling with the measurement, never a
# separately configured value. See ADR 0009.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ImuMeasurement:
    INTERFACE: ClassVar[str] = "sensing.imu.v1"

    valid_time_s: float
    interval_s: float                       # interval this sample is held over
    specific_force_body_mps2: np.ndarray    # [x forward, y right]
    angular_rate_rad_s: float
    specific_force_sigma_mps2: float        # declared, per axis
    angular_rate_sigma_rad_s: float         # declared


@dataclass(frozen=True)
class GnssFix:
    INTERFACE: ClassVar[str] = "sensing.gnss.v1"

    valid_time_s: float
    position_m: np.ndarray                  # [north, east]
    position_sigma_m: float
    velocity_mps: np.ndarray | None         # [north, east], None if not provided
    velocity_sigma_mps: float | None


@dataclass(frozen=True)
class AirDataMeasurement:
    INTERFACE: ClassVar[str] = "sensing.airdata.v1"

    valid_time_s: float
    airspeed_mps: float
    airspeed_sigma_mps: float


@dataclass(frozen=True)
class ClockMeasurement:
    """The platform clock's own reading of elapsed time, not true elapsed
    time. There is deliberately no true-interval field here, unlike
    ImuMeasurement.interval_s: for every other sensor the interval is
    sampling metadata alongside separately-corrupted quantities, but for a
    clock, elapsed time IS the corrupted quantity -- publishing the true
    interval here would leak exactly the truth this component exists to
    hide."""

    INTERFACE: ClassVar[str] = "sensing.clock.v1"

    valid_time_s: float
    elapsed_s: float          # the platform clock's own reading of elapsed time
    elapsed_sigma_s: float    # declared; covers the white-noise term only


@dataclass(frozen=True)
class FuelMeasurement:
    """A direct reading of the mass a platform carries above its dry mass --
    not an integrated one; unlike Imu/Clock there is no drift term here, just
    additive white noise.

    `fuel_remaining_kg` is fuel only on a clean aircraft. A gauge cannot know
    what a platform is carrying, so this is mass above dry, and a consumer
    that decomposes mass as dry + payload + fuel must subtract the payload it
    believes in before treating this as fuel. VehicleManager does; correcting
    on the raw reading double-counted the payload. See ADR 0026.

    The field name predates the distinction and is kept, because renaming a
    published field is not backward compatible and would cost a version
    increment for a clarification the docstring can carry.
    """

    # Minted with the registry (ADR 0020). The other four sensing interfaces
    # had names in docs/interfaces/README.md and this one did not, which is
    # the drift the registry exists to make impossible rather than a
    # deliberate omission.
    INTERFACE: ClassVar[str] = "sensing.fuel.v1"

    valid_time_s: float
    fuel_remaining_kg: float
    fuel_remaining_sigma_kg: float    # declared


# ---------------------------------------------------------------------------
# Guidance setpoints
#
# What travels in ActionSet.motion, below: the single-ship action planner
# decides one of these and vehicle guidance flies it. VehicleGuidance.command()
# dispatches on setpoint type, the same reason ingest() does, so a further
# mode can be added as a new type without changing the protocol.
#
# There is deliberately no waypoint setpoint. The planner decides WHERE to go
# and guidance decides HOW to fly there, so a route is converted to a bearing
# a layer up, where the route is known -- see single_ship/action_planner.py.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class HeadingSpeedSetpoint:
    """Hold a heading and a speed.

    psi_rate_cmd_rad_s is how fast the commanded heading is itself moving,
    and exists so guidance can feed it forward. A proportional law chasing a
    ramp settles at an error of rate/gain, not at zero -- commanding a
    20 deg/s sweep at a gain of 0.3 leaves the vehicle 67 degrees behind the
    setpoint indefinitely. Guidance cannot recover the rate by
    differentiating psi_cmd_rad, both because it is memoryless by design and
    because that signal steps discontinuously whenever the commander changes
    its mind. So whoever builds the setpoint, which knows the rate exactly,
    states it. Zero for a stationary command, which is the default and the
    common case.
    """

    INTERFACE: ClassVar[str] = "guidance.setpoint.v1"

    psi_cmd_rad: float
    v_cmd_mps: float
    psi_rate_cmd_rad_s: float = 0.0


@dataclass(frozen=True)
class TurnRateSpeedSetpoint:
    """Turn at a rate and hold a speed, with no heading to aim at.

    Exists because a heading setpoint cannot express "turn as hard as you
    can". Ask for a rate above what the airframe can deliver and a heading
    command laps the vehicle: the error passes through 180 degrees, changes
    sign, and guidance obligingly reverses the turn. There is no error to
    wrap here, so a deliberately unreachable rate simply saturates and stays
    saturated, and the clipping is reported as usual.

    That makes this the natural way to fly the envelope -- maximum-rate
    turns, corner-speed sweeps -- and the wrong way to hold a bearing, since
    nothing corrects the heading that results. The two setpoint types are
    complements, and the dispatch in VehicleGuidance.command() is what ADR
    0011 put there to allow it.
    """

    # The same name as HeadingSpeedSetpoint: they are two forms of one
    # interface, and a consumer binds the interface and dispatches on the
    # form. So the catalogue maps a name to a SET of records, not to one.
    INTERFACE: ClassVar[str] = "guidance.setpoint.v1"

    omega_cmd_rad_s: float
    v_cmd_mps: float


# ---------------------------------------------------------------------------
# Committed actions -- planning.action.v1
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class ActionSet:
    """What a single-ship action planner commits to, this cycle.

    A bundle with one field per subsystem rather than a bare motion setpoint,
    because docs/40-composition-spec.md binds one planner's `action_out` to
    several subsystems at once -- vehicle, effector and sensor in its worked
    example. Today only `motion` exists, since no other subsystem does; the
    others arrive as new fields, which is backward compatible, rather than as
    a change to this record's type, which would not be.

    That shape is the cheap insurance for a planner that eventually decides
    motion, sensing, communication and effect *together*. Joint planning is
    not merely more fields -- it means reasoning about couplings, such as
    flying a path that keeps a target inside a sensor's field of regard --
    but a planner that does it can publish through this record unchanged,
    and consumers that read only their own field never notice the difference.

    A field set to None means "no new action, continue as before", NOT
    "stop". A planner with nothing new to say about motion leaves the vehicle
    doing what it was already doing. Saying "stop" is an action in its own
    right and has to be expressed as one -- a zero-rate setpoint, say -- not
    by omission, because omission is what silence looks like and silence has
    to be safe.

    `merged_onto` is that rule made executable, and lives here rather than in
    a consumer because it is per-FIELD, not per-consumer. Once this record
    carries sensing and effect alongside motion, a vehicle system would latch
    motion, a sensor system would latch sensing, and each would reimplement
    the identical rule slightly differently. The record that defines the
    semantics is the one place that only has to get them right once.
    """

    INTERFACE: ClassVar[str] = "planning.action.v1"

    t_s: float
    motion: "HeadingSpeedSetpoint | TurnRateSpeedSetpoint | None" = None
    # The propulsion mode being asked for, in whatever type the vehicle model
    # defines. A sibling of motion rather than part of the setpoint, because
    # the two have different lifetimes: a heading may be held for a minute
    # while boost runs for ten seconds.
    #
    # Deciding it is a planning act, not a consequence of thrust demand.
    # Engaging boost automatically whenever guidance asked for more than
    # nominal thrust would let a speed-hold loop quietly spend a thermal
    # budget and a fuel reserve that are finite and contested -- and the
    # planner would find boost unavailable exactly when it had been saving
    # it. Guidance never sees this field.
    propulsion: object | None = None

    def merged_onto(self, previous: "ActionSet") -> "ActionSet":
        """This cycle's action, with absent fields carried over from the last
        committed one.

        The consumer still holds the state -- one ActionSet, however many
        fields it grows -- because being partway through a plan is a property
        of the thing executing it, not of a record describing one instant.

        Every field except t_s must appear here.
        test_every_action_field_participates_in_the_merge walks the dataclass
        and fails if one does not, because a field added to the record and
        forgotten here would silently reset to None every cycle: a subsystem
        losing its last commanded action with nothing raising.
        """
        return ActionSet(
            t_s=self.t_s,
            motion=self.motion if self.motion is not None else previous.motion,
            propulsion=(
                self.propulsion if self.propulsion is not None
                else previous.propulsion
            ),
        )


@runtime_checkable
class ActionPlanner(Protocol):
    """Single-ship layer. Decides what the platform should do next.

    `capability` is what the planner is commanding, asked what it can
    currently achieve, so the planner reasons against the real envelope
    instead of reimplementing the dynamics. It is a single capability today
    because there is a single subsystem to command; a joint planner will need
    several, and that is a protocol change to make when it happens rather
    than a bundle to invent now.
    """

    def plan(
        self,
        t_s: float,
        own_state: OwnStateEstimate,
        capability: GuidanceCapability,
    ) -> ActionSet: ...


# ---------------------------------------------------------------------------
# Sensor and estimator protocols
# ---------------------------------------------------------------------------

@runtime_checkable
class InertialSensor(Protocol):
    def sample(
        self,
        t_s: float,
        dt_s: float,
        true_state: VehicleState,
        true_command: VehicleCommand,
        true_disturbance: Disturbance,
    ) -> ImuMeasurement: ...


@runtime_checkable
class PositioningSensor(Protocol):
    def sample(
        self, t_s: float, true_state: VehicleState, true_disturbance: Disturbance
    ) -> GnssFix | None: ...      # None when denied


@runtime_checkable
class AirDataSensor(Protocol):
    def sample(self, t_s: float, true_state: VehicleState) -> AirDataMeasurement: ...


@runtime_checkable
class ClockSensor(Protocol):
    def sample(self, t_s: float, dt_s: float) -> ClockMeasurement: ...


@runtime_checkable
class FuelSensor(Protocol):
    def sample(self, t_s: float, true_state: VehicleState) -> FuelMeasurement: ...


# ---------------------------------------------------------------------------
# Capability
#
# Every component answers "what can I currently achieve?" without being
# simulated forward. This is what makes composition-time validation possible
# and what a planner queries instead of reimplementing a component's
# internals -- see docs/10-concepts.md and docs/40-composition-spec.md sec 4.1.
#
# The return type is deliberately not fixed here. Per that spec the envelope's
# structure varies by category (vehicle, sensor, communicator, effector) and
# the binder treats it as opaque; forcing one record shape onto all four would
# either bloat it with inapplicable fields or flatten away what each category
# actually needs to declare. The protocol fixes the question, not the answer.
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class MeasurementChannel:
    """One measurable quantity a sensor declares an accuracy for.

    Sensors are routinely multi-channel with different units per channel --
    GNSS declares metres of position and metres per second of velocity --
    so an accuracy claim cannot be a single number without silently
    dropping part of it. `units` is carried explicitly rather than encoded
    in a field name, the one place this repository cannot use its usual
    `position_sigma_m` convention: the field name is generic by design so
    a consumer can iterate channels it was not written against.
    """

    name: str       # e.g. "position", "angular_rate"
    sigma: float    # declared 1-sigma accuracy, in `units`
    units: str      # e.g. "m", "rad/s", "rad/s/sqrt(Hz)"


@dataclass(frozen=True)
class SensorCapability:
    """What a sensing component can currently achieve: how often, how well
    on each channel, and whether at all right now.

    `rate_hz` is None for a sensor that does not own its rate -- Imu and
    Clock are sampled at whatever interval the caller chooses, so claiming
    a rate for them would be an invention. Such sensors also declare noise
    *densities* rather than per-sample sigmas, since their per-sample
    accuracy is only defined once an interval is known; the channel's
    `units` says which is being reported.
    """

    rate_hz: float | None
    channels: tuple[MeasurementChannel, ...]
    available: bool             # False when denied, failed, or otherwise mute

    @property
    def interval_s(self) -> float | None:
        """None when the sensor does not own its rate."""
        if self.rate_hz is None:
            return None
        return 1.0 / self.rate_hz if self.rate_hz > 0.0 else math.inf

    def channel(self, name: str) -> MeasurementChannel:
        for c in self.channels:
            if c.name == name:
                return c
        raise KeyError(
            f"no channel {name!r}; declared channels are "
            f"{[c.name for c in self.channels]}"
        )


@dataclass(frozen=True)
class GuidanceCapability:
    """What a guidance law can currently achieve.

    Composed from two layers, because a control loop is bounded by both
    the vehicle it commands and the navigation it steers on, and the two
    bound different things:

    The vehicle bounds which setpoints are *reachable at all* -- a speed
    below stall or above the airframe limit cannot be held by any control
    law, however good.

    Navigation bounds how *tightly* a reachable setpoint can be held.
    Guidance drives the believed state to the setpoint, so at steady state
    the true error is the navigation error, one for one: a heading-hold
    loop steering on an estimate with a one-degree sigma holds true heading
    to one degree, no better, no matter what its gains are.

    The hold sigmas are floors, not guarantees. They say the loop cannot do
    better than this once settled; during a transient, or while the command
    is saturated, the actual error is larger. A consumer wanting "will it be
    within X right now" must look at the tracking error, not at this.
    """

    # Reachable setpoints -- from the vehicle's own capability.
    max_turn_rate_rad_s: float
    min_speed_mps: float
    max_speed_mps: float

    # Hold accuracy -- floored by the navigation estimate being steered on.
    heading_hold_sigma_rad: float
    speed_hold_sigma_mps: float

    # How many sigma of mass uncertainty the reachable-setpoint bounds above
    # were widened by before being reported. Zero means they are the point
    # estimate. Carried so that a promised envelope is distinguishable from a
    # best guess by the consumer rather than only by whoever configured the
    # platform -- the numbers coincide once a filter has converged, and a
    # planner should not have to guess which it is holding.
    mass_margin_sigma: float = 0.0

    def admits(self, setpoint: HeadingSpeedSetpoint) -> bool:
        """Whether the commanded speed is one the vehicle can hold at all.

        Any heading is reachable given time -- max_turn_rate_rad_s says how
        long, not whether -- so only speed can make a HeadingSpeedSetpoint
        unreachable.
        """
        return self.min_speed_mps <= setpoint.v_cmd_mps <= self.max_speed_mps


@dataclass(frozen=True)
class PromisedEnvelope:
    """What a platform is willing to promise, given how well it knows its own
    mass -- published by VehicleManager.capability_bound(). See ADR 0016.

    A deliberately narrower record than the vehicle's own `Capability`, and
    the narrowing is the entire point. The margin is applied by evaluating the
    vehicle at `mass + k*sigma`, which is conservative only for channels where
    heavier is worse. Three groups of `Capability` fields are therefore absent
    rather than merely unset:

      fuel_mass_kg, endurance_s   ANTI-conservative. Mass uncertainty here is
                                  fuel uncertainty, and fuel enters twice with
                                  opposite senses: a heavier aircraft
                                  manoeuvres worse but is carrying MORE fuel,
                                  so adding mass reports a longer endurance
                                  than the point estimate. Publishing that as
                                  a bound would invite a consumer to plan a
                                  mission it cannot fly, which is the worst
                                  possible direction to be wrong in.

      accel_max_mps2,             Non-monotone in mass -- they move both ways
      accel_min_mps2              across the speed range, because mass enters
                                  both the induced drag and the division by m.
                                  No single signed margin is conservative.

      thrust_required_N,          Not capabilities. A required thrust is an
      v_corner_mps                input to a control law, which must use the
                                  point estimate, and a characteristic speed
                                  is neither better nor worse when it moves.

    An earlier version returned the vehicle's full `Capability` from
    `capability_bound()`. Every field was a true statement about the vehicle
    at the margined mass, so nothing was fabricated -- but `endurance_s` was
    674 seconds longer than the point estimate while wearing the name of a
    bound. Truthful and dangerously misleading are not exclusive.

    mass_kg and mass_margin_sigma travel with the record so a consumer can see
    what it was evaluated at instead of inferring it from configuration.
    """

    # Manoeuvre limits. All conservative under added mass: a heavier aircraft
    # turns no faster, pulls no more g, needs more room, and stalls no slower.
    max_turn_rate_rad_s: float
    sustained_turn_rate_rad_s: float
    min_turn_radius_m: float
    load_factor_available: float
    min_speed_mps: float
    max_speed_mps: float

    # Provenance.
    mass_kg: float                   # the mass this was evaluated at
    mass_margin_sigma: float         # how many sigma above the believed mass


class Vehicle(Protocol):
    """A planar vehicle model, as its consumers use it.

    A platform carries one vehicle. PlanarPointMass and
    PlanarPointMassWithBooster are alternatives, not collaborators, and this is
    the port a consumer binds so that which of them is composed is a
    configuration decision rather than a code one.

    It exists because the annotations lied. `Imu.__init__` and
    `VehicleManager.__init__` both said `PlanarPointMass` while the code was
    duck-typed all along -- tests/test_vehicle_manager.py builds a manager on
    reference_boosted_fighter(), a PlanarPointMassWithBooster, and always has.
    The generated architecture diagram drew the annotation, showed one model
    connected and the other stranded, and that is how the mismatch surfaced.
    See ADR 0023.

    The members are what the two consumers actually use, no more: Imu reads
    drag_N, VehicleManager uses the other four. A model offers far more --
    derivative, admissible, the aerodynamic queries -- and none of it belongs
    here, because a port states what a consumer may rely on rather than what an
    implementation happens to have.

    NOT runtime_checkable, deliberately, and it could not be: dry_mass_kg is a
    property, and `issubclass` against a protocol with a non-method member
    raises `Protocols with non-method members don't support issubclass()`.
    Dropping it to keep issubclass working would mean a port that cannot state
    something its consumer reads, which is the wrong way round. Conformance is
    checked structurally instead -- see tests/test_capability.py and
    tools/generate_architecture_diagram.py.
    """

    @property
    def dry_mass_kg(self) -> float: ...

    def drag_N(self, v_mps: float, mass_kg: float, omega_rad_s: float) -> float: ...

    def capability(self, state, omega_rad_s: float = 0.0) -> Capability: ...

    def project_command(
        self, state, command: VehicleCommand
    ) -> tuple[VehicleCommand, Saturation]: ...

    def state_from(self, estimate, beliefs): ...


@runtime_checkable
class CapabilityModel(Protocol):
    """Answerable without integrating anything forward.

    Deliberately loose in its arguments: a vehicle's capability depends on
    its state, a sensor's does not, and a future effector's will depend on
    engagement geometry. Implementations name their own parameters; what
    the protocol pins down is that every component can be asked.
    """

    def capability(self, *args, **kwargs): ...


@runtime_checkable
class NavigationEstimator(Protocol):
    """`ingest` dispatches on measurement type: an ImuMeasurement drives
    prediction, the others drive correction. Unknown types raise TypeError.
    """

    def ingest(self, measurement) -> None: ...
    def estimate(self, t_s: float) -> OwnStateEstimate: ...


@runtime_checkable
class OwnStateSource(Protocol):
    """Anything publishing vehicle.state.v1, whatever layer it sits in."""

    def estimate(self, t_s: float) -> OwnStateEstimate: ...


@runtime_checkable
class TimeEstimator(Protocol):
    """`ingest` dispatches on measurement type, mirroring NavigationEstimator,
    so a future correction source (a second clock, a time-sync message) can
    be added without changing the protocol. Unknown types raise TypeError.
    """

    def ingest(self, measurement) -> None: ...
    def estimate(self, t_s: float) -> TimeEstimate: ...


@runtime_checkable
class VehicleGuidance(Protocol):
    """`command` dispatches on setpoint type -- HeadingSpeedSetpoint or
    TurnRateSpeedSetpoint -- the same reasoning as `ingest`. Unknown setpoint
    types raise TypeError.

    There is no mass parameter. An implementation binds to a vehicle manager,
    which owns the platform's believed mass and answers vehicle questions at
    it, so a caller has nothing to supply and no reason to reach for the true
    value. See ADR 0015.
    """

    def command(
        self, t_s: float, setpoint, own_state: OwnStateEstimate
    ) -> tuple[VehicleCommand, Saturation]: ...


# ---------------------------------------------------------------------------
# The catalogue
# ---------------------------------------------------------------------------

# Records visible from this module that deliberately carry no interface name,
# with the reason. This is not documentation: catalogue() raises on any record
# that is neither registered nor listed here, so the negative declaration is
# what makes a forgotten registration an error rather than a silence.
NOT_A_PORT: dict[type, str] = {
    PlatformBeliefs: (
        "an argument a vehicle model needs to dress an estimate as a state, "
        "not something published on a wire"
    ),
    MeasurementChannel: "a field of SensorCapability, never sent alone",
    SensorCapability: (
        "capability is a query surface, not a port -- a consumer asks a bound "
        "component what it can do; it does not subscribe to it (ADR 0012)"
    ),
    Capability: (
        "the vehicle's own query surface, the answer to capability() rather "
        "than something published (ADR 0012). Imported here only so the "
        "Vehicle protocol can name it"
    ),
    GuidanceCapability: "as SensorCapability, composed across two layers (ADR 0013)",
    PromisedEnvelope: "as SensorCapability, widened by the mass margin (ADR 0016)",
    VehicleState: (
        "ground truth. truth.query.v1 is planned and unimplemented, and only "
        "the equipment layer may ever hold this (ADR 0008)"
    ),
    Disturbance: "ground truth, as VehicleState",
}


def catalogue() -> dict[str, tuple[type, ...]]:
    """Interface name -> the records carrying it, discovered not listed.

    A name maps to several records when one interface has several forms:
    `guidance.setpoint.v1` covers both setpoint types, and a consumer binds
    the interface and dispatches on the form. So this is a name-to-set map,
    and uniqueness is a property of the record-to-name direction instead --
    which the ClassVar gives structurally, since a record has one.

    Raises TypeError naming any record that is neither registered nor in
    NOT_A_PORT. That is the whole point of the pairing: without it, a port
    record added to this module and left unregistered would be invisible,
    which is exactly how FuelMeasurement went unnamed for as long as it did.
    """
    found: dict[str, list[type]] = {}
    unclassified = []
    for name, obj in sorted(globals().items()):
        if not isinstance(obj, type) or not dataclasses.is_dataclass(obj):
            continue
        # __dict__ rather than getattr: an inherited INTERFACE would be a name
        # the subclass never chose. BoostCapability(Capability) is the live
        # example of a record subclass existing at all.
        interface = obj.__dict__.get("INTERFACE")
        if interface is not None:
            found.setdefault(interface, []).append(obj)
        elif obj not in NOT_A_PORT:
            unclassified.append(name)

    if unclassified:
        raise TypeError(
            "record(s) in ose.interfaces are neither given an INTERFACE nor "
            f"declared NOT_A_PORT: {sorted(unclassified)}. Give each an "
            "INTERFACE: ClassVar[str] if it is published on a wire, or add it "
            "to NOT_A_PORT with the reason if it is not."
        )
    return {name: tuple(records) for name, records in sorted(found.items())}
