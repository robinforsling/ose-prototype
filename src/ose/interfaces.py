"""
Interface definitions. Contracts only -- no implementations.

Every component depends on this module; no component depends on another
component. Adding a field to a published record is backward compatible;
removing or renaming one is not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from ose.resource.vehicle import Disturbance, Saturation, VehicleCommand, VehicleState


@dataclass
class OwnStateEstimate:
    """The platform's belief about its own state.

    Expressed in the same coordinates as VehicleState so that guidance and
    planning need not know which navigation implementation produced it. The
    covariance refers to [p_x, p_y, psi, v_air].
    """

    t_s: float
    p_x_m: float
    p_y_m: float
    psi_rad: float
    v_air_mps: float

    ground_velocity_mps: np.ndarray          # [north, east]
    wind_estimate_mps: np.ndarray            # [north, east]
    covariance: np.ndarray                   # 4x4, over [p_x, p_y, psi, v_air]
    gnss_available: bool = True

    def as_vehicle_state(self, mass_kg: float) -> VehicleState:
        """Believed vehicle state. Mass is not estimated by navigation; it is
        taken from the fuel accounting of the vehicle system."""
        return VehicleState(
            self.p_x_m, self.p_y_m, self.psi_rad, self.v_air_mps, mass_kg
        )

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

    t_s: float
    platform_time_s: float
    drift_rate: float                # estimated fractional frequency offset
    covariance: np.ndarray           # 2x2, over [offset_s, drift_rate]

    @property
    def offset_sigma_s(self) -> float:
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


@dataclass(frozen=True)
class ClockMeasurement:
    """The platform clock's own reading of elapsed time, not true elapsed
    time. There is deliberately no true-interval field here, unlike
    ImuMeasurement.interval_s: for every other sensor the interval is
    sampling metadata alongside separately-corrupted quantities, but for a
    clock, elapsed time IS the corrupted quantity -- publishing the true
    interval here would leak exactly the truth this component exists to
    hide."""

    valid_time_s: float
    elapsed_s: float          # the platform clock's own reading of elapsed time
    elapsed_sigma_s: float    # declared; covers the white-noise term only


@dataclass(frozen=True)
class FuelMeasurement:
    """A direct reading of remaining fuel mass, not an integrated one --
    unlike Imu/Clock there is no drift term here, just additive white
    noise."""

    valid_time_s: float
    fuel_remaining_kg: float
    fuel_remaining_sigma_kg: float    # declared


# ---------------------------------------------------------------------------
# Guidance setpoints
#
# Stand in for planning.action.v1 until the single-ship layer's action
# planner exists. VehicleGuidance.command() dispatches on setpoint type, the
# same reason ingest() does: a further mode (e.g. waypoint pursuit) can be
# added as a new type without changing the protocol.
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
    """

    t_s: float
    motion: "HeadingSpeedSetpoint | TurnRateSpeedSetpoint | None" = None


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
    """What a sensing resource can currently achieve: how often, how well
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

    def admits(self, setpoint: HeadingSpeedSetpoint) -> bool:
        """Whether the commanded speed is one the vehicle can hold at all.

        Any heading is reachable given time -- max_turn_rate_rad_s says how
        long, not whether -- so only speed can make a HeadingSpeedSetpoint
        unreachable.
        """
        return self.min_speed_mps <= setpoint.v_cmd_mps <= self.max_speed_mps


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
    """`command` dispatches on setpoint type -- today only
    HeadingSpeedSetpoint, later also a waypoint mode -- the same reasoning
    as `ingest`. Unknown setpoint types raise TypeError.

    mass_kg is a plain parameter, not sourced from own_state: no component
    estimates mass yet (no vehicle-system/fuel-accounting component exists),
    so this is today's acknowledged simplification rather than truth read
    through the back door.
    """

    def command(
        self, t_s: float, setpoint, own_state: OwnStateEstimate, mass_kg: float
    ) -> tuple[VehicleCommand, Saturation]: ...
