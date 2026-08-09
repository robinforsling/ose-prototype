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
    psi_cmd_rad: float
    v_cmd_mps: float


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
