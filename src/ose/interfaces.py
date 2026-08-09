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

from ose.resource.vehicle import Disturbance, VehicleCommand, VehicleState


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


# ---------------------------------------------------------------------------
# Measurement records
#
# Every record carries valid_time_s -- the time the measurement refers to,
# not the time it was delivered -- and its own declared uncertainty. The
# estimator uses the sigma travelling with the measurement, never a
# separately configured value. See docs/refactor-navigation-split.md.
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
