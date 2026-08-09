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


@runtime_checkable
class NavigationSystem(Protocol):
    """Interface every navigation implementation satisfies."""

    def update(
        self,
        t_s: float,
        dt_s: float,
        true_state: VehicleState,
        true_command: VehicleCommand,
        true_disturbance: Disturbance,
    ) -> OwnStateEstimate:
        """Advance one step and return the current estimate.

        The true arguments are supplied by the simulation core. An
        implementation may use them only to generate corrupted measurements;
        passing truth through unmodified would defeat the purpose of the
        component.
        """
        ...


