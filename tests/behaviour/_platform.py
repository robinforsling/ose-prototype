"""One assembled platform, and a loop that flies it.

A behaviour test composes what a demo composes -- vehicle, gauge, vehicle
manager, guidance, and optionally a planner -- and runs it forward. That
assembly is identical across the tests here, so it lives once.

Deliberately NOT imported from demos/. The demos are throwaway prototypes of
the simulation core (docs/50-tooling.md), they take command-line arguments and
open windows, and a test that depended on one would break whenever a
demonstration was re-staged. What is shared with them is the composition, which
is small and worth restating, not the code.

Navigation is a perfect estimate, as in the demos. These tests are about what
the platform DOES; estimation error is the subject of
test_navigation_state_estimator.py, and mixing the two would leave a failure
here ambiguous between a guidance fault and a filter fault -- which is exactly
the distinction the categories exist to keep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ose.equipment.fuel_gauge import FuelGauge
from ose.equipment.reference_configs.reference_fuel_gauge import (
    STANDARD as FUEL_GAUGE_STANDARD,
)
from ose.equipment.reference_configs.vehicle.planar_point_mass import reference_fighter
from ose.equipment.vehicle import Disturbance, VehicleState
from ose.equipment.vehicle.records import NO_DISTURBANCE
from ose.integration import step_rk4
from ose.interfaces import OwnStateEstimate
from ose.single_ship.action_planner import WaypointPlanner
from ose.single_ship.reference_configs.reference_action_planner import (
    STANDARD as PLANNER_STANDARD,
)
from ose.subsystem.reference_configs.reference_vehicle_guidance import (
    STANDARD as GUIDANCE_STANDARD,
)
from ose.subsystem.reference_configs.reference_vehicle_manager import (
    BELIEVED_TSFC_KG_PER_N_S,
    STANDARD as MANAGER_STANDARD,
)
from ose.subsystem.vehicle_guidance import VehicleGuidance
from ose.subsystem.vehicle_manager import VehicleManager

DT = 0.05


def perfect_estimate(
    t_s: float, state: VehicleState, disturbance: Disturbance = NO_DISTURBANCE
) -> OwnStateEstimate:
    """Stands in for navigation, so a failure here is about behaviour.

    The wind has to reach it. Ground velocity is air velocity PLUS wind, and a
    stub that left the wind out would be publishing a ground velocity the
    platform does not have -- which is not a perfect estimate, it is a wrong
    one.

    It went unnoticed until the track loop became the first thing to read the
    field: while ground velocity was published and unconsumed, the omission
    could not affect anything. The first run of the track-hold behaviour tests
    showed the bow unchanged at 1 390 m, because guidance was closing a track
    loop on a track that did not exist.
    """
    air = state.v_mps * np.array([math.cos(state.psi_rad), math.sin(state.psi_rad)])
    wind = np.array([disturbance.wind_x_mps, disturbance.wind_y_mps])
    return OwnStateEstimate(
        t_s=t_s,
        p_x_m=state.p_x_m,
        p_y_m=state.p_y_m,
        psi_rad=state.psi_rad,
        v_air_mps=state.v_mps,
        ground_velocity_mps=air + wind,
        wind_estimate_mps=wind,
        covariance=np.zeros((4, 4)),
    )


@dataclass
class Flight:
    """What a run recorded. Lists while flying, arrays afterwards."""

    t: list[float] = field(default_factory=list)
    p_x: list[float] = field(default_factory=list)
    p_y: list[float] = field(default_factory=list)
    psi: list[float] = field(default_factory=list)
    v: list[float] = field(default_factory=list)
    mass: list[float] = field(default_factory=list)
    omega_cmd: list[float] = field(default_factory=list)
    omega_delivered: list[float] = field(default_factory=list)
    omega_available: list[float] = field(default_factory=list)
    omega_sustained: list[float] = field(default_factory=list)
    omega_clipped: list[bool] = field(default_factory=list)
    captures: list[float] = field(default_factory=list)

    def arrays(self) -> dict[str, np.ndarray]:
        return {
            k: np.asarray(v) for k, v in self.__dict__.items() if k != "captures"
        }


def fly(setpoint_at, t_end: float, *, route=None, initial: VehicleState | None = None,
        disturbance: Disturbance = NO_DISTURBANCE, stop_when_route_done: bool = False):
    """Compose a platform and run it forward.

    `setpoint_at(t, capability)` supplies the motion setpoint, unless `route`
    is given -- then a WaypointPlanner supplies it and captures are recorded.

    `disturbance` reaches the integrator and nothing else, which is the whole
    point of it: wind enters the position rows only, so the platform drifts
    without guidance seeing anything change. Navigation here is a perfect
    estimate, so the drift is real rather than an estimation artefact.

    `stop_when_route_done` ends the run at capture instead of flying on. A
    measurement taken over the whole window would otherwise include whatever
    the platform did after the route ran out.
    """
    vehicle = reference_fighter()
    gauge = FuelGauge(
        FUEL_GAUGE_STANDARD, vehicle.lam.mass_dry_kg, np.random.default_rng(7)
    )
    manager = VehicleManager(vehicle, MANAGER_STANDARD)
    guidance = VehicleGuidance(manager, GUIDANCE_STANDARD)
    planner = WaypointPlanner(route, PLANNER_STANDARD) if route else None

    state = initial or VehicleState(0.0, 0.0, 0.0, 280.0, 16000.0)
    rec = Flight()
    previous_index = 0
    committed = None

    t = 0.0
    while t < t_end:
        if gauge.due(t):
            manager.ingest(gauge.sample(t, state))

        estimate = perfect_estimate(t, state, disturbance)
        capability = guidance.capability(estimate)

        if planner is not None:
            actions = planner.plan(t, estimate, capability)
            if actions.motion is not None:
                committed = actions.motion
            if planner.index > previous_index:
                rec.captures.append(t)
                previous_index = planner.index
            if stop_when_route_done and planner.finished:
                break
            setpoint = committed
        else:
            setpoint = setpoint_at(t, capability)

        if setpoint is None:
            t += DT
            continue

        command, saturation = guidance.command(t, setpoint, estimate)
        envelope = vehicle.capability(state)

        rec.t.append(t)
        rec.p_x.append(state.p_x_m)
        rec.p_y.append(state.p_y_m)
        rec.psi.append(state.psi_rad)
        rec.v.append(state.v_mps)
        rec.mass.append(state.mass_kg)
        rec.omega_cmd.append(saturation.requested.omega_rad_s)
        rec.omega_delivered.append(command.omega_rad_s)
        rec.omega_available.append(envelope.omega_available_rad_s)
        rec.omega_sustained.append(envelope.omega_sustained_rad_s)
        rec.omega_clipped.append(saturation.omega_clipped)

        manager.predict(t + DT, command.thrust_N, BELIEVED_TSFC_KG_PER_N_S)
        state = step_rk4(vehicle, state, command, DT, disturbance)
        t += DT

    return rec, vehicle, planner
