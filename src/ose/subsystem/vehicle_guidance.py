"""
Vehicle guidance: converts a commanded setpoint into an admissible
VehicleCommand.

Subsystem-layer: purely cyber. This module must not import VehicleState or
Disturbance from ose.resource.vehicle, and no public method may take a
parameter whose name begins with true_ -- see test_guidance_cannot_see_truth
in tests/test_vehicle_guidance.py, which checks both by parsing this file
with ast. Guidance never reads truth: its only state input is
OwnStateEstimate, converted to a VehicleState-shaped "believed state" via
its own as_vehicle_state() helper, which is not a truth read -- the values
came from an estimate, not a privileged query.

It does hold a Vehicle2D reference, and that is a different thing from
reading truth: Vehicle2D is a stateless capability/constraints model, not
live truth state, and a subsystem component binding down to the resource
layer for a model reference is exactly the composition rule ("a layer may
bind to the layer below it"). Imu already does the same for drag_N.

Guidance decides WHAT to command; it does not decide what is admissible.
The raw command from the control law is handed to Vehicle2D.project_command(),
which enforces the vehicle's own declared sets and reports any clipping via
Saturation. That report is returned to the caller rather than swallowed --
ADR 0006 exists so that a control law persistently commanding outside the
envelope is a visible finding, not a silent clip, and nothing has exercised
that path until this component.

mass_kg is today a plain parameter to command(), not sourced from
own_state: no component estimates mass yet (no vehicle-system/fuel-
accounting component exists). This is an acknowledged simplification, not
truth smuggled in under a different name -- see the module's own docstring
on OwnStateEstimate.as_vehicle_state() in interfaces.py.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ose.interfaces import HeadingSpeedSetpoint, OwnStateEstimate
from ose.resource.vehicle import Saturation, Vehicle2D, VehicleCommand


@dataclass
class VehicleGuidanceParameters:
    """Shape only, no defaults -- gains are a tuning choice, not a
    universal, so they belong in a named reference config
    (subsystem/reference_configs/reference_vehicle_guidance.py), not baked
    in here."""

    heading_gain_per_s: float   # omega_cmd = heading_gain_per_s * heading_error
    speed_gain_per_s: float     # v_dot_cmd = speed_gain_per_s * speed_error


class VehicleGuidance:
    """Proportional heading/speed-hold guidance.

    Memoryless: no integral or derivative term, no internal state beyond
    its parameters and the vehicle model it was built against.
    """

    def __init__(self, vehicle: Vehicle2D, parameters: VehicleGuidanceParameters) -> None:
        self.vehicle = vehicle
        self.par = parameters

    def command(
        self,
        t_s: float,
        setpoint,
        own_state: OwnStateEstimate,
        mass_kg: float,
    ) -> tuple[VehicleCommand, Saturation]:
        """Dispatches on setpoint type. Unknown types raise TypeError."""
        if isinstance(setpoint, HeadingSpeedSetpoint):
            return self._command_heading_speed(setpoint, own_state, mass_kg)
        raise TypeError(f"VehicleGuidance cannot command from {type(setpoint).__name__}")

    def _command_heading_speed(
        self, setpoint: HeadingSpeedSetpoint, own_state: OwnStateEstimate, mass_kg: float
    ) -> tuple[VehicleCommand, Saturation]:
        believed = own_state.as_vehicle_state(mass_kg)

        heading_error = math.remainder(setpoint.psi_cmd_rad - believed.psi_rad, 2.0 * math.pi)
        omega_cmd = self.par.heading_gain_per_s * heading_error

        # Feedforward the thrust needed to hold steady flight at the
        # commanded turn rate, then a proportional correction on speed
        # error -- cancels drag rather than fighting it.
        drag = self.vehicle.drag_N(believed.v_mps, believed.mass_kg, omega_cmd)
        speed_error = setpoint.v_cmd_mps - believed.v_mps
        thrust_cmd = drag + believed.mass_kg * self.par.speed_gain_per_s * speed_error

        return self.vehicle.project_command(believed, VehicleCommand(thrust_cmd, omega_cmd))
