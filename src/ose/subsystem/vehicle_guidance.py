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

from ose.interfaces import GuidanceCapability, HeadingSpeedSetpoint, OwnStateEstimate
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

    def capability(
        self, own_state: OwnStateEstimate, mass_kg: float
    ) -> GuidanceCapability:
        """Compose the vehicle's capability with navigation's.

        A control loop is bounded by both, and by different things: the
        vehicle decides which setpoints are reachable at all, navigation
        decides how tightly a reachable one can be held. Neither alone is
        the answer, which is what makes this the first capability model in
        the repository that is genuinely composed rather than reported.

        The navigation half is read from the covariance travelling with
        own_state rather than by querying a navigation component. That
        follows the rule ADR 0009 set for measurements -- the consumer uses
        the uncertainty that arrives with the data -- and it avoids
        coupling guidance to whichever estimator happens to be installed.
        It is also the more useful number: the covariance is what
        navigation's uncertainty *is right now*, degraded by a GNSS outage
        or not, where a static claim from the estimator would not be.
        """
        believed = own_state.as_vehicle_state(mass_kg)
        envelope = self.vehicle.capability(believed)

        return GuidanceCapability(
            max_turn_rate_rad_s=envelope.omega_available_rad_s,
            # The speed floor is whichever binds: aerodynamic stall at this
            # mass, or the airframe's hard minimum. Same rule the vehicle's
            # own admissible() applies.
            min_speed_mps=max(self.vehicle.lam.v_min_mps, envelope.v_stall_mps),
            max_speed_mps=self.vehicle.lam.v_max_mps,
            heading_hold_sigma_rad=math.sqrt(max(own_state.covariance[2, 2], 0.0)),
            speed_hold_sigma_mps=math.sqrt(max(own_state.covariance[3, 3], 0.0)),
        )

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

        # Ask the vehicle what it can currently do rather than deriving it
        # from the vehicle's internals. This is the capability model being
        # used for what it is for: a consumer querying instead of
        # reimplementing (docs/10-concepts.md, ADR 0012).
        envelope = self.vehicle.capability(believed)

        # Feedforward the thrust to hold steady flight through the turn the
        # vehicle will ACTUALLY fly, not the one the error term asked for.
        #
        # These differ whenever the setpoint is aggressive, and not
        # slightly: induced drag scales with load factor squared, so
        # feedforwarding an unachievable turn rate demands an absurd
        # thrust. A commanded heading reversal at 250 m/s asks for 54 deg/s,
        # a 24 g turn against a 9 g airframe, and 1330 kN from a 130 kN
        # engine. Evaluating at the achievable rate instead gives 214 kN --
        # still saturating, but a number that means something.
        #
        # The command still carries the desired omega_cmd, unclipped, so
        # project_command below is what performs and reports enforcement
        # (ADR 0006) rather than guidance quietly pre-limiting itself. It
        # clips to exactly omega_available_rad_s, which is what the thrust
        # was computed for, so the pair that comes out is consistent.
        omega_achievable = math.copysign(
            min(abs(omega_cmd), envelope.omega_available_rad_s), omega_cmd
        )
        steady = self.vehicle.capability(believed, omega_rad_s=omega_achievable)

        speed_error = setpoint.v_cmd_mps - believed.v_mps
        thrust_cmd = (
            steady.thrust_required_N
            + believed.mass_kg * self.par.speed_gain_per_s * speed_error
        )

        return self.vehicle.project_command(believed, VehicleCommand(thrust_cmd, omega_cmd))
