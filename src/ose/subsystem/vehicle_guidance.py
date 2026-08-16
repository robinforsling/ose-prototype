"""
Vehicle guidance: converts a commanded setpoint into an admissible
VehicleCommand.

Subsystem-layer: purely cyber. This module must not import VehicleState or
Disturbance from ose.equipment.vehicle, and no public method may take a
parameter whose name begins with true_ -- see test_guidance_cannot_see_truth
in tests/test_vehicle_guidance.py, which checks both by parsing this file
with ast. Guidance never reads truth: its only state input is an
OwnStateEstimate published by the navigation manager.

It binds to a VehicleManager, a peer in the same layer on the same platform,
and not to PlanarPointMass. That is the change ADR 0015 made. Guidance used to hold
the vehicle model directly and take mass_kg as a plain parameter, which left
the truth boundary intact here and breached in every *composition* of this
component: nothing estimated mass, so every caller reached for the true
value. The manager owns the believed mass now, so there is no mass parameter
to supply and nothing for a caller to reach for.

It follows that guidance no longer constructs a believed VehicleState. Every
vehicle question goes through the manager, which answers at the mass it
believes. Guidance contributes what it alone knows -- how well navigation can
hold what the vehicle can reach -- and nothing else.

Guidance decides WHAT to command; it does not decide what is admissible.
The raw command from the control law is handed to the manager's
project_command(), which forwards to the vehicle's own declared sets and
reports any clipping via Saturation. That report is returned to the caller
rather than swallowed -- ADR 0006 exists so that a control law persistently
commanding outside the envelope is a visible finding, not a silent clip.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.equipment.vehicle import Saturation, VehicleCommand
from ose.interfaces import (
    GuidanceCapability,
    HeadingSpeedSetpoint,
    OwnStateEstimate,
    TrackSpeedSetpoint,
    TurnRateSpeedSetpoint,
)
from ose.subsystem.vehicle_manager import VehicleManager


@dataclass
class VehicleGuidanceParameters:
    """Shape only, no defaults -- gains are a tuning choice, not a
    universal, so they belong in a named reference config
    (subsystem/reference_configs/reference_vehicle_guidance.py), not baked
    in here."""

    heading_gain_per_s: float   # omega_cmd = heading_gain_per_s * heading_error
    speed_gain_per_s: float     # v_dot_cmd = speed_gain_per_s * speed_error
    track_gain_per_s: float     # omega_cmd = track_gain_per_s * track_error


def _track_sigma_rad(own_state: OwnStateEstimate) -> float:
    """How well a ground track can be held, from the ground-velocity covariance.

    The track angle is atan2(v_y, v_x), so its variance is the ground-velocity
    covariance projected through that function's gradient:

        J     = [-v_y, v_x] / |v|^2
        sigma^2 = J P J^T

    Not the heading sigma. A track loop steers on ground velocity, so its floor
    is the uncertainty in ground velocity; reporting the heading sigma under a
    track name would be the anti-conservative mislabelling ADR 0016 exists
    about, and in wind the two are not even close.

    Zero at a standstill, where the angle is undefined and the claim would be
    meaningless anyway.
    """
    v = own_state.ground_velocity_mps
    speed_squared = float(v[0] ** 2 + v[1] ** 2)
    if speed_squared < 1.0:
        return 0.0
    jacobian = np.array([-v[1], v[0]]) / speed_squared
    variance = float(jacobian @ own_state.ground_velocity_covariance @ jacobian)
    return math.sqrt(max(variance, 0.0))


class VehicleGuidance:
    """Proportional heading/speed-hold guidance.

    Memoryless: no integral or derivative term, no internal state beyond
    its parameters and the vehicle model it was built against.
    """

    def __init__(
        self, manager: VehicleManager, parameters: VehicleGuidanceParameters
    ) -> None:
        self.manager = manager
        self.par = parameters

    def capability(self, own_state: OwnStateEstimate) -> GuidanceCapability:
        """Compose the vehicle manager's capability with navigation's.

        A control loop is bounded by both, and by different things: the
        vehicle decides which setpoints are reachable at all, navigation
        decides how tightly a reachable one can be held. Neither alone is
        the answer, which is what makes this a genuinely composed capability
        model rather than a reported one.

        The vehicle half now arrives already evaluated at the platform's
        believed mass, because the manager owns that belief. The chain is
        vehicle -> manager (adds mass) -> guidance (adds navigation), each
        layer adding exactly what it knows.

        This is the envelope guidance *promises*, so it asks the manager for
        the bound rather than the point estimate: a planner deciding whether
        a leg is flyable should be told what the platform is confident of,
        not its best guess. The control law below does the opposite and uses
        the point estimate, because feedforward computed for a mass the
        aircraft does not have is wrong rather than cautious. See
        VehicleManager.capability_bound().

        The navigation half is read from the covariance travelling with
        own_state rather than by querying a navigation component. That
        follows the rule ADR 0009 set for measurements -- the consumer uses
        the uncertainty that arrives with the data -- and it avoids
        coupling guidance to whichever estimator happens to be installed.
        It is also the more useful number: the covariance is what
        navigation's uncertainty *is right now*, degraded by a GNSS outage
        or not, where a static claim from the estimator would not be.
        """
        envelope = self.manager.capability_bound(own_state)

        return GuidanceCapability(
            max_turn_rate_rad_s=envelope.max_turn_rate_rad_s,
            # Straight from the manager's promised envelope. Guidance used to
            # compose the floor itself from v_stall and Constraints, which
            # meant a consumer reimplementing a rule the vehicle already owns.
            min_speed_mps=envelope.min_speed_mps,
            max_speed_mps=envelope.max_speed_mps,
            heading_hold_sigma_rad=math.sqrt(max(own_state.covariance[2, 2], 0.0)),
            speed_hold_sigma_mps=math.sqrt(max(own_state.covariance[3, 3], 0.0)),
            track_hold_sigma_rad=_track_sigma_rad(own_state),
            # Self-describing: a consumer can tell a promised envelope from a
            # best guess without knowing how this component was configured.
            mass_margin_sigma=envelope.mass_margin_sigma,
        )

    def command(
        self,
        t_s: float,
        setpoint,
        own_state: OwnStateEstimate,
    ) -> tuple[VehicleCommand, Saturation]:
        """Dispatches on setpoint type. Unknown types raise TypeError."""
        if isinstance(setpoint, HeadingSpeedSetpoint):
            return self._command_heading_speed(setpoint, own_state)
        if isinstance(setpoint, TrackSpeedSetpoint):
            return self._command_track_speed(setpoint, own_state)
        if isinstance(setpoint, TurnRateSpeedSetpoint):
            return self._command_turn_rate_speed(setpoint, own_state)
        raise TypeError(f"VehicleGuidance cannot command from {type(setpoint).__name__}")


    def _project(self, own_state: OwnStateEstimate, omega_cmd: float, v_cmd_mps: float):
        """Thrust feedforward and enforcement, shared by both setpoint types.

        Both want the same thing once a turn rate has been decided: hold
        speed through the turn the vehicle will actually fly, correct any
        speed error, then let the vehicle enforce its own sets.
        """
        # Ask what the vehicle can currently do rather than deriving it from
        # the vehicle's internals. This is the capability model being used for
        # what it is for: a consumer querying instead of reimplementing
        # (docs/10-concepts.md, ADR 0012).
        envelope = self.manager.capability(own_state)

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
        steady = self.manager.capability(own_state, omega_rad_s=omega_achievable)

        speed_error = v_cmd_mps - own_state.v_air_mps
        # The believed mass, from the component that owns it. This term turns
        # a desired acceleration into a force, so it needs a mass and has no
        # business deciding what that mass is.
        thrust_cmd = (
            steady.thrust_required_N
            + self.manager.mass_kg * self.par.speed_gain_per_s * speed_error
        )

        return self.manager.project_command(
            own_state, VehicleCommand(thrust_cmd, omega_cmd)
        )

    def _command_turn_rate_speed(
        self, setpoint: TurnRateSpeedSetpoint, own_state: OwnStateEstimate
    ) -> tuple[VehicleCommand, Saturation]:
        """No heading loop at all: the commanded rate is the command. An
        unreachable rate saturates against omega_available and stays there,
        which is the whole reason this setpoint type exists."""
        return self._project(own_state, setpoint.omega_cmd_rad_s, setpoint.v_cmd_mps)

    def _command_track_speed(
        self, setpoint: TrackSpeedSetpoint, own_state: OwnStateEstimate
    ) -> tuple[VehicleCommand, Saturation]:
        """Hold a ground track, by closing the loop on ground velocity.

        Structurally the heading law with a different error, and exact for the
        same reason: the plant integrates omega into psi, so omega settling at
        zero requires the TRACK error to be zero, whatever the wind is doing.
        The crab angle is where the heading ends up, not something computed --
        heading is left free precisely so that it can absorb the wind.

        Which is why there is no crab feedforward from own_state's wind
        estimate, though one would be easy and the field is otherwise unread.
        It cannot enter as a rate, so it can only enter as a heading command,
        and a heading-error term and a track-error term both feeding one rate
        command balance at a NON-zero equilibrium. Measured on the kinematics
        with a 30 per cent wind-estimate error: feedback alone holds 0.000
        degrees, feedback plus feedforward holds 1.03 degrees and never
        settles. It buys seven seconds of settling in the case that did not
        need help and a standing error in the case that did, and this
        platform's wind is only observable after a turn. See ADR 0029.

        The track angle is undefined at zero ground speed. A stationary
        platform has no track to hold, so the setpoint is refused rather than
        answered with a fabricated one.
        """
        v_ground = own_state.ground_velocity_mps
        speed = float(math.hypot(v_ground[0], v_ground[1]))
        if speed < 1.0:
            raise ValueError(
                f"ground speed {speed:.3f} m/s is too low for a track to be "
                "defined; a stationary platform has no track to hold"
            )

        track = math.atan2(v_ground[1], v_ground[0])
        track_error = math.remainder(
            setpoint.psi_g_cmd_rad - track, 2.0 * math.pi
        )
        omega_cmd = (
            self.par.track_gain_per_s * track_error + setpoint.psi_g_rate_cmd_rad_s
        )
        return self._project(own_state, omega_cmd, setpoint.v_cmd_mps)

    def _command_heading_speed(
        self, setpoint: HeadingSpeedSetpoint, own_state: OwnStateEstimate
    ) -> tuple[VehicleCommand, Saturation]:
        heading_error = math.remainder(
            setpoint.psi_cmd_rad - own_state.psi_rad, 2.0 * math.pi
        )

        # Proportional correction plus the setpoint's own rate fed forward.
        # Without the feedforward term a moving setpoint is never caught: the
        # loop settles where the correction alone supplies the whole turn
        # rate, leaving a standing error of rate/gain. With it, the error
        # settles at zero and the correction only has to make up the
        # difference.
        omega_cmd = (
            self.par.heading_gain_per_s * heading_error + setpoint.psi_rate_cmd_rad_s
        )

        return self._project(own_state, omega_cmd, setpoint.v_cmd_mps)

