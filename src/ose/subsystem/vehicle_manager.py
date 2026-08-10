"""
Vehicle manager: the platform's single publisher of vehicle.mass.v1, and the
only component entitled to ask the vehicle what it can do.

Subsystem-layer: purely cyber. It must not import VehicleState or Disturbance
from ose.resource.vehicle, and no public method may take a parameter whose
name begins with true_ -- see test_manager_cannot_see_truth. Its view of the
world is a FuelMeasurement from the fuel gauge and an OwnStateEstimate from
the navigation manager, both already-published estimates.

What it is
----------
Vehicle2D with the mass argument closed over by a believed value.

That is the whole idea, and it is not a pass-through. Every question worth
asking the vehicle model -- what turn rate is available, what thrust holds
this turn, is this command admissible -- takes a state whose mass_kg field is
precisely the quantity nobody was estimating. Guidance used to supply it as a
plain parameter, which meant every caller reached for the true mass and the
truth boundary was intact in guidance while being breached in every
composition of it (ADR 0011 said so; ADR 0015 is the fix).

So the manager owns the believed mass, owns the conversion from an estimate to
a believed VehicleState, and answers vehicle questions at that mass. Consumers
never construct a believed state and never see a mass parameter.

    Vehicle2D            physics, needs a mass
      +- VehicleManager      binds believed mass    <- this module
           +- VehicleGuidance    adds navigation uncertainty
                +- WaypointPlanner

Each layer adds exactly what it knows.

Sum, not filter -- and what that costs
--------------------------------------
Today the belief is dry + payload + fuel, where dry mass is a vehicle design
constant, payload is configuration, and only the fuel term is measured. There
is no filtering: the last fuel reading is used as it stands, and the published
sigma is the sigma that travelled with it.

That is a real component but a thin estimator, and it should not be mistaken
for more. Two consequences a reader should know about:

  - Between fuel measurements the belief is stale. Mass is falling
    continuously at a rate the platform could predict, because mdot follows
    from the commanded thrust, and nothing here uses that. A dead-reckoning
    version -- predict mass forward on commanded thrust, correct on each
    measurement, grow the covariance in between -- is the same structure
    TimeEstimator already uses for the clock (ADR 0010), and is the intended
    next step.

  - The published sigma therefore describes the measurement, not the belief.
    It is honest about the reading and silent about the staleness, so it is a
    floor rather than a bound. Do not build a consistency test against it and
    conclude the belief is calibrated.

Payload is a single configured scalar. Effectors and stores that are released
during a run would each contribute their own term and change the sum at
discrete instants; that is a later problem, and the record already publishes
the contributions separately so it can be extended without a version bump.
"""

from __future__ import annotations

from dataclasses import dataclass

from ose.interfaces import FuelMeasurement, MassEstimate, OwnStateEstimate
from ose.resource.vehicle import Capability, Saturation, Vehicle2D, VehicleCommand


@dataclass
class VehicleManagerParameters:
    """Shape only, no defaults, as everywhere else in this repository."""

    # What the platform is carrying beyond airframe and fuel. Zero is a
    # legitimate value and is what the reference config uses, since no
    # effector or store exists yet.
    payload_mass_kg: float
    # What to believe before the fuel gauge has said anything. A platform has
    # a mass from the instant it exists, and refusing to answer until the
    # first measurement would make every consumer carry a special case for
    # the first cycle. Declared with its own sigma so the guess is visible as
    # a guess rather than borrowing the gauge's precision.
    initial_fuel_kg: float
    initial_fuel_sigma_kg: float


class VehicleManager:
    """Owns the platform's believed mass and answers vehicle questions at it."""

    def __init__(self, vehicle: Vehicle2D, parameters: VehicleManagerParameters) -> None:
        self.vehicle = vehicle
        self.par = parameters
        self._fuel_kg = parameters.initial_fuel_kg
        self._fuel_sigma_kg = parameters.initial_fuel_sigma_kg
        self._valid_time_s = 0.0

    # -- the mass belief --------------------------------------------------

    def ingest(self, measurement) -> None:
        """Dispatches on measurement type, mirroring the estimators, so a
        second mass source can be added without changing the protocol.
        Unknown types raise TypeError."""
        if isinstance(measurement, FuelMeasurement):
            # Taken as it stands. The sigma is the one that travelled with the
            # reading (invariant 4); this component does not substitute a
            # configured value and does not get to improve on it.
            self._fuel_kg = measurement.fuel_remaining_kg
            self._fuel_sigma_kg = measurement.fuel_remaining_sigma_kg
            self._valid_time_s = measurement.valid_time_s
            return
        raise TypeError(
            f"VehicleManager cannot ingest {type(measurement).__name__}"
        )

    @property
    def mass_kg(self) -> float:
        """The believed mass, as a bare number, for internal use and for
        callers that genuinely want only the scalar."""
        return self.par.payload_mass_kg + self.vehicle.lam.mass_dry_kg + self._fuel_kg

    def mass(self, t_s: float) -> MassEstimate:
        """Publish vehicle.mass.v1.

        t_s is when this was asked, not when the fuel was measured; the
        belief itself is as of the last measurement's valid time, which is
        why the staleness noted in the module docstring is invisible in the
        sigma.
        """
        return MassEstimate(
            t_s=t_s,
            mass_kg=self.mass_kg,
            # Dry and payload are exact, so the whole uncertainty is the
            # fuel term's and passes through unchanged.
            mass_sigma_kg=self._fuel_sigma_kg,
            dry_mass_kg=self.vehicle.lam.mass_dry_kg,
            payload_mass_kg=self.par.payload_mass_kg,
            fuel_mass_kg=self._fuel_kg,
        )

    # -- vehicle questions, answered at the believed mass -----------------
    #
    # These are why the component is the sole consumer of Vehicle2D rather
    # than merely a mass publisher. Guidance needs three different things
    # from the vehicle and only one of them is a capability record: it also
    # needs a parametrised thrust query for its feedforward, and enforcement.
    # Republishing a Capability alone would have left guidance holding a
    # Vehicle2D anyway, and the mass parameter with it.

    def believed_state(self, own_state: OwnStateEstimate):
        """The estimate, dressed as a VehicleState at the believed mass.

        The single home for as_vehicle_state(). Guidance called it before,
        which is what forced a mass parameter through every signature above
        it. Returned rather than kept private because the demos and the
        eventual simulation core need it, but note that it is a *believed*
        state: the values came from an estimate, not a privileged query.
        """
        return own_state.as_vehicle_state(self.mass_kg)

    def capability(
        self, own_state: OwnStateEstimate, omega_rad_s: float = 0.0
    ) -> Capability:
        """What the vehicle can currently do, at the believed mass.

        omega_rad_s is forwarded: the vehicle reports thrust_required_N for
        steady flight at a given turn rate, and guidance's feedforward needs
        it evaluated at the rate that will actually be flown.
        """
        return self.vehicle.capability(
            self.believed_state(own_state), omega_rad_s=omega_rad_s
        )

    def project_command(
        self, own_state: OwnStateEstimate, command: VehicleCommand
    ) -> tuple[VehicleCommand, Saturation]:
        """Enforce the vehicle's admissible sets, at the believed mass.

        Forwarded rather than reimplemented: the vehicle declares its own
        sets and this component has no opinion about them (ADR 0006). What it
        contributes is the mass those sets are evaluated at, which matters --
        the stall floor moves with mass, so a command admissible at one
        believed mass may not be at another.
        """
        return self.vehicle.project_command(self.believed_state(own_state), command)
