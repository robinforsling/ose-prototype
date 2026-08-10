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

Estimating the mass
-------------------
Mass is dry + payload + fuel. Dry mass is a vehicle design constant and
payload is a configuration decision, so both are exact; the whole estimation
problem is the fuel term, and the whole uncertainty is the fuel term's.

Fuel is tracked by a two-state Kalman filter that predicts on the commanded
thrust and corrects on each FuelMeasurement:

    State (2 states)
        0   fuel_kg      remaining fuel                                  [kg]
        1   tsfc_error   fractional error in the burn coefficient        [-]

    d(fuel)/dt = -c_believed * (1 + tsfc_error) * thrust_N
    d(tsfc_error)/dt = 0, plus a slow random walk

This is the structure TimeEstimator uses for the clock (ADR 0010), with one
difference that matters: there IS a correction source here, so this filter
actually closes the loop where that one can only predict.

Why tsfc_error is a state and not process noise
-----------------------------------------------
A miscalibrated burn coefficient is a bias. Its contribution to the fuel error
grows linearly in integrated thrust, while a white process-noise term makes
the covariance grow as the square root of time. Model the bias as noise and
the filter is briefly overconservative and then permanently overconfident --
the exact failure the NEES tests in this repository exist to catch, and the
reason the INS/GNSS filter carries IMU bias states rather than inflating Q.

It is weakly observable, which is realistic rather than a defect: a five per
cent coefficient error accumulates about 18 kg over five minutes of cruise,
against a 20 kg gauge sigma. The filter separates it slowly, and the
covariance says so.

The believed coefficient is this component's own parameter and is NOT read
from the vehicle
----------------------------------------------------------------------------
Reading Vehicle2D.theta.c_tsfc would be legal -- it is a model parameter, not
truth -- and it would be wrong. Predicting with the same coefficient the
vehicle burns at makes the prediction exact by construction, so the filter
would look excellent for a reason that will never hold on a real platform, no
mismatch could exist, and the consistency test would be vacuous. That is the
same class of self-congratulating configuration as fusing the black-box nav
unit with the real estimator (ADR 0014).

So the manager declares what it believes and how well, exactly as
EstimatorParameters is decoupled from ImuParameters (ADR 0009) and
TimeEstimatorParameters from the Clock resource. The reference config sets the
believed coefficient to the nominal one, and tsfc_sigma_fraction declares that
the platform does not know it is right.

Payload is a single configured scalar. Effectors and stores that are released
during a run would each contribute their own term and change the sum at
discrete instants; that is a later problem, and the record already publishes
the contributions separately so it can be extended without a version bump.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from ose.interfaces import FuelMeasurement, MassEstimate, OwnStateEstimate
from ose.resource.vehicle import Capability, Saturation, Vehicle2D, VehicleCommand

I_FUEL = 0
I_TSFC = 1
N_ERR = 2


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
    # The filter's own assumed burn model, deliberately decoupled from the
    # vehicle's true coefficient -- see the module docstring on why reading
    # Vehicle2D.theta.c_tsfc would make this component untestable.
    tsfc_kg_per_N_s: float
    tsfc_sigma_fraction: float        # how well the coefficient is believed known
    # Allows the coefficient error to drift slowly rather than being pinned
    # forever once observed. Without it the filter's confidence in tsfc_error
    # only ever grows, and a real engine does change.
    tsfc_walk_per_sqrt_s: float
    # Unmodelled burn -- bleed air, leaks, anything the thrust-proportional
    # term does not capture.
    fuel_walk_kg_per_sqrt_s: float


class VehicleManager:
    """Owns the platform's believed mass and answers vehicle questions at it."""

    def __init__(self, vehicle: Vehicle2D, parameters: VehicleManagerParameters) -> None:
        self.vehicle = vehicle
        self.par = parameters

        self._fuel_kg = parameters.initial_fuel_kg
        self._tsfc_error = 0.0
        self.P = np.diag(
            [parameters.initial_fuel_sigma_kg**2, parameters.tsfc_sigma_fraction**2]
        )

        self._t = 0.0
        self._last_ingest_time = -math.inf

    # -- prediction -------------------------------------------------------

    def predict(self, t_s: float, thrust_N: float) -> None:
        """Propagate the fuel belief to t_s, burning at the commanded thrust.

        The thrust is held constant across the interval. That is the same
        zero-order hold the command itself has -- guidance emits one value per
        cycle and the vehicle flies it until the next -- so it is not an
        approximation the filter is making on its own.

        Called by whoever drives the cycle, after the command is decided.
        Deliberately not folded into project_command(): asking whether a
        command is admissible must not commit the platform to having flown it.
        """
        dt = t_s - self._t
        if dt <= 0.0:
            self._t = max(self._t, t_s)
            return

        c = self.par.tsfc_kg_per_N_s
        # The vehicle stops burning at dry mass, so the filter must too --
        # otherwise it predicts fuel through zero and into negative mass.
        burning = self._fuel_kg > 0.0
        rate = c * (1.0 + self._tsfc_error) * thrust_N if burning else 0.0

        # F is nilpotent for this model, so Phi = I + F*dt is exact rather
        # than a first-order truncation.
        F = np.zeros((N_ERR, N_ERR))
        F[I_FUEL, I_TSFC] = -c * thrust_N if burning else 0.0
        Phi = np.eye(N_ERR) + F * dt

        self._fuel_kg = max(self._fuel_kg - rate * dt, 0.0)

        Qc = np.diag([
            self.par.fuel_walk_kg_per_sqrt_s**2,
            self.par.tsfc_walk_per_sqrt_s**2,
        ])
        self.P = Phi @ self.P @ Phi.T + Qc * dt
        self.P = 0.5 * (self.P + self.P.T)
        self._t = t_s

    # -- correction -------------------------------------------------------

    def ingest(self, measurement) -> None:
        """Dispatches on measurement type, mirroring the estimators, so a
        second mass source can be added without changing the protocol.
        Measurements must arrive in non-decreasing valid_time_s order.
        Unknown types raise TypeError."""
        if isinstance(measurement, FuelMeasurement):
            if measurement.valid_time_s < self._last_ingest_time:
                raise ValueError(
                    f"measurement at t={measurement.valid_time_s} arrived after "
                    f"t={self._last_ingest_time}; measurements must be ingested "
                    "in non-decreasing valid_time_s order"
                )
            self._last_ingest_time = measurement.valid_time_s
            self._correct_fuel(measurement)
            return
        raise TypeError(
            f"VehicleManager cannot ingest {type(measurement).__name__}"
        )

    def _correct_fuel(self, m: FuelMeasurement) -> None:
        """Scalar Kalman update on the fuel channel.

        R is the variance the measurement declared, never a configured one
        (invariant 4). A gauge that reports a worse sigma is trusted less, and
        this component has no opinion about whether that sigma is right.
        """
        H = np.zeros((1, N_ERR))
        H[0, I_FUEL] = 1.0
        R = np.array([[m.fuel_remaining_sigma_kg**2]])

        innovation = m.fuel_remaining_kg - self._fuel_kg
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        correction = K.flatten() * innovation
        self._fuel_kg = max(self._fuel_kg + float(correction[I_FUEL]), 0.0)
        self._tsfc_error += float(correction[I_TSFC])

        # Joseph form: stays symmetric positive-definite where the short form
        # can drift negative after many updates.
        A = np.eye(N_ERR) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    # -- publication ------------------------------------------------------

    @property
    def mass_kg(self) -> float:
        """The believed mass, as a bare number, for internal use and for
        callers that genuinely want only the scalar."""
        return self.par.payload_mass_kg + self.vehicle.lam.mass_dry_kg + self._fuel_kg

    def mass(self, t_s: float) -> MassEstimate:
        """Publish vehicle.mass.v1.

        Does not propagate: t_s labels when the belief was asked for, and the
        belief is as of the last predict() or ingest(). A caller that wants it
        current calls predict() first, which is the cycle this component is
        driven by.
        """
        return MassEstimate(
            t_s=t_s,
            mass_kg=self.mass_kg,
            dry_mass_kg=self.vehicle.lam.mass_dry_kg,
            payload_mass_kg=self.par.payload_mass_kg,
            fuel_mass_kg=self._fuel_kg,
            tsfc_error=self._tsfc_error,
            covariance=self.P.copy(),
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
