"""
Vehicle manager: the platform's single publisher of vehicle.mass.v1, and the
only component entitled to ask the vehicle what it can do.

Subsystem-layer: purely cyber. It must not import VehicleState or Disturbance
from ose.equipment.vehicle, and no public method may take a parameter whose
name begins with true_ -- see test_manager_cannot_see_truth. Its view of the
world is a FuelMeasurement from the fuel gauge and an OwnStateEstimate from
the navigation manager, both already-published estimates.

What it is
----------
PlanarPointMass with the mass argument closed over by a believed value.

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

    PlanarPointMass            physics, needs a mass
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

One fuel sink, and what a second one would do
---------------------------------------------
The prediction assumes thrust is the only thing consuming fuel. A power
generator would be a second sink, and its failure mode is worth knowing
before anyone adds one, because it is not a broken filter.

Measured over 150 s of steady cruise: an unmodelled drain of one to three per
cent of the thrust burn leaves ANEES at about 1.1, which is consistent. The
tsfc_error state absorbs it, because at constant thrust a constant drain is
indistinguishable from a slightly wrong burn coefficient. Only around ten per
cent does the covariance stop covering the error (ANEES 1.5).

So the filter stays calibrated while its coefficient estimate quietly becomes
wrong, which is worse than failing outright. And the absorption is an artefact
of constant thrust: a constant drain cannot be represented as a
thrust-proportional coefficient once thrust varies, and it integrates freely
through a gauge outage, where nothing corrects it at all.

A generator therefore needs its own term in the prediction rather than being
left to the coefficient, and adding one should come with a consistency run at
varying thrust, which no test here currently does.

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

The coefficient is supplied, and must never be the vehicle's true one
--------------------------------------------------------------------
predict() takes the burn coefficient rather than holding it, so that the
filter is independent of how the engine is built -- see that method. What it
does hold is the *doubt*: tsfc_sigma_fraction is this component's prior on how
well the coefficient is known, and tsfc_error is the fractional calibration
error it estimates.

Whatever supplies the coefficient must supply the platform's BELIEF about it.
Reading PlanarPointMass.theta.c_tsfc would be legal -- it is a model parameter, not
truth -- and it would be wrong. Predicting with the same coefficient the
vehicle burns at makes the prediction exact by construction, so the filter
would look excellent for a reason that will never hold on a real platform, no
mismatch could exist, and the consistency test would be vacuous. That is the
same class of self-congratulating configuration as fusing two navigation
sources that are not independent (ADR 0014).

Moving the coefficient to an argument moves that risk to the caller, which is
exactly where the mass parameter went wrong before ADR 0015: the component
stayed clean while every composition of it reached for truth. So the rule is
enforced across the repository rather than in this file --
test_no_cyber_component_reads_the_true_burn_coefficient walks everything above
the equipment layer.

The fractional form is what lets one state cover a switched engine. The error
multiplies whatever coefficient it is handed, so c_actual = c_given * (1 + e)
holds for nominal and boost alike as long as both come from one calibration.
Independent errors per mode would be a second state and a deliberate
modelling decision, not something this formulation quietly gets wrong.

Payload is a single configured scalar. Effectors and stores that are released
during a run would each contribute their own term and change the sum at
discrete instants; that is a later problem, and the record already publishes
the contributions separately so it can be extended without a version bump.
"""

from __future__ import annotations

import math
import dataclasses
from dataclasses import dataclass

import numpy as np

from ose.equipment.vehicle import Capability, Saturation, VehicleCommand
from ose.interfaces import (
    FuelMeasurement,
    MassEstimate,
    OwnStateEstimate,
    PlatformBeliefs,
    PromisedEnvelope,
    Vehicle,
)

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
    # How well the burn coefficient is believed known, as a fraction. The
    # coefficient itself is supplied to predict() rather than held here, so
    # that this component need not know how many operating modes the engine
    # has; the doubt about it is the filter's own prior and stays.
    tsfc_sigma_fraction: float
    # Allows the coefficient error to drift slowly rather than being pinned
    # forever once observed. Without it the filter's confidence in tsfc_error
    # only ever grows, and a real engine does change.
    tsfc_walk_per_sqrt_s: float
    # Unmodelled burn -- bleed air, leaks, anything the thrust-proportional
    # term does not capture.
    fuel_walk_kg_per_sqrt_s: float
    # How many sigma of mass uncertainty capability_bound() adds before
    # reporting an envelope. See that method for why the margin is added
    # rather than subtracted, and why only the promised envelope carries it.
    capability_margin_sigma: float


class VehicleManager:
    """Owns the platform's believed mass and answers vehicle questions at it."""

    def __init__(self, vehicle: Vehicle, parameters: VehicleManagerParameters) -> None:
        self.vehicle = vehicle
        self.par = parameters

        self._fuel_kg = parameters.initial_fuel_kg
        self._tsfc_error = 0.0
        self.P = np.diag(
            [parameters.initial_fuel_sigma_kg**2, parameters.tsfc_sigma_fraction**2]
        )

        self._t = 0.0
        self._last_ingest_time = -math.inf

        # The platform's other beliefs. Mode is opaque here on purpose: this
        # component carries it between whoever selects it and the model that
        # interprets it, and has no opinion about what the values mean. A
        # single-mode model never sets it and never reads it.
        self._mode: object | None = None
        self._thermal = 0.0
        self._mode_changed_at_s = 0.0

    # -- prediction -------------------------------------------------------

    def predict(
        self,
        t_s: float,
        thrust_N: float,
        tsfc_kg_per_N_s: float,
        own_state: OwnStateEstimate | None = None,
    ) -> None:
        """Propagate the fuel belief to t_s, burning at the commanded thrust.

        Both the thrust and the coefficient are held constant across the
        interval. That is the same zero-order hold the command itself has --
        one value per cycle, flown until the next -- so it is not an
        approximation the filter is making on its own.

        The coefficient is an argument rather than a parameter of this
        component, and that is what keeps the filter independent of how the
        engine is built. A two-mode engine, a three-mode one, a continuously
        variable throttle and a different vehicle model entirely all pass a
        number here; none of them requires this component to know that modes
        exist. Enumerating equipment-layer operating modes inside a subsystem
        component would be the wrong shape, the same way taking a mass
        parameter was (ADR 0015).

        It must be the platform's BELIEVED coefficient, never the vehicle's
        true one -- see the module docstring, and
        test_no_cyber_component_reads_the_true_burn_coefficient, which
        enforces that on the callers rather than only here.

        Called by whoever drives the cycle, after the command is decided.
        Deliberately not folded into project_command(): asking whether a
        command is admissible must not commit the platform to having flown it.
        """
        dt = t_s - self._t
        if dt <= 0.0:
            self._t = max(self._t, t_s)
            return

        c = tsfc_kg_per_N_s
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

        self._predict_thermal(own_state, dt)
        self._t = t_s

    def _predict_thermal(self, own_state: OwnStateEstimate | None, dt: float) -> None:
        """Dead-reckon the thermal accumulator, if the model has one.

        Same shape as the mass belief: something is consumed at a rate the
        platform can predict, nothing measures it, so the belief is
        propagated. Mass burns at the commanded thrust; the thermal state
        fills at the commanded mode.

        The LAW belongs to the model, which publishes it as a rate (ADR 0004),
        and this component only integrates it. Recomputing sigma_q here would
        be a consumer reimplementing a rule the vehicle already owns, which is
        the mistake guidance made with the stall floor.

        There is no correction source, so this is prediction only -- the
        TimeEstimator situation. The covariance is not tracked because nothing
        yet consumes a thermal uncertainty; when something does, it belongs in
        the filter alongside fuel rather than bolted on here.
        """
        rate = getattr(self.vehicle, "thermal_rate", None)
        if rate is None:
            return                      # the model has nothing to accumulate
        if own_state is None:
            # Loudly, because the alternative is a thermal belief that never
            # moves: boost would look free, capability would over-report how
            # long it could be held, and nothing would fail. The argument is
            # optional only so that a single-mode platform need not pass it.
            raise ValueError(
                "predict() needs own_state for a model with a thermal state, "
                "or the belief silently never accumulates"
            )
        believed = self.vehicle.state_from(own_state, self.beliefs())
        self._thermal = max(self._thermal + rate(believed) * dt, 0.0)

    # -- the propulsion mode ----------------------------------------------

    def select_mode(self, t_s: float, mode: object) -> None:
        """Record the mode the platform is now in.

        Carried, not decided and not interpreted. The decision is a planning
        one -- the thermal budget and the fuel reserve are finite and
        contested, so a guidance loop chasing a speed setpoint must not be
        able to spend them. This component holds the mode because it already
        holds the beliefs a model needs, and because holding it is what lets
        it track the time since the last transition that the switching set
        asks for.
        """
        if mode != self._mode:
            self._mode_changed_at_s = t_s
        self._mode = mode

    def since_mode_change_s(self, t_s: float) -> float:
        """What the model's switching set needs and cannot hold itself.

        t - t_q is history, not state, and a pure declaration cannot reach it.
        Adding a state to carry it would pay for a decision-hygiene rule with
        the integrator, so the model takes it as an argument and this
        component -- which sees every transition -- supplies it.
        """
        return t_s - self._mode_changed_at_s

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

        The payload is subtracted first, because the gauge and this component
        do not mean the same thing by "fuel". A fuel gauge reports the mass
        above DRY -- it has no way to know what a platform is carrying, and
        should not -- while this component decomposes mass as dry + payload +
        fuel. The two agree only on a clean aircraft.

        Correcting on the raw reading put the payload into the fuel state and
        then added it again in the mass sum, so a platform with 500 kg of
        stores believed itself 500 kg heavier than it was, at a stated sigma
        of 1.4 kg -- 351 sigma wrong while looking perfectly confident. Every
        fixture in the tests left payload at zero, where the two definitions
        coincide and nothing could go wrong.

        Subtracting here rather than configuring the gauge with dry + payload
        is what lets payload become a runtime quantity: a payload manager that
        publishes the current mass as stores are released would leave a
        gauge-side constant stale from the first release onwards.
        """
        H = np.zeros((1, N_ERR))
        H[0, I_FUEL] = 1.0
        R = np.array([[m.mass_above_dry_sigma_kg**2]])

        observed_fuel_kg = m.mass_above_dry_kg - self.par.payload_mass_kg
        innovation = observed_fuel_kg - self._fuel_kg
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
        return self.par.payload_mass_kg + self.vehicle.dry_mass_kg + self._fuel_kg

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
            dry_mass_kg=self.vehicle.dry_mass_kg,
            payload_mass_kg=self.par.payload_mass_kg,
            fuel_mass_kg=self._fuel_kg,
            tsfc_error=self._tsfc_error,
            covariance=self.P.copy(),
        )

    # -- vehicle questions, answered at the believed mass -----------------
    #
    # These are why the component is the sole consumer of PlanarPointMass rather
    # than merely a mass publisher. Guidance needs three different things
    # from the vehicle and only one of them is a capability record: it also
    # needs a parametrised thrust query for its feedforward, and enforcement.
    # Republishing a Capability alone would have left guidance holding a
    # PlanarPointMass anyway, and the mass parameter with it.

    def beliefs(self) -> PlatformBeliefs:
        """Everything this component believes about the platform that
        navigation does not estimate."""
        return PlatformBeliefs(
            mass_kg=self.mass_kg,
            thermal=self._thermal,
            mode=self._mode,
        )

    def believed_state(self, own_state: OwnStateEstimate):
        """The estimate, dressed as the model's own state at the beliefs this
        component holds.

        The model builds it, because the model owns the shape of its state --
        five elements for a point mass, six and a mode for a switched one.
        This component supplies what navigation does not estimate and never
        names a state type, which is what lets it serve either model without
        asking which it holds.

        Returned rather than kept private because the demos and the eventual
        simulation core need it, but note that it is a *believed* state: the
        values came from an estimate and from beliefs, not from a privileged
        query.
        """
        return self.vehicle.state_from(own_state, self.beliefs())

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

    def capability_bound(self, own_state: OwnStateEstimate) -> PromisedEnvelope:
        """The envelope the platform is willing to promise, not the one it
        expects.

        Evaluated at mass + margin * sigma rather than at the believed mass.
        Adding mass is conservative for a manoeuvre limit -- a heavier
        aircraft turns no faster, pulls no more g and stalls no slower -- so
        a single signed margin narrows those claims in every direction at
        once and can never widen them.

        It is NOT conservative for everything the vehicle reports, which is
        why this returns a narrower record than Capability rather than
        forwarding it. Fuel and endurance move the wrong way, because mass
        uncertainty here is fuel uncertainty and a heavier aircraft is
        carrying more of it; accelerations are not monotone in mass at all.
        See PromisedEnvelope for the full accounting. A test walks every
        field of that record and asserts its direction, so adding one without
        deciding which way it should move fails rather than passes.

        This is the only method that applies the margin, and deliberately so.
        Feedforward must use the best estimate: thrust computed for an
        aircraft heavier than the real one is not cautious, it simply
        accelerates. Enforcement likewise clips against what the airframe
        can do, not against what the platform is confident of, or a
        Saturation finding would report the estimator's doubt as though it
        were the vehicle's limit and ADR 0006 would stop meaning anything.

        So: capability() for what you compute with, capability_bound() for
        what you promise upward.

        The margin is invisible once the filter has converged -- three sigma
        of a 2 kg uncertainty against 15 tonnes changes nothing measurable.
        It earns its place exactly when the belief is poor: before the first
        gauge reading, where sigma is the configured 200 kg, and during a
        gauge outage, where it grows without bound.
        """
        sigma = math.sqrt(max(self.P[I_FUEL, I_FUEL], 0.0))
        heavier = self.mass_kg + self.par.capability_margin_sigma * sigma
        margined = dataclasses.replace(self.beliefs(), mass_kg=heavier)
        envelope = self.vehicle.capability(
            self.vehicle.state_from(own_state, margined)
        )

        return PromisedEnvelope(
            max_turn_rate_rad_s=envelope.omega_available_rad_s,
            sustained_turn_rate_rad_s=envelope.omega_sustained_rad_s,
            min_turn_radius_m=envelope.turn_radius_min_m,
            load_factor_available=envelope.load_factor_available,
            min_speed_mps=envelope.v_min_achievable_mps,
            max_speed_mps=envelope.v_max_achievable_mps,
            mass_kg=heavier,
            mass_margin_sigma=self.par.capability_margin_sigma,
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
