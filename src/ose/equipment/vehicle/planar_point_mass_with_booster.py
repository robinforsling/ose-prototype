"""
Planar point-mass vehicle model with two propulsion modes, nominal and boost.

Section 5 of docs/vehicle/vehicle_model.pdf. The aerodynamics, the disturbance
model and the turn-rate bound are unchanged from the baseline; what boost
changes is how much thrust is available, how fast fuel is consumed, and how
long either can be sustained.

    State (6)   x = [p_x, p_y, psi, v, m, s]
    Input       u = [T, omega], plus the discrete mode q in {NOMINAL, BOOST}

The sixth state s is a thermal accumulator: it fills at 1/tau_h while boost is
engaged and decays at -s/tau_c otherwise. It is what makes boost a finite
resource rather than a free upgrade, and it is why this is a separate model
rather than a flag on the baseline -- a state the baseline never writes has no
business in the baseline's Jacobian or integrator.

Why the turn-rate bound carries no mode index
---------------------------------------------
Remark 5.1 of the document, and worth repeating because the opposite is the
intuitive guess. Instantaneous turn rate is limited by available lift and by
structural strength, and boost affects neither. Boost improves turn
performance through the SUSTAINED rate, which depends on available thrust: it
does not let the aircraft turn tighter, it lets a given turn be held without
losing airspeed. An earlier formulation with a mode-dependent omega_max is
recorded in the document as inconsistent with the aerodynamic model.

Declared, not enforced
----------------------
The mode is a discrete input, so the switching set S_q is an input constraint
and ADR 0006 applies to it exactly as to thrust. This model DECLARES which
transitions are admissible via admissible_modes(); project_command() OFFERS a
fallback to nominal and reports it; derivative() integrates whatever mode it
is given. Commanding boost past the thermal limit therefore behaves like
commanding 200 kN against a 130 kN engine: still integrable, not claimed
valid, and reported -- s runs past s_max and state_violations() says so.

The dwell time is an argument, not a state
------------------------------------------
The document writes the switching set in terms of t - t_q, the time since the
last transition. That is history, not state, and a pure declaration cannot
reach it -- adding a seventh state to carry it would be paying for a
decision-hygiene rule with the integrator. So admissible_modes() takes the
elapsed time as an argument and whoever owns the mode supplies it, the same
way the mass filter takes the burn coefficient rather than holding it.

Physics and policy are mixed in lambda, deliberately but not silently
---------------------------------------------------------------------
Of the three restrictions on engaging boost, only the thermal one is physical:
the engine genuinely cannot. A fuel reserve and a minimum dwell are declared
policy -- an aircraft can physically light its afterburner on the last
hundred kilograms, it simply should not, and chattering is a hazard to the
planner and to the integrator rather than to the engine. Both live in lambda
alongside v_min, which is already described as a hard floor independent of
stall. A contributor modelling a different aircraft should know that the
thermal limit is the one they cannot simply relax.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

import numpy as np

from ose.environment import Environment
from ose.equipment.vehicle.records import (
    NO_DISTURBANCE,
    Capability,
    Constraints,
    Disturbance,
    Saturation,
    VehicleCommand,
    VehicleParameters,
)


class Mode(Enum):
    """q. NOMINAL is the mode a platform falls back to, never one it is
    refused."""

    NOMINAL = "nominal"
    BOOST = "boost"


@dataclass
class BoostState:
    """x = [p_x, p_y, psi, v, m, s], plus the mode currently engaged.

    The mode is part of the state, not of the command, and the model document
    is already written that way: S_q(x, lambda) describes transitions FROM the
    current mode, so the current mode is a discrete state and only the
    REQUESTED mode is an input. That is the ordinary hybrid-systems
    formulation, and it is what makes every method here take the same
    arguments as the baseline's -- which is what lets one vehicle manager
    serve both models without asking which it holds.

    It is excluded from to_array(). The integrator advances the six continuous
    states; the mode is held constant across a step by construction, so a
    caller closes over it and hands it back to from_array(). Integrating
    across a switch would integrate a discontinuity.
    """

    p_x_m: float
    p_y_m: float
    psi_rad: float
    v_mps: float
    mass_kg: float
    thermal: float              # s, dimensionless, 0 is cold and 1 is the limit
    mode: "Mode" = None         # type: ignore[assignment]

    def __post_init__(self) -> None:
        if self.mode is None:
            self.mode = Mode.NOMINAL

    def to_array(self) -> np.ndarray:
        """The continuous states only. The mode is discrete and held."""
        return np.array(
            [self.p_x_m, self.p_y_m, self.psi_rad, self.v_mps, self.mass_kg,
             self.thermal],
            dtype=float,
        )

    @classmethod
    def from_array(cls, a: np.ndarray, mode: "Mode" = None) -> "BoostState":
        return cls(*(float(v) for v in a), mode=mode or Mode.NOMINAL)


@dataclass(frozen=True)
class BoostParameters:
    """theta = [c_p, c_i, c_l, c_tsfc_nom, c_tsfc_boost, tau_h, tau_c].

    The first four are exactly the baseline's, so they are composed rather
    than restated: `nominal.c_tsfc` is c_tsfc_nom. One definition of the drag
    polar, and a geometry record lumps into it unchanged.
    """

    nominal: VehicleParameters
    c_tsfc_boost: float         # > nominal.c_tsfc
    tau_h_s: float              # thermal accumulation time constant
    tau_c_s: float              # thermal recovery time constant

    def c_tsfc(self, mode: Mode) -> float:
        return self.c_tsfc_boost if mode is Mode.BOOST else self.nominal.c_tsfc


@dataclass(frozen=True)
class BoostConstraints:
    """lambda, extended. The baseline's seven entries are the NOMINAL ones."""

    nominal: Constraints
    thrust_max_boost_N: float   # > nominal.thrust_max_N
    v_max_boost_mps: float      # > nominal.v_max_mps
    mass_reserve_kg: float      # fuel below which boost is inhibited (policy)
    thermal_max: float          # s_max (physics)
    dwell_s: float              # minimum time between transitions (policy)

    def thrust_max_N(self, mode: Mode) -> float:
        return self.thrust_max_boost_N if mode is Mode.BOOST else self.nominal.thrust_max_N

    def v_max_mps(self, mode: Mode) -> float:
        return self.v_max_boost_mps if mode is Mode.BOOST else self.nominal.v_max_mps


@dataclass
class BoostCapability(Capability):
    """The baseline's channels, evaluated at the current mode, plus two.

    Subclasses rather than replaces, so a consumer typed to Capability keeps
    working and only a consumer that cares about boost needs to know the type.
    """

    boost_available: bool
    boost_time_remaining_s: float


class PlanarPointMassWithBooster:
    """Two-mode planar point mass. See the module docstring."""

    def __init__(
        self,
        parameters: BoostParameters,
        constraints: BoostConstraints,
        environment: Environment,
    ) -> None:
        self.theta = parameters
        self.lam = constraints
        self.eta = environment

    @property
    def dry_mass_kg(self) -> float:
        """The airframe's mass without fuel or payload.

        Exposed by the model rather than left for a consumer to find in
        lambda, because lambda's shape is model specific -- a switched model
        composes the baseline's limits inside its own record -- and a
        consumer navigating that structure would be coupled to the model it
        is meant to be independent of.
        """
        return self.lam.nominal.mass_dry_kg

    def state_from(self, estimate, beliefs) -> BoostState:
        """Dress a published own-state estimate as this model's state.

        Same contract as the baseline's, and the reason both have one: a
        consumer holding either model calls this and gets a state it can use,
        without knowing how many elements that state has. This model reads two
        more beliefs than the baseline -- the thermal accumulator and the mode
        currently engaged -- neither of which navigation estimates.
        """
        return BoostState(
            estimate.p_x_m,
            estimate.p_y_m,
            estimate.psi_rad,
            estimate.v_air_mps,
            beliefs.mass_kg,
            beliefs.thermal,
            beliefs.mode or Mode.NOMINAL,
        )

    # ---------------- aerodynamics, unchanged from the baseline -----------

    def drag_N(self, v_mps: float, mass_kg: float, omega_rad_s: float) -> float:
        n = self.load_factor(v_mps, omega_rad_s)
        parasite = self.eta.rho * self.theta.nominal.c_p * v_mps**2
        induced = (
            (self.theta.nominal.c_i / self.eta.rho) * (mass_kg**2 / v_mps**2) * n**2
        )
        return parasite + induced

    def load_factor(self, v_mps: float, omega_rad_s: float) -> float:
        return math.sqrt(1.0 + (v_mps * omega_rad_s / self.eta.g) ** 2)

    def lift_limited_load_factor(self, v_mps: float, mass_kg: float) -> float:
        return (
            self.eta.rho * self.theta.nominal.c_l * v_mps**2
            / (mass_kg * self.eta.g)
        )

    def v_stall_mps(self, mass_kg: float, load_factor: float = 1.0) -> float:
        return math.sqrt(
            load_factor * mass_kg * self.eta.g
            / (self.eta.rho * self.theta.nominal.c_l)
        )

    def omega_max_rad_s(self, v_mps: float, mass_kg: float) -> float:
        """Mode independent. See remark 5.1 and the module docstring."""
        n_lift = self.lift_limited_load_factor(v_mps, mass_kg)
        n = min(n_lift, self.lam.nominal.n_structural)
        if n <= 1.0:
            return 0.0
        return min(
            self.eta.g * math.sqrt(n**2 - 1.0) / v_mps,
            self.lam.nominal.omega_max_rad_s,
        )

    def thrust_required_N(
        self, v_mps: float, mass_kg: float, omega_rad_s: float = 0.0
    ) -> float:
        return self.drag_N(v_mps, mass_kg, omega_rad_s)

    # ---------------- mode-dependent dynamics -----------------------------

    def thermal_rate(self, state: BoostState) -> float:
        """sigma_q(s). Published as a derivative so a consumer integrates it
        (ADR 0004) -- the vehicle manager dead-reckons the thermal belief this
        way rather than reimplementing the law."""
        if state.mode is Mode.BOOST:
            return 1.0 / self.theta.tau_h_s
        return -state.thermal / self.theta.tau_c_s

    def derivative(
        self,
        state: BoostState,
        command: VehicleCommand,
        disturbance: Disturbance = NO_DISTURBANCE,
    ) -> np.ndarray:
        """xdot = f_q(x, u, theta, eta) + G(x) w, equation f-mode.

        The mode is a separate argument rather than a field of the command
        because it is held constant across an integration step by the caller;
        a switch inside the step would integrate a discontinuity. Pure: no
        hidden state, no clamping, no opinion about whether the mode was
        admissible.
        """
        v, m = state.v_mps, state.mass_kg
        T, omega = command.thrust_N, command.omega_rad_s

        drag = self.drag_N(v, m, omega)
        burning = m > self.lam.nominal.mass_dry_kg
        mdot = -self.theta.c_tsfc(state.mode) * T if burning else 0.0

        return np.array(
            [
                v * math.cos(state.psi_rad) + disturbance.wind_x_mps,
                v * math.sin(state.psi_rad) + disturbance.wind_y_mps,
                omega + disturbance.omega_dist_rad_s,
                (T - drag + disturbance.force_long_N) / m,
                mdot,
                self.thermal_rate(state),
            ],
            dtype=float,
        )

    def normalise_state(self, x: np.ndarray) -> np.ndarray:
        """Heading is still the only angular state. The thermal accumulator is
        deliberately not clamped here: clamping would be enforcement, and
        running past s_max is a finding for state_violations() to report."""
        out = np.array(x, dtype=float, copy=True)
        out[2] = math.remainder(out[2], 2.0 * math.pi)
        return out

    # ---------------- constraints, declared not enforced ------------------

    def admissible_modes(
        self, state: BoostState, since_transition_s: float
    ) -> frozenset[Mode]:
        """S_q(x, lambda), equation switching-concrete.

        Declared. Nothing here refuses a transition; a caller that ignores
        this and commands boost anyway gets boost, and the consequences show
        up in state_violations().
        """
        fuel = state.mass_kg - self.lam.nominal.mass_dry_kg
        can_boost = (
            state.thermal < self.lam.thermal_max
            and fuel > self.lam.mass_reserve_kg
            and since_transition_s >= self.lam.dwell_s
        )
        return frozenset(Mode) if can_boost else frozenset({Mode.NOMINAL})

    def project_command(
        self,
        state: BoostState,
        command: VehicleCommand,
        since_transition_s: float = math.inf,
    ) -> tuple[VehicleCommand, Saturation]:
        """Project onto U_q(x, lambda) and S_q. OFFERED, NOT APPLIED.

        The requested mode rides on the command; the current one is in the
        state. That is the q+ and q of the model document, and it is why this
        takes the same arguments as the baseline's project_command -- a
        consumer need not know which model it is holding.

        A mode cannot be clipped by degree the way thrust can -- there is no
        "less boost" -- so an inadmissible request falls back to NOMINAL,
        which is what the document's S_q = {nominal} otherwise already says.
        The delivered mode is returned on the command, so a caller that flies
        what it is given flies a consistent pair.
        """
        notes: list[str] = []
        requested = command.mode or state.mode
        delivered_mode = requested
        if requested not in self.admissible_modes(state, since_transition_s):
            delivered_mode = Mode.NOMINAL
            notes.append(
                f"mode {requested.value} not admissible, fell back to nominal"
            )

        thrust_max = self.lam.thrust_max_N(delivered_mode)
        thrust = min(max(command.thrust_N, self.lam.nominal.thrust_min_N), thrust_max)
        if thrust != command.thrust_N:
            notes.append(
                f"thrust {command.thrust_N:.0f} N clipped to {thrust:.0f} N "
                f"in {delivered_mode.value}"
            )

        omega_cap = self.omega_max_rad_s(state.v_mps, state.mass_kg)
        omega = math.copysign(min(abs(command.omega_rad_s), omega_cap),
                              command.omega_rad_s)
        if omega != command.omega_rad_s:
            notes.append(
                f"turn rate {math.degrees(command.omega_rad_s):.2f} deg/s clipped "
                f"to {math.degrees(omega):.2f} deg/s"
            )

        return (
            VehicleCommand(thrust, omega, mode=delivered_mode),
            Saturation(
                thrust_clipped=thrust != command.thrust_N,
                omega_clipped=omega != command.omega_rad_s,
                notes=notes,
                requested=command,
            ),
        )

    def state_violations(self, state: BoostState) -> list[str]:
        """X_q(lambda). Reported, never corrected."""
        out: list[str] = []
        v_floor = max(self.lam.nominal.v_min_mps, self.v_stall_mps(state.mass_kg))
        if state.v_mps < v_floor:
            out.append(f"speed {state.v_mps:.1f} below floor {v_floor:.1f} m/s")
        v_ceiling = self.lam.v_max_mps(state.mode)
        if state.v_mps > v_ceiling:
            out.append(
                f"speed {state.v_mps:.1f} above {state.mode.value} limit "
                f"{v_ceiling:.1f} m/s"
            )
        if state.mass_kg < self.lam.nominal.mass_dry_kg:
            out.append(f"mass {state.mass_kg:.0f} below dry mass")
        if state.thermal > self.lam.thermal_max:
            out.append(
                f"thermal state {state.thermal:.2f} above limit "
                f"{self.lam.thermal_max:.2f}"
            )
        if state.thermal < 0.0:
            out.append(f"thermal state {state.thermal:.2f} below zero")
        return out

    # ---------------- capability ------------------------------------------

    def thrust_available_N(self, state: BoostState) -> float:
        if state.mass_kg <= self.lam.nominal.mass_dry_kg:
            return 0.0
        return self.lam.thrust_max_N(state.mode)

    def omega_sustained_rad_s(
        self, v_mps: float, mass_kg: float, mode: Mode
    ) -> float:
        """The channel boost actually improves. Evaluated with T_av,q."""
        rho, g = self.eta.rho, self.eta.g
        thrust = self.lam.thrust_max_N(mode)
        A = self.theta.nominal.c_i * mass_kg**2 / (rho * v_mps**2)
        available = thrust - rho * self.theta.nominal.c_p * v_mps**2
        if available <= 0.0 or A <= 0.0:
            return 0.0
        ratio = available / A
        if ratio <= 1.0:
            return 0.0
        return min(
            g * math.sqrt(ratio - 1.0) / v_mps,
            self.omega_max_rad_s(v_mps, mass_kg),
        )

    def capability(
        self,
        state: BoostState,
        omega_rad_s: float = 0.0,
        since_transition_s: float = math.inf,
    ) -> BoostCapability:
        """c = g_q(x, theta, eta, lambda, w), section 5.4."""
        v, m = state.v_mps, state.mass_kg
        thrust_av = self.thrust_available_N(state)
        drag = self.drag_N(v, m, omega_rad_s)
        omega_av = self.omega_max_rad_s(v, m)
        n_av = min(self.lift_limited_load_factor(v, m), self.lam.nominal.n_structural)
        fuel = max(m - self.lam.nominal.mass_dry_kg, 0.0)
        thrust_req = self.thrust_required_N(v, m, omega_rad_s)
        c_tsfc = self.theta.c_tsfc(state.mode)

        return BoostCapability(
            thrust_available_N=thrust_av,
            thrust_required_N=thrust_req,
            accel_max_mps2=(thrust_av - drag) / m,
            accel_min_mps2=(self.lam.nominal.thrust_min_N - drag) / m,
            omega_available_rad_s=omega_av,
            omega_sustained_rad_s=self.omega_sustained_rad_s(v, m, state.mode),
            turn_radius_min_m=v / omega_av if omega_av > 0.0 else math.inf,
            load_factor_available=n_av,
            v_stall_mps=self.v_stall_mps(m),
            v_corner_mps=self.v_stall_mps(m) * math.sqrt(self.lam.nominal.n_structural),
            fuel_mass_kg=fuel,
            endurance_s=(
                fuel / (c_tsfc * thrust_req)
                if thrust_req > 0.0 and c_tsfc > 0.0
                else math.inf
            ),
            v_min_achievable_mps=max(
                self.lam.nominal.v_min_mps, self.v_stall_mps(m)
            ),
            v_max_achievable_mps=self.lam.v_max_mps(state.mode),
            boost_available=Mode.BOOST in self.admissible_modes(
                state, since_transition_s
            ),
            # How long boost could be held from here, equation boost-available.
            boost_time_remaining_s=max(
                self.theta.tau_h_s * (self.lam.thermal_max - state.thermal), 0.0
            ),
        )
