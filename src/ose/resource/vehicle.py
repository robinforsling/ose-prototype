"""
Two-dimensional vehicle model: dynamics, constraints, capability.

Implements the baseline (nominal-mode) model, with two additions to the
constraints and capability layers that are discussed in the accompanying notes:

  * a lift parameter c_l = 0.5 * S * C_Lmax, mirroring c_p, giving a
    speed- and mass-dependent turn rate limit instead of a constant omega_max;
  * a closed-form sustained turn rate.

Conventions
-----------
Frame       : planar projection of NED. p_x towards north, p_y towards east,
              metres. Constant altitude.
Heading     : psi measured clockwise from north, that is from +p_x towards
              +p_y, radians. Hence pdot_x = v cos(psi), pdot_y = v sin(psi).
Turn rate   : omega > 0 is a right turn, corresponding to positive bank.
Speed       : v is AIRSPEED. Wind enters the position kinematics only, so psi
              is an air-relative heading and ground track differs from psi
              whenever wind is non-zero.
Units       : SI throughout. Angles in radians internally.

Enforcement
-----------
This model DECLARES the admissible sets X(lambda) and U(lambda). It does not
enforce them. No method here projects, saturates or clamps a command or a
state. Responsibility for respecting the sets lies with guidance, motion
planning, or an interposed runtime assurance layer.

Two utilities are offered for that layer to call -- admissible() and
project_command() -- but nothing in the integration path invokes them. Outside
the declared sets the dynamics remain integrable, but the model is not claimed
to represent the vehicle faithfully.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ose.environment import Environment


# --------------------------------------------------------------------------
# Parameter groups (theta, eta, lambda in the notation of the model document)
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class VehicleParameters:
    """theta: aerodynamic and propulsion parameters of the vehicle."""

    c_p: float          # parasite drag parameter, 0.5 * S * C_D0            [m^2]
    c_i: float          # induced drag parameter, 2 g^2 / (pi e AR S)        [1/s^4]
    c_l: float          # lift parameter, 0.5 * S * C_Lmax                   [m^2]
    c_tsfc: float       # thrust specific fuel consumption                   [kg/(N s)]

    @classmethod
    def from_geometry(
        cls,
        wing_area_m2: float,
        cd0: float,
        oswald_e: float,
        aspect_ratio: float,
        cl_max: float,
        tsfc_kg_per_N_s: float,
        g: float,
    ) -> "VehicleParameters":
        """Build the lumped parameters from conventional aerodynamic geometry.

        Useful when a contributor knows S, C_D0, e, AR rather than c_p, c_i.
        g is the gravitational acceleration used to derive the induced drag
        parameter; callers supply it explicitly (e.g. from a reference
        environment) rather than relying on an implicit standard-gravity
        default here, per the shape/data split in ose/environment.py.
        """
        return cls(
            c_p=0.5 * wing_area_m2 * cd0,
            c_i=2.0 * g**2 / (math.pi * oswald_e * aspect_ratio * wing_area_m2),
            c_l=0.5 * wing_area_m2 * cl_max,
            c_tsfc=tsfc_kg_per_N_s,
        )


@dataclass(frozen=True)
class Constraints:
    """lambda: limits defining admissible states and inputs."""

    thrust_min_N: float
    thrust_max_N: float
    n_structural: float         # structural load factor limit          [-]
    omega_max_rad_s: float      # absolute turn rate cap (roll/control)  [rad/s]
    v_min_mps: float            # hard floor, independent of stall       [m/s]
    v_max_mps: float
    mass_dry_kg: float


@dataclass(frozen=True)
class Disturbance:
    """w: exogenous disturbance. Ground-truth quantity, not known to the platform."""

    wind_x_mps: float = 0.0
    wind_y_mps: float = 0.0
    omega_dist_rad_s: float = 0.0
    force_long_N: float = 0.0

    @property
    def is_zero(self) -> bool:
        return (
            self.wind_x_mps == 0.0
            and self.wind_y_mps == 0.0
            and self.omega_dist_rad_s == 0.0
            and self.force_long_N == 0.0
        )


NO_DISTURBANCE = Disturbance()


# --------------------------------------------------------------------------
# State and command
# --------------------------------------------------------------------------

@dataclass
class VehicleState:
    """x = [p_x, p_y, psi, v, m]"""

    p_x_m: float
    p_y_m: float
    psi_rad: float
    v_mps: float
    mass_kg: float

    def to_array(self) -> np.ndarray:
        return np.array(
            [self.p_x_m, self.p_y_m, self.psi_rad, self.v_mps, self.mass_kg],
            dtype=float,
        )

    @classmethod
    def from_array(cls, a: np.ndarray) -> "VehicleState":
        return cls(float(a[0]), float(a[1]), float(a[2]), float(a[3]), float(a[4]))


@dataclass
class VehicleCommand:
    """u = [T, omega]"""

    thrust_N: float
    omega_rad_s: float


@dataclass
class Capability:
    """c = g(x, theta, eta, lambda, w) -- what the vehicle can currently achieve.

    This is the vehicle's contribution to the platform capability descriptor.
    Every field is answerable without integrating the dynamics forward.
    """

    thrust_available_N: float
    thrust_required_N: float        # to hold current v at current omega
    accel_max_mps2: float
    accel_min_mps2: float
    omega_available_rad_s: float    # instantaneous, limit of the turn
    omega_sustained_rad_s: float    # largest rate holdable without losing speed
    turn_radius_min_m: float
    load_factor_available: float
    v_stall_mps: float              # at the current mass and 1 g
    v_corner_mps: float             # speed of peak instantaneous turn rate
    fuel_mass_kg: float
    endurance_s: float


@dataclass
class Saturation:
    """Record of any command clipping, so violations are visible not silent."""

    thrust_clipped: bool = False
    omega_clipped: bool = False
    notes: list[str] = field(default_factory=list)

    @property
    def any(self) -> bool:
        return self.thrust_clipped or self.omega_clipped


# --------------------------------------------------------------------------
# The model
# --------------------------------------------------------------------------

class Vehicle2D:
    """Baseline nominal-mode vehicle.

    Three public surfaces, deliberately kept separate:
      derivative()   -- the dynamics, f(x, u, theta, eta) + G(x) w
      admissible()   -- the constraint sets X(lambda), U(lambda)
      capability()   -- the capability model g(.)
    """

    def __init__(
        self,
        parameters: VehicleParameters,
        constraints: Constraints,
        environment: Environment,
    ) -> None:
        self.theta = parameters
        self.lam = constraints
        self.eta = environment

    # ---------------- aerodynamics ----------------

    def drag_N(self, v_mps: float, mass_kg: float, omega_rad_s: float) -> float:
        """D(v, m, omega) -- parabolic polar with load-factor-scaled induced term."""
        rho, g = self.eta.rho, self.eta.g
        parasite = rho * self.theta.c_p * v_mps**2
        n_squared = 1.0 + (v_mps * omega_rad_s / g) ** 2
        induced = (self.theta.c_i / rho) * (mass_kg**2 / v_mps**2) * n_squared
        return parasite + induced

    def load_factor(self, v_mps: float, omega_rad_s: float) -> float:
        """n = sqrt(1 + (v omega / g)^2), from the coordinated turn relations."""
        return math.sqrt(1.0 + (v_mps * omega_rad_s / self.eta.g) ** 2)

    def bank_angle_rad(self, v_mps: float, omega_rad_s: float) -> float:
        return math.atan2(v_mps * omega_rad_s, self.eta.g)

    def lift_limited_load_factor(self, v_mps: float, mass_kg: float) -> float:
        """n_l = rho c_l v^2 / (m g) -- the most the wing can pull at this speed."""
        return (
            self.eta.rho * self.theta.c_l * v_mps**2 / (mass_kg * self.eta.g)
        )

    def v_stall_mps(self, mass_kg: float, load_factor: float = 1.0) -> float:
        """Speed at which the required load factor exactly exhausts available lift."""
        return math.sqrt(
            load_factor * mass_kg * self.eta.g / (self.eta.rho * self.theta.c_l)
        )

    def v_corner_mps(self, mass_kg: float) -> float:
        """Corner speed: v_stall * sqrt(n_structural). Peak instantaneous turn rate."""
        return self.v_stall_mps(mass_kg) * math.sqrt(self.lam.n_structural)

    # ---------------- turn performance ----------------

    def omega_max_rad_s(self, v_mps: float, mass_kg: float) -> float:
        """Instantaneous turn rate limit.

        Binding constraint is the smallest of:
          - the lift the wing can generate at this speed and mass,
          - the structural load factor limit,
          - an absolute rate cap standing in for roll and control authority.
        """
        n_lift = self.lift_limited_load_factor(v_mps, mass_kg)
        n_eff = min(n_lift, self.lam.n_structural)
        if n_eff <= 1.0:
            return 0.0                      # cannot even hold a level turn
        omega_aero = (self.eta.g / v_mps) * math.sqrt(n_eff**2 - 1.0)
        return min(omega_aero, self.lam.omega_max_rad_s)

    def omega_sustained_rad_s(self, v_mps: float, mass_kg: float) -> float:
        """Largest turn rate at which max thrust still balances drag.

        Solving T_max = D(v, m, omega) for omega:
            omega = (g/v) sqrt( (T_max - rho c_p v^2 - A) / A ),  A = c_i m^2/(rho v^2)
        Returns 0 when the vehicle cannot sustain even wings-level flight.
        Never exceeds the instantaneous limit.
        """
        rho, g = self.eta.rho, self.eta.g
        A = self.theta.c_i * mass_kg**2 / (rho * v_mps**2)
        excess = self.lam.thrust_max_N - rho * self.theta.c_p * v_mps**2 - A
        if excess <= 0.0:
            return 0.0
        omega = (g / v_mps) * math.sqrt(excess / A)
        return min(omega, self.omega_max_rad_s(v_mps, mass_kg))

    def thrust_required_N(
        self, v_mps: float, mass_kg: float, omega_rad_s: float = 0.0
    ) -> float:
        """Thrust for steady flight at this speed and turn rate. T_req = D."""
        return self.drag_N(v_mps, mass_kg, omega_rad_s)

    # ---------------- dynamics ----------------

    def derivative(
        self,
        state: VehicleState,
        command: VehicleCommand,
        disturbance: Disturbance = NO_DISTURBANCE,
    ) -> np.ndarray:
        """xdot = f(x, u, theta, eta) + G(x) w."""
        v, m = state.v_mps, state.mass_kg
        T, omega = command.thrust_N, command.omega_rad_s

        drag = self.drag_N(v, m, omega)

        # Fuel flow stops once the tanks are dry.
        burning = m > self.lam.mass_dry_kg
        mdot = -self.theta.c_tsfc * T if burning else 0.0

        return np.array(
            [
                v * math.cos(state.psi_rad) + disturbance.wind_x_mps,
                v * math.sin(state.psi_rad) + disturbance.wind_y_mps,
                omega + disturbance.omega_dist_rad_s,
                (T - drag + disturbance.force_long_N) / m,
                mdot,
            ],
            dtype=float,
        )

    # ---------------- constraints ----------------

    def project_command(
        self, state: VehicleState, command: VehicleCommand
    ) -> tuple[VehicleCommand, Saturation]:
        """Project a command onto U(x, lambda). OFFERED, NOT APPLIED.

        The vehicle never calls this itself. It exists so that a guidance law
        or a runtime assurance layer can perform the projection explicitly, and
        so that the projection is reported rather than silent -- a controller
        persistently commanding outside the envelope is a finding, not a detail
        to be swallowed by the integrator.
        """
        sat = Saturation()

        T = command.thrust_N
        T_max = self.thrust_available_N(state)
        if T > T_max:
            T, sat.thrust_clipped = T_max, True
            sat.notes.append(f"thrust {command.thrust_N:.0f} N clipped to {T_max:.0f} N")
        elif T < self.lam.thrust_min_N:
            T, sat.thrust_clipped = self.lam.thrust_min_N, True
            sat.notes.append(f"thrust {command.thrust_N:.0f} N raised to minimum")

        omega_lim = self.omega_max_rad_s(state.v_mps, state.mass_kg)
        omega = command.omega_rad_s
        if abs(omega) > omega_lim:
            omega = math.copysign(omega_lim, omega)
            sat.omega_clipped = True
            sat.notes.append(
                f"turn rate {math.degrees(command.omega_rad_s):.2f} deg/s clipped "
                f"to {math.degrees(omega_lim):.2f} deg/s"
            )

        return VehicleCommand(T, omega), sat

    def state_violations(self, state: VehicleState) -> list[str]:
        """Report which elements of X(lambda) are violated. Diagnostic only.

        Nothing is corrected. A violation means the model is being evaluated
        outside its declared region of validity, which is information the
        guidance layer or the analyst needs, not something to be hidden.
        """
        out: list[str] = []
        v_floor = max(self.lam.v_min_mps, self.v_stall_mps(state.mass_kg))
        if state.v_mps < v_floor:
            out.append(f"airspeed {state.v_mps:.1f} below floor {v_floor:.1f} m/s")
        if state.v_mps > self.lam.v_max_mps:
            out.append(f"airspeed {state.v_mps:.1f} above {self.lam.v_max_mps:.1f} m/s")
        if state.mass_kg < self.lam.mass_dry_kg:
            out.append(f"mass {state.mass_kg:.0f} below dry mass")
        return out

    def admissible(self, state: VehicleState, command: VehicleCommand) -> bool:
        v_floor = max(self.lam.v_min_mps, self.v_stall_mps(state.mass_kg))
        return (
            v_floor <= state.v_mps <= self.lam.v_max_mps
            and state.mass_kg >= self.lam.mass_dry_kg
            and self.lam.thrust_min_N <= command.thrust_N <= self.thrust_available_N(state)
            and abs(command.omega_rad_s)
            <= self.omega_max_rad_s(state.v_mps, state.mass_kg)
        )

    def thrust_available_N(self, state: VehicleState) -> float:
        return self.lam.thrust_max_N if state.mass_kg > self.lam.mass_dry_kg else 0.0

    # ---------------- capability ----------------

    def capability(
        self,
        state: VehicleState,
        omega_rad_s: float = 0.0,
        disturbance: Disturbance = NO_DISTURBANCE,
    ) -> Capability:
        """c = g(x, theta, eta, lambda, w).

        Note on the disturbance argument: passing the TRUE disturbance yields
        the vehicle's true capability, which only the simulation core is
        entitled to know. Anything on the cyber side of the platform should
        call this with an estimated disturbance, or with none, and will
        therefore get a slightly wrong answer -- which is the correct
        behaviour, not a defect.
        """
        v, m = state.v_mps, state.mass_kg
        T_avail = self.thrust_available_N(state)
        drag = self.drag_N(v, m, omega_rad_s)
        F = disturbance.force_long_N

        omega_avail = self.omega_max_rad_s(v, m)
        fuel = max(m - self.lam.mass_dry_kg, 0.0)
        T_for_endurance = self.thrust_required_N(v, m, omega_rad_s)
        endurance = (
            fuel / (self.theta.c_tsfc * T_for_endurance)
            if T_for_endurance > 0.0 and self.theta.c_tsfc > 0.0
            else math.inf
        )

        return Capability(
            thrust_available_N=T_avail,
            thrust_required_N=T_for_endurance,
            accel_max_mps2=(T_avail - drag + F) / m,
            accel_min_mps2=(self.lam.thrust_min_N - drag + F) / m,
            omega_available_rad_s=omega_avail,
            omega_sustained_rad_s=self.omega_sustained_rad_s(v, m),
            turn_radius_min_m=v / omega_avail if omega_avail > 0.0 else math.inf,
            load_factor_available=self.load_factor(v, omega_avail),
            v_stall_mps=self.v_stall_mps(m),
            v_corner_mps=self.v_corner_mps(m),
            fuel_mass_kg=fuel,
            endurance_s=endurance,
        )

    # ---------------- reachability helpers for the planner ----------------

    def time_to_turn_s(self, state: VehicleState, delta_psi_rad: float) -> float:
        """Time to change heading by delta_psi at the sustained rate.

        Uses the sustained rather than instantaneous rate, so the answer is one
        the vehicle can actually hold without decaying to stall.
        """
        omega = self.omega_sustained_rad_s(state.v_mps, state.mass_kg)
        if omega <= 0.0:
            return math.inf
        return abs(math.remainder(delta_psi_rad, 2.0 * math.pi)) / omega

    def can_reach(
        self,
        state: VehicleState,
        target_xy_m: tuple[float, float],
        horizon_s: float,
        tolerance_m: float = 0.0,
    ) -> bool:
        """Conservative reachability test.

        Approximates the Dubins-style manoeuvre as: turn to face the target at
        the sustained rate, then run straight at current speed. Conservative
        because it ignores the distance covered during the turn and assumes no
        acceleration. Cheap enough to call inside a planning loop.
        """
        dx = target_xy_m[0] - state.p_x_m
        dy = target_xy_m[1] - state.p_y_m
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        t_turn = self.time_to_turn_s(state, bearing - state.psi_rad)
        if not math.isfinite(t_turn) or t_turn >= horizon_s:
            return False
        t_run = max(distance - tolerance_m, 0.0) / state.v_mps
        return t_turn + t_run <= horizon_s


# --------------------------------------------------------------------------
# Integration
# --------------------------------------------------------------------------

def step_rk4(
    vehicle: Vehicle2D,
    state: VehicleState,
    command: VehicleCommand,
    dt_s: float,
    disturbance: Disturbance = NO_DISTURBANCE,
) -> VehicleState:
    """Advance the state by dt using classical RK4, holding the command constant.

    Integrates whatever it is given. No projection onto U(lambda), no clamping
    onto X(lambda). If the command is inadmissible the caller will get the
    integral of an inadmissible command, which is the intended behaviour --
    enforcement belongs to guidance, not to the vehicle.
    """

    def f(a: np.ndarray) -> np.ndarray:
        return vehicle.derivative(VehicleState.from_array(a), command, disturbance)

    x = state.to_array()
    k1 = f(x)
    k2 = f(x + 0.5 * dt_s * k1)
    k3 = f(x + 0.5 * dt_s * k2)
    k4 = f(x + dt_s * k3)
    x_next = x + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x_next[2] = math.remainder(x_next[2], 2.0 * math.pi)   # wrap heading only

    return VehicleState.from_array(x_next)
