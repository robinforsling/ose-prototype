"""
Records shared by every planar vehicle model.

Not a model. These are the types that do not change when the dynamics do: the
authored aerodynamics, the lumped parameters and limits, the disturbance, the
command, the capability record and the saturation receipt. The model document
states outright that the drag and disturbance models are unchanged under the
two-mode formulation, and a saturation receipt is model-independent by nature.

A model's own state vector is NOT here, and deliberately. A switched model
carries a thermal accumulator the baseline never writes, and a shared state
record would put that field in the baseline's state vector, its Jacobian and
its integrator, where nothing would maintain it. States live with their model.

Parameters and constraints are shared as the *nominal* core and composed, not
duplicated: the two-mode parameter vector of the model document is
[c_p, c_i, c_l, c_tsfc_nom, c_tsfc_boost, tau_h, tau_c], whose first four are
exactly VehicleParameters, and the extended constraint vector likewise contains
the baseline's seven. Composition keeps one definition of the drag polar.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import ClassVar

from ose.environment import Environment


@dataclass(frozen=True)
class VehicleGeometry:
    """What a contributor actually knows about an airframe.

    The authored form of the aerodynamics, as distinct from the lumped form
    the dynamics use. A configuration states wing area and a drag polar;
    VehicleParameters holds c_p, c_i and c_l, which are products of those and
    of the environment. Keeping the two apart is what lets a configuration be
    data -- a record, and one day a block of YAML -- rather than a function
    that has to run to produce a vehicle.

    Note what to_parameters() needs: g. The lumped induced-drag parameter is
    not a property of the airframe alone, so the same geometry yields
    different parameters in different environments. That is the reason a
    configuration must not pin an environment: doing so silently fixes an
    airframe to one gravity and one air density.
    """

    wing_area_m2: float
    cd0: float                  # zero-lift drag coefficient
    oswald_e: float             # span efficiency
    aspect_ratio: float
    cl_max: float
    tsfc_kg_per_N_s: float

    def to_parameters(self, environment: Environment) -> "VehicleParameters":
        """Lump the geometry into the parameters the dynamics integrate."""
        return VehicleParameters(
            c_p=0.5 * self.wing_area_m2 * self.cd0,
            c_i=(
                2.0 * environment.g**2
                / (math.pi * self.oswald_e * self.aspect_ratio * self.wing_area_m2)
            ),
            c_l=0.5 * self.wing_area_m2 * self.cl_max,
            c_tsfc=self.tsfc_kg_per_N_s,
        )


@dataclass(frozen=True)
class VehicleParameters:
    """theta: aerodynamic and propulsion parameters of the vehicle.

    The lumped form, derived from VehicleGeometry and an Environment. Written
    directly only when someone genuinely has c_p and c_i rather than a drag
    polar; a configuration should author geometry instead.
    """

    c_p: float          # parasite drag parameter, 0.5 * S * C_D0            [m^2]
    c_i: float          # induced drag parameter, 2 g^2 / (pi e AR S)        [1/s^4]
    c_l: float          # lift parameter, 0.5 * S * C_Lmax                   [m^2]
    c_tsfc: float       # thrust specific fuel consumption                   [kg/(N s)]


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
    # Maximum total mass: airframe, fuel and everything mounted on it.
    #
    # Unlike every other entry here this one can only be violated by how the
    # platform was LOADED, never by how it is flown -- mass falls
    # monotonically as fuel burns, so a state that starts inside stays
    # inside. It is declared here anyway, rather than left to the composition
    # checks, for two reasons: it is a property of the airframe and belongs
    # with the airframe's other limits, and a scenario that hand-builds an
    # overloaded initial state deserves the same finding a badly specified
    # platform gets.
    mass_max_kg: float


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
class VehicleCommand:
    """u = [T, omega], and for a switched model the mode being requested.

    A model with one propulsion setting ignores the mode; a model with two
    reads it as q+, the mode asked for, against the q it finds in the state.
    An unused field on a command costs nothing -- unlike an unused state,
    which would sit in the Jacobian and the integrator -- so this is shared
    rather than split per model, and every project_command takes the same
    arguments as a result.
    """

    INTERFACE: ClassVar[str] = "vehicle.command.v1"

    thrust_N: float
    omega_rad_s: float
    mode: object | None = None


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

    # The speed band the vehicle can actually hold right now. The floor is
    # composed -- whichever of stall-at-this-mass and the airframe's hard
    # minimum binds -- which is why it is reported here rather than left for
    # each consumer to assemble from v_stall_mps and Constraints. Working it
    # out is the same rule admissible() applies, and a consumer that
    # reimplements it will eventually reimplement it differently.
    v_min_achievable_mps: float     # max(hard minimum, stall at this mass)
    v_max_achievable_mps: float     # airframe limit; not mass dependent


@dataclass
class Saturation:
    """Record of any command clipping, so violations are visible not silent.

    `requested` carries the command as it arrived, before enforcement, and is
    always populated -- not only when something was clipped. Without it the
    pre-enforcement command is unobservable from outside, since
    project_command() returns only what survived, and a caller wanting to show
    or log what was asked for has to recompute it by duplicating the control
    law. demos/demo_vehicle_guidance.py did exactly that and the duplicate
    went stale the first time the law changed, plotting a feedforward the
    guidance no longer used.

    The notes stay: they are for a human reading a log. The numbers are for
    everything else.
    """

    # The same interface as the command it describes, deliberately. This is the
    # receipt that travels with a VehicleCommand rather than a wire format of
    # its own -- docs/interfaces/README.md documents it under that heading --
    # and naming it separately would suggest a consumer could bind one without
    # the other.
    INTERFACE: ClassVar[str] = "vehicle.command.v1"

    thrust_clipped: bool = False
    omega_clipped: bool = False
    notes: list[str] = field(default_factory=list)
    requested: "VehicleCommand | None" = None

    @property
    def any(self) -> bool:
        return self.thrust_clipped or self.omega_clipped


# --------------------------------------------------------------------------
# The model
# --------------------------------------------------------------------------

