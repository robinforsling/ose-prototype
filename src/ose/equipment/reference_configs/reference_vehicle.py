"""Reference configurations for the vehicle. See package docstring.

Authored as records, not as a factory that builds them. Every other reference
config in this package is a plain parameter record -- `TACTICAL_GRADE =
ImuParameters(...)` -- and the vehicle used to be the exception: a function
that derived parameters, chose an environment and constructed a component, all
at once. That made "add a custom vehicle configuration" mean "write Python
that constructs a component", which is a barrier for someone who only has
numbers, lets logic leak into what should be data, and cannot be expressed as
the YAML the descriptor format assumes.

Split apart, a configuration is FIGHTER_GEOMETRY and FIGHTER_LIMITS: two
records a contributor can read, copy, or vary with dataclasses.replace. The
factory below is a convenience over them, not the definition of the vehicle.
"""

from __future__ import annotations

import math

from ose.environment import Environment
from ose.equipment.vehicle import Constraints, Vehicle2D, VehicleGeometry
from ose.reference_configs.reference_environment import ISA_SEA_LEVEL

# Generic fighter-like airframe. Plausible, not a real aircraft.
FIGHTER_GEOMETRY = VehicleGeometry(
    wing_area_m2=38.0,
    cd0=0.022,
    oswald_e=0.80,
    aspect_ratio=3.0,
    cl_max=1.20,
    tsfc_kg_per_N_s=2.5e-5,
)

FIGHTER_LIMITS = Constraints(
    thrust_min_N=5.0e3,
    thrust_max_N=130.0e3,
    n_structural=9.0,
    omega_max_rad_s=math.radians(30.0),
    v_min_mps=90.0,
    v_max_mps=600.0,
    mass_dry_kg=12000.0,
)


def reference_fighter(environment: Environment = ISA_SEA_LEVEL) -> Vehicle2D:
    """The reference fighter, at sea level unless told otherwise.

    The environment is an argument because it is not part of the airframe:
    the same geometry lumps to different parameters under different gravity,
    so a configuration that pinned one would be fixing an aeroplane to an
    altitude. It is defaulted rather than required only because the standard
    atmosphere is itself a named reference config and every demo wants it --
    the pairing is a convenience, not a property of the vehicle.
    """
    return Vehicle2D(
        FIGHTER_GEOMETRY.to_parameters(environment), FIGHTER_LIMITS, environment
    )
