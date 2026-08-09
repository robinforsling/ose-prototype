"""Reference configurations for the vehicle resource. See package docstring."""

from __future__ import annotations

import math

from ose.reference_configs.reference_environment import G_STANDARD, ISA_SEA_LEVEL
from ose.resource.vehicle import Constraints, Vehicle2D, VehicleParameters


def reference_fighter() -> Vehicle2D:
    """Generic fighter-like configuration, at sea level. Plausible, not a
    real aircraft."""
    theta = VehicleParameters.from_geometry(
        wing_area_m2=38.0,
        cd0=0.022,
        oswald_e=0.80,
        aspect_ratio=3.0,
        cl_max=1.20,
        tsfc_kg_per_N_s=2.5e-5,
        g=G_STANDARD,
    )
    lam = Constraints(
        thrust_min_N=5.0e3,
        thrust_max_N=130.0e3,
        n_structural=9.0,
        omega_max_rad_s=math.radians(30.0),
        v_min_mps=90.0,
        v_max_mps=600.0,
        mass_dry_kg=12000.0,
    )
    return Vehicle2D(theta, lam, ISA_SEA_LEVEL)
