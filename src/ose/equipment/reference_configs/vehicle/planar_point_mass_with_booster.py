"""Reference configurations for the two-mode planar point mass.

The same airframe as the baseline reference fighter -- identical geometry,
identical structural and control limits -- with an afterburner added. Sharing
the geometry record rather than restating it is deliberate: a comparison
between the two models is only meaningful if the aerodynamics are the same,
and the model document says the drag model is unchanged under boost.

Numbers are fictional and plausible, as everywhere in this repository.
"""

from __future__ import annotations

from ose.environment import Environment
from ose.equipment.reference_configs.vehicle.planar_point_mass import (
    FIGHTER_GEOMETRY,
    FIGHTER_LIMITS,
)
from ose.equipment.vehicle.planar_point_mass_with_booster import (
    BoostConstraints,
    BoostParameters,
    PlanarPointMassWithBooster,
)
from ose.reference_configs.reference_environment import ISA_SEA_LEVEL

# Afterburner: about 40 per cent more thrust for well over twice the fuel flow.
# The ratio is the point -- boost is expensive, or there would be no reason to
# ever leave it.
FIGHTER_BOOST_TSFC_KG_PER_N_S = 6.0e-5      # against 2.5e-5 nominal
FIGHTER_THERMAL_TAU_HEAT_S = 30.0           # s reaches its limit in 30 s of boost
FIGHTER_THERMAL_TAU_COOL_S = 90.0           # and takes about three times as long back

FIGHTER_BOOST_LIMITS = BoostConstraints(
    nominal=FIGHTER_LIMITS,
    thrust_max_boost_N=180.0e3,             # against 130 kN nominal
    v_max_boost_mps=700.0,                  # against 600 m/s nominal
    # Policy, not physics: 300 kg is held back so that engaging boost cannot
    # commit the fuel a recovery would need.
    mass_reserve_kg=300.0,
    # Physics: the thermal accumulator is normalised, so the limit is 1.
    thermal_max=1.0,
    # Policy: five seconds between transitions, to stop a planner chattering.
    dwell_s=5.0,
)


def reference_boosted_fighter(
    environment: Environment = ISA_SEA_LEVEL,
) -> PlanarPointMassWithBooster:
    """The reference fighter with an afterburner, at sea level unless told
    otherwise. Same argument for the environment as the baseline: the lumped
    induced-drag parameter carries g, so an airframe is not tied to one
    altitude by its configuration."""
    return PlanarPointMassWithBooster(
        BoostParameters(
            nominal=FIGHTER_GEOMETRY.to_parameters(environment),
            c_tsfc_boost=FIGHTER_BOOST_TSFC_KG_PER_N_S,
            tau_h_s=FIGHTER_THERMAL_TAU_HEAT_S,
            tau_c_s=FIGHTER_THERMAL_TAU_COOL_S,
        ),
        FIGHTER_BOOST_LIMITS,
        environment,
    )
