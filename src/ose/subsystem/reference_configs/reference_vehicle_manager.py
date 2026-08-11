"""Reference configurations for the vehicle manager. See package docstring.

BELIEVED_TSFC_KG_PER_N_S is not a field of VehicleManagerParameters: the filter
takes the burn coefficient as an argument to predict() so that it need not know
how many operating modes an engine has. It is declared here, next to the
component that will be driven with it, and it is the platform's BELIEF about
its engine -- deliberately a separate number from the coefficient the vehicle
actually burns at, even though the two currently agree. Deriving it from
PlanarPointMass.theta.c_tsfc would make the prediction exact by construction and
every consistency test vacuous;
test_no_cyber_component_reads_the_true_burn_coefficient enforces that.

It is homeless in the strict sense -- a believed engine model belongs to the
vehicle system, which does not exist. When it does, this moves there. When the
booster model lands it becomes one value per mode, and predict() does not
change.
"""

from __future__ import annotations

from ose.subsystem.vehicle_manager import VehicleManagerParameters

# Matches the reference fighter: 12 000 kg dry, so this is the 16 000 kg
# platform every demo and test in the repository flies.
STANDARD = VehicleManagerParameters(
    payload_mass_kg=0.0,
    initial_fuel_kg=4000.0,
    # A tenth of the load, and an order of magnitude worse than the fuel
    # gauge's 20 kg. Deliberately: this is what the platform assumes before
    # anything has measured, and it should not read as though it had been.
    initial_fuel_sigma_kg=200.0,

    # Five per cent. Engine-to-engine variation plus calibration error; over
    # five minutes of cruise this is worth about 18 kg, comparable to the
    # gauge's own noise, which is why the coefficient is weakly observable
    # rather than either free or pinned.
    tsfc_sigma_fraction=0.05,
    # Slow: the coefficient moves with engine wear, not minute to minute.
    # Non-zero only so the filter cannot become arbitrarily certain of it.
    tsfc_walk_per_sqrt_s=2.0e-5,
    # Unmodelled burn. Small against a 1.5 kg/s cruise flow.
    fuel_walk_kg_per_sqrt_s=0.02,

    # Three sigma on the promised envelope. Worth nothing once the filter has
    # converged -- three sigma of 2 kg against 15 tonnes -- and worth 600 kg
    # before the first gauge reading, which is when a planner most needs to
    # be told what the platform is actually confident of.
    capability_margin_sigma=3.0,
)

# The platform's believed burn coefficient. Matches the reference fighter's
# authored value today; it is a separate declaration so that it CAN differ,
# which is what makes tsfc_error a meaningful state.
BELIEVED_TSFC_KG_PER_N_S = 2.5e-5
