"""Reference configurations for the vehicle manager. See package docstring."""

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

    # The nominal coefficient, matching the reference fighter -- but declared
    # here rather than read from it, so that a mismatch is possible and the
    # filter is testable. See the module docstring in vehicle_manager.py.
    tsfc_kg_per_N_s=2.5e-5,
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
)
