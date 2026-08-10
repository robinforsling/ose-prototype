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
)
