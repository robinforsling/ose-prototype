"""
Environmental parameters: physical constants describing the atmosphere a
vehicle -- or any other physics in this repository -- operates in.

No dependencies, the same role as frames.py: any layer might need to know
ambient gravity and air density, so this cannot live inside vehicle.py or
any other single component. It does not need truth-boundary treatment
either (contrast Disturbance, which does): g and rho are fixed simulation
constants in this project's scope, not something estimated from noisy
sensors, so there is nothing being hidden from anyone.

Environment declares shape only, no default values -- the same rule as
VehicleParameters and Constraints in vehicle.py, and for the same reason.
A specific reference point (e.g. sea-level ISA) is reference-config data,
not shape, so it lives in reference_configs/reference_environment.py, not
here. G_STANDARD and RHO_SEA_LEVEL_ISA are the exception: standardised
physical constants, not scenario choices, so they stay alongside the shape
they parameterise.

A full altitude-varying atmosphere model is explicitly out of scope --
constant altitude, per docs/00-scope.md's "Explicitly out of scope" list --
so resist the temptation to add one here; that needs a scope decision
first, not just a bigger Environment.
"""

from __future__ import annotations

from dataclasses import dataclass

G_STANDARD = 9.80665        # standard gravity, ISA                [m/s^2]
RHO_SEA_LEVEL_ISA = 1.225   # ISA sea-level air density             [kg/m^3]


@dataclass(frozen=True)
class Environment:
    """eta: environmental parameters."""

    g: float
    rho: float
