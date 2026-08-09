"""
Environmental parameters: physical constants describing the atmosphere a
vehicle -- or any other physics in this repository -- operates in.

No dependencies, the same role as frames.py: any layer might need to know
ambient gravity and air density, so this cannot live inside vehicle.py or
any other single component. It does not need truth-boundary treatment
either (contrast Disturbance, which does): g and rho are fixed simulation
constants in this project's scope, not something estimated from noisy
sensors, so there is nothing being hidden from anyone.

Shape only: no default values and no named constants, the same rule as
VehicleParameters and Constraints in vehicle.py, and for the same reason.
Specific values -- including standard gravity and ISA sea-level density --
are reference-config data, not shape, however standardised or universal
they are, and belong in reference_configs/reference_environment.py, not
here.

A full altitude-varying atmosphere model is explicitly out of scope --
constant altitude, per docs/00-scope.md's "Explicitly out of scope" list --
so resist the temptation to add one here; that needs a scope decision
first, not just a bigger Environment.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Environment:
    """eta: environmental parameters."""

    g: float
    rho: float
