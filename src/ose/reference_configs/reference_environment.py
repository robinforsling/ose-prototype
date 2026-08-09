"""Reference environment configurations. See package docstring."""

from __future__ import annotations

from ose.environment import Environment

G_STANDARD = 9.80665        # standard gravity, ISA                [m/s^2]
RHO_SEA_LEVEL_ISA = 1.225   # ISA sea-level air density             [kg/m^3]

ISA_SEA_LEVEL = Environment(g=G_STANDARD, rho=RHO_SEA_LEVEL_ISA)
