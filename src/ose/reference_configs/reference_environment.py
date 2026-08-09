"""Reference environment configurations. See package docstring."""

from __future__ import annotations

from ose.environment import G_STANDARD, RHO_SEA_LEVEL_ISA, Environment

ISA_SEA_LEVEL = Environment(g=G_STANDARD, rho=RHO_SEA_LEVEL_ISA)
