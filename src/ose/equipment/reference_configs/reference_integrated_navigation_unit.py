"""Reference configurations for the integrated navigation unit. See
package docstring."""

from __future__ import annotations

import math

from ose.equipment.integrated_navigation_unit import IntegratedNavParameters

STANDARD = IntegratedNavParameters(
    position_sigma_m=15.0,
    heading_sigma_rad=math.radians(0.5),
    airspeed_sigma_mps=1.5,
)
