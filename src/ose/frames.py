"""
Rotation utilities for the planar NED frame. No dependencies on any other
ose module, so both the resource-layer sensors and the subsystem estimator
can depend on it without binding to each other.

Frame and sign conventions follow vehicle.py: p_x north, p_y east, psi
clockwise from north, omega positive right.
"""

from __future__ import annotations

import math

import numpy as np


def rotation(psi: float) -> np.ndarray:
    """Body to navigation frame. Body x forward, y right; nav x north, y east."""
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s], [s, c]])


def d_rotation(psi: float) -> np.ndarray:
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[-s, -c], [c, -s]])
