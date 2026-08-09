"""Reference configurations for the clock resource. See package docstring.

Illustrative values loosely in the range of a disciplined but not
laboratory-grade oscillator. Fictional, per CLAUDE.md.
"""

from __future__ import annotations

from ose.resource.clock import ClockParameters

STANDARD = ClockParameters(
    drift_sigma=1.0e-9,          # steady-state fractional frequency offset [s/s]
    drift_tau_s=3600.0,          # correlation time of the drift
    white_noise_sigma_s=1.0e-8,  # per-reading timing jitter [s]
)
