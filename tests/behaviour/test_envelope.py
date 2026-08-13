"""Flying the envelope: the delivered turn rate traces the corner-speed curve.

Ask for a turn rate no speed can deliver and hold a low speed command, so
thrust falls to idle. The platform decelerates through the envelope pinned
against its instantaneous limit the whole way, and the delivered rate therefore
follows omega_max as a function of speed -- rising while lift allows more,
peaking where the lift and structural limits meet, falling as g/v shrinks.

That peak is `v_corner`, and the vehicle computes it in closed form from mass.
So the sweep is a check that the turn-performance model is self-consistent
across three separate paths: the closed-form corner speed, the instantaneous
limit the capability reports, and what the integrated dynamics actually
deliver. `demos/demo_live_flight.py` calls this the sharpest such check in the
repository and prints the residual instead of asserting it.

It is a behaviour test rather than a capability test because no component makes
the claim. `test_capability.py` checks that a claimed rate is deliverable at
one state; this checks the shape of the whole curve, which only appears when
the platform is flown across it.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from _platform import fly

from ose.equipment.vehicle import VehicleState
from ose.interfaces import TurnRateSpeedSetpoint

# Far above anything achievable, so the command never stops asking. A heading
# setpoint cannot express this: ask for a rate above the achievable one and the
# setpoint laps the vehicle, the error wraps through 180 degrees and guidance
# reverses the turn. That is why TurnRateSpeedSetpoint exists.
IMPOSSIBLE_RATE = math.radians(60.0)
SLOW_COMMAND = 140.0
T_END = 95.0


@pytest.fixture(scope="module")
def sweep():
    return fly(
        lambda t, capability: TurnRateSpeedSetpoint(IMPOSSIBLE_RATE, SLOW_COMMAND),
        T_END,
        initial=VehicleState(0.0, 0.0, 0.0, 380.0, 16000.0),
    )


@pytest.mark.performance
def test_the_turn_stays_pinned_against_the_limit(sweep):
    """An unreachable rate saturates and stays saturated -- no reversals, no
    sawtooth. If this fails the sweep below is measuring something else."""
    rec, _, _ = sweep
    a = rec.arrays()
    settled = a["t"] > 2.0

    assert a["omega_clipped"][settled].all(), (
        f"only {100 * a['omega_clipped'][settled].mean():.0f}% of the sweep was "
        "saturated"
    )
    delivered = a["omega_delivered"][settled]
    assert (delivered > 0).all(), "the turn reversed during the sweep"

    # Against the TRUE limit, and the tolerance is not slack. Guidance clips at
    # the mass the fuel gauge reports, while omega_available here was sampled
    # from the vehicle at its true mass, so the two differ by the gauge error
    # and nothing else -- about one part in ten thousand. An exact match would
    # mean guidance had gone back to reading truth (ADR 0015).
    limit = a["omega_available"][settled]
    assert np.allclose(delivered, limit, rtol=1e-3), (
        "the delivered rate is not the instantaneous limit it was clipped to"
    )
    drift = float(np.abs(delivered - limit).max() / limit.mean())
    assert drift < 1e-3, f"clipped rate drifted {drift:.1e} from the true limit"


@pytest.mark.performance
def test_the_fastest_turn_happens_at_corner_speed(sweep):
    """The peak of the swept curve is the closed-form corner speed.

    Three independent statements of the same physics have to agree: the speed
    at which the flown turn rate peaks, `v_corner_mps` evaluated in closed form
    at the mass flown, and the capability the vehicle reported at that instant.

    The tolerance is a few m/s rather than exact because the sweep samples the
    curve at discrete steps and the peak is flat near the top -- and because
    guidance clips at the BELIEVED mass while v_corner below is evaluated at
    the true one, so the fuel gauge's error appears here as a small residual.
    """
    rec, vehicle, _ = sweep
    a = rec.arrays()
    settled = a["t"] > 2.0

    peak = int(np.argmax(a["omega_delivered"][settled]))
    v_at_peak = a["v"][settled][peak]
    mass_at_peak = a["mass"][settled][peak]
    predicted = vehicle.v_corner_mps(mass_at_peak)

    assert abs(v_at_peak - predicted) < 5.0, (
        f"peak turn rate at {v_at_peak:.1f} m/s, corner speed predicted "
        f"{predicted:.1f} m/s at {mass_at_peak:.0f} kg"
    )


@pytest.mark.performance
def test_the_sweep_actually_crosses_the_corner(sweep):
    """Anti-vacuity, and it is not decorative.

    If the platform decelerated only to just above corner speed, the peak would
    sit at the last sample and the test above would pass while having found the
    end of the data rather than the top of a curve. So the swept band must
    straddle the corner on both sides.
    """
    rec, vehicle, _ = sweep
    a = rec.arrays()
    settled = a["t"] > 2.0
    speeds = a["v"][settled]
    corner = vehicle.v_corner_mps(a["mass"][settled].mean())

    assert speeds.max() > corner + 20.0, "the sweep started too near the corner"
    assert speeds.min() < corner - 20.0, "the sweep stopped before the corner"

    peak = int(np.argmax(a["omega_delivered"][settled]))
    assert 0 < peak < len(speeds) - 1, "the peak is at an end of the sweep"
