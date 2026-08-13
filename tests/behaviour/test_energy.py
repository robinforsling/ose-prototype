"""A turn harder than the airframe can sustain costs speed, and keeps costing.

Command a rate between the sustained and the instantaneous limit and the
platform can enter the turn but cannot hold it: drag at that load factor
exceeds the thrust available, so speed bleeds, and as speed falls the
instantaneous limit falls with it. The circle spirals inward.

Nothing in the code says this. The vehicle reports two turn rates and does not
relate them over time; guidance flies what it is asked within the envelope and
has no notion of energy. That a sustained-rate exceedance is *paid for* in
speed rather than being free is a property of the whole loop run forward, which
is what makes it a behaviour test.

`demos/demo_live_flight.py` flies this as "360 at 9 g" and describes the energy
cost as the point of the manoeuvre.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from _platform import fly

from ose.equipment.vehicle import VehicleState
from ose.interfaces import TurnRateSpeedSetpoint

ENTRY_SPEED = 250.0
T_END = 40.0


@pytest.fixture(scope="module")
def hard_turn():
    """Commanded at the instantaneous limit for the entry speed, which is well
    above what can be sustained there. The speed command asks to hold entry
    speed, so nothing is deliberately throttling back -- the loss is the turn's
    doing."""
    return fly(
        lambda t, capability: TurnRateSpeedSetpoint(
            capability.max_turn_rate_rad_s, ENTRY_SPEED
        ),
        T_END,
        initial=VehicleState(0.0, 0.0, 0.0, ENTRY_SPEED, 16000.0),
    )


@pytest.mark.performance
def test_the_commanded_turn_is_beyond_what_can_be_sustained(hard_turn):
    """The premise. If the commanded rate were sustainable there would be no
    energy cost to find, and every assertion below would pass vacuously on a
    platform that simply flew a circle."""
    rec, _, _ = hard_turn
    a = rec.arrays()
    assert (a["omega_delivered"] > a["omega_sustained"]).mean() > 0.9, (
        "the commanded turn was sustainable -- this test proves nothing"
    )


@pytest.mark.performance
def test_speed_bleeds_monotonically_through_the_turn(hard_turn):
    """Energy goes somewhere. Speed falls, and keeps falling, for as long as
    the turn is held."""
    rec, _, _ = hard_turn
    a = rec.arrays()
    settled = a["t"] > 1.0
    speed = a["v"][settled]

    assert speed[-1] < speed[0] - 20.0, (
        f"speed only fell from {speed[0]:.0f} to {speed[-1]:.0f} m/s"
    )
    # Monotone to within integration noise, not merely lower at the end: a
    # platform that lost speed and recovered it would satisfy the endpoints
    # while not being in a sustained-rate exceedance at all.
    assert np.diff(speed).max() < 1e-3, "speed recovered during the held turn"


@pytest.mark.performance
def test_the_achievable_turn_rate_falls_with_the_speed(hard_turn):
    """The spiral. As speed bleeds the instantaneous limit falls too, so the
    turn the platform can fly gets tighter and slower -- the reason the circle
    does not close on itself."""
    rec, _, _ = hard_turn
    a = rec.arrays()
    settled = a["t"] > 1.0

    speed = a["v"][settled]
    available = a["omega_available"][settled]
    assert available[-1] < available[0], (
        "the available turn rate did not fall as speed bled"
    )

    # Below corner speed the limit is lift-bound and falls WITH speed, so the
    # two move together. Above it they move oppositely, which is why the entry
    # speed is chosen below the corner.
    assert np.corrcoef(speed, available)[0, 1] > 0.9, (
        "available rate and speed did not move together below corner speed"
    )


@pytest.mark.performance
def test_the_turn_tightens_as_it_decays(hard_turn):
    """Radius is v/omega. Speed falls faster than the rate does, so the circle
    spirals in rather than opening out -- the visible signature of the
    manoeuvre, and the thing a plot of the ground track shows."""
    rec, _, _ = hard_turn
    a = rec.arrays()
    settled = a["t"] > 1.0

    radius = a["v"][settled] / a["omega_delivered"][settled]
    assert radius[-1] < radius[0], (
        f"turn radius grew from {radius[0]:.0f} to {radius[-1]:.0f} m"
    )
    assert math.isfinite(radius[-1]) and radius[-1] > 0.0
