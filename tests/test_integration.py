"""Tests for the integrators, which live outside the components they step
(ADR 0004).

Two things are being pinned here. That rk4_step is genuinely generic --
exercised against a system with no connection to vehicles at all, because
the whole reason it was lifted out of vehicle.py is so a second component
with continuous dynamics does not need a second copy of Runge-Kutta. And
that the normalise hook is applied where it must be: once, at the end. The
heading wrap it carries was previously buried inside the vehicle's own
integrator and had no test at all.
"""

import math

import numpy as np
import pytest

from ose.equipment.reference_configs.reference_vehicle import reference_fighter
from ose.equipment.vehicle import VehicleCommand, VehicleState
from ose.integration import rk4_step, step_rk4


@pytest.fixture
def vehicle():
    return reference_fighter()


# --------------------------------------------------------------------------
# The generic integrator
# --------------------------------------------------------------------------

def test_rk4_step_integrates_a_system_unrelated_to_vehicles():
    """A harmonic oscillator: xdot = v, vdot = -x. Nothing vehicle-shaped
    anywhere. After one full period the state must return to its start."""

    def f(x):
        return np.array([x[1], -x[0]])

    # dt divides the period exactly, so what is left over is integrator
    # error and not the loop stopping short of a full revolution.
    n_steps = 6283
    dt = 2.0 * math.pi / n_steps
    x = np.array([1.0, 0.0])
    for _ in range(n_steps):
        x = rk4_step(f, x, dt)

    assert x[0] == pytest.approx(1.0, abs=1e-9)
    assert x[1] == pytest.approx(0.0, abs=1e-9)


def test_rk4_step_is_fourth_order():
    """Halving the step must cut the error by roughly 2^4, on a problem
    with a known closed form."""

    def f(x):
        return np.array([x[1], -x[0]])

    def endpoint(dt):
        x = np.array([1.0, 0.0])
        for _ in range(int(round(1.0 / dt))):
            x = rk4_step(f, x, dt)
        return x

    exact = np.array([math.cos(1.0), -math.sin(1.0)])
    e_coarse = np.linalg.norm(endpoint(0.1) - exact)
    e_fine = np.linalg.norm(endpoint(0.05) - exact)
    assert e_coarse / e_fine > 8.0


def test_rk4_step_without_normalise_leaves_the_state_untouched():
    """The hook is optional: a component with no angular states passes
    nothing and gets the raw weighted sum."""

    def f(x):
        return np.ones_like(x)

    x = rk4_step(f, np.zeros(3), 0.5)
    assert x == pytest.approx(np.full(3, 0.5))


def test_normalise_is_applied_to_the_result():
    def f(x):
        return np.array([1.0])

    x = rk4_step(f, np.array([0.0]), 1.0, normalise=lambda a: a * 10.0)
    assert x[0] == pytest.approx(10.0)


def test_normalise_is_not_applied_to_intermediate_stages():
    """The subtle one. Stages are derivative evaluations at trial points;
    wrapping between them can inject a 2*pi jump into a difference meant to
    be small. Counting calls is the cheapest way to pin that the hook runs
    once per step, not once per stage."""
    calls = []

    def f(x):
        return np.array([1.0])

    def normalise(a):
        calls.append(a.copy())
        return a

    rk4_step(f, np.array([0.0]), 0.1, normalise=normalise)
    assert len(calls) == 1


def test_rk4_step_does_not_mutate_the_state_it_was_given():
    def f(x):
        return np.ones_like(x)

    x0 = np.zeros(2)
    rk4_step(f, x0, 1.0)
    assert x0 == pytest.approx(np.zeros(2))


# --------------------------------------------------------------------------
# The vehicle's normalisation hook
# --------------------------------------------------------------------------

def test_normalise_state_wraps_heading_into_range(vehicle):
    x = np.array([0.0, 0.0, 3.0 * math.pi, 250.0, 16000.0])
    out = vehicle.normalise_state(x)
    assert out[2] == pytest.approx(math.remainder(3.0 * math.pi, 2.0 * math.pi))
    assert abs(out[2]) <= math.pi


def test_normalise_state_leaves_non_angular_states_alone(vehicle):
    """Speed and mass are deliberately not clamped here. Clamping would be
    enforcement, and enforcement belongs to guidance, not to the model or
    the integrator (ADR 0006)."""
    x = np.array([1.0, -2.0, 0.0, -50.0, -3.0])
    out = vehicle.normalise_state(x)
    assert out[0] == 1.0 and out[1] == -2.0
    assert out[3] == -50.0     # nonsensical speed, passed through
    assert out[4] == -3.0      # nonsensical mass, passed through


def test_normalise_state_does_not_mutate_its_argument(vehicle):
    x = np.array([0.0, 0.0, 3.0 * math.pi, 250.0, 16000.0])
    vehicle.normalise_state(x)
    assert x[2] == pytest.approx(3.0 * math.pi)


def test_stepping_a_vehicle_keeps_heading_wrapped(vehicle):
    """End to end: turning for long enough to pass through several
    revolutions must not accumulate an unbounded heading."""
    state = VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0)
    cmd = VehicleCommand(vehicle.lam.thrust_max_N, 0.3)
    for _ in range(int(120.0 / 0.02)):
        state = step_rk4(vehicle, state, cmd, 0.02)
        assert abs(state.psi_rad) <= math.pi + 1e-12
