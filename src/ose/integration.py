"""
Integrators. External to the components they step, per ADR 0004.

A component with continuous dynamics publishes `f(x, u, theta, eta, w)` and
nothing else -- no discretisation, no integrator, no time step. Choosing how
to discretise is the caller's job, and this module is where the choices
live so that no component has to ship one. When the simulation core exists
it becomes the principal caller; until then the demos and tests are.

Two levels are offered deliberately:

  rk4_step   generic, over a plain state vector. Reusable by any component
             with continuous dynamics, which is the point -- a second such
             component (an effector with its own flyout dynamics, say) must
             not need a second copy of Runge-Kutta.

  step_rk4   a typed convenience for Vehicle2D, built on rk4_step. It lives
             here rather than in vehicle.py precisely so the vehicle module
             ships dynamics and nothing else; it is the seam the simulation
             core will eventually subsume.

On state normalisation: a generic integrator cannot know that a particular
element of the state vector is an angle needing wrapping, so it does not
guess. The component knows, and says so through a `normalise` callable --
`Vehicle2D.normalise_state` for the vehicle. Integrating without it still
produces correct dynamics, just an unwrapped heading that grows without
bound.
"""

from __future__ import annotations

from typing import Callable

import numpy as np

from ose.resource.vehicle import (
    NO_DISTURBANCE,
    Disturbance,
    Vehicle2D,
    VehicleCommand,
    VehicleState,
)


def rk4_step(
    f: Callable[[np.ndarray], np.ndarray],
    x: np.ndarray,
    dt_s: float,
    normalise: Callable[[np.ndarray], np.ndarray] | None = None,
) -> np.ndarray:
    """Classical fourth-order Runge-Kutta over one step, holding f's inputs
    other than the state constant across the step.

    `normalise` is applied once to the result, after the weighted sum, and
    is where a component wraps angular states. Applying it to the
    intermediate stages instead would be wrong: the stages are derivative
    evaluations at trial points, and wrapping between them can introduce a
    2*pi jump into a difference that is supposed to be small.
    """
    k1 = f(x)
    k2 = f(x + 0.5 * dt_s * k1)
    k3 = f(x + 0.5 * dt_s * k2)
    k4 = f(x + dt_s * k3)
    x_next = x + (dt_s / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return normalise(x_next) if normalise is not None else x_next


def step_rk4(
    vehicle: Vehicle2D,
    state: VehicleState,
    command: VehicleCommand,
    dt_s: float,
    disturbance: Disturbance = NO_DISTURBANCE,
) -> VehicleState:
    """Advance a vehicle state by dt using classical RK4, holding the
    command constant.

    Integrates whatever it is given. No projection onto U(lambda), no
    clamping onto X(lambda). If the command is inadmissible the caller will
    get the integral of an inadmissible command, which is the intended
    behaviour -- enforcement belongs to guidance, not to the vehicle, and
    not to the integrator either. See ADR 0006.
    """

    def f(a: np.ndarray) -> np.ndarray:
        return vehicle.derivative(VehicleState.from_array(a), command, disturbance)

    x_next = rk4_step(f, state.to_array(), dt_s, normalise=vehicle.normalise_state)
    return VehicleState.from_array(x_next)
