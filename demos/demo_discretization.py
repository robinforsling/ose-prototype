"""
One continuous-time model, several discretizations.

Demonstrates that the vehicle's derivative() -- a pure function f(x,u,theta,eta,w)
-- can be consumed by any integrator without the model knowing anything about
time steps, and that a discrete linear model can be derived from it numerically
rather than hand-derived.

Three consumers of the same f:
  1. fixed-step RK4 at several step sizes      (the simulation core)
  2. adaptive high-accuracy solver              (offline reference / validation)
  3. Jacobian + zero-order-hold linearisation   (filters, MPC, digital twin)

Run with:  python demo_discretization.py
"""

from __future__ import annotations

import math

import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import expm

from ose.integration import step_rk4
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import NO_DISTURBANCE, VehicleCommand, VehicleState

N_STATE = 5
N_INPUT = 2


# ---------------------------------------------------------------------------
# Adapters: wrap the model's derivative() for each consumer.
# The model itself is untouched by all of this.
# ---------------------------------------------------------------------------

def f_array(vehicle, x: np.ndarray, u: np.ndarray, w=NO_DISTURBANCE) -> np.ndarray:
    """Plain array interface to the model, for numerical tooling."""
    return vehicle.derivative(
        VehicleState.from_array(x), VehicleCommand(float(u[0]), float(u[1])), w
    )


def integrate_fixed_step(vehicle, x0, u, t_end, dt):
    """Consumer 1: fixed-step RK4. Deterministic, reproducible, no adaptivity."""
    state = VehicleState.from_array(x0.copy())
    cmd = VehicleCommand(float(u[0]), float(u[1]))
    n = int(round(t_end / dt))
    for _ in range(n):
        state = step_rk4(vehicle, state, cmd, dt)
    return state.to_array()


def integrate_adaptive(vehicle, x0, u, t_end, rtol=1e-12, atol=1e-12):
    """Consumer 2: adaptive solver. Accurate reference, but NOT reproducible
    across platform counts or orderings -- unsuitable for the simulation core."""
    sol = solve_ivp(
        lambda t, x: f_array(vehicle, x, u),
        (0.0, t_end),
        x0,
        method="DOP853",
        rtol=rtol,
        atol=atol,
        dense_output=False,
    )
    return sol.y[:, -1]


def jacobians(vehicle, x, u, eps_rel=1e-6):
    """Central-difference Jacobians A = df/dx, B = df/du at an operating point.

    Finite differences are used here to keep the dependency footprint at zero.
    An automatic differentiation package (CasADi, JAX) would give these exactly
    from the same derivative() with no additional model code.
    """
    A = np.zeros((N_STATE, N_STATE))
    B = np.zeros((N_STATE, N_INPUT))

    for j in range(N_STATE):
        h = eps_rel * max(abs(x[j]), 1.0)
        xp, xm = x.copy(), x.copy()
        xp[j] += h
        xm[j] -= h
        A[:, j] = (f_array(vehicle, xp, u) - f_array(vehicle, xm, u)) / (2.0 * h)

    for j in range(N_INPUT):
        h = eps_rel * max(abs(u[j]), 1.0)
        up, um = u.copy(), u.copy()
        up[j] += h
        um[j] -= h
        B[:, j] = (f_array(vehicle, x, up) - f_array(vehicle, x, um)) / (2.0 * h)

    return A, B


def discretize_zoh(A, B, dt):
    """Consumer 3: exact zero-order-hold discretisation of the linearisation.

    Uses the block matrix exponential
        expm([[A, B], [0, 0]] dt) = [[Ad, Bd], [0, I]].
    """
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


# ---------------------------------------------------------------------------

def main() -> None:
    vehicle = reference_fighter()

    # Operating point: 250 m/s, 16 t, in a steady right turn at the sustained rate.
    v0, m0 = 250.0, 16000.0
    omega0 = vehicle.omega_sustained_rad_s(v0, m0)
    T0 = vehicle.thrust_required_N(v0, m0, omega0)

    x0 = np.array([0.0, 0.0, 0.0, v0, m0])
    u0 = np.array([T0, omega0])

    print("Operating point")
    print("-" * 66)
    print(f"  v      = {v0:8.1f} m/s")
    print(f"  m      = {m0:8.0f} kg")
    print(f"  omega  = {math.degrees(omega0):8.3f} deg/s  (sustained)")
    print(f"  T      = {T0 / 1e3:8.2f} kN     (trim)")

    # --- 1 & 2: same f, different integrators ------------------------------
    t_end = 60.0
    reference = integrate_adaptive(vehicle, x0, u0, t_end)

    print(f"\nSame f(x,u), different discretisations, t = {t_end:.0f} s")
    print("-" * 66)
    print(f"{'method':>22} {'dt [s]':>8} {'pos err [m]':>13} {'v err [m/s]':>13}")
    print(f"{'DOP853 adaptive':>22} {'--':>8} {'reference':>13} {'reference':>13}")

    for dt in (0.5, 0.1, 0.02, 0.005):
        xf = integrate_fixed_step(vehicle, x0, u0, t_end, dt)
        pos_err = math.hypot(xf[0] - reference[0], xf[1] - reference[1])
        v_err = abs(xf[3] - reference[3])
        print(f"{'fixed-step RK4':>22} {dt:>8.3f} {pos_err:>13.2e} {v_err:>13.2e}")

    # --- 3: Jacobians and ZOH discrete model --------------------------------
    A, B = jacobians(vehicle, x0, u0)

    print("\nA = df/dx at the operating point (rows: px py psi v m)")
    print("-" * 66)
    with np.printoptions(precision=4, suppress=True, linewidth=100):
        print(A)
        print("\nB = df/du  (columns: T, omega)")
        print(B)

    # Eigenvalues: what a control theorist actually wants from this.
    eig = np.linalg.eigvals(A)
    print("\neigenvalues of A:", np.array2string(eig, precision=5, suppress_small=True))

    # Compare linear prediction against the nonlinear model over a short horizon.
    dt_pred = 0.1
    Ad, Bd = discretize_zoh(A, B, dt_pred)

    print(f"\nLinear ZOH prediction vs nonlinear truth, dt = {dt_pred} s")
    print("-" * 66)
    print(f"{'horizon [s]':>12} {'pos err [m]':>14} {'v err [m/s]':>14}")

    for horizon in (0.5, 2.0, 5.0, 20.0):
        n_steps = int(round(horizon / dt_pred))

        # linear rollout about the operating point
        dx = np.zeros(N_STATE)
        for _ in range(n_steps):
            dx = Ad @ dx + Bd @ np.zeros(N_INPUT)
        x_lin = x0 + dx + np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0]
        )  # deviation form; nominal drift added below
        # nominal motion under constant u, from the linearisation's own f
        x_lin = x0 + dx + f_array(vehicle, x0, u0) * horizon

        x_true = integrate_adaptive(vehicle, x0, u0, horizon)
        pos_err = math.hypot(x_lin[0] - x_true[0], x_lin[1] - x_true[1])
        v_err = abs(x_lin[3] - x_true[3])
        print(f"{horizon:>12.1f} {pos_err:>14.2f} {v_err:>14.4f}")

    print(
        "\nThe linear model is exact to first order and degrades with horizon,"
        "\nwhich is the expected and useful behaviour for a filter or an MPC."
        "\nNothing in vehicle.py was modified to produce any of the above."
    )


if __name__ == "__main__":
    main()
