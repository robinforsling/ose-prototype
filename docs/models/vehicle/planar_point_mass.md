# `planar_point_mass` — planar point-mass vehicle

`ose.equipment.vehicle.planar_point_mass.PlanarPointMass`

The baseline vehicle. A point mass in the horizontal plane with a parabolic
drag polar, one propulsion setting, and no discrete modes. Mathematics and
derivations are in [`docs/preliminary_models/vehicle/vehicle_model.pdf`](../../preliminary_models/vehicle/vehicle_model.pdf)
sections 2–3; this file describes what the implementation does and how it
behaves, with the notation unchanged.

All numbers below are for the bundled `reference_fighter()` at sea level
(`ISA_SEA_LEVEL`, $g = 9.80665$, $\rho = 1.225$). They are fictional and
plausible, never claims about a real aircraft.

---

## Notation

Symbols are plain: no bold, no blackboard, no calligraphic. A weighted glyph
depends on a font file the reader's browser may never load, and a font that
fails to load is not an error — it falls back to the face a plain scalar
already uses, so the distinction would live in the source and not on the page.
It is declared here instead, which says more than weight could, and it matches
the Python, where nothing is bold either (ADR 0018).

| | kind | meaning |
|---|---|---|
| $x$ | vector, 5 | state |
| $u$ | vector, 2 | motion command |
| $w$ | vector, 2 | process noise, wind |
| $\theta$ | vector, 4 | lumped vehicle parameters |
| $\eta$ | vector, 2 | environment |
| $\lambda$ | vector, 8 | declared constraints |
| $f$ | vector field, 5 | system dynamics |
| $G$ | matrix, 5×2 | noise input |
| $U$, $X$ | sets | admissible commands, admissible states |
| everything else | scalar | $v$, $m$, $T$, $\omega$, $c_p$, … |

---

## 1. State, input, parameters

$$
x = \begin{bmatrix} p_x & p_y & \psi & v & m \end{bmatrix}^{T},
\qquad
u = \begin{bmatrix} T & \omega \end{bmatrix}^{T}
$$

| | meaning | units | code |
|---|---|---|---|
| $p_x$ | position, north | m | `VehicleState.p_x_m` |
| $p_y$ | position, east | m | `VehicleState.p_y_m` |
| $\psi$ | heading, clockwise from north | rad | `VehicleState.psi_rad` |
| $v$ | **airspeed**, not ground speed | m/s | `VehicleState.v_mps` |
| $m$ | total mass | kg | `VehicleState.mass_kg` |
| $T$ | thrust | N | `VehicleCommand.thrust_N` |
| $\omega$ | turn rate, positive right | rad/s | `VehicleCommand.omega_rad_s` |

`to_array()` and `from_array()` move between the record and the raw vector the
integrator sees; element 2 is an angle, which is why the model also publishes
`normalise_state()` — an integrator cannot know that on its own.

### Parameters $\theta$

$$
\theta = \begin{bmatrix} c_p & c_i & c_\ell & c_{\mathrm{TSFC}} \end{bmatrix}^{T}
$$

These are *lumped* and are not what a contributor authors. `VehicleGeometry`
holds the authored form — wing area and a drag polar — and `to_parameters(η)`
lumps it:

$$
c_p = \tfrac{1}{2} S\thinspace C_{D0},
\qquad
c_i = \frac{2 g^{2}}{\pi e\thinspace AR\thinspace S},
\qquad
c_\ell = \tfrac{1}{2} S\thinspace C_{L\max},
\qquad
c_{\mathrm{TSFC}} = c_{\mathrm{TSFC}}
$$

<!-- generated: theta -->
| authored | reference value | | lumped | reference value |
|---|---|---|---|---|
| $S$ | 38.0 m² | | $c_p$ | 0.418 m² |
| $C_{D0}$ | 0.022 | | $c_i$ | 0.671 s⁻⁴ |
| $e$ | 0.8 | | $c_\ell$ | 22.8 m² |
| $AR$ | 3.0 | | $c_{\mathrm{TSFC}}$ | $2.5\times10^{-5}$ kg/(N s) |
| $C_{L\max}$ | 1.2 | | | |
<!-- end generated: theta -->

**$c_i$ carries $g$.** The same airframe therefore lumps to different
parameters under different gravity, which is why a configuration takes an
environment as an argument rather than pinning one.

### Environment $\eta$

$$
\eta = \begin{bmatrix} g & \rho \end{bmatrix}^{T}
$$

Held by the model and supplied at construction. Not part of the vehicle.

---

## 2. Dynamics

$$
\dot{x} = f(x, u, \theta, \eta) + G(x)w,
\qquad
f =
\begin{bmatrix}
v\cos\psi \cr
v\sin\psi \cr
\omega \cr
\dfrac{T - D(v,m,\omega)}{m} \cr
-c_{\mathrm{TSFC}}\thinspace T
\end{bmatrix}
$$

with drag

$$
D(v, m, \omega) = \rho c_p v^{2} + \underbrace{\frac{c_i m^{2}}{\rho v^{2}}}_{A(v,m)}\left[1 + \left(\frac{v\omega}{g}\right)^{2}\right]
$$

`derivative()` is **pure**: no hidden state, no RNG, no clamping, no opinion
about whether the command was admissible (ADR 0004). The consumer chooses the
discretisation; `ose/integration.py` holds the integrators.

Three behaviours worth knowing:

- **Wind enters position only.** $w$ adds to $\dot p_x, \dot p_y$
  and never to drag. Heading is air-relative, so ground track differs from
  heading whenever wind is non-zero. Conflating the two is a recurring source
  of error.
- **Induced drag scales with $n^2$**, and $n$ with $v\omega$. A hard turn is
  expensive in a way a naive linear model misses entirely.
- **Fuel flow stops at dry mass** via a `burning` gate. That gate is
  discontinuous, so RK4 undershoots $m_{\mathrm{dry}}$ by tens of grams at
  exhaustion — an $O(\Delta t)$ artefact, surfaced by `state_violations()`
  rather than hidden.

---

## 3. Constraints — declared, not enforced

$$
\lambda = \begin{bmatrix} T_{\min} & T_{\max} & n_{\max} & \omega_{\mathrm{cap}} & v_{\min} & v_{\max} & m_{\mathrm{dry}} & m_{\max} \end{bmatrix}^{T}
$$

<!-- generated: lambda -->
| | reference | note |
|---|---|---|
| $T_{\min}$ | 5 kN | idle |
| $T_{\max}$ | 130 kN | |
| $n_{\max}$ | 9 | structural |
| $\omega_{\mathrm{cap}}$ | 30 °/s | roll and control authority |
| $v_{\min}$ | 90 m/s | hard floor, **independent of stall** |
| $v_{\max}$ | 600 m/s | airframe |
| $m_{\mathrm{dry}}$ | 12 000 kg | |
| $m_{\max}$ | 19 500 kg | |
<!-- end generated: lambda -->

$$
U(x) = \lbrace \thinspace u : T_{\min} \le T \le T_{\max},\ |\omega| \le \omega_{\max}(v,m) \thinspace \rbrace
$$

$$
X(\lambda) = \lbrace \thinspace x : v_{\mathrm{s}}(m,1) \le v \le v_{\max},\ v \ge v_{\min},\ m_{\mathrm{dry}} \le m \le m_{\max} \thinspace \rbrace
$$

**The vehicle declares these and does not apply them** (ADR 0006).
`project_command()` *offers* a projection onto $U$ and returns a
`Saturation` receipt carrying what was asked for; `derivative()` integrates
whatever it is given. A control law persistently commanding outside the
envelope is therefore a visible finding rather than a silent clip.

$X$ has no projection at all — a state cannot be projected without
falsifying the dynamics that produced it — so `state_violations()` reports and
never corrects.

> **$m \le m_{\max}$ is the odd one out.** It is the only element of
> $X$ that flying cannot violate: mass falls monotonically as fuel
> burns, so a trajectory starting inside stays inside. It is declared because
> it is a property of the airframe, and it fires on a badly built *initial*
> state — the same finding a badly specified platform gets from the
> composition-time mass budget in `ose/composition/`.

---

## 4. Capability

`capability(state, omega_rad_s, disturbance)` answers "what can I do right
now?" without integrating anything forward. Fourteen channels; the ones with
behaviour worth describing:

### Turn performance

$$
n_\ell(v,m) = \frac{\rho c_\ell v^{2}}{mg},
\qquad
n_{\mathrm{av}} = \min\lbrace n_{\max},\thinspace n_\ell\rbrace ,
\qquad
\omega_{\mathrm{av}} = \min\left\lbrace \omega_{\mathrm{cap}},\ \frac{g}{v}\sqrt{n_{\mathrm{av}}^{2}-1}\right\rbrace
$$

$$
\omega_{\mathrm{sus}}(v,m) = \min\left\lbrace \omega_{\mathrm{av}},\ \frac{g}{v}\sqrt{\frac{T_{\mathrm{av}} - \rho c_p v^{2} - A(v,m)}{A(v,m)}}\right\rbrace
$$

At $m = 16$ t:

<!-- generated: turn-performance -->
| $v$ [m/s] | $n_\ell$ | $n_{\mathrm{av}}$ | $\omega_{\mathrm{av}}$ [°/s] | $\omega_{\mathrm{sus}}$ [°/s] | $R_{\min}$ [m] | $T_{\mathrm{req}}$ [kN] |
|---|---|---|---|---|---|---|
| 100 | 1.78 | 1.78 | 8.27 | 8.27 | 692 | 19.1 |
| 150 | 4.01 | 4.01 | 14.53 | 14.53 | 592 | 17.8 |
| 200 | 7.12 | 7.12 | 19.81 | 15.45 | 579 | 24.0 |
| **225** | **9.01** | **9.00** | **22.34** | 15.10 | **577** | 28.7 |
| 250 | 11.13 | 9.00 | 20.10 | 14.68 | 713 | 34.2 |
| 300 | 16.02 | 9.00 | 16.75 | 13.61 | 1026 | 47.6 |
| 400 | 28.48 | 9.00 | 12.56 | 10.31 | 1824 | 82.8 |
| 500 | 44.50 | 9.00 | 10.05 | 1.79 | 2850 | 128.6 |
| 600 | 64.08 | 9.00 | 8.38 | 0.00 | 4104 | 184.7 |
<!-- end generated: turn-performance -->

**Which limit binds changes with speed.** Below the corner speed the aircraft
is lift-limited and cannot reach 9 g; above it, structure binds and $\omega$
falls as $g\sqrt{n^2-1}/v$. The peak sits exactly at the corner speed and the
minimum turn radius with it.

**The gap $[\omega_{\mathrm{sus}},\ \omega_{\mathrm{av}}]$ is the interesting
quantity.** Turning inside it is possible only by spending kinetic energy. At
250 m/s that is 14.7 to 20.1 °/s — command 16.5 °/s and the aircraft turns and
bleeds. Both bounds are published so a planner trades against them explicitly
rather than inferring them.

### Characteristic speeds

$$
v_{\mathrm{s}}(m,n) = \sqrt{\frac{nmg}{\rho c_\ell}},
\qquad
v_{\mathrm{c}}(m) = v_{\mathrm{s}}(m,1)\sqrt{n_{\max}}
$$

At 16 t: $v_{\mathrm{s}} = 75.0$ m/s, $v_{\mathrm{c}} = 224.9$ m/s. The usable
floor is $\max\lbrace v_{\min}, v_{\mathrm{s}}\rbrace  = 90$ m/s, so at this mass the hard
floor binds and stall does not. Above roughly 23 t it is the other way round,
and `test_claimed_speed_floor_reports_whichever_limit_binds` exercises both
regimes deliberately — a test at one mass alone would pass against an
implementation that had dropped the stall term entirely.

### The ceiling is not $v_{\max}$

$v_{\max} = 600$ m/s is declared, but level flight needs
$T_{\mathrm{req}} = 184.7$ kN there against 130 kN available. **The real
level-flight ceiling is about 503 m/s**, where $T_{\mathrm{req}}$ crosses
$T_{\max}$. Above it the vehicle is inside $X$ and decelerating:
$X$ is a validity claim, not a promise of trim.

`endurance_s` at 16 t runs from about 150 min at the drag minimum near 150 m/s
down to 14 min at 600 m/s.

---

## 5. Verification

Behaviour is pinned against the model document rather than against recorded
output — see `tests/test_vehicle.py` and `tests/test_capability.py`.

- **Capability is checked by integrating**, not by inspection: the vehicle is
  flown at its claimed sustained turn rate and must actually hold speed. An
  overconfident capability would silently corrupt every planner that trusts
  it.
- `demo_discretization.py` shows the same $f$ under Euler, RK2 and
  RK4 and confirms fourth-order convergence.
- `demo_live_flight.py`'s corner-speed sweep drives the vehicle down through
  the envelope pinned against $\omega_{\max}$; the delivered turn rate peaks
  within about 0.2 m/s of the closed-form $v_{\mathrm{c}}$. That residual is
  not model error — guidance now flies on the *believed* mass (ADR 0015) while
  the closed form uses the true one.

## 6. Deliberate omissions

Altitude, sideslip, angle of attack, rotational dynamics, thrust lag, and any
notion of control surfaces. The emphasis is integration, not fidelity: the
model exists to preserve the integration problem, not to predict a real
aircraft. A two-mode variant is
[`planar_point_mass_with_booster`](planar_point_mass_with_booster.md).
