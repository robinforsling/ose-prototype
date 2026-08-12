# `planar_point_mass_with_booster` — two propulsion modes

`ose.equipment.vehicle.planar_point_mass_with_booster.PlanarPointMassWithBooster`

The baseline planar point mass with an afterburner: a discrete propulsion mode
$q$, a sixth continuous state tracking thermal load, and mode-dependent thrust,
fuel flow and speed limits. Mathematics in
[`docs/vehicle/vehicle_model.pdf`](../../vehicle/vehicle_model.pdf) section 5;
this file describes the implementation and its behaviour.

**The aerodynamics are unchanged.** Drag, lift, the turn-rate bound and the
disturbance model are all the baseline's, and the reference configuration
shares its `VehicleGeometry` record rather than restating it — a comparison
between the two models is only meaningful if the airframe is identical. Read
[`planar_point_mass.md`](planar_point_mass.md) first; only the differences are
described here.

---

## 1. What changes

$$
q \in \mathcal{Q} = \{\mathrm{nom},\ \mathrm{boost}\},
\qquad
\boldsymbol{x} = \begin{bmatrix} p_x & p_y & \psi & v & m & s \end{bmatrix}^{T}
$$

$s$ is a **thermal accumulator**, normalised so that $s_{\max} = 1$. It is what
makes boost a finite resource rather than a free upgrade, and it is why this is
a separate model rather than a flag on the baseline: a state the baseline never
writes has no business in the baseline's Jacobian or integrator.

### The mode is state, not input

$\mathcal{S}_q(\boldsymbol{x}, \boldsymbol{\lambda})$ describes transitions
*from* the current mode, so the current mode is a discrete state and only the
*requested* mode is an input. `BoostState` carries $q$; `VehicleCommand`
carries the request. Every method therefore takes the same arguments as the
baseline's, which is what lets one `VehicleManager` serve both models without
asking which it holds.

$q$ is **excluded from `to_array()`**. The integrator advances the six
continuous states; the mode is held constant across a step by construction,
and integrating across a switch would integrate a discontinuity.

### Parameters and constraints

$$
\boldsymbol{\theta} = \begin{bmatrix} c_p & c_i & c_\ell & c^{\mathrm{nom}}_{\mathrm{TSFC}} & c^{\mathrm{boost}}_{\mathrm{TSFC}} & \tau_{\mathrm{h}} & \tau_{\mathrm{c}} \end{bmatrix}^{T}
$$

$$
\boldsymbol{\lambda} = \begin{bmatrix} T_{\min} & T^{\mathrm{nom}}_{\max} & T^{\mathrm{boost}}_{\max} & n_{\max} & \omega_{\mathrm{cap}} & v_{\min} & v^{\mathrm{nom}}_{\max} & v^{\mathrm{boost}}_{\max} & m_{\mathrm{dry}} & m_{\max} & m_{\mathrm{res}} & s_{\max} \end{bmatrix}^{T}
$$

Both **compose** the baseline's records rather than restating them —
`BoostParameters.nominal` is a `VehicleParameters`, `BoostConstraints.nominal`
is a `Constraints` — so the drag polar has exactly one definition.

| | reference | against baseline |
|---|---|---|
| $c^{\mathrm{boost}}_{\mathrm{TSFC}}$ | $6.0\times10^{-5}$ kg/(N s) | **2.4×** the nominal rate |
| $T^{\mathrm{boost}}_{\max}$ | 180 kN | +38% |
| $v^{\mathrm{boost}}_{\max}$ | 700 m/s | +100 m/s |
| $\tau_{\mathrm{h}}$ | 30 s | cold to limit |
| $\tau_{\mathrm{c}}$ | 90 s | $3\tau_{\mathrm{h}}$ |
| $m_{\mathrm{res}}$ | 300 kg | fuel reserve — **policy** |
| $s_{\max}$ | 1 | thermal limit — **physics** |
| $\Delta t_{\mathrm{dwell}}$ | 5 s | anti-chatter — **policy** |

**Only the thermal limit is physical.** An aircraft can light its afterburner
on its last hundred kilograms and can switch twice in a second; it simply
should not. Anyone configuring a different aircraft needs to know which entry
they cannot relax.

---

## 2. Dynamics

$$
\dot{\boldsymbol{x}} = \boldsymbol{f}_q(\boldsymbol{x}, \boldsymbol{u}, \boldsymbol{\theta}, \boldsymbol{\eta}) + \boldsymbol{G}(\boldsymbol{x})\boldsymbol{w},
\qquad
\boldsymbol{f}_q =
\begin{bmatrix}
v\cos\psi \\ v\sin\psi \\ \omega \\
\dfrac{T - D(v,m,\omega)}{m} \\
-c_{\mathrm{TSFC},q}\,T \\
\sigma_q(s)
\end{bmatrix},
\qquad
\sigma_q(s) =
\begin{cases}
\dfrac{1}{\tau_{\mathrm{h}}}, & q = \mathrm{boost} \\[8pt]
-\dfrac{s}{\tau_{\mathrm{c}}}, & q = \mathrm{nom}
\end{cases}
$$

Accumulation is **linear** and recovery is **exponential**. From cold, boost
reaches the limit in exactly $\tau_{\mathrm{h}} = 30$ s. From $s = 1$, recovery
is asymptotic: $s = 0.37$ after 90 s, and about 125 s to fall below 0.25.

`thermal_rate()` is published as a *derivative* so a consumer integrates it
(ADR 0004) — that is how `VehicleManager` dead-reckons its thermal belief
without reimplementing $\sigma_q$.

---

## 3. The switching set

$$
\mathcal{S}_{q}(\boldsymbol{x}, \boldsymbol{\lambda}) =
\begin{cases}
\{\mathrm{nom}\}, & s \ge s_{\max}\ \text{ or }\ m \le m_{\mathrm{dry}} + m_{\mathrm{res}} \\
\{q\}, & t - t_{q} < \Delta t_{\mathrm{dwell}} \\
\{\mathrm{nom}, \mathrm{boost}\}, & \text{otherwise}
\end{cases}
$$

**evaluated in that order.** Three outcomes, not two, and the distinction is
not cosmetic:

- **forced out** — the budget is exhausted; leave boost regardless of dwell,
  or the aircraft is held past its own limit
- **locked in** — the dwell has not elapsed; stay where you are, which is what
  an anti-chatter rule is *for*
- **free** — choose

An earlier formulation listed the permissive cases by mode and returned
$\{\mathrm{nom}\}$ otherwise. With $q = \mathrm{boost}$ and the dwell unserved
that says $\{\mathrm{nom}\}$, so boost was granted on one step and revoked on
the next indefinitely — an anti-chattering condition that produced chattering.
It was found by simulating rather than reading, and the thermal state never
rose above 0.02. The ordering matters too: test the dwell first and an aircraft
reaching $s_{\max}$ shortly after engaging is *held* in boost past its thermal
limit.

### Declared, not enforced

$\mathcal{S}_q$ is an input constraint, so ADR 0006 applies as it does to
thrust. Three separate things happen, in three places:

1. `admissible_modes()` **declares** — it answers a question and refuses
   nothing.
2. `project_command()` **offers** — an inadmissible request comes back as
   `mode=NOMINAL` with a note in `Saturation`. A mode cannot be clipped by
   degree, so the projection is a *fallback*, not a reduction.
3. **The caller** decides to fly what it was offered. That is the only step
   that refuses anything.

`derivative()` reads $q$ from the state, so a caller that ignores the offer
gets boost dynamics — $s$ past $s_{\max}$, fuel at the boost rate — and the
consequence surfaces through $\mathcal{X}_q$ rather than being prevented.
Exactly the treatment a 200 kN command against a 130 kN engine receives.

The caller also owns $t_q$. `VehicleManager.select_mode()` restarts the clock
whenever the delivered mode differs from the current one, including when the
vehicle forced the fallback.

$$
\mathcal{X}_q(\boldsymbol{\lambda}) = \{\, \boldsymbol{x} : v_{\mathrm{s}}(m,1) \le v \le v^{q}_{\max},\ v \ge v_{\min},\ m_{\mathrm{dry}} \le m \le m_{\max},\ 0 \le s \le s_{\max} \,\}
$$

---

## 4. Behaviour: what boost buys

### It does **not** improve the instantaneous turn

$\omega_{\max}(v,m)$ **carries no mode index.** The instantaneous bound comes
from available lift and structural strength, and an afterburner changes
neither. The method takes no mode argument at all, and a test asserts it
matches the baseline's exactly.

This is the intuitive guess and it is wrong. What boost moves is the
*sustained* rate, through $T_{\mathrm{av},q}$: it does not let the aircraft
turn tighter, it lets a given turn be held.

At $m = 16$ t, $s = 0$:

| $v$ [m/s] | $\omega_{\mathrm{av}}$ [°/s] | $\omega_{\mathrm{sus}}$ nom | $\omega_{\mathrm{sus}}$ boost | gain | $t_{\mathrm{end}}$ nom | $t_{\mathrm{end}}$ boost |
|---|---|---|---|---|---|---|
| 150 | 14.53 | 14.53 | 14.53 | 0.00 | 150 min | 63 min |
| 200 | 19.81 | 15.45 | 18.74 | +3.29 | 111 min | 46 min |
| 225 | 22.34 | 15.10 | 18.45 | +3.35 | 93 min | 39 min |
| 250 | 20.10 | 14.68 | 18.11 | +3.43 | 78 min | 32 min |
| 300 | 16.75 | 13.61 | 16.75 | +3.14 | 56 min | 23 min |
| 400 | 12.56 | 10.31 | 12.56 | +2.26 | 32 min | 13 min |
| 500 | 10.05 | 1.79 | 10.05 | +8.26 | 21 min | 9 min |

Two things to read off it. At **150 m/s the gain is zero** — the aircraft is
already lift-limited and thrust is not what is stopping it, so boost buys
nothing at all. At **300 m/s and above, $\omega_{\mathrm{sus}}$ reaches
$\omega_{\mathrm{av}}$** in boost: the sustained turn *is* the instantaneous
turn, and there is no energy-bleeding region left to trade in.

### It raises the ceiling, and costs endurance

Level flight at $T^{\mathrm{boost}}_{\max} = 180$ kN is possible to about
**592 m/s**, against 503 m/s nominal. Declared $v^{q}_{\max}$ is 700 and 600
respectively, so in both modes the trim ceiling is well below the airframe
limit.

Endurance is roughly **2.4× worse** in boost at any speed, which is
$c^{\mathrm{boost}}_{\mathrm{TSFC}} / c^{\mathrm{nom}}_{\mathrm{TSFC}}$
directly — before counting the extra thrust actually commanded.

### Capability adds two channels

`BoostCapability` subclasses `Capability`, so a consumer typed to the base
record keeps working. It adds

$$
c_{\mathrm{boost}} = \mathbb{1}[\mathrm{boost} \in \mathcal{S}_q],
\qquad
t_{\mathrm{boost}} = \tau_{\mathrm{h}}(s_{\max} - s)
$$

— whether boost is selectable now, and for how long it could be held. From
cold that is the full 30 s; at $s = 0.5$, 15 s; at the limit, zero.

---

## 5. Behaviour: boost is a budget, and policy decides how to spend it

Deciding the mode is a **planning act, not a consequence of thrust demand**.
Engaging boost automatically whenever guidance asked for more than nominal
thrust would let a speed-hold loop quietly spend a thermal budget and a fuel
reserve that are finite and contested — and the planner would find boost
unavailable exactly when it had been saving it. So the mode travels in
`ActionSet.propulsion`, and guidance never sees it.

`demos/demo_boost.py` flies one mission with two policies to show why that
matters. Both command 16.5 °/s at 250 m/s — above the nominal sustained rate,
below the boosted one — and differ only in when they ask:

| | boost held | mode switches | peak $s$ | violations | fuel |
|---|---|---|---|---|---|
| ask whenever allowed | 44.7 s | 20 | **1.0016** | 20 s | 968 kg |
| stop at $s=0.85$, resume below 0.25 | 25.5 s | 2 | 0.8500 | 0 | 835 kg |

**The naive policy is not simply worse**, which is what makes it worth
showing. It extracts *more* boost — precisely by living on the limit — and
pays with twenty mode changes, a violated thermal limit and 133 kg of fuel.

### The limit cycle

Asking whenever allowed produces a duty cycle, and the dwell time causes it
rather than preventing it:

1. boost held 30 s ($=\tau_{\mathrm{h}}$), $s$ fills from cold to $s_{\max}$
2. $\mathcal{S}_q$ withdraws it — a transition, so the dwell clock restarts
3. **5.0 s DENIED**: still asking, dwell forbids a switch; $s$ sheds 0.054
4. dwell expires just under the limit, boost granted, **1.6 s** at
   $1/\tau_{\mathrm{h}}$ puts it back at $s_{\max}$
5. repeat — nine times

$\tau_{\mathrm{c}} = 3\tau_{\mathrm{h}}$, so five seconds of cooling is undone
by under two of boost. **A dwell time stops fast chattering and does nothing
about this**, because nothing here switches faster than the dwell allows. The
anti-chatter rule is aimed at the wrong timescale.

### The overshoot is discretisation

Peak $s = 1.0016$, above $s_{\max}$. The mode is re-evaluated at step
boundaries, so $s$ crosses the limit before anything notices; with
$\tau_{\mathrm{c}} = 90$ s that 0.2 % takes a minute to decay back under, and
`state_violations()` reports it for the whole minute — correctly, since the
state really is outside $\mathcal{X}_q$.

None of this is the model misbehaving. The constraints do what they declare
and the vehicle flies what it is given; the naive policy is a bad one.

---

## 6. Verification

`tests/test_planar_point_mass_with_booster.py`. The two that carry the most
weight:

- **`test_turn_rate_bound_is_mode_independent`** pins the counter-intuitive
  claim against the baseline directly, and pins the *signature* as much as the
  number — the method takes no mode argument.
- **`test_the_dwell_locks_the_current_mode_in_rather_than_out`** pins the
  correction to $\mathcal{S}_q$; restoring the document's original formulation
  fails it.

Sustained boost is checked by *integrating* to the thermal limit rather than
asserting on the rate, because that number is what makes boost finite.
