# `vehicle_guidance` — proportional heading and speed hold

`ose.subsystem.vehicle_guidance.VehicleGuidance`

The platform's motion control law. It converts a commanded setpoint into a
`VehicleCommand`, and it is the first component in the stack that is not a
model of anything physical — there is no preliminary modelling document behind
it, because a proportional law needs no derivation. What it needs is a
description of the three things it does that are not obvious from reading it.

Subsystem layer, purely cyber. Its only state input is an `OwnStateEstimate`
published by the navigation manager; it never reads truth. It binds a
`VehicleManager`, which owns the platform's believed mass and answers every
vehicle question at that mass (ADR 0015), so guidance takes no mass parameter
and there is nothing for a caller to reach for.

All numbers below are for the bundled `reference_fighter()` at sea level with
`STANDARD` guidance gains, at 16 000 kg. They are fictional and plausible,
never claims about a real aircraft.

---

## Notation

Symbols are plain, per ADR 0018, and reuse the vehicle notation so the two
pages read together — $\omega$, $T$, $v$, $m$, $\psi$, $\lambda$ all mean what
[`planar_point_mass.md`](../vehicle/planar_point_mass.md) says they mean.

Two are new, and one obvious choice was unavailable: $e$ is already the Oswald
efficiency factor, so the error terms are written as differences instead.

| | kind | meaning |
|---|---|---|
| $k_\psi$ | scalar | heading gain, 1/s |
| $k_v$ | scalar | speed gain, 1/s |
| $\Delta\psi$ | scalar | heading error, wrapped to $[-\pi, \pi]$ |
| $\Delta v$ | scalar | airspeed error |

<!-- generated: guidance-gains -->
| | reference | units |
|---|---|---|
| $k_\psi$ | 0.3 | 1/s |
| $k_v$ | 0.05 | 1/s |
<!-- end generated: guidance-gains -->

---

## 1. The law

Proportional, and **memoryless** — no integral term, no derivative term, no
state at all beyond the two gains. A memoryless law cannot wind up, cannot
carry an error across a setpoint change, and is a pure function of
`(setpoint, own_state)`, which is what makes it replayable.

Two setpoint types, dispatched on by `command()`. An unknown type raises
rather than defaulting.

**`HeadingSpeedSetpoint`** — hold a heading:

$$
\Delta\psi = \mathrm{wrap}(\psi_{\mathrm{cmd}} - \hat\psi), \qquad
\omega_{\mathrm{cmd}} = k_\psi \thinspace \Delta\psi + \dot\psi_{\mathrm{cmd}}
$$

**`TurnRateSpeedSetpoint`** — turn at a rate, with no heading loop at all:

$$
\omega_{\mathrm{cmd}} = \omega_{\mathrm{cmd}}^{\thinspace \mathrm{setpoint}}
$$

That second type exists because a heading command cannot express *turn as hard
as you can*. Ask for a rate above the achievable one and the setpoint laps the
vehicle: the wrapped error passes through 180°, changes sign, and the loop
obligingly reverses the turn. With no heading to chase there is no error to
wrap, so an unreachable rate simply saturates and stays saturated.

Both then share one projection stage:

$$
\omega_{\mathrm{ach}} = \mathrm{clamp}(\omega_{\mathrm{cmd}},
\thinspace \pm\omega_{\mathrm{av}}(\hat v, \hat m)), \qquad
\Delta v = v_{\mathrm{cmd}} - \hat v
$$

$$
T_{\mathrm{cmd}} = T_{\mathrm{req}}(\hat v, \hat m, \omega_{\mathrm{ach}})
+ \hat m \thinspace k_v \thinspace \Delta v
$$

and the pair $(T_{\mathrm{cmd}}, \omega_{\mathrm{cmd}})$ goes to the manager's
`project_command()`.

### It is a feedback law

Both channels are closed on the navigation estimate. $\hat\psi$ and $\hat v$
come from the `OwnStateEstimate` the navigation manager publishes, so the
error terms above are the feedback, and the loop is closed every cycle:

$$
\omega_{\mathrm{cmd}} = \underbrace{k_\psi \thinspace \Delta\psi}_{\mathrm{feedback}}
+ \underbrace{\dot\psi_{\mathrm{cmd}}}_{\mathrm{feedforward}}
\qquad
T_{\mathrm{cmd}} = \underbrace{T_{\mathrm{req}}}_{\mathrm{feedforward}}
+ \underbrace{\hat m \thinspace k_v \thinspace \Delta v}_{\mathrm{feedback}}
$$

Worth saying plainly, because the two feedforward terms get most of the
discussion below and are the parts a reader would not guess. They are also of
different kinds:

| Term | Kind | Does what |
|---|---|---|
| $\dot\psi_{\mathrm{cmd}}$ | reference feedforward | cancels the lag against a moving setpoint (section 4) |
| $T_{\mathrm{req}}$ | plant-model feedforward | supplies the steady-state thrust, so the speed loop only trims (section 3) |

Remove either and a worse controller remains. Remove the feedback and nothing
useful is left — and section 7's claim that steady-state accuracy equals the
navigation sigma one for one is only true *because* the loop is closed on the
estimate.

It is closed on the estimate and never on truth. That is the truth boundary,
and it is what makes the hold accuracy navigation's number rather than
guidance's.

Note which $\omega$ appears where. The feedforward is evaluated at the
**achievable** rate; the command carries the **requested** one, unclipped.
Section 3 is about why.

---

## 2. Declares nothing, enforces nothing

Guidance decides *what* to command. It does not decide what is admissible, and
it does not clip.

The raw command goes to `VehicleManager.project_command()`, which forwards to
the vehicle's own declared sets and returns a `Saturation` receipt. That
receipt is handed back to the caller rather than swallowed, so a control law
persistently commanding outside the envelope is a visible finding rather than a
silent clip (ADR 0006).

This is why guidance passes $\omega_{\mathrm{cmd}}$ unclipped even though it
has just computed $\omega_{\mathrm{ach}}$ and knows perfectly well the command
will be cut. Pre-limiting itself would make the `Saturation` report empty and
the exceedance invisible — the component would look better and the system
would tell you less.

---

## 3. The feedforward, and why it uses a different turn rate

The thrust term holds speed through the turn the vehicle will **actually
fly**, not the one the error term asked for. The two differ enormously,
because induced drag scales with load factor squared.

<!-- generated: guidance-feedforward -->
| $\Delta\psi$ [°] | $\omega$ requested [°/s] | $\omega$ achievable [°/s] | $n$ requested | $n$ achievable | $T$ requested [kN] | $T$ achievable [kN] |
|---|---|---|---|---|---|---|
| 10 | 3.0 | 3.0 | 1.67 | 1.67 | 38 | 38 |
| 45 | 13.5 | 13.5 | 6.09 | 6.09 | 115 | 115 |
| 90 | 27.0 | 20.1 | 12.05 | 9.00 | 358 | 214 |
| 135 | 40.5 | 20.1 | 18.05 | 9.00 | 763 | 214 |
| 180 | 54.0 | 20.1 | 24.05 | 9.00 | 1330 | 214 |
<!-- end generated: guidance-feedforward -->

Read the last row. A commanded heading reversal at 250 m/s asks for 54.0 °/s,
which is a 24 g turn against a 9 g airframe and would need 1330 kN from a
130 kN engine. Feedforwarding that produces a number with no physical meaning
attached to it. Evaluating at the achievable 20.1 °/s asks for 214 kN instead —
still far into saturation, but a number that means something, and one the
thrust limiter can act on sensibly.

The two columns agree exactly until the requested rate crosses
$\omega_{\mathrm{av}}$, which at this speed and mass is 20.1 °/s — so for
errors up to about 67° the distinction does not arise at all. It is the large
errors, which is to say the interesting ones, where it decides whether the
command is meaningful.

`project_command()` then clips to exactly $\omega_{\mathrm{av}}$, which is what
the thrust was computed for, so the pair that comes out is self-consistent
rather than a thrust for one turn and a rate for another.

---

## 4. A moving setpoint is never caught without its rate

A proportional law chasing a ramp settles where the correction term alone
supplies the whole turn rate. The heading error does not go to zero; it goes to
$\dot\psi_{\mathrm{cmd}} / k_\psi$ and stays there.

<!-- generated: guidance-ramp-lag -->
| commanded sweep [°/s] | standing error [°] |
|---|---|
| 2 | 6.7 |
| 5 | 16.7 |
| 10 | 33.3 |
| 20 | 66.7 |
| 30 | 100.0 |
<!-- end generated: guidance-ramp-lag -->

At the reference gain a 20 °/s sweep therefore leaves the vehicle 66.7°
behind the commanded heading, indefinitely. That is not a small error being
tolerated — it is most of a quadrant, and a mission flown on it would look like
the aircraft was ignoring its instructions.

The fix is not a bigger gain, which buys a proportionally smaller lag at the
cost of stability margin. It is `HeadingSpeedSetpoint.psi_rate_cmd_rad_s`:
whoever builds the setpoint knows the rate exactly, so the setpoint declares
it and guidance feeds it forward. The error then settles at zero and the
proportional term only makes up the difference.

Guidance cannot recover that rate by differentiating $\psi_{\mathrm{cmd}}$, for
two independent reasons: it is memoryless by design and has no previous
setpoint to difference against, and the commanded heading steps
discontinuously whenever the commander changes its mind.

---

## 5. Two masses, deliberately

Guidance uses the platform's believed mass twice, and asks for it in two
different forms.

| Where | Which | Why |
|---|---|---|
| `capability()`, what it publishes | `capability_bound()` — margined by the mass sigma | a planner deciding whether a leg is flyable should be told what the platform is *confident* of |
| the control law, sections 1 and 3 | `capability()` — the point estimate | thrust computed for a mass the aircraft does not have is *wrong*, not cautious |

Clipping against a margin would also make a `Saturation` finding mean estimator
doubt rather than an airframe limit, which is a different claim entirely. See
ADR 0016.

---

## 6. Wind, and what guidance does about it

Nothing. That is worth stating with numbers rather than leaving to be found.

Wind enters the dynamics in the two position rows only — it moves the platform
without pushing on it, so airspeed and heading are untouched and
$T_{\mathrm{req}}$, which is drag at an *airspeed*, is unaffected and correct.

But guidance holds **air-relative** quantities: a heading and an airspeed.
Neither is a ground track. So a crosswind produces a standing track error that
the loop is not merely failing to remove — it is not looking at it.

<!-- generated: guidance-wind -->
| crosswind [m/s] | track error, heading held [°] | crab that would hold track [°] |
|---|---|---|
| 5 | 1.15 | 1.15 |
| 10 | 2.29 | 2.29 |
| 20 | 4.57 | 4.59 |
| 30 | 6.84 | 6.89 |
| 50 | 11.31 | 11.54 |
<!-- end generated: guidance-wind -->

Both columns are the wind triangle: hold a heading in a crosswind and the
ground track sits $\arctan(w/v)$ off it, while holding the *track* instead
would need a crab of $\arcsin(w/v)$ into the wind. At 250 m/s a 30 m/s
crosswind is 6.84° of track error, which over 200 s of straight flight is six
kilometres of drift with the commanded heading held exactly.

A route recovers most of it, because `WaypointPlanner` recomputes the bearing
to the active waypoint every cycle — that is a pursuit law, and pursuit
converges. Flown against a 30 km leg with that crosswind it still captures, and
takes 118 s against 117 s still, but bows about 1.4 km off the direct line on
the way: the platform is always *pointing* at the waypoint and always *moving*
somewhere else. Head and tailwinds only change the clock.
[`tests/behaviour/test_wind.py`](../../../tests/behaviour/test_wind.py) pins
all of that.

**The information to do better is already on the wire.** `OwnStateEstimate`
carries `wind_estimate_mps` and `ground_velocity_mps`; the INS/GNSS filter
estimates the wind and is tested for it. Guidance references neither, and
neither does the planner. Setting $\psi_{\mathrm{cmd}}$ to the bearing minus
$\arcsin(w_{\perp}/v)$ would cancel the crab using data the platform already
publishes.

That is not a missing line of code, though. It is the question of whether
guidance should hold a *track* as well as a heading — a third setpoint type and
a different control objective, which is an architectural decision rather than a
tweak. See section 9.

---

## 7. What it publishes

`GuidanceCapability`, composed from two layers because a control loop is
bounded by both and they bound different things (ADR 0013).

The **vehicle** half — maximum turn rate, speed band — arrives already
evaluated at the believed mass, from the manager. The **navigation** half is
read straight from the covariance travelling with `own_state`:

$$
\sigma_{\psi\thinspace \mathrm{hold}} = \sqrt{P_{22}}, \qquad
\sigma_{v\thinspace \mathrm{hold}} = \sqrt{P_{33}}
$$

Those are floors, not guarantees, and they are navigation's numbers rather
than guidance's own. The loop drives the *believed* state to the setpoint, so
once settled the true error **is** the navigation error, one for one: a
heading-hold loop steering on an estimate with a one-degree sigma holds true
heading to one degree, no better, whatever its gains are.

Reading the covariance rather than querying the estimator follows the rule
ADR 0009 set for measurements — the consumer uses the uncertainty that arrives
with the data — and it is the more useful number, because it reflects a GNSS
outage as it happens where a static claim would not.

---

## 8. Verification

| Claim | Test |
|---|---|
| the law converges on heading and speed | `test_holds_heading_and_speed_setpoint` |
| feedforward is evaluated at the achievable rate | `test_feedforward_matches_the_turn_actually_commanded` |
| a ramp without feedforward lags by rate/gain | `test_ramping_setpoint_without_feedforward_lags_by_rate_over_gain` |
| declaring the rate removes the lag | `test_declaring_the_rate_removes_the_lag` |
| an unreachable rate saturates and stays saturated | `test_unreachable_turn_rate_saturates_and_stays_saturated` |
| clipping is reported, not swallowed | `test_reports_saturation_when_setpoint_exceeds_envelope` |
| published capability is the bound, not the estimate | `test_published_capability_is_the_promised_envelope_not_the_estimate` |
| the control law still uses the point estimate | `test_the_control_law_still_uses_the_point_estimate` |
| hold accuracy is honest | `test_claimed_hold_accuracy_is_honest` |
| guidance cannot see truth | `conformance/test_truth_boundary.py` |

All in [`tests/integration/test_vehicle_guidance.py`](../../../tests/integration/test_vehicle_guidance.py)
unless noted. The whole-platform consequences — a route flown to completion, a
corner-speed sweep, an unsustainable turn bleeding speed — are in
[`tests/behaviour/`](../../../tests/behaviour/).

---

## 9. Deliberate omissions

**No integral term.** Nothing here corrects a standing bias, and with a
feedforward-declared setpoint there is none to correct. An integral term would
also give the component state, which would end its replayability.

**No waypoint setpoint.** The planner decides *where* and guidance decides
*how*, so a route is converted to a heading one layer up, where the route is
known. See [`action_planner.py`](../../../src/ose/single_ship/action_planner.py).

**No time.** Neither setpoint type carries one, and neither does a `Waypoint`,
which is position and speed only. So *be at this position, on this heading, at
time t* is not expressible anywhere in the motion pipeline: there is no
required time of arrival, no arrival heading, and no schedule. `plan()` takes
`t_s` and uses it solely to stamp the `ActionSet`; it never reaches a decision,
and capture is by radius rather than by clock.

That is coherent for what exists — a single platform following a geometric
route — and it is a ceiling rather than an oversight. Coordinating two
platforms is where a time constraint stops being optional, and adding one turns
the planner from geometric into scheduled, which is an architectural decision
rather than a field on a record.

**No lateral acceleration or bank command.** The vehicle is a planar point mass
whose input is a turn rate; there is no roll axis to command.

**No wind correction, and no track hold.** Guidance holds a heading, not a
ground track, so a crosswind leaves a standing track error it never looks at
(section 6). The platform publishes everything needed to fix it —
`wind_estimate_mps` and `ground_velocity_mps` travel on every
`OwnStateEstimate`, and nothing in the repository consumes either.

What is missing is not the arithmetic but the objective: holding a track is a
different thing to hold, wanting a third setpoint type and a decision about
what a planner means when it names a heading. Worth an ADR when it is done,
and worth knowing about until then, because a route flown in wind looks
correct — it captures every waypoint — while flying a bowed path nobody asked
for.

**No mode logic.** Choosing boost is a planning act, published in
`ActionSet.propulsion`, and guidance never sees that field — engaging boost
because the speed loop wanted more thrust would let a hold loop quietly spend a
contested thermal and fuel budget.
