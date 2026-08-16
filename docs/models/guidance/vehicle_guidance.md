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

Four are new, and two obvious choices were unavailable. $e$ is already the
Oswald efficiency factor, so the error terms are written as differences. And
$\chi$, the usual symbol for a course angle, was spent by ADR 0018 on the
indicator function — hence $\psi_g$, which says what it is without a new
letter.

| | kind | meaning |
|---|---|---|
| $k_\psi$ | scalar | heading gain, 1/s |
| $k_v$ | scalar | speed gain, 1/s |
| $\Delta\psi$ | scalar | heading error, wrapped to $[-\pi, \pi]$ |
| $\Delta v$ | scalar | airspeed error |
| $k_g$ | scalar | track gain, 1/s |
| $\psi_g$ | scalar | ground track, the ground-referenced counterpart of $\psi$ |
| $\Delta\psi_g$ | scalar | track error, wrapped to $[-\pi, \pi]$ |

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

**`TrackSpeedSetpoint`** — hold a ground track, which is where the platform
is *going* rather than where it is *pointing*:

$$
\hat\psi_g = \mathrm{atan2}(v_{g,y}, v_{g,x}), \qquad
\Delta\psi_g = \mathrm{wrap}(\psi_{g,\mathrm{cmd}} - \hat\psi_g), \qquad
\omega_{\mathrm{cmd}} = k_g \thinspace \Delta\psi_g + \dot\psi_{g,\mathrm{cmd}}
$$

Structurally the heading law with a different error, and section 6 is about why
that difference matters and why nothing computes a crab angle.

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

## 6. Wind, and holding a track

Wind enters the dynamics in the two position rows only — it moves the platform
without pushing on it, so airspeed and heading are untouched and
$T_{\mathrm{req}}$, which is drag at an *airspeed*, is unaffected and correct.

But a heading is air-relative and a ground track is not, so holding one is not
holding the other:

<!-- generated: guidance-wind -->
| crosswind [m/s] | track error on a heading setpoint [°] | crab a track setpoint settles at [°] |
|---|---|---|
| 5 | 1.15 | 1.15 |
| 10 | 2.29 | 2.29 |
| 20 | 4.57 | 4.59 |
| 30 | 6.84 | 6.89 |
| 50 | 11.31 | 11.54 |
<!-- end generated: guidance-wind -->

Both columns are the wind triangle. Hold a *heading* in a crosswind and the
ground track sits $\arctan(w/v)$ off it — at 250 m/s a 30 m/s crosswind is
6.84° of track error, which over 200 s of straight flight is six kilometres of
drift with the commanded heading held to a hundredth of a degree. Hold a
*track* and the heading instead settles at $\arcsin(w/v)$ into the wind, which
is the second column and is the crab angle.

**Guidance finds that crab by closing the loop, not by computing it.** Nothing
in `_command_track_speed` evaluates an arcsine. The heading is left free, the
track error is driven to zero, and the place the heading ends up *is* the crab.
It is exact for the same reason the heading loop is: the plant integrates
$\omega$ into $\psi$, so $\omega$ settling at zero requires the error it is
driven by to be zero, whatever the wind is doing.

That exactness is why there is **no crab feedforward from the wind estimate**,
though `own_state.wind_estimate_mps` carries one and is otherwise unread. A
crab correction cannot enter as a rate, so it can only enter as a heading
command — and a heading-error term and a track-error term both feeding one rate
command balance at a *non-zero* equilibrium. Measured on the kinematics:

| wind estimate | feedback only | feedback + crab feedforward |
|---|---|---|
| perfect | 0.000° in 14.0 s | 0.000° in 6.9 s |
| 30% wrong | **0.000°** in 14.0 s | **1.03°**, never settles |

Seven seconds of settling in the case that did not need help, in exchange for a
standing error in the case that did — and this platform's wind is only
observable after a turn, so a poor estimate is the normal condition. See
ADR 0029.

`WaypointPlanner` commands a track, because a bearing to a waypoint is a
direction over the ground. On a 30 km leg in that crosswind the flown path used
to bow 1 390 m off the direct line; it now bows 92 m, and the heading settles
at −7.09° against a −6.89° crab. The residual is the transient while the loop
turns onto the leg, not a standing error.
[`tests/behaviour/test_wind.py`](../../../tests/behaviour/test_wind.py) pins
all of it.

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

`track_hold_sigma_rad` is a third and is **not** the heading sigma. A track
loop steers on ground velocity, so its floor is the uncertainty in ground
velocity, which the 4x4 above cannot supply — it is over heading and airspeed,
both air-relative. It comes from `OwnStateEstimate.ground_velocity_covariance`
projected onto the track angle:

$$
J = \frac{[-v_{g,y}, \thinspace v_{g,x}]}{\lVert v_g \rVert^2}, \qquad
\sigma_{\psi_g}^2 = J \thinspace P_{v_g} \thinspace J^{\mathsf{T}}
$$

Reporting the heading sigma under a track name would be the anti-conservative
mislabelling ADR 0016 exists about, and in wind the two are not close.

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
| a track is held through a crosswind | `test_track_setpoint_holds_a_track_through_a_crosswind` |
| the heading settles at the crab angle | `test_holding_a_track_leaves_the_heading_crabbed` |
| the track law ignores the wind estimate | `test_the_track_law_does_not_read_the_wind_estimate` |
| a track cannot be held at a standstill | `test_a_track_cannot_be_held_at_a_standstill` |
| the track hold claim is the projected ground-velocity sigma | `test_claimed_track_hold_accuracy_is_the_projected_ground_velocity_sigma` |
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

**No mode logic.** Choosing boost is a planning act, published in
`ActionSet.propulsion`, and guidance never sees that field — engaging boost
because the speed loop wanted more thrust would let a hold loop quietly spend a
contested thermal and fuel budget.
