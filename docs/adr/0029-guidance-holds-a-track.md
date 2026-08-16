# 0029 — Guidance holds a ground track, by feedback alone

**Status:** Accepted.

## Context

Guidance held a heading and an airspeed, neither of which is a ground track, so
a crosswind produced a standing track error it was not merely failing to remove
but not looking at. Measured before this change: at 250 m/s a 30 m/s crosswind
put the ground track 6.84° off the commanded heading — exactly $\arctan(w/v)$ —
and 200 s of straight flight ended six kilometres downwind with the heading held
to within a hundredth of a degree.

`WaypointPlanner` masked most of it. It recomputes the bearing to the active
waypoint every cycle, which is a pursuit law, and pursuit converges: the same
30 km leg still captured, one second later, having bowed **1 390 m** off the
direct line. A route flown in wind therefore looked correct — every waypoint
reached — while flying a track nobody asked for.

The platform already published everything needed to do better.
`OwnStateEstimate` carries `ground_velocity_mps` and `wind_estimate_mps`, the
INS/GNSS filter estimates both, and neither guidance nor the planner referenced
either.

## Decision

**A third setpoint form, `TrackSpeedSetpoint`**, on the existing
`guidance.setpoint.v1` — additive, following the precedent set when
`TurnRateSpeedSetpoint` was added. It names a track and a speed; the speed is
still airspeed, because the vehicle controls airspeed and a ground-speed command
is a different decision that only matters once something has to arrive at a
time.

**The law is feedback on track error and nothing else:**

$$
\hat\psi_g = \mathrm{atan2}(v_{g,y}, v_{g,x}), \qquad
\omega_{\mathrm{cmd}} = k_g \thinspace \mathrm{wrap}(\psi_{g,\mathrm{cmd}} - \hat\psi_g)
+ \dot\psi_{g,\mathrm{cmd}}
$$

Exact for the same reason the heading loop is: the plant integrates $\omega$
into $\psi$, so $\omega$ settling at zero requires the track error to be zero,
whatever the wind is doing. **The crab angle is found, not computed** — nothing
evaluates an arcsine; the heading is left free and ends up wherever zero track
error requires.

**No crab feedforward from the wind estimate**, and this reverses the option
originally chosen for this work. A crab correction cannot enter as a rate, so it
can only enter as a heading command, and a heading-error term and a track-error
term both feeding one rate command balance at a non-zero equilibrium. Measured
on the kinematics:

| wind estimate | feedback only | feedback + crab feedforward |
|---|---|---|
| perfect | 0.000°, settles 14.0 s | 0.000°, settles 6.9 s |
| 30% low | **0.000°**, settles 14.0 s | **+1.033°**, never settles |
| 30% high | **0.000°** | −1.034° |

Seven seconds of settling in the case that did not need help, in exchange for a
standing error in the case that did — and this filter's wind is only observable
after a turn, so a poor estimate is the normal condition rather than the
exception. `wind_estimate_mps` therefore remains unconsumed, deliberately.

**`WaypointPlanner` commands a track.** A bearing to a waypoint is a direction
over the ground, so commanding it as a heading was a category error that only
showed in wind. Without this the feature would have been reachable by nothing,
which is the fault this repository has spent several sessions finding elsewhere.

**Navigation publishes `ground_velocity_covariance`**, a 2×2 added to
`OwnStateEstimate` rather than folded into the existing 4×4, because widening a
published field is breaking and adding one is not. It is not new information:
the filter's error state already carries ground velocity, and the block was
computed every cycle and discarded. `GuidanceCapability` gains
`track_hold_sigma_rad`, that block projected onto the track angle.

## Consequences

The 30 km leg in a 30 m/s crosswind bows **92 m** instead of 1 390 m, a
fifteenfold improvement, and the heading settles at −7.09° against a −6.89°
crab. The residual is the transient while the loop turns onto the leg, not a
standing error, and it is not worth tuning away.

**Track accuracy is a different claim from heading accuracy and is reported
separately.** A track loop steers on ground velocity, so its floor is the
uncertainty in ground velocity; publishing the heading sigma under a track name
would have been the anti-conservative mislabelling ADR 0016 was written about,
and in wind the two are not close. Ground velocity now carries a NEES test
because publishing an uncertainty obliges one — measured 0.50 to 2.24 across
seeds against 2 degrees of freedom.

**A stale test double was exposed, and it is the finding worth remembering.**
`tests/behaviour/_platform.py`'s "perfect estimate" built ground velocity from
airspeed alone, omitting the wind. That was invisible while nothing read the
field, and the first run of the track-hold behaviour tests showed the bow
unchanged at 1 390 m — guidance closing a track loop on a track that did not
exist. A stub is only as perfect as the fields something reads, so consuming a
previously-ignored field is exactly when its stubs need checking.

**Heading hold is unchanged and remains the right thing** when what you mean is
a heading. Three setpoint forms is more surface, and the dispatch in `command()`
is now three branches, but each is a different control objective rather than a
variation on one.

**A track cannot be held at a standstill** — the angle is undefined — so the
setpoint raises rather than being answered with a fabricated one. Nothing
currently flies slowly enough for that to arise.

## References

- ADR 0009 — the consumer uses the uncertainty travelling with the data, which
  is why the track sigma comes from the estimate rather than from a query
- ADR 0016 — why a claim named for one thing must not be another thing's number
- ADR 0018 — $\chi$ was spent on the indicator function, hence $\psi_g$
- ADR 0026 — the previous case of two components meaning different things by
  one name, found the same way
