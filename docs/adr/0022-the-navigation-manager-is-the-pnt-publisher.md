# 0022 — The navigation manager is the platform's PNT publisher

**Status:** Accepted. Extends ADR 0014, amends ADR 0010.

## Context

`TimeEstimator` published `platform.time.v1` and nothing consumed it. The
generated diagram listed it under "published with no consumer", which was true
and was the first time anyone looked at it as a structural fact rather than a
sentence in a docstring.

Meanwhile `NavigationManager` published the platform's position and heading,
and a consumer wanting both position and time had to bind two components. A
platform would then hold two answers to "where am I and when is it", with
nothing making them one thing.

Position, navigation and timing are one concept in every real system that has
them — PNT. In a GNSS receiver they are not merely co-located but statistically
coupled: clock bias is a filter state, and a position fix and a time solution
come out of the same update. This repository models none of that coupling and
should not pretend to, but the shape that would let it arrive later is the
shape where one component owns all three.

ADR 0014 established one own-state publisher per platform and argued at length
against fusion. Nothing about adding timing changes that argument.

## Decision

**`NavigationManager` publishes `platform.time.v1` alongside
`vehicle.state.v1`.** It binds a time source and republishes what that source
says, exactly as `estimate()` republishes the own-state source.

**The sources publish on their own ports** — `platform.time_source.v1` and
`vehicle.state_source.v1` — so that a source estimate and the platform's answer
are distinguishable. See ADR 0021.

**It still does not fuse, and still owns one own-state source.** This is a
structural change only. Nothing here couples position and time; `time()` is a
republication with no arithmetic in it.

**`time_source` is keyword-only.** `test_manager_refuses_to_fuse_alternatives`
asserts that `NavigationManager(a, b)` raises, which is how the no-fusion rule
is enforced rather than merely documented. A positional second parameter would
have absorbed that argument as a time source, the guard would have stopped
raising, and the test would have gone on passing while checking nothing. That
was verified by making it positional: two tests fail immediately.

**A platform with no time source raises rather than defaulting.** It has no
belief about the time, and returning a zero would be inventing one.

## Consequences

A consumer needing position and time binds one component. Nothing does yet —
`platform.time.v1` still has no consumer, and the diagram still says so, now
against the manager instead of the estimator.

`demos/demo_navigation.py` composes a `Clock` and a `TimeEstimator` for the
first time; before this, nothing in the repository composed either with
anything else. It reports the clock's 3-sigma offset bound alongside the
navigation errors, which is the only product that filter has (ADR 0010).

**Adding the clock to that demo moved its seed spawn from four children to
five, and the navigation results are unchanged** — `SeedSequence.spawn` is
incremental, so the first four children are identical. That is ADR 0005's
promise that adding a component does not perturb another component's stream,
demonstrated rather than asserted, and it was checked before the comment
claiming it was written.

**The manager now has two reasons to exist**, and a reader could reasonably ask
why timing lives with navigation rather than in a component of its own. The
answer is PNT, and it is a claim about where the architecture is going rather
than about what the code does today — the honest version of which is that
`time()` currently does nothing `TimeEstimator.estimate()` did not already do.

**`ingest()` still forwards only to the own-state source.** A
`ClockMeasurement` handed to the manager does not reach the time source; the
demo feeds it directly. That asymmetry is visible and unresolved: making
`ingest` dispatch to whichever source accepts the measurement would be the
obvious fix and would make the manager a router, which is a decision rather
than a tidy-up.

## References

- ADR 0014 — one navigation publisher per platform, and why it does not fuse
- ADR 0010 — the clock filter, and why its covariance is the whole product
- ADR 0021 — the port naming that made "source" and "published" distinguishable
- ADR 0005 — per-component RNG streams
