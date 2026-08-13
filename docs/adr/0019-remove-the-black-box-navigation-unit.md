# 0019 — Remove the black-box navigation unit

Status: accepted
Date: 2026-08-12

## Context

ADR 0009 split navigation across the equipment and subsystem layers and, in
the same record, kept `IntegratedNavUnit`: a single equipment-layer component
that read ground truth, added white noise, and published `vehicle.state.v1`
directly. It was scaffolding — a way to exercise guidance or planning without
standing up an IMU, a GNSS receiver, an air-data sensor, an estimator, an
initial alignment and a measurement schedule. Its own docstring said it was
never a valid baseline for a claim about navigation performance.

The composed alternative has since been measured properly rather than assumed
to work. Over eight seeds, the published four-channel ANEES averages 4.04
against four degrees of freedom. Every internal error state is consistent:
accelerometer bias 1.0–1.7 against 2 dof, gyro bias 0.1–1.1 against 1, wind
0.5 against 2. Wind is recovered to within 0.35 m/s of truth. Airspeed
uncertainty holds at 0.481 m/s through a hundred-second GNSS outage while
position degrades from 0.86 m to 8.16 m. A fifteen-degree initial heading
error converges to 0.001°. Each of those is now a test, and each fails under a
matching sabotage.

Reviewing the black box against that, three properties turned out to be worse
than "low fidelity":

**It published the true wind, uncorrupted.** It read `true_disturbance` and
put the wind straight into `wind_estimate_mps`, and built `ground_velocity_mps`
from it. The equipment layer may read truth — that is ADR 0008's whole
allowance — but what it *publishes* must be an estimate. Any consumer reading
the wind channel was reading truth, and would have looked correct while being
unable to fail.

**It could not represent GNSS loss.** It never set `gnss_available`, so the
field took its dataclass default and every estimate claimed GNSS was fine. A
consumer that degrades on GNSS loss could not be exercised against it at all.

**Its error was white.** No temporal correlation, so any consumer that
filtered or differenced the estimate averaged the error away. Real navigation
error is strongly correlated in time. Every downstream result was flattered by
an amount that would not appear on any plot.

Two further facts settled it. Nothing outside the tests constructed it — both
demos already used the composed stack. And roughly forty lines of
`navigation_manager`'s docstring existed to warn maintainers not to fuse it,
not to arbitrate on it, and not to let its constant covariance win a
lowest-sigma contest. A docstring is the weakest guard in this repository;
everything else here is enforced by a test.

## Decision

**`IntegratedNavUnit` and its reference configuration are removed.** The
composed IMU + GNSS + air data + `InsGnssEstimator` stack is the only
navigation solution.

**`OwnStateSource` and `consumes_measurements` stay.** They are not
scaffolding for the black box. A datalink-supplied position, or another
platform's published estimate, satisfies `OwnStateSource` without consuming a
measurement stream, and that case is worth keeping the branch for. It is
covered by `_SourceWithoutMeasurements` in `tests/test_navigation_manager.py`.

**A test double belongs in `tests/`.** That is the general rule this record
establishes, and it is the reason the stub was not simply moved to another
module under `src/`. A fake own-state source in the shipped equipment layer
can be composed into a real platform and used for a claim about navigation
performance. Its docstring saying otherwise does not prevent that; being
unimportable from `ose` does.

**The arbitration rule generalises and is kept.** Never arbitrate on a source
whose covariance cannot degrade: it wins every lowest-sigma contest at exactly
the moment the honest source starts struggling, and the outage disappears from
the results. That was written about a specific component and is true of any.

## Consequences

**Standing up navigation is now more work — and that is all it is.** Four
components, independent RNG streams, an initial guess, and a per-step
measurement schedule: about twenty lines, against three. That is a *wiring*
cost, not a capability gap, and the distinction decides what to do about it.

A fake is warranted only when the real thing cannot be used. This one can:
consistent on every channel and every internal state, honest about GNSS loss,
and correlated the way real navigation error is. There is no property a
fiction would supply that the composed stack does not already supply better,
so the answer to the wiring cost is never a fiction. It is composition, which
this repository has already named as a thing it will own — the binder in
`docs/40-composition-spec.md` is the designated home for exactly this, and
until it exists the tests share `_build_components` and the demos each wire it
once.

**Using navigation and reading it are different, and only the second got
harder.** A consumer binds to `NavigationManager` and receives an
`OwnStateEstimate`; that is unchanged, and nobody has to read the error-state
filter to fly a planner against it. What is gone is the version a student
could read end to end in one sitting. For a teaching repository that is a
genuine loss on the *reading* side — but the black box was never what made
navigation usable, only what made it unwired.

**Some history now names a component that does not exist.** ADRs 0009, 0012
and 0014 refer to it, and their Context is kept as written because that is the
reasoning those decisions came from. Each statement that stopped being true
carries a pointer here.

**ADR 0014 loses its concrete second source.** Its decision — one publisher,
no fusion, no arbitration on a non-degrading source — is unchanged and still
worth having, but nothing currently violates it, so the constructor taking one
source now prevents a configuration that cannot presently be built. That is
deliberate: the second source is the one that arrives without anyone thinking
about the arithmetic.

**One equipment component fewer answers `capability()`.** The discovery test
walks the package, so it adapted without edit.

## Related

- ADR 0008 — the truth boundary. The wind channel was the leak.
- ADR 0009 — the navigation split, which introduced the component.
- ADR 0014 — one navigation publisher per platform, and why it does not fuse.
