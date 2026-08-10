# 0014. One navigation publisher per platform, and it does not fuse

Status: accepted
Date: 2026-08-10

## Context

Two components published `vehicle.state.v1`: `InsGnssEstimator` in the
subsystem layer, and `IntegratedNavUnit` in the resource layer as the
deliberate layer collapse ADR 0009 describes. A consumer could bind to either,
and nothing prevented a platform being configured with both -- sensors and an
estimator alongside a black box that ignores them.

Nothing in the code was actually coupled to a source: `VehicleGuidance` takes
`own_state` as a parameter rather than holding a reference. So this was a
composition question, not a wiring one. But it left two things unsettled: which
component a consumer is supposed to bind to, and what a platform configured
with both sources should do.

The tempting answer to the second was to fuse them, and that is a trap worth
recording. `IntegratedNavUnit` is a fiction -- truth plus white noise, used
precisely when navigation is *not* the thing under test. `InsGnssEstimator` is
a model of a real navigation system. They are alternatives, not complements.
Fusing them would reduce the variance, so the platform would publish an
estimate better than either input with a covariance shrunk to match. Every
number would be internally consistent, and the result would mean nothing: a lab
using the black box as cheap scaffolding would silently get artificially good
navigation. That is the same class of silent invalidation as a truth leak
(ADR 0008) or the overconfident filter that motivated the NEES tests.

## Decision

`NavigationManager` (`subsystem/navigation_manager.py`) is the platform's
single publisher of `vehicle.state.v1`. It owns exactly one own-state source
and republishes what that source says. Consumers bind to the manager and to
nothing below it.

A *navigation system* is therefore composed: the manager, plus whatever
produces the estimate underneath -- an `InsGnssEstimator` fed by `Imu`,
`GnssReceiver` and `AirDataSensor`, or a black-box `IntegratedNavUnit`.

It does not fuse, and the nonsensical configuration is made impossible rather
than given an averaging rule: the constructor takes one source, so there is no
way to hand it two. `ingest()` forwards measurements to sources that consume
them and raises `TypeError` for those that do not, because sensors publishing
into a black box that ignores them is a configuration error that a silent
no-op would hide behind plausible output.

`IntegratedNavUnit` stays in the resource layer and keeps reading truth. The
manager cannot drive it -- `update()` takes ground truth, which a subsystem
component may not touch -- so the simulation core updates it as it does any
resource, and the manager only reads the estimate it publishes.

The choice of source is made at composition time and never revisited at
runtime. Runtime arbitration -- hold several, publish whichever is currently
best -- is sound in principle, and unlike fusion it adds no false confidence,
since choosing an estimate does not shrink its covariance. But it must never
treat an `IntegratedNavUnit` as a candidate. Its covariance is constant,
because truth plus fixed noise cannot degrade; a real estimator's grows
honestly, from under a metre to around twenty during a GNSS outage. Any
lowest-sigma rule would therefore select the estimator while aided and switch
to the fiction exactly when the real system starts struggling, and outages
would vanish from every result. A source that never degrades always wins
against one that honestly does; arbitrate only between sources that can all
be wrong.

Real fusion belongs in this component when a platform genuinely carries more
than one independent source: INS/GNSS alongside terrain-referenced navigation,
or a second independent INS. Whoever adds it must handle the cross-covariance
or use covariance intersection, and must add a NEES test. Naive fusion of
correlated estimates is overconfident, which is the classic track-fusion trap.

## Consequences

Guidance and planning bind to one component whose identity does not change
when the navigation underneath is swapped, which is the substitution property
the layering exists to provide. `demo_navigation.py` now drives the manager
and produces numerically identical output, confirming it is a pass-through.

The manager is, today, almost nothing: it holds a reference, forwards two
calls, and raises on a third. That is deliberate -- it names the seam where
fusion and source arbitration will live without inventing either now -- but it
is fair to call it ceremony until something fills it. The first thing likely
to fill it is not fusion but failover, dropping to a degraded source when the
primary estimator diverges. That needs a second source which degrades
honestly, such as an air-data-and-heading dead reckoner; the black box cannot
serve, for the reason above.

`InsGnssEstimator` still satisfies `OwnStateSource`, so "one publisher" is a
composition rule rather than something the type system enforces. A consumer
can still reach past the manager to the estimator. The descriptor validator
should reject a platform with more than one bound `vehicle.state.v1` producer
once it exists; until then the rule is documentation and a constructor that
takes one argument.
