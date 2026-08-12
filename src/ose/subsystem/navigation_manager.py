"""
Navigation manager: the platform's single publisher of vehicle.state.v1.

Subsystem-layer: purely cyber. It must not import VehicleState or Disturbance
from ose.equipment.vehicle, and no public method may take a parameter whose
name begins with true_ -- see test_manager_cannot_see_truth. It reads only
estimates that other components have already published.

A *navigation system* on a platform is composed of this manager plus whatever
produces the estimate underneath it -- today, an InsGnssEstimator fed by Imu,
GnssReceiver and AirDataSensor. Consumers bind to the manager and to nothing
below it, so guidance and planning do not change when the navigation
underneath is swapped.

The source need not be an estimator. OwnStateSource asks only for estimate();
a position arriving over a datalink, or another platform's published estimate,
would satisfy it without consuming a measurement stream. That is why
consumes_measurements exists.

WHAT THIS DOES NOT DO, and why
------------------------------
It does not fuse. It owns exactly one source and republishes what that source
says.

Only one source exists today, so nothing here is currently prevented. The
constructor takes one source and there is no way to hand it two, and that is
deliberate: the second source is the one that will arrive without anyone
thinking about the arithmetic.

Fusion of two sources that are not independent is worse than useless. Merging
them reduces the reported variance, so the platform publishes an estimate
better than either input with a covariance shrunk to match, and every number
is internally consistent while meaning nothing. That is the same class of
silent invalidation as the truth boundary (ADR 0008) and the overconfident
filter that motivated the NEES tests. Real fusion belongs here when a platform
genuinely carries more than one *independent* source -- INS/GNSS alongside
terrain-referenced navigation, or a second independent INS. Whoever adds it
must handle the cross-covariance, or use covariance intersection, and must add
a NEES test: naive fusion of correlated estimates is overconfident, which is
the classic track-fusion trap.

Which source a platform uses is therefore settled at composition time, not at
runtime. The manager holds the one it was built with and never reconsiders.

AND IF YOU ADD RUNTIME ARBITRATION
----------------------------------
Arbitration -- hold several sources, publish whichever is currently best -- is
sound where fusion is not, because choosing an estimate does not shrink its
covariance. It is the most likely thing to be added here.

The rule it must obey: **arbitrate only between sources that can all
degrade.** A source whose covariance is constant -- because it is not
modelling anything that could get worse -- wins every "pick the lowest sigma"
contest at exactly the moment the honest source starts struggling. An
InsGnssEstimator's position sigma grows from under a metre to around eight
during a GNSS outage; anything with a fixed sigma would be selected there, and
the outage would silently disappear from the results.

This is not hypothetical. The repository shipped such a source, a black-box
unit publishing truth plus fixed white noise, and it was removed for reasons
that included this one. See ADR 0019, and ADR 0014 for the original reasoning
about one publisher per platform.
"""

from __future__ import annotations

from ose.interfaces import NavigationEstimator, OwnStateEstimate, OwnStateSource


class NavigationManager:
    """Owns one own-state source and publishes the platform's estimate."""

    def __init__(self, source: OwnStateSource) -> None:
        self.source = source

    @property
    def consumes_measurements(self) -> bool:
        """Whether the source underneath is fed a measurement stream, as an
        InsGnssEstimator is, or arrives at its estimate some other way -- a
        position received over a datalink would not."""
        return isinstance(self.source, NavigationEstimator)

    def ingest(self, measurement) -> None:
        """Forward a measurement to the source.

        Present so a caller talks to the navigation system through one object
        rather than reaching past the manager to the estimator it owns; the
        ordering contract in ADR 0009 is the caller's to honour either way.

        Raises TypeError when the source does not consume measurements. That
        combination -- sensors publishing measurements while the platform's
        navigation ignores them -- is a configuration error, and a silent
        no-op here would hide it behind plausible-looking output.
        """
        if not self.consumes_measurements:
            raise TypeError(
                f"{type(self.source).__name__} does not consume measurements: it "
                "produces its own estimate. Configuring navigation sensors "
                "alongside it is a mistake -- the measurements would be "
                "silently discarded."
            )
        self.source.ingest(measurement)

    def estimate(self, t_s: float) -> OwnStateEstimate:
        """The platform's own-state estimate. Satisfies OwnStateSource."""
        return self.source.estimate(t_s)
