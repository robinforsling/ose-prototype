"""
Navigation manager: the platform's single publisher of vehicle.state.v1 and
platform.time.v1 -- position, navigation and timing, published from one place.

Subsystem-layer: purely cyber. It must not import VehicleState or Disturbance
from ose.equipment.vehicle, and no public method may take a parameter whose
name begins with true_ -- see test_manager_cannot_see_truth. It reads only
estimates that other components have already published.

A *navigation system* on a platform is composed of this manager plus whatever
produces the estimates underneath it -- today, an InsGnssEstimator fed by Imu,
GnssReceiver and AirDataSensor, and a TimeEstimator fed by the platform Clock.
Consumers bind to the manager and to nothing below it, so guidance and
planning do not change when the navigation underneath is swapped.

Which is why the sources publish on their own ports -- vehicle.state_source.v1
and platform.time_source.v1 -- rather than on the ones this manager publishes.
The records are identical; the ports are not, and until they were named apart
nothing could tell a source estimate from the platform's answer. Binding a
consumer straight to the estimator worked and was wrong. See ADR 0021.

Timing is here rather than beside it because a platform has one answer to
"where am I and when is it", not two. Nothing couples them yet: `time()`
republishes what the time source says, exactly as `estimate()` does. In a real
GNSS receiver they are coupled -- clock bias is a filter state -- and this is
the shape that lets that arrive without moving consumers. See ADR 0022.

A source need not be an estimator. OwnStateSource asks only for estimate();
a position arriving over a datalink, or another platform's published estimate,
would satisfy it without consuming a measurement stream. That is why
consumes_measurements exists.

WHAT THIS DOES NOT DO, and why
------------------------------
It does not fuse. It owns exactly one own-state source and one time source,
and republishes what each says.

One own-state source exists today, so nothing here is currently prevented. The
constructor takes one and there is no way to hand it two alternatives -- the
time source is keyword-only precisely so that adding it did not open that door
-- and that is deliberate: the second source is the one that will arrive
without anyone thinking about the arithmetic.

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

from ose.interfaces import (
    NavigationEstimator,
    OwnStateEstimate,
    OwnStateSource,
    TimeEstimate,
    TimeEstimator,
)


class NavigationManager:
    """Owns the platform's own-state and time sources, and publishes both."""

    def __init__(
        self,
        source: OwnStateSource,
        *,
        time_source: TimeEstimator | None = None,
    ) -> None:
        # time_source is KEYWORD-ONLY, and that is load-bearing rather than
        # stylistic. test_manager_refuses_to_fuse_alternatives asserts that
        # NavigationManager(a, b) raises, which is how the no-fusion rule is
        # enforced rather than merely documented. A positional second
        # parameter would silently absorb that second argument as a time
        # source, the guard would stop raising, and the test would keep
        # passing while checking nothing.
        self.source = source
        self.time_source = time_source

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

    def time(self, t_s: float) -> TimeEstimate:
        """The platform's belief about the time.

        Republished from the bound time source, exactly as estimate() is
        republished from the own-state source, and for the same reason: one
        publisher per platform. A consumer that needs position and time gets
        both from the component that owns navigation, rather than binding a
        clock filter directly and leaving the platform with two answers about
        when it is.

        That is the P, N and T of PNT arriving at one place. It is a
        structural claim only -- nothing here couples the two, and in a real
        GNSS receiver they are coupled, with clock bias as a filter state. See
        ADR 0022.

        Raises when no time source was composed. A platform without one has no
        belief about the time, and returning a default would be inventing one.
        """
        if self.time_source is None:
            raise TypeError(
                "no time source was composed on this platform, so it has no "
                "belief about the time. Construct the manager with "
                "time_source= to publish platform.time.v1."
            )
        return self.time_source.estimate(t_s)
