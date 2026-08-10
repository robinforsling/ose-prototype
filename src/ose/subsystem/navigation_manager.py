"""
Navigation manager: the platform's single publisher of vehicle.state.v1.

Subsystem-layer: purely cyber. It must not import VehicleState or Disturbance
from ose.resource.vehicle, and no public method may take a parameter whose
name begins with true_ -- see test_manager_cannot_see_truth. It reads only
estimates that other components have already published.

A *navigation system* on a platform is composed of this manager plus whatever
produces the estimate underneath it: an InsGnssEstimator fed by Imu,
GnssReceiver and AirDataSensor, or a black-box IntegratedNavUnit. Consumers
bind to the manager and to nothing below it, so guidance and planning do not
change when the navigation underneath is swapped.

WHAT THIS DOES NOT DO, and why
------------------------------
It does not fuse. It owns exactly one source and republishes what that source
says.

Fusing the two sources that exist today would be worse than useless. An
IntegratedNavUnit is a fiction -- truth plus white noise, used precisely when
navigation is not the thing under test -- and an InsGnssEstimator is a model of
a real one. They are alternatives, not complements. Merging them would reduce
the variance, so the platform would report an estimate better than either input
with a covariance that shrank to match, and every number would be internally
consistent while meaning nothing. A lab using the black box as cheap
scaffolding would silently get artificially good navigation. That is the same
class of silent invalidation as the truth boundary (ADR 0008) and the
overconfident filter that motivated the NEES tests.

So a nonsensical configuration is made impossible rather than given an
averaging rule: the constructor takes one source, and there is no way to hand
it two.

Real fusion belongs here when there is genuinely more than one independent
source -- INS/GNSS alongside terrain-referenced navigation, or a second
independent INS. Whoever adds it must handle the cross-covariance, or use
covariance intersection, and must add a NEES test: naive fusion of correlated
estimates is overconfident, which is the classic track-fusion trap and exactly
what this repository's consistency discipline exists to catch. See ADR 0014.
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
        InsGnssEstimator is, or produces its estimate some other way, as a
        black-box IntegratedNavUnit does."""
        return isinstance(self.source, NavigationEstimator)

    def ingest(self, measurement) -> None:
        """Forward a measurement to the source.

        Present so a caller talks to the navigation system through one object
        rather than reaching past the manager to the estimator it owns; the
        ordering contract in ADR 0009 is the caller's to honour either way.

        Raises TypeError when the source does not consume measurements. That
        combination -- sensors publishing measurements while the platform's
        navigation is a black box that ignores them -- is a configuration
        error, and a silent no-op here would hide it behind plausible-looking
        output.
        """
        if not self.consumes_measurements:
            raise TypeError(
                f"{type(self.source).__name__} does not consume measurements: it "
                "produces its own estimate. Configuring navigation sensors "
                "alongside it is a mistake -- use either a black-box unit or "
                "sensors with an estimator, not both."
            )
        self.source.ingest(measurement)

    def estimate(self, t_s: float) -> OwnStateEstimate:
        """The platform's own-state estimate. Satisfies OwnStateSource."""
        return self.source.estimate(t_s)
