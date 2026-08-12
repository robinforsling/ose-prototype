"""Tests for the navigation manager.

The one worth reading is test_manager_refuses_to_fuse_alternatives. The manager
exists partly to make a nonsensical configuration impossible rather than to
give it an averaging rule -- fusing two sources that are not independent
reports an estimate better than either input, with a covariance shrunk to
match, and looks entirely self-consistent while meaning nothing. See ADR 0014.

The manager supports sources that do not consume measurements, and after
ADR 0019 removed the black-box unit no such component ships. The stub below
covers that half of the interface. It lives here rather than in src precisely
because it is a test double: a fake own-state source in the equipment layer
can be composed into a real platform and used for a claim about navigation
performance, which is what happened to the component it replaces.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from _truth_boundary import (
    assert_no_truth_parameters,
    assert_no_truth_types,
    component_path,
)

from ose import interfaces
from ose.interfaces import ClockMeasurement, OwnStateEstimate
from ose.subsystem.navigation_manager import NavigationManager
from ose.subsystem.navigation_state_estimator import (
    InitialUncertainty,
    InsGnssEstimator,
)


def _estimator() -> InsGnssEstimator:
    return InsGnssEstimator(
        np.zeros(2), 0.0, np.array([250.0, 0.0]),
        initial_uncertainty=InitialUncertainty(),
    )


class _SourceWithoutMeasurements:
    """Satisfies OwnStateSource and deliberately not NavigationEstimator.

    A datalink-supplied position or another platform's published estimate
    would look like this: it answers estimate() and has no ingest(). The
    manager's consumes_measurements branch exists for that case, and without
    something shaped like this the branch would be untested.

    It returns a fixed estimate. That is fine for a binding test and is the
    reason it must never be a shipped component -- a constant covariance wins
    every arbitration contest against a source that degrades honestly. See the
    navigation_manager docstring and ADR 0019.
    """

    def estimate(self, t_s: float) -> OwnStateEstimate:
        return OwnStateEstimate(
            t_s=t_s,
            p_x_m=100.0,
            p_y_m=-50.0,
            psi_rad=0.3,
            v_air_mps=250.0,
            ground_velocity_mps=np.array([238.8, 73.9]),
            wind_estimate_mps=np.zeros(2),
            covariance=np.diag([25.0, 25.0, 1.0e-4, 1.0]),
        )


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_manager_cannot_see_truth():
    path = component_path("subsystem", "navigation_manager.py")
    assert_no_truth_types(path)
    assert_no_truth_parameters(path)


# --------------------------------------------------------------------------
# Publishing
# --------------------------------------------------------------------------

def test_satisfies_own_state_source():
    assert isinstance(NavigationManager(_estimator()), interfaces.OwnStateSource)
    assert isinstance(NavigationManager(_SourceWithoutMeasurements()), interfaces.OwnStateSource)


def _same_estimate(a: OwnStateEstimate, b: OwnStateEstimate) -> bool:
    """OwnStateEstimate carries numpy arrays, so `==` on the dataclass is
    elementwise and ambiguous. Compare field by field."""
    return (
        (a.t_s, a.p_x_m, a.p_y_m, a.psi_rad, a.v_air_mps, a.gnss_available)
        == (b.t_s, b.p_x_m, b.p_y_m, b.psi_rad, b.v_air_mps, b.gnss_available)
        and np.array_equal(a.ground_velocity_mps, b.ground_velocity_mps)
        and np.array_equal(a.wind_estimate_mps, b.wind_estimate_mps)
        and np.array_equal(a.covariance, b.covariance)
    )


def test_publishes_exactly_what_the_source_says():
    """A pass-through today, and that is the point: the manager adds a single
    binding point, not arithmetic. Nothing is scaled, blended or re-covaried
    on the way through."""
    for source in (_estimator(), _SourceWithoutMeasurements()):
        manager = NavigationManager(source)
        assert _same_estimate(manager.estimate(3.0), source.estimate(3.0))


def test_consumers_need_not_know_which_source_is_underneath():
    """The substitution property the manager exists for: both configurations
    publish the same shape, so guidance does not change when navigation is
    swapped."""
    for source in (_estimator(), _SourceWithoutMeasurements()):
        est = NavigationManager(source).estimate(1.0)
        assert isinstance(est, OwnStateEstimate)
        assert est.covariance.shape == (4, 4)
        assert math.isfinite(est.p_x_m)


# --------------------------------------------------------------------------
# Refusing the configuration that does not make sense
# --------------------------------------------------------------------------

def test_manager_refuses_to_fuse_alternatives():
    """There is no way to hand the manager two sources. Fusing sources that
    are not independent is actively misleading, so the constructor makes it
    impossible rather than defining behaviour for it."""
    with pytest.raises(TypeError):
        NavigationManager(_estimator(), _SourceWithoutMeasurements())      # type: ignore[call-arg]


def test_forwards_measurements_to_an_estimator_source():
    estimator = _estimator()
    manager = NavigationManager(estimator)
    assert manager.consumes_measurements

    before = estimator.estimate(0.0)
    manager.ingest(
        interfaces.GnssFix(
            valid_time_s=1.0,
            position_m=np.array([500.0, -200.0]),
            position_sigma_m=3.0,
            velocity_mps=None,
            velocity_sigma_mps=None,
        )
    )
    after = estimator.estimate(1.0)
    assert (before.p_x_m, before.p_y_m) != (after.p_x_m, after.p_y_m)


def test_ingesting_into_a_source_that_ignores_measurements_raises():
    """Sensors publishing measurements while the platform's navigation
    ignores them is a configuration error. A silent no-op would hide it behind
    plausible output."""
    manager = NavigationManager(_SourceWithoutMeasurements())
    assert not manager.consumes_measurements

    with pytest.raises(TypeError, match="does not consume measurements"):
        manager.ingest(ClockMeasurement(1.0, 0.05, 1.0e-8))
