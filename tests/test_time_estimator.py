"""Tests for the time estimator.

The important one is test_offset_uncertainty_is_consistent -- the same NEES
argument as test_filter_is_consistent in test_navigation_state_estimator.py.
This component makes no correction, but the covariance it reports is still
a real claim: that platform_time_s lies within a few offset_sigma_s of true
elapsed time. An overconfident clock estimate is exactly as dangerous as an
overconfident nav filter -- anything downstream that trusts a tight bound
around a badly-drifted clock will silently misinterpret timestamps.

test_estimator_cannot_see_truth and test_replay_determinism check the same
two properties established for the navigation estimator (ADR 0009): no
truth-carrying type in the signature, and purity as a function of the
measurement stream.
"""

import ast
from pathlib import Path

import numpy as np
import pytest

from _truth_boundary import (
    assert_no_equipment_imports,
    assert_no_truth_parameters,
    component_path,
)

from ose import interfaces
from ose.equipment.clock import Clock
from ose.equipment.reference_configs.reference_clock import STANDARD
from ose.interfaces import ClockMeasurement
from ose.subsystem.time_state_estimator import TimeEstimator, TimeEstimatorParameters


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_estimator_cannot_see_truth():
    path = component_path("subsystem", "time_state_estimator.py")
    assert_no_equipment_imports(path)
    assert_no_truth_parameters(path)


def test_estimator_satisfies_the_protocol():
    estimator = TimeEstimator()
    assert isinstance(estimator, interfaces.TimeEstimator)


# --------------------------------------------------------------------------
# Ordering contract
# --------------------------------------------------------------------------

def test_out_of_order_ingestion_raises_value_error():
    estimator = TimeEstimator()
    estimator.ingest(ClockMeasurement(5.0, 1.0, 1.0e-8))
    with pytest.raises(ValueError):
        estimator.ingest(ClockMeasurement(1.0, 1.0, 1.0e-8))


def test_ingesting_unknown_type_raises_type_error():
    estimator = TimeEstimator()
    with pytest.raises(TypeError):
        estimator.ingest(object())


# --------------------------------------------------------------------------
# Purity: replay determinism
# --------------------------------------------------------------------------

def test_replay_determinism():
    clock = Clock(STANDARD, rng=np.random.default_rng(9))
    live = TimeEstimator()
    record = []
    live_results = []

    dt = 0.5
    t = 0.0
    for _ in range(500):
        m = clock.sample(t, dt)
        live.ingest(m)
        record.append(m)
        live_results.append(live.estimate(t))
        t += dt

    replay = TimeEstimator()
    replay_results = []
    for m in record:
        replay.ingest(m)
        replay_results.append(replay.estimate(0.0))  # t_s is just a stamp

    assert len(live_results) == len(replay_results)
    for a, b in zip(live_results, replay_results):
        assert a.platform_time_s == b.platform_time_s
        assert a.drift_rate == b.drift_rate
        assert np.array_equal(a.covariance, b.covariance)


# --------------------------------------------------------------------------
# Dead reckoning: what this component actually does
# --------------------------------------------------------------------------

def test_platform_time_is_the_running_sum_of_readings():
    """No correction exists, so the point estimate is exactly the
    accumulated readings -- nothing is filtered out of the mean."""
    clock = Clock(STANDARD, rng=np.random.default_rng(2))
    estimator = TimeEstimator()

    dt = 0.1
    t = 0.0
    total = 0.0
    for _ in range(1000):
        m = clock.sample(t, dt)
        total += m.elapsed_s
        estimator.ingest(m)
        t += dt

    assert estimator.estimate(t).platform_time_s == total


def test_uncertainty_grows_monotonically():
    """Dead reckoning only: nothing ever shrinks offset_sigma_s."""
    clock = Clock(STANDARD, rng=np.random.default_rng(4))
    estimator = TimeEstimator()

    dt = 1.0
    t = 0.0
    sigmas = []
    for _ in range(500):
        m = clock.sample(t, dt)
        estimator.ingest(m)
        sigmas.append(estimator.estimate(t).offset_sigma_s)
        t += dt

    assert all(b >= a - 1e-15 for a, b in zip(sigmas, sigmas[1:]))


# --------------------------------------------------------------------------
# Consistency -- the one that matters
# --------------------------------------------------------------------------

@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_offset_uncertainty_is_consistent(seed):
    """NEES of platform_time_s against the true accumulated elapsed time,
    which only this test tracks -- the estimator never sees it. NEES far
    above one means the reported offset_sigma_s cannot be trusted.
    """
    clock = Clock(STANDARD, rng=np.random.default_rng(seed))
    estimator = TimeEstimator(TimeEstimatorParameters())

    dt = 1.0
    t = 0.0
    true_elapsed = 0.0
    nees = []
    for _ in range(2000):
        m = clock.sample(t, dt)
        true_elapsed += dt
        estimator.ingest(m)
        est = estimator.estimate(t + dt)
        err = est.platform_time_s - true_elapsed
        var = max(est.covariance[0, 0], 1e-30)
        nees.append(err * err / var)
        t += dt

    # Skip the initial transient, same rationale as test_filter_is_consistent.
    tail = nees[len(nees) // 2 :]
    assert np.mean(tail) < 6.0, f"offset NEES = {np.mean(tail):.1f}, estimator overconfident"
