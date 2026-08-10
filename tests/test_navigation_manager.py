"""Tests for the navigation manager.

The one worth reading is test_manager_refuses_to_fuse_alternatives. The manager
exists partly to make a nonsensical configuration impossible rather than to
give it an averaging rule -- fusing a black-box IntegratedNavUnit with a real
InsGnssEstimator would report an estimate better than either input, with a
covariance shrunk to match, and look entirely self-consistent while meaning
nothing. See ADR 0014.
"""

import ast
import math
from pathlib import Path

import numpy as np
import pytest

from ose import interfaces
from ose.interfaces import ClockMeasurement, OwnStateEstimate
from ose.resource.integrated_navigation_unit import IntegratedNavUnit
from ose.resource.reference_configs.reference_integrated_navigation_unit import (
    STANDARD as INTEGRATED_NAV_STANDARD,
)
from ose.resource.reference_configs.reference_vehicle import reference_fighter
from ose.resource.vehicle import Disturbance, VehicleState
from ose.subsystem.navigation_manager import NavigationManager
from ose.subsystem.navigation_state_estimator import InitialUncertainty, InsGnssEstimator


def _estimator() -> InsGnssEstimator:
    return InsGnssEstimator(
        np.zeros(2), 0.0, np.array([250.0, 0.0]),
        initial_uncertainty=InitialUncertainty(),
    )


def _black_box() -> IntegratedNavUnit:
    unit = IntegratedNavUnit(INTEGRATED_NAV_STANDARD, rng=np.random.default_rng(0))
    # The core drives a resource-layer unit with truth; the manager never does.
    unit.update(0.0, VehicleState(0.0, 0.0, 0.0, 250.0, 16000.0), Disturbance())
    return unit


# --------------------------------------------------------------------------
# The truth boundary
# --------------------------------------------------------------------------

def test_manager_cannot_see_truth():
    path = (
        Path(__file__).resolve().parents[1]
        / "src" / "ose" / "subsystem" / "navigation_manager.py"
    )
    tree = ast.parse(path.read_text())

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "ose.resource.vehicle":
            names = {alias.name for alias in node.names}
            leaked = names & {"Disturbance", "VehicleState"}
            assert not leaked, f"imports truth-carrying types: {leaked}"

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and not node.name.startswith("_"):
            params = [a.arg for a in node.args.args + node.args.kwonlyargs]
            leaked = [p for p in params if p.startswith("true_")]
            assert not leaked, f"public method {node.name} takes truth: {leaked}"


# --------------------------------------------------------------------------
# Publishing
# --------------------------------------------------------------------------

def test_satisfies_own_state_source():
    assert isinstance(NavigationManager(_estimator()), interfaces.OwnStateSource)
    assert isinstance(NavigationManager(_black_box()), interfaces.OwnStateSource)


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
    for source in (_estimator(), _black_box()):
        manager = NavigationManager(source)
        assert _same_estimate(manager.estimate(3.0), source.estimate(3.0))


def test_consumers_need_not_know_which_source_is_underneath():
    """The substitution property the manager exists for: both configurations
    publish the same shape, so guidance does not change when navigation is
    swapped."""
    for source in (_estimator(), _black_box()):
        est = NavigationManager(source).estimate(1.0)
        assert isinstance(est, OwnStateEstimate)
        assert est.covariance.shape == (4, 4)
        assert math.isfinite(est.p_x_m)


# --------------------------------------------------------------------------
# Refusing the configuration that does not make sense
# --------------------------------------------------------------------------

def test_manager_refuses_to_fuse_alternatives():
    """There is no way to hand the manager two sources. Fusing the two that
    exist today would be actively misleading, so the constructor makes it
    impossible rather than defining behaviour for it."""
    with pytest.raises(TypeError):
        NavigationManager(_estimator(), _black_box())      # type: ignore[call-arg]


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


def test_ingesting_into_a_black_box_source_raises():
    """Sensors publishing measurements while the platform's navigation is a
    black box that ignores them is a configuration error. A silent no-op
    would hide it behind plausible output."""
    manager = NavigationManager(_black_box())
    assert not manager.consumes_measurements

    with pytest.raises(TypeError, match="does not consume measurements"):
        manager.ingest(ClockMeasurement(1.0, 0.05, 1.0e-8))
