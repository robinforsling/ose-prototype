"""Descriptors must describe components that exist, and describe them correctly.

`ose/composition/catalogue.py` is authored by hand, because a descriptor has to
be readable without constructing anything -- that is what lets a binder
validate a platform before it is built. Authored data drifts, and this
repository has been bitten by that often enough to have a rule about it.

So every derivable field is checked against the code:
tools/generate_architecture_diagram.py already derives, per component class,
what it publishes and consumes and which layer it lives in. A descriptor that
claims a port the class does not have, or misses one it does, fails here.

This is the join between the two models of "a component" that had never met.
Before it, the composition checks ran entirely against five fixtures naming
classes that do not exist. See ADR 0025.
"""

from __future__ import annotations

import dataclasses
import importlib
import importlib.util
import sys
from pathlib import Path

import pytest

from ose.composition.catalogue import CATALOGUE
from ose.composition.descriptor import ComponentDescriptor, Port

ROOT = Path(__file__).resolve().parents[1]
GENERATOR = ROOT / "tools" / "generate_architecture_diagram.py"


def _load_generator():
    spec = importlib.util.spec_from_file_location("generate_architecture_diagram",
                                                  GENERATOR)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def resolve(implementation: str) -> type:
    """"module:Class" -> the class. The binder's job, done here to check it."""
    module_name, _, class_name = implementation.partition(":")
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


@pytest.fixture(scope="module")
def derived():
    """What the source says, per component CLASS.

    Uncollapsed: a descriptor names an implementation and an implementation is
    a class, so the Vehicle port the diagram draws is the wrong granularity
    here.
    """
    generator = _load_generator()
    graph = generator.build_graph(collapse=False)
    return {
        name: {
            "layer": layer,
            "provides": {i for c, i in graph.publishes if c == name},
            "requires": (
                {i for c, i in graph.consumes if c == name}
                | {i for _, c, i in graph.port_edges() if c == name}
            ),
        }
        for name, layer in graph.components.items()
    }


def test_the_catalogue_is_not_vacuous():
    """Every test below iterates the catalogue."""
    assert len(CATALOGUE) >= 10, f"only {len(CATALOGUE)} descriptors"
    assert all(isinstance(d, ComponentDescriptor) for d in CATALOGUE.values())


def test_every_type_key_matches_its_descriptor():
    for key, descriptor in CATALOGUE.items():
        assert key == descriptor.type, f"catalogue key {key} != type {descriptor.type}"


def test_every_implementation_resolves_to_a_class():
    """A descriptor naming a class that does not exist is the failure this
    whole module exists for -- it is what every descriptor in the repository
    did before the catalogue."""
    for descriptor in CATALOGUE.values():
        assert descriptor.implementation, f"{descriptor.type} names no implementation"
        cls = resolve(descriptor.implementation)
        assert isinstance(cls, type), f"{descriptor.implementation} is not a class"


def test_every_component_class_has_a_descriptor(derived):
    """Set equality against the components the source has.

    A component with no descriptor cannot be composed, and would be invisible
    to every composition-time check. Equality rather than containment, so a
    descriptor for a class that has been deleted fails too.
    """
    described = {resolve(d.implementation).__name__ for d in CATALOGUE.values()}
    assert described == set(derived), (
        f"undescribed components {sorted(set(derived) - described)}; "
        f"descriptors for nothing {sorted(described - set(derived))}"
    )


def test_declared_layer_matches_the_package_the_class_lives_in(derived):
    for descriptor in CATALOGUE.values():
        name = resolve(descriptor.implementation).__name__
        assert descriptor.layer == derived[name]["layer"], (
            f"{descriptor.type} declares layer {descriptor.layer}, but "
            f"{name} lives in {derived[name]['layer']}"
        )


def test_declared_ports_match_the_derived_ones(derived):
    """The heart of it.

    Compared as sets of interfaces, not of ports: a component may hold two
    ports on one interface, and matching at bind time is on the interface
    anyway.
    """
    for descriptor in CATALOGUE.values():
        name = resolve(descriptor.implementation).__name__
        for direction in ("provides", "requires"):
            declared = {p.interface for p in getattr(descriptor, direction)}
            actual = derived[name][direction]
            assert declared == actual, (
                f"{descriptor.type} {direction}: declared {sorted(declared)}, "
                f"code says {sorted(actual)}"
            )


def test_a_wrong_port_is_caught(derived):
    """Sabotage.

    Without it the test above asserts only that a correct catalogue is
    correct, and would pass just as happily if the comparison were removed.
    Both directions are checked: a descriptor claiming an interface its class
    does not publish, and one omitting an interface its class does.
    """
    good = CATALOGUE["nav_sensor.imu.tactical"]
    name = resolve(good.implementation).__name__

    bent = dataclasses.replace(
        good, provides=(Port("measurement", "sensing.gnss.v1"),)
    )
    declared = {p.interface for p in bent.provides}
    assert declared != derived[name]["provides"], (
        "a descriptor claiming the wrong interface was indistinguishable from "
        "a correct one"
    )

    missing = dataclasses.replace(good, provides=())
    assert {p.interface for p in missing.provides} != derived[name]["provides"]


def test_a_bad_implementation_path_is_caught():
    """Sabotage for the resolution test."""
    with pytest.raises((ImportError, AttributeError)):
        resolve("ose.equipment.imu:NoSuchClass")
    with pytest.raises(ImportError):
        resolve("ose.equipment.no_such_module:Imu")


def test_only_equipment_declares_a_physical_part():
    """A cyber component with a mass would be inventing hardware.

    The truth boundary's quieter cousin: mass, power and stations are
    properties of something with a physical part, and only the equipment layer
    has one.
    """
    for descriptor in CATALOGUE.values():
        if descriptor.layer == "equipment":
            continue
        assert descriptor.consumes.mass_kg == 0.0, f"{descriptor.type} has mass"
        assert not descriptor.consumes.power_kw, f"{descriptor.type} draws power"
        assert descriptor.consumes.station_type is None, (
            f"{descriptor.type} wants a station"
        )
        assert descriptor.supplies.stations == (), f"{descriptor.type} has stations"


def test_the_vehicle_mass_ceiling_matches_its_own_constraints():
    """A descriptor's mass ceiling is authored, but it is not free to disagree
    with the model it describes -- a ceiling that contradicted the constraints
    would be worse than none, because the mass budget check is built on it."""
    from ose.equipment.reference_configs.vehicle.planar_point_mass import (
        FIGHTER_LIMITS,
    )

    for type_name in ("vehicle.fighter.generic_2d", "vehicle.fighter.boosted_2d"):
        descriptor = CATALOGUE[type_name]
        assert descriptor.supplies.max_mass_kg == FIGHTER_LIMITS.mass_max_kg, (
            f"{type_name} declares a ceiling of "
            f"{descriptor.supplies.max_mass_kg} kg against the model's "
            f"{FIGHTER_LIMITS.mass_max_kg} kg"
        )
