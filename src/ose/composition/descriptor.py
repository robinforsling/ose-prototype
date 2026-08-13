"""
Records mirroring the capability descriptor and platform specification of
docs/40-composition-spec.md, sections 4 and 5.

Python records rather than a YAML loader, deliberately. The checks are the
part with reasoning in them and the part that can be wrong; parsing is
mechanical and needs a dependency decision this repository has not made --
there is no YAML or schema library in its dependencies today. Splitting them
means the rules can be built and tested now, and a loader added later without
touching them.

Ports are modelled now that a check reads them (section 4 of the
specification, `provides` and `requires`). The envelope and the parameter
schema are still absent, on the same rule: they belong with the checks that
consume them, not here in anticipation.

Authored data, cross-checked against the code
---------------------------------------------
A descriptor states what a component is without constructing it -- that is what
lets a binder validate a platform before anything is built, and what lets this
be YAML one day. So the fields are written by hand, including the ones a
program could derive.

Which makes them able to drift, and this repository has been bitten by that
often enough to have a rule about it: tests/test_descriptor_catalogue.py
resolves every `implementation` and asserts the declared layer and ports match
what the architecture generator derives from the source. Authored, and checked
-- the same split as the generated tables in the model pages. See ADR 0025.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping


@dataclass(frozen=True)
class Station:
    """A mounting point on an airframe. Vehicles only, per the schema:
    stations are structure, and only the thing with structure has them."""

    name: str
    type: str                       # nose | fuselage | wing | internal | conformal
    mass_limit_kg: float


@dataclass(frozen=True)
class Consumes:
    """What a component draws when mounted.

    power_kw is per operating mode, keyed by mode name -- a radar draws
    differently in cruise and in combat. A component with no entry for a mode
    is taken to draw nothing in it, which is the reading that lets a
    descriptor stay silent about modes it does not care about.
    """

    mass_kg: float = 0.0
    power_kw: Mapping[str, float] = field(default_factory=dict)
    station_type: str | None = None
    station_count: int = 1


@dataclass(frozen=True)
class Supplies:
    """What a component makes available.

    power_kw is a single number, not per mode, and that asymmetry with
    Consumes is the specification's as written rather than a simplification
    made here. It does not survive contact with a switched engine: a vehicle
    whose generator is driven by the engine supplies different power in
    nominal and boost, and there is nowhere to say so. Recorded in section 11
    of the specification as an open question rather than silently fixed,
    because changing it is a format change and the format has other consumers
    coming.
    """

    power_kw: float = 0.0
    # Vehicles only, like stations and for the same reason: a mass ceiling is
    # a property of structure. It is the counterpart of Consumes.mass_kg --
    # the airframe provides a mass budget and what is mounted on it consumes
    # that budget, exactly as it provides stations that attachments occupy.
    max_mass_kg: float = 0.0
    stations: tuple[Station, ...] = ()


@dataclass(frozen=True)
class Port:
    """One service port a component offers or needs.

    `name` is local to the component and exists so a component can hold two
    ports of the same interface and tell them apart -- a fuser with two
    own-state inputs would need exactly that. Matching at bind time is on
    `interface`, never on the name.

    `optional` is the difference between a component that cannot run without a
    port and one that runs degraded. Nothing is optional today; the field is
    the specification's and costs nothing.
    """

    name: str
    interface: str                  # family.name.vN
    optional: bool = False


@dataclass(frozen=True)
class ComponentDescriptor:
    """Section 4, reduced to what the load checks and the binder need."""

    type: str
    layer: str
    category: str

    # Importable path to the class this type is built from, "module:Class".
    # A string rather than the class itself: a descriptor is data, and
    # resolving it is the binder's job, not this record's. It is also what
    # makes the cross-check possible without this module importing any layer.
    implementation: str = ""

    provides: tuple[Port, ...] = ()
    requires: tuple[Port, ...] = ()

    consumes: Consumes = field(default_factory=Consumes)
    supplies: Supplies = field(default_factory=Supplies)


@dataclass(frozen=True)
class Attachment:
    """One entry under a platform's `attachments`."""

    station: str                    # the station NAME it occupies
    type: str                       # the component type mounted there
    quantity: int = 1


@dataclass(frozen=True)
class PlatformSpec:
    """Section 5, reduced likewise.

    empty_mass_kg and fuel_kg are per-instance vehicle parameters in the
    specification's worked example, not descriptor fields: two platforms of
    the same type can be loaded differently. The ceiling they are checked
    against is a descriptor field, because that is a property of the type.
    """

    id: str
    vehicle_type: str
    attachments: tuple[Attachment, ...] = ()
    empty_mass_kg: float = 0.0
    fuel_kg: float = 0.0

    # The cyber layers, as type names. No stations and no attachment record:
    # a cyber component has no physical part, so there is nothing to mount it
    # on and nothing for the station and mass checks to say about it. The
    # specification's worked example has had these sections all along; only
    # `equipment` was modelled, because only the load checks existed.
    subsystems: tuple[str, ...] = ()
    single_ship: tuple[str, ...] = ()

    def component_types(self) -> tuple[str, ...]:
        """Every type this platform composes, in composition order."""
        return (
            (self.vehicle_type,)
            + tuple(a.type for a in self.attachments)
            + self.subsystems
            + self.single_ship
        )
