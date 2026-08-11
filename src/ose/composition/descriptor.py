"""
Records mirroring the capability descriptor and platform specification of
docs/40-composition-spec.md, sections 4 and 5.

Python records rather than a YAML loader, deliberately. The checks are the
part with reasoning in them and the part that can be wrong; parsing is
mechanical and needs a dependency decision this repository has not made --
there is no YAML or schema library in its dependencies today. Splitting them
means the rules can be built and tested now, and a loader added later without
touching them.

Only the fields the load checks read are modelled. A descriptor carries more
than this -- ports, an envelope, parameter bounds -- and those belong with the
checks that consume them, not here in anticipation.
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
    stations: tuple[Station, ...] = ()


@dataclass(frozen=True)
class ComponentDescriptor:
    """Section 4, reduced to what the load checks need."""

    type: str
    layer: str
    category: str
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
    """Section 5, reduced likewise."""

    id: str
    vehicle_type: str
    attachments: tuple[Attachment, ...] = ()
