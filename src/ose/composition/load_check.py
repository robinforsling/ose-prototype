"""
Composition-time load checks: can this platform, as specified, be built?

Checks 1 and 3 of docs/40-composition-spec.md section 6.1. Both are answerable
from the descriptors alone, before anything is constructed and long before the
clock starts.

What is NOT here, and why
-------------------------
Check 2, the mass budget, is specified as "empty mass, plus fuel, plus the sum
of attachment masses times quantity, is within the vehicle's maximum" -- and
nothing declares that maximum. The vehicle's constraint vector carries
mass_dry_kg and no upper bound, the model document's lambda has seven entries
and none of them is a maximum mass, and the specification's own worked example
gives a vehicle empty_mass_kg and fuel_initial_kg but no ceiling. Section 5 of
the specification says the envelope includes "empty and maximum mass", so the
intent exists; the declaration does not.

Attachment masses ARE summed here, per station, against that station's
mass_limit_kg, which is declared. So the structural half of the mass question
is checked and the whole-aircraft half is not. Adding a maximum would be a
change to the vehicle model's lambda and therefore to the model document,
which is a modelling decision rather than a validator one.

Checks 4 to 7 -- port satisfaction, the truth boundary, layer discipline and
parameter bounds -- need the port graph and the parameter schema, neither of
which is modelled yet.

Findings, not exceptions
------------------------
Every check returns findings rather than raising, so that one run reports
everything wrong with a platform instead of the first thing. A caller decides
what a finding means; check_load simply gathers them. This is not the
declare-and-report pattern the vehicle uses for its admissible sets -- there,
reporting exists because a run is already under way and clipping would hide
something. Here there is no run yet, and a platform with findings should not
be built.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from ose.composition.descriptor import ComponentDescriptor, PlatformSpec

CRUISE = "cruise"


@dataclass(frozen=True)
class Finding:
    """One reason a platform cannot be built as specified."""

    rule: str                       # "station" | "power"
    message: str

    def __str__(self) -> str:       # pragma: no cover - convenience only
        return f"[{self.rule}] {self.message}"


def _vehicle(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> ComponentDescriptor | None:
    return catalogue.get(platform.vehicle_type)


def check_stations(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Check 1. Each attachment names a station the vehicle has, of a type the
    attachment accepts, and the mass mounted there is within its limit.

    Quantity multiplies mass, per the specification's wording, and several
    attachments may share a station -- so the limit is checked against the
    total at that station rather than against each entry.
    """
    out: list[Finding] = []
    vehicle = _vehicle(platform, catalogue)
    if vehicle is None:
        return [Finding("station",
                        f"vehicle type {platform.vehicle_type!r} is not in the catalogue")]

    stations = {s.name: s for s in vehicle.supplies.stations}
    mounted: dict[str, float] = {}

    for attachment in platform.attachments:
        descriptor = catalogue.get(attachment.type)
        if descriptor is None:
            out.append(Finding(
                "station", f"{attachment.type!r} is not in the catalogue"))
            continue

        station = stations.get(attachment.station)
        if station is None:
            out.append(Finding(
                "station",
                f"{attachment.type} asks for station {attachment.station!r}, "
                f"which {vehicle.type} does not have "
                f"(it has {sorted(stations)})",
            ))
            continue

        wanted = descriptor.consumes.station_type
        if wanted is not None and wanted != station.type:
            out.append(Finding(
                "station",
                f"{attachment.type} needs a {wanted!r} station but "
                f"{station.name!r} is {station.type!r}",
            ))

        mounted[station.name] = (
            mounted.get(station.name, 0.0)
            + descriptor.consumes.mass_kg * attachment.quantity
        )

    for name, mass in sorted(mounted.items()):
        limit = stations[name].mass_limit_kg
        if mass > limit:
            out.append(Finding(
                "station",
                f"station {name!r} carries {mass:.0f} kg against a limit of "
                f"{limit:.0f} kg",
            ))
    return out


def operating_modes(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[str]:
    """Every mode any attachment has an opinion about.

    Discovered from the descriptors rather than configured, because there is
    nowhere in the schema for a vehicle to enumerate its modes -- `supplies`
    carries a single power figure and no mode list. A component silent about a
    mode draws nothing in it, so the union is the set that needs checking.

    CRUISE is always included: a platform whose attachments all happen to be
    silent still has to be able to fly, and a check that vacuously passed on
    an empty mode set would be worse than no check.
    """
    modes = {CRUISE}
    for attachment in platform.attachments:
        descriptor = catalogue.get(attachment.type)
        if descriptor is not None:
            modes.update(descriptor.consumes.power_kw)
    return sorted(modes)


def check_power_budget(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Check 3. In every operating mode, what is drawn is within what is
    supplied.

    Supply is summed across everything on the platform that declares it, not
    read off the vehicle alone. That is what allows a generator to be a
    component rather than a vehicle parameter, and it is the line that would
    silently under-count if it were left as a single lookup.
    """
    out: list[Finding] = []
    vehicle = _vehicle(platform, catalogue)
    if vehicle is None:
        return [Finding("power",
                        f"vehicle type {platform.vehicle_type!r} is not in the catalogue")]

    supplied = vehicle.supplies.power_kw
    for attachment in platform.attachments:
        descriptor = catalogue.get(attachment.type)
        if descriptor is not None:
            supplied += descriptor.supplies.power_kw * attachment.quantity

    for mode in operating_modes(platform, catalogue):
        drawn = 0.0
        for attachment in platform.attachments:
            descriptor = catalogue.get(attachment.type)
            if descriptor is None:
                continue
            drawn += descriptor.consumes.power_kw.get(mode, 0.0) * attachment.quantity
        if drawn > supplied:
            out.append(Finding(
                "power",
                f"in {mode!r} the platform draws {drawn:.1f} kW against "
                f"{supplied:.1f} kW supplied",
            ))
    return out


def check_load(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Every load check, gathered. See the module docstring for what is not
    included and why."""
    return check_stations(platform, catalogue) + check_power_budget(
        platform, catalogue
    )
