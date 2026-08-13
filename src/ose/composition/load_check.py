"""
Composition-time load checks: can this platform, as specified, be built?

Checks 1 and 3 of docs/40-composition-spec.md section 6.1. Both are answerable
from the descriptors alone, before anything is constructed and long before the
clock starts.

Two questions about mass, not one
---------------------------------
A platform can be within every station's limit and over its total, by
spreading the load, or over one station's limit while comfortably under its
total. Both are checked: mass per station against that station's declared
mass_limit_kg, and the whole aircraft against the vehicle's maximum.

That maximum did not exist when these checks were first written -- lambda
carried mass_dry_kg and no upper bound, and the worked example gave a vehicle
an empty mass and a fuel load but no ceiling, while section 5 of the
specification said the envelope included "empty and maximum mass". The intent
was there and the declaration was not. It is now m_max in lambda, which made
this a modelling change before it could be a validator one.

Check 4, port satisfaction, is here too, since descriptors now carry ports and
name components that exist (ADR 0025).

What is NOT here, and why
-------------------------
Checks 5 to 7 -- the truth boundary, layer discipline and parameter bounds.

The first two are enforced already, at import time and by the architecture
generator (ADR 0008, ADR 0024), which is earlier and stricter than a
composition check could be: they are properties of the code rather than of a
platform. Restating them here would catch a descriptor that lied about its
layer, which is what tests/test_descriptor_catalogue.py is for.

Parameter bounds need the parameter schema, which is still not modelled.

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

    rule: str                       # "station" | "mass" | "power"
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


def check_mass_budget(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Check 2. Empty mass, plus fuel, plus attachment masses times quantity,
    within the vehicle's maximum.

    The whole-aircraft counterpart to the per-station limit checked above.
    Both are needed and neither implies the other: a platform can be within
    every station's limit and over its total, by spreading the load, or over
    a single station's limit while comfortably under its total.

    Checked at full fuel, which is the worst case and the only one that can
    be checked without knowing the mission. Mass falls monotonically
    thereafter, so a platform that is legal at take-off stays legal.
    """
    vehicle = _vehicle(platform, catalogue)
    if vehicle is None:
        return [Finding("mass",
                        f"vehicle type {platform.vehicle_type!r} is not in the catalogue")]

    ceiling = vehicle.supplies.max_mass_kg
    if ceiling <= 0.0:
        return [Finding(
            "mass",
            f"{vehicle.type} declares no maximum mass, so the mass budget "
            "cannot be checked",
        )]

    carried = sum(
        catalogue[a.type].consumes.mass_kg * a.quantity
        for a in platform.attachments
        if a.type in catalogue
    )
    total = platform.empty_mass_kg + platform.fuel_kg + carried
    if total > ceiling:
        return [Finding(
            "mass",
            f"platform masses {total:.0f} kg "
            f"({platform.empty_mass_kg:.0f} empty + {platform.fuel_kg:.0f} fuel "
            f"+ {carried:.0f} carried) against a maximum of {ceiling:.0f} kg",
        )]
    return []


def _major_version(interface: str) -> str:
    """`family.name.v2` -> `2`. Two components bind only if the names match
    and the major versions are equal."""
    return interface.rsplit(".v", 1)[-1]


def check_ports(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Check 4. Every required port is satisfied, exactly once.

    Two failures, and the second is the interesting one.

    A required port with no provider is a platform that cannot run: guidance
    with no navigation under it has nothing to steer on, and finding that out
    at composition time is the entire argument for descriptors.

    A required port with SEVERAL providers is a platform the binder cannot
    resolve. It is not obviously an error -- two components can legitimately
    publish the same interface -- but nothing says which one a consumer gets,
    and a binder that picked either would be choosing an architecture by
    accident. This is the shape of the defect that had InsGnssEstimator and
    NavigationManager both publishing vehicle.state.v1 (ADR 0021): it was
    visible in a rendered diagram, and it would have been visible here.

    Matching is on interface name and equal major version, per
    docs/interfaces/README.md. A port is checked against every component on
    the platform, including the one that declares it, because a component may
    legitimately consume what it also produces -- and excluding itself would
    hide a component that satisfies its own requirement, which is a cycle
    worth seeing rather than a composition worth allowing.
    """
    out: list[Finding] = []

    descriptors = []
    for type_name in platform.component_types():
        descriptor = catalogue.get(type_name)
        if descriptor is None:
            out.append(Finding("port", f"type {type_name!r} is not in the catalogue"))
            continue
        descriptors.append(descriptor)

    providers: dict[tuple[str, str], list[str]] = {}
    for descriptor in descriptors:
        for port in descriptor.provides:
            key = (port.interface, _major_version(port.interface))
            providers.setdefault(key, []).append(descriptor.type)

    for descriptor in descriptors:
        for port in descriptor.requires:
            key = (port.interface, _major_version(port.interface))
            matched = providers.get(key, [])
            if not matched:
                if port.optional:
                    continue
                out.append(Finding(
                    "port",
                    f"{descriptor.type} requires {port.interface} on port "
                    f"{port.name!r}, and nothing on this platform provides it",
                ))
            elif len(matched) > 1:
                out.append(Finding(
                    "port",
                    f"{descriptor.type} requires {port.interface} on port "
                    f"{port.name!r}, and {len(matched)} components provide it "
                    f"({', '.join(sorted(matched))}) -- the binder cannot "
                    "choose",
                ))
    return out


def check_load(
    platform: PlatformSpec, catalogue: Mapping[str, ComponentDescriptor]
) -> list[Finding]:
    """Every load check, gathered. See the module docstring for what is not
    included and why."""
    return (
        check_stations(platform, catalogue)
        + check_mass_budget(platform, catalogue)
        + check_power_budget(platform, catalogue)
        + check_ports(platform, catalogue)
    )
