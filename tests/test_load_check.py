"""Tests for the composition-time load checks.

These check a *rule*, not a model, so there is no uncertainty to be honest
about and no dynamics to integrate. What they have to get right instead is
that every failure the specification names is actually detected, and that a
platform which should be buildable is not rejected.

The two that matter most are the ones a naive implementation gets wrong:

test_supply_is_summed_across_the_platform -- reading the vehicle's figure
alone would under-count, which is what makes a generator a component rather
than a vehicle parameter (ADR 0016 changed the specification for this).

test_a_station_is_checked_against_its_total_load -- two attachments can share
a station, so a per-entry check passes while the station is overloaded.
"""

# Default category for this file; a test that differs carries its own
# marker, which wins. See tests/conftest.py.
TEST_KIND = "unit"

import dataclasses

import pytest

from ose.composition import (
    Attachment,
    ComponentDescriptor,
    Consumes,
    Finding,
    PlatformSpec,
    Station,
    Supplies,
    check_load,
    check_mass_budget,
    check_power_budget,
    check_stations,
)
from ose.composition.catalogue import CATALOGUE as REAL_CATALOGUE
from ose.composition.descriptor import Port
from ose.composition.load_check import check_ports, operating_modes

FIGHTER = ComponentDescriptor(
    type="vehicle.fighter.generic_2d",
    layer="equipment",
    category="vehicle",
    supplies=Supplies(
        power_kw=40.0,
        max_mass_kg=19500.0,
        stations=(
            Station("nose", "nose", mass_limit_kg=300.0),
            Station("wing_inner_left", "wing", mass_limit_kg=1200.0),
            Station("nav_bay", "internal", mass_limit_kg=60.0),
        ),
    ),
)

RADAR = ComponentDescriptor(
    type="sensor.radar.pulse_doppler",
    layer="equipment",
    category="sensor",
    consumes=Consumes(
        mass_kg=180.0,
        power_kw={"cruise": 12.0, "combat": 28.0},
        station_type="nose",
    ),
)

MISSILE = ComponentDescriptor(
    type="effector.missile.bvr_generic",
    layer="equipment",
    category="effector",
    consumes=Consumes(mass_kg=170.0, station_type="wing"),
)

INS = ComponentDescriptor(
    type="nav_sensor.ins_gnss",
    layer="equipment",
    category="nav_sensor",
    consumes=Consumes(mass_kg=14.0, power_kw={"cruise": 0.3}, station_type="internal"),
)

GENERATOR = ComponentDescriptor(
    type="power.generator.turbine_driven",
    layer="equipment",
    category="vehicle",
    consumes=Consumes(mass_kg=45.0, station_type="internal"),
    supplies=Supplies(power_kw=60.0),
)

CATALOGUE = {
    d.type: d for d in (FIGHTER, RADAR, MISSILE, INS, GENERATOR)
}


EMPTY_KG = 12000.0
FUEL_KG = 4000.0


def _platform(*attachments: Attachment, **overrides) -> PlatformSpec:
    return PlatformSpec(
        "blue_01", FIGHTER.type, attachments,
        empty_mass_kg=overrides.get("empty_mass_kg", EMPTY_KG),
        fuel_kg=overrides.get("fuel_kg", FUEL_KG),
    )


# --------------------------------------------------------------------------
# A platform that should build
# --------------------------------------------------------------------------

def test_a_sound_platform_produces_no_findings():
    """The check has to be able to say yes, or it is useless."""
    platform = _platform(
        Attachment("nose", RADAR.type),
        Attachment("wing_inner_left", MISSILE.type, quantity=2),
        Attachment("nav_bay", INS.type),
    )
    assert check_load(platform, CATALOGUE) == []


# --------------------------------------------------------------------------
# Stations
# --------------------------------------------------------------------------

def test_a_station_the_vehicle_does_not_have_is_rejected():
    platform = _platform(Attachment("tail_cone", RADAR.type))
    findings = check_stations(platform, CATALOGUE)
    assert len(findings) == 1
    assert "tail_cone" in findings[0].message
    assert findings[0].rule == "station"


def test_a_station_of_the_wrong_type_is_rejected():
    """The radar wants a nose station; nav_bay exists but is internal."""
    platform = _platform(Attachment("nav_bay", RADAR.type))
    messages = [f.message for f in check_stations(platform, CATALOGUE)]
    assert any("needs a 'nose' station" in m for m in messages)


def test_a_station_is_checked_against_its_total_load():
    """Two attachments can share a station, and each can be legal while the
    pair is not. Two entries of four missiles are 680 kg apiece against a
    1200 kg pylon -- fine separately, 1360 kg together. A check that tested
    each entry against the limit would pass this."""
    platform = _platform(
        Attachment("wing_inner_left", MISSILE.type, quantity=4),
        Attachment("wing_inner_left", MISSILE.type, quantity=4),
    )
    findings = check_stations(platform, CATALOGUE)

    assert len(findings) == 1
    assert "1360" in findings[0].message and "1200" in findings[0].message


def test_quantity_multiplies_the_mass():
    """Per the specification's wording: attachment masses times quantity."""
    seven = _platform(Attachment("wing_inner_left", MISSILE.type, quantity=7))
    eight = _platform(Attachment("wing_inner_left", MISSILE.type, quantity=8))

    assert check_stations(seven, CATALOGUE) == [], "7 x 170 = 1190 kg, under 1200"
    assert check_stations(eight, CATALOGUE) != [], "8 x 170 = 1360 kg, over 1200"


def test_an_unknown_component_is_a_finding_not_a_crash():
    """A specification naming a type nobody has is a load error like any
    other, and reporting it beats raising: one run should surface everything
    wrong with a platform."""
    platform = _platform(Attachment("nose", "sensor.radar.nonexistent"))
    findings = check_load(platform, CATALOGUE)
    assert any("not in the catalogue" in f.message for f in findings)


# --------------------------------------------------------------------------
# The whole-aircraft mass budget
# --------------------------------------------------------------------------

def test_the_mass_budget_catches_what_station_limits_do_not():
    """The two mass checks are independent and neither implies the other.

    Three missiles per pylon is 510 kg against a 1200 kg limit, so every
    station is happy -- and 12 t empty plus 4 t of fuel plus 2 040 kg of
    stores is 18 040 kg, still under 19 500. Spread the same load wider and
    the stations stay happy while the aircraft does not.
    """
    within = _platform(
        Attachment("wing_inner_left", MISSILE.type, quantity=3),
        Attachment("wing_inner_left", MISSILE.type, quantity=3),
        Attachment("wing_inner_left", MISSILE.type, quantity=3),
        Attachment("wing_inner_left", MISSILE.type, quantity=3),
    )
    # 12 x 170 = 2040 kg on one pylon: over its 1200 kg limit, under the total.
    assert [f.rule for f in check_stations(within, CATALOGUE)] == ["station"]
    assert check_mass_budget(within, CATALOGUE) == []

    # And the converse: light enough per station, too heavy in total.
    heavy = _platform(
        Attachment("wing_inner_left", MISSILE.type, quantity=6),
        fuel_kg=6800.0,
    )
    assert check_stations(heavy, CATALOGUE) == [], "1020 kg is under the 1200 limit"
    findings = check_mass_budget(heavy, CATALOGUE)
    assert len(findings) == 1 and findings[0].rule == "mass"
    assert "19500" in findings[0].message


def test_the_mass_budget_names_its_terms():
    """A finding a contributor can act on says which part is too heavy."""
    heavy = _platform(Attachment("nose", RADAR.type), fuel_kg=7500.0)
    message = check_mass_budget(heavy, CATALOGUE)[0].message
    for term in ("12000", "7500", "180", "19500"):
        assert term in message


def test_an_undeclared_maximum_is_reported_not_skipped():
    """A vehicle with no ceiling cannot be checked, and saying so is the only
    honest answer. Returning no findings would read as 'this platform is
    fine', which is exactly the silent pass this check exists to prevent --
    and it is the state every vehicle was in before m_max was added to
    lambda."""
    unlimited = ComponentDescriptor(
        type="vehicle.fighter.unlimited", layer="equipment", category="vehicle",
        supplies=Supplies(power_kw=40.0, stations=FIGHTER.supplies.stations),
    )
    catalogue = {**CATALOGUE, unlimited.type: unlimited}
    platform = PlatformSpec("blue_01", unlimited.type, (), 12000.0, 4000.0)

    findings = check_mass_budget(platform, catalogue)
    assert len(findings) == 1
    assert "no maximum mass" in findings[0].message


def test_the_budget_is_checked_at_full_fuel():
    """The worst case, and the only one checkable without knowing the mission.
    Mass falls monotonically afterwards, so a platform legal at take-off stays
    legal."""
    at_limit = _platform(Attachment("nose", RADAR.type), fuel_kg=7300.0)
    over = _platform(Attachment("nose", RADAR.type), fuel_kg=7400.0)

    assert check_mass_budget(at_limit, CATALOGUE) == []   # 19480 kg
    assert check_mass_budget(over, CATALOGUE) != []       # 19580 kg


# --------------------------------------------------------------------------
# Power
# --------------------------------------------------------------------------

def test_power_is_checked_in_every_mode_not_just_cruise():
    """The radar fits the budget in cruise at 12 kW and does not in combat at
    28 kW plus the rest. A check that looked only at cruise would pass."""
    platform = _platform(
        Attachment("nose", RADAR.type), Attachment("nav_bay", INS.type)
    )
    # 40 kW supplied; cruise draws 12.3, combat draws 28.
    assert check_power_budget(platform, CATALOGUE) == []

    hungry = ComponentDescriptor(
        type="sensor.radar.hungry", layer="equipment", category="sensor",
        consumes=Consumes(mass_kg=180.0, power_kw={"cruise": 12.0, "combat": 55.0},
                          station_type="nose"),
    )
    catalogue = {**CATALOGUE, hungry.type: hungry}
    platform = _platform(Attachment("nose", hungry.type))

    findings = check_power_budget(platform, catalogue)
    assert len(findings) == 1
    assert "'combat'" in findings[0].message
    assert findings[0].rule == "power"


def test_supply_is_summed_across_the_platform():
    """A generator is a component, not a vehicle parameter (ADR 0016). Reading
    the vehicle's figure alone would reject a platform that carries its own
    supply."""
    hungry = ComponentDescriptor(
        type="sensor.radar.hungry", layer="equipment", category="sensor",
        consumes=Consumes(mass_kg=180.0, power_kw={"combat": 80.0},
                          station_type="nose"),
    )
    catalogue = {**CATALOGUE, hungry.type: hungry}

    without = _platform(Attachment("nose", hungry.type))
    assert check_power_budget(without, catalogue) != [], "40 kW cannot feed 80 kW"

    # The same radar, with a 60 kW generator aboard: 100 kW supplied.
    with_gen = _platform(
        Attachment("nose", hungry.type), Attachment("nav_bay", GENERATOR.type)
    )
    assert check_power_budget(with_gen, catalogue) == []


def test_a_component_silent_about_a_mode_draws_nothing_in_it():
    """A descriptor should not have to enumerate every mode the platform has
    just to say it is idle in most of them."""
    platform = _platform(
        Attachment("nose", RADAR.type),      # has a 'combat' entry
        Attachment("nav_bay", INS.type),     # does not
    )
    assert "combat" in operating_modes(platform, CATALOGUE)
    assert check_power_budget(platform, CATALOGUE) == []


def test_cruise_is_always_checked():
    """A platform whose attachments are all silent about modes still has to
    fly. Discovering modes purely from the descriptors would give an empty set
    and a check that passes by having nothing to look at."""
    quiet = ComponentDescriptor(
        type="effector.dumb", layer="equipment", category="effector",
        consumes=Consumes(mass_kg=10.0, station_type="wing"),
    )
    catalogue = {**CATALOGUE, quiet.type: quiet}
    platform = PlatformSpec("blue_01", FIGHTER.type,
                            (Attachment("wing_inner_left", quiet.type),))

    assert operating_modes(platform, catalogue) == ["cruise"]


def test_power_quantity_multiplies_too():
    """Four radars draw four radars' worth."""
    platform = _platform(Attachment("nose", RADAR.type, quantity=4))
    findings = check_power_budget(platform, CATALOGUE)
    assert any("'combat'" in f.message for f in findings)   # 4 x 28 = 112 kW


# --------------------------------------------------------------------------
# The shape of the answer
# --------------------------------------------------------------------------

def test_findings_accumulate_rather_than_stopping_at_the_first():
    """One run should say everything that is wrong. A validator that raises on
    the first problem makes fixing a platform an iterative guessing game."""
    platform = _platform(
        Attachment("tail_cone", RADAR.type),                      # no such station
        Attachment("wing_inner_left", MISSILE.type, quantity=9),  # overloaded
    )
    findings = check_load(platform, CATALOGUE)
    assert len(findings) >= 2
    assert {f.rule for f in findings} == {"station"}, (
        "a station problem should not produce mass or power findings"
    )


def test_check_load_runs_every_check():
    """A platform failing on two different rules should hear about both."""
    hungry = ComponentDescriptor(
        type="sensor.radar.hungry", layer="equipment", category="sensor",
        consumes=Consumes(mass_kg=400.0, power_kw={"cruise": 90.0},
                          station_type="nose"),
    )
    catalogue = {**CATALOGUE, hungry.type: hungry}
    platform = _platform(Attachment("nose", hungry.type))

    rules = {f.rule for f in check_load(platform, catalogue)}
    assert rules == {"station", "power"}, "one rule masked the other"


# --------------------------------------------------------------------------
# Check 4 -- port satisfaction
#
# Over the REAL catalogue, not the worked example above. The fixtures at the
# top of this file are the specification's illustration and name no class that
# exists; these check the platform the demos actually fly. See ADR 0025.
# --------------------------------------------------------------------------

def _blue_platform(**overrides) -> PlatformSpec:
    """The aircraft demo_live_route.py and demo_navigation.py fly between them:
    a fighter, the navigation sensors, the navigation and vehicle subsystems,
    and a planner."""
    spec = dict(
        id="blue_01",
        vehicle_type="vehicle.fighter.generic_2d",
        attachments=(
            Attachment("nav_bay", "nav_sensor.imu.tactical"),
            Attachment("nav_bay", "nav_sensor.gnss.standard"),
            Attachment("nav_bay", "nav_sensor.airdata.standard"),
            Attachment("nav_bay", "nav_sensor.clock.standard"),
            Attachment("nav_bay", "sensor.fuel_gauge.standard"),
        ),
        empty_mass_kg=12000.0,
        fuel_kg=4000.0,
        subsystems=(
            "subsystem.navigation.ins_gnss",
            "subsystem.navigation.manager",
            "subsystem.time.estimator",
            "subsystem.vehicle_system.manager",
            "subsystem.vehicle_system.guidance",
        ),
        single_ship=("single_ship.planner.waypoint",),
    )
    spec.update(overrides)
    return PlatformSpec(**spec)


def test_a_real_platform_passes_every_check():
    """The one that matters.

    Before the catalogue existed, every check in this file ran against
    components that do not exist. This is a platform the repository can
    actually fly, validated before anything is constructed.
    """
    findings = check_load(_blue_platform(), REAL_CATALOGUE)
    assert findings == [], "\n".join(str(f) for f in findings)


def test_a_missing_provider_is_reported_for_every_consumer():
    """Remove navigation and three components lose their own-state port.

    All three are reported, not the first: a platform is fixed by seeing
    everything wrong with it at once.
    """
    crippled = _blue_platform(subsystems=(
        "subsystem.navigation.ins_gnss",
        "subsystem.time.estimator",
        "subsystem.vehicle_system.manager",
        "subsystem.vehicle_system.guidance",
    ))
    findings = check_ports(crippled, REAL_CATALOGUE)

    assert all(f.rule == "port" for f in findings)
    assert all("vehicle.state.v1" in f.message for f in findings)
    complaining = {f.message.split()[0] for f in findings}
    assert complaining == {
        "subsystem.vehicle_system.manager",
        "subsystem.vehicle_system.guidance",
        "single_ship.planner.waypoint",
    }, complaining


def test_two_providers_of_one_interface_are_reported():
    """The check this exists for.

    An interface with two providers is not obviously wrong -- but nothing says
    which one a consumer binds, so a binder would be choosing an architecture
    by accident. This is the shape of the defect that had InsGnssEstimator and
    NavigationManager both publishing vehicle.state.v1, which was visible in a
    rendered diagram and would have been visible here (ADR 0021).
    """
    rogue = dataclasses.replace(
        REAL_CATALOGUE["subsystem.navigation.ins_gnss"],
        type="subsystem.navigation.rogue",
        provides=(Port("estimate", "vehicle.state.v1"),),
    )
    catalogue = {**REAL_CATALOGUE, rogue.type: rogue}
    platform = _blue_platform(subsystems=(
        "subsystem.navigation.ins_gnss",
        "subsystem.navigation.manager",
        "subsystem.navigation.rogue",
        "subsystem.time.estimator",
        "subsystem.vehicle_system.manager",
        "subsystem.vehicle_system.guidance",
    ))
    findings = check_ports(platform, catalogue)

    assert findings, "two publishers of one interface went unreported"
    assert all("cannot choose" in f.message for f in findings)
    assert all("vehicle.state.v1" in f.message for f in findings)


def test_an_optional_port_may_go_unsatisfied():
    """The difference between a component that cannot run and one that runs
    degraded. Nothing is optional in the real catalogue, so this is checked
    against a constructed descriptor rather than left untested."""
    hopeful = ComponentDescriptor(
        type="subsystem.hopeful", layer="subsystem", category="cyber",
        requires=(
            Port("nice_to_have", "tracking.tracks.v1", optional=True),
            Port("essential", "sa.picture.v1"),
        ),
    )
    catalogue = {**REAL_CATALOGUE, hopeful.type: hopeful}
    # Added to the whole platform rather than replacing it: dropping the other
    # subsystems would strip navigation and produce findings about them
    # instead, which is a different test.
    platform = _blue_platform(
        subsystems=_blue_platform().subsystems + (hopeful.type,)
    )

    mine = [f for f in check_ports(platform, catalogue)
            if f.message.startswith(hopeful.type)]
    assert len(mine) == 1, [str(f) for f in mine]
    assert "sa.picture.v1" in mine[0].message, "the essential port went unreported"
    assert not any("tracking.tracks.v1" in f.message for f in mine), (
        "an optional port with no provider was reported"
    )


def test_a_type_not_in_the_catalogue_is_reported():
    platform = _blue_platform(single_ship=("single_ship.planner.imaginary",))
    findings = check_ports(platform, REAL_CATALOGUE)
    assert any("not in the catalogue" in f.message for f in findings)
