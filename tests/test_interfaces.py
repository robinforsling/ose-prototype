"""The interface catalogue must stay honest about itself.

Interface names live as `INTERFACE` class variables on the records that carry
them (ADR 0020). Before that they lived in docstrings and in a hand-written
table, and drifted three ways with nothing able to notice: `vehicle.mass.v1`
had a prose section and no table row, `FuelMeasurement` had no name at all,
and docs/40-composition-spec.md carried a second copy of the table stating the
opposite direction for `vehicle.state.v1`.

Moving the names into code only helps if the code is checked, and the failures
worth checking are all quiet ones -- a name that never reaches the catalogue, a
ClassVar that turns into a dataclass field, a name inherited by a record that
never chose it. None of them raise on their own.
"""

from __future__ import annotations

import dataclasses
import re

import pytest

from ose import interfaces
from ose.interfaces import NOT_A_PORT, catalogue

# family.name.vN, per docs/interfaces/README.md. The version suffix is the
# part that matters: two components bind only if the names match and the major
# versions are equal, so a name without one cannot be bound against.
NAME_PATTERN = re.compile(r"^[a-z][a-z0-9]*\.[a-z0-9_]+\.v[0-9]+$")


def registered_records() -> list[type]:
    """Every record the catalogue knows, flattened.

    Walks the catalogue rather than naming records, so a record added later is
    covered without touching this file. The tests below then apply to all of
    them -- the "check the whole object" rule from CLAUDE.md aimed at the
    registry itself, since checking three representative records would pass
    while a fourth carried a field named INTERFACE.
    """
    return [record for records in catalogue().values() for record in records]


def test_the_catalogue_is_not_vacuous():
    """Every test here walks the catalogue, so all of them pass trivially if
    the walk returns nothing."""
    names = catalogue()
    assert len(names) >= 10, f"only {len(names)} interfaces found; the walk is wrong"
    assert len(registered_records()) >= len(names), "a name with no records"


def test_every_record_is_classified():
    """A record in ose.interfaces is either a port or declared not to be one.

    This is the pairing that makes a forgotten registration an error. A record
    with no INTERFACE is indistinguishable from one that deliberately has none
    -- SensorCapability, PromisedEnvelope and the two truth types all
    legitimately have none -- so absence alone proves nothing and the negative
    has to be declared too.
    """
    catalogue()   # raises TypeError naming anything unclassified

    # And the negative list is not a dumping ground: everything in it must
    # still exist and still be a record.
    for record in NOT_A_PORT:
        assert dataclasses.is_dataclass(record), f"{record} in NOT_A_PORT is not a record"


def test_an_unregistered_record_is_refused():
    """The sabotage for the test above.

    Without this, test_every_record_is_classified only asserts that currently
    clean code is clean -- it would pass just as happily against a catalogue()
    whose check had been deleted.
    """
    @dataclasses.dataclass
    class UnregisteredRecord:
        x: float

    interfaces.UnregisteredRecord = UnregisteredRecord
    try:
        with pytest.raises(TypeError, match="UnregisteredRecord"):
            catalogue()
    finally:
        del interfaces.UnregisteredRecord

    catalogue()   # and the module is left as it was found


def test_interface_is_a_classvar_not_a_field():
    """The trap this defends against is silent.

    `INTERFACE: str = "..."` without ClassVar becomes a real dataclass field.
    It raises at class-definition time only when a non-defaulted field follows
    it -- and INTERFACE is naturally written after the defaulted ones, where
    nothing raises at all. The record then gains a constructor argument, grows
    a field in dataclasses.asdict, and starts appearing in every test that
    walks its fields.
    """
    for record in registered_records():
        fields = {f.name for f in dataclasses.fields(record)}
        assert "INTERFACE" not in fields, (
            f"{record.__name__}.INTERFACE is a dataclass field, not a class "
            "variable -- it needs the ClassVar[str] annotation"
        )


def test_interface_is_declared_not_inherited():
    """A record subclass must choose its own name or have none.

    BoostCapability(Capability) is the live proof that record subclassing
    happens here. Were Capability ever registered, its subclass would silently
    inherit the name and the catalogue would report two records on an
    interface only one of them publishes.
    """
    for record in registered_records():
        assert "INTERFACE" in record.__dict__, (
            f"{record.__name__} inherits INTERFACE from a base class rather "
            "than declaring its own"
        )


def test_every_name_is_well_formed():
    for name in catalogue():
        assert NAME_PATTERN.match(name), (
            f"{name!r} is not family.name.vN; a name without a major version "
            "cannot be version-matched at bind time"
        )


def test_every_name_has_records_and_every_record_has_one_name():
    """Uniqueness is asserted on the record-to-name direction only.

    A name legitimately covers several records -- `guidance.setpoint.v1` has
    two forms, and a consumer binds the interface and dispatches on the form.
    The direction that must be single-valued is the other one, and the
    ClassVar gives it structurally: a record has exactly one INTERFACE.
    """
    seen: dict[type, str] = {}
    for name, records in catalogue().items():
        assert records, f"{name} maps to no records"
        for record in records:
            assert record not in seen, (
                f"{record.__name__} appears under both {seen[record]} and {name}"
            )
            seen[record] = name


def test_a_port_record_is_not_also_declared_not_a_port():
    """The two declarations are exclusive. A record in both would have its
    INTERFACE win silently, leaving a reason in NOT_A_PORT that describes
    something untrue."""
    overlap = {r.__name__ for r in registered_records()} & {
        r.__name__ for r in NOT_A_PORT
    }
    assert not overlap, f"records both registered and declared NOT_A_PORT: {overlap}"


def test_no_interface_is_both_implemented_and_planned():
    """The catalogue page has a generated half and a written half.

    The generated table cannot drift from the code. The planned table is
    hand-written and can: implementing an interface without deleting its
    planned row leaves the page listing it twice, once as working and once as
    not. That is the shape of drift the registry cannot fix on its own.
    """
    from pathlib import Path

    page = (Path(__file__).resolve().parents[1] / "docs" / "interfaces"
            / "README.md").read_text()
    planned = page.split("## Planned", 1)[1].split("## Implemented interfaces", 1)[0]
    listed = set(re.findall(r"`([a-z0-9_.]+\.v[0-9]+)`", planned))

    assert listed, "no planned interfaces parsed -- this comparison is vacuous"
    overlap = listed & set(catalogue())
    assert not overlap, (
        f"interface(s) listed as planned but implemented in code: {sorted(overlap)}. "
        "Delete the planned row; the generated table already covers them."
    )


def test_every_implemented_interface_is_written_up():
    """A generated row says an interface exists; the prose says what it means.

    A new interface reaching the table with no section beneath it is how a
    catalogue becomes a list of names. `sensing.fuel.v1` was exactly that case
    -- a record in the code with no name and no description anywhere.
    """
    from pathlib import Path

    page = (Path(__file__).resolve().parents[1] / "docs" / "interfaces"
            / "README.md").read_text()
    documented = set(re.findall(r"^### `([^`]+)`", page, re.M))

    assert documented, "no interface sections found -- this comparison is vacuous"
    missing = set(catalogue()) - documented
    assert not missing, (
        f"implemented interface(s) with no section in the catalogue page: "
        f"{sorted(missing)}"
    )


def test_every_vehicle_model_provides_the_vehicle_port():
    """Both models satisfy Vehicle, discovered rather than listed.

    The port exists because the annotations lied: Imu and VehicleManager both
    said PlanarPointMass while the code was duck-typed, and a manager has been
    built on the boosted model in the tests all along. A model that stopped
    providing the port would break a consumer that never named it.

    Checked structurally, member by member, because Vehicle cannot be
    runtime_checkable -- dry_mass_kg is a property, and issubclass against a
    protocol with a non-method member raises. isinstance would work but needs
    an instance, and a model needs parameters to build.
    """
    from _truth_boundary import vehicle_model_names
    from ose.interfaces import Vehicle

    members = [
        name for name in vars(Vehicle)
        if not name.startswith("_") and name not in {"mro"}
    ]
    assert members, "the Vehicle protocol declares nothing -- this test is vacuous"

    models = vehicle_model_names()
    assert models, "no vehicle models discovered -- this test is vacuous"

    import importlib
    import pkgutil
    from ose.topology import TRUTH_PACKAGE

    package = importlib.import_module(TRUTH_PACKAGE)
    found = {}
    for info in pkgutil.iter_modules(package.__path__):
        module = importlib.import_module(f"{package.__name__}.{info.name}")
        for name in models:
            cls = getattr(module, name, None)
            if cls is not None and cls.__module__ == module.__name__:
                found[name] = cls

    assert set(found) == set(models), (
        f"could not locate every model: missing {sorted(set(models) - set(found))}"
    )
    for name, cls in sorted(found.items()):
        missing = [m for m in members if not hasattr(cls, m)]
        assert not missing, f"{name} does not provide Vehicle members {missing}"


def test_truth_types_are_never_ports():
    """Invariant 1, asserted at the catalogue rather than at a component.

    Giving VehicleState an interface name would make it bindable, and a port
    is exactly the thing a cyber-layer component is allowed to hold. The truth
    boundary would then be crossed by configuration rather than by code.
    """
    from ose.topology import TRUTH_TYPES

    named = {r.__name__ for r in registered_records()}
    leaked = named & TRUTH_TYPES
    assert not leaked, f"truth-carrying types registered as ports: {leaked}"
