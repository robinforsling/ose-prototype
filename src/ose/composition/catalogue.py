"""
Descriptors for the component types that actually exist.

A *type* is a component class paired with a configuration, not a class. `Imu`
with TACTICAL_GRADE is one type; the same class with a navigation-grade
configuration would be another, and they do not weigh the same -- which is why
mass belongs to the type. `implementation` names the class; the configuration
is named in the table below and will be a `parameters` block once the spec's
parameter schema is modelled.

Before this module there were no descriptors anywhere in src/. The only ones
were five fixtures in tests/test_load_check.py -- `sensor.radar.pulse_doppler`
and friends -- and none of them named a class that exists, so the
composition-time checks were exercised entirely against fiction. Those fixtures
stay: they are the specification's worked example, and a platform richer than
what is implemented is worth keeping a test of. See ADR 0025.

Authored, and checked
---------------------
Everything here is written by hand, because a descriptor has to be readable
without constructing anything -- that is what lets a binder validate a platform
before it is built, and what lets this be YAML one day.

Which makes it able to drift, so it is checked:
tests/test_descriptor_catalogue.py resolves every `implementation` and asserts
the declared layer and ports match what tools/generate_architecture_diagram.py
derives from the source. Get a port wrong here and the suite fails.

The numbers are fictional and plausible, per CLAUDE.md, with one exception: the
vehicle's `max_mass_kg` is the real `mass_max_kg` from its own constraints, and
the cross-check asserts it stays that way. A mass ceiling that disagreed with
the model it describes would be worse than no ceiling at all.

Two dependencies this cannot express
------------------------------------
`VehicleGuidance` binds `VehicleManager` by concrete class rather than through
a port, so it cannot appear in `requires` and port satisfaction cannot check
it. The architecture diagram shows the same gap, from the other side. It is the
argument for making that binding a port, and it is recorded rather than worked
around.

`FuelGauge` takes `mass_dry_kg: float` from the vehicle. A dependency carried
as a scalar is invisible here exactly as it is to the diagram.
"""

from __future__ import annotations

from ose.composition.descriptor import (
    ComponentDescriptor,
    Consumes,
    Port,
    Station,
    Supplies,
)

# ---------------------------------------------------------------------------
# Equipment layer -- the only layer with a physical part, and therefore the
# only one that declares mass, power or stations.
# ---------------------------------------------------------------------------

FIGHTER = ComponentDescriptor(
    type="vehicle.fighter.generic_2d",
    layer="equipment",
    category="vehicle",
    implementation="ose.equipment.vehicle.planar_point_mass:PlanarPointMass",
    requires=(Port("command", "vehicle.command.v1"),),
    supplies=Supplies(
        power_kw=40.0,
        # The real ceiling from FIGHTER_LIMITS.mass_max_kg, cross-checked.
        max_mass_kg=19500.0,
        stations=(
            Station("nose", "nose", mass_limit_kg=300.0),
            Station("nav_bay", "internal", mass_limit_kg=60.0),
            Station("wing_inner_left", "wing", mass_limit_kg=1200.0),
            Station("wing_inner_right", "wing", mass_limit_kg=1200.0),
        ),
    ),
)

BOOSTED_FIGHTER = ComponentDescriptor(
    type="vehicle.fighter.boosted_2d",
    layer="equipment",
    category="vehicle",
    implementation=(
        "ose.equipment.vehicle.planar_point_mass_with_booster:"
        "PlanarPointMassWithBooster"
    ),
    requires=(Port("command", "vehicle.command.v1"),),
    supplies=Supplies(
        power_kw=40.0,
        max_mass_kg=19500.0,
        stations=(
            Station("nose", "nose", mass_limit_kg=300.0),
            Station("nav_bay", "internal", mass_limit_kg=60.0),
            Station("wing_inner_left", "wing", mass_limit_kg=1200.0),
            Station("wing_inner_right", "wing", mass_limit_kg=1200.0),
        ),
    ),
)

IMU_TACTICAL = ComponentDescriptor(
    type="nav_sensor.imu.tactical",
    layer="equipment",
    category="nav_sensor",
    implementation="ose.equipment.imu:Imu",
    provides=(Port("measurement", "sensing.imu.v1"),),
    consumes=Consumes(
        mass_kg=14.0, power_kw={"cruise": 0.3}, station_type="internal"
    ),
)

GNSS = ComponentDescriptor(
    type="nav_sensor.gnss.standard",
    layer="equipment",
    category="nav_sensor",
    implementation="ose.equipment.gnss:GnssReceiver",
    provides=(Port("fix", "sensing.gnss.v1"),),
    consumes=Consumes(
        mass_kg=3.5, power_kw={"cruise": 0.05}, station_type="internal"
    ),
)

AIR_DATA = ComponentDescriptor(
    type="nav_sensor.airdata.standard",
    layer="equipment",
    category="nav_sensor",
    implementation="ose.equipment.air_data:AirDataSensor",
    provides=(Port("measurement", "sensing.airdata.v1"),),
    consumes=Consumes(
        mass_kg=2.0, power_kw={"cruise": 0.02}, station_type="internal"
    ),
)

CLOCK = ComponentDescriptor(
    type="nav_sensor.clock.standard",
    layer="equipment",
    category="nav_sensor",
    implementation="ose.equipment.clock:Clock",
    provides=(Port("reading", "sensing.clock.v1"),),
    consumes=Consumes(
        mass_kg=0.8, power_kw={"cruise": 0.01}, station_type="internal"
    ),
)

FUEL_GAUGE = ComponentDescriptor(
    type="sensor.fuel_gauge.standard",
    layer="equipment",
    category="sensor",
    implementation="ose.equipment.fuel_gauge:FuelGauge",
    provides=(Port("reading", "sensing.fuel.v1"),),
    # Its dependency on the vehicle arrives as mass_dry_kg: float and has no
    # port to declare. See the module docstring.
    consumes=Consumes(
        mass_kg=1.2, power_kw={"cruise": 0.01}, station_type="internal"
    ),
)

# ---------------------------------------------------------------------------
# Subsystem layer -- purely cyber. No mass, no power, no station: a descriptor
# that gave one of these a mass would be inventing hardware.
# ---------------------------------------------------------------------------

INS_GNSS = ComponentDescriptor(
    type="subsystem.navigation.ins_gnss",
    layer="subsystem",
    category="cyber",
    implementation="ose.subsystem.navigation_state_estimator:InsGnssEstimator",
    provides=(Port("estimate", "vehicle.state_source.v1"),),
    requires=(
        Port("imu", "sensing.imu.v1"),
        Port("gnss", "sensing.gnss.v1"),
        Port("airdata", "sensing.airdata.v1"),
    ),
)

NAVIGATION_MANAGER = ComponentDescriptor(
    type="subsystem.navigation.manager",
    layer="subsystem",
    category="cyber",
    implementation="ose.subsystem.navigation_manager:NavigationManager",
    provides=(
        Port("own_state", "vehicle.state.v1"),
        Port("time", "platform.time.v1"),
    ),
    requires=(
        Port("own_state_source", "vehicle.state_source.v1"),
        Port("time_source", "platform.time_source.v1"),
    ),
)

TIME_ESTIMATOR = ComponentDescriptor(
    type="subsystem.time.estimator",
    layer="subsystem",
    category="cyber",
    implementation="ose.subsystem.time_state_estimator:TimeEstimator",
    provides=(Port("estimate", "platform.time_source.v1"),),
    requires=(Port("clock", "sensing.clock.v1"),),
)

VEHICLE_MANAGER = ComponentDescriptor(
    type="subsystem.vehicle_system.manager",
    layer="subsystem",
    category="cyber",
    implementation="ose.subsystem.vehicle_manager:VehicleManager",
    provides=(Port("mass", "vehicle.mass.v1"),),
    requires=(
        Port("fuel", "sensing.fuel.v1"),
        Port("own_state", "vehicle.state.v1"),
    ),
)

VEHICLE_GUIDANCE = ComponentDescriptor(
    type="subsystem.vehicle_system.guidance",
    layer="subsystem",
    category="cyber",
    implementation="ose.subsystem.vehicle_guidance:VehicleGuidance",
    provides=(Port("command", "vehicle.command.v1"),),
    # Its binding to the vehicle manager is by concrete class, not a port, so
    # it is absent here and port satisfaction cannot check it. See the module
    # docstring.
    requires=(
        Port("setpoint", "guidance.setpoint.v1"),
        Port("own_state", "vehicle.state.v1"),
    ),
)

# ---------------------------------------------------------------------------
# Single-ship layer
# ---------------------------------------------------------------------------

WAYPOINT_PLANNER = ComponentDescriptor(
    type="single_ship.planner.waypoint",
    layer="single_ship",
    category="cyber",
    implementation="ose.single_ship.action_planner:WaypointPlanner",
    provides=(
        Port("action", "planning.action.v1"),
        Port("motion", "guidance.setpoint.v1"),
    ),
    requires=(Port("own_state", "vehicle.state.v1"),),
)


CATALOGUE: dict[str, ComponentDescriptor] = {
    d.type: d
    for d in (
        FIGHTER,
        BOOSTED_FIGHTER,
        IMU_TACTICAL,
        GNSS,
        AIR_DATA,
        CLOCK,
        FUEL_GAUGE,
        INS_GNSS,
        NAVIGATION_MANAGER,
        TIME_ESTIMATOR,
        VEHICLE_MANAGER,
        VEHICLE_GUIDANCE,
        WAYPOINT_PLANNER,
    )
}
