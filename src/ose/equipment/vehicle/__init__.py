"""Vehicle models, one module per model.

A *model* is code: dynamics, the shape of its parameters, and the admissible
sets it declares. A *configuration* is data: values for one model, living in
equipment/reference_configs/vehicle/ under the same filename. Adding a vehicle
with new dynamics is a new module here; adding a variant of an existing one is
a record there, and needs no Python at all once the descriptor validator
exists.

    records.py                         shared by every planar model
    planar_point_mass.py               the baseline: one propulsion setting
    planar_point_mass_with_booster.py  two modes, nominal and boost

Everything is re-exported here, so `from ose.equipment.vehicle import
VehicleState` keeps working and a consumer that needs only the shared records
need not name a model.

What is shared and what is not was settled by building the second model rather
than guessed before it. Shared: the authored aerodynamics, the lumped
parameters and limits, the disturbance, the command, the capability record and
the saturation receipt. Not shared: a model's state vector, because a switched
model carries a thermal accumulator the baseline never writes, and a common
state record would put that field in the baseline's Jacobian and integrator
with nothing maintaining it.

Extended parameters and constraints COMPOSE the shared ones rather than
duplicating them. The model document's two-mode parameter vector begins with
exactly the baseline's four entries, and its constraint vector contains the
baseline's seven, so the drag polar keeps one definition.
"""

from ose.equipment.vehicle.planar_point_mass import (  # noqa: F401
    PlanarPointMass,
    VehicleState,
)
from ose.equipment.vehicle.planar_point_mass_with_booster import (  # noqa: F401
    BoostCapability,
    BoostConstraints,
    BoostParameters,
    BoostState,
    Mode,
    PlanarPointMassWithBooster,
)
from ose.equipment.vehicle.records import (  # noqa: F401
    NO_DISTURBANCE,
    Capability,
    Constraints,
    Disturbance,
    Saturation,
    VehicleCommand,
    VehicleGeometry,
    VehicleParameters,
)
