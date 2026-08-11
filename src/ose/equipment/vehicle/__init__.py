"""Vehicle models, one module per model.

A *model* is code: dynamics, the shape of its parameters, and the admissible
sets it declares. A *configuration* is data: values for one model, living in
equipment/reference_configs/vehicle/ under the same filename. Adding a vehicle
with new dynamics is a new module here; adding a variant of an existing one is
a record there, and needs no Python at all once the descriptor validator
exists.

    planar_point_mass.py                  the baseline: point mass, planar,
                                          single propulsion setting

Everything the model module defines is re-exported here, so
`from ose.equipment.vehicle import VehicleState` keeps working and consumers
that only need the shared records need not name a model.

WHICH RECORDS ARE SHARED IS NOT YET DECIDED, deliberately. Some are clearly
common to every planar model -- Disturbance and Saturation, and the model
document states the drag and disturbance models are unchanged under boost.
Others are not: a two-mode engine's parameter vector carries two fuel
coefficients and two thermal time constants, and its constraint vector carries
mode-dependent thrust and speed limits. Splitting them now would mean guessing,
and the guess would be load-bearing for VehicleManager, which must stay
model-agnostic. The split is settled by building the second model, not before
it -- the same reason layer packages here are created when they acquire their
first component rather than in anticipation.
"""

from ose.equipment.vehicle.planar_point_mass import (  # noqa: F401
    NO_DISTURBANCE,
    Capability,
    Constraints,
    Disturbance,
    PlanarPointMass,
    Saturation,
    VehicleCommand,
    VehicleGeometry,
    VehicleParameters,
    VehicleState,
)
