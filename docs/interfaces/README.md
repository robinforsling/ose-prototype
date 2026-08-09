# Interface catalogue

Status: draft

Ports are typed by interface, named `family.name.vN`. Two components bind only
if the interface names match and the major versions are equal.

Adding a field to a published record is backward compatible. Removing or
renaming one is not, and requires a version increment.

## Catalogue

| Interface | Direction | Carries | Status |
|---|---|---|---|
| `truth.query.v1` | core to resource | Privileged read of ground-truth state. Grantable only to `layer: resource`. | planned |
| `power.bus.v1` | vehicle to resource | Abstract power draw negotiation, electrical and cooling combined. | planned |
| `vehicle.command.v1` | subsystem to resource | Commanded thrust and turn rate. | **implemented** |
| `vehicle.state.v1` | resource to subsystem | Own-ship state as the platform believes it, with covariance. | **implemented** |
| `sensing.detections.v1` | resource to subsystem | Time-stamped detections with measurement uncertainty. | planned |
| `sensing.control.v1` | subsystem to resource | Sensor tasking: pointing, mode, priority. | planned |
| `comms.message.v1` | bidirectional | Addressed transport with loss and latency applied. | planned |
| `effect.request.v1` | subsystem to resource | Employment request against a designated track. | planned |
| `effect.status.v1` | resource to subsystem | Inventory, readiness, in-flight effector state. | planned |
| `tracking.tracks.v1` | subsystem to single-ship | Fused track picture. | planned |
| `sa.picture.v1` | within single-ship | Assessed situation, threat evaluation. | planned |
| `planning.action.v1` | single-ship to subsystem | Committed actions for execution. | planned |
| `coord.intent.v1` | multi-ship to single-ship | Assigned role, tasking, constraints. | planned |

## Implemented interfaces

### `vehicle.command.v1`

`VehicleCommand(thrust_N, omega_rad_s)`. Thrust in newtons, turn rate in radians
per second, positive right.

The vehicle declares admissible sets but does not enforce them. A command
outside `U(x, lambda)` is integrated as given. See ADR 0006.

### `vehicle.state.v1`

`OwnStateEstimate`, published by any component satisfying the
`NavigationSystem` protocol. Carries position, heading, airspeed, ground
velocity, wind estimate, a 4x4 covariance over `[p_x, p_y, psi, v_air]`, and a
GNSS availability flag.

The covariance is part of the contract, not an optional extra. A consumer that
ignores it is choosing to, and a producer that supplies an inconsistent one
corrupts everything downstream. `tests/test_navigation.py` checks consistency by
NEES for exactly this reason.

## What every interface file must state

For each new interface, record: what it carries, at what rate, in what frame and
units, and what either side may assume about the other. Frames and units are the
classic integration killer; time semantics is the classic modularity killer.
