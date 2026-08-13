# Architecture decision records

One file per decision, numbered. A new decision means a new record that
references the ones it changes, never a renumbering.

Accepted records are revised when a statement in them stops being true, so that
reading one never misleads. A forward pointer goes in the status line as well,
so the relationship between records is visible. What is preserved is the
*reasoning*: the Context section says what the problem looked like at the time
and stays as written even when the decision it led to has moved on. Git holds
the full history, so nothing is lost by keeping the working copy accurate.

| # | Decision | Status |
|---|---|---|
| [0001](0001-record-architecture-decisions.md) | Record architecture decisions | accepted |
| [0002](0002-python-as-the-primary-language.md) | Python as the primary language | accepted |
| [0003](0003-reject-ros2-hla-and-defer-fmi.md) | Reject ROS 2 and HLA, defer FMI | accepted |
| [0004](0004-continuous-time-component-contract.md) | Components publish continuous-time dynamics | accepted |
| [0005](0005-fixed-step-simulation-core.md) | Fixed-step simulation core with rate groups | accepted |
| [0006](0006-constraint-enforcement-outside-the-vehicle.md) | Constraint enforcement lies outside the vehicle | accepted |
| [0007](0007-planar-two-dimensional-modelling.md) | Planar two-dimensional modelling | accepted |
| [0008](0008-truth-perception-boundary.md) | Only the equipment layer may read ground truth | accepted |
| [0009](0009-navigation-split-across-layers.md) | Split navigation across the equipment and subsystem layers | accepted, amended by 0019 |
| [0010](0010-platform-clock-dead-reckoning.md) | Platform clock, estimated by dead reckoning only | accepted |
| [0011](0011-vehicle-guidance-decides-not-enforces.md) | Vehicle guidance decides what to command, not what is admissible | accepted |
| [0012](0012-capability-is-a-tested-multi-channel-claim.md) | Capability is a tested, multi-channel claim | accepted, extended by 0013 |
| [0013](0013-guidance-capability-composes-two-layers.md) | Guidance capability composes the vehicle's and navigation's | accepted |
| [0014](0014-one-navigation-publisher-per-platform.md) | One navigation publisher per platform, and it does not fuse | accepted, amended by 0019, extended by 0022 |
| [0015](0015-the-vehicle-manager-owns-believed-mass.md) | The vehicle manager owns the platform's believed mass | accepted, extended by 0016 |
| [0016](0016-promised-envelope-carries-the-mass-margin.md) | Only the promised envelope carries the mass margin | accepted |
| [0017](0017-equipment-layer.md) | The resource layer is renamed the equipment layer | accepted |
| [0018](0018-unweighted-mathematical-notation.md) | Mathematical symbols are written plain, in every artefact | accepted |
| [0019](0019-remove-the-black-box-navigation-unit.md) | The black-box navigation unit is removed | accepted |
| [0020](0020-the-implemented-topology-is-derived-from-the-code.md) | The implemented topology is derived from the code | accepted, amended by 0021 |
| [0021](0021-an-interface-name-belongs-to-the-port.md) | An interface name belongs to the port, not the record | accepted |
| [0022](0022-the-navigation-manager-is-the-pnt-publisher.md) | The navigation manager is the platform's PNT publisher | accepted |
| [0023](0023-alternatives-behind-a-port-are-one-component.md) | Alternative implementations of a port are one component | accepted |
| [0024](0024-layer-discipline-is-enforced-not-documented.md) | Layer discipline is enforced, not documented | accepted |
| [0025](0025-descriptors-describe-components-that-exist.md) | Descriptors describe components that exist | accepted |
| [0026](0026-the-fuel-gauge-reports-mass-above-dry.md) | The fuel gauge reports mass above dry, and the manager reconciles it | accepted, amended by 0027 |
| [0027](0027-sensing-fuel-v2-renames-the-measured-quantity.md) | `sensing.fuel.v2` names the quantity actually measured | accepted |

## Template

```markdown
# NNNN. Title

Status: proposed | accepted | accepted, extended by ADR-XXXX | superseded by ADR-XXXX
Date: YYYY-MM-DD

## Context
What forces are at play. What makes this a real decision rather than an obvious one.

## Decision
What was decided, stated plainly.

## Consequences
What follows, including the consequences we dislike. An ADR listing only
benefits is not finished.
```
