# Architecture decision records

One file per decision, numbered, immutable once accepted. Superseding or
extending a decision means writing a new record that references the old one,
never rewriting it.

The one edit an accepted record may receive is a forward pointer in its status
line, so that nobody reads a superseded or extended decision as current. The
reasoning below it stays as written: an ADR is a dated record of why a choice
was made, not a description of how things are now. For current state, read
`docs/10-concepts.md`, `docs/20-architecture.md` and `docs/interfaces/`.

| # | Decision | Status |
|---|---|---|
| [0001](0001-record-architecture-decisions.md) | Record architecture decisions | accepted |
| [0002](0002-python-as-the-primary-language.md) | Python as the primary language | accepted |
| [0003](0003-reject-ros2-hla-and-defer-fmi.md) | Reject ROS 2 and HLA, defer FMI | accepted |
| [0004](0004-continuous-time-component-contract.md) | Components publish continuous-time dynamics | accepted |
| [0005](0005-fixed-step-simulation-core.md) | Fixed-step simulation core with rate groups | accepted |
| [0006](0006-constraint-enforcement-outside-the-vehicle.md) | Constraint enforcement lies outside the vehicle | accepted |
| [0007](0007-planar-two-dimensional-modelling.md) | Planar two-dimensional modelling | accepted |
| [0008](0008-truth-perception-boundary.md) | Only the resource layer may read ground truth | accepted |
| [0009](0009-navigation-split-across-layers.md) | Split navigation across the resource and subsystem layers | accepted |
| [0010](0010-platform-clock-dead-reckoning.md) | Platform clock, estimated by dead reckoning only | accepted |
| [0011](0011-vehicle-guidance-decides-not-enforces.md) | Vehicle guidance decides what to command, not what is admissible | accepted |
| [0012](0012-capability-is-a-tested-multi-channel-claim.md) | Capability is a tested, multi-channel claim | accepted, extended by 0013 |
| [0013](0013-guidance-capability-composes-two-layers.md) | Guidance capability composes the vehicle's and navigation's | accepted |

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
