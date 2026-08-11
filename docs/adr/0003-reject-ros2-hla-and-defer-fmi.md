# 0003. Reject ROS 2 and HLA, defer FMI

Status: accepted
Date: 2026-08-09
## Context

Three established standards address parts of this problem: ROS 2 for
component messaging, HLA (IEEE 1516) for simulation federation, and FMI/FMU for
exchangeable model components.

## Decision

ROS 2 and HLA are rejected. FMI is deferred, and if adopted will be an optional
adapter at the equipment layer rather than the integration backbone.

## Consequences

ROS 2 and HLA each impose an ontology and an operational burden disproportionate
to a low-fidelity teaching and research environment, and both introduce
asynchrony that is hostile to reproducible Monte Carlo.

FMI is an excellent fit for continuous numerically-integrated components such as
vehicle dynamics, and a poor fit for the discrete, event-driven, stateful cyber
layers above. Wrapping an action planner as an FMU is a known source of pain.
Adopting FMI as the backbone would therefore impose co-simulation semantics on
components that do not need them.

The cost is that a capability descriptor plus a typed port interface must be
maintained ourselves rather than inherited. This is accepted as the smaller
burden.
