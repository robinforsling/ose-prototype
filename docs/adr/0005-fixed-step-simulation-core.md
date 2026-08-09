# 0005. Fixed-step simulation core with rate groups

Status: accepted
Date: 2026-08-09
## Context

The core may step components at a fixed rate, or advance by discrete events, or
use adaptive integration. Monte Carlo campaigns of thousands of runs must be
exactly reproducible, and any run must be reproducible in isolation without
replaying its campaign.

## Decision

The core steps at a fixed rate, with named rate groups (fast, medium, slow) that
components declare membership of. Adaptive solvers are permitted only for offline
validation, never in the core.

Random seeds are derived hierarchically as a pure function of campaign seed,
sweep cell index, and replication index; each component then derives its own
stream from the run seed and its component identifier.

## Consequences

Runs are bit-reproducible and any single run can be reproduced alone. Adding a
component to a platform does not perturb the random streams of the others.
Debugging and visualisation are straightforward.

The cost is efficiency, and some awkwardness for genuinely event-driven
phenomena such as weapon fuzing, which must be handled within a step. Adaptive
step sequences depend on the trajectory, which is precisely why they are
excluded: determinism would not survive them.
