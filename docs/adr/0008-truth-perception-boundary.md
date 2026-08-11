# 0008. Only the equipment layer may read ground truth

Status: accepted
Date: 2026-08-09
## Context

The simulation core owns true world state. In a combat simulation the entire
research value lies in components acting on imperfect information derived from
sensors.

## Decision

Only components declaring `layer: equipment` may hold a port of type `truth.*`.
This is enforced by the descriptor validator at import time, not by convention.
Everything above the equipment layer consumes published estimates.

The rule extends to capability assessment. The capability model is a function of
the disturbance, which is a property of the environment; evaluating it with true
disturbance yields true capability, available only to the simulation. Cyber-layer
components evaluate it with estimated disturbance and therefore obtain a slightly
wrong answer, which is correct behaviour rather than a defect.

## Consequences

Results mean what they appear to mean. Sensor, estimation, and fusion components
are genuinely exercised.

The rule cannot be relaxed for convenience anywhere, including in debugging aids
and visualisation, because a single leak invalidates every result produced after
it and is nearly impossible to detect retrospectively.
