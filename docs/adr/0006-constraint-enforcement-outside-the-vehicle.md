# 0006. Constraint enforcement lies outside the vehicle

Status: accepted
Date: 2026-08-09
## Context

The vehicle model declares admissible sets for state and input. Something must
decide what happens when a command violates them.

## Decision

The vehicle declares the sets and does not enforce them. It performs no
projection, saturation, or clamping. Responsibility lies with guidance, motion
planning, or an interposed runtime assurance layer.

Two utilities are offered for that layer to call explicitly, `project_command`
and `state_violations`, but nothing in the integration path invokes them.

## Consequences

A control law that persistently commands outside the envelope produces a visible
finding rather than a silently clipped command. Different enforcement strategies
can be compared against the same vehicle. The enforcement policy is an explicit
design decision rather than an unstated property of the vehicle.

The cost is that a careless controller can drive the model outside its declared
region of validity and receive physically meaningless results. This is accepted:
the alternative conceals control design faults.
