# 0002. Python as the primary language

Status: accepted
Date: 2026-08-09
## Context

The value of the environment depends entirely on people plugging components
into it. The intended contributors are students and researchers in control,
estimation, and autonomy, not software engineers.

Performance was considered and found not to bind: fidelity is low, dynamics are
planar, and Monte Carlo parallelises across independent replications, which
sidesteps the GIL entirely.

## Decision

Python is the implementation language for components and for the simulation
core. Numerical work uses numpy and scipy.

## Consequences

The contributor pool is as wide as it can be. Hot inner loops, if any appear,
are a numba or extension-module problem confined to one component rather than an
architectural problem.

A contributor wishing to supply a component in another language has no route in
at present. See ADR 0003 for what was rejected, and note that FMI import at the
equipment layer remains the intended escape hatch if this becomes pressing.
