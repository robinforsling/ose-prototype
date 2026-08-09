# 0004. Components publish continuous-time dynamics

Status: accepted
Date: 2026-08-09
## Context

Every consumer of a dynamics model needs it discretised, but they do not need
the same discretisation. The simulation core wants fixed-step accuracy. A
planner's digital twin wants speed. A filter wants a linearised discrete model.

## Decision

A component with continuous dynamics publishes the derivative function
`f(x, u, theta, eta, w)` and nothing else. It never publishes `x_{k+1}`. No
discretisation, integrator, or time step appears inside the component.
Integrators are external and pluggable, chosen by whoever is stepping the model.

The derivative function must be pure: no hidden state, no internal random number
generation, no logging, no dependence on wall-clock time.

## Consequences

Any integrator can consume the model. Exact ZOH-linearised discrete models
follow from numerical or automatic differentiation of the same function, with no
model-specific code. `demos/integration_demo.py` demonstrates three consumers of
one untouched model.

Purity is load-bearing rather than stylistic. Violating it breaks adaptive
solvers, Jacobians, parallel Monte Carlo, and reproducibility simultaneously.

Discretisation is free; model *reduction* is not. A planner wanting a Dubins-style
abstraction must declare that approximation explicitly, parameterised by the
component's capability model.
