# 0007. Planar two-dimensional modelling

Status: accepted
Date: 2026-08-09
## Context

The environment exists to exercise integration, not to reproduce flight
mechanics. Three dimensions cost computation, complicate interpretation, and
make debugging harder.

Against this, altitude and specific energy are the tactical currency of air
combat: weapon envelopes, radar horizon, and energy manoeuvring all depend on
them.

## Decision

Modelling is planar. Altitude is suppressed. Airspeed carries the energy state.

## Consequences

Visualisation, interpretation, and debugging are markedly simpler, and the
integration questions the environment exists to study are unaffected.

Manoeuvres trading altitude for airspeed cannot be represented, and air density
is a fixed parameter describing a single flight level. Any conclusion about
energy tactics drawn from this environment is therefore suspect and should be
labelled as such.

To limit the damage, quantities that physically depend on launch energy -- weapon
envelopes in particular -- are modelled as functions of launch speed and geometry
rather than as fixed constants.
