# Scope

Status: draft

## Purpose

An open simulation environment for research and teaching in autonomous combat
aircraft systems. It exists so that a student or researcher can test one
component, algorithm, or subsystem inside a complete, integrated system of
interest without first having to implement everything around it.

The emphasis is integration, not fidelity. Where a choice arises between a
faithful model and a simple one that preserves the integration problem, the
simple one is chosen.

## System of interest

A combat aircraft system, possibly multi-ship, operating in air combat:
defensive counter-air (DCA) and offensive counter-air (OCA).

Three affiliations may be present: blue, red, and yellow (neutral). Affiliation
is a scenario attribute, not an architectural one. Red platforms are composed
from the same layers and the same component types as blue.

## In scope

- Planar (2D) modelling of aerial platforms.
- Four-layer platform composition: equipment, subsystem, single-ship, multi-ship.
- Low-fidelity probabilistic models of sensing, communication, and effect.
- Deterministic, reproducible Monte Carlo campaigns.
- Isolated lab environments for developing components against stubs.

## Explicitly out of scope

- Physical wave propagation for sensors. Detection is probabilistic.
- High-fidelity aerodynamics. See ADR 0007.
- Altitude, terrain, and terrain masking.
- Six-degree-of-freedom dynamics, roll and pitch attitude.
- Real-time operation and hardware in the loop.
- Classified or export-controlled data of any kind. All parameter sets in this
  repository are fictional and plausible, and are not claims about any real
  system.

## Planned but not yet present

- Ground-based air defence systems as an additional platform type.
- Composition GUI, scenario builder, Monte Carlo runner.
- Sensor, communicator, and effector equipment components.
- A model of what actually happens when state or control constraints are
  violated -- departure, structural failure, engine limits -- rather than
  today's silence. `state_violations()` already detects; nothing yet models
  the consequence. Whatever it does physically, it must keep the violation
  visible: degraded-but-plausible dynamics would hide a bad control law,
  which is the failure mode ADR 0006 exists to prevent.

## Non-goals

The environment is not a training simulator, not an analysis tool for
operational decisions, and not a substitute for higher-fidelity study. Results
carry the limitations of a planar low-fidelity model and should be reported with
them.
