# Composition specification

Status: draft
Applies to: OSE-ACAS (open simulation environment, autonomous combat aircraft systems)

This document defines the declarative format that describes how a platform is
assembled from components, how scenarios are assembled from platforms, and how
Monte Carlo campaigns are generated from scenarios.

It is the single source of truth for the environment. The composition GUI is an
editor for these files, the scenario builder is a generator of them, the Monte
Carlo runner is a transform over them, and a lab is one of them with stubs
substituted for absent components. Nothing in the toolchain may hold state that
is not expressible here.

---

## 1. Two distinct artifacts

A recurring source of confusion in component-based simulators is mixing the
description of *what a component is* with the description of *which components a
scenario uses*. They are kept strictly separate.

**Capability descriptor** — ships with the component, authored once by whoever
wrote it, version-controlled alongside its source. It declares what the component
provides, what it requires, and what its performance envelope is. This is the
shape of the puzzle piece.

**Composition spec** — authored per scenario. It names component types, supplies
parameters, and states where things attach. This is which pieces are used and
where they go.

The registry indexes descriptors. The composition spec references them by
`type` and `version`. Validation is the act of checking a spec against the
descriptors it references.

---

## 2. Conventions

**Units.** SI base units throughout, with the unit encoded in the field name
(`max_speed_mps`, `mass_kg`, `range_m`). Angles are the exception: degrees in
authored files (`fov_deg`), radians internally. A field without a unit suffix is
dimensionless or a rate in Hz.

**Frame.** 2D world frame, x east, y north, metres. Heading is degrees clockwise
from north, aviation convention. This is deliberately not the mathematical
convention and it will bite someone at least once; it is chosen because every
tactical convention in the domain assumes it.

**Identifiers.** `snake_case`. Platform IDs unique per scenario, port names
unique per component, station names unique per vehicle.

**Versioning.** Interfaces use `name.vN`. A component declares the exact
interface versions it speaks. Two components bind only if their interface names
match and their major versions are equal.

**Nothing is implicit.** Every binding is either stated in the spec or derived by
a documented rule. There is no default wiring that "just works" — a spec that
does not fully determine the run is invalid.

---

## 3. Interface catalogue

Ports are typed by interface. The full catalogue lives in `30-interfaces/`; this
is the subset used in the worked example.

| Interface | Direction | Carries |
|---|---|---|
| `truth.query.v1` | core → equipment | Privileged read of ground-truth world state. Only equipment-layer components may hold this port. |
| `power.bus.v1` | vehicle → equipment | Abstract power draw negotiation (electrical plus cooling, combined). |
| `vehicle.command.v1` | subsystem → equipment | Commanded speed, turn rate, throttle. |
| `vehicle.state.v1` | equipment → subsystem | Own-ship state as the platform believes it to be, from navigation sensors. |
| `sensing.detections.v1` | equipment → subsystem | Time-stamped detections with measurement uncertainty. |
| `sensing.control.v1` | subsystem → equipment | Sensor tasking: pointing, mode, priority. |
| `comms.message.v1` | bidirectional | Addressed message transport with loss and latency applied. |
| `effect.request.v1` | subsystem → equipment | Employment request against a designated track. |
| `effect.status.v1` | equipment → subsystem | Inventory, readiness, in-flight effector state. |
| `tracking.tracks.v1` | subsystem → single-ship | Fused track picture. |
| `sa.picture.v1` | single-ship → single-ship | Assessed situation, threat evaluation. |
| `planning.action.v1` | single-ship → subsystem | Committed actions for execution. |
| `coord.intent.v1` | multi-ship → single-ship | Assigned role, tasking, constraints. |

The truth boundary is enforced by port type. `truth.query.v1` is only grantable
to components whose descriptor declares `layer: equipment`. The binder refuses it
to anything else. This is the one rule that must never be relaxed for
convenience.

---

## 4. Capability descriptor schema

```yaml
descriptor_version: 1

type: string            # unique type name, e.g. sensor.radar.pulse_doppler
version: string         # semver of this component implementation
layer: enum             # equipment | subsystem | single_ship | multi_ship
category: enum          # vehicle | sensor | nav_sensor | communicator | effector | cyber
summary: string
implementation: string  # importable Python path to the factory

# Physical resources this component consumes when mounted. Equipment layer only.
consumes:
  mass_kg: number
  power_kw:                      # per operating mode
    <mode_name>: number
  station:
    type: enum                   # nose | fuselage | wing | internal | conformal
    count: integer

# Physical resources this component makes available. Equipment layer only.
#
# The two are not owned by the same thing, which is why this block is not
# restricted as a whole. Stations are airframe structure, so only a vehicle
# can declare them. Power is made by whatever is built to make it, so any
# equipment component may: an engine-driven generator modelled inside the
# vehicle and a separate generator component then compose the same way, and
# neither is privileged.
supplies:
  power_kw: number               # any equipment component
  stations:                      # vehicles only
    - name: string
      type: enum
      mass_limit_kg: number

# Service ports.
provides:
  - port: string
    interface: string
requires:
  - port: string
    interface: string
    optional: boolean            # default false

# Self-assessed performance envelope. Structure varies by category; the binder
# treats it as opaque, but the component must be able to answer queries against
# it at composition time and at runtime.
envelope: {}

# Parameters the composition spec may set, with defaults and bounds.
parameters:
  <name>:
    type: enum                   # number | integer | string | boolean | enum
    default: any
    min: number
    max: number
    units: string

# Scheduling.
update_rate_hz: number
rate_group: enum                 # fast | medium | slow | event
```

### 4.1 Envelope conventions by category

The envelope is where self-assessment lives. It must be sufficient for a planner
to ask "can I do this?" without executing the component.

**Vehicle** — speed bounds, turn performance as a function of speed, acceleration
limits, fuel capacity and consumption, empty and maximum mass.

**Sensor** — field of view, field of regard, detection model parameters
(reference range against a reference target, probability of detection, false
alarm rate), measurement noise, revisit behaviour.

**Communicator** — range, capacity, latency distribution, loss model.

**Effector** — launch envelope as a function of own speed and engagement
geometry, kill probability model, time of flight, inventory.

---

## 5. Composition spec schema

```yaml
spec_version: 1

platform:
  id: string
  affiliation: enum              # blue | red | yellow
  name: string

  # Bottom-up. Each layer may only bind to the layer below it, or to peers
  # within the same layer on the same platform.
  equipment:
    vehicle:
      type: string
      version: string
      parameters: {}
    attachments:
      - station: string          # must exist in the vehicle's descriptor
        type: string
        version: string
        parameters: {}
        quantity: integer        # default 1, for effector loadouts

  subsystem:
    - id: string
      type: string
      version: string
      parameters: {}
      bindings:                  # port -> target
        <port>: <platform-local reference>

  single_ship:
    - id: string
      type: string
      version: string
      parameters: {}
      bindings: {}

# Multi-ship components are declared at formation level in the scenario, not
# on the platform.
```

### 5.1 Reference syntax for bindings

A binding value resolves a required port to a providing port:

```
<component_id>.<port_name>
```

Component IDs are platform-local. `vehicle` and attachment station names are
reserved as implicit component IDs at the equipment layer, so
`radar_main.detections` and `vehicle.state` both resolve.

Where exactly one component on the platform provides the required interface, the
binding may be omitted and the binder will resolve it, recording the resolution
in the run manifest. Where two or more provide it, an explicit binding is
mandatory and omission is an error, not a coin flip.

---

## 6. Worked example — a blue platform

One aircraft, composed bottom-up. Fictional components; the numbers are
plausible placeholders, not claims about any real system.

```yaml
spec_version: 1

platform:
  id: blue_01
  affiliation: blue
  name: Lead

  # ---- Equipment layer: the only layer with a physical part ------------
  equipment:
    vehicle:
      type: vehicle.fighter.generic_2d
      version: "1.2.0"
      parameters:
        fuel_initial_kg: 3200
        empty_mass_kg: 9800

    attachments:
      - station: nose
        type: sensor.radar.pulse_doppler
        version: "0.4.1"
        parameters:
          reference_range_m: 92000
          fov_deg: 120
          for_deg: 120
          pd_at_reference: 0.85
          false_alarm_rate_hz: 0.02
          range_sigma_m: 45
          bearing_sigma_deg: 0.4

      - station: tail_cone
        type: sensor.rwr.generic
        version: "0.3.0"
        parameters:
          for_deg: 360
          bearing_sigma_deg: 6.0

      - station: spine
        type: comms.datalink.generic
        version: "1.0.0"
        parameters:
          range_m: 250000
          latency_mean_s: 0.15
          loss_probability: 0.02

      - station: nav_bay
        type: nav_sensor.ins_gnss
        version: "1.1.0"
        parameters:
          position_sigma_m: 12
          drift_rate_m_per_s: 0.05

      - station: wing_inner_left
        type: effector.missile.bvr_generic
        version: "0.5.2"
        quantity: 2
        parameters:
          max_range_m: 70000
          min_range_m: 3000
          pk_nominal: 0.55

      - station: wing_inner_right
        type: effector.missile.bvr_generic
        version: "0.5.2"
        quantity: 2

  # ---- Subsystem layer: integrates equipment. Purely cyber. ------------
  subsystem:
    - id: vehicle_system
      type: subsystem.vehicle_system.basic
      version: "0.3.0"
      bindings:
        command_out: vehicle.command
        state_in: nav_ins.state

    - id: sensor_system
      type: subsystem.sensor_system.multi
      version: "0.4.0"
      parameters:
        association_gate_sigma: 3.0
      bindings:
        detections_in:
          - radar_main.detections
          - rwr_tail.detections
        sensor_control_out:
          - radar_main.control
        own_state_in: nav_ins.state

    - id: effector_system
      type: subsystem.effector_system.basic
      version: "0.2.1"
      bindings:
        request_in: effector_bank.request
        status_out: effector_bank.status

    - id: comms_system
      type: subsystem.comms_system.basic
      version: "0.2.0"
      bindings:
        transport: datalink_main.message

  # ---- Single-ship layer: one aircraft's decision-making ---------------
  single_ship:
    - id: tracker
      type: single_ship.tracker.kalman_2d
      version: "0.6.0"
      parameters:
        process_noise_q: 2.5
        track_drop_timeout_s: 12.0
      bindings:
        detections_in: sensor_system.tracks

    - id: situation_awareness
      type: single_ship.sa.threat_ranking
      version: "0.3.0"
      bindings:
        tracks_in: tracker.tracks

    - id: action_planner
      type: single_ship.planner.reactive_baseline
      version: "0.2.0"
      parameters:
        commit_range_m: 55000
        abort_range_m: 18000
      bindings:
        picture_in: situation_awareness.picture
        intent_in: coordination.intent      # resolved at formation level
        action_out:
          - vehicle_system.action
          - effector_system.action
          - sensor_system.action
```

### 6.1 What validation does with this

Composition-time checks, all before the clock starts:

1. **Station compatibility.** `nose` exists on `vehicle.fighter.generic_2d`, is
   of a type the radar accepts, and its `mass_limit_kg` is not exceeded.
2. **Mass budget.** Empty mass, plus fuel, plus the sum of attachment masses
   times quantity, is within the vehicle's maximum.
3. **Power budget.** For each vehicle operating mode, the sum of attachment draws
   in their corresponding modes is within the sum of `supplies.power_kw` over
   everything on the platform that declares it. Summing the suppliers rather
   than reading the vehicle's figure alone is what lets a generator be a
   component instead of a vehicle parameter. A radar that cannot be powered in
   the vehicle's cruise mode is a load error, not a runtime surprise.
4. **Port satisfaction.** Every non-optional `requires` resolves to exactly one
   `provides` with a matching interface name and major version.
5. **Truth boundary.** No component with `layer` other than `equipment` holds a
   `truth.query.v1` port.
6. **Layer discipline.** No binding skips a layer or points upward.
7. **Parameter bounds.** Every supplied parameter is declared in the descriptor
   and within its stated bounds.
8. **Rate coherence.** A consumer running faster than its producer is a warning,
   not an error, but it is reported — it is almost always a modelling mistake.

Failures are reported with the file path and line, all at once rather than one
per run. This validation pass is what the composition GUI calls to grey out
incompatible pieces, so it must be fast and side-effect free.

---

## 7. Scenario spec

```yaml
spec_version: 1

scenario:
  id: dca_2v2_baseline
  description: Two-ship defensive counter-air against a two-ship penetrator.
  duration_s: 900
  step_hz: 50
  rate_groups:
    fast: 50        # vehicle dynamics
    medium: 10      # sensors, tracking
    slow: 1         # planning, coordination

  world:
    bounds_m: [0, 0, 400000, 400000]

  formations:
    - id: blue_flight
      affiliation: blue
      members: [blue_01, blue_02]
      multi_ship:
        - id: coordination
          type: multi_ship.coordinator.role_assignment
          version: "0.3.0"
          parameters:
            objective: deny_penetration

  platforms:
    - spec: platforms/blue_fighter.yaml
      id: blue_01
      initial:
        position_m: [180000, 60000]
        heading_deg: 20
        speed_mps: 240
    - spec: platforms/blue_fighter.yaml
      id: blue_02
      initial:
        position_m: [195000, 52000]
        heading_deg: 20
        speed_mps: 240
    - spec: platforms/red_fighter.yaml
      id: red_01
      initial:
        position_m: [210000, 330000]
        heading_deg: 195
        speed_mps: 265
    - spec: platforms/red_fighter.yaml
      id: red_02
      initial:
        position_m: [228000, 338000]
        heading_deg: 195
        speed_mps: 265
    - spec: platforms/airliner.yaml
      id: yellow_01
      initial:
        position_m: [90000, 200000]
        heading_deg: 270
        speed_mps: 230

  termination:
    - type: time_elapsed
    - type: all_of_affiliation_destroyed
      affiliation: red
    - type: boundary_crossed
      affiliation: red
      line_y_m: 100000

  metrics:
    - red_penetration_count
    - blue_losses
    - yellow_engaged           # must remain zero
    - effectors_expended
    - time_to_first_detection_s
```

Note that yellow platforms use the same platform spec structure as blue and red.
The affiliation is a scenario-level attribute, not an architectural one — an
airliner is a vehicle with a transponder, no sensors worth the name, and a cyber
stack that flies a route.

---

## 8. Monte Carlo campaign spec

```yaml
spec_version: 1

campaign:
  id: radar_range_sensitivity
  base_scenario: scenarios/dca_2v2_baseline.yaml
  replications: 300
  master_seed: 20260808

  sweep:
    - path: platforms[blue_01].equipment.attachments[nose].parameters.reference_range_m
      values: [60000, 80000, 100000]
    - path: formations[blue_flight].multi_ship[coordination].parameters.objective
      values: [deny_penetration, preserve_force]

  stochastic:
    - path: platforms[red_01].initial.position_m[0]
      distribution: uniform
      min: 190000
      max: 240000

  output:
    directory: results/radar_range_sensitivity
    format: parquet
    record_traces: false
    record_traces_for_seeds: [0, 1, 2]
```

`sweep` is a full factorial over the listed values — here six cells, three
hundred replications each, eighteen hundred runs. `stochastic` values are drawn
per replication.

**Seed derivation is deterministic and hierarchical.** Every run's seed is
derived as a pure function of the master seed, the sweep cell index, and the
replication index. Each component then derives its own stream from the run seed
and its component ID. Two consequences worth the discipline: any single run can
be reproduced in isolation without replaying the campaign, and adding a component
to a platform does not perturb the random streams of the others.

---

## 9. Lab configuration

A lab is a scenario with one component under test and stubs everywhere else. The
same schema, one extra block:

```yaml
scenario:
  id: lab_radar_main
  mode: lab
  under_test: blue_01.radar_main
  stubs:
    default: passthrough        # unbound ports return declared neutral values
    overrides:
      blue_01.action_planner: scripted
      blue_01.vehicle_system: scripted
  script:
    blue_01:
      - t_s: 0
        command: {speed_mps: 240, heading_deg: 0}
      - t_s: 120
        command: {heading_deg: 90}
```

Because ports are typed and the truth boundary is enforced by the binder, a stub
is generatable from the interface definition alone. That is what makes four
separate lab environments a single feature rather than four.

---

## 10. Pydantic sketch

```python
from typing import Literal
from pydantic import BaseModel, Field, model_validator

Affiliation = Literal["blue", "red", "yellow"]
Layer = Literal["equipment", "subsystem", "single_ship", "multi_ship"]
StationType = Literal["nose", "fuselage", "wing", "internal", "conformal"]


class PortSpec(BaseModel):
    port: str
    interface: str = Field(pattern=r"^[a-z_]+(\.[a-z_]+)+\.v\d+$")
    optional: bool = False


class Station(BaseModel):
    name: str
    type: StationType
    mass_limit_kg: float


class Consumes(BaseModel):
    mass_kg: float = 0.0
    power_kw: dict[str, float] = {}
    station_type: StationType | None = None
    station_count: int = 1


class Supplies(BaseModel):
    power_kw: float = 0.0
    stations: list[Station] = []


class Descriptor(BaseModel):
    descriptor_version: Literal[1]
    type: str
    version: str
    layer: Layer
    category: str
    implementation: str
    consumes: Consumes = Consumes()
    supplies: Supplies = Supplies()
    provides: list[PortSpec] = []
    requires: list[PortSpec] = []
    envelope: dict = {}
    parameters: dict[str, "ParameterSpec"] = {}
    update_rate_hz: float
    rate_group: Literal["fast", "medium", "slow", "event"]

    @model_validator(mode="after")
    def truth_port_is_equipment_only(self):
        holds_truth = any(r.interface.startswith("truth.") for r in self.requires)
        if holds_truth and self.layer != "equipment":
            raise ValueError(
                f"{self.type}: truth.* ports are only grantable to equipment-layer "
                f"components, not {self.layer}"
            )
        return self


class ParameterSpec(BaseModel):
    type: Literal["number", "integer", "string", "boolean", "enum"]
    default: object | None = None
    min: float | None = None
    max: float | None = None
    units: str | None = None
    choices: list[object] | None = None
```

The binder consumes validated `Descriptor` and `PlatformSpec` objects and emits a
frozen `Binding` graph plus a run manifest recording every resolution it made,
including the implicit ones. The manifest is written alongside results — it is
the answer to "what did run 1174 actually consist of?", which is a question every
Monte Carlo campaign eventually raises.

---

## 11. Open questions

- Should effector inventory live on the station (per-pylon) or be pooled per
  platform? Per-station is more faithful; pooled is simpler for planners. Leaning
  per-station with a pooled view exposed by `effector_system`.
- Does `sensing.control.v1` need a priority scheme in v1, or is round-robin
  tasking enough until someone builds a sensor manager worth the name?
- Is `rate_group` the right abstraction, or should components declare a raw
  `update_rate_hz` and let the scheduler bucket them? Named groups are easier for
  contributors to reason about and easier to sweep in a campaign.
- The mass budget of section 6.1 checks attachment masses against "the
  vehicle's maximum", and nothing declares one. The vehicle's constraint
  vector carries `mass_dry_kg` and no upper bound, the model document's lambda
  has seven entries and none is a maximum mass, and the worked example above
  gives a vehicle `empty_mass_kg` and `fuel_initial_kg` but no ceiling.
  Section 5 says the envelope includes "empty and maximum mass", so the intent
  is there and the declaration is not. Adding one is a change to the vehicle
  model's lambda and therefore to the model document, which is a modelling
  decision rather than a validator one. Until then the load checks verify the
  structural half -- mass per station against that station's declared limit --
  and not the whole-aircraft half.
- `consumes.power_kw` is per operating mode and `supplies.power_kw` is a
  single number. That asymmetry does not survive a switched engine: a vehicle
  whose generator is driven by the engine supplies different power in nominal
  and in boost, and there is nowhere to say so. Making supplies per-mode is
  the obvious fix and is a format change, so it waits until the descriptor
  validator has consumers to migrate.
- A power generator consumes fuel, and `consumes` has no fuel field. Fuel is
  also the wrong shape for the power budget above: that check is about
  instantaneous capacity, while fuel is a rate drawn against a finite tank, so
  a platform can pass the power budget and still be unable to fly the mission.
  That is an endurance question rather than a load question, and answering it
  properly needs the energy manager rather than one more descriptor field.
  Deliberately not invented here.
- Formation membership is currently static. Dynamic re-formation would need
  formations to be mutable at runtime, which touches the freeze-after-binding
  rule. Defer.
