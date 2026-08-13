# CLAUDE.md

Context for agentic work in this repository. Read this first; follow the
pointers rather than inferring structure from the file tree.

## What this is

An open simulation environment for research and teaching in autonomous combat
aircraft systems. A student or researcher should be able to test one component
inside a complete integrated system without implementing everything around it.

**The emphasis is integration, not fidelity.** Given a choice between a faithful
model and a simple one that preserves the integration problem, choose the simple
one.

Planar (2D). Python. Low fidelity by design. All parameter values in this
repository are fictional and plausible, never claims about any real system.

## Layer structure

Composition is bottom-up. A component may bind to the layer *directly* below it
and to peers in the same layer on the same platform. **Nothing binds upward, and
nothing reaches past a layer.**

Enforced rather than remembered (ADR 0024): `binding_is_allowed()` in
`ose/topology.py` is the rule, `tests/test_layer_discipline.py` applies it to
every component module's imports, and the architecture generator applies it to
derived bindings. Both are needed — a component bound through a protocol has no
import naming what it binds.

| Layer | Contains | Physical? | Package |
|---|---|---|---|
| Multi-ship | Mission objectives, coordination, tasking | No | `ose.multi_ship` |
| Single-ship | Tracker, situation awareness, action planner | No | `ose.single_ship` |
| Subsystem | Vehicle system, sensor system, navigation estimator | No | `ose.subsystem` |
| Equipment | Vehicle, IMU, GNSS, sensors, communicators, effectors | Yes | `ose.equipment` |

Below the equipment layer sits the simulation core, which owns ground truth: the
clock, kinematics, detection, and engagement resolution.

Layer packages are created when they acquire their first component, not before.

**Deciding where a new component goes.** Does it have a physical part, or does
it read ground truth? Equipment. Does it integrate equipment on one platform?
Subsystem. Does it decide for one ship? Single-ship. Does it coordinate across
ships? Multi-ship. If a component seems to span two layers, it is two
components.

Authoritative detail: `docs/10-concepts.md`. Diagram and principles:
`docs/20-architecture.md`.

## Invariants — do not break these

**1. Only equipment-layer components may read ground truth.** Everything above
consumes published estimates. This holds for debugging aids and visualisation
too: one leak invalidates every result produced afterwards and is nearly
impossible to detect later. A cyber-layer component whose signature contains a
truth-carrying type is wrong regardless of what it does with it. See ADR 0008.

**2. Components publish continuous dynamics, never discrete updates.** A model
publishes `f(x, u, ...)`; the consumer chooses the discretisation. That function
must be pure: no hidden state, no internal RNG, no wall-clock dependence, no
logging. Purity is load-bearing — violating it breaks adaptive solvers,
Jacobians, parallel Monte Carlo and reproducibility simultaneously. See ADR 0004.

**3. Components declare constraints; they do not enforce them.** The vehicle
states its admissible sets and integrates whatever command it is given.
Enforcement belongs to guidance or a runtime assurance layer, so that a control
law commanding outside the envelope produces a visible finding rather than a
silently clipped command. Two tests assert this; if they fail, the separation
has been broken. See ADR 0006.

**4. Every measurement record carries its own valid-time and its own declared
uncertainty.** Valid-time is the time the measurement refers to, not the time it
was delivered. The consumer uses the sigma travelling with the measurement,
never a separately configured value.

**5. Each component owns its own RNG stream.** Never share a generator across
components to keep results bit-identical. Adding a component must not perturb
any other component's stream. See ADR 0005.

## Conventions

- **Frame**: planar NED. `p_x` north, `p_y` east, metres.
- **Heading**: `psi` clockwise from north, radians. `omega` positive right.
- **Airspeed vs ground speed**: vehicle state carries airspeed; wind enters
  position kinematics only. Heading is air-relative and ground track differs
  whenever wind is non-zero. Conflating them is a recurring source of error.
- **Units**: SI, angles in radians internally, degrees in authored files with the
  unit in the field name (`fov_deg`).
- **Interfaces** live in `src/ose/interfaces.py` and contain no implementations.
  Components depend on that module, never on each other.
- **Licensing**: Apache 2.0, copyright Saab AB. `LICENSE` and `NOTICE` at the
  repository root cover every file; do not add per-file licence headers or SPDX
  lines. Apache 2.0 recommends them but does not require them, and they would
  displace the module docstring that every file here opens with.

## Working in this repository

Run the tests after every change:

```bash
pytest
```

**Never weaken a test threshold to make a test pass.** The NEES bound, the
three-sigma containment fractions, and the observability ratios are calibrated.
If one starts failing, the change is wrong.

**Record architectural decisions as ADRs** in `docs/adr/`, numbered, stating
consequences you dislike as well as benefits. A new decision means a new record
referencing the ones it changes, plus a forward pointer in their status lines.
Revise an accepted record when a statement in it stops being true — a reader
should never be misled by one — but keep its Context as written, since that is
the reasoning the decision came from. Git holds the history.

**Prefer adding fields to published records over changing them.** Adding is
backward compatible; removing or renaming requires a version increment on the
interface.

**Changing a vehicle model or its reference configuration means regenerating
its page.**

```bash
python tools/generate_model_docs.py
```

`docs/models/vehicle/` carries a page per model, and the numeric parts of it —
parameter tables, limit tables, turn-performance tables — are generated from
the code between `<!-- generated: NAME -->` markers. `pytest` runs the
generator with `--check` and fails when they are stale, so this is enforced
rather than remembered.

The prose between the markers is written, not generated, and no version of
that tool should try to produce it. The value of those pages is the behaviour:
which limit binds where, what is counter-intuitive, what a naive policy does.
That is a judgement about what a reader would otherwise get wrong, and a page
assembled from field names would be a worse version of the source code. When a
model's behaviour changes, the tables update themselves and **the prose is
still yours to fix**.

A new vehicle model needs a page before the suite will pass; the check is
named after the module.

**Adding a component or an interface means regenerating the architecture
diagram.**

```bash
python tools/generate_architecture_diagram.py
```

`docs/20-architecture.md` carries a Mermaid diagram of the implemented
topology, and `docs/interfaces/README.md` the implemented half of the
interface catalogue. Both are derived from the source between
`<!-- generated: NAME -->` markers — components, layers, publications,
consumptions — and `pytest` fails while either is stale. `--dump` prints the
derived graph and writes nothing, which is how to see what changed before it
reaches a page. The tool also exits non-zero if a component outside the
equipment layer reads truth, so it enforces invariant 1 as well as drawing it.
See ADR 0020.

The prose around the blocks is written, not generated, and says what the
derivation cannot see. Two things today: `FuelGauge` depends on the vehicle
through a `float`, and `Clock`'s truth input is deliberately unprefixed.

**An interface name belongs to the port, not the record** (ADR 0021). A record
declares the default as `INTERFACE: ClassVar[str]`, e.g. `"sensing.imu.v1"`,
and either has one or is listed in `NOT_A_PORT` in `ose/interfaces.py` with the
reason; `catalogue()` raises on any record that is neither, which is what makes
a forgotten registration an error rather than a silence. A component overrides
it with `PUBLISHES: ClassVar[dict[str, str]]`, method name to interface, where
one record serves two ports — an `OwnStateEstimate` from a navigation source is
not the platform's published state, and until the ports were named apart
nothing could tell.

The `ClassVar` annotation is load-bearing: without it `INTERFACE` becomes a
real dataclass field, silently, and the name turns into a constructor argument.

**A constructor parameter typed by a protocol is a declared port**; one typed
by a concrete class is composition. The diagram draws the first as a labelled
publication and the second as a plain binding, which is what keeps it from
inventing a consumption that is really just a method call.

**Shared colours live in `prefs/palette.json`**, keyed by semantic role
(`equipment`, `subsystem`, `truth`), not by colour. Plots and animations should
read the same file rather than choosing their own, so one concept is one
colour everywhere. The values are constrained by contrast tests, not free.

**Maths in markdown is checked too.**

```bash
python tools/check_markdown_math.py
```

Two passes. Static rules always run and catch *silent fallbacks* — markup that
renders without error and produces the wrong glyph, which a renderer cannot
catch by definition. `\mathbb{1}` was accepted by KaTeX and emitted a plain
`1`, because its blackboard font has no digits. The second pass renders every
span through KaTeX and needs `npm install katex`; it is skipped when node is
absent rather than becoming a dependency.

**Symbols are written plain — no `\bm`, `\boldsymbol`, `\mathbf`, `\mathbb`,
`\mathcal`, in the markdown or in the LaTeX** (ADR 0018). Each one selects a
font file the reader's browser may not have, and a font that fails to load is
not an error: it falls back to the face a plain scalar already uses, so the
distinction lives in the source and never reaches the page. Nothing can test
for that — the markup is valid and the renderer is content — so the markup is
banned instead. `\mathrm` is kept for multi-letter subscripts; it selects
upright shape and no font, so it has no failure mode.

What a symbol is gets **declared, not drawn**: every reference page opens with
a notation table giving each aggregate its kind and dimension, and the
per-element tables map each symbol to its field in the code. That is also why
the alphabet is now the only namespace — check a new symbol against every
existing one, not just those of the same kind. `A` is taken, twice.

**In markdown, never write a backslash before punctuation.** It is a
CommonMark character escape, eaten before the maths renderer sees the span, so
the command vanishes and the punctuation stays: `\,` rendered as a comma, and
`\{` dropped the brace entirely — a bare `{` is a TeX group, so set-builder
notation lost its braces while still looking deliberate. Backslash-*letter*
commands are unaffected, which is why `\frac` worked on the same page and made
this look like a font problem for two rounds of diagnosis. Write `\thinspace`,
`\lbrace`, `\rbrace`, `\cr`, `\Vert`; a backslash before a *space* is safe.
The LaTeX keeps the short forms — nothing parses it as markdown.

**Matrices and any `&` go in `$$` blocks, never inline `$…$`.** An `&` is an
HTML entity introducer, escaped during inline processing, and the span then
stops being recognised as maths at all — the reader gets the source, dollars
included. Note what that means for the checker: KaTeX renders such a span
perfectly, because the breakage is in the markdown parser's delimiter scan,
upstream of the renderer. Three separate defects in these pages have now been
of that shape — valid TeX, contented renderer, wrong page — which is the
argument for the static rules existing at all.

## Testing philosophy

Test properties, not appearances. Any component that publishes an uncertainty
must have a test that its stated uncertainty is honest.

This is not boilerplate. The INS/GNSS filter shipped with a one-step
misalignment between the mechanised state and the truth used to form measurement
residuals. In straight flight the velocity vector is not rotating, so the
residual vanished and every plot looked correct. Under turn it injected a
systematic 3 m/s residual against 0.15 m/s of measurement noise, and the filter
absorbed it into heading and bias, finishing thirty times overconfident. Four
wrong hypotheses were investigated before the cause was found. A NEES test would
have pointed at it in seconds.

Related: prefer testing a property directly over testing a downstream
consequence. A test that an inadmissible command was not clipped should check
that heading advanced by exactly `omega * dt`, not that speed increased — speed
depends on thrust and turn rate together and cannot isolate either.

**Check the whole object, not the part a consumer happens to read.** This has
bitten three times:

- `capability_bound()` was checked on the three channels `GuidanceCapability`
  republishes, so it passed while `endurance_s` came back *longer* than the
  point estimate — an anti-conservative number wearing the name of a bound.
- A two-state filter's consistency was checked as a scalar on the fuel
  channel, so it passed at ANEES 1.05 against an unmodelled fuel sink that put
  the full state at 9.07. The damage had landed in the state nobody looked at.
- Six truth-boundary guards named their subject with a literal module string,
  so a package rename would have left them matching nothing and passing.

The shape is always the same: the assertion covers the convenient part rather
than the thing actually returned. Walk the dataclass, use the full covariance,
resolve the name at runtime. Each of these was found by asking what the test
would do if the object grew, not by the test failing.

Excite the system, do not just run it. The navigation consistency test flies a
turn because the bug it exists for is invisible in straight flight; the mass
filter's flies a throttle profile because a constant fuel sink is degenerate
with a burn-coefficient error at constant thrust.

## Current state

**`NavigationManager` is the platform's single PNT publisher** (ADR 0022). It
publishes `vehicle.state.v1` and `platform.time.v1`, over one own-state source
and one time source, each of which publishes on its own `*_source.v1` port so
that a source estimate is distinguishable from the platform's answer. It does
not fuse; `time_source` is keyword-only so that the constructor still refuses
two own-state sources.

Implemented, with tests: the baseline vehicle model; navigation sensors and the
INS/GNSS estimator; a platform clock and a dead-reckoning time estimator; a fuel
gauge feeding a vehicle manager; vehicle guidance; and a single-ship action
planner following a route of waypoints. Integrators live in
`ose/integration.py`, outside the components they step. Every equipment component publishes a `capability()`, as does `VehicleGuidance`, whose capability is
composed from the vehicle's envelope and the navigation covariance it steers
on.

**`VehicleManager` is the only component that may bind a vehicle model** (ADR
0015). It and `Imu` bind the `Vehicle` port rather than a concrete model, which
is why the diagram shows one `vehicle` node: the two models are alternatives
chosen at composition time, not collaborators (ADR 0023). It owns the
platform's believed mass — dry + payload + fuel, where only
fuel is measured — and answers vehicle questions at that mass, so nothing above
it takes a mass parameter. An import test enforces this; `ose/integration.py`
and the equipment layer are the exemptions. Fuel is tracked by a two-state
filter over `[fuel_kg, tsfc_error]`, predicting on commanded thrust and
correcting on each gauge reading; the burn-coefficient error is a state
because a bias modelled as process noise makes a filter overconfident.

**`capability()` for what you compute with, `capability_bound()` for what you
promise** (ADR 0016). The bound widens the envelope by the live mass
uncertainty and is what guidance publishes upward; feedforward and enforcement
use the point estimate, because thrust computed for a mass the aircraft does
not have is wrong rather than cautious, and clipping against a margin would
make a `Saturation` finding mean estimator doubt instead of an airframe
limit.

Partly implemented: the composition-time checks live in `ose/composition/` --
station compatibility, the mass budget, the power budget and port satisfaction,
over descriptor records rather than YAML, since the repository has no schema
library and parsing is separable from the rules.

`ose/composition/catalogue.py` holds a descriptor per implemented type, where a
**type is a component class paired with a configuration** (`Imu` with
`TACTICAL_GRADE`), not a class. Descriptors are authored, because a binder must
be able to read one without constructing anything -- and cross-checked against
the code, because authored data drifts: `tests/test_descriptor_catalogue.py`
resolves every `implementation` and asserts the declared layer and ports match
what the architecture generator derives. Add a port to a component and the
descriptor has to follow, or the suite fails. See ADR 0025.

Not implemented: the simulation core, the service registry, the composition
binder, the rest of the descriptor validator, and every component type other than the above
— no sensors beyond navigation, no communicators, no effectors, no tracker or
situation awareness, and nothing at the multi-ship layer. `docs/40-composition-spec.md` describes the
intended specification format; nothing consumes it yet.

Do not describe unimplemented parts as working, in code comments or in
documentation.

## Where to look

| For | See |
|---|---|
| Scope, and what is deliberately excluded | `docs/00-scope.md` |
| Vocabulary, frames, conventions | `docs/10-concepts.md` |
| Structure and principles | `docs/20-architecture.md` |
| Why something is the way it is | `docs/adr/` |
| Interface catalogue | `docs/interfaces/README.md` |
| Composition specification format | `docs/40-composition-spec.md` |
| Shared colours for diagrams and plots | `prefs/` |
| Vehicle model mathematics | `docs/preliminary_models/vehicle/vehicle_model.pdf` |
| What a model does, and its reference numbers | `docs/models/` |
| Planned tooling | `docs/50-tooling.md` |
