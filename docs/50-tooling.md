# Tooling

Status: draft. None of the four *planned* tools below is built. The
composition-time load checks the GUI section depends on are partly implemented
in `ose/composition/`; the live demos are throwaway prototypes of the
renderer, not the renderer.

All four planned tools are views or transforms over the composition
specification. Building them that way is what keeps them from becoming four
separate systems.

## Tools that exist

Three, all of the same shape: a script under `tools/`, run by hand, and run
again by `pytest` so that what it checks cannot be quietly left undone.

| Tool | Does | Enforced by |
|---|---|---|
| `generate_model_docs.py` | Rewrites the computed tables in `docs/models/vehicle/` from the models and their reference configurations. | `tests/test_model_docs.py` |
| `generate_architecture_diagram.py` | Derives the implemented topology from the source and writes the Mermaid diagram in `docs/20-architecture.md` and the implemented interface table in `docs/interfaces/README.md`. Also fails if a component outside the equipment layer reads truth, or if a binding goes upward or reaches past a layer. See ADR 0020, ADR 0024. | `tests/test_architecture_diagram.py` |
| `check_markdown_math.py` | Static rules against markup that renders wrongly without erroring, plus an optional KaTeX render pass when node is present. | `tests/test_markdown_math.py` |

Each takes `--check` (or, for the maths checker, no arguments) and exits
non-zero with the command to run. `generate_architecture_diagram.py` also takes
`--dump`, which prints the derived graph and writes nothing — the way to see
what changed before it reaches a page.

They generate numbers and structure, never prose. What a page *means* is a
judgement, and a generator that tried to produce it would write a worse version
of the source code.

## Planned

None of these exists. Each is a view or a transform over the
composition specification, which nothing consumes yet.

### Visualisation

Priority one, ahead of everything else in this file. A crude 2D plan-view
renderer showing platform positions, headings, and detections, updating during a
run. Debugging a multi-platform simulation without it is guesswork.

Three throwaway prototypes of this exist, sharing their transport controls
via `demos/_player.py` and each standing in for the simulation core, which
does not exist:

  `demo_live_flight.py`  a scripted sequence of guidance setpoints
  `demo_live_route.py`   the same machinery driven by the action planner, so
                         the whole stack built so far is in the loop
  `demo_boost.py`        two platforms flown together, differing only in a
                         policy, which is the shape a comparison takes

None is meant to survive as written, but eight things they ran into are worth
carrying over.

**Simulate first, render second.** The prototype's loop records everything and
returns; the renderer reads the recording afterwards. Interleaving them couples
frame rate to step size and makes a run unrepeatable whenever the display
stalls. Keeping them apart also means one recording can be replayed live,
written to video, or diffed against another run — which is most of what a
debugging tool is for.

**Record what was asked for, not only what happened.** A clipped command looks
exactly like a command. Enforcement is invisible unless the pre-enforcement
value is kept alongside the delivered one, which is why `Saturation` carries
`requested`. An earlier demo lacked it, duplicated the control law to recover
the number, and the duplicate silently went stale.

**Record the bounds too.** The turn-rate limit moves with speed and mass, so a
fixed line drawn on a plot would be wrong. Anything state-dependent that a
reader will compare a signal against has to be sampled at the same instant as
the signal.

**Decimate for display, not for simulation.** Physics wants 50 Hz and an eye
wants 25 fps. Conflating the two either wastes frames or corrupts the dynamics.

**A viewer needs transport controls, not just playback.** The first version
played once at a fixed rate, which is enough to see that something happened
and not enough to work out why. Pause, single-step, scrub and a speed control
turned it from a demonstration into an instrument. Keeping the recording in
memory is what makes seeking trivial, which is the first lesson paying for
itself.

**Draw the decision, not only the trajectory.** Once a planner is in the loop a
track alone stops being enough: an infeasible corner and a deliberate orbit
draw the same shape. The route demo has to show the route as authored, which
waypoint is active, the capture radius holding at that instant, and the
interval where the planner has stopped publishing motion at all — otherwise a
plan that failed looks like a plan that finished. The rule generalises: for
every component that decides something, render what it decided and what it
decided it against, beside the outcome.

**A plan view has to follow when the features are smaller than the track.**
The boost demo's mission spans about 25 kilometres while the turn radii that
carry its entire argument are one to two. Fitting the whole track renders both
turns as dots; a window a few kilometres wide, recentred on the platforms each
frame, keeps them legible throughout. The general form is that a fixed extent
is only right when the interesting scale and the travelled scale are within an
order of magnitude of each other, which for a combat aircraft they rarely are.

**The renderer reads truth and must stay write-only.** It is a tool rather than
a component, so ADR 0008 does not deny it truth — but nothing it computes may
flow back into the simulation, or the truth boundary has been breached through
a debugging aid, the case that ADR calls hardest to detect afterwards. A real
renderer must also draw truth and belief as two separate things, because they
will differ; the prototype hands guidance a perfect estimate, so they coincide
and it cannot show the distinction it will eventually need to.

### Scenario builder

Generates scenario specifications: N platforms from named platform specs, with
initial conditions, termination conditions, and metrics. Command-line first.

### Monte Carlo runner

Takes a campaign specification, expands the sweep, runs replications in parallel
processes, reduces results. Writes the run manifest alongside results so that
"what was run 1174 made of?" has an answer.

Parallelism is across independent replications, which is why the GIL does not
bind. See ADR 0002.

### Lab environments

A lab is a scenario with one real component and stubs elsewhere. Because ports
are typed and the truth boundary is enforced by the binder, a stub is generatable
from the interface definition alone. This is what makes four lab environments one
feature rather than four.

Labs should carry acceptance tests, not just plots. A navigation filter that is
overconfident is invisible in straight flight and corrupts every tracker
downstream; the check that catches it is a NEES test, and it belongs in the lab.

### Composition GUI

A graphical editor for platform specifications, with pieces that will not fit
together greyed out. The greying-out is the composition-time validator, already
required for headless runs, so the GUI adds presentation rather than logic.

Part of that validator exists: `ose/composition/` checks station
compatibility, the mass budget and the power budget over descriptor records.
It returns findings rather than raising, which is what a GUI needs -- a
listing of everything wrong with a configuration, not the first thing.

Deliberately last. Building the GUI as the primary artifact and the file format
as an export produces an unmodular tool; building it the other way gives headless
batch runs, reproducibility, git-diffable scenarios and student contributions for
free.
