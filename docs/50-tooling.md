# Tooling

Status: draft. Nothing here is implemented.

All four tools below are views or transforms over the composition specification.
Building them that way is what keeps them from becoming four separate systems.

## Visualisation

Priority one, ahead of everything else in this file. A crude 2D plan-view
renderer showing platform positions, headings, and detections, updating during a
run. Debugging a multi-platform simulation without it is guesswork.

`demos/demo_live_flight.py` is a throwaway prototype of this, flying a sequence
of guidance setpoints and rendering either to a live window or to video. It
also stands in for the simulation core, which does not exist.
`demos/demo_live_route.py` is the same machinery driven by the action planner
instead of a script, so the whole stack built so far is in the loop; the
transport controls both share live in `demos/_player.py`. None of it is meant
to survive as written, but six things they ran into are worth carrying over.

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

**The renderer reads truth and must stay write-only.** It is a tool rather than
a component, so ADR 0008 does not deny it truth — but nothing it computes may
flow back into the simulation, or the truth boundary has been breached through
a debugging aid, the case that ADR calls hardest to detect afterwards. A real
renderer must also draw truth and belief as two separate things, because they
will differ; the prototype hands guidance a perfect estimate, so they coincide
and it cannot show the distinction it will eventually need to.

## Scenario builder

Generates scenario specifications: N platforms from named platform specs, with
initial conditions, termination conditions, and metrics. Command-line first.

## Monte Carlo runner

Takes a campaign specification, expands the sweep, runs replications in parallel
processes, reduces results. Writes the run manifest alongside results so that
"what was run 1174 made of?" has an answer.

Parallelism is across independent replications, which is why the GIL does not
bind. See ADR 0002.

## Lab environments

A lab is a scenario with one real component and stubs elsewhere. Because ports
are typed and the truth boundary is enforced by the binder, a stub is generatable
from the interface definition alone. This is what makes four lab environments one
feature rather than four.

Labs should carry acceptance tests, not just plots. A navigation filter that is
overconfident is invisible in straight flight and corrupts every tracker
downstream; the check that catches it is a NEES test, and it belongs in the lab.

## Composition GUI

A graphical editor for platform specifications, with pieces that will not fit
together greyed out. The greying-out is the composition-time validator, already
required for headless runs, so the GUI adds presentation rather than logic.

Deliberately last. Building the GUI as the primary artifact and the file format
as an export produces an unmodular tool; building it the other way gives headless
batch runs, reproducibility, git-diffable scenarios and student contributions for
free.
