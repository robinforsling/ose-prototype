# Tooling

Status: draft. Nothing here is implemented.

All four tools below are views or transforms over the composition specification.
Building them that way is what keeps them from becoming four separate systems.

## Visualisation

Priority one, ahead of everything else in this file. A crude 2D plan-view
renderer showing platform positions, headings, and detections, updating during a
run. Debugging a multi-platform simulation without it is guesswork.

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
