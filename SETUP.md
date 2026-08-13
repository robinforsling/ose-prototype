# Setup and run notes

Environment setup for the OSE prototype. Ubuntu / Linux, Python 3.10 or later.

## Layout

An installed package under `src/`, not a flat directory of scripts. Everything
imports `ose.*` by its full path -- demos, tests and the tools alike -- so
nothing depends on which directory it is run from.

```
src/ose/                 the library; see docs/20-architecture.md
pyproject.toml           package metadata, and the pytest configuration
tools/                   generators and checkers, run by hand and by pytest
prefs/                   shared colours for diagrams and plots
demos/                   runnable demonstrations
tests/                   pinning and consistency tests
docs/                    scope, concepts, architecture, ADRs, interfaces
```

## First time

```bash
cd ~/work/projects/saab/ose-prototype
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -e ".[dev]"
```

**The `-e .` matters and is not optional.** It installs `ose` in editable mode,
which is what puts the package on the path for anything that is not pytest.
Skipping it produces a partly-working checkout that is confusing rather than
obviously broken: `pytest` still passes almost everything, because
`pyproject.toml` sets `pythonpath = ["src"]` for pytest only, while every demo
and every tool fails with `ModuleNotFoundError: No module named 'ose'`. The one
test that catches it is the one that runs a tool in a subprocess.

Editable means the install points at `src/` rather than copying it, so edits
take effect with no reinstall. It does record an absolute path -- see
Troubleshooting if the project directory is ever moved or renamed.

The `[dev]` matters too, and for a duller reason: numpy, scipy and matplotlib
are runtime dependencies in `pyproject.toml`, but **pytest is in the `dev`
extra**, so a plain `pip install -e .` leaves you with a checkout whose tests
cannot be run at all. `requirements.txt` lists the same four for anyone who
wants them without the package. Pin exact versions (`numpy==2.4.4`) once
reproducibility of results matters.

The venv is not optional on recent Ubuntu -- the system Python refuses
`pip install` without one. Note that `.venv/bin/pip` is called directly rather
than activating first: every binary inside a venv knows its own environment,
and activation only matters for an interactive shell.

## Every session

```bash
source .venv/bin/activate     # prompt gains (.venv)
deactivate                    # leaves it; closing the terminal does the same
```

A script cannot do the activation for you -- a script runs in a child process
and its environment changes die with it. Use one of the two options below.

### Option A -- shell alias

In `~/.bashrc`:

```bash
alias ose='cd ~/work/projects/saab/ose-prototype && source .venv/bin/activate'
```

### Option B -- direnv, activates on `cd`

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
```

`.envrc` is already in the repository; run `direnv allow` once.

## Running

Tests, after every change:

```bash
pytest
```

That runs the generators and checkers too, so a stale generated document fails
the suite rather than being noticed later.

Demos write into `demos/plots/`, which is gitignored:

```bash
python demos/demo_vehicle.py             # turn envelope and an open-loop manoeuvre
python demos/demo_discretization.py      # one model, several discretisations, tables only
python demos/demo_navigation.py          # INS/GNSS through a GNSS outage
python demos/demo_vehicle_guidance.py    # guidance in closed loop
python demos/demo_mass_estimation.py     # believed mass, and what it promises
python demos/demo_boost.py               # what an afterburner buys and costs
python demos/demo_live_flight.py         # live window, or --video if headless
python demos/demo_live_route.py          # the same, driven by the action planner
```

The two live demos open an interactive window with transport controls, or write
an mp4 with `--video` (which needs `ffmpeg`). They fall back to video
automatically when there is no display.

View output with `xdg-open demos/plots/vehicle_trajectory.png`.

## Tools

```bash
python tools/generate_model_docs.py            # rewrite the model reference tables
python tools/generate_architecture_diagram.py  # rewrite the topology diagram and
                                               # the interface catalogue table
python tools/check_markdown_math.py            # check maths in the markdown
```

Each takes `--check` to report instead of rewriting, which is how `pytest` runs
them. `generate_architecture_diagram.py` also takes `--dump`, which prints the
derived component graph and writes nothing.

Optional, for the maths checker's render pass:

```bash
npm install          # installs katex, per package.json
```

Without node the static rules still run and the render pass is skipped. Nothing
in the library needs node, and `node_modules/` is gitignored.

## Troubleshooting

| Symptom | Cause |
|---|---|
| `ModuleNotFoundError: No module named 'ose'` from a demo or a tool, while `pytest` mostly passes | The package was never installed. Run `pip install -e ".[dev]"`. pytest gets `src/` from `pythonpath` in `pyproject.toml`; nothing else does. |
| `pytest: command not found`, or `No module named 'pytest'` | Installed without the extra. Run `pip install -e ".[dev]"`. |
| `bad interpreter: .../\.venv/bin/python3: No such file or directory` from `pip`, `pytest` or any other venv script | The project directory was moved or renamed after the venv was created. Console scripts hardcode an absolute shebang and the editable install records an absolute path. Easiest fix is to delete `.venv` and redo First time; a targeted fix is to rewrite the old path inside `.venv/bin/*` and in `.venv/lib/python3.*/site-packages/__editable__.ose-*.pth`. |
| `ModuleNotFoundError: No module named 'numpy'` | venv not activated, or dependencies not installed. |
| `error: externally-managed-environment` | Installing outside a venv. Create one. |
| `deactivate: command not found` | The venv was not active to begin with. |
| pytest fails during collection with `ModuleNotFoundError: No module named 'yaml'`, traceback through `launch_testing`/`launch` | A ROS installation's `PYTHONPATH` (e.g. `/opt/ros/humble/...`) is set in the shell. pytest auto-discovers *any* `pytest11` plugin visible on `sys.path`, including ROS's `launch_testing` and `launch_ros`, regardless of venv activation -- activating a venv does not clear `PYTHONPATH`. Already worked around in `pyproject.toml` (`addopts` disables both plugins by name), so this should not recur; if it does elsewhere, `unset PYTHONPATH` before running `pytest` or `python`. |
| A generated document is reported stale and you did not touch it | Something it is derived from changed. Run the generator named in the failure message; do not edit between the `<!-- generated: -->` markers. |

## LaTeX

For the model document:

```bash
sudo apt install texlive-latex-recommended texlive-latex-extra
cd docs/preliminary_models/vehicle
pdflatex vehicle_model.tex
pdflatex vehicle_model.tex      # twice, to resolve cross-references
```

The second pass is needed because `cleveref` writes reference targets to the
`.aux` file on the first pass.

## Worth doing early

`demos/demo_navigation.py` is seeded with `RUN_SEED = 20260808`. Change it and
re-run several times. Filter consistency is a claim about the ensemble, not
about one trajectory, and a single seed can flatter a filter that is quietly
wrong. Each component draws from its own stream spawned off that seed (ADR
0005), so changing it moves every component's noise together and adding a
component does not perturb the others.

Consider replacing `venv` and `pip` with `uv` when convenient. It is a drop-in
replacement, considerably faster, and produces a real lockfile:

```bash
uv venv
uv pip install -e ".[dev]"
```
