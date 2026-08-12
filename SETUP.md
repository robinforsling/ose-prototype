# Setup and run notes

Environment setup for the OSE prototype. Ubuntu / Linux.

## Layout

All source files must sit in the same directory, since the demos import
`vehicle` and `navigation` by name.

```
ose/
  vehicle/               vehicle models, one module per model
  navigation.py          navigation systems (additive noise, INS/GNSS)
  demo_vehicle.py        turn envelope + open-loop manoeuvre
  demo_navigation.py     INS/GNSS with a GNSS outage
  integration_demo.py    one continuous model, several discretisations
  requirements.txt
  .gitignore
  .envrc                 optional, for direnv
```

## First time

```bash
cd ~/ose
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r requirements.txt
```

`requirements.txt`:

```
numpy
scipy
matplotlib
```

Pin exact versions (`numpy==2.4.4`) once reproducibility of results matters.

Note that `.venv/bin/pip` is called directly rather than activating first.
Every binary inside a venv knows its own environment; activation is only needed
to make the venv the default for an interactive shell.

The venv is not optional on recent Ubuntu — the system Python refuses
`pip install` without one.

## Every session

```bash
source .venv/bin/activate     # prompt gains (.venv)
deactivate                    # leaves it; closing the terminal does the same
```

A script cannot do the activation for you. A script runs in a child process and
its environment changes die with it. Use one of the two options below instead.

### Option A — shell alias

In `~/.bashrc`:

```bash
alias ose='cd ~/ose && source .venv/bin/activate'
```

### Option B — direnv, activates on `cd`

```bash
sudo apt install direnv
echo 'eval "$(direnv hook bash)"' >> ~/.bashrc
```

Then in the project, create `.envrc`:

```bash
source .venv/bin/activate
```

and run `direnv allow` once. Commit `.envrc` so contributors get it for free.

## Running

```bash
python integration_demo.py    # prints tables only, ~2 s
python demo_vehicle.py        # writes 2 PNGs, ~5 s
python demo_navigation.py     # writes navigation_errors.png, ~30-60 s
```

View output with `xdg-open navigation_errors.png`.

## .gitignore

```
.venv/
__pycache__/
*.pyc
*.png
*.aux
*.log
*.out
```

## Troubleshooting

| Symptom | Cause |
|---|---|
| `ModuleNotFoundError: No module named 'vehicle'` | Wrong directory. `cd` to where the `.py` files are. |
| `ModuleNotFoundError: No module named 'numpy'` | venv not activated, or dependencies not installed. |
| `error: externally-managed-environment` | Installing outside a venv. Create one. |
| `deactivate: command not found` | The venv was not active to begin with. |
| `pytest` fails during collection with `ModuleNotFoundError: No module named 'yaml'`, traceback through `launch_testing`/`launch` | A ROS installation's `PYTHONPATH` (e.g. `/opt/ros/humble/...`) is set in the shell. pytest auto-discovers *any* `pytest11` plugin visible on `sys.path`, including ROS's `launch_testing` and `launch_ros`, regardless of venv activation -- activating a venv does not clear `PYTHONPATH`. Already worked around in `pyproject.toml` (`addopts` disables both plugins by name), so this should not recur; if it does elsewhere, `unset PYTHONPATH` before running `pytest` or `python`. |

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

`demo_navigation.py` is seeded with `default_rng(20260808)`. Change the seed in
`run()` and re-run several times. Filter consistency is a claim about the
ensemble, not about one trajectory, and a single seed can flatter a filter that
is quietly wrong.

Consider replacing `venv` and `pip` with `uv` when convenient. It is a drop-in
replacement, considerably faster, and produces a real lockfile:

```bash
uv venv
uv pip install -r requirements.txt
```
