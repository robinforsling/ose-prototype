# Graphic preferences

Presentation choices shared by everything that draws: the generated
architecture diagram today, plots and animations as they adopt it.

Data, not code. A colour is a value, and `docs/20-architecture.md` states the
rule this follows -- code declares shape, data supplies values. JSON rather
than YAML or TOML because it needs no dependency on Python 3.10 (`tomllib`
arrives in 3.11, and this project targets 3.10), and because a browser-based
tool can read it unchanged.

## `palette.json`

Keys are **roles, not colours**. `equipment`, never `blue`. The point is that a
plot about navigation and a diagram node for `InsGnssEstimator` come out the
same colour without either author having thought about it -- which only works
if the name says what the thing *is*. A role renamed after its colour would
have to be renamed again the first time the colour changed.

Each role carries three values, because both kinds of consumer need all three:

| Field | Mermaid `classDef` | matplotlib |
|---|---|---|
| `fill` | `fill:` | `facecolor` |
| `stroke` | `stroke:` | `edgecolor`, line colour |
| `text` | `color:` | label colour |

They are one record per role rather than three parallel maps so that a role
cannot acquire a fill without a matching text colour.

Reading it needs no helper:

```python
import json
from pathlib import Path

roles = json.loads((ROOT / "prefs" / "palette.json").read_text())["roles"]
ax.plot(t, x, color=roles["subsystem"]["stroke"])
```

`tools/generate_architecture_diagram.py` treats a missing file or a missing
role as a hard failure rather than falling back to defaults. A diagram that
quietly stopped using the shared colours would look fine and be wrong, which
is the failure mode this repository is least able to detect.

### Why these values

The colours are constrained rather than chosen freely, because the diagram
renders on GitHub in both light and dark themes and the same hex has to work in
each. `tests/test_palette.py` enforces the constraints:

- text against fill is at least 4.5:1 by WCAG relative luminance;
- fill luminance sits in [0.45, 0.90] -- light enough to carry dark text, and
  bright enough to stay ~12:1 against GitHub's dark background `#0d1117`;
- stroke luminance is below 0.25, so an outline still reads against white;
- fills are pairwise separated, so two layers cannot silently converge.

A filled node paints its own background, so the page theme shows only *around*
it -- which is what lets one palette serve both themes. Untinted text does not
have that property, which is why the diagram leaves subgraph titles and edge
labels to the theme instead of colouring them.

Those tests are a **proxy**. Nothing here renders a GitHub page, so they check
the property that makes legibility likely, not legibility itself.

### `multi_ship` has no components yet

Deliberate. The palette is authored preference, not a claim about the code, so
a role may exist before the layer does. Nothing looks it up today.

### What this does not provide

A categorical series cycle -- the colours you would give six lines on one plot.
Four layer roles are not a qualitative palette, and inventing one now would be
describing an unimplemented part as working. When a demo needs one it gets a
separately named key, authored then, against the plots that actually need it.
