"""The shared palette must stay legible in both GitHub themes.

prefs/palette.json colours the generated architecture diagram, and one set of
hex values has to work against a white page and against GitHub's dark
background. A colour that fails is not an error anywhere -- the page renders,
the diagram appears, and some fraction of readers cannot read it.

So the properties that make legibility likely are pinned here. They are a
PROXY and worth naming as one: no test in this repository renders a GitHub
page, and passing these does not prove the diagram is readable. It proves the
contrast maths holds, which is the part a test can reach.

Every role in the file is checked, not only the roles the diagram currently
uses. A role added for a future layer and quietly given an unreadable colour
would otherwise sit there until the layer arrived.
"""

from __future__ import annotations

import itertools
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PALETTE = ROOT / "prefs" / "palette.json"

HEX = re.compile(r"^#[0-9a-f]{6}$")

# GitHub's dark canvas. The fill has to stay bright against this at the same
# time as carrying dark text, which is what bounds fill luminance from both
# sides below.
GITHUB_DARK = "#0d1117"

FIELDS = ("fill", "stroke", "text")

# Minimum euclidean RGB separation between any two fills. Two layers rendering
# in near-identical pastels is a diagram that looks informative and is not.
MIN_FILL_DISTANCE = 25.0


def roles() -> dict[str, dict[str, str]]:
    return json.loads(PALETTE.read_text())["roles"]


def _channels(colour: str) -> list[float]:
    raw = [int(colour.lstrip("#")[i:i + 2], 16) / 255.0 for i in (0, 2, 4)]
    return [c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4 for c in raw]


def luminance(colour: str) -> float:
    """WCAG 2.1 relative luminance."""
    r, g, b = _channels(colour)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def contrast(a: str, b: str) -> float:
    """WCAG 2.1 contrast ratio, 1.0 to 21.0."""
    la, lb = luminance(a), luminance(b)
    return (max(la, lb) + 0.05) / (min(la, lb) + 0.05)


def test_the_palette_is_not_vacuous():
    """Every test below iterates the roles, so all of them pass on an empty
    file."""
    assert roles(), "no roles in the palette"
    assert len(roles()) >= 4, "fewer roles than there are layers"


def test_every_role_declares_every_field():
    """Three fields per role, so a role cannot acquire a fill without a text
    colour to sit on it."""
    for name, role in roles().items():
        missing = set(FIELDS) - set(role)
        assert not missing, f"role {name} is missing {sorted(missing)}"


def test_every_value_is_a_lowercase_six_digit_hex():
    """Mermaid accepts other forms; the tests here do not parse them, so a
    three-digit or named colour would slip past every contrast check below
    with a crash rather than a verdict."""
    for name, role in roles().items():
        for field in FIELDS:
            assert HEX.match(role[field]), (
                f"{name}.{field} is {role[field]!r}, not a #rrggbb hex"
            )


def test_text_is_readable_on_its_own_fill():
    for name, role in roles().items():
        ratio = contrast(role["text"], role["fill"])
        assert ratio >= 4.5, (
            f"{name}: text {role['text']} on fill {role['fill']} is {ratio:.2f}:1, "
            "below the 4.5:1 needed for body text"
        )


def test_fills_are_light_enough_for_dark_text_and_bright_enough_for_a_dark_page():
    """The two-sided bound is what makes one palette serve both themes.

    A filled node paints its own background, so the page theme shows only
    around it. Too dark and the pinned dark text fails; too light and the node
    stops reading as a distinct shape.
    """
    for name, role in roles().items():
        lum = luminance(role["fill"])
        assert 0.45 <= lum <= 0.90, (
            f"{name}: fill {role['fill']} has luminance {lum:.3f}, outside "
            "[0.45, 0.90]"
        )
        against_dark = contrast(role["fill"], GITHUB_DARK)
        assert against_dark >= 10.0, (
            f"{name}: fill {role['fill']} is {against_dark:.2f}:1 against "
            f"GitHub dark {GITHUB_DARK}"
        )


def test_strokes_read_against_a_white_page():
    for name, role in roles().items():
        lum = luminance(role["stroke"])
        assert lum < 0.25, (
            f"{name}: stroke {role['stroke']} has luminance {lum:.3f}; too "
            "light to outline a node on white"
        )


def test_fills_are_pairwise_distinct():
    """Checked over every pair, not against a reference colour.

    Two roles converging is a property of the pair, and a diagram whose
    equipment and subsystem layers render the same colour is worse than one
    with no colour at all -- it looks like it is saying something.
    """
    fills = {name: role["fill"] for name, role in roles().items()}
    for a, b in itertools.combinations(sorted(fills), 2):
        pa = [int(fills[a].lstrip("#")[i:i + 2], 16) for i in (0, 2, 4)]
        pb = [int(fills[b].lstrip("#")[i:i + 2], 16) for i in (0, 2, 4)]
        distance = sum((x - y) ** 2 for x, y in zip(pa, pb)) ** 0.5
        assert distance >= MIN_FILL_DISTANCE, (
            f"fills for {a} ({fills[a]}) and {b} ({fills[b]}) are {distance:.1f} "
            f"apart, below {MIN_FILL_DISTANCE}"
        )


def test_the_contrast_maths_is_calibrated():
    """The sabotage for every threshold above.

    Without it, a bug in luminance() that returned a constant would make every
    other test in this file pass unconditionally. The reference values are
    WCAG's own: black on white is 21:1, and a colour against itself is 1:1.
    """
    assert luminance("#ffffff") == 1.0
    assert luminance("#000000") == 0.0
    assert round(contrast("#000000", "#ffffff"), 2) == 21.0
    assert round(contrast("#777777", "#777777"), 2) == 1.0
    # A mid grey that WCAG puts just over 4.5:1 on white, so the threshold
    # used above is anchored to a known point rather than to itself.
    assert 4.4 <= contrast("#767676", "#ffffff") <= 4.6


def test_every_role_the_diagram_uses_exists():
    """Layer roles are looked up by layer name, so a layer package with no
    matching role would fail at generation time rather than here."""
    from ose.topology import LAYER_PACKAGES

    missing = set(LAYER_PACKAGES) - set(roles())
    assert not missing, f"no palette role for layer(s) {sorted(missing)}"
    assert "truth" in roles(), "no role for the truth boundary"
