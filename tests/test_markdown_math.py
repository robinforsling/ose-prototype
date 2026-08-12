"""The maths in the documentation must render.

A formula that renders as raw `$\\boldsymbol{x}$` in a browser, or silently as
the wrong glyph, is worse than no formula: the page still looks authoritative.

The check has two halves and they catch different things, which is the reason
it is not simply "run KaTeX and see":

  static rules   silent fallbacks -- markup that renders WITHOUT error and
                 produces the wrong output. KaTeX cannot catch these by
                 definition. It accepted `\\mathbb{1}` and emitted a plain `1`,
                 because its blackboard font has no digits, and the indicator
                 in the boost page meant nothing for a week.

  katex          outright errors: unknown commands, bad environments,
                 unbalanced braces.

The second half needs node and the katex package, which is a heavy thing to
require of someone editing a docstring, so it is skipped when unavailable and
the static rules still run. `npm install katex` enables it.
"""

import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "tools" / "check_markdown_math.py"


def _run(*args):
    return subprocess.run(
        [sys.executable, str(CHECKER), *args],
        capture_output=True, text=True, cwd=ROOT,
    )


def test_the_documentation_maths_renders():
    result = _run()
    assert result.returncode == 0, (
        "maths in the documentation will not render:\n" + result.stdout + result.stderr
    )


def test_the_checker_is_not_vacuous():
    """It reports how many spans it looked at, and the number has to be real.
    A checker that silently found nothing to check would pass forever -- which
    is what would happen if the fenced-code stripping ever ate the maths too.
    """
    result = _run()
    assert "maths spans in" in result.stdout
    count = int(result.stdout.split("maths spans in")[0].strip().split()[-1])
    assert count > 100, f"only {count} spans found; the extractor is broken"


def test_it_catches_a_silent_fallback(tmp_path):
    """The class of bug the static rules exist for: valid markup, no error,
    wrong glyph. Written to a scratch file so the real pages are untouched."""
    page = tmp_path / "scratch.md"
    page.write_text("An indicator $\\mathbb{1}[x \\in S]$ and a vector $\\bm{x}$.\n")

    result = _run(str(page))
    assert result.returncode == 1
    assert "\\mathbb has no glyphs" in result.stdout
    assert "\\bm is a LaTeX package command" in result.stdout


def test_it_ignores_maths_inside_code_blocks(tmp_path):
    """Without this, `echo 'eval \"$(direnv hook bash)\"'` in SETUP.md and a
    regex ending `\\.v\\d+$` in the composition spec both parse as maths and
    the checker reports nonsense on files that contain none."""
    page = tmp_path / "scratch.md"
    page.write_text(
        "Text.\n\n```bash\necho 'eval \"$(direnv hook bash)\"'\n```\n\n"
        "And inline `pattern=r\"^[a-z]+\\.v\\d+$\"` too.\n"
    )

    result = _run(str(page))
    assert result.returncode == 0, result.stdout
    assert "0 maths spans" in result.stdout


@pytest.mark.skipif(
    subprocess.run(["node", "-e", "require.resolve('katex')"],
                   cwd=ROOT, capture_output=True).returncode != 0,
    reason="katex not installed; npm install katex to enable the render pass",
)
def test_katex_catches_an_outright_error(tmp_path):
    page = tmp_path / "scratch.md"
    page.write_text("$$\n\\sigma \\frobnicate = 1\n$$\n")

    result = _run(str(page))
    assert result.returncode == 1
    assert "Undefined control sequence" in result.stdout
