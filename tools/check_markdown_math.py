#!/usr/bin/env python3
"""
Check that the maths in the markdown documentation actually renders.

    python tools/check_markdown_math.py            every tracked .md
    python tools/check_markdown_math.py FILE ...   just these

Two passes, because they catch different things.

**Static rules** always run and catch SILENT FALLBACKS -- markup that renders
without complaint and produces the wrong glyph. KaTeX cannot help with these
by definition: it accepted `\\mathbb{1}` happily and quietly emitted a plain
`1`, because its blackboard font has no digits, and the indicator in the boost
page lost its meaning for a week before anyone rendered it.

**KaTeX** runs when node and the katex package are available and catches
outright errors: unknown commands, unbalanced braces, environments that do not
exist. It is the thorough pass, and it is optional because a LaTeX renderer is
a heavy thing to require of someone editing a docstring.

Enable the second pass with:

    npm install katex        # anywhere on NODE_PATH, or in the repository root

Fenced code blocks and inline code spans are stripped before anything is
extracted. Without that, `echo 'eval "$(direnv hook bash)"'` in SETUP.md and a
regex ending `\\.v\\d+$` in the composition spec both parse as maths.
"""

from __future__ import annotations

import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

FENCE = re.compile(r"^```.*?^```", re.M | re.S)
INLINE_CODE = re.compile(r"`[^`\n]*`")
DISPLAY = re.compile(r"\$\$(.*?)\$\$", re.S)
INLINE = re.compile(r"(?<!\$)\$([^$\n]+)\$")

# (pattern, what is wrong). Each of these renders without error and is wrong
# anyway, which is exactly why a renderer cannot be the only check.
SILENT_FALLBACKS = [
    (re.compile(r"\\mathbb\{[^A-Z}]"),
     r"\mathbb has no glyphs outside A-Z in KaTeX; it falls back to plain "
     r"text. Use \mathbf{1} for an indicator"),
    (re.compile(r"\\bm\{"),
     r"\bm is a LaTeX package command KaTeX does not implement. Markdown "
     r"wants \boldsymbol, which renders identically"),
    (re.compile(r"\\vec(x|u|w|c|f|g)\b|\\mat[A-Z]"),
     "a macro from vehicle_model.tex leaked into markdown; those are defined "
     "in the preamble and mean nothing here"),
    (re.compile(r"\\(label|cref|Cref|ref)\{"),
     "a LaTeX cross-reference leaked into markdown; link to the section "
     "instead"),
]


def strip_code(text: str) -> str:
    """Blank out code, preserving line count so reported numbers are right."""
    def blank(m: re.Match) -> str:
        return "\n" * m.group(0).count("\n")
    return INLINE_CODE.sub("", FENCE.sub(blank, text))


def math_spans(text: str):
    """(kind, source, line) for every maths span outside code."""
    stripped = strip_code(text)
    out = []
    for kind, pattern in (("display", DISPLAY), ("inline", INLINE)):
        for m in pattern.finditer(stripped):
            out.append((kind, m.group(1), stripped[: m.start()].count("\n") + 1))
    return out


def check_display_spacing(path: Path, text: str) -> list[str]:
    """GitHub only treats `$$` as maths when it stands alone with blank lines
    around it. Elsewhere it renders the dollars literally."""
    lines = strip_code(text).splitlines()
    marks = [i for i, l in enumerate(lines) if l.strip() == "$$"]
    problems = []
    for k in range(0, len(marks) - 1, 2):
        o, c = marks[k], marks[k + 1]
        if o > 0 and lines[o - 1].strip():
            problems.append(f"{path}:{o + 1}: no blank line before opening $$")
        if c + 1 < len(lines) and lines[c + 1].strip():
            problems.append(f"{path}:{c + 2}: no blank line after closing $$")
    return problems


def katex_available() -> bool:
    try:
        subprocess.run(["node", "-e", "require.resolve('katex')"],
                       cwd=ROOT, capture_output=True, check=True, timeout=30)
        return True
    except (OSError, subprocess.SubprocessError):
        return False


def katex_errors(spans) -> list[str]:
    """Render every span and report what throws."""
    payload = json.dumps([{"display": k == "display", "src": s, "where": w}
                          for k, s, w in spans])
    script = """
const katex = require('katex');
const items = JSON.parse(process.argv[1]);
const out = [];
for (const it of items) {
  try { katex.renderToString(it.src, {displayMode: it.display, throwOnError: true}); }
  catch (e) { out.push({where: it.where, msg: e.message.split('\\n')[0]}); }
}
console.log(JSON.stringify(out));
"""
    result = subprocess.run(["node", "-e", script, payload],
                            cwd=ROOT, capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        return [f"katex failed to run: {result.stderr.strip().splitlines()[-1:]}"]
    return [f"{e['where']}: {e['msg']}" for e in json.loads(result.stdout)]


def markdown_files(argv) -> list[Path]:
    if argv:
        return [Path(a) for a in argv]
    listed = subprocess.run(["git", "ls-files", "*.md"], cwd=ROOT,
                            capture_output=True, text=True).stdout.split()
    return [ROOT / f for f in listed]


def main(argv: list[str]) -> int:
    problems: list[str] = []
    spans_for_katex = []
    checked = 0

    for path in markdown_files(argv):
        text = path.read_text()
        spans = math_spans(text)
        if not spans:
            continue
        checked += 1
        try:
            rel = path.relative_to(ROOT)
        except ValueError:
            rel = path          # a file outside the repository, e.g. a scratch page
        problems += check_display_spacing(rel, text)
        for kind, src, line in spans:
            for pattern, why in SILENT_FALLBACKS:
                if pattern.search(src):
                    problems.append(f"{rel}:{line}: {why}")
            spans_for_katex.append((kind, src, f"{rel}:{line}"))

    print(f"  {len(spans_for_katex)} maths spans in {checked} file(s)")

    if katex_available():
        errors = katex_errors(spans_for_katex)
        problems += errors
        print(f"  katex: {len(spans_for_katex) - len(errors)} rendered, "
              f"{len(errors)} failed")
    else:
        print("  katex: not installed, skipping the render pass "
              "(npm install katex to enable)")

    if problems:
        print()
        for p in problems:
            print(f"  {p}")
        print(f"\n  {len(problems)} problem(s)")
        return 1
    print("  no problems")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
