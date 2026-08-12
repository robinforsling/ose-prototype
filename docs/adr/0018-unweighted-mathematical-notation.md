# 0018 — Unweighted mathematical notation

**Status:** Accepted

## Context

The reference pages under `docs/models/` and the preliminary modelling under
`docs/preliminary_models/` shared a notation in which vectors and matrices were
bold: `\bm` in the LaTeX source, `\boldsymbol` in the markdown, which render
identically. Sets were calligraphic, `\mathcal{U}`, `\mathcal{X}`,
`\mathcal{S}_q`, and the state space was `\mathbb{R}^5`.

In a browser, `\boldsymbol{x}` and `x` came out identical — same face, same
weight — with no error reported anywhere.

The mechanism is font selection. KaTeX renders `\boldsymbol` by emitting a CSS
class whose rule is `font-family: KaTeX_Math; font-weight: bold`, which resolves
to a separate file, `KaTeX_Math-BoldItalic.woff2`. When that file is not served,
the browser falls back to `KaTeX_Math-Italic` — the face a plain scalar already
uses. `\mathbb` and `\mathcal` have the same shape of dependency on
`KaTeX_AMS` and `KaTeX_Caligraphic`.

Three properties of this made it worth a decision rather than a fix:

**It is invisible to every check we have.** `tools/check_markdown_math.py`
renders each span through KaTeX and reports what throws. Nothing throws. The
markup is valid, the renderer is content, and the output is wrong. It is the
`\mathbb{1}` failure again — that one was accepted by KaTeX and emitted a plain
`1`, because the blackboard font has no digits — except one level lower, in
font *delivery* rather than font *coverage*, and therefore not a property of
the document at all. A static rule can ban markup; it cannot check a viewer.

**No markup covers the symbols we actually use.** `\mathbf` is more robust — it
selects `KaTeX_Main-Bold`, the same family as the surrounding upright text — but
it has no effect on lowercase Greek, in real LaTeX and in KaTeX alike. The two
most-used vectors in these documents are $\theta$ and $\lambda$. So the only
markup that could mark them was the one that was failing.

**A distinction that renders sometimes is worse than none.** Weight that
survives in one viewer and vanishes in another means a reader cannot tell an
unmarked scalar from a vector whose marking did not arrive, and the source
keeps looking careful either way.

Three responses were considered and rejected. Fixing the viewer leaves the
documents depending on a font file arriving, which nothing in the repository
can assert. Marking with drawn geometry — `\underline` for a vector, doubled
for a matrix — cannot fail, since KaTeX emits rules rather than glyphs, but it
matches neither the LaTeX source nor any convention a reader of an aircraft
dynamics document expects. Abandoning TeX in the markdown entirely would have
cost the derivations their structure, and converting the preliminary modelling
to markdown would have cost 67 numbered equations, 37 labels and 30
cross-references that markdown cannot express.

## Decision

**Mathematical symbols are written plain in every artefact.** No `\bm`,
`\boldsymbol`, `\mathbf`, `\mathbb`, `\mathcal`, or any other font-selecting
command, in the LaTeX or in the markdown.

**What a symbol is, is declared rather than drawn.** Each reference page opens
with a notation table giving every aggregate symbol its kind — vector and
dimension, matrix and shape, set, indicator — and the per-element tables
already map each symbol to its field in the code. The preliminary modelling
states the kind in the sentence that introduces the symbol.

**`\mathrm` is kept**, for multi-letter subscripts only. Its KaTeX rule is
`font-style: normal` and nothing else: no family, no weight, no file that can
fail to arrive. It has no failure mode to remove, and without it
$c_{\mathrm{TSFC}}$ sets as four italic letters, which reads as a product of
four variables.

**Two symbols were renamed** where removing a font created a collision. Aspect
ratio was `A\!R`, which rendered as a literal `A!R` in a viewer where `\frac`
rendered correctly. The mechanism was not identified — a character escape
applied before the maths renderer would explain it, but the same explanation
predicts that the `\\` row separators in every `bmatrix` would also be eaten,
and those render. It is dropped rather than diagnosed: `\!` is a negative thin
space, and the two letters do not need one. It is now `AR`, and not the bare
`A` first proposed, because `A(v,m)` is already the induced-drag coefficient
in the same derivation.

Other backslash-punctuation spacing survives in these pages — fifteen `\,` and
the `\{ \}` of set-builder notation. If a viewer is found that eats those too,
`\thinspace`, `\lbrace` and `\rbrace` are the backslash-letter spellings and
render identically. Nothing has been changed on speculation. The boost
indicator was `\mathbf{1}[\cdot]` and is now `\chi[\cdot]`, since a plain `1`
is the number one. Wing area $S$ and the switching map $S_q$ coexist because
the latter is never written without its subscript.

**The LaTeX macros are kept and redefined**, not expanded. `\vecx` now expands
to `x`. A reader of the source still sees which symbols are aggregates, the
diffs stay small, and the decision reverses in fifteen lines rather than across
161 call sites.

`tools/check_markdown_math.py` rejects the banned commands in markdown, with
`tests/test_markdown_math.py` parametrised over each one separately so a rule
that stops covering one fails on that one.

## Consequences

**The distinction is weaker on the page, and that is the real cost.** $x$ and
$v$ now look alike, and a reader who has not read the notation table cannot
tell a vector from a scalar by glancing at an equation. Bold did that at a
glance, when it arrived. The table is more informative — it gives dimension and
the code field, which weight never could — but it is a lookup rather than a
glyph, and lookups get skipped.

**The mathematics reads less like mathematics.** Set-builder notation in
particular loses something: $U(x)$ and $X(\lambda)$ were visibly sets when they
were calligraphic. This is a teaching repository, and a student connecting
these pages to a textbook will find the textbook uses the fonts we dropped.

**A collision is now one edit away.** With fonts gone, the alphabet is the only
namespace, and it is already crowded: $A$ against $A(v,m)$, $S$ against $S_q$.
A new symbol has to be checked against every existing one rather than only
those of the same kind. Nothing checks this.

**The PDF gives up weight it could have had.** `\bm` never failed in LaTeX. The
preliminary modelling is unweighted only so that it stays the authority on a
notation the markdown can honour — consistency bought by degrading the artefact
that had no problem.

**In exchange:** the notation renders the same everywhere, in every viewer, with
no dependency on font delivery; a whole class of silent failure is gone rather
than fixed; and all three artefacts — `.tex`, `.md` and `.py` — are in one
register, which is the argument that decided it.

## Related

- ADR 0006 — constraints are declared, not enforced; the admissible sets were
  introduced there as `\mathcal{U}` and `\mathcal{X}`, and are now $U$ and $X$.
- `docs/preliminary_models/README.md` — the notation authority, and the record
  of which artefact derives from which.
