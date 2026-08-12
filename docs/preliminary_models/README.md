# Preliminary models

Where modelling starts. These documents establish the **notation** and the
**basic formulation** — state and input vectors, the dynamics, the admissible
sets, the capability model, and the reasoning behind each choice. The code in
`src/ose/` is then written from them.

They are not a specification the code is measured against, and they are not
where a reader should go to find out what the simulation currently does.

## Three artefacts, one direction

```
docs/preliminary_models/     notation and the basic formulation
        │                    the modelling that the code is written FROM
        ↓
src/ose/                     THE MODEL. This is what runs
        │
        ↓
docs/models/                 descriptions, derived from the code
```

**The developed model is the code.** A preliminary document is an input to it.
A reference page is an account of it.

**The reference pages are derived from the code, never from these documents.**
That is the rule worth stating outright, because writing a page from the
mathematics is the easy mistake and produces something that agrees with the
theory and may quietly disagree with the software — a description of what was
intended rather than what exists.

It is enforced for everything that can be: `tools/generate_model_docs.py`
imports from `ose.equipment.*` and computes every table and every quoted
figure from the reference configuration, and `pytest` fails while any of them
is stale. Nothing in that path reads a `.tex` file. What a generator cannot
derive is the prose describing behaviour, and that is written from the code
too — by running it.

## What these documents remain authoritative on

**Notation.** The symbols used by the code, the reference pages and the ADRs
are defined here: $x$, $u$, $\theta$, $\lambda$, $\eta$, $U$, $X$, $S_q$, and
the rest. A page under `docs/models/` uses the same symbols so the two read
together, and a field name in the code maps to one of them.

Symbols are **unweighted** — no `\bm`, no `\mathbb`, no `\mathcal` — and that
is a decision this document is the authority on, because the reference pages
follow it (ADR 0018). Whether a symbol is a scalar, a vector, a matrix or a
set is stated, in the sentence that introduces it here and in the notation
table of each reference page. Drawing it instead would have meant a font, and
a font that does not load is not an error: it falls back to the face a plain
scalar already uses, leaving a distinction that is real in the source and
absent on the page. It also puts all three artefacts in the same register —
there is no bold in a `.py` file either.

`\mathrm` is the one exception, kept for multi-letter subscripts. It selects
no font, only upright shape, so it has no failure mode; without it
$c_{\mathrm{TSFC}}$ sets as four italic letters and reads as a product.

**Why a formulation is what it is.** The derivations, the sign conventions,
and the arguments for one choice over another live here rather than being
re-derived in a docstring.

## When they disagree with the code

They will, and the direction of the fix is not automatic:

- **A deliberate change to the model** should come back into these documents.
  They stop being useful the moment they describe a formulation nobody
  implements.
- **An accidental difference** is a bug in the code, and having the reasoning
  written down here is what makes it possible to tell the two apart.

Both have happened. The switching set $S_q$ carried a formulation
whose single "otherwise" branch made an anti-chattering rule produce
chattering, found by simulating the implementation rather than by reading the
document. The composition-time mass budget referred to "the vehicle's maximum"
when no $\lambda$ declared one. In both cases the document was
corrected and an ADR records why.

## Contents

| Document | Covers |
|---|---|
| [`vehicle/vehicle_model.tex`](vehicle/vehicle_model.tex) | both planar vehicle models: dynamics, constraints, capability, and the two-mode nominal/boost extension |

The PDF is committed beside the source so it can be read without a LaTeX
installation. Rebuild after editing, twice — `cleveref` resolves its
references on the second pass. See [`SETUP.md`](../../SETUP.md).
