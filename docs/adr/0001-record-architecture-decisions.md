# 0001. Record architecture decisions

Status: accepted
Date: 2026-08-09
## Context

The environment is intended to outlive any single project and to accept
contributions from students and researchers who were not present when its
structure was chosen. Decisions made once and left undocumented are re-litigated
annually, usually by someone who has just been bitten by a consequence and does
not know it was deliberate.

## Decision

Every architectural decision is recorded as a short, numbered file in
`docs/adr/`. An ADR states the context, the decision, and the consequences
including the ones we dislike. A new decision means a new record that references
the ones it changes, plus a forward pointer in their status lines. An accepted
record is revised when a statement in it stops being true, so that reading one
never misleads; its Context stays as written, being the reasoning the decision
came from. Git holds the history.

## Consequences

There is a written answer to "why is it like this", which is the question that
otherwise consumes supervision time. The cost is a few paragraphs per decision.
