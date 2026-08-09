"""Reference configurations for resource-layer components.

Fixtures for tests and demonstrations -- fictional and plausible, per
CLAUDE.md, not evidence that a real configuration belongs in source. One
module per resource (reference_vehicle.py, ...) or per shared, cross-cutting
shape used by more than one resource (reference_environment.py), each
holding one or several named configurations.

See "Code declares shape; data supplies values" in docs/20-architecture.md.
Migrating these to real composition-spec YAML (docs/40-composition-spec.md
sec 4) is the natural next step once the descriptor validator exists; today
they are Python because nothing consumes YAML yet, not because that is
where this data should live long term.
"""
