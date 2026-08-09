"""Reference configurations for resource-layer components.

Fixtures for tests and demonstrations -- fictional and plausible, per
CLAUDE.md, not evidence that a real configuration belongs in source. One
module per resource (reference_vehicle.py, ...), each holding one or
several named configurations for that resource.

Reference configs for shapes shared across layers (e.g. Environment) live
in ose/reference_configs/ instead, not here -- see its package docstring.

See "Code declares shape; data supplies values" in docs/20-architecture.md.
Migrating these to real composition-spec YAML (docs/40-composition-spec.md
sec 4) is the natural next step once the descriptor validator exists; today
they are Python because nothing consumes YAML yet, not because that is
where this data should live long term.
"""
