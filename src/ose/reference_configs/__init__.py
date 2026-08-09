"""Reference configurations for shapes shared across layers.

Not layer-specific, so not nested under resource/, subsystem/, or any other
layer package -- the same reasoning that puts frames.py and environment.py
at this level rather than inside a layer. Layer-specific reference configs
(reference_vehicle.py, ...) live in that layer's own reference_configs/
package instead, e.g. ose/resource/reference_configs/.

Fixtures for tests and demonstrations -- fictional and plausible, per
CLAUDE.md, not evidence that a real configuration belongs in source. See
"Code declares shape; data supplies values" in docs/20-architecture.md.
"""
