"""Composition-time checking of platform specifications.

Not a layer and not a component: this is the machinery that decides whether a
platform *as specified* can be built at all, before the clock starts. Section
6.1 of docs/40-composition-spec.md lists seven such checks. Four live here --
station compatibility, the mass budget, the power budget and port satisfaction
-- and load_check.py says plainly which of the seven it does not do, and which
are enforced elsewhere and earlier.

`catalogue.py` holds a descriptor per implemented component type. Before it,
every descriptor in the repository was a test fixture naming a class that does
not exist, so these checks ran entirely against fiction. See ADR 0025.

The distinction that makes this worth having is in the specification's own
words: a radar that cannot be powered in the vehicle's cruise mode is "a load
error, not a runtime surprise". Composition-time checks fail loudly with a
list of findings; they are not the declare-and-report pattern the vehicle uses
for its admissible sets, because there is no run yet to report into.
"""

from ose.composition.catalogue import CATALOGUE  # noqa: F401
from ose.composition.descriptor import (  # noqa: F401
    Attachment,
    ComponentDescriptor,
    Consumes,
    PlatformSpec,
    Port,
    Station,
    Supplies,
)
from ose.composition.load_check import (  # noqa: F401
    Finding,
    check_load,
    check_mass_budget,
    check_ports,
    check_power_budget,
    check_stations,
)
