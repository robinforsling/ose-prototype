"""Composition-time checking of platform specifications.

Not a layer and not a component: this is the machinery that decides whether a
platform *as specified* can be built at all, before the clock starts. Section
6.1 of docs/40-composition-spec.md lists seven such checks. Only the load
checks live here so far -- station compatibility and the power budget -- and
the module says plainly which of the seven it does not do.

The distinction that makes this worth having is in the specification's own
words: a radar that cannot be powered in the vehicle's cruise mode is "a load
error, not a runtime surprise". Composition-time checks fail loudly with a
list of findings; they are not the declare-and-report pattern the vehicle uses
for its admissible sets, because there is no run yet to report into.
"""

from ose.composition.descriptor import (  # noqa: F401
    Attachment,
    ComponentDescriptor,
    Consumes,
    PlatformSpec,
    Station,
    Supplies,
)
from ose.composition.load_check import (  # noqa: F401
    Finding,
    check_load,
    check_mass_budget,
    check_power_budget,
    check_stations,
)
