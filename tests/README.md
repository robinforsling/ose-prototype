# Tests

```bash
pytest              # all
pytest -q -k vehicle
pytest -q -k consistent
```

## What is being pinned

`test_vehicle.py` checks identities that follow from the model document rather
than numbers that happened to come out of a run: the coordinated-turn relation,
induced drag scaling with load factor squared, the stall speed closed form,
corner speed maximising instantaneous turn rate, thrust required equalling drag,
sustained turn rate balancing thrust and drag, fourth-order convergence of the
integrator, and the frame conventions.

Two tests assert what the vehicle deliberately does *not* do: it integrates an
inadmissible command as given rather than clipping it. If those start failing,
the separation in ADR 0006 has been broken.

`test_navigation.py` checks filter consistency by NEES across several seeds,
that the published covariance is positive semi-definite, that estimates are not
truth passed through, and the observability structure — heading variance must
not shrink before the first turn, and must collapse during it.

## Why consistency is tested rather than eyeballed

The INS/GNSS filter shipped with a one-step misalignment between the mechanised
state and the truth used to form measurement residuals. In straight flight the
velocity vector is not rotating, so the residual vanished and everything looked
correct. Under turn it injected a systematic 3 m/s velocity residual against
0.15 m/s of measurement noise, and the filter absorbed it into heading and bias,
finishing thirty times overconfident.

Four wrong hypotheses were investigated before the real cause was found. A NEES
test would have pointed at it immediately. Any component that publishes an
uncertainty should carry one.
