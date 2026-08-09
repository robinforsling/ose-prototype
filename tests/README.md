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

Navigation is split across four files, one per component (ADR 0009):

`test_navigation_sensors.py` checks that each resource-layer sensor's
declared sigma is honest — sample mean and standard deviation against many
draws — plus the IMU bias's Gauss-Markov steady state and GNSS
denial/restoration.

`test_navigation_state_estimator.py` checks the subsystem-layer filter: NEES
consistency across several seeds, that the published covariance is positive
semi-definite, the observability structure (heading variance must not shrink
before the first turn, and must collapse during it), `ast`-parses the module
to confirm it cannot see truth, and replays a recorded measurement stream
into a fresh estimator to confirm it is a pure function of that stream.

`test_integrated_navigation_unit.py` checks the resource-layer black-box
stand-in: protocol conformance and that its declared uncertainty is honest,
nothing about navigation performance (see its docstring and ADR 0009 for
why).

`test_clock.py` and `test_time_estimator.py` are the same pattern applied to
the platform clock (ADR 0010): declared sigma honesty and the drift's
Gauss-Markov steady state for the sensor; NEES consistency, the truth
boundary, and replay determinism for the estimator, plus that
`platform_time_s` is exactly the running sum of readings and its uncertainty
never decreases — there is no correction source yet, so nothing should ever
look more confident than dead reckoning warrants.

`test_vehicle_guidance.py` (ADR 0011) is where enforcement is finally
exercised: `test_vehicle.py`'s two tests above check that the vehicle
itself does *not* clip an inadmissible command; `test_reports_saturation_
when_setpoint_exceeds_envelope` checks that guidance does, and that the
clipping comes back as a visible `Saturation` finding rather than being
absorbed silently. Also checks the truth boundary (guidance only ever
touches `OwnStateEstimate`) and closed-loop convergence to a commanded
heading and speed.

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
