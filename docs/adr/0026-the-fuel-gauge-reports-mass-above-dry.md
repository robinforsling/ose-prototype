# 0026 — The fuel gauge reports mass above dry, and the manager reconciles it

**Status:** Accepted. Amended by ADR 0027, which renames the field this record
kept and takes the interface to v2.

## Context

Two components disagreed about what "fuel" means, in two files, with neither
referencing the other.

`FuelGauge.sample()` computed `true_state.mass_kg - self.mass_dry_kg` and
published it as `fuel_remaining_kg`. `VehicleManager` decomposes mass as
`dry + payload + fuel`, and corrected its fuel state on that reading directly:
`innovation = m.fuel_remaining_kg - self._fuel_kg`.

The two definitions agree only when payload is zero.

With payload, the gauge's reading includes it, the filter drives the fuel state
toward `payload + fuel`, and the mass sum then adds the payload a second time.
Measured on a fighter with 500 kg of stores:

```
dry 12000  payload 500  fuel 4000   TRUE TOTAL 16500
gauge reports as fuel:  4502.5
believed fuel            4500.3   (true 4000.0)
believed mass           17000.3   (true 16500.0)   error +500.3
stated sigma                1.43 kg  ->  351 sigma
```

351 sigma wrong while reporting 1.4 kg of confidence — the same failure mode as
the INS/GNSS bug in CLAUDE.md, in a different filter.

It survived because **every fixture in the repository set
`payload_mass_kg=0.0`**, which is exactly where the two definitions coincide.
The calibrated consistency test — ensemble ANEES against a two-sided band, the
instrument built for precisely this class of error — ran only on a clean
aircraft and reported a healthy 1.00 throughout. This is the navigation bug's
lesson repeating: *excite the thing the fixture holds constant.* A turn there,
a payload here.

Neither component was individually wrong. `sensing.fuel.v1` documented "true
mass less dry mass" and delivered it. The manager documented `dry + payload +
fuel` and computed it. The defect lived in the seam, which is where a reader
checking either file alone would never look.

## Decision

**A fuel gauge reports mass above dry, and never knows about payload.** A gauge
measures a tank, not a loadout. Giving it `dry + payload` would make an
equipment component depend on a platform configuration decision.

**The vehicle manager subtracts the payload it believes in** before treating
the reading as fuel:

```python
observed_fuel_kg = m.mass_above_dry_kg - self.par.payload_mass_kg
innovation = observed_fuel_kg - self._fuel_kg
```

It is the component that owns the mass decomposition, so it is the component
that reconciles the two meanings.

**The consistency test runs loaded as well as clean.** `_fly_one` takes a
`payload_kg`, and `test_the_filter_is_consistent_through_the_run` is
parameterised over `[clean, loaded]`. Against the unfixed code the loaded cases
report ANEES 13,979 against an expectation of 1; the clean cases still pass, so
the new parameter is what catches it rather than a change in the old path.

## Consequences

Payload becomes a **runtime** quantity rather than a construction-time
constant, which is what this decision is really for. A payload manager
publishing the current mass as stores are released will feed the vehicle
manager, and nothing about the gauge changes. Had the gauge been configured
with `dry + payload` instead — the smaller-looking fix — that constant would be
stale from the first release onwards, and stale in the direction that makes a
platform believe it is heavier than it is.

**`fuel_remaining_kg` was a misleading field name**, and this record kept it,
reasoning that a version increment was more than a clarification needed. That
was wrong, and the sentence that followed it here is why: *a reader who trusts
the field name and not the docstring can still make the original mistake.* The
name is not incidental to the defect, it is the defect — a consumer reading
`fuel_remaining_kg` has no reason to suspect it is not fuel remaining, and
therefore no reason to read the docstring saying so. ADR 0027 renames the
fields to `mass_above_dry_*` and takes the interface to `sensing.fuel.v2`.

**The subtraction uses believed payload, which is currently exact.**
`MassEstimate` documents payload as "configuration, exact", so it contributes
no uncertainty. When a payload manager publishes an estimated payload that
stops being true, and the covariance will have to grow a payload term. Nothing
here anticipates that beyond leaving the subtraction in the component that
would own it.

**Two definitions still exist**, reconciled in one place rather than unified.
The alternative — a fuel state in `VehicleState`, so truth carries fuel and no
constant is needed anywhere — is the physically right answer and was not taken:
it changes the state vector, the integrator and the Jacobian, for a bug one
line fixes. It remains the better long-term shape.

## References

- ADR 0015 — the vehicle manager owns believed mass, which is why it reconciles
- ADR 0025 — the descriptor gap this gauge also sits in, its dependency on the
  vehicle being carried as a bare float
- CLAUDE.md, "Testing philosophy" — excite the system, do not just run it
