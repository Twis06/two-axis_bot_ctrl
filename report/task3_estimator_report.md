# Task 3 L1 — standalone payload-estimator execution record

## Scope and result

This packet implements only the standalone `PayloadEstimator` kernel and its
behavioral tests.  It does not edit or invoke the controller, governor,
supervisor, simulator, experiment scripts, baseline reports, or Task 3 adoption
claim.  It therefore provides no closed-loop tracking or learning-benefit
result.

`ctrl/payload_estimator.py` supplies an immutable `PayloadObservation`,
immutable snapshots/conditioning diagnostics, and a bounded estimator with the
public `reset`, `observe`, `tick`, `snapshot`, and `correction` operations.  It
also exposes `raw_prediction(q, t_now)` so a future adapter can log the raw
model torque beside the clipped feed-forward correction.

The learned target is exactly

```
y = 0.140 * i_meas - 0.120 * sin(q)
tau_extra(q) = theta_s * sin(q) + theta_c * cos(q)
```

and only delayed measured current appears in that calculation.  The kernel has
no controller state, requested current, reference acceleration, yaw plan,
plant object, or simulator configuration.

## Fixed behavior implemented

- A physical timestamp must be strictly increasing.  Ordinary duplicate and
  out-of-order packets are ignored without refreshing receipt health; hard
  fault/saturation/invalid-limit status takes precedence over de-duplication
  and immediately disables application and discards pending work. Finite
  timestamps are reserved before a nonfinite payload can be replayed.
- Training uses a sliding 250 ms healthy dwell with at least 20 samples, no
  gap above 15 ms, and roll/yaw ranges no larger than two encoder counts.
  Raw dwell storage is capped at 256 samples; over-rate input enters
  `input_overload`, clears the dwell, and requires normal recovery.
  Healthy motion and near-limit nonsaturated current preserve an existing
  correction but cannot add a dwell aggregate.
- At most one aggregate and one modeled fit submission may occur per 20 ms.
  A seeded 2--8 ms delayed worker snapshot is nonblocking and contains only
  aggregates received by submission time.  Results more than 100 ms late or
  from an invalidated generation are discarded.
- Fitting is an explicit normalized 2-by-2 ridge least-squares calculation.
  It requires 30 aggregates, 60 degrees roll span, 0.25 cosine-feature span,
  and both the 0.01 normalized-Gram eigenvalue/ratio gates.  The candidate is
  projected to a 0.40 N m coefficient norm before residual RMS is checked
  against 0.065 N m.
- The applied model has a 0.20 N m torque cap.  Coefficient publication is
  limited to 0.10 N m/s, and the first publication is measured from its first
  eligible fit submission rather than process start.
- Planner clarification implemented: fresh-dwell mismatch compares against the
  latest full, physically projected accepted fit before adding the fresh
  aggregate.  It does **not** compare against the intentionally slew-limited
  applied coefficients.  A mismatch clears the relevant training history,
  invalidates the pending job and application, and retains the prior full fit
  only as diagnostics.
- Planner clarification implemented: fault, stale, saturation, or invalid-input
  recovery resets the *applied* coefficient/rate epoch to zero.  A retained
  full fit and retained training may support a later re-fit only after a new
  continuous dwell and normal coverage/health gates; disabled time cannot buy
  publication slew credit or restore old applied torque in one step.
- Estimate usability also requires latest measurement age at most 15 ms, latest
  valid receipt age at most 20 ms, and newest accepted-job training data age at
  most 30 s.  Snapshot state distinguishes applied coefficients from the full
  fit used for mismatch diagnostics.

## Test-first record and verification

The three assigned files were absent before work began.  The first red run was:

```text
uv run --quiet --with numpy python -m unittest tests.test_payload_estimator -v
ModuleNotFoundError: No module named 'ctrl.payload_estimator'
```

The final focused verification was:

```text
python3 -m unittest tests.test_payload_estimator -v
Ran 15 tests
OK
```

Those behavioral tests cover the required synthetic three-angle convergence,
single-angle abstention, moving/yaw and quantized dwell handling, timestamp and
nonfinite rejection, fault/saturation/staleness invalidation, worker latency and
cadence, parameter/torque/rate bounds, moving application plus 30 s expiry,
recovery dwell gating, fresh-dwell residual mismatch, and the recovery
restart-from-zero ramp ruling.

The final repository verification was:

```text
python3 -m unittest discover -s tests -v
Ran 196 tests
OK
```

The L1 suite is standalone; the repository-wide result includes the accepted
Task 2 and Task 4 regression suites. No controller integration or closed-loop
benefit is claimed by this packet.

## Source record

Before work: `ctrl/payload_estimator.py`, `tests/test_payload_estimator.py`,
and this report did not exist.

Final SHA-256 values for the implementation and behavioral-test sources:

```text
cefc2b5169bfd66593dbb525ec8f143f0c829dd51c53611e64a9784060aab392  ctrl/payload_estimator.py
ed7ce87fe62d81d60e6f81a387c6dc8b0e6069ef231ebf4fe1d40750f59d573e  tests/test_payload_estimator.py
```

## Limits for review and L2

This is an effective gravity-like residual estimate, not an identified payload
mass, center of mass, uncertainty interval, or safety-envelope expansion.
Angle-correlated friction, current bias, unmodeled disturbance, and residual
motion can imitate the fitted terms.  Dwell, coverage, residual, timestamp,
and bounds gates are validity checks rather than proof that this model is
correct.  L2 audited those biases, partial coverage, data loss, payload
changes, and abstention; see [the audit packet](task3_estimator_audit.md).
The audit passed structural gates but did not establish a closed-loop benefit.
