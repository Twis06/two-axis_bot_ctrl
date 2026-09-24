# Task 3 — Decide whether learning belongs

**Decision: reject the tested adaptive candidate for this submission and retain the frozen deterministic baseline.** This rejection is now based on the registered matched comparison, not on missing evidence. The result applies to the frozen estimator configuration and feed-forward-only adapter; it does not claim that every future learning design is useless.

The candidate was a two-parameter effective gravity-like residual,

```text
tau_extra(q) = theta_s sin(q) + theta_c cos(q)
```

It uses only delayed measured roll/yaw, measured current, the active current limit, drive mode, saturation status, and physical timestamps. It has no access to the reference, requested current, future yaw plan, integral state, true plant parameters, or simulator configuration. The L3 adapter routes its correction to feed-forward only and keeps the governor and safety envelope on the deterministic model.

## What was implemented and audited

The standalone L1 kernel is in [`ctrl/payload_estimator.py`](../ctrl/payload_estimator.py), with behavioral tests in [`tests/test_payload_estimator.py`](../tests/test_payload_estimator.py). The implementation has these fixed limits:

| Contract | Frozen rule |
|---|---|
| Training data | Healthy, settled, yaw-stationary 250 ms dwell; at least 20 samples; no gap above 15 ms |
| Identification | At least 30 aggregates, 60° roll span, cosine span 0.25, normalized Gram gates, two-parameter ridge fit |
| Update/worker | At most 50 Hz; one pending job; modeled completion delay 2–8 ms; results older than 100 ms are discarded |
| Bounds | Coefficient norm ≤0.40 N·m; applied correction ≤0.20 N·m; coefficient slew ≤0.10 N·m/s |
| Health | Correction is zero on stale, invalid, faulted, saturated, mismatched, under-covered, or expired data |

The independent review found three issues in the first L1 implementation. They were fixed and covered by regressions: hard fault/saturation/invalid-limit status now invalidates even when a packet reuses an old physical timestamp; raw dwell storage is capped at 256 samples and enters `input_overload`; and a finite physical timestamp is reserved before a nonfinite payload can be replayed as valid input.

The L2 audit is [`exp/task3_estimator_audit.py`](../exp/task3_estimator_audit.py), with results in [`task3_estimator_audit.md`](task3_estimator_audit.md) and [`task3_estimator_audit.json`](task3_estimator_audit.json). It used the public estimator API, 500 Hz timestamped replay-like observations, and seeds 201–205. All 60 runs passed the structural gate.

| Audit case | Result across five seeds | Meaning |
|---|---|---|
| Ideal quantized sensing | 5/5 usable; coefficient error below 0.000001 N·m | The kernel can identify the chosen effective law when assumptions hold. |
| Bounded 0.05 N·m disturbance | 5/5 usable; maximum coefficient error 0.054 N·m | Disturbance is absorbed as an effective residual; it is not physical payload identification. |
| Constant disturbance | 5/5 usable; maximum coefficient error 0.058 N·m | Angle-correlated bias is non-identifiable from gravity. |
| ±0.10 A current bias | 5/5 usable; maximum coefficient error 0.019 N·m | Calibration bias shifts the learned coefficients. |
| Narrow coverage / 150 ms dwell | 0/5 usable in each case | Required abstention works. |
| Moving trajectory | 5/5 retained usable models; no dynamic training violation | Motion does not create new training weight. |
| 60 ms feedback gap | All seeds disabled during the gap and recovered only after a new healthy dwell | Pending work cannot re-enable a stale model. |
| Duplicate/replayed packets | All seeds retained the same learned result | Fresh receipt time does not create new physical information. |
| Payload change | All seeds detected `residual_mismatch` and disabled | Old training is not silently averaged through a load change. |
| Pending fault packet | All seeds disabled the pending result and later recovered through the normal ramp | Fault status takes precedence over de-duplication. |
| Training stopped for >30 s | All seeds expired the model | Receipt traffic cannot refresh old training data. |

## Why this candidate is rejected

The L1/L2 audit establishes a bounded estimator kernel, not a useful controller improvement. The registered L4 comparison then tested the exact frozen L1 configuration through the feed-forward-only adapter on the same paired loads, limits, motions, and seeds as the deterministic comparator.

The adaptive candidate achieved **0.0% median paired reduction** against `int1`, with a range of 0.0–78.7%. Only **37/50 held-out runs (74%)** obtained a usable estimate, below the preregistered 80% minimum. It lost no comparator-completed sequences and introduced no watchdog trips, suspensions, or request rejections, so the failure is primarily insufficient and unreliable benefit rather than a demonstrated safety regression.

The [known-load ceiling](<repo>/report/task3_ceiling_numbers.md) shows why the comparison was necessary. Under the frozen protocol, the feed-forward-only diagnostic oracle reduced the primary metric by **76.9% median paired** against `int1` and completed **50/50** held-out sequences. The oracle uses true load and is not deployable; it shows that the two-parameter load law can matter, while the tested online estimator did not realize that potential reliably enough.

The explicit `INF-P` case remains a separate capacity boundary: its 0.41 N·m static load exceeds the 0.336 N·m derated capacity, so no bounded feed-forward correction can make that request physically holdable. This does not explain away the feasible-load comparison.

The L3 adapter is in [`ctrl/adaptive.py`](../ctrl/adaptive.py) with focused contract tests in [`tests/test_adaptive.py`](../tests/test_adaptive.py). A future redesign could be reconsidered only with a new preregistered experiment that improves estimate availability and meets the same benefit, waypoint, current-limit, and fault-containment gates. The oracle ceiling alone is not adoption evidence.

## Verification and provenance

The focused estimator suite passed **15 tests**; the L3 adapter contract suite passed **7 tests**; the full repository suite passed **203 tests** with zero failures. The matched L4 packet contains **357 manifested runs**, including 50 paired adaptive held-out cases. The published L2 packet contains 60 rows and zero structural violations. The final estimator source hash is `cefc2b5169bfd66593dbb525ec8f143f0c829dd51c53611e64a9784060aab392`.

All estimator, ceiling, audit, and matched adaptive results are simulated or replay-like. They are not hardware observations, payload-mass identification, uncertainty intervals, or safety certification. The deterministic baseline at fingerprint `7d857df507c389c9` remains the final controller for this assessment; Task 5's prospective simulation is recorded separately and hardware confirmation remains pending.
