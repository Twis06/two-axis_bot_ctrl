# Task 3 — Decide whether learning belongs

**Decision: reject the tested adaptive candidate for this submission and retain the frozen deterministic baseline.** This rejection is based on the registered matched comparison, not on missing evidence. *Ordering evidence:* the protocol's loads, seeds, metric and 10% gate are cited in commit `9554de8` (10:38), before the candidate was run. The plan file itself was first committed together with the adaptive results (`35a8aae`/`4ae154a`, 16:05). The result applies to the frozen estimator configuration and feed-forward-only adapter; it does not claim that every future learning design is useless.

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

### What the comparison found

The registered adoption gate ([`exp/task3_gate.py`](../exp/task3_gate.py)) returns **fail** (Simulated; 50 held-out pairs against `int1`; scoring metrics version 4A.4, phase-aware completion):

| Gate criterion | Status | Value |
|---|---|---|
| Median paired reduction ≥10% | **fail** | 0.0% (range 0.0–78.7%) |
| ≥80% of runs ever obtain a usable estimate | **fail** | 37/50 (74%) |
| No comparator waypoint or completion lost | pass | 0 lost |
| No ordinary feasible case more than 20% worse | pass | worst +0.0% |
| Pre-clamp current command within the active limit | pass | 0.000 A excess |
| No new fault, suspension, rejection or lockout | pass | 0 |
| No learned correction applied while unusable | pass | 0 samples |
| No benefit credited to slower motion | pass | 0 |
| Supplementary challenges | **incomplete** | payload change not executed |
| Registered grid complete | pass | 50/50 held-out, 96 challenge cells |

**The median hides a split outcome.** The frozen estimator's data gates are rarely met during the calibration dwells:

- **Late or never:** it first becomes usable at a median of 16.6 s (over the 37 runs where it ever does; the test phase is 9–19 s), and its median usable share within the scored dwells is 0%.
- **When it is active, it helps:** it is active during scoring in 22/50 runs, 20/50 pairs improve, and 8 improve by 42–79%.

The candidate therefore fails on availability rather than on the value of the correction.

**Why the median is exactly 0.0%, not "about zero".** The candidate *is* the frozen `int1` controller plus the learned feed-forward term (`AdaptiveController` subclasses `BaselineController` at the same integral rate), run on the same seed, load and noise draw. When the learned correction is zero, the two runs are the same computation, so the primary metric is bit-identical and the paired reduction is exactly 0.

| Adaptive vs `int1`, 50 held-out pairs | Pairs |
|---|---:|
| Bit-identical: the correction never changed a scored command | **30** |
| — estimate never usable | 13 |
| — first usable only at 18.3–21.6 s, at the end of the test. In 2 of these, usable inside the last scored dwell, but at constant reference, where the bumpless transfer (`integ -= bump`) cancels the change | 17 |
| Improved | 20: twelve by 0.1–5.4%, eight by 42–79% |
| Worse | **0** |

- **Median:** with 30 of 50 values exactly zero, the median is exactly 0.0%.
- **Mean (secondary, not registered):** the mean paired reduction is 9.5%.
- **Takeaway:** the candidate never hurt, and helped substantially when it was on in time. It was simply off in most runs. The gate uses the median by registration, and the change that would matter is earlier or more reliable availability, not a larger correction.

**Completion.** Both `int1` and the candidate complete 40/50 held-out test sequences. The 10 incomplete runs of each are all at load (−0.06, +0.14). They reach every waypoint but overshoot the ±65° range by 5.65–6.89° (by seed), against the 5° allowed; the peak is on the +65° test move. They are not failures to reach a target. The phase-aware scorer ([R1](packets/R1.md)) is stricter than the old one in one respect: an intermediate target reached only after the governor clock has left its window is not credited. No L4 verdict depends on this.

**Safety scope.** No safety criterion failed. That holds only in the evaluated conditions:

- **Held-out set:** 5 loads × 2 limits × 5 seeds.
- **Supplementary challenges:** registered yaw B and C, a 60 ms feedback outage and a 3.2 → 2.4 A derate, at 9, 12.5 and (late-onset amendment) 17.5 s; loads (0, 0.18) and (0.06, 0.14); seeds 201–203.
- **What those runs showed:** there was no limit, fault or waypoint regression. The gate compares every event type per pair, not only watchdog trips. The correction was usable after onset in **23 of 48** candidate challenge runs, and in none of the 9 s-onset yaw B/C runs; the gate reports this per onset set. The 23 is an upper bound on meaningful exercise: in two runs the correction never exceeded 0.0004 N·m, and in one it was usable only 0.3% of the post-onset time. Most challenge runs therefore exercise the baseline. See [challenge numbers](task3_challenges_numbers.md).
- **Not executed:** payload change mid-run, which the frozen simulator cannot represent without a harness patch.
- **Configuration:** all results are for the exact frozen configuration, `AdaptiveController(est_seed = seed)` with the L1 stationarity gate.

The [known-load ceiling](task3_ceiling_numbers.md) shows why the comparison was necessary. Under the frozen protocol, the feed-forward-only diagnostic oracle reduced the primary metric by **76.9% median paired** against `int1` and completed **50/50** held-out sequences. The oracle uses the true load and is not deployable. It shows that the two-parameter load law can matter; the tested online estimator did not realize that potential reliably enough. (The full oracle, which also informs the governor, never starts the path in 15 runs at 2.4 A and is reported only as a diagnostic.)

The explicit `INF-P` case remains a separate capacity boundary: its 0.41 N·m static load exceeds the 0.336 N·m derated capacity, so no bounded feed-forward correction can make that request physically holdable. This does not explain away the feasible-load comparison.

The L3 adapter is in [`ctrl/adaptive.py`](../ctrl/adaptive.py) with focused contract tests in [`tests/test_adaptive.py`](../tests/test_adaptive.py). A future redesign could be reconsidered only with a new preregistered experiment that improves estimate availability and meets the same benefit, waypoint, current-limit, and fault-containment gates. The oracle ceiling alone is not adoption evidence.

## Verification and provenance

- **Test counts:** the final suite has **266 tests**. An earlier snapshot at `039f83b` passed 255 tests; that count is historical. The focused suites cover the estimator, adapter contract, phase-aware scorer and adoption gate. Passing tests show the code does what the tests check; they do not validate the experiment design.
- **Evidence:** the L4 packet contains **357 manifested runs** and the challenge packet **96**, both regenerated from committed sources by [R4](packets/R4.md). The published L2 packet contains 60 rows and zero structural violations.
- **Corrections:** these are recorded in packets [R1](packets/R1.md) (scoring) and [R3](packets/R3.md) (gate and challenges).

All estimator, ceiling, audit and matched adaptive results are simulated or replay-like. They are not hardware observations, payload-mass identification, uncertainty intervals or safety certification. The deterministic baseline at fingerprint `7d857df507c389c9` remains the final controller for this assessment.
