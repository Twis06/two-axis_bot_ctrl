# Task 3 — Bounded payload adaptation and adoption decision

**Owner:** primary agent plans and reviews; execution agents implement and test. **Execution model:** gpt-6-luna, maximum reasoning effort, bounded assignments with reports and independent review. Use Terra at maximum effort as a fallback if Luna cannot execute. Do not silently escalate to a more expensive implementer.

**Goal:** Determine whether a small adaptive gravity-load model provides a measurable advantage over fixed feed-forward and a reasonably retuned integrator on untuned payloads, without weakening deterministic safety or using unavailable information.

**Architecture:** Learn two gravity-like residual coefficients from settled, unsaturated, yaw-stationary observations. Apply only bounded feed-forward correction initially. Keep the governor's admitted envelope and drive safety independent of the learned estimate. Do not implement neural networks, reinforcement learning, or online gain tuning.

**Requirements:** assessment Task 3, [current answer](../../report/task3.md), and [execution gates](../../EXECUTION_PLAN.md). This focused plan allows estimator development while the baseline is unfinished; it does not waive Task 2 acceptance before comparative claims.

## 1. Current evidence and the question being tested

The latest working baseline has nominal D/E governed RMS 4.05°/4.30° at mean path-clock rates 83%/50%. Its uncertainty trials have fault events in 5/20 D and 7/20 E cases. A known governor failure and loaded recovery questions remain. These are reasons to investigate model mismatch, not proof that learning is needed.

**Hypothesis:** Reusing a learned angle-dependent payload torque can reduce post-move error and saturation relative to an integral controller that must rebuild the load correction at each new pose.

**Competing explanations:** insufficient physical torque, reference-governor defects, inaccurate motor constant, friction, derivative noise, or yaw coupling errors. Adaptation must not be credited for fixing those through privileged plant access or relaxed requests.

**Scope limit:** Initial learning occurs during naturally available dwells. The controller does not generate extra excitation without a declared calibration request. It may abstain if the operating trajectory lacks sufficient informative dwells. Such abstention is an acceptable result.

## 2. Gates and work packets

| Packet | Output | May execute now? |
|---|---|---|
| L1: estimator kernel | Standalone estimator, behavioral tests, report | Yes; no baseline/simulator edits |
| L2: synthetic/replay audit | Causality, bias and abstention evidence; frozen estimator configuration | Yes after L1 review; no closed-loop benefit claims |
| L3: controller integration | Feed-forward-only adapter and fault/staleness tests | Only after Task 2 governor/recovery acceptance |
| L4: matched comparison | Frozen protocol, train/held-out results and ablations | Only after L3 review and metrics/provenance gate |
| L5: keep/drop decision | Final Task 3 answer with evidence and limitations | After L4; “do not use learning” is a valid completion |

Agent file ownership is exclusive. Other work is occurring in shared controller files; do not overwrite it. Record source hashes at the beginning/end of an experiment. A changing baseline invalidates paired comparisons and requires a new freeze, not an undocumented mixture of versions.

## 3. Model and observation contract

Represent **additional** gravity-like torque as:

`tau_extra(q) = theta_s * sin(q) + theta_c * cos(q)`.

The nominal `0.120 sin(q)` term stays in the baseline and is subtracted from the observation target. During a validated dwell with yaw stationary, form:

`y = 0.140 * measured_current - 0.120 * sin(measured_roll)`.

Use measured current, not requested current, integral state, commanded acceleration, future yaw values, or true plant parameters. The dwell gate intentionally avoids estimating acceleration and subtracting noisy derivatives. Residual motion/friction, current calibration and bounded disturbance still bias this approximation.

This is an **effective gravity-like residual estimate**, not an identified payload mass or a proven physical center of mass. A persistent disturbance correlated with angle can be indistinguishable from gravity. Report that limitation, do not invent confidence intervals implying otherwise.

### Kernel API

The L1 executor owns the precise implementation of these public types in `ctrl/payload_estimator.py`:

- Immutable `PayloadObservation`: `t_meas`, `t_received`, `q`, `qy`, `i_meas`, `i_limit`, `mode`, `saturated`.
- `PayloadEstimator(seed=0)`: nominal constants/settings only; no `SimConfig` or plant object.
- `reset()` clears data, pending jobs, coefficients and health.
- `observe(observation)` ingests at most one newly timestamped physical sample. It must not fit or block waiting for a worker.
- `tick(t_now)` polls completion and schedules at most one modeled job at each allowed update; never sleeps. Results use only observations received by submission time.
- `snapshot(t_now)` returns immutable state with coefficients, usability, reason, last training time, last publication time, last observation time, accepted sample count, and conditioning information.
- `correction(q, t_now)` returns zero when unusable; otherwise a finite bounded torque for feed-forward only.

Use named status/reason strings rather than magic integers. Unit tests may use synthetic observations; production classes may not contain test-only hooks.

## 4. Fixed initial estimator design

These settings are engineering assumptions for this experiment. Freeze them before held-out evaluation; change them only using tuning data and document the reason.

| Setting | Initial value / rule |
|---|---|
| Physical sample timestamp | Strictly increasing; duplicates/out-of-order samples rejected without adding training weight |
| Sensor freshness | Arrival age 0–15 ms; at application, latest measurement age ≤15 ms and latest valid receipt age ≤20 ms |
| Dwell window | 250 ms of valid observations; at least 20 samples; no inter-sample gap >15 ms |
| Stationary roll/yaw | Each position's range in that window ≤2 encoder counts, with count = 2π/16384 rad |
| Eligible state | Drive normal, saturation false, finite values, positive limit, abs(measured current) <95% of active limit throughout the window |
| Learning cadence | At most 50 Hz; at most one new dwell aggregate per 20 ms |
| Learning buffer | Most recent 600 accepted dwell aggregates; each target is the window mean current/feature residual, not an instantaneous spike |
| Minimum coverage | At least 30 aggregates, roll span ≥60°, cosine-feature span ≥0.25, and smallest/largest eigenvalue of normalized feature Gram matrix ≥0.01 |
| Fit | Two-parameter ridge least squares; ridge = 1e-6 times identity after normalizing the Gram matrix by sample count |
| Parameter bound | Euclidean coefficient norm ≤0.40 N·m; projection is an experimental bound, not proof that the true load lies inside it |
| Fit-consistency gate | RMS residual ≤0.065 N·m; failure retains no newly trusted fit and is reported |
| Model uncertainty | Do not publish statistical confidence or an expanded safe envelope; excitation and residual gates are validity checks, not guarantees |
| Worker timing | Seeded uniform 2–8 ms simulated completion delay; at most one pending job; no synchronous wait |
| Late job | Discard if published more than 100 ms after submission, or if its health generation was invalidated by a fault/stale/invalid sample |
| Estimate age | At most 30 s since the newest training measurement in its accepted job; publication cannot refresh old data artificially |
| Application bound | abs(extra torque) ≤0.20 N·m |
| Coefficient publication rate | Norm of coefficient change ≤0.10 N·m/s times elapsed time since the prior publication; first publication uses elapsed time since its first eligible fit submission, not time since program start |

Coefficient updates and result availability are distinct. A new estimate must not become usable before its worker completion time. Repeated evaluation calls at the same timestamp must not increment counters, advance rate limits or schedule extra jobs.

### Health and stale behavior

- Movement alone suspends **learning**, while a previously accepted estimate may remain applicable up to the estimate-age limit if sensing remains healthy.
- Stale/missing input, nonfinite data, fault mode or saturation makes correction unusable immediately. Discard/invalidate pending results and clear the dwell window. Do not let an old pending job re-enable correction after a fault.
- On recovery, require a new continuous 250 ms healthy observation window before reuse/publication. Do not count the lost interval as a settled pose.
- Retaining the full-fit coefficient value for diagnostics is allowed while unusable; `correction` still returns zero. Fault/stale invalidation resets the applied-coefficient ramp to zero. Its next publication starts a new ramp from the first eligible recovery fit submission, with no slew credit accumulated while disabled. Integration must handle the total-command transition on both disable and enable.
- Large residual mismatch during new valid dwells must invalidate the learned correction rather than silently retain it for the full 30 s. Compare the fresh dwell mean residual against the last full accepted, physically projected fit with the same 0.065 N·m threshold, before adding it to any refit or long training buffer. Do not compare against intentionally slew-limited applied coefficients: that would falsely reject the initial ramp. Recheck residual after coefficient projection. On detected mismatch, invalidate pending results/use and clear/restart training history; require normal coverage and health gates for a replacement fit. L2 will test payload changes and identify whether this policy is sufficiently responsive.

**Known restriction:** the application torque cap may truncate larger gravity loads. Report both raw model prediction and applied correction so clipping cannot be mistaken for convergence. Safety depends on the deterministic current/supervisor constraints, not the coefficient bound alone.

## 5. L1 — Standalone estimator implementation

**Allowed writes:** `ctrl/payload_estimator.py`, `tests/test_payload_estimator.py`, `report/task3_estimator_report.md`. No edits to baseline, governor, supervisor, plant, metrics, other tests, dependency files, or existing report claims.

**Required behavior tests:**

1. Construct synthetic settled observations at −50°, 0°, +50° with known extra coefficients `(0.08, 0.16)` and current derived from nominal plus extra gravity. Check convergence within 0.01 N·m per coefficient after the allowed publication ramp.
2. A single-angle hold remains unidentifiable and produces no usable learned torque, even with thousands of repeated observations.
3. Moving roll or moving yaw does not add training observations; quantized dwell jitter within the declared two-count range is handled consistently.
4. Duplicate, old, future-dated and nonfinite observations cannot add training weight or refresh health falsely.
5. Saturation/fault/staleness disables correction and prevents a pre-fault pending job from publishing as valid.
6. Updates occur no faster than 50 Hz; completion is delayed 2–8 ms; duplicate ticks cannot increase the update rate.
7. Parameter norm, torque cap and coefficient-change-rate limits hold for adversarial current inputs as well as an ordinary learned model.
8. A healthy moving trajectory may use a learned model without learning from dynamic residuals; coefficients expire when the underlying training data age exceeds 30 s.
9. Recovery requires a fresh complete healthy window; a single fresh packet cannot reinstate an old correction.
10. Residual-inconsistent fresh data is rejected. A valid fit must not be manufactured by clipping huge observed current or replacing invalid observations with zeros.

The executor should begin with failing tests for the public contracts, implement the smallest bounded estimator, and run both its tests and the whole repository suite. Pre-existing failures from concurrent work must be identified separately, not fixed by this agent. Record exact test commands and results. Do not claim closed-loop improvement.

## 6. L2 — Estimator audit before controller integration

**Files:** a standalone audit script under `exp/`, new Task 3 audit outputs. Keep the L1 interface stable unless review identifies a defect.

Evaluate: encoder quantization; bounded slow disturbance including a one-sided bias; current measurement bias; partial angular coverage; delayed/out-of-order feedback; data loss while a job is pending; payload changes; and insufficient dwell time. Use separate deterministic seeds for tuning and audit.

The audit must distinguish:

- coefficient accuracy when identification assumptions hold;
- prediction error at unseen angles;
- abstention frequency when assumptions fail;
- time to learn and time to disable a mismatched estimate;
- modeled worker latency versus actual measured calculation time.

**Gate:** all structural safety/causality tests pass. Biased-disturbance cases need honest error/abstention reporting, not necessarily accurate physical coefficients. If they can produce large harmful-looking predictions while passing every validity check, revise the design or reject the candidate before integration.

## 7. L3 — Feed-forward-only integration after the Task 2 gate

The current `load_model.torque` hook feeds both the governor and feed-forward. **Do not attach this estimator to that hook unchanged.** It would change request admission and invalidate the intended comparison.

The integration executor must provide a small reviewed adapter separating learned compensation from governor feasibility. Required behavior:

- estimator receives only delayed observable samples and current saturation/fault state;
- worker polling and health invalidation occur even when the host returns early for fallback;
- learned correction affects feed-forward only;
- anti-windup uses the final total torque/current command after learned correction;
- adding learned feed-forward does not double-count an existing integral load correction: implement and test a bumpless transfer policy that preserves total commanded torque at the update instant;
- current clipping and local faults remain authoritative;
- a bounded/stale/invalid estimator cannot cause NaNs, bypass current limits, modify safety thresholds or expand the reference envelope;
- disabling the estimator reproduces the frozen baseline within numerical tolerance.

Do not promise improved performance merely because the module is integrated. If a benefit requires learning to influence the governor, return that finding to the planner as a separate design decision requiring uncertainty-aware admission; it is outside this initial candidate.

## 8. L4 — Preregistered matched experiment

### Comparators

1. Frozen deterministic baseline with nominal feed-forward and its existing integrator.
2. Best deterministic integral setting from tuning-only candidates `0.5x`, `1x`, `2x` the existing integral rate, retaining anti-windup and verifying applicable loop/fault constraints.
3. Adaptive feed-forward candidate, with all other baseline settings unchanged.
4. Diagnostic oracle using the known extra gravity function, explicitly labeled unavailable to a deployable controller. It is a ceiling/diagnostic, never a fair deployable competitor.

### Motion and data split

Use finite minimum-jerk pose sequences, not only periodic sine RMS. Give every deployable comparator the same initial calibration trajectory and charge its elapsed time to the total budget. No candidate receives free offline fitting on the held-out payload.

- Tuning payloads: additional coefficients `(0,0.08)` and `(0.04,0.12)` N·m; nominal limits and transport; seeds 11, 12, 13.
- Held-out load coefficients: `(0,0.10)`, `(0,0.18)`, `(0,-0.18)`, `(0.06,0.14)`, `(-0.06,0.14)` N·m. These are explicit load-moment test parameters, not masses inferred from the brief. The experiment must represent both sine and cosine shifts without exposing them to controllers. A negative cosine moment must be represented by shift direction, not a negative physical mass or negative added inertia.
- Same calibration sequence for all: 0° → −50° → 0° → +50° → 0°, 1 s minimum-jerk moves and 1 s dwells, plus a 1 s initial dwell.
- Test sequence: +30°, −30°, +65°, −65°, 0°, each a 1 s minimum-jerk move followed by a 1 s dwell. Primary learning occurs only during eligible dwells; online learning continues identically according to the frozen policy.
- Held-out seeds: 101–105, paired across comparators.
- Primary operating matrix: the five held-out loads at 3.2 A and 2.4 A, nominal yaw stationary. Add B/C yaw disturbance, a 60 ms feedback-only outage, a mid-run derate, and a payload change as separately reported safety/generalization challenges.
- Include an explicitly unholdable load case as a rejection check, not a tracking-performance failure to optimize away.

The executor may adjust the proposed finite trajectory only before registration if a feasibility calculation exposes a problem. Record the reason. No tuning against the held-out outcomes.

### Metrics and adoption criteria

Primary metric: governed-reference RMS during the **first 0.5 s of each test dwell**, aggregated per run after calibration. This targets reusable load compensation versus integral reacquisition. Also report original-reference error, final dwell bias, completed waypoints, completion time, integrated current squared, saturation duration/entries, peak error, fallback/rejection and learning availability.

Register the following before held-out execution:

- At least **10% median paired reduction** in the primary metric against the stronger deterministic comparator.
- No held-out case loses a completed waypoint that its deterministic comparator completes; no benefit is credited to slower motion or a smaller requested range.
- No new current-target limit violation, stale-result application, safety bypass or harmful re-arm sequence. Any new fault requires cause review before adoption.
- Report every per-case regression. More than 20% primary-metric deterioration in any ordinary feasible held-out case fails the default adoption gate; analyze rather than silently drop it.
- Primary comparison remains conditional if fewer than 80% of ordinary held-out cases obtain a usable estimate. Do not advertise a generally useful adaptive controller on a narrow subset.

These are project decision thresholds, not hardware safety certification. Five seeds quantify repeatability over chosen disturbances; they are not a calibrated probability estimate or a statistical significance claim.

## 9. L5 — Final Task 3 answer

The final answer must directly address what learning observes, what it changes, when it is ignored, why it is bounded, and whether it improves untuned cases against feed-forward/integral alternatives.

Possible outcomes:

- **Adopt:** passes the preregistered benefit and safety gates; retain the smallest estimator and its explicit operating assumptions.
- **Conditional:** helps only after enough calibration, with known yaw or within a limited load range; state the exact applicability and abstention behavior.
- **Reject:** no material benefit, weak identifiability, excessive calibration, unsafe interaction, or physical capacity dominates. Keep the deterministic baseline and explain what evidence would change the decision.

Update `report/task3.md` only with results that actually exist. Include failed/abstaining cases and the cost of calibration. Do not use the old 48% / 2.93° baseline figures as current evidence.

## 10. Execution and review protocol

Implementers receive one bounded packet and its allowed files, not the full project conversation. They do not spawn additional agents or expand scope. Parent reviews the report and dispatches an independent reviewer before integration or performance claims.

The L1 assignment is authorized now by the user's request. Subsequent packets retain their prerequisite gates. If a smaller-model attempt is blocked by quota/tool availability, report the blocker accurately; do not claim work is underway or silently switch models.

Execution status: [task3-execution-log.md](task3-execution-log.md).
