# Assessment Tasks 1–5 — Planning and execution handoff

**Planning checkpoint: 2026-09-23.**

**Role boundary:** The primary agent owns high-level reasoning, sequencing, acceptance criteria, and synthesis. Other agents implement, run experiments, and review changes. This plan does not authorize the planner to resume implementation. No new execution agents are dispatched by this document.

**Goal:** Produce defensible answers to all five assessment tasks, backed by reproducible evidence, with uncertainty and delivery tradeoffs stated plainly.

**Architecture:** Keep deterministic feedback, feed-forward, request handling, and local supervision as the baseline. Correct their contracts and known failures before considering a bounded adaptive load estimate. Separate hardware observations, analytic predictions, simulation outputs, and proposed tests throughout.

**Stack:** Existing Python, NumPy, SciPy, Matplotlib, standard-library unittest. Avoid new control frameworks unless an executor demonstrates a specific need.

**Requirements:** [Original assessment](Robotics%20Controls%20Technical%20Assessment.pdf), [task index](report/README.md), and the user-approved Task 2 hardening scope. This document supersedes the old development sequence in [PLAN.md](PLAN.md), not the assessment requirements.

## 1. Current state: what is actually established

| Assessment task | Current progress | Planning status |
|---|---|---|
| 1. Understand the failure | A complete working diagnosis distinguishes evidence from hypotheses | Ready for a focused consistency review; do not reopen model fitting without new evidence |
| 2. Build a baseline | Existing design retained; initial safety fixes implemented; 40 tests passed; fresh evidence generated | **Open. A new governor blocker was found after those tests.** Do not label the baseline ready |
| 3. Decide whether learning belongs | Structured payload estimation proposed; no implementation/comparison | Hold until Task 2 is stable and useful performance is measurable |
| 4. Make evidence | Simulator, legacy comparison, uncertainty/fault evaluations and plots exist | Evidence infrastructure needs provenance and metric improvements; final comparison follows Task 3 |
| 5. Test the explanation | Component-level motor-strength prediction drafted; no frozen closed-loop prediction/test | Preserve prospective status; run only after prediction registration on a frozen controller |

The latest results are in [task2_numbers.md](report/task2_numbers.md) and [trial data](report/task2_results.json). They describe the current working implementation, which still contains the review blocker. They are diagnostic results, not accepted final performance.

### Results that determine the plan

- Nominal A/B/C governed RMS: **0.69° / 0.22° / 0.25°**; no fault events in the five-seed median tables.
- Nominal D/E governed RMS: **4.05° / 4.30°**, with path-clock rates **83% / 50%**. Original wall-clock RMS is **76.28° / 77.59°**. The large original error principally reflects timing sacrificed during periodic motion; it must be reported but is not a geometric path-error metric.
- In twenty uncertainty trials per run, A/B/C have no recorded fault events; D has **5/20**, E **7/20**. D/E governed peak-error p95 is about **25° / 24°**. These ranges were selected as stress cases, not calibrated distributions.
- Payload E plus feedback-only loss produces a watchdog event and **21.80°** peak governed error. Communication-loss handling is therefore not sufficient evidence of acceptable loaded fallback/recovery.
- The three nominal communication-outage experiments also shrink yaw to **0.8×**. Investigate whether this is necessary recovery action or a monitor-history/transient artifact before describing it as a successful recovery without changed motion.
- The revised local linear design improves the combined light/strong, delayed corner to **32.1° phase margin and 6.1 dB gain margin**. Nominal crossover is **4.46 Hz**. Retain this conservative design unless matched evidence justifies another change.
- Applied current targets stayed within the active limit in the new evaluated cases. Measured current can transiently exceed a newly reduced limit because current cannot change instantaneously.

### Known blocker from independent review

The new governor follower can use roll acceleration to cancel an excessive yaw/load torque by **moving the governed reference away from a requested hold**. It can also search for a torque-feasible acceleration without confirming that one exists inside its speed-limited interval.

Reviewer-reported reproducer: `RollGovernor.reset(0)` followed by 1,000 calls to `step(0.002, Hold(0).eval, 3.2, tau_couple=0.8)` produced approximately **−37.36 rad**, velocity **−20 rad/s**, and nominal torque demand **0.640 N·m** against a **0.3084 N·m** budget, with rejection false. At 0.4 N·m coupling, the reference reportedly settled near −0.641 rad despite a zero hold request. These are standalone governor probes, not claims about a completed physical closed-loop run. The execution agent must reproduce them and test the full integration.

The review ended before a complete review report was delivered. Its absence is not approval. Existing 40 passing tests do not cover this failure.

## 2. Global rules for execution agents

1. Use only the A–E summaries as observed hardware evidence. Model fit is not identification.
2. Preserve 1 kHz local current/safety updates, host rate at most 500 Hz, 14-bit position sensing, 1.2 ms current lag, fixed 1 ms command delay, and the specified CAN range/bursts.
3. Enforce 3.2 A nominal / 2.4 A derated target limits. Distinguish target clamping from measured-current settling.
4. Only use information available to the controller at that time. Observable yaw does not imply a perfect future yaw plan or authority over yaw.
5. Separate controller health, physically feasible requests, and tracking performance. A correctly handled fault is not successful tracking; a large error on an infeasible request is not automatically a controller instability.
6. Keep plant truth inaccessible to deployed controllers. An oracle may be used only as an explicitly labeled diagnostic comparison.
7. Numerical performance targets introduced by the project are design assumptions. The brief does not specify 2° RMS / 5° peak or a certification standard.
8. Write regression tests for identified behavior failures before fixing them. Run the complete suite after each integrated change; do not weaken existing tests to preserve attractive performance numbers.
9. Freeze source/configuration before generating evidence. Do not edit running experiment code or overwrite final results with partial runs.
10. This folder currently has no Git repository. Do not assume commits or worktrees exist. Preserve a named snapshot/checksum manifest before execution; version-control setup is a separate workspace decision.

## 3. Sequence and ownership

```text
Task 1 consistency review ───────────────────────────────┐
Task 2A governor contract → Task 2B safety/information     │
                              ↓                         │
                       Task 2C baseline freeze           │
                              ↓                         │
Task 4A metrics/provenance → Task 4B matched evidence     │
                              ↓                         │
                       Task 3 learning decision         │
                              ↓                         │
                       Task 5 prediction/test           │
                              ↓                         │
                       Task 4C final comparison ────────┤
                                                       ↓
                                      Memo / qualification / references
```

Task 4A can proceed alongside Task 2A if it does not modify controller or governor files. Task 1 prose review can also proceed independently. The Task 2 executor owns shared controller/drive interfaces; other agents propose interface changes through that owner. Do not run competing controller implementations in the same shared folder.

Each execution packet ends with: changed files, reproduced failure, verification command/output, before/after evidence on matched conditions, remaining limitations, and an explicit pass/fail against its gate. A different agent reviews safety-critical changes before the next dependent packet starts.

## 4. Task 1 — Understand the failure

**Owner:** analysis/documentation agent.

**Inputs:** `report/task1.md`, original PDF, `report/phase0_*`, `report/phase1_*`, current evidence labels.

**Deliverable:** a stable diagnostic argument, not another controller search.

- [ ] Verify all observed numbers against the assessment, including distinctions between clipped commands and measured current.
- [ ] Retain the B/C coupling calculation, A headroom analysis, D bias decomposition, and D→E capacity change.
- [ ] Preserve uncertainties: controller structure unknown, windup unproven, payload mass/direction unidentifiable, thermal sequence unknown, and derated C marginal under a conservative reserve rather than categorically impossible.
- [ ] Identify old claims superseded by later evidence; annotate or link them without rewriting the observation record.
- [ ] Keep one decisive hardware experiment: timestamped roll-held yaw excitation with current, motion, active limit and transport/electrical measurements.

**Acceptance:** every causal claim is labeled as observation, calculation, hypothesis, or simulation finding. No simulated success is used to prove the original hardware failure cause.

## 5. Task 2 — Build a baseline

### Packet 2A — Repair the governor's physical and reference contract (first priority)

**Owner/files:** controller agent; `ctrl/governor.py`, its call site in `ctrl/baseline.py`, governor regressions in `tests/test_ctrl.py` / `tests/test_hardening.py`.

**Required contract:** A governor may slow traversal of an admitted path or explicitly restrict/reject a request. It must not silently create a different path to exploit motion as disturbance cancellation. A requested stationary hold remains stationary in its admitted reference; if it cannot be supported, change the decision, not the reference geometry.

- [ ] Reproduce the reviewer's 0.4 and 0.8 N·m hold cases, both standalone and through the host/drive.
- [ ] Test abrupt but admissible reference changes, empty static sets, speed-limited acceleration intervals, derating during motion, and boundaries of the reachable feasible interval.
- [ ] Explicitly distinguish static infeasibility, transient dynamic infeasibility, and lack of braking room. Do not substitute zero for an empty set.
- [ ] Replace the follower projection with a method that respects both reference/path constraints and attainable acceleration. Check existence of a feasible solution before root finding. If the state cannot remain in the envelope, surface that condition to the supervisor/planner.
- [ ] Ensure emitted position, velocity and acceleration are mutually consistent. Do not clamp position while retaining incompatible velocity/acceleration.
- [ ] Make request disposition observable: accepted, reshaped, restricted, rejected; retain a reason and freeze/replan semantics.

**Acceptance:** the zero-hold examples do not drift their reference; no unbracketed solver result is reported feasible; no empty set is accepted; regression sweeps stay within declared geometric/numerical tolerances. An infeasible initial physical state must produce explicit degraded/fallback behavior, not a claim that software can guarantee containment.

**Design guidance:** prefer a simple feasible path-time scaler with a separate explicit rejection path over adding another high-gain reference-tracking loop. Do not replace the entire controller unless these contracts cannot be achieved with the existing structure.

### Packet 2B — Close fault, recovery, and information-access gaps

**Owner/files:** controller agent; `ctrl/baseline.py`, `ctrl/supervisor.py`, `ctrl/interfaces.py`, `sim/drive.py`, directional transport tests in `sim/config.py` / `sim/engine.py`.

- [ ] Preserve and review the existing stale/nonfinite feedback handling, post-queue current clamp, overspeed recovery condition, and fresh-command dwell tests.
- [ ] Define recovery by fault class. Automatic recovery is appropriate for a cleared transient communication outage only after fresh, valid, aligned, healthy operation persists. A persistent tracking/load fault must not be cleared solely by moving the reference to the measured position. Use an explicit replan/acknowledgment or evidence that the new request is feasible.
- [ ] Exercise feedback-only, command-only and bidirectional outages under nominal and shifted payloads. Check loaded fallback displacement, peak velocity/current, fault sequence and repeated re-arm behavior.
- [ ] Diagnose yaw shrinkage after brief outages. Reset or retain monitor history deliberately, and require a complete valid observation window when computing saturation occupancy; do not exclude inconvenient unsaturated samples from its denominator.
- [ ] Establish a realizable yaw-information mode. Recommended primary baseline: causal filtered yaw estimates from observable feedback. Planned-yaw look-ahead is an optional mode whose stronger assumptions are declared and tested with plan-following error.
- [ ] Define what happens without yaw-control authority. Report incompatibility and request coordinated stop/replan; local damping alone cannot contain arbitrary continuing external yaw disturbance.
- [ ] Validate nonfinite command/reference values, timestamps, and reportable fault causes at the relevant boundary. Avoid silently turning invalid numbers into a maximum-current command.

**Acceptance:** fault recovery cannot bypass the condition that caused the fault; communication loss cannot be concealed by fresh outgoing commands; the baseline's claimed operating mode uses no unavailable future information. Fallback must be characterized as motion reduction, not an unverified safe pose or gravity hold.

**Scope guard:** a causal yaw estimator is deterministic state estimation for Task 2, not the learning feature of Task 3. If it materially changes architecture, the executor returns a short design decision to the planner before implementation.

### Packet 2C — Freeze the deterministic baseline

**Owner:** controller agent, then independent reviewer.

- [ ] Keep the revised 4.46 Hz design unless the corrected implementation or measured-information mode requires re-derivation. Preserve nominal PM ≥45° / GM ≥6 dB and selected combined-corner PM ≥30° / GM ≥6 dB as explicit local design criteria.
- [ ] Check linearization limitations: gravity stiffness varies with roll angle/payload; continuous-time delay approximations do not prove sampled nonlinear stability.
- [ ] Run all simulator/controller tests and matched nominal, delay, derating, loaded and voltage-limited cases.
- [ ] Publish a controller/configuration fingerprint and list known operating-envelope limitations.
- [ ] Rewrite `report/task2.md` only around accepted implementation behavior and frozen evidence.

**Acceptance:** no open critical governor/fault-handling findings. All required regression tests pass. Difficult D/E trials either track within declared goals or explicitly reshape/reject with the promised fault behavior. Zero event counts alone do not constitute success.

## 6. Task 3 — Decide whether learning belongs

**Owner:** adaptation experiment agent. **Starts after:** baseline freeze and metrics gate.

**Decision first:** Is poor D/E performance caused by unknown but estimable gravity torque, lack of physical capacity, or a still-defective request/recovery policy? Learning is only a candidate for the first case.

- [ ] Run a clearly labeled known-load diagnostic to estimate the performance ceiling under the same torque limits. It may access truth only as an oracle, never as deployable controller evidence.
- [ ] Compare the fixed baseline, a reasonably retuned integral baseline, and at most one structured estimator for `theta_s sin(q) + theta_c cos(q)`.
- [ ] Specify measurement history, time alignment, excitation requirements, parameter/rate bounds, stale-result timeout, and saturated/faulted update exclusion before implementation.
- [ ] Use at most 50 Hz asynchronous updates with 2–8 ms latency and zero-order hold. The deterministic safety system remains authoritative.
- [ ] Separate fit/tuning cases from held-out payload masses, COM directions, trajectories, and transport seeds. Both COM shift directions must be representable in the test model before claiming generalization to them.
- [ ] Protect against confusing inertia, friction, derivative noise, and coupling residual with gravity. Freeze estimates when the data cannot identify the parameters.
- [ ] Require conservative uncertainty treatment before an estimate can expand the governor's admitted envelope. An apparently smaller load must not silently create claimed capacity.

**Adoption gate:** predeclare one useful-motion measure, such as finite-path completion time within a tracking tolerance, and require a practically meaningful improvement (proposed default: ≥10% median improvement) against the stronger deterministic comparator. No new limit violations or worse fault containment; report per-case regressions and spread, not only the median. The 10% threshold is a project decision to freeze before evaluation, not an assessment requirement.

**If it fails:** retain the deterministic baseline, document why, and state the measurement that would change the decision. A rigorous decision against learning completes Task 3.

## 7. Task 4 — Make evidence

### Packet 4A — Metrics and provenance

**Owner/files:** evidence agent; `sim/metrics.py`, `exp/task2_eval.py`, additional evaluation helpers, run manifests. Controller changes require coordination with Packet 2 owner.

- [ ] Report original wall-clock error, governed-reference error, geometric/path error where well-defined, and delivered progress separately.
- [ ] Add finite motion sequences for completion-time comparisons. Mean path-clock rate on a periodic sine is not delivered physical speed and is not an adequate success measure by itself.
- [ ] Stratify metrics by normal tracking, reshaping, fallback and recovery. A reference realigned to the plant can make governed error artificially small during failure.
- [ ] Report current commands before/after limiting, measured current, active limits, duration/entries of saturation, thermal/voltage activity, and per-fault timelines.
- [ ] Record original and delivered yaw amplitude/frequency and any coordination requests.
- [ ] Store code hash, exact configuration, seed, environment versions, time window, controller mode and metric definitions with each run.
- [ ] Preserve named old artifacts. Avoid a reproduction command silently changing a historical table while leaving its narrative unchanged. Stage output then publish it only after the full evaluation succeeds.

**Acceptance:** a rejected/stationary controller cannot score as a successful tracker; every report row traces to an exact reproducible run; original/modified requests remain distinguishable.

### Packet 4B — Matched baseline validation

**Starts after:** Task 2C and Packet 4A.

- [ ] Rerun A–E using the same stated trajectory/payload assumptions and seeds for every controller variant.
- [ ] Include yaw estimator/plan mismatch, asymmetric packet loss, derating, low bus/high resistance, loaded recovery, quantization near rest, and combined parameter/delay corners.
- [ ] Treat current D/E faulting cases as a diagnostic set to classify, not cases to quietly exclude or tune against indefinitely.
- [ ] Include one explicit physically/request-policy infeasible case. Record rejection and actual subsequent motion; do not count fallback as tracking success.
- [ ] Plot saturation, phase response, fault/recovery sequence and generalization. Inspect plots against raw traces before accepting aggregate claims.

**Acceptance:** the controller meets its stated contracts, and unmet performance goals are declared with their delivered-motion tradeoff. Monte Carlo is sampled evidence, not proof of all possible combinations.

### Packet 4C — Final design comparison and reproduction

**Starts after:** Task 3 decision and Task 5 result.

- [ ] Compare the frozen deterministic baseline and selected final design under matched inputs, limits, seeds and access to information.
- [ ] If learning was rejected, state that baseline equals final; do not fabricate a different final architecture to fill a comparison table.
- [ ] Reproduce tests and all current evidence from a clean environment with one documented command. Capture failed runs as failures, not partial successes.
- [ ] Update `report/task4.md`, the report index and README so current versus historical evidence is unambiguous.

## 8. Task 5 — Test the explanation prospectively

**Owner:** prediction/experiment agent. **Starts after:** baseline/final-controller freeze. Prediction registration must precede the new experiment.

Use the existing proposed motor-strength perturbation only after checking experiment history. Randomized uncertainty sweeps are prior knowledge; do not portray the exact 15% weakening as wholly unexplored. A new, frozen, matched intervention remains useful if its provenance and prior exposure are declared. If the exact proposed controlled experiment has already run, choose a genuinely unexecuted condition before calculating/registering its prediction.

- [ ] Specify one explanatory claim. For example: at fixed yaw disturbance, a weaker motor reduces torque generated by nominal feed-forward, leaving a predictable residual for feedback.
- [ ] Register exact nominal/perturbed parameters, controller mode, initial state, duration, scoring window, seeds and one primary metric.
- [ ] Calculate the closed-loop prediction using the actual frozen controller and the same information mode as execution. Include delay and filtered-estimation effects where applicable.
- [ ] Distinguish ideal current scaling from measured total current. The existing 15% example predicts 1/0.85 current scaling and a coupling residual; it does not yet predict total RMS tracking error.
- [ ] State a numerical predicted value and justified tolerance before running. Retain the original prediction even if it fails.
- [ ] Run the registered test, report discrepancy, and revise the explanation where necessary.
- [ ] Make only the smallest justified design change, then compare against the unchanged frozen version.
- [ ] Specify hardware confirmation: effective torque/current calibration and a timestamped yaw-disturbance experiment in the actual drive convention.

**Acceptance:** a reader can establish prediction-before-result ordering; the comparison addresses the declared explanation; unfavorable outcomes are retained; any design change follows evidence rather than simultaneous untargeted tuning.

## 9. Final deliverables and review gates

**Owner:** synthesis/documentation agent; planner owns claim consistency.

- [ ] Technical memo, maximum four pages excluding plots/appendices: diagnosis, baseline/safety, learning decision, prediction result and limitations.
- [ ] One-page hardware qualification plan: instrumentation; identification → static holds → limited sweeps → yaw disturbance → fault testing; numerical stop/acceptance thresholds tied to a declared operating envelope.
- [ ] References/reused-code/automated-tools note. Include generated-code and agent contributions accurately.
- [ ] Runnable code, current evidence plots and one-command reproduction instructions.
- [ ] Final independent review of current code, evidence provenance and memo claims. No open critical safety-contract findings; explicitly list accepted limitations.

## 10. First agent handoff

**Dispatch Packet 2A first.** Give the executor this plan, `ctrl/governor.py`, `ctrl/baseline.py`, existing governor tests, and the exact review reproducer. The assignment is to restore the governor contract, not improve headline RMS or begin adaptation. Require a minimal failure reproduction, proposed correction rationale, regression evidence, and an independent review before advancing to Packet 2B.

Packet 4A may run in parallel on metric/provenance design with clear file ownership. Do not begin new learning work or the Task 5 perturbation until their gates are met.
