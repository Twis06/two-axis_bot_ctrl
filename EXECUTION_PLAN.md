# Assessment Tasks 1–5 — Planning and execution handoff

**Planning checkpoint: 2026-09-24 — reopened after high-level evidence review.**

**Role boundary:** The primary agent owns high-level reasoning, sequencing, acceptance criteria, and synthesis. Other agents implement, run experiments, and review changes. This plan does not authorize the planner to resume implementation. No new execution agents are dispatched by this document.

**START HERE:** Execute the corrective packets **R1–R6 in §10**. They supersede the original “dispatch Packet 2A first” handoff. The current controller remains frozen; the immediate work is evaluation correctness, missing validation, provenance and reporting. Do not interpret old completed checkboxes as approval of the findings reopened below.

**Decision that remains supported:** retain the deterministic baseline and do not adopt this frozen adaptive candidate. Recalculation of the stored held-out rows confirms 0.0% median paired primary-error improvement and 37/50 runs ever obtaining a usable estimate. These findings do not depend on the completion-scoring defect. Completion counts, comprehensive safety claims and Task 5 quantitative validation require correction. No controller redesign, gain retuning or new estimator candidate is authorized by this correction plan.

**Goal:** Produce defensible answers to all five assessment tasks, backed by reproducible evidence, with uncertainty and delivery tradeoffs stated plainly.

**Architecture:** Keep deterministic feedback, feed-forward, request handling, and local supervision as the baseline. Correct their contracts and known failures before considering a bounded adaptive load estimate. Separate hardware observations, analytic predictions, simulation outputs, and proposed tests throughout.

**Stack:** Existing Python, NumPy, SciPy, Matplotlib, standard-library unittest. Avoid new control frameworks unless an executor demonstrates a specific need.

**Requirements:** [Original assessment](Robotics%20Controls%20Technical%20Assessment.pdf), [task index](report/README.md), and the user-approved Task 2 hardening scope. This document supersedes the old development sequence in [PLAN.md](PLAN.md), not the assessment requirements.

## 1. Current state: what is actually established

| Assessment task | Current progress | Planning status |
|---|---|---|
| 1. Understand the failure | A complete working diagnosis distinguishes evidence from hypotheses | Ready for a focused consistency review; do not reopen model fitting without new evidence |
| 2. Build a baseline | Packets 2A–2C complete; baseline fingerprint `7d857df507c389c9`; important review findings resolved | **Frozen comparator.** D/E and loaded M2 remain explicitly reshaped/suspended limitations |
| 3. Decide whether learning belongs | 357 L4 runs rescored under phase-aware completion (R1); machine-audited gate and 96 registered challenge runs (R3) | **Non-adoption supported:** gate FAIL on benefit (0.0%) and availability (74%). Payload-change challenge not executed (`incomplete`) |
| 4. Make evidence | All current evidence, including Packet 4B round 2, republished from committed sources (`dff8efd`) | **R4:** clean-snapshot reproduction and comparison, see [R4](report/packets/R4.md) |
| 5. Test the explanation | 12 conditions replayed with one window and correct units (R2), plus 6 plan-mode diagnostic rows | **Done.** A prospective closed-loop prediction (command delay 1 → 5 ms at 3 Hz), committed before the runs, is supported in simulation. The earlier motor-strength study stays a retrospectively corrected secondary result. Hardware confirmation is proposed |

The current accepted baseline evidence is in [task2_numbers.md](report/task2_numbers.md), [Packet 2C](report/packets/2C.md), and the matched [Packet 4B tables](report/task4b_numbers.md). The older blocker description below is retained as planning history; Packet 2C supersedes it.

### Current results that determine the plan

- Frozen estimate-mode baseline A/B/C governed RMS: **0.68° / 0.27° / 0.40°**; tracked 5/5 each in the nominal reconstruction.
- D/E governed RMS: **4.10° / 4.25°**, original-request RMS **76.68° / 78.49°**, net path-clock progress **83% / 51%**; tracked 0/5 each. Progress is not measured velocity.
- Selected uncertainty trials: D events **5/20**, E **6/20**; governed peak-error p95 **12.87° / 14.21°**. These are stress samples, not calibrated probabilities.
- The nominal combined corner has **32.1° phase margin / 6.1 dB gain margin**; the design crossover is **4.46 Hz**. The older 24° corner belongs to a historical design.
- Task 3 primary-error benefit and availability support non-adoption. Its completion/waypoint claims must be rescored because calibration can satisfy a test waypoint.
- Task 5's published current values use 2–8 s while tracking errors use 0–8 s. Do not treat them as one-window results or validate the registered component prediction with total-current peaks.

Sources: `report/task2_numbers.md`, `report/task3_ceiling_results.json`, `exp/task5_prediction.py`, and the §10 review findings.

### Historical pre-hardening results — superseded, not current acceptance evidence

- Nominal A/B/C governed RMS: **0.69° / 0.22° / 0.25°**; no fault events in the five-seed median tables.
- Nominal D/E governed RMS: **4.05° / 4.30°**, with path-clock rates **83% / 50%**. Original wall-clock RMS is **76.28° / 77.59°**. The large original error principally reflects timing sacrificed during periodic motion; it must be reported but is not a geometric path-error metric.
- In twenty uncertainty trials per run, A/B/C have no recorded fault events; D has **5/20**, E **7/20**. D/E governed peak-error p95 is about **25° / 24°**. These ranges were selected as stress cases, not calibrated distributions.
- Payload E plus feedback-only loss produces a watchdog event and **21.80°** peak governed error. Communication-loss handling is therefore not sufficient evidence of acceptable loaded fallback/recovery.
- The three nominal communication-outage experiments also shrink yaw to **0.8×**. Investigate whether this is necessary recovery action or a monitor-history/transient artifact before describing it as a successful recovery without changed motion.
- The revised local linear design improves the combined light/strong, delayed corner to **32.1° phase margin and 6.1 dB gain margin**. Nominal crossover is **4.46 Hz**. Retain this conservative design unless matched evidence justifies another change.
- Applied current targets stayed within the active limit in the new evaluated cases. Measured current can transiently exceed a newly reduced limit because current cannot change instantaneously.

### Historical governor blocker — resolved by Packets 2A–2C

The following records the original handoff. It is not an open defect and must not trigger a second governor rewrite.

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
10. This folder now has a Git repository and substantial uncommitted/untracked work. Preserve a named snapshot and checksum manifest including untracked required sources before execution. A clean export of HEAD alone omits current Task 3/5/report work. Do not reset, clean, discard another agent's changes, or claim a clean-checkout reproduction of files absent from that checkout.

## 3. Sequence and ownership

**Current correction sequence:** R1 (Task 3 scorer) and R2 (Task 5 evaluator) may be prepared independently under disjoint file ownership; then R3 (adaptive validation), R4 (single frozen publication/reproduction), R5 (claim reconciliation and HTML), and R6 (independent review). Serialize evidence publication: source edits anywhere in the declared hash set can invalidate another worker's runs. The diagram below preserves the original development sequence.

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

- [x] Verify all observed numbers against the assessment, including distinctions between clipped commands and measured current.
- [x] Retain the B/C coupling calculation, A headroom analysis, D bias decomposition, and D→E capacity change.
- [x] Preserve uncertainties: controller structure unknown, windup unproven, payload mass/direction unidentifiable, thermal sequence unknown, and derated C marginal under a conservative reserve rather than categorically impossible.
- [x] Identify old claims superseded by later evidence; annotate or link them without rewriting the observation record.
- [x] Keep one decisive hardware experiment: timestamped roll-held yaw excitation with current, motion, active limit and transport/electrical measurements.

**Acceptance:** every causal claim is labeled as observation, calculation, hypothesis, or simulation finding. No simulated success is used to prove the original hardware failure cause.

## 5. Task 2 — Build a baseline

### Packet 2A — Repair the governor's physical and reference contract (first priority)

**Owner/files:** controller agent; `ctrl/governor.py`, its call site in `ctrl/baseline.py`, governor regressions in `tests/test_ctrl.py` / `tests/test_hardening.py`.

**Required contract:** A governor may slow traversal of an admitted path or explicitly restrict/reject a request. It must not silently create a different path to exploit motion as disturbance cancellation. A requested stationary hold remains stationary in its admitted reference; if it cannot be supported, change the decision, not the reference geometry.

- [x] Reproduce the reviewer's 0.4 and 0.8 N·m hold cases, both standalone and through the host/drive.
- [x] Test abrupt but admissible reference changes, empty static sets, speed-limited acceleration intervals, derating during motion, and boundaries of the reachable feasible interval.
- [x] Explicitly distinguish static infeasibility, transient dynamic infeasibility, and lack of braking room. Do not substitute zero for an empty set.
- [x] Replace the follower projection with a method that respects both reference/path constraints and attainable acceleration. Check existence of a feasible solution before root finding. If the state cannot remain in the envelope, surface that condition to the supervisor/planner.
- [x] Ensure emitted position, velocity and acceleration are mutually consistent. Do not clamp position while retaining incompatible velocity/acceleration.
- [x] Make request disposition observable: accepted, reshaped, restricted, rejected; retain a reason and freeze/replan semantics.

**Acceptance:** the zero-hold examples do not drift their reference; no unbracketed solver result is reported feasible; no empty set is accepted; regression sweeps stay within declared geometric/numerical tolerances. An infeasible initial physical state must produce explicit degraded/fallback behavior, not a claim that software can guarantee containment.

**Design guidance:** prefer a simple feasible path-time scaler with a separate explicit rejection path over adding another high-gain reference-tracking loop. Do not replace the entire controller unless these contracts cannot be achieved with the existing structure.

### Packet 2B — Close fault, recovery, and information-access gaps

**Owner/files:** controller agent; `ctrl/baseline.py`, `ctrl/supervisor.py`, `ctrl/interfaces.py`, `sim/drive.py`, directional transport tests in `sim/config.py` / `sim/engine.py`.

- [x] Preserve and review the existing stale/nonfinite feedback handling, post-queue current clamp, overspeed recovery condition, and fresh-command dwell tests.
- [x] Define recovery by fault class. Automatic recovery is appropriate for a cleared transient communication outage only after fresh, valid, aligned, healthy operation persists. A persistent tracking/load fault must not be cleared solely by moving the reference to the measured position. Use an explicit replan/acknowledgment or evidence that the new request is feasible.
- [x] Exercise feedback-only, command-only and bidirectional outages under nominal and shifted payloads. Check loaded fallback displacement, peak velocity/current, fault sequence and repeated re-arm behavior.
- [x] Diagnose yaw shrinkage after brief outages. Reset or retain monitor history deliberately, and require a complete valid observation window when computing saturation occupancy; do not exclude inconvenient unsaturated samples from its denominator.
- [x] Establish a realizable yaw-information mode. Recommended primary baseline: causal filtered yaw estimates from observable feedback. Planned-yaw look-ahead is an optional mode whose stronger assumptions are declared and tested with plan-following error.
- [x] Define what happens without yaw-control authority. Report incompatibility and request coordinated stop/replan; local damping alone cannot contain arbitrary continuing external yaw disturbance.
- [x] Validate nonfinite command/reference values, timestamps, and reportable fault causes at the relevant boundary. Avoid silently turning invalid numbers into a maximum-current command.

**Acceptance:** fault recovery cannot bypass the condition that caused the fault; communication loss cannot be concealed by fresh outgoing commands; the baseline's claimed operating mode uses no unavailable future information. Fallback must be characterized as motion reduction, not an unverified safe pose or gravity hold.

**Scope guard:** a causal yaw estimator is deterministic state estimation for Task 2, not the learning feature of Task 3. If it materially changes architecture, the executor returns a short design decision to the planner before implementation.

### Packet 2C — Freeze the deterministic baseline

**Owner:** controller agent, then independent reviewer.

- [x] Keep the revised 4.46 Hz design unless the corrected implementation or measured-information mode requires re-derivation. Preserve nominal PM ≥45° / GM ≥6 dB and selected combined-corner PM ≥30° / GM ≥6 dB as explicit local design criteria.
- [x] Check linearization limitations: gravity stiffness varies with roll angle/payload; continuous-time delay approximations do not prove sampled nonlinear stability.
- [x] Run all simulator/controller tests and matched nominal, delay, derating, loaded and voltage-limited cases.
- [x] Publish a controller/configuration fingerprint and list known operating-envelope limitations.
- [x] Rewrite `report/task2.md` only around accepted implementation behavior and frozen evidence.

**Acceptance:** no open critical governor/fault-handling findings. All required regression tests pass. Difficult D/E trials either track within declared goals or explicitly reshape/reject with the promised fault behavior. Zero event counts alone do not constitute success.

## 6. Task 3 — Decide whether learning belongs

**Owner:** adaptation experiment agent. **Starts after:** baseline freeze and metrics gate.

**Decision first:** Is poor D/E performance caused by unknown but estimable gravity torque, lack of physical capacity, or a still-defective request/recovery policy? Learning is only a candidate for the first case.

- [x] Run a clearly labeled known-load diagnostic to estimate the performance ceiling under the same torque limits. It may access truth only as an oracle, never as deployable controller evidence.
- [x] Compare the fixed baseline, a reasonably retuned integral baseline, and at most one structured estimator for `theta_s sin(q) + theta_c cos(q)`.
- [x] Specify measurement history, time alignment, excitation requirements, parameter/rate bounds, stale-result timeout, and saturated/faulted update exclusion before implementation.
- [x] Use at most 50 Hz asynchronous updates with 2–8 ms latency and zero-order hold. The deterministic safety system remains authoritative.
- [x] Separate fit/tuning cases from held-out payload masses, COM directions, trajectories, and transport seeds. Both COM shift directions must be representable in the test model before claiming generalization to them.
- [x] Protect against confusing inertia, friction, derivative noise, and coupling residual with gravity. Freeze estimates when the data cannot identify the parameters.
- [x] Require conservative uncertainty treatment before an estimate can expand the governor's admitted envelope. An apparently smaller load must not silently create claimed capacity.

**Adoption gate:** predeclare one useful-motion measure, such as finite-path completion time within a tracking tolerance, and require a practically meaningful improvement (proposed default: ≥10% median improvement) against the stronger deterministic comparator. No new limit violations or worse fault containment; report per-case regressions and spread, not only the median. The 10% threshold is a project decision to freeze before evaluation, not an assessment requirement.

**If it fails:** retain the deterministic baseline, document why, and state the measurement that would change the decision. A rigorous decision against learning completes Task 3.

**Recorded outcome, qualified after review:** L1/L2 passed their structural gates, the L3 suite has 7 passing focused tests, and L4 produced 357 manifested simulations. The exact adaptive candidate achieved 0.0% median paired reduction and 37/50 runs ever usable, supporting non-adoption. No new watchdog trips, suspensions or rejections were recorded in the 50 held-out candidate runs. Published 40/50 adaptive and 50/50 oracle-FF completion counts and waypoint-preservation claims are provisional until R1. The oracle-FF primary-error reduction of 76.9% remains evidence of potential benefit, not deployability or a proven mathematical upper bound. Full adaptive stress validation remains open under R3. The L3 tests include a widened stationarity-window helper; passing that helper is not evidence that every integration challenge was exercised on the exact frozen candidate.

## 7. Task 4 — Make evidence

### Packet 4A — Metrics and provenance

**Owner/files:** evidence agent; `sim/metrics.py`, `exp/task2_eval.py`, additional evaluation helpers, run manifests. Controller changes require coordination with Packet 2 owner.

- [x] Report original wall-clock error, governed-reference error, geometric/path error where well-defined, and delivered progress separately.
- [x] Add finite motion sequences for completion-time comparisons. Mean path-clock rate on a periodic sine is not delivered physical speed and is not an adequate success measure by itself.
- [x] Stratify metrics by normal tracking, reshaping, fallback and recovery. A reference realigned to the plant can make governed error artificially small during failure.
- [x] Report current commands before/after limiting, measured current, active limits, duration/entries of saturation, thermal/voltage activity, and per-fault timelines.
- [x] Record original and delivered yaw amplitude/frequency and any coordination requests.
- [x] Store code hash, exact configuration, seed, environment versions, time window, controller mode and metric definitions with each run.
- [x] Preserve named old artifacts. Avoid a reproduction command silently changing a historical table while leaving its narrative unchanged. Stage output then publish it only after the full evaluation succeeds.

**Acceptance:** a rejected/stationary controller cannot score as a successful tracker; every report row traces to an exact reproducible run; original/modified requests remain distinguishable.

### Packet 4B — Matched baseline validation

**Starts after:** Task 2C and Packet 4A.

- [x] Rerun A–E using the same stated trajectory/payload assumptions and seeds for every controller variant.
- [x] Include yaw estimator/plan mismatch, asymmetric packet loss, derating, low bus/high resistance, loaded recovery, quantization near rest, and combined parameter/delay corners.
- [x] Treat current D/E faulting cases as a diagnostic set to classify, not cases to quietly exclude or tune against indefinitely.
- [x] Include one explicit physically/request-policy infeasible case. Record rejection and actual subsequent motion; do not count fallback as tracking success.
- [x] Plot saturation, phase response, fault/recovery sequence and generalization. Inspect plots against raw traces before accepting aggregate claims.

**Acceptance:** the controller meets its stated contracts, and unmet performance goals are declared with their delivered-motion tradeoff. Monte Carlo is sampled evidence, not proof of all possible combinations.

### Packet 4C — Final design comparison and reproduction

**Starts after:** Task 3 decision and Task 5 result.

- [x] Compare the frozen deterministic baseline and selected final design under matched inputs, limits, seeds and access to information.
- [x] If learning was rejected, state that baseline equals final; do not fabricate a different final architecture to fill a comparison table.
- [x] Reproduce tests and all current evidence from a clean environment with one documented command. Capture failed runs as failures, not partial successes.
- [x] Reconcile current versus historical evidence in Task 4, the report index, README and HTML after R1–R4. Earlier cleanup was insufficient; see R5.

## 8. Task 5 — Test the explanation prospectively

**Owner:** prediction/experiment agent. **Starts after:** baseline/final-controller freeze. Prediction registration must precede the new experiment.

Use the existing proposed motor-strength perturbation only after checking experiment history. Randomized uncertainty sweeps are prior knowledge; do not portray the exact 15% weakening as wholly unexplored. A new, frozen, matched intervention remains useful if its provenance and prior exposure are declared. If the exact proposed controlled experiment has already run, choose a genuinely unexecuted condition before calculating/registering its prediction.

- [x] Specify one explanatory claim. For example: at fixed yaw disturbance, a weaker motor reduces torque generated by nominal feed-forward, leaving a predictable residual for feedback.
- [x] Audit the existing registration against the executed parameters, initial state, duration, scoring window and primary metric; preserve the original registration and document discrepancies (R2).
- [x] Validate a closed-loop prediction using the actual frozen controller and information mode, including delay/filter effects. **Not established:** existing predictions are ideal component arithmetic. R2 must report this gap explicitly; a new prospective study requires a separate frozen protocol before execution. *(Done 2026-09-25: prospective command-delay test, prediction committed before the runs, supported; see docs/plans/task5-prospective-protocol.md.)*
- [x] Distinguish ideal current scaling from measured total current. The existing 15% example predicts 1/0.85 current scaling and a coupling residual; it does not yet predict total RMS tracking error.
- [x] Review the original numerical prediction and tolerance justification. Preserve 0.107637 A ±0.15 A; acknowledge that its interval includes zero change. Do not tighten it retrospectively to manufacture a pass.
- [x] Correct the scored quantities/windows, compare like quantities, and report untested predictions as untested rather than passed (R2).
- [x] Make only the smallest justified design change, then compare against the unchanged frozen version; no design change was justified.
- [x] Specify hardware confirmation: effective torque/current calibration and a timestamped yaw-disturbance experiment in the actual drive convention.

**Acceptance:** a reader can establish prediction-before-result ordering; the comparison addresses the declared explanation; unfavorable outcomes are retained; any design change follows evidence rather than simultaneous untargeted tuning.

**Recorded outcome, qualified after review:** Twelve paired simulated rows exist for nominal/weakened motor, feed-forward on/off and seeds 21–23. The record states that prediction registration preceded execution; preserve the original artifact and available ordering evidence. The coupling match is a model-consistency check. Tracking metrics currently use the wrong declared window and a current-labeled field contains torque. Neither the ideal current increment nor the actual closed-loop residual has been validated by the published comparison. No controller retuning is justified by this finding.

## 9. Final deliverables and review gates

**Owner:** synthesis/documentation agent; planner owns claim consistency.

- [x] Technical memo, maximum four pages excluding plots/appendices: diagnosis, baseline/safety, learning decision, prediction result and limitations.
- [x] One-page hardware qualification plan: instrumentation; identification → static holds → limited sweeps → yaw disturbance → fault testing; numerical stop/acceptance thresholds tied to a declared operating envelope.
- [x] References/reused-code/automated-tools note. Include generated-code and agent contributions accurately.
- [x] Runnable code, current evidence plots and one-command reproduction instructions.
- [x] Final independent review of current code, evidence provenance and memo claims. No open critical safety-contract findings; explicitly list accepted limitations.

## 10. Corrective execution handoff — start here

**Authorization/scope:** assigned execution agents should implement the corrections below, verify them, and deliver their evidence without asking whether to fix these confirmed findings. The planner updates requirements and reviews decisions; this document does not itself dispatch agents. Escalate a proposed controller change or a new learning candidate rather than including it in an evidence repair.

Before edits, record the working-tree source snapshot, original report hashes and current baseline fingerprint. Preserve original registrations and published results in a named historical archive. Do not overwrite `note.md`. Every packet must produce `report/packets/R<n>.md` with the reproduced issue, changed files, actual commands/results, before/after differences, unresolved limitations and gate verdict. Never substitute expected output for executed output.

### R1 — Correct Task 3 test-phase completion scoring (priority 1)

**Owner:** evaluation/scoring agent. **Files:** `exp/task3_l4.py`, `sim/metrics.py` only if the shared API needs an explicit scoring boundary, affected metric tests and a focused L4 scoring regression. Read `exp/evidence.py`, `exp/motions.py`, and `docs/plans/task3-learning.md` before choosing the interface. Preserve default behavior for existing Task 2/4B callers unless a separate defect is demonstrated.

**Confirmed finding:** `completion()` searches from run start. L4 supplies test waypoints but no test-phase boundary. Stored manifests credit the first +30° waypoint at approximately 5.5 s during calibration in all 50 baseline, adaptive and oracle-FF held-out rows checked; the test segment starts after the calibration phase, at 9 s of requested path time.

- [x] Reproduce with a small synthetic trace: calibration visits +30°, the actual test fails to revisit it, and the old scorer incorrectly credits it.
- [x] Define a phase-aware scoring contract from the actual sequence metadata. Account for governor path time and distinguish request-phase time from elapsed wall time; do not blindly cut at wall-clock 9 s if reshaping delays the sequence. Calibration elapsed time must still count toward the total completion budget.
- [x] Require test waypoints in the intended order during their test phase. Cover calibration-only crossings, omitted/reordered waypoints, delayed test entry, a valid completed sequence, and post-arrival faults. Preserve failure when a waypoint is not visited.
- [x] Report each waypoint's visit time/phase, missing waypoints and completion. Confirm that calibration cannot satisfy a test target. Compare the candidate's individual waypoint outcomes against its paired comparator, not only the final completion boolean.
- [x] Rescore retained traces if sufficient traces exist; otherwise replay the exact frozen protocol from the preserved source/configuration. Prepare staged corrected L4 output for publication under R4. Report whether 40/50 and 50/50 change; do not assume they must change.

**Gate:** synthetic counterexamples fail before the correction and pass after; legitimate sequences still pass; old caller behavior is checked; every revised completion claim traces to phase-correct visits. The 0.0% primary-benefit result must remain separately computed and any unexpected change investigated.

### R2 — Repair Task 5 quantities, windows and inference (priority 1)

**Owner:** prediction/evaluation agent. **Files:** `exp/task5_prediction.py`, `tests/test_task5_prediction.py`, staged Task 5 results/narrative. Read `exp/common.py`, `sim/metrics.py`, `ctrl/baseline.py`; preserve `report/task5_registration.json` unchanged.

**Confirmed findings:** `_row()` masks current at 2–8 s but copies tracking summaries produced over 0–8 s. `predicted_coupling_peak_current_A` copies `c_tau_cpl`, which is torque in N·m. The registered ideal component increment is not the measured total-current maximum. The ±0.15 A tolerance is wider than the predicted +0.107637 A change.

- [x] Add a regression with deliberately different startup and scoring-window error/current, exposing inconsistent windows; add a dimensional check exposing torque mislabeled as current.
- [x] Use the registered `[2, 8)` s mask for windowed performance quantities. Keep whole-run safety events separately labeled rather than hiding startup faults. Record the metric definitions in the output schema.
- [x] Give controller-estimated coupling torque an explicit N·m field. If reporting current, divide by the declared appropriate motor constant and distinguish nominal commanded FF current from ideal true-plant required current; do not interchange the two. Mark the old field as corrected in the packet.
- [x] Replay the original 12 conditions with frozen controller/scenario/seeds; this is a **retrospective correction**, not a newly unseen prospective experiment. Publish paired corrected differences and any changed conclusions through R4.
- [x] For each original prediction, provide a matrix: predicted physical quantity, comparison quantity actually measured, units/window, discrepancy, tolerance basis, and verdict (`supported`, `failed`, or `not tested`). A repeated prescribed coupling calculation is a consistency check, not validation of the hardware explanation or the controller's residual.
- [x] Remove the unsupported completed closed-loop-prediction claim. Preserve the original numerical prediction/tolerance and available registration-order evidence. A tighter tolerance selected now cannot retrospectively validate the old study.

**Gate:** every metric has a consistent documented window and unit; regression checks discriminate the old bug; corrected results preserve the original conditions; unsupported prediction claims remain visibly open. No controller change is required. If a further prospective closed-loop prediction is needed to close §8, return a separate proposed protocol and analytical prediction to the planner **before running it**; this packet does not authorize silently inventing or backdating a new study.

### R3 — Close adaptive validation gaps and make the gate complete (priority 2)

**Owner:** adaptive-validation agent. **Starts after:** R1 scoring contract. **Files:** `exp/task3_l4.py` or a separate adaptive challenge harness, focused gate/challenge tests, Task 3 validation packet. The frozen estimator, adapter settings and baseline must remain unchanged.

**Confirmed findings:** the existing L4 matrix has stationary yaw and unholdable checks, but lacks the separately planned B/C yaw, 60 ms feedback-only outage, mid-run derating and payload-change integration groups. Some L3 engagement/outage tests use `stationary_counts=10`, whereas the submitted L4 candidate uses the frozen default. The gate currently checks overall completion loss and selected event counts but omits other registered criteria.

- [x] Make every registered criterion explicit and machine-auditable: ≥10% median paired primary improvement; ≥80% ever-usable runs; no lost individual comparator waypoints; no >20% primary deterioration in an ordinary feasible case; target-current compliance; stale-result/safety/re-arm checks; and no benefit credited to reduced range or slower motion. Missing metrics or required case groups yield `incomplete`, never `pass`.
- [x] Add small adversarial gate tests: a hidden lost waypoint, a severe single-case regression masked by an improved median, unavailable metrics, a target-limit violation, and missing required challenges. Record per-pair outcomes, not only an aggregate boolean.
- [x] Distinguish `ever usable` from usable fraction and application during scored dwells. The existing 37/50 figure means ever usable, not sustained coverage; median usable fraction across the 50 rows is approximately 9.4% of the whole run. Keep the original availability definition for historical comparison and add the coverage diagnostics.
- [x] Register an explicit supplementary challenge matrix before execution: exact frozen candidate versus `int1`; B and C yaw excitation, 60 ms feedback-only loss, 3.2→2.4 A derating and a change of payload law; fixed paired seeds, perturbation onset, limits, scoring windows and recovery observations. Reuse existing scenario definitions where appropriate. Label these supplementary validation runs, not original held-out discovery data.
- [x] Ensure challenge timing exercises an actually active learned correction where possible; use the same allowed calibration history/time for both variants. If the exact candidate never activates, report the challenge as unexercised/abstaining rather than claiming its active behavior passed. Do not widen stationarity gates to force engagement.
- [x] Report bounds, current targets, fault timelines, correction disable/re-enable, stale-job rejection, delivered motion and per-case regressions. Keep the failed benefit decision separate from validation coverage.

**Gate:** every criterion has evidence or explicit incomplete status; supplementary outcomes identify the exact candidate configuration. The candidate remains not adopted even if all safety challenges pass, because its original benefit gate failed. Any newly exposed controller/adapter defect returns to planner/reviewer for a scoped correction rather than automatic retuning.

### R4 — Publish consistent evidence and verify clean reproduction (priority 2)

**Owner:** provenance/reproduction agent; sole owner of final generated artifact publication. **Starts after:** R1–R3 source changes are frozen. **Files:** `run_all.py`, reproduction documentation, generated Task 3/4B/5 packets and manifests; source changes needed for correct staging only.

- [x] Preserve the old artifacts and registrations. Verify the frozen baseline fingerprint remains `7d857df507c389c9`; do not refreeze merely to suppress a mismatch.
- [x] Republish Packet 4B from its reviewed round-2 source, incorporating any shared scoring change that genuinely affects it. Verify generated tables/figures and source hashes correspond; reconcile the existing scratch-export/round-1 publication note.
- [x] Stage and publish R1/R2/R3 results only after each complete run succeeds and source hashes remain stable. New run IDs caused by an expanded declared source set must be explained, not mistaken for metric changes.
- [x] Make the documented full reproduction command cover tests, the declared current analyses, Task 2 evaluation/robustness, Task 3 L2/L4/supplementary validation, Task 4B and Task 5. Define quick mode honestly; it cannot satisfy the full gate. Preserve original prospective registration instead of regenerating it after results.
- [x] Construct a clean isolated delivery snapshot containing every required tracked and currently untracked source plus dependency versions. Identify it by full hashes. Run the full command there, retaining failures and logs. Avoid publishing partial success over the working evidence.
- [x] Compare regenerated metrics to corrected reference outputs using declared numerical tolerances. Classify differences as deliberate corrections, numerical/environment differences, or unexplained failures. Verify all selected figures and cited rows have traceable sources.

**Gate:** a fresh recipient can reproduce the declared current evidence from the delivered snapshot with the documented command; no omitted scripts, source/result mismatch or failed stage is described as success. Do not claim a committed clean checkout unless the required sources are actually included in one.

### R5 — Reconcile claims and rebuild the report (priority 3)

**Owner:** synthesis/documentation agent; planner reviews scope of claims. **Starts after:** corrected results are published. **Files:** `report/task1.md` through `task5.md`, `report/README.md`, root README, this plan, `report/html_assets/report.md`, HTML builder outputs and relevant execution-log status notes.

- [x] Complete Task 1's observation/calculation consistency check against the assessment. Retain uncertainty rather than promoting simulation fit to hardware diagnosis.
- [x] Correct Task 2's contradictory statement that the 2° RMS target is not used for success: it is a project tracking-classifier threshold, while 5° peak is not the same classifier's criterion and neither is prescribed by the brief.
- [x] Make the frozen design's 32.1° corner margin the current claim. Label the old 24° result and earlier Phase 2 performance explicitly historical at the point of use.
- [x] Replace Task 3 completion counts only with R1 results. Scope “no safety regressions” to evaluated conditions; state missing/unexercised cases and exact configuration. Do not generalize this candidate's non-adoption to all learning designs.
- [x] Replace Task 5 and HTML mixed-window descriptions/numbers with R2 results. State exactly which predictions were tested; leave the stronger closed-loop-prediction gate open if it was not completed.
- [x] Update Packet 4B publication and reproduction status from actual R4 output. Qualify test counts by command, snapshot and date; passing tests alone do not validate an experiment design.
- [x] Rebuild `report/assessment_report.html` and its manifest with the report builder. Verify all 18 existing figures plus any new published figures, interactive data, local links, mobile layout and source labels. Preserve historical figures without presenting them as current results.
- [x] Keep final four-page memo, page-verified hardware qualification plan and final review checkboxes open until those separate deliverables actually exist. The full HTML report does not automatically satisfy a four-page limit.

**Gate:** one consistent current account across plan, task answers, generated evidence and HTML; each acceptance claim points to its corrected evidence; unresolved items are visible, not hidden by a “complete” headline.

### R6 — Independent final review (priority 3)

**Owner:** reviewer who did not implement the corrective packets. **Starts after:** R4/R5 completion.

- [x] Independently challenge R1's calibration/test boundary, test waypoint order and failure behavior.
- [x] Check R2's units, windows and prediction-to-measurement mapping against the source and corrected rows.
- [x] Check R3's gate completeness, exact candidate configuration and supplementary challenge coverage; verify why non-adoption remains supported.
- [x] Sample manifest-to-source-to-table-to-plot traces; inspect the clean reproduction log and current report claims.
- [x] Deliver prioritized findings with reproduction/evidence and an explicit verdict. Keep any unresolved important issue open; planner self-review does not count as independent review.

**Final release gate** (met: R6 round 4 PASS WITH ISSUES, see report/packets/R6.md): no unresolved important scoring, provenance or misleading-claim findings. Retain declared physical limitations, untested hardware behavior and any deferred prospective-prediction work in the final status.
