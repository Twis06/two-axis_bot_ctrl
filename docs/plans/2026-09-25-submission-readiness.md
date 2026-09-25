# Close Task 5 and finalize submission — approximately one-hour execution plan

> For agentic workers: implement this plan sequentially using `superpowers:executing-plans`, or delegate implementation and independent review using `superpowers:subagent-driven-development` when authorized. This document is a plan, not a registered numerical prediction and not authorization to claim a new experiment has passed.

**Goal:** Demonstrate prediction-before-test ordering on one genuinely new, measurable closed-loop experiment, then deliver consistent, reproducible submission documents.

**Architecture:** Add a separate prospective Task 5 packet beside the historical motor-strength study. Keep the frozen baseline and existing evidence intact. Commit the new protocol, predictor and scoring implementation before the first new simulation; publish the result in a later commit.

**Tech stack:** Existing Python 3.9 / NumPy / SciPy simulator, unittest, Matplotlib, locked uv environment and report builders.

**Spec:** Assessment brief, Task 5: “Before a new test of your choice, write a numerical prediction and why you expect it.” Then change payload, motor strength, delay or bus voltage, compare the outcome, revise the explanation, identify the smallest justified change and its hardware confirmation. See `EXECUTION_PLAN.md` §8 and `report/packets/R6.md` round 4.

## One-hour operating schedule

**Target:** about 60 minutes elapsed, assuming the existing environment works and the analytical predictor can be adapted without a new modeling investigation. The deadline does not override registration order or verification. Start the timer when execution begins; planning has not started the experiment.

**Staffing:** one executor owns code, numerical registration and publication; one independent reviewer checks the protocol first and the final evidence later. A runner uses the already-configured server for clean reproduction if it is available. These are bounded parallel roles, not three agents editing the same files. The reviewer is read-only except for their review packet; the runner writes only to its isolated copy.

| Elapsed | Executor | Independent review / reproduction | Required exit condition |
|---|---|---|---|
| 0–5 min | Record starting commit/fingerprint; check exact-condition history and source-hash dependencies. Confirm the preferred delay test is unused. | Reviewer reads Task 5 requirement and proposed metric. Check server access, environment and available output transfer. | Scope is fixed. If server readiness is not confirmed within 3 minutes, use the Mac. |
| 5–18 min | Adapt the analytical predictor and phasor scorer, define numerical tolerance, implement the 10-run harness, and run synthetic contract tests. Prepare full-reproduction integration. | Reviewer checks prediction independence, delay accounting, metric definition and whether the tolerance distinguishes no change. | Complete numerical registration, harness and scoring tests committed **before any new simulation**. |
| 18–28 min | Execute all 10 registered runs, score all pairs and produce one plot/table. Commit results after the registration commit. | Reviewer verifies the registration ancestry and independently recomputes the primary metric from the recorded data. | Result is supported, contradicted or explicitly inconclusive; no missing/hidden rows. |
| 28–35 min | Finish reproduction/comparison integration; run the full unit suite; freeze the code/evidence commit. | Start full clean reproduction as soon as this commit is available. Use the server if ready; otherwise the Mac. | Exact reproducible source snapshot identified, registration hash checked, all required code tests pass. |
| 35–48 min | Replace memo Task 5 text; generate Task 5 answer/HTML, add the plot, update status and disclosures, render PDFs. | Full reproduction runs concurrently (previously ~8 min server / ~12 min Mac; allow extra time for the new packet). Reviewer checks interpretation and proposed design response. | Four-page memo body and one-page hardware plan retained; source claims match results. |
| 48–57 min | Resolve only material review findings; finish source/status edits, then rebuild HTML/manifest last. Commit final documents. | Compare reproduced evidence and verify source hashes. Review final PDFs and the diff since the independently reviewed code/evidence commit. | No important unresolved finding; all affected results reproduce. |
| 57–60 min | Package tracked submission files and record the final commit/verification scope. | Verify final document-only changes introduce no experiment/source changes; check final archive builds and links. | Submission package and concise readiness verdict delivered. |

**Parallelism boundary:** prediction → registration commit → first simulation is strictly sequential. Full reproduction may overlap final documentation only after code and evidence are frozen. If any experiment dependency changes afterward, the earlier reproduction does not verify that change: rerun affected packets and update the scope before release.

**Timing checkpoints:**

- At minute 18, if no defensible numerical prediction and acceptance region are committed, report the blocker and revised estimate. Do not start a trial merely to obtain numbers for the prediction.
- At minute 30, if the experiment is complete but unfavorable, proceed to reporting. Do not add seeds, conditions, retuning or new metrics to rescue the result.
- At minute 45, stop optional analysis and formatting work. Prioritize correctness, reproduction and the required documents.
- At minute 60, release only if the final acceptance gate below is met. Otherwise report the exact unfinished check and remaining estimate; do not convert the time target into a false readiness claim.

## Shortest implementation path

- Reuse `calc_tracking`'s derivation and the existing sine/cosine least-squares pattern; keep the new predictor/scorer in `exp/task5_prospective.py`. Do not refactor shared controller, simulator or evidence infrastructure.
- Implement only the synthetic phasor/window, invalid-data, paired-grid and registration-integrity checks needed for this packet. Reuse existing test/evidence utilities where their contracts match.
- Keep the historical Task 5 generator and registration unchanged. Run the new module after the historical Task 5 step. It writes its own numbered evidence report and a current `report/task5.md` overview using the new packet plus the preserved historical results. The overview must be reproducible by the final step, never a manual edit that reproduction overwrites.
- Use one new plot, one compact result table and a replacement Task 5 memo section. Keep the existing report layout and baseline recommendation unless the new evidence actually requires a change.
- Use one environment for initial publication and the clean copy for verification. Cross-platform differences are compared with the established numerical tolerance, not by run_id alone. Do not spend the hour configuring a new server or making a second full platform comparison.
- Preserve existing source sets: inspect whether adding the module or modifying `run_all.py` affects old run specifications. If it does, publish the affected packets from the new committed sources; never retain stale hashes to meet the time budget.

## Scope and priorities

- The new prospective test is the only substantive submission gap being closed here. The brief does not demand a successful prediction, hardware execution, or a deployed learning component.
- Preserve all previous Task 5 registrations/results and their retrospective qualifications. The new test supplements them; it does not repair their chronology retroactively.
- Retain baseline fingerprint `7d857df507c389c9`, current/supervision limits, causal controller information and the registered Task 3 rejection.
- No new learning study, payload-change challenge, LaTeX migration or controller redesign is required for this pass.
- Keep R6's remaining latent code issues disclosed. They do not affect the published rows and need not become a prerequisite for this bounded experiment.

## Review focus

1. Prediction leakage: a predictor using delays, fit parameters or thresholds measured from the new runs is retrospective.
2. Wrong signal: an ideal component-torque calculation is not a prediction of measured closed-loop response.
3. Hidden reshaping: governor motion changes must remain visible beside the tracking score.
4. Weak acceptance bands: a wide interval containing no change cannot establish a directional effect.
5. Selective reporting: missing pairs, faults, saturation, non-finite metrics and failed predictions must not disappear from the result.

## Task 1 — Freeze a prospective experiment before execution

**Create:** `docs/plans/task5-prospective-protocol.md`, `report/task5_prospective_registration.json`, `exp/task5_prospective.py`, `tests/test_task5_prospective.py`.

**Read:** `exp/task4b_eval.py` (`calc_tracking`, `post_freq`), `ctrl/baseline.py`, `ctrl/loopshape.py`, `sim/config.py`, `sim/engine.py`, `exp/common.py`, existing experiment manifests.

- [ ] Audit history for the exact proposed comparison. Disclose previous frequency/delay studies as prior knowledge; new seeds alone do not make a previously executed deterministic condition unseen.
- [ ] Preferred experiment: a 5° roll sine at 3 Hz, yaw held at zero, 1 s ramp, 12 s duration, nominal plant/current limit, frozen `BaselineController()`. Compare current-command delay 1 ms against 5 ms. Use matched seeds 301–305, nominal CAN jitter and sensing, and the same disturbance realization within each pair. Set `d_amp=0` for this controlled delay-mechanism experiment and disclose that simplification. Keep friction, encoder quantization, actuator lag, voltage limits, governor and fault logic enabled.
- [ ] Primary measured quantity: the paired change in complex fundamental tracking transfer, `ΔH = H_delayed − H_nominal`, where `H` is actual roll divided by the governed roll reference. Fit sine, cosine and intercept over `[4, 12)` s. At 3 Hz this contains 24 cycles. Use the arithmetic mean of the five paired complex changes as the primary aggregate and show every pair alongside it; report gain/phase and original-request tracking as secondary quantities.
- [ ] Derive the numerical prediction from the linearized plant, frozen feedback and reference feed-forward paths, actuator lag, and independently specified timing. `calc_tracking` is a starting point, not a complete oracle: its current reporting usage estimates timing from logs, which is prohibited for this registration. Derive timing bounds from configured scheduling/transport; use governed-reference transfer consistently and account for feedback age and discretization. State friction/quantization limitations.
- [ ] Before any new simulation, calculate and write the nominal and delayed complex `H`, predicted `ΔH`, units, numerical acceptance region and rationale into the registration. Bound approximation error using the declared timing range and analytical discretization comparison; do not infer tolerance from new outcomes. Report whether the acceptance region includes zero change. If the proposed effect is not distinguishable analytically, revise the protocol now, record why, and freeze one identifiable condition before any trial. No condition changes after the first trial.
- [ ] Use synthetic signals to verify phasor extraction, phase wrapping and exact window boundaries. For `q = 2 sin(ωt + π/6)` and `r = sin(ωt)`, require `H = √3 + i` within numerical tolerance. Missing/non-finite samples or negligible reference amplitude must produce an explicit invalid metric, never zero or a pass.
- [ ] Write paired-grid checks: exactly two delay conditions for each registered seed; duplicates/missing cells fail completeness. Record and report saturation, all faults, governor changes and request suspension for every run.
- [ ] Verify registration checks reject altered constants, windows, seeds and source hashes. Unit tests must use synthetic inputs, not execute the new condition.
- [ ] Commit predictor, harness, tests and numerical registration together **without result artifacts**. Record the commit ID and registration SHA-256. Only then may the next task run.

**Gate:** A reviewer can read a complete numerical prediction, its independent derivation and exact scoring rule in a committed ancestor of the first execution/result record. A plan saying “predict later” does not pass this gate.

## Task 2 — Execute once and interpret honestly

**Create:** `report/task5_prospective_results.json`, `report/task5_prospective_runs.json`, `report/task5_prospective_numbers.md`, `report/figs/task5_prospective.png`.

- [ ] Execute the frozen 10-run grid into a staging directory. Capture start/end times, registration hash, source hashes, software environment and the pre-run commit ID in the result packet.
- [ ] Score only the preregistered primary metric/window. Publish all five pairs, paired differences, the aggregate declared in the registration, predicted region, current/fault/reshaping data and a clear supported/contradicted/inconclusive outcome.
- [ ] Plot the predicted and measured complex-transfer changes, with gain/phase or representative response traces for readability. Keep diagnostic reanalyses clearly separate from the primary result.
- [ ] If the governor reshapes, faults intervene, or the reference becomes unsuitable for the registered harmonic measurement, retain that outcome. Explain the resulting limitation; do not silently disable protection or select an easier window.
- [ ] Revise the mechanism explanation using the measured outcome. If no controller change is justified, explicitly retain the baseline. If delay causes unacceptable response, propose measuring/reducing transport age or tightening admission first; a new controller change requires its own paired validation and is outside the default scope of this pass.
- [ ] Name the hardware confirmation: synchronized command-generation, drive-application and encoder timestamps during a guarded low-amplitude sweep, measuring actual gain/phase and delivered references. Hardware qualification remains proposed.
- [ ] Commit result artifacts separately after the registration commit. Preserve both favorable and unfavorable results.

**Gate:** Task 5 can be fulfilled by a contradicted numerical prediction followed by a reasoned revision. It cannot be fulfilled by changing the prediction after seeing the outcome or presenting arithmetic self-consistency as an observed closed-loop effect.

## Task 3 — Integrate the new evidence and reconcile claims

**Modify:** `run_all.py`, `tools/compare_evidence.py`, the new overview generator in `exp/task5_prospective.py`, generated `report/task5.md`, `report/memo.md`, `report/html_assets/report.md`, `report/html_assets/build_report.py`, both READMEs, `EXECUTION_PLAN.md`, `report/references_and_tools.md` as applicable. Preserve `exp/task5_prediction.py` and its historical registration.

- [ ] Add the prospective harness to full reproduction. Registration is an immutable input: ordinary reproduction must never create or overwrite it. Support clean archives without `.git`; reproduce from registration/source hashes and preserve the committed chronology in the packet narrative.
- [ ] Ensure `tools/compare_evidence.py` covers the new packet explicitly, with current numerical tolerances and source/registration checks. Do not assume its existing file list discovers new artifacts.
- [ ] Make the Task 5 narrative lead with the new prospective study and keep the earlier motor-strength study as a qualified historical/diagnostic result. Keep generated narratives reproducible; do not hand-edit a generated file in a way the next run overwrites.
- [ ] Replace the corresponding memo paragraphs rather than growing the four-page body. Add the new plot to the appendix and HTML; derive the new figure count from the builder rather than guessing it.
- [ ] Update Task 5 completion status only after Tasks 1 and 2 gates pass. Distinguish the brief's completed experiment from still-proposed hardware work and optional future learning research.
- [ ] Inspect declared source sets before publication. If a modified reporting/generator file belongs to existing run specifications, republish affected evidence honestly rather than retaining stale run_ids. Preserve numerical comparisons proving which values changed.

**Gate:** One consistent account across task answer, memo, HTML, execution plan and registered output. No claim that the historical prediction was prospective.

## Task 4 — Reproduce and release

**Create:** `report/packets/task5-prospective-review.md` documenting the fresh review and final commit scope.

- [ ] Run `uv run --frozen python -m unittest discover -s tests` and `uv lock --check`; record the actual final test count.
- [ ] Publish from committed sources, then make a clean `git archive` and run the full `uv run --frozen python run_all.py` there. Compare the new packet and all affected older packets, manifests and figures using the recorded comparison tool. Retain any failed attempt and its correction.
- [ ] Rebuild the HTML and submission PDFs from the final document sources. Verify four memo body pages, one qualification-plan page, legible figures, working local links and matching manifest hashes. Regenerate HTML after the last execution-plan/status edit so its source index is current.
- [ ] An independent reviewer checks registration ancestry/order, predictor independence, metric extraction, all five matched pairs, non-success outcomes, interpretation and final document consistency. Review the actual release commit; distinguish any later packaging-only changes.
- [ ] Prepare the submission from tracked files. Exclude the user's annotated working PDF, temporary outputs and local environments. Include the memo, qualification plan, references/tools note, runnable repository, lockfile and reproduction instructions.

**Final acceptance:** No important unresolved correctness/provenance issue; the new Task 5 experiment demonstrably follows a committed numerical prediction; every outcome is reported; all required artifacts build and fit their page limits. A positive experimental result and physical hardware tests are not prerequisites.

## Effort and completion rule

Target about one hour using the schedule above, existing utilities and bounded parallel review/reproduction. Server use saves only a few minutes of computation; narrow scope and overlapping independent work produce most of the saving. Do not run a new parameter search to obtain a favorable prediction. A failed prediction with a valid measurement and a reasoned revision can complete the brief. An invalid or non-discriminating measurement must remain an explicit limitation; it does not establish the intended closed-loop claim merely because the prediction was registered.
