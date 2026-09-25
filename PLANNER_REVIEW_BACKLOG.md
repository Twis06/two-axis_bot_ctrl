# Planner review backlog — 2026-09-24

This file preserves the findings, rationale and acceptance checks from the planner review. Its original findings below describe the state when they were recorded; the live disposition is here. R6 round 2 passed with minor issues on commit `961b6f2`. The later editorial/qualification changes have been self-verified but were outside that independent review. Declared control experiments and their results remain frozen.

| Item | Current disposition |
|---|---|
| 1–2 · hardware stop state and electrical identification | Addressed in the one-page qualification plan: passive containment is required; blocked-axis `Kt` and guarded moving `Ke` are separate stages. |
| 3 · Task 5 inference | Memo and HTML now narrow the selected post hoc residual metric and its Run B inference. A genuinely prospective closed-loop prediction remains unperformed and is not claimed. |
| 4–6 · `int2`, Run A, B/C figure | Addressed in the memo/HTML and Task 2 answer; original experiment figures and numbers are unchanged. |
| 7 · clean-checkout HTML | Builder no longer requires the ignored brief; the current 19-image HTML has matching source hashes and valid local links. Clean-archive rebuild remains to be checked after this editorial work is committed. |
| 8–9 · progress and release status | B/C path-clock meaning is qualified; current 262-test count and R6 round-two verdict are scoped to the reviewed commit. |
| 10–11 · appendix and historical plot | Memo body remains four pages, with one full plot per appendix page; the early torque budget is labelled as an assumed historical fit. |
| 12–16 · annotated memo | Opening, portable PDF paths, failure hypothesis, static holdability diagram and measured learning-redesign plan are addressed. The diagram is calculated for the explicit INF-P assumption, not a D/E payload estimate. |

**Open before treating the edited tree as a reviewed release:** rebuild the HTML from a clean committed archive and obtain an independent review of the post-R6 editorial/qualification changes. R6 minor code findings r2-1 and r2-2 remain; fixing them would change the registered gate source and require a full L4 republication. R6 minor r2-6 still needs a documented float-leaf counting method. These do not alter the published R6 gate verdict. Future research work (prospective Task 5 test, payload-change challenge and physical qualification) is outside this submission's simulated evidence.

## 1. Correct the hardware stop-state promise — high priority

**Finding.** [`report/hardware_qualification_plan.md`](report/hardware_qualification_plan.md) says every global stop leads to a “safe hold.” The frozen controller uses drive-local damping after communication loss, and the simulated loaded fallback can move up to 42°; [`report/memo.md`](report/memo.md) reports that excursion. Without an independently verified brake or support, a safe position hold is not established.

**Requested change.** State the actual stop state for each fault class. Require an independently protected fixture, verified clearance/end stops or brake, and a measured excursion limit before fault-injection tests. Avoid promising a hold when feedback or actuation needed for it is unavailable. Keep the hardware plan's acceptance criteria consistent with this limitation.

**Verify.** The plan and memo use the same fault-state terminology; no stop condition promises a hold that the tested controller cannot supply.

## 2. Split blocked-axis and moving electrical identification — high priority

**Finding.** Stage 1 of [`report/hardware_qualification_plan.md`](report/hardware_qualification_plan.md) proposes identifying both torque constant `Kt` and back-EMF/voltage convention with the axis blocked. In the stated electrical model, `V = Ri + L di/dt + Ke qdot`; blocked-axis speed is zero, so the `Ke qdot` term cannot identify `Ke` or available voltage at speed.

**Requested change.** Use the blocked fixture for sign, current offset, `Kt`, and stationary electrical checks. Add a separate, guarded low-speed rotation or equivalent electrical test for `Ke` and drive-voltage convention before advancing to higher-speed motion. Keep the test order and stop rules explicit.

**Verify.** Each parameter is measured under conditions in which it affects an observable quantity.

## 3. Keep Task 5's prediction status precise — high priority

**Finding.** [`report/task5.md`](report/task5.md) records that registration and original results first entered git together; ordering is therefore not independently established. The exact numerical matches mostly recheck simulator equations. The extra-current prediction is not separable from total current, and the residual-peak metric was chosen after the results. The brief's “prediction before a new test” requirement is not fully demonstrated by this record. The memo's “failed” shorthand and revised causal explanation can read more strongly than the post hoc metric supports.

**Requested change.** In [`report/memo.md`](report/memo.md), [`report/task5.md`](report/task5.md), and the HTML report, describe the residual result as *the predicted effect was not resolved by this selected peak metric*, while retaining the actual measured value and the diagnostic exact-feed-forward result. Restrict the “estimator transients dominate” inference to this Run B metric and operating point. Mark Task 5 as partially evidenced until a new, separately recorded prospective prediction tests an independently measurable closed-loop response; do not relabel the retrospective correction as prospective.

**Verify.** Every Task 5 summary distinguishes registered arithmetic checks, post hoc diagnostics, untested quantities, and any genuinely prospective test.

## 4. Explain the deterministic `int2` tuning tradeoff — medium priority

**Finding.** [`report/task3_ceiling_numbers.md`](report/task3_ceiling_numbers.md) reports that `int2` improves the held-out median primary error by 42.6% against `int1` and completes 50/50 sequences versus 40/50. It misses the declared phase/gain margin criteria slightly (43.7° nominal PM versus 45° required; 28.1° corner PM versus 30°; 5.7 dB corner GM versus 6 dB). This exclusion is a chosen design rule, not evidence that `int2` is unsafe on hardware.

**Requested change.** Explain in the final recommendation why the frozen `int1` margin reserve is preferred despite `int2`'s simulated tracking/completion gain. Present `int2` as ineligible under the declared margins, not proven unsafe. Carry this tradeoff into hardware qualification or future tuning work; it does not change the narrow rejection of the tested adaptive candidate.

**Verify.** The memo, Task 2/3 answers, and HTML make the same distinction between a design margin and a measured failure.

## 5. Soften Run A's causal shorthand — low priority

**Finding.** [`report/memo.md`](report/memo.md) says Run A's error “points to response, not current capacity.” Its measured 2.1 A peak is below the nominal 3.2 A limit, but the shaped trajectory, voltage utilization, current-command trace, and timing are unknown. [`report/task1.md`](report/task1.md) already states the narrower conclusion well.

**Requested change.** Say that Run A does not show exhaustion of the *nominal current limit* and motivates examining response, compensation, and timing. Retain the caveat that full motion feasibility and electrical limits are unverified.

**Verify.** The memo no longer sounds more certain than Task 1 or the observed summary.

## 6. Clarify the B/C tracking figure without changing its data — low priority

**Finding.** In [`exp/scenarios.py`](exp/scenarios.py), Runs B and C command roll `Hold(0.0)` while yaw follows a ±75° sine at 1.5 and 2.2 Hz. [`exp/task2_eval.py`](exp/task2_eval.py) plots original and governed *roll* references, so their flat zero-degree lines overlap. Actual roll moves slightly because yaw exerts coupling torque. A reduction of the yaw request in C would not make the roll reference oscillate. The current figure is mathematically correct but easy to misread without the yaw stimulus visible or named.

**Requested change.** Add a concise caption or panel annotation: “B/C: roll held at 0°; yaw oscillates at 1.5/2.2 Hz; original and governed roll traces overlap.” Consider a yaw trace only if a caption is insufficient. Update the generated HTML caption and memo figure label to match. **Do not** turn B/C's roll command into a sine wave. A plotting-source change requires republishing the Task 2 figure and rechecking provenance after the active evidence run finishes.

**Verify.** A reader can tell which axis is commanded to move, why the two reference traces coincide, and why the actual roll deviates.

## Decision already supported; no change requested

The R1/R3 evidence supports retaining the deterministic baseline **over the tested adaptive configuration**: the registered median benefit is 0.0% and availability is 37/50, below its gate. Twenty pairs improve and the zero median is explained by late or absent correction. Keep the rejection limited to this candidate and the tested conditions; the incomplete payload-change challenge remains disclosed. Do not restore the earlier blanket claim that learning is generally ineffective.

## Additional overview findings — 2026-09-24, committed snapshot `72c6d56`

The items below came from an independent read-only arithmetic, figure, PDF and HTML audit while R6's clean-snapshot run and other workspace edits were active. Recheck them against the eventual release commit. The central torque arithmetic, A–E medians, Task 3 gate counts and Task 5 paired values agreed with the generated JSON; these items concern delivery and interpretation.

### 7. Make the HTML report rebuildable without the ignored brief PDF — high priority

**Finding.** [`report/html_assets/build_report.py`](report/html_assets/build_report.py) includes `Robotics Controls Technical Assessment.pdf` in the source index and hashes it. [`.gitignore`](.gitignore) excludes that PDF. In a disposable `git archive 72c6d56` snapshot, the PDF is absent; with the report's Markdown dependency stubbed, the builder fails with `FileNotFoundError` for that PDF. The published HTML also links to it, so that link is broken in the delivered checkout even though it works in the author's local workspace. R6's clean `run_all.py` check does not exercise the HTML builder.

**Requested change.** Describe the brief as an outside source without requiring or linking the ignored file. Keep the generated HTML source index and build manifest consistent. Rebuild the HTML and test its documented `uv run --with markdown python report/html_assets/build_report.py` command from a clean committed snapshot; check links against tracked files, not only the local workspace.

### 8. Qualify B/C “net progress” as a clock metric during roll hold — medium priority

**Finding.** [`sim/metrics.py`](sim/metrics.py) defines progress as governor path-clock time divided by request time. Runs B/C command a constant 0° roll hold, so their 100%/97% values do not measure roll distance or speed delivered. Run C separately reduces yaw amplitude to 0.88×. [`report/task2_numbers.md`](report/task2_numbers.md) and the HTML do define the clock metric, but the summary tables still invite a physical-progress reading.

**Requested change.** In Task 2, the memo and HTML, identify B/C progress as a clock/accounting value for a stationary roll request and report yaw delivery beside it. Retain the registered metric and scoring; do not silently replace its values.

### 9. Reconcile release-status numbers after R6 — medium priority

**Finding.** [`report/memo.md`](report/memo.md) and its PDF say 257 tests pass, while [`report/packets/R6.md`](report/packets/R6.md) records 262 after the R6 fixes. The report index and HTML still say R6 is open, appropriately while the clean check is underway. Task 3 and HTML also cite 255 tests at the earlier `039f83b` commit; those historical counts are properly scoped by commit.

**Requested change.** After the final clean run and verdict, update the *current* test count and review status in the memo/PDF, README and HTML. Keep explicitly commit-scoped historical counts if useful. Do not claim R6 passed before its clean comparison completes.

### 10. Enlarge the PDF plot appendix — medium priority

**Finding.** [`report/memo.pdf`](report/memo.pdf) has four text pages plus four appendix pages, satisfying the page limit. The appendix places eight complex PNG figures in table cells; at normal page size several axes, legends and annotations are too small to read. The full-size PNGs and HTML enlargement are readable.

**Requested change.** Re-layout the appendix with one large figure per page, or use landscape pages where needed. Replace the two-column “What it shows | Figure” table with numbered figures and short, paper-style captions. Each caption should identify the scenario/axes, plotted quantities and units, the specific conclusion, and any condition needed to interpret it (for example, B/C's stationary roll command and moving yaw). Keep the technical memo body at four pages; plots and appendices are excluded from the limit. Visually verify the PDF at ordinary page size after rendering.

### 11. Mark the early torque-budget plot as assumed/historical at point of display — low priority

**Finding.** [`report/figs/p0_torque_budget.png`](report/figs/p0_torque_budget.png) plots D/E load bars from an early quasi-static ~1.79 kg fit, although the assessment does not give payload mass and the current Task 1 answer says it is not identifiable. The HTML caption calls it an early analytic budget, but the figure is placed with current Task 1 figures, where the large D/E bars may appear to be measured demand.

**Requested change.** Move it to the historical group or state the assumed fit and exclusion of sweep inertia directly in the figure title/caption. Do not treat the 1.79 kg value as hardware identification.

### Audit checks that passed on snapshot `72c6d56`

- Independent arithmetic from the brief matched the reported 0.448/0.336 N·m torque ceilings, 0.136/0.247 N·m B/C coupling peaks, D bias decomposition and E's 38% RMS increase.
- Recomputed five-seed Task 2 medians, Task 3's 30 exact zero/20 improved pairs and 37/50 availability, the 23/48 active challenge count, and Task 5's paired residual values from the published JSON.
- After R6's clean `run_all.py` finished, six result JSONs had zero non-provenance/non-timing differences against the repository results under the declared numerical tolerance; all 18 generated PNGs were byte-identical.
- The generated HTML manifest's input and output hashes matched current files. Its 18 figures were embedded, and local links resolved in the author's workspace; the ignored-PDF link in item 7 fails in a clean delivery snapshot.
- The memo PDF had four body pages and four plot-appendix pages; the hardware plan PDF had one page. They rendered without clipping. The appendix legibility issue remains item 10.

## Annotated-memo feedback — 2026-09-24

Source: [`report/memo_annotated.pdf`](report/memo_annotated.pdf), pages 1–3 and 5. This section evaluates the *written comments*, not every unlabelled highlight or stray one-letter caret. These are planning decisions; the memo, figures and generated PDFs have not been changed by this pass. Coordinate document rebuilds with the agent finishing R6, and retain the frozen experiment data.

### 12. State the actual contribution and decision logic up front — high priority

**Annotation (page 1).** “key contribution”; “why do you want to state that even you dont recommend?” beside the recommendation against the tested adaptive correction.

**Assessment.** Good editorial concern, but the recommendation itself is necessary: Task 3 explicitly asks whether learning belongs, and the evidence supports rejecting *this candidate* while taking the deterministic baseline to hardware qualification. The present opening gives the decision before explaining what the work contributes, which makes the negative clause sound like the main result.

**Plan.** Rewrite the opening as a short claim–evidence–decision sequence: (1) the contribution is a torque-aware deterministic control/supervision design that distinguishes feasible tracking, reshaped delivery, and unholdable requests; (2) nominal simulated A–C improve, while D/E and unknown-load/fallback limits remain explicit; (3) the tested adaptive correction misses its registered median-benefit and availability gates, so it is not in the recommended build. Keep “hardware qualification” as the next stage, not an implication of hardware validation. Mirror the same framing in the HTML executive summary and Task 3 conclusion. Avoid presenting a general rejection of learning.

**Verify.** In the first paragraph, a reader can identify the positive technical contribution, its demonstrated scope, and the specific reason the learning component is excluded. All reported numbers remain tied to their current evidence.

### 13. Remove machine-specific links from the delivered PDF — high priority

**Annotation (page 1).** The task-document links “wouldn't work since they are the address on my local.”

**Assessment.** Confirmed. PDFKit reads the links to `task1.md`, `task5.md`, R2/R4 and the qualification plan in `memo_annotated.pdf` as absolute `file://<home>/...` URLs. The Markdown links are relative, but `render_pdf.py` supplies a local `<base href=...>` before Chrome prints the PDF. There is no repository remote configured that can substitute as a stable public URL.

**Plan.** Keep relative Markdown links for repository/HTML reading. In the PDF-render path, render external-document references as readable repository-relative paths (for example, `report/task1.md`) without machine-local clickable links. Use actual hyperlinks only when a stable published destination exists; internal PDF anchors are fine. Check all exported PDFs, not only the two annotated task links. Document where the full sources are found, without implying the PDF can open a sibling Markdown file on every reader's machine.

**Verify.** Inspect every PDF link annotation programmatically; none begins with `file:///Users/` or another author-machine path. Printed source paths still identify the right files in a delivered checkout.

### 14. Explain the failure hypothesis and the proposed yaw experiment in plain terms — medium priority

**Annotation (page 1).** “what does this suppose to mean?” on “neither anticipates nor rejects the disturbance torque”; “improve this. what can you get out of this? what would the contribution be” near the proposed yaw sweep.

**Assessment.** Both are valid. The current hypothesis compresses different possibilities (missing feed-forward, feedback bandwidth/delay, saturation) into one sentence and says “within its current limit” although the five observed summaries do not establish the active current-limit mechanism. The experiment lists measurements but does not say what decision they enable. This also overlaps item 5's caution about Run A.

**Plan.** Explain that yaw motion and shifted payloads add roll torque; if the controller predicts too little of that torque or responds too late, roll deviates even when nominal current headroom exists. Present it as a hypothesis, not an identified failure mode. For the roll-held yaw sweep, state the inference path: fit velocity- and acceleration-dependent coupling on some frequencies, predict roll response/current at a held-out frequency, and compare the timing and magnitude of residual error with the measured current limit and voltage. A good fit would justify updating the coupling feed-forward and safe yaw envelope; a poor fit would redirect investigation to latency, calibration, friction or other unmodelled effects. Make explicit that this is a *proposed hardware-identification experiment*, not new evidence already obtained. Carry the shorter explanation into the HTML and Task 1 answer.

**Verify.** The reader can say what will be measured, what each possible outcome means for the controller, and what remains unproved from Runs A–E alone.

### 15. Show static holdability separately from dynamic path reshaping — medium priority

**Annotation (page 2).** A graph of positions that are holdable/reachable versus unable positions would help.

**Assessment.** Good and necessary to explain the central governor distinction. Existing figures show an infeasible run's outcome, but do not display the static torque boundary across roll angle. A single “reachable” region would overclaim: static holdability depends on payload and active current limit, while dynamic reachability also depends on speed, acceleration and voltage. Unknown D/E payload mass must not be drawn as a measured curve.

**Plan.** Add one compact angle-versus-torque (or angle-versus-hold-margin) figure using the stated model. Plot static holding demand against nominal/derated capacity and the governor's conservative budget; shade angles that are statically admissible for the explicitly labelled nominal and assumed INF-P load cases. Mark the 0° INF-P check (about 0.41 N·m demand versus 0.336 N·m derated capacity), and distinguish physical torque capacity from the stricter admission budget. Put “static holdability only; motion may require additional torque/voltage” in the caption. Link this figure from the governor section and the Task 4 infeasibility discussion; keep the existing trajectory/fault timeline for what the controller actually did. If the figure is generated from model equations, mark it **Calculated/assumed**, not observed or simulated hardware.

**Verify.** Recompute selected plotted points from the model, check units/sign and payload convention, and confirm that the plot agrees with the reported INF-P boundary without turning unspecified D/E payloads into facts.

### 16. Turn the learning follow-up into a measured redesign plan — medium priority

**Annotation (page 3).** “what steps can improve this model? what would make the model actually usable. is it the delay the worst stopping factor?”

**Assessment.** Good. The current one-sentence “more reliably usable” condition is too vague to direct a next experiment. The available evidence points to *late or absent eligibility* as the immediate obstacle: 30/50 pairs are bit-identical, 13 never become usable, and 17 first become usable only at 18.3–21.6 s. The modeled estimator worker delay is only 2–8 ms, so it cannot by itself explain a many-second availability problem. This does **not** prove that communication/control delay is unimportant for tracking, nor does it quantify which estimator gate is the dominant cause; that attribution needs a measured breakdown.

**Read-only diagnosis completed.** [`docs/plans/task3-availability-review.md`](docs/plans/task3-availability-review.md) contains a 50-run per-case gate-timing table and an execution brief. An in-memory replay matched every published first-usable time within one 500 Hz host tick. Only **4/50** runs had a usable model before the 9 s test phase; 20 first became usable during it, 13 after it, and 13 never did. Across the grid, 45 reached the 30-aggregate gate, 42 the 60° roll-span gate, 42 the 0.25 cosine-span gate, and all 50 the two Gram gates; only 37 met all gates simultaneously. Five never-usable runs lacked 30 aggregates, while eight had enough aggregates but insufficient angle/feature coverage. Every run that satisfied coverage submitted a fit immediately and published in **4–8 ms**. Representative calibration-hold probes found 250 ms measured roll spans commonly above the frozen **two-encoder-count** stationarity window. This supports an acquisition/coverage bottleneck; loosening the window without measuring bias is not yet justified.

**Plan.** Give an execution agent the linked review brief. On *tuning cases only*, compare longer/explicit diverse-angle calibration dwells with a sensor-aware stationarity detector, measuring accepted-window supply and residual-motion bias before selecting either or a bounded combination. Freeze one candidate and calibration-time budget before a newly registered held-out comparison; charge added time to both candidate and comparator. Keep feed-forward-only correction and the governor/safety bounds unchanged until revalidated. Add a prospective *ready by scoring onset* availability criterion (proposed ≥80%) alongside the original ever-usable and ≥10% paired-benefit gates. Measure coefficient-ramp time and actual command effect, since first usability alone can be functionally too late. Execute the missing payload-change, yaw, feedback-loss and derating challenges with the learned correction actually active. Preserve the frozen candidate's failure as a historical result.

**Verify.** A gate-by-gate timing table supports the chosen intervention; the revised candidate meets the same ≥10% paired median and ≥80% availability gates without safety/completion regression on genuinely new held-out cases. If it does not, retain the deterministic recommendation.

### Annotation handling

- The page-5 request for paper-style figures is accepted in item 10; its captions must explain each plotted element and conclusion while the figures remain readable at normal PDF size.
- The page-1 highlights on the simulation sentence and “voltage use and thermal history,” plus the page-2 unlabelled highlights, carry no separate written instruction. The former informs item 12; uncertainty about voltage/thermal data is already disclosed and should remain.
- The two single-letter page-1 caret comments (`n`, `e`) have no interpretable action. Do not infer a technical requirement from them.
