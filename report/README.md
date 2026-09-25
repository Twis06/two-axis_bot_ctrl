# Assessment answers

Read the [interactive HTML report](assessment_report.html) for a consolidated answer to Tasks 1–5, all 18 figures, expandable evidence, and learning / prediction charts. It records the remaining gaps explicitly. Open directly in a browser; no server or network is required. Build instructions: [HTML report README](html_assets/README.md).

Current execution priorities and agent handoffs: [EXECUTION_PLAN.md](../EXECUTION_PLAN.md).

Read the answers in the order of the five tasks in the [assessment](../Robotics%20Controls%20Technical%20Assessment.pdf).

| Assessment task | Answer | Current state |
|---|---|---|
| 1. Understand the failure | [task1.md](task1.md) | Complete working answer; distinguishes observations, calculations, and hypotheses |
| 2. Build a baseline | [task2.md](task2.md) | Frozen deterministic baseline ([Packet 2C](packets/2C.md): independent review PASS WITH ISSUES, findings resolved); governor contract ([2A](packets/2A.md)) and fault-class recovery / causal yaw information ([2B](packets/2B.md)) reviewed |
| 3. Decide whether learning belongs | [task3.md](task3.md) | L1/L2 audited, L3 tested, and 357-run L4 comparison rescored (R1) under a machine-audited gate (R3). The tested candidate fails on benefit and availability; the payload-change challenge is missing (`incomplete`); baseline retained |
| 4. Make evidence | [task4.md](task4.md) | Metrics/provenance and matched 4B evidence; all current evidence republished from committed sources and clean-snapshot reproduced ([R4](packets/R4.md)); historical tables are labelled |
| 5. Test your explanation | [task5.md](task5.md) | Registered component-level prediction tested in simulation; analysis retrospectively corrected ([R2](packets/R2.md)). Two checks are consistency-only, one is not tested, one fails and one is supported. The closed-loop prediction and hardware confirmation remain |

The task documents are the primary narrative. The phase documents preserve the development history and detailed calculations. Where a historical diagnosis is stronger than the evidence permits, Task 1 supplies the qualified interpretation.

## Evidence conventions

- **Observed:** only the five summaries in the assessment. There are no raw hardware time series in this project.
- **Calculated:** consequences of the supplied model under stated assumptions.
- **Simulated:** outputs recorded in the existing generated reports; these are not additional hardware observations.
- **Proposed:** work not yet performed.

Task 2 evidence is regenerated on the frozen baseline; see [current numbers](task2_numbers.md) and [robustness](task2_robustness.md). Each row cites a run_id resolved in [task2_runs.json](task2_runs.json). Historical phase reports and Task 4 tables retain their original recorded results.

## Supporting material

| Material | Location |
|---|---|
| Original personal notes | [note.md](../note.md) |
| Development plan | [PLAN.md](../PLAN.md) |
| Initial analytic investigation | [phase0_analysis.md](phase0_analysis.md), [generated calculations](phase0_numbers.md) |
| Simulator assumptions and reconstruction | [phase1_sim.md](phase1_sim.md), [generated results](phase1_numbers.md) |
| Frozen baseline evidence | [Task 2 numbers](task2_numbers.md), [robustness](task2_robustness.md), [trial data](task2_results.json), [run manifests](task2_runs.json) |
| Execution packets (changes, reviews, gates) | [2A](packets/2A.md), [2B](packets/2B.md), [2C](packets/2C.md), [4A](packets/4A.md), [4B](packets/4B.md); corrective [R1](packets/R1.md), [R2](packets/R2.md), [R3](packets/R3.md), [R4](packets/R4.md) |
| Historical baseline design and evaluation | [phase2_baseline.md](phase2_baseline.md), [generated results](phase2_numbers.md) |
| Generated plots | [figs/](figs/) |
| Task 3 estimator audit, L4 and challenges | [audit report](task3_estimator_audit.md), [audit rows](task3_estimator_audit.json), [L4 numbers](task3_ceiling_numbers.md), [challenges](task3_challenges_numbers.md) |
| Task 5 prediction record | [registration](task5_registration.json), [results](task5_results.json) |

## Remaining submission work

These working answers are not yet the final four-page memo. Condense the answers into the memo. Also prepare the one-page hardware qualification plan and the references/reused-code/automated-tools note. The final comparison must report delivered motion as well as tracking error when a request is reshaped.
