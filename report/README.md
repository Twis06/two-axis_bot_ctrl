# Assessment answers

Current execution priorities and agent handoffs: [EXECUTION_PLAN.md](../EXECUTION_PLAN.md).

Read the answers in the order of the five tasks in the [assessment](../Robotics%20Controls%20Technical%20Assessment.pdf).

| Assessment task | Answer | Current state |
|---|---|---|
| 1. Understand the failure | [task1.md](task1.md) | Complete working answer; distinguishes observations, calculations, and hypotheses |
| 2. Build a baseline | [task2.md](task2.md) | Hardening in progress; governor blocker and loaded recovery review remain |
| 3. Decide whether learning belongs | [task3.md](task3.md) | Provisional decision; adaptive comparison not yet performed |
| 4. Make evidence | [task4.md](task4.md) | Simulator and baseline evidence available; final-design comparison pending |
| 5. Test your explanation | [task5.md](task5.md) | Prospective test protocol; prediction and new test pending |

The task documents are the primary narrative. The phase documents preserve the development history and detailed calculations. Where a historical diagnosis is stronger than the evidence permits, Task 1 supplies the qualified interpretation.

## Evidence conventions

- **Observed:** only the five summaries in the assessment. There are no raw hardware time series in this project.
- **Calculated:** consequences of the supplied model under stated assumptions.
- **Simulated:** outputs recorded in the existing generated reports; these are not additional hardware observations.
- **Proposed:** work not yet performed.

Task 2 has been re-evaluated after controller hardening; see [current numbers](task2_numbers.md). Historical phase reports and Task 4 tables retain their original recorded results.

## Supporting material

| Material | Location |
|---|---|
| Original personal notes | [note.md](../note.md) |
| Development plan | [PLAN.md](../PLAN.md) |
| Initial analytic investigation | [phase0_analysis.md](phase0_analysis.md), [generated calculations](phase0_numbers.md) |
| Simulator assumptions and reconstruction | [phase1_sim.md](phase1_sim.md), [generated results](phase1_numbers.md) |
| Current hardened baseline evidence | [Task 2 numbers](task2_numbers.md), [trial data](task2_results.json) |
| Historical baseline design and evaluation | [phase2_baseline.md](phase2_baseline.md), [generated results](phase2_numbers.md) |
| Generated plots | [figs/](figs/) |

## Remaining submission work

These working answers are not yet the final four-page memo. Complete the learning decision and prospective prediction test, then condense the answers into the memo. Also prepare the one-page hardware qualification plan and the references/reused-code/automated-tools note. The final comparison must report delivered motion as well as tracking error when a request is reshaped.
