# Independent review — Task 3 estimator kernel

Review only. The parent selected a small-model executor for L1; do not assume passing tests establish compliance. Read `task3-learning.md` sections 3–5 and the implementation report. Review only the estimator, its new tests, and report. Do not edit shared baseline files or broaden this into a controller redesign.

## Highest-priority checks

1. **Causality and age:** training uses only observations received before job submission. Measurement, receipt, submission and publication times remain distinct. Old samples/duplicate ticks cannot refresh health, estimate age, or publication budget.
2. **Invalidation:** stale/nonfinite/faulted/saturated input immediately disables correction and invalidates pending jobs. A job submitted before a fault cannot reinstate trust after it. Recovery requires the specified fresh continuous window.
3. **Learning versus application:** healthy movement prevents training but may use a still-current model. Lack of new training expires the model after the specified training-age deadline. Detect a payload-change mismatch before old data can dilute it.
4. **Identification and bounds:** use two actual sine/cosine features and measured-current residual; no ground truth or integral-state fitting. Check angular/conditioning gates, finite linear solves, parameter norm, output torque cap, residual rejection and publication slew. A projected parameter vector that no longer fits the data cannot be reported as an accurate fit without rechecking the residual.
5. **Worker behavior:** scheduling <=50 Hz, one pending job, delayed visibility, overdue-result discard, no sleeps, and bounded stored history/work. Distinguish modeled asynchronous availability from actual host execution time.
6. **Tests:** exercise public behavior and a known synthetic model. Include counterexamples that would fail if a gate were removed. Watch for tests that only inspect internal flags or reproduce the implementation's own equations as their sole oracle.
7. **Scope:** only the three authorized L1 files are written. No unchanged baseline guarantees, closed-loop benefit claims, physical payload identification claims, new dependencies or hidden simulator integration.

## Report format

Return prioritized Critical / Important / Minor findings with file and line, a precise triggering sequence or reproducer, expected versus actual behavior, and required regression coverage. Distinguish plan defects from implementation defects. If there are no important findings, say so explicitly and list what was actually checked.

The review may run bounded unit tests and synthetic probes. Do not rerun expensive full experiments. Disclose runtime/environment limitations and do not infer passing tests when execution was unavailable.
