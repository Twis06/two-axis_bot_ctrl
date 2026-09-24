# Task 3 execution ledger

## Authorized scope

Primary agent: planning, delegation, acceptance decisions, and review only.
Execution: Luna or Terra at maximum reasoning effort.
Initial packet: L1, standalone estimator and tests; no baseline integration.
Plan: [task3-learning.md](task3-learning.md). Brief: [task3-l1-brief.md](task3-l1-brief.md).

## Attempts

1. `task3_l1_luna` — gpt-6-luna, max effort. Stopped with a tool-reported usage limit before producing the assigned implementation/report files. Not completed or reviewed.
2. `task3_l1_terra` — gpt-5.6-terra, max effort. Dispatched the same bounded assignment as the user-authorized fallback. Awaiting execution report.

## Planning decisions

- The governor/recovery gate remains in force. Building the standalone estimator is independent work; it does not mean Task 2 is accepted or that learning is effective.
- The initial adaptive candidate changes feed-forward only. The existing shared load-model hook also changes governor feasibility, so it cannot be used unchanged at integration.
- Parameter bounds and fit-quality gates do not establish identified physical mass or statistical confidence. Biased and angle-correlated disturbances belong in the audit.
- No implementation is claimed until files, tests and an execution report exist; no closed-loop benefit is claimed until the matched comparison passes review.

## L1 progress and clarified decision

Terra reported requirements understood and is writing the failing behavioral tests.

Ruling: the fresh-dwell mismatch gate compares against the last full accepted, projected fit before refitting, not the intentionally slew-limited applied coefficients. Otherwise initial bounded publication would look like a model mismatch. Preserve separate full-fit and applied diagnostics and recheck residual after coefficient projection. Cost if wrong: delayed detection of a load change; the L2 audit must test this explicitly. Mismatch resets the training history and invalidates pending/use until normal coverage and health gates pass again.

Terra milestone: ran `uv run --quiet --with numpy python -m unittest tests.test_payload_estimator -v`; the initial red outcome was missing `ctrl.payload_estimator`. Implementation is underway. This is an execution-agent report, not an independently verified passing result.

Terra checkpoint: focused suite 11/11 passed; full suite reported 107 passed with 1 existing skip. These are executor-reported results pending independent review. Other repository work has increased the suite size since the earlier 40-test baseline checkpoint.

Ruling: fault/stale invalidation resets the applied coefficient ramp to zero, without accumulating disabled-time slew credit. Old full-fit values may remain diagnostic but cannot restore applied torque immediately; the normal recovery/validity gates still apply. Cost if wrong: slower re-acquisition after outages. This favors bounded recovery over instant compensation; L3 must implement bumpless total-command handling for enable and disable. Executor is adding a narrow regression before freezing for review.

## Concurrent baseline progress

A newer `report/packets/2A.md` now records the governor contract gate as passed after independent review. It also records a serious carried-forward Packet 2B issue: loaded-outage watchdog/re-arm cycling, and the unresolved no-yaw-authority fallback case. Therefore Task 3's integration gate is still closed. The current learning plan's earlier numeric baseline snapshot is historical context, not the final frozen comparator. L4 must consume the eventual accepted baseline manifest.

L1 executor froze files and supplied report/hashes. Independent `task3_l1_review` dispatched on gpt-5.6-terra at max effort. Report text states `Ran 108 tests ... OK (skipped=1)`, meaning 108 total including the skip; use that precise count rather than the earlier shorthand “108 passed plus one skip.” L2 audit brief prepared but not dispatched before review acceptance.

Independent review checkpoint: 12 focused tests passed, but reviewer reproduced two important issues: duplicate/out-of-order bad-health packets bypass invalidation, and the time-window dwell deque lacks a cardinality bound. Full line-referenced report is pending before fix dispatch.

Ruling: retain the conservative health contract even for ineligible training data. A received fault/saturation/invalid-status packet must invalidate application and pending work; ordinary duplicate/out-of-order data remains ignored without refreshing health. Cost if wrong: an obsolete fault packet can cause unnecessary abstention; the upstream timestamp/sequence-filtered interface and later L2 audit must characterize that tradeoff. Bounded storage/work is required regardless of nominal sampling rate.

## Completion update

The attempt and checkpoint entries above are historical. The completion update below supersedes their earlier “pending” wording.

The initial independent-review findings were fixed in the three authorized L1 files. Regression tests cover hard-status precedence over duplicate/out-of-order timestamps, the 256-sample raw-dwell overload policy, and reservation of finite physical timestamps before rejecting nonfinite payload fields. The focused estimator suite now runs 15 tests; the latest full repository suite ran 196 tests with zero failures.

L2 was executed after the fixes using the public estimator API, 500 Hz replay-like observations, and seeds 201–205. The audit produced 60 rows across the twelve cases in `docs/plans/task3-l2-audit-brief.md`; every structural safety/causality check passed. It records the expected abstention and bias limits, disables on a feedback gap and payload mismatch, rejects pending fault results, and expires a model after 30 s without new training data.

Historical ruling before L4: do not adopt the adaptive feature yet, but do not reject it before the registered comparison. The kernel audit establishes bounded behavior but no closed-loop benefit. The known-load ceiling then showed 76.9% median paired improvement for the feed-forward-only oracle with 50/50 held-out completions, so the missing adaptive comparison was decision-critical. Packet 4B also contains physically infeasible loaded cases (`INF-P`: 0.41 N·m static load versus 0.336 N·m derated capacity) that a feed-forward correction cannot make feasible; this is a capacity boundary, not a general rejection of feasible-load learning. The frozen deterministic baseline remained the current submission controller while L3 review and the registered adaptive L4 comparison were pending.

## L3/L4 completion update

The feed-forward-only adapter passed 7 focused contract tests. The registered L4 harness then ran 357 manifested simulations using the exact frozen L1 estimator configuration, paired loads/limits/seeds, and the frozen baseline fingerprint. The adaptive candidate achieved a 0.0% median paired reduction against `int1`, obtained a usable estimate in 37/50 held-out runs (74%, below the 80% gate), lost no comparator-completed sequences, and added no watchdog trips, suspensions, or request rejections. The feed-forward-only oracle still achieved 76.9% median paired reduction and 50/50 completion, so the model class has headroom even though this online candidate did not realize it.

Ruling: reject this frozen adaptive candidate for the assessment and retain the deterministic baseline. The rejection is now evidence-based on the registered benefit and availability gates; it is not a claim that every future estimator or calibration design must fail. A redesigned candidate requires a new registered comparison.
