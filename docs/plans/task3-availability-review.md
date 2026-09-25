# Task 3 estimator availability — planner diagnostic

**Status:** Read-only analysis replay of the frozen candidate, 2026-09-24. This is not a new registered comparison, a redesigned estimator result, or hardware evidence. It does not alter the published Task 3 verdict. **Provenance limit:** the in-memory wrapper used for the replay was not committed, so the per-gate timings below are not reproducible from the repository or by `run_all.py`. The first-usable counts (4 before, 20 during, 13 after the test phase, 13 never) do reproduce from `report/task3_ceiling_results.json`. R6 round 2 reviewed the earlier release commit `961b6f2`; this later diagnostic and its editorial use were outside that independent review.

## What the existing runs reveal

The 50 held-out candidate runs were replayed with the same five loads, two current limits, five seeds and frozen `PayloadEstimator`. An in-memory wrapper observed public estimator snapshots on each 500 Hz host tick; no controller, simulator, or published result was changed. Every replayed first-usable time agreed with the published `report/task3_ceiling_results.json` row within one host tick (2 ms). The five relevant source hashes were unchanged across the replay; the frozen estimator hash was `cefc2b5169bfd66593dbb525ec8f143f0c829dd51c53611e64a9784060aab392`.

| First usable | Runs | Implication |
|---|---:|---|
| Before the 9 s test phase | 4/50 | The planned calibration made a model ready in only 8% of held-out cases. |
| During the 9–19 s test phase | 20/50 | These models missed at least some scored dwells. |
| After 19 s | 13/50 | The correction arrived too late for the test phase. |
| Never | 13/50 | No fit was submitted. |

Across all 50 runs, 45 ever accumulated 30 aggregates, 42 ever reached 60° accepted-angle span, 42 ever reached 0.25 cosine-feature span, and all 50 exceeded both 0.01 Gram-conditioning thresholds. Only 37 satisfied **all** gates at once. In five of the 13 never-usable runs the aggregate count remained below 30; the other eight had enough aggregates but lacked roll span, cosine span, or both. In 29 of the 37 successful runs, aggregate count was among the last gates to cross (two runs had simultaneous last crossings). Once all gates crossed, the worker submitted immediately and published a usable fit within 4–8 ms. No run with full coverage was blocked by the fit or worker.

The most plausible immediate bottleneck is obtaining enough diverse, genuinely stationary dwell data, not the worker's modeled 2–8 ms latency. The calibration schedule visits 0°, −50°, 0°, +50°, 0° with one-second holds, but the estimator admits only 250 ms windows whose measured roll *and* yaw spans each stay within two encoder counts (about 0.044°), followed by 30 aggregates and geometric coverage. In two representative `(0,+0.10)` N·m replays at 3.2 A, minimum/median measured roll span over 250 ms on the four commanded calibration holds was frequently above two counts; one never-usable seed collected just 24 aggregates by the end of the 22 s run. This is a diagnostic association, not proof that simply widening the stationarity gate yields an unbiased or safe model. The existing memo also reports about 0.2° hold hunting, roughly nine 14-bit counts, under another scenario.

A “usable” snapshot is still not the same as a useful correction: publication is slew-limited to 0.10 N·m/s, and the controller uses bumpless transfer. The existing Task 3 result reports 17 late first-use cases whose scored commands remained bit-identical to baseline. Early data acquisition is the first bottleneck to solve; correction ramp and actual command effect must be measured separately.

The estimator's availability gate checks whether a run **ever** becomes usable. That is weaker than being useful during the planned test. A future registered redesign should add a time-to-usable criterion at the start of scoring, while retaining the frozen comparison's 10% paired-median benefit, 80% ever-usable, completion and safety gates. Do not reinterpret the original gate after seeing these results.

## Recommended redesign study (execution by a separate agent)

1. On **tuning cases only**, log each commanded calibration pose, accepted 250 ms windows, aggregate timestamps and angles, stationarity-span failures, near-limit/saturation exclusions, coverage checks, fit submissions, publication, coefficient ramp, and first applied correction during scoring. Use one timebase and distinguish wall time from governor path time. The replay below provides the target failure patterns to explain; it is not tuning data.
2. Compare two bounded acquisition changes on those tuning cases: longer or explicitly requested diverse-angle calibration dwells with a finite time budget, and a sensor-aware stationarity detector that tolerates measured encoder hunting while excluding real movement. Evaluate each alone before combining them. Preserve the measured-current-only observation contract, feed-forward-only correction, coefficient/torque bounds, and unchanged governor and fault logic. Do not select a looser count threshold merely because it makes these held-out rows pass; quantify residual-motion bias, disturbance/current-bias sensitivity and false acceptance with L2-style replay.
3. Select and freeze one candidate and its calibration budget using tuning results. Register fresh held-out loads/seeds **before** running them, including mid-run payload change, yaw disturbance, feedback outage and derating with correction active. Charge any added calibration time to both candidate and deterministic comparator. Report availability by the first scored dwell, time to reach 50% of the fitted bounded correction, active fraction and actual command effect during scoring, paired RMS, completion, current-limit and fault outcomes. Require at least 80% ready by the start of scoring as a proposed additional deployment criterion; retain the original registered gates for comparability.
4. If enough diverse and unbiased data cannot be acquired within that time budget, keep the deterministic recommendation. A different observation model would be a separate registered candidate, not a retrospective repair of this one.

## Per-run gate timing

Times are wall-clock seconds. `—` means the gate was never reached. “30” is first 30 accepted aggregates; “60°” and “cos .25” are first geometric threshold crossings; “usable” is first published usable snapshot. `n`, span and cosine are final training-buffer values, which explain the never-usable rows. The Gram gates passed in every run and are omitted from the table. The table is a diagnostic replay, not a scored outcome table.

| Load (θs, θc) N·m | Limit A | Seed | 30 | 60° | cos .25 | Usable | Final n | Final span ° | Final cos span |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| (-0.06,+0.14) | 2.4 | 101 | 4.938 | 7.004 | 4.726 | 7.010 | 64 | 116.2 | 0.602 |
| (-0.06,+0.14) | 2.4 | 102 | 2.918 | — | 2.514 | — | 64 | 50.1 | 0.351 |
| (-0.06,+0.14) | 2.4 | 103 | 18.288 | 10.188 | 2.230 | 18.296 | 99 | 82.1 | 0.358 |
| (-0.06,+0.14) | 2.4 | 104 | 18.748 | 16.760 | 16.760 | 18.756 | 85 | 66.0 | 0.589 |
| (-0.06,+0.14) | 2.4 | 105 | 10.514 | 12.188 | — | — | 54 | 62.8 | 0.147 |
| (-0.06,+0.14) | 3.2 | 101 | 4.938 | 7.004 | 4.726 | 7.010 | 77 | 116.2 | 0.602 |
| (-0.06,+0.14) | 3.2 | 102 | 2.918 | — | 2.514 | — | 52 | 50.1 | 0.351 |
| (-0.06,+0.14) | 3.2 | 103 | 18.278 | 10.188 | 2.230 | 18.286 | 100 | 82.1 | 0.358 |
| (-0.06,+0.14) | 3.2 | 104 | 18.708 | 16.694 | 16.694 | 18.716 | 81 | 66.0 | 0.589 |
| (-0.06,+0.14) | 3.2 | 105 | 10.514 | 12.188 | — | — | 58 | 62.8 | 0.147 |
| (+0.00,-0.18) | 2.4 | 101 | 12.620 | 12.220 | — | — | 41 | 60.4 | 0.139 |
| (+0.00,-0.18) | 2.4 | 102 | 20.328 | 13.004 | 6.214 | 20.332 | 74 | 77.9 | 0.331 |
| (+0.00,-0.18) | 2.4 | 103 | 20.974 | 12.448 | 6.342 | 20.982 | 45 | 79.1 | 0.344 |
| (+0.00,-0.18) | 2.4 | 104 | 21.586 | 16.642 | 16.642 | 21.594 | 32 | 96.2 | 0.602 |
| (+0.00,-0.18) | 2.4 | 105 | 12.732 | — | — | — | 66 | 32.1 | 0.135 |
| (+0.00,-0.18) | 3.2 | 101 | 12.620 | 12.220 | — | — | 38 | 60.4 | 0.139 |
| (+0.00,-0.18) | 3.2 | 102 | 20.282 | 13.004 | 6.214 | 20.286 | 76 | 77.9 | 0.331 |
| (+0.00,-0.18) | 3.2 | 103 | 20.852 | 12.448 | 6.342 | 20.860 | 56 | 79.1 | 0.344 |
| (+0.00,-0.18) | 3.2 | 104 | 20.784 | 16.692 | 16.692 | 20.792 | 39 | 96.2 | 0.602 |
| (+0.00,-0.18) | 3.2 | 105 | 12.732 | — | — | — | 65 | 32.1 | 0.135 |
| (+0.00,+0.10) | 2.4 | 101 | — | — | 20.076 | — | 28 | 49.4 | 0.343 |
| (+0.00,+0.10) | 2.4 | 102 | 10.652 | 6.864 | 2.912 | 10.656 | 95 | 130.9 | 0.589 |
| (+0.00,+0.10) | 2.4 | 103 | 10.718 | 10.198 | 8.204 | 10.726 | 97 | 80.3 | 0.352 |
| (+0.00,+0.10) | 2.4 | 104 | 8.470 | 6.206 | 8.340 | 8.478 | 90 | 131.2 | 0.588 |
| (+0.00,+0.10) | 2.4 | 105 | 11.050 | 12.392 | 16.588 | 16.596 | 97 | 95.6 | 0.578 |
| (+0.00,+0.10) | 3.2 | 101 | — | — | 20.190 | — | 24 | 49.4 | 0.343 |
| (+0.00,+0.10) | 3.2 | 102 | 10.652 | 6.864 | 2.912 | 10.656 | 91 | 130.9 | 0.589 |
| (+0.00,+0.10) | 3.2 | 103 | 10.718 | 10.198 | 8.204 | 10.726 | 98 | 80.3 | 0.352 |
| (+0.00,+0.10) | 3.2 | 104 | 8.470 | 6.206 | 8.340 | 8.478 | 115 | 131.2 | 0.588 |
| (+0.00,+0.10) | 3.2 | 105 | 11.050 | 12.392 | 16.592 | 16.600 | 93 | 95.6 | 0.578 |
| (+0.00,+0.18) | 2.4 | 101 | 21.618 | 14.248 | 14.248 | 21.624 | 33 | 98.7 | 0.637 |
| (+0.00,+0.18) | 2.4 | 102 | 13.004 | 16.202 | 2.232 | 16.206 | 89 | 64.7 | 0.564 |
| (+0.00,+0.18) | 2.4 | 103 | 20.020 | 10.292 | 8.688 | 20.028 | 38 | 95.2 | 0.562 |
| (+0.00,+0.18) | 2.4 | 104 | 20.220 | 6.318 | 2.332 | 20.228 | 42 | 99.4 | 0.383 |
| (+0.00,+0.18) | 2.4 | 105 | 12.948 | 12.204 | 16.574 | 16.582 | 48 | 95.2 | 0.568 |
| (+0.00,+0.18) | 3.2 | 101 | — | 14.248 | 14.248 | — | 23 | 98.7 | 0.637 |
| (+0.00,+0.18) | 3.2 | 102 | 13.004 | 16.438 | 2.232 | 16.442 | 76 | 64.8 | 0.564 |
| (+0.00,+0.18) | 3.2 | 103 | 19.932 | 16.768 | 8.670 | 19.940 | 42 | 65.0 | 0.562 |
| (+0.00,+0.18) | 3.2 | 104 | 20.194 | 6.318 | 2.332 | 20.202 | 44 | 99.4 | 0.383 |
| (+0.00,+0.18) | 3.2 | 105 | 12.948 | 12.204 | 16.554 | 16.562 | 54 | 95.2 | 0.569 |
| (+0.06,+0.14) | 2.4 | 101 | — | — | — | — | 28 | 31.0 | 0.137 |
| (+0.06,+0.14) | 2.4 | 102 | 10.154 | 10.154 | 2.300 | 10.158 | 69 | 114.6 | 0.613 |
| (+0.06,+0.14) | 2.4 | 103 | 19.922 | 14.410 | 14.410 | 19.930 | 48 | 127.6 | 0.591 |
| (+0.06,+0.14) | 2.4 | 104 | 10.364 | 6.212 | 8.210 | 10.372 | 73 | 96.9 | 0.358 |
| (+0.06,+0.14) | 2.4 | 105 | 16.368 | 14.198 | 14.198 | 16.376 | 35 | 127.9 | 0.590 |
| (+0.06,+0.14) | 3.2 | 101 | — | — | — | — | 24 | 31.0 | 0.137 |
| (+0.06,+0.14) | 3.2 | 102 | 10.154 | 10.154 | 2.300 | 10.158 | 79 | 113.5 | 0.595 |
| (+0.06,+0.14) | 3.2 | 103 | 19.938 | 14.410 | 14.410 | 19.946 | 47 | 127.6 | 0.591 |
| (+0.06,+0.14) | 3.2 | 104 | 10.364 | 6.212 | 8.210 | 10.372 | 115 | 131.1 | 0.586 |
| (+0.06,+0.14) | 3.2 | 105 | 16.362 | 14.198 | 14.198 | 16.370 | 35 | 127.8 | 0.590 |
