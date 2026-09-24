# Task 5 — motor-strength explanation test (R2 retrospective correction of the analysis)

Prediction registration precedes the new experiment. The exact nominal Run B, Kt×0.90 paired intervention was not present in the prior matched evidence; earlier Kt±15% random/corner cases remain prior context and are not relabeled.

Claim: With yaw feed-forward unchanged, a weaker true motor produces a predictable coupling-torque residual and requires proportionally more current.

Frozen protocol: Run B nominal yaw trajectory, true Kt and Ke multiplied by 0.90, controller parameters unchanged; seeds [21, 22, 23]; window 2.0–8.0 s; controller uses nominal Kt and Ke.

## Registered prediction

The requested 1.5 Hz yaw trajectory has a calculated peak coupling torque of **0.135622 N·m**, equivalent to **0.968730 A** at nominal Kt. With true Kt×0.90 and unchanged nominal feed-forward, the predicted extra ideal current is **0.107637 A** and the uncancelled coupling residual is **0.013562 N·m**. Registered tolerances are ±0.15 A for the component-current check and ±0.01 N·m for the residual calculation.

These are component-level predictions. Measured total current and tracking error also include feedback, friction, transport, voltage, and governor effects.

## Matched results

| Kt ratio | Yaw FF | Yaw info | Seed | Peak current A [2,8) s | RMS current A [2,8) s | Governed RMS ° [2,8) s | Original RMS ° [2,8) s | Yaw scale | Clipped % [2,8) s | Events (whole run) |
|---:|:---:|---|---:|---:|---:|---:|---:|---:|---:|---|
| 1.00 | on | estimate | 21 | 1.256 | 0.728 | 0.374 | 0.374 | 1.000 | 0.00 | none |
| 1.00 | off | estimate | 21 | 1.054 | 0.580 | 3.511 | 3.511 | 1.000 | 0.00 | none |
| 1.00 | on | plan (diagnostic) | 21 | 1.134 | 0.692 | 0.345 | 0.345 | 1.000 | 0.00 | none |
| 0.90 | on | estimate | 21 | 1.289 | 0.736 | 0.432 | 0.432 | 1.000 | 0.00 | none |
| 0.90 | off | estimate | 21 | 1.173 | 0.650 | 3.936 | 3.936 | 1.000 | 0.00 | none |
| 0.90 | on | plan (diagnostic) | 21 | 1.149 | 0.698 | 0.373 | 0.373 | 1.000 | 0.00 | none |
| 1.00 | on | estimate | 22 | 1.204 | 0.723 | 0.243 | 0.243 | 1.000 | 0.00 | none |
| 1.00 | off | estimate | 22 | 1.122 | 0.584 | 3.498 | 3.498 | 1.000 | 0.00 | none |
| 1.00 | on | plan (diagnostic) | 22 | 1.074 | 0.688 | 0.193 | 0.193 | 1.000 | 0.00 | none |
| 0.90 | on | estimate | 22 | 1.203 | 0.728 | 0.241 | 0.241 | 1.000 | 0.00 | none |
| 0.90 | off | estimate | 22 | 1.261 | 0.654 | 3.919 | 3.919 | 1.000 | 0.00 | none |
| 0.90 | on | plan (diagnostic) | 22 | 1.104 | 0.694 | 0.251 | 0.251 | 1.000 | 0.00 | none |
| 1.00 | on | estimate | 23 | 1.231 | 0.731 | 0.274 | 0.274 | 1.000 | 0.00 | none |
| 1.00 | off | estimate | 23 | 1.121 | 0.626 | 3.758 | 3.758 | 1.000 | 0.00 | none |
| 1.00 | on | plan (diagnostic) | 23 | 1.138 | 0.695 | 0.260 | 0.260 | 1.000 | 0.00 | none |
| 0.90 | on | estimate | 23 | 1.257 | 0.735 | 0.328 | 0.328 | 1.000 | 0.00 | none |
| 0.90 | off | estimate | 23 | 1.252 | 0.701 | 4.214 | 4.214 | 1.000 | 0.00 | none |
| 0.90 | on | plan (diagnostic) | 23 | 1.155 | 0.700 | 0.314 | 0.314 | 1.000 | 0.00 | none |

## Prediction versus measurement (R2 retrospective correction)

Rows above and below use one window, [2, 8) s, for every windowed quantity; whole-run safety events are in the JSON `whole_run` field. This is a **retrospective correction** of the original analysis (same frozen controller, conditions and seeds, replayed), not a new prospective experiment. The registration is unchanged.

| Prediction | Basis | Predicted | Unit | Quantity actually measured | Measured | Tolerance | Verdict |
|---|---|---:|---|---|---:|---|---|
| peak coupling torque of the registered yaw trajectory | registered | 0.135622 | N m | logged true coupling peak, weakened runs (median) | 0.135622 | none registered | **consistency check only** |
| peak ideal coupling current at nominal Kt (registered primary component metric) | registered | 0.968730 | A | ideal true-plant coupling current peak, nominal motor (median) | 0.968730 | +/-0.15 A | **consistency check only** |
| DIAGNOSTIC (post hoc): commanded coupling FF current of the frozen baseline | post hoc | 0.968730 | A | controller's commanded coupling FF current peak, nominal motor (median) | 1.159736 | +/-0.15 A applied for reference only | **diagnostic (peak outside tolerance)** |
| extra ideal coupling current at Kt x 0.90 | registered | 0.107637 | A | ideal true-plant coupling current, weak minus nominal (median, paired) | 0.107637 | +/-0.15 A registered; it includes 0 A, so it cannot discriminate the predicted change from no change | **not tested** |
| coupling residual left by unchanged nominal FF | post hoc operationalization | 0.013562 | N m | increase of the measured coupling-residual PEAK, weak minus nominal, FF on (median, paired); metric chosen after the results | -0.000736 | +/-0.01 N m (registered) | **failed** |
| qualitative: yaw feed-forward remains valuable with the weaker motor | post hoc criterion | — | deg | governed RMS, FF off minus FF on, weak motor (median, paired) | 3.678444 | sign in every pair (criterion chosen afterwards) | **supported** |
| qualitative: a weaker motor leaves more tracking error for feedback | post hoc criterion | — | deg | governed RMS, weak minus nominal motor, FF on (median, paired) | 0.053977 | sign in every pair (criterion chosen afterwards) | **not uniform** |

- *peak coupling torque of the registered yaw trajectory:* the simulator's coupling uses the same equation; this checks arithmetic, not the hardware explanation.
- *peak ideal coupling current at nominal Kt (registered primary component metric):* the like-for-like quantity is again the prescribed model's arithmetic.
- *DIAGNOSTIC (post hoc): commanded coupling FF current of the frozen baseline:* the causal estimate's sustained gain is 1.046-1.047 (least squares, about +5 %) while its peak ratio is 1.172-1.200: the +20 % is transient overshoot at the coupling extrema, not a sustained gain error.
- *extra ideal coupling current at Kt x 0.90:* the ideal value is the prescribed model's arithmetic; the commanded coupling FF current does not rise (paired change +0.0000 A, nominal FF unchanged), and the extra current feedback supplies for the coupling component is not separable in the total current.
- *coupling residual left by unchanged nominal FF:* paired range -0.0027 to +0.0003 N m; by alignment 0/2/4/6 ms: +0.0017, +0.0003, -0.0007, -0.0021 (fails at every alignment); RMS residual change -0.0003 N m. The residual PEAK is dominated by the estimator's transient error (about 0.03 N m), so a difference of peaks cannot resolve an added 0.0136 N m component. DIAGNOSTIC plan mode (exact FF): +0.0123 N m, within tolerance: the prediction holds when FF equals the true coupling.
- *qualitative: yaw feed-forward remains valuable with the weaker motor:* FF off is worse by +3.505 to +3.886 deg in the 3 pairs.
- *qualitative: a weaker motor leaves more tracking error for feedback:* 2 of 3 pairs increase; paired range -0.002 to +0.058 deg (the original analysis had the same negative pair).

What this does and does not show: yaw feed-forward matters (every pair). The registered numerical predictions are arithmetic on the prescribed model (consistency checks) or were not measurable (extra current). Under the one operationalization of the residual chosen afterwards, the frozen baseline's residual increase is not resolved: the causal estimate's transient peak error dominates the residual peak, while the exact-FF plan-mode diagnostic shows the predicted increase. The weaker motor does not raise tracking error in every pair. The registration audit is in the JSON (`registration_audit`). The registered extra-current prediction was not tested as a measurable quantity, and its tolerance could not have discriminated it from zero. No closed-loop tracking-error prediction was registered, so none is claimed; a new prospective one would need its own registration first.

The paired rows preserve the same scenario, seed, request, disturbance realization, transport draw, and initial state. The weakened-motor totals are reported separately from the registered component prediction; no controller retuning or design change was made.

## Hardware follow-up

Hardware confirmation still requires calibrated torque-versus-current identification and a timestamped roll-held yaw experiment with the same stop limits. This simulated result does not certify the drive convention or hardware Kt.
