# Task 5 — prospective motor-strength explanation test

**Status: completed in simulation; hardware confirmation remains pending.**

Prediction registration precedes the new experiment. The exact nominal Run B, Kt×0.90 paired intervention was not present in the prior matched evidence; earlier Kt±15% random/corner cases remain prior context and are not relabeled.

Claim: With yaw feed-forward unchanged, a weaker true motor produces a predictable coupling-torque residual and requires proportionally more current.

Frozen protocol: Run B nominal yaw trajectory, true Kt and Ke multiplied by 0.90, controller parameters unchanged; seeds [21, 22, 23]; window 2.0–8.0 s; controller uses nominal Kt and Ke.

## Registered prediction

The requested 1.5 Hz yaw trajectory has a calculated peak coupling torque of **0.135622 N·m**, equivalent to **0.968730 A** at nominal Kt. With true Kt×0.90 and unchanged nominal feed-forward, the predicted extra ideal current is **0.107637 A** and the uncancelled coupling residual is **0.013562 N·m**. Registered tolerances are ±0.15 A for the component-current check and ±0.01 N·m for the residual calculation.

These are component-level predictions. Measured total current and tracking error also include feedback, friction, transport, voltage, and governor effects.

## Matched results

| Kt ratio | Yaw FF | Seed | Measured peak A | RMS current A | Governed RMS ° | Original RMS ° | Yaw scale | Clips % | Events |
|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1.00 | on | 21 | 1.256 | 0.728 | 0.362 | 0.362 | 1.000 | 0.00 | none |
| 1.00 | off | 21 | 1.054 | 0.580 | 3.363 | 3.363 | 1.000 | 0.00 | none |
| 0.90 | on | 21 | 1.289 | 0.736 | 0.411 | 0.411 | 1.000 | 0.00 | none |
| 0.90 | off | 21 | 1.173 | 0.650 | 3.768 | 3.768 | 1.000 | 0.00 | none |
| 1.00 | on | 22 | 1.204 | 0.723 | 0.222 | 0.222 | 1.000 | 0.00 | none |
| 1.00 | off | 22 | 1.122 | 0.584 | 3.284 | 3.284 | 1.000 | 0.00 | none |
| 0.90 | on | 22 | 1.203 | 0.728 | 0.220 | 0.220 | 1.000 | 0.00 | none |
| 0.90 | off | 22 | 1.261 | 0.654 | 3.677 | 3.677 | 1.000 | 0.00 | none |
| 1.00 | on | 23 | 1.231 | 0.731 | 0.242 | 0.242 | 1.000 | 0.00 | none |
| 1.00 | off | 23 | 1.121 | 0.626 | 3.526 | 3.526 | 1.000 | 0.00 | none |
| 0.90 | on | 23 | 1.257 | 0.735 | 0.289 | 0.289 | 1.000 | 0.00 | none |
| 0.90 | off | 23 | 1.252 | 0.701 | 3.950 | 3.950 | 1.000 | 0.00 | none |

## Prediction versus observation

The logged true coupling peak was 0.135622 N·m across the weakened runs, matching the registered 0.135622 N·m trajectory calculation. The registered **0.107637 A** is an ideal coupling-component increase, not a prediction of the measured total-current maximum. The paired measured total-current peak changed by a median **+0.026 A** (range -0.001 to +0.033 A), because feedback, phase, and the maximum operator contribute to the total trace.
With yaw feed-forward enabled, weakened-motor governed RMS was 0.289° versus 0.242° nominal (paired change +0.047°). With feed-forward disabled under the same weakened motor, governed RMS was 3.768°. No run clipped or generated a fault event. The result supports the qualitative explanation that yaw feed-forward is valuable and motor strength affects the residual, while the component-level current prediction cannot be equated with total measured peak current.

The paired rows preserve the same scenario, seed, request, disturbance realization, transport draw, and initial state. The weakened-motor totals are reported separately from the registered component prediction; no controller retuning or design change was made.

## Hardware follow-up

Hardware confirmation still requires calibrated torque-versus-current identification and a timestamped roll-held yaw experiment with the same stop limits. This simulated result does not certify the drive convention or hardware Kt.
