# L2 audit brief — estimator limits, not controller benefit

**Prerequisite:** independent L1 review accepted or open findings explicitly resolved. Do not start this audit against an implementation still being fixed. The parent owns that gate.

**Owner:** small-model execution agent. Allowed new outputs: `exp/task3_estimator_audit.py`, `report/task3_estimator_audit.md`, `report/task3_estimator_audit.json`, and optional `report/figs/task3_estimator_*.png`. Do not change the kernel to improve audit outcomes; report a defect and return it for a separately reviewed fix.

Read the estimator plan and L1 report. All cases below are synthetic/replay-like observations. They are not robot simulations, hardware observations, or closed-loop benefit evidence.

## Frozen protocol

Use the public observation/tick/snapshot/correction API. Base extra coefficients are `(0.08, 0.16)` N·m. Generate nominal current from `0.120 sin(q) + tau_extra(q) - d`, divided by `0.140`; the sign matches the plant convention where +d assists motor torque. Both fitting data and true coefficients belong only to the observation generator/evaluator, never the estimator.

Use 500 Hz physical samples, quantize both positions to 14-bit counts, and deliver at nonnegative delayed receipt times. Normal delays are independent seeded 0.6–1.8 ms; allow out-of-order delivery explicitly. Tick at actual simulated host times and do not hand the estimator an observation before its receipt. Preserve the executed event ordering in the manifest.

Use finite moves with stationary endpoint dwells, not instantaneous synthetic position jumps for the main audit. Starting at 0°, use 1 s minimum-jerk moves and 1 s dwells at −50°, 0°, +50°, 0°. For moving segments, either generate dynamically consistent current or deliberately mark them as untrusted dynamic residual challenges; the estimator must not train on them. Maintain healthy measurement delivery while testing application at unseen angles −65°, −30°, +30°, +65°. Report whether the estimate remains valid and prediction errors using full-fit versus applied/ramped values separately.

Seeds: 201–205. Do not tune kernel settings on these outcomes.

## Cases

1. Ideal extra-gravity law, quantized sensing, no d: coefficient/prediction sanity.
2. Bounded sinusoidal d of amplitude 0.05 N·m at 0.4 Hz and seed-dependent initial phase.
3. Constant d of +0.05 N·m: report non-identifiability/bias, not expected accurate mass recovery.
4. Current measurement offset +0.10 A, then −0.10 A: calibration-bias sensitivity.
5. Narrow angular coverage within ±15°: required abstention.
6. Adequate angle coverage but dwell duration 150 ms: required abstention.
7. Healthy moving roll/yaw while dynamic current is arbitrary but unsaturated: no new training weight; prior valid model can remain usable only within its stated age/health rules.
8. A 60 ms feedback gap during a pending fit, followed by healthy data: zero correction during stale interval, invalidated old job, full recovery window, and restarted publication ramp.
9. Duplicate/replayed packets with fresh receipt times: no false freshness/training credit.
10. Payload changes from `(0.08,0.16)` to `(-0.04,0.04)` after the initial model is learned: fresh-dwell mismatch must disable correction before old training averages conceal it. Report detection and relearning times and abstention.
11. A saturation/fault flag during the pending worker interval: no pre-fault result can reinstate correction.
12. Stop training but maintain healthy moving measurements beyond 30 s: estimate expires based on training data age, not publication/receipt recency.

## Report

For each seed/case record the source hash, kernel settings, event-generation parameters, valid training count, first usable time, disabled duration, full-fit coefficient error, unseen-angle maximum prediction error, applied torque/cap/ramp compliance, late/stale application count, and reason transitions. Report actual calculation wall time separately from modeled 2–8 ms completion delay. Avoid comparing wall times across unrecorded environments.

Structural violations—stale application, wrong generation publication, exceeding bounds, training during disallowed motion, or missing required abstention—fail the audit and return to L1 review. Bias in fundamentally unidentifiable cases must be reported honestly, with its maximum torque consequence and an explicit recommendation for L3; do not label it statistical confidence or quietly remove the case.

Do not change the final Task 3 adoption answer. The audit can establish kernel behavior and limitations only. Any closed-loop integration remains gated on Task 2B acceptance.
