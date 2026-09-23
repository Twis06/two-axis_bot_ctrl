# Task 3 — Decide whether learning belongs

**Status:** Provisional decision only. No adaptive estimator or comparative generalization result is currently presented as completed work.

## Current answer

Keep the deterministic baseline as the reference design. The supplied coupling law already provides a structured way to compensate yaw disturbance, so the summaries do not justify adding a general learned torque policy. The most plausible adaptive addition is a bounded estimate of payload gravity torque, because the payload is uncertain and the baseline currently handles it by slowing down substantially.

The existing baseline delivers E at approximately 48% of requested path speed, with 2.93° RMS error against its governed reference. This motivates an investigation; it does not prove that adaptation will recover speed safely.

## Candidate to evaluate

Represent gravity load as `theta_s * sin(q_roll) + theta_c * cos(q_roll)`. Estimate these coefficients from current and motion history using only measurements available at each update. Separate gravity estimation from inertial and friction residuals; otherwise the estimator may incorrectly explain acceleration torque as payload gravity.

| Question from the assessment | Proposed answer |
|---|---|
| What does it observe? | Timestamped roll motion, measured current, yaw kinematics, active limits, and known commanded motion |
| What does it change? | Gravity feed-forward and, only with conservative uncertainty handling, the governor's load prediction |
| What is its rate? | At most 50 Hz; asynchronous computation and zero-order hold |
| When is it ignored? | Stale/invalid estimates, insufficient excitation, saturation, faults, or poor residual consistency |
| How is it bounded? | Physically justified coefficient and update-rate limits; deterministic current and fault limits remain authoritative |

These are design requirements for a candidate, not descriptions of implemented behavior. Physical coefficient bounds and excitation criteria still need definition. Freeze or revert updates when the data cannot distinguish gravity from other loads. A stale estimate must not cause the governor to assume extra torque capacity.

## Required comparison before adoption

Compare the fixed baseline, a reasonably retuned integral controller, and the structured estimator on identical held-out payload/COM cases, including derating and timing faults. Keep the trajectory requests and safety rules identical; document any difference in model information available to each method.

Measure tracking against both original and governed requests, delivered speed/amplitude, RMS current, saturation duration, faults, and estimator staleness. An adaptive controller only earns its place if it improves useful delivered motion or error without degrading limit/fault behavior on untuned cases.

## Decision rule

Adopt adaptation only after a predeclared comparison demonstrates a repeatable practical advantage. If it does not, retain fixed feed-forward and conservative request shaping, and state that learning was investigated but not justified. Better payload identification or evidence of repeatable model mismatch would change that decision.
