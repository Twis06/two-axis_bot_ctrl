# Task 3 L2 — standalone estimator audit

This audit uses synthetic/replay-like timestamped observations through the public estimator API. It is not a plant simulation, hardware observation, or closed-loop benefit result.

Protocol: 500 Hz, seeds [201, 202, 203, 204, 205], modeled worker delay 2–8 ms, source hash `cefc2b5169bfd66593dbb525ec8f143f0c829dd51c53611e64a9784060aab392`.

## Case summary

| Case | Usable seeds | Final reasons | Max coefficient error (N·m) | Max unseen prediction error (N·m) | Structural violations |
|---|---:|---|---:|---:|---:|
| ideal | 5/5 | usable | 7.430572871987097e-07 | 7.427570172802156e-07 | 0 |
| bounded_disturbance | 5/5 | usable | 0.05418329150840637 | 0.0519398204538148 | 0 |
| constant_disturbance | 5/5 | usable | 0.058345546641491185 | 0.05465628593846744 | 0 |
| current_bias | 5/5 | usable | 0.01894013755944664 | 0.0186302884594532 | 0 |
| narrow_coverage | 0/5 | awaiting_estimate | — | — | 0 |
| short_dwell | 0/5 | awaiting_estimate | — | — | 0 |
| moving | 5/5 | usable | 7.430572871987097e-07 | 7.427570172802156e-07 | 0 |
| feedback_gap | 5/5 | usable | 7.430572871987097e-07 | 7.427570172802156e-07 | 0 |
| duplicates | 5/5 | usable | 7.430572871987097e-07 | 7.427570172802156e-07 | 0 |
| payload_change | 5/5 | residual_mismatch | 0.1625484388647317 | 0.1578616300743425 | 0 |
| pending_fault | 5/5 | usable | 7.161526883337092e-07 | 7.157303671401216e-07 | 0 |
| expiry | 5/5 | recovery_dwell_required | 7.430572871987097e-07 | 7.427570172802156e-07 | 0 |

## Interpretation

- Ideal quantized sensing checks convergence and unseen-angle prediction under the fixed gravity-like law.
- Sinusoidal and constant disturbance, current-offset, and payload-change cases are bias/identifiability checks. Their errors are reported as limitations, not converted into confidence intervals.
- Narrow coverage and short dwells are required abstention cases.
- Moving, feedback-gap, duplicate, and pending-fault cases check that dynamic or stale data cannot train or re-enable an old result.
- Expiry is based on the newest training measurement; fresh moving packets do not refresh an old model.

No controller integration, matched comparator, or adoption claim is made by this packet.
