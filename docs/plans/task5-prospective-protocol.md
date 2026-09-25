# Task 5 — prospective closed-loop prediction: protocol (frozen before execution)

**Status:** registered (version 2), not yet run. The numbers below are copied from `report/task5_prospective_registration.json`, which `python -m exp.task5_prospective --register` wrote and which is committed together with the predictor, scorer and tests, **before any simulation of this condition**. The first run happens in a later commit.

## Why this test

- **The requirement:** the brief asks for a numerical prediction written before a new test of our choice.
- **Why a new study:** the earlier motor-strength study (`report/task5.md`, packet R2) has no git evidence of prediction-before-test ordering, and its closed-loop quantities were chosen after the results.
- **What this adds:** one new, measurable closed-loop test.

## Prior knowledge (disclosed)

- **Frequency response at the nominal command delay:** Packet 4B measured it at 0.5, 1, 2, 4 and 6 Hz, with bench points at 2.2, 8 and 10 Hz. So the nominal 3 Hz condition is interpolable from published data. It was not used to fit anything below.
- **CAN-latency study:** Packet 4B's delay-only bench varied CAN latency with a stationary reference. No run has changed the **current-command delay**, and no run has used a 3 Hz sine.

## Condition

| Setting | Value |
|---|---|
| Roll request | 5° sine at 3 Hz, 1 s ramp, 12 s run |
| Yaw | held at 0 |
| Plant | nominal, bounded disturbance d(t) = 0 (a disclosed simplification for this mechanism test); friction, encoder quantization, current lag, voltage limits, governor and fault logic on |
| Controller / supervisor | frozen `BaselineController()` / `DriveSupervisor()` defaults, baseline fingerprint `7d857df507c389c9` |
| Change | current-command delay 1 ms (nominal) vs 5 ms |
| Seeds | 301–305, the same seed for both delays in each pair |

**Primary quantity:**

- **Per run:** H = fundamental(actual roll) / fundamental(governed reference `c_q_c`), each from a sin/cos/intercept least-squares fit over [4, 12) s (24 cycles).
- **Primary result:** the arithmetic mean over the five seeds of ΔH = H(5 ms) − H(1 ms).
- **Secondary, reported for every run:** gain and phase, error against the original request, governor limiting, clipping, peak current, events, suspension, rejection and fallback.

## Prediction (Calculated, from the model and configuration only)

**Model.** The linearized loop at q = 0:

- **Friction:** Coulomb friction replaced by its describing function at the predicted velocity amplitude.
- **Controller:** the discrete PI × lead (Tustin, 500 Hz, integrator updated after use).
- **Feed-forward:** inertia, viscous, gravity and 80% friction on the governed reference.
- **Actuator:** the 1.2 ms current lag.

**Timing from configuration:**

- **Command path:** host compute 0.5 ms, then CAN 0.6–1.8 ms (mean 1.2) plus expected bursts, then half a drive tick (0.5 ms), then the delay FIFO, then half the 2 ms host hold. That gives 4.28 ms nominal and 8.28 ms delayed.
- **Feedback age:** CAN plus half a drive tick, 1.78 ms.

| | Predicted |
|---|---|
| H(1 ms) | 1.0391 − 0.0078j (1.039∠−0.43°) |
| H(5 ms) | 1.1058 − 0.0276j (1.106∠−1.43°) |
| **ΔH** | **+0.0667 − 0.0199j** (0.070∠−16.6°): about +0.54 dB gain, −1.0° phase |
| Acceptance | a disc of radius **0.0264** around ΔH (model bound 0.0164 + declared floor 0.010); **excludes 0** (|ΔH| = 2.6 × radius) |

**Model bound.** The largest deviation of ΔH over the declared grid:

- command-path and feedback-age accounting ±0.5 ms;
- CAN bursts on or off;
- discrete vs continuous controller;
- the plant's friction describing-function gain × 0.5–1.5. The controller's own friction feed-forward is known exactly and is not varied.

**Floor.** The 0.010 floor covers effects the linear model omits: friction harmonics, quantization and integrator transients. It is a declared judgment, not derived or fitted.

**Why this direction.** The extra 4 ms would lag the feed-forward by about 4.3° at 3 Hz. But it also removes about 6° of phase margin near the 4.46 Hz crossover, and that raises the closed-loop peaking. So the model predicts mainly a gain increase.

## Outcome rules (fixed now)

- **Supported:** the measured mean ΔH lies inside the disc.
- **Contradicted:** it lies outside.
- **Inconclusive:** any registered cell is missing or duplicated, any transfer is invalid, or any run has a latched fault event, suspension or rejection, or drive fallback **inside the scoring window**. The drive's normal start-up fallback, the first ticks before the first valid command, is not a fault. Any later fallback latches an event.

Every run is published whatever the outcome. No seeds, conditions, windows or metrics are added or changed after the first run.

## Revision before any run: version 1 → version 2

An independent pre-run review of version 1 (commit `1f4d982`) found one blocking defect.

- **The defect:** the scorer counted drive fallback over the whole log. The drive is always in fallback for 2–3 ticks before the first valid command, so every run would have been scored inconclusive.
- **Also adopted (non-blocking suggestions):**
  - Friction uncertainty is applied to the plant only.
  - The registration now binds a SHA-256 of the scoring and outcome functions (`scorer_sha256`), so any later change to them makes the run refuse.
- **Unchanged:** the central prediction. Only the model bound, and hence the radius (0.0217 → 0.0264), changed.
- **Ordering:** both versions were committed before any simulation of this condition.

## Disclosed limitations of the prediction (from the pre-run review)

- **Absolute gain may be low:** against Packet 4B's frictionless bench, the model's absolute |H| is low by about 0.025 at 4 Hz, consistent with 0.5–1 ms of unmodelled effective delay. Only ΔH is scored, and ΔH moves about 0.0024 per ms of extra delay.
- **Reference sampling:** `c_q_c` is updated at 500 Hz and logged at 1 kHz, an effective 0.5 ms reference advance. It is common to both arms, about 0.0007 in ΔH.
- **Not in the outcome rules:** governor limiting and clipping are recorded for every run but are not outcome rules. Neither is expected at this demand (about 0.17 N·m against a 0.308 N·m budget).
