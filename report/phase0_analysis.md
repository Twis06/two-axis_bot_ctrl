# Phase 0 — Paper Analysis

> Historical investigation. See [Task 1](task1.md) for the current diagnosis and its uncertainty qualifications, and [the task index](README.md) for the assessment answers.

All numbers come from `analysis/phase0.py` (full tables: [phase0_numbers.md](phase0_numbers.md)). No simulation has been run yet, and no gains have been tuned.

## 0. Assumptions
1. The run summaries come from one unknown controller. Its gains, rates, and feed-forward terms are not known.
2. B/C: yaw is a pure sine, q_y = 75°·sin(ωt), and roll is held near 0°. That makes q̈_r ≈ 0 and gravity small.
3. A: moves are minimum-jerk, and peak current coincides with peak acceleration.
4. D/E: the sweep is a symmetric sinusoid of ±80°. Its speed is unknown, so the static fits are upper bounds.
5. CAN latency applies to the command path only; a conservative case also adds it to the feedback path. The compute/release delay is taken as ≤ Ts/4.
6. Current "clipping" means the *command* hit the limit. That is why Run B can clip at a measured peak of 3.1 A: the 1.2 ms lag filters the command.
7. The "tracking goal" is not specified. I assume ≤ 2° RMS and ≤ 5° peak (to be revisited).

## 1. What the motor can do in each run

Capacity is 0.448 N·m at 3.2 A and 0.336 N·m at 2.4 A.

| Run | Model demand (worst case: + \|d\| + τc) | Share of limit | Observed | Verdict |
|---|---|---|---|---|
| A ±45° moves | static 0.125 N·m; inertial 0.12–0.22 N·m (a ≈ 30–55 rad/s²) | 66 % at the observed 2.1 A | 2.8° RMS error, no saturation | **Feasible with margin.** The error is not caused by the torque limit. |
| B yaw 1.5 Hz | coupling 0.136 N·m (0.97 A); worst 0.226 N·m (1.61 A) | 50 % of 3.2 A, 67 % of 2.4 A | 3.1 A peak, 8 % clipped | **Feasible.** The controller uses **1.9×** the current the model needs. |
| C yaw 2.2 Hz | coupling 0.247 N·m (1.76 A); worst 0.337 N·m (2.41 A) | 75 % of 3.2 A, **100 % of 2.4 A** | 31 % at the limit, watchdog trip | **Feasible at 3.2 A; infeasible when derated.** |
| D payload, ±80° | τg′ unknown; the fits need 0.5–1.8 kg shifted 35 mm | ≈ 100 %+ at the sweep ends | +4.6° mean error, 38 % limited | **Marginal to infeasible.** The load is uncompensated. |
| E D at 2.4 A | same load, capacity −25 % | the static fit predicts ~53 % limited | 12.6° RMS, repeated entry and exit from saturation | **Infeasible as requested.** The request should be reshaped or rejected. |

## 2. Leading explanation: three mechanisms, in priority order

**(i) B/C: the yaw disturbance is rejected by feedback alone, through about 6 ms of lag.**
- The error scales with the modelled coupling torque:
  - C/B disturbance ratio: 1.82
  - C/B RMS error ratio: 1.86
  - C/B peak error ratio: 1.80
- In both runs the ratio of coupling torque to error is the same, about 1.3 N·m/rad.
- The coupling model therefore explains the *shape* of the failure, and the loop behaves roughly linearly across 1.5–2.2 Hz.
- The logs imply a sensitivity |S| of 0.29 (B) and 0.61 (C). A loop designed at the delay limit would give 0.15 and 0.29, so the existing loop is about **2× softer than the delay allows**.
- Even at that limit, feedback alone leaves a peak error of **3.1° (B) and 5.3° (C)**. Retuning gains cannot meet the goal. That matches the brief's "don't start by tuning gains".
- Yaw kinematics are observable, so feed-forward is possible. Allowing 20 % coefficient error plus a 4 ms-old yaw sample, the residual is 24–26 % of the disturbance, giving about **0.75° (B) and 1.4° (C)** peak.
- Feed-forward tolerates delay well here because ωT ≈ 0.055 rad at 2.2 Hz with 4 ms of latency.

**(ii) D/E: an uncompensated, asymmetric gravity load.**
- A symmetric sweep against an odd load (τg′·sin q) gives near-zero *mean* error. The signed +4.6° mean therefore needs a one-signed load, such as a lateral centre-of-mass shift that adds m·g·s·cos q.
- The bias carries 26 % of the mean-square error.
- The two mass estimates disagree: 0.5–1.2 kg from the mean error and 1.8 kg from the static saturation fit. The gap suggests sweep dynamics also contribute. **The mass cannot be identified from the summaries**, so it will be treated as a swept parameter.
- A 1 kg-class shift also raises inertia by roughly 30–55 %.

**(iii) E: windup against an infeasible demand, not a linear instability.**
- The PI + double-integrator loop is conditionally stable. However, its lower gain margin is 35–43 dB, so saturation would have to cut the effective gain by more than 30 dB to destabilise the linear loop.
- Repeated entry into and exit from saturation fits integrator windup plus a request beyond capacity better. **Action:** anti-windup plus request limiting.

**Run A is consistent with (i) without any disturbance.** With 34 % headroom, the error is J·a divided by an effective stiffness of about 1.1–2.1 N·m/rad. That points to missing inertia and gravity feed-forward, not a lack of torque.

**Coupling from heat:** The saturated time in Run C alone dissipates ≥ 5.7–7.1 W, while the disturbance only requires 2.8–3.5 W with ideal feed-forward. If derating is driven by RMS current, poor disturbance rejection may be what *causes* the derated state in Run E.

## 3. Other causes, kept only where they could change the decision

| Alternative | Evidence against it | Why it could matter |
|---|---|---|
| The coupling model is wrong (e.g. a cable spring in q_y, or different coefficients) | Errors scale with the modelled torque in both B and C | Feed-forward from (q̇y, q̈y) would under-cancel. It would need identification, or a learned/adaptive residual. |
| Current spikes driven by noise (derivative of the quantized velocity) | Run A has the same sensing and never clips | The fix would be the velocity filter, not feed-forward. |
| Voltage or back-EMF limiting | The runs need ≤ 5 rad/s. The worst case (20 V, R +25 %, SVPWM) allows 31 rad/s at 3.2 A, and **10 rad/s during a full-current reversal** | Not the cause of A–E, but it **does bind** for fast, large moves at 20 V. It must be included in the request envelope and monitored. |
| The existing loop is resonant near 2 Hz | The same torque/error ratio at 1.5 and 2.2 Hz argues against sharp peaking | If present, retuning would help more than predicted above. |

## 4. What the summaries cannot prove
- Whether saturation caused the error or the error caused the saturation. There is no time ordering.
- The gains, rates, and feed-forward terms actually in use. Every |S| inference assumes a linear loop.
- The payload mass and the direction of the shift, the sweep speed, and the sign convention of the "signed mean".
- The latency actually present, including whether bursts coincided with the peaks. The peak/RMS ratio of about 2.7 (a sine gives 1.41) says the error is episodic, but not why.
- Whether the logged current was measured or commanded, and when derating started.
- Bus voltage and duty cycle. The voltage conclusion is a calculation only.

## 5. The single most informative experiment
**Repeat Run B at three yaw frequencies (e.g. 0.75, 1.5 and 2.2 Hz, reducing the amplitude where needed), holding roll at 0°, logging at 1 kHz:**
- i_cmd and i_meas
- q_r and q_y
- CAN timestamps
- V_bus

Fit i·Kt − (inertia, friction, gravity) against (q̇y, q̈y, q_y).
- **If** the fitted coefficients are within ±20 % of the model and the q_y term is negligible, yaw feed-forward is the right fix. Phase 2 proceeds as planned.
- **If not,** the coupling model is wrong, and the design shifts toward identifying the coupling. Only in that case would a learned residual earn its place.

The same data also gives the true distribution of loop delay and shows whether voltage is binding.

## 6. Decisions carried into Phase 2
| Item | Decision | Basis |
|---|---|---|
| Controller rate | **500 Hz** (at 250 Hz the delay grows by 1.5 ms) | delay budget: 4.8 ms worst-normal pure delay + 1.2 ms lag |
| Feedback target | ωc ≈ **44 rad/s (7 Hz)**, lead ratio α = 16, Kp ≈ 1.9 N·m/rad | PM 46° at nominal, 39° during a burst, 35° in the conservative case, 45° with J +55 %; GM ≥ 7.8 dB; 85 mA per encoder count |
| Rejected option | α = 25 (9 Hz) | 178 mA per count of noise, and PM drops to 32° in the conservative case |
| Feed-forward | J·q̈_ref + τ̂g(q) + τcouple(q̇y, q̈y) + friction on the **reference** velocity | a velocity LSB of 0.19 rad/s is 10× the friction scale, so tanh(v̂/0.02) would chatter at ±0.29 A |
| Gravity/payload | an online estimate of the load, clamped, as the candidate "learning" component | Run D's bias; the mass cannot be identified offline |
| Saturation | back-calculation anti-windup; a torque-budget request check | Run E |
| Request envelope | yaw ±75° is allowed up to **2.3 Hz at 3.2 A** but only **1.8 Hz at 2.4 A**, with a 20 % reserve. Beyond that: reshape the yaw or accept a roll error, with the error declared | Run C when derated is the "don't track" case |
| Voltage | a speed/current limit that accounts for voltage (SVPWM, 20 V, R +25 %) | the reversal headroom is 10 rad/s |

## 7. Changes to the plan
- The earlier estimate of an "8–15 Hz" crossover was too optimistic. With the lag counted properly, it is **4–9 Hz depending on the lead ratio**, and 7 Hz is the chosen design point.
- I dropped the hypothesis that saturation-induced conditional instability causes Run E, because the lower gain margin is too large. Windup is now the leading explanation.
- The payload mass will be swept rather than fitted.

Figures: `figs/p0_torque_budget.png`, `figs/p0_bandwidth_vs_delay.png`, `figs/p0_yaw_envelope.png`, `figs/p0_payload_fit.png`.
