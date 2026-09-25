# Safe control for a coupled two-axis robot — technical memo

**Recommendation.** Take the frozen deterministic baseline (fingerprint `7d857df507c389c9`) to hardware qualification, and do not deploy the tested adaptive load correction.

- **In simulation:** the baseline tracks the A–C scenarios to 0.3–0.7° RMS with no faults and slows the loaded D/E requests rather than saturating blindly.
- **Infeasible requests:** it rejects, stops or suspends them instead of pretending to track.

**Labels:** *Observed* = the brief's five summaries only; *Calculated* = the supplied model; *Simulated* = our simulator, not hardware; *Proposed* = not yet done.

`python3 run_all.py` reproduces everything from a clean checkout. The full answers are in [task1](task1.md)–[task5](task5.md).

## 1. Understanding the failure

**What the motor permits (Calculated).**

- **Torque ceiling:** K<sub>t</sub>I<sub>max</sub> = 0.448 N·m at 3.2 A and 0.336 N·m derated.
- **Run A has headroom:** its 2.1 A peak is 0.294 N·m, 66% of the ceiling. At 45°, gravity plus the friction and disturbance allowances need only 0.175 N·m. So A's 2.8° RMS error is a response problem, not a capacity problem.
- **Yaw coupling (±75°):** 0.008 q̇<sub>y</sub> + 0.0008 q̈<sub>y</sub> peaks at 0.136 N·m at 1.5 Hz (B) and 0.247 N·m at 2.2 Hz (C). Including the 0.05 N·m disturbance and friction allowances, B needs 50% of nominal capacity. C needs 75% of nominal but 100.3% of derated: holdable at 3.2 A, marginal at 2.4 A.
- **Payload:** the 35 mm centre-of-mass shift adds 0.343·m N·m. The mass is not given, so D/E feasibility cannot be settled from the summaries.

**Leading explanation (hypothesis).** The controller neither anticipates nor rejects the disturbance torque within its current limit.

- **B→C:** modelled coupling rises 1.82×; observed RMS rises 1.86× and peak error 1.80×.
- **D:** its +4.6° signed mean is 26% of the mean-square error, consistent with an unmodelled gravity load.
- **E:** 25% less torque gives 38% more error, assuming E keeps D's payload, which the brief does not state.

**What the summaries cannot prove:** the original controller and whether it has an integrator (E's cycling does not prove windup), the payload mass and direction, trajectory timing, what "clipped" counts, voltage use and thermal history. Our reconstructed legacy controller matches the RMS ordering only to about 25% and misses the peaks and the sign of D's mean. It is a consistency check, not identification.

**The experiment that would most change this view:** hold roll near zero and sweep yaw in frequency within a qualified envelope. Log synchronized encoders, current command and measurement, active limit, timestamps and bus voltage. Fit the roll torque residual against yaw velocity and acceleration, and validate at a held-out frequency.

## 2. Baseline controller

| Element | Choice and reason |
|---|---|
| Rates | 500 Hz position loop, the brief's maximum. Phase loss is dominated by delay (1 ms command delay, 0.6–1.8 ms CAN, 1.2 ms current lag; 7 ms design, 11 ms burst corner), so a faster loop gains little. 1 kHz drive-side supervisor. |
| Feedback | PI × lead, C(s) = 0.7553(1 + 2.8/s)(1 + s/7)/(1 + s/112), Tustin-discretized; 4.46 Hz crossover. |
| Feed-forward | Nominal inertia, gravity and 80% friction on the *governed* reference. Yaw coupling from a causal Kalman estimate of the yaw encoder, predicted 4 ms ahead; no future yaw plan. |
| Estimates | 14-bit encoder, 0.022° per count. One count differentiated at 500 Hz is 0.19 rad/s, ten times the friction speed scale, so velocity is filtered and friction compensation uses reference velocity. |
| Saturation | Feed-forward-priority clamp, conditional-integration anti-windup, target clamped to the latest 3.2/2.4 A limit after the delay queue. |
| Governor | Admits motion within 0.8·K<sub>t</sub>I<sub>lim</sub> − 0.05 N·m (nominal load model). |
| Faults | Communication loss → drive-local damping; re-arm after 50 ms of fresh aligned commands. Error > 12° for 40 ms → latched fault, host catch, request suspended until replanned. 3 latches in 30 s → lockout. Overspeed and thermal trips. |

**When to reshape, derate or reject (governor rules):**

- **Reshape** (slow the path clock) when dynamic torque exceeds the budget but every pose is holdable.
- **Derate** the budget as soon as the current limit drops.
- **Restrict or reject** poses whose static holding demand exceeds capacity; slowing cannot fix a static overload.
- **Request yaw reduction,** or suspend with a coordinated stop, when coupling exceeds roll authority.
- **Never** shift a stationary reference to absorb a disturbance.

**Why it should stay well behaved.**

- **Margins (Calculated):** 49.4° phase margin and 13.5 dB gain margin at 7 ms; 32.1° and 6.1 dB at the 11 ms corner with J −30% and K<sub>t</sub> +15%; 37.8 ms pure-delay margin.
- **Stress tests (Simulated):** 60 ms outages in either or both directions, burst storms, 5% message loss, the combined corner, and 20 V with R +25% are event-free or re-arm automatically. The 20 V fast sweep is reshaped to 58% progress.
- **Quantization:** about 31 mA of current jitter in Run B (7.6 mA at 24 bits), with no instability. Hunting at hold is about 0.2° of friction stick-slip.
- **Not proven:** stability under deep saturation with an unknown load, the main residual risk given negative gravity stiffness.

## 3. Learning: evaluated and rejected

**Candidate.** A bounded two-parameter load residual, θ<sub>s</sub> sin q + θ<sub>c</sub> cos q.

- **What it observes:** delayed measured roll/yaw, measured current, the active limit and health timestamps, during healthy, settled, yaw-stationary 250 ms dwells.
- **Timing:** it updates at ≤ 50 Hz on a non-blocking worker (2–8 ms latency) and discards results older than 100 ms.
- **What it changes:** feed-forward only, ≤ 0.20 N·m. The governor, limits and faults keep the nominal model.
- **When it is ignored:** on stale, invalid, faulted, saturated, mismatched, under-covered or expired data.

**Registered comparison (Simulated).** We fixed the protocol, the untuned held-out loads (5 loads × 2 limits × 5 seeds), the metric (RMS over the first 0.5 s of each test dwell) and the gate before testing. The gate requires ≥ 10% median paired improvement over the best retuned deterministic comparator, ≥ 80% availability, and no safety or completion regression.

| 50 held-out pairs | Median primary | vs `int1` | Completed |
|---|---:|---:|---:|
| `int1`: retuned comparator (= frozen baseline) | 2.130° | — | 40/50 |
| `int2`: faster integrator, fails margin criteria | 1.157° | +42.6% | 50/50 |
| **Adaptive candidate** | 2.088° | **0.0%** | 40/50 |
| Known-load feed-forward oracle (not deployable) | 0.499° | +76.9% | 50/50 |

- **The 0.0% is exact:** the candidate is `int1` plus the learned term on the same seed. In 30 of 50 pairs no correction reached the scored samples (never usable: 13; first usable at 18.3–21.6 s: 17), so they are bit-identical to `int1`.
- **When it was on in time, it helped:** 20 pairs improved, 8 of them by 42–79%, and none got worse (mean +9.5%, not registered).
- **Gate:** fails on benefit and on availability (74%). All safety criteria pass across the held-out runs and 96 registered challenge runs (yaw B/C, feedback loss, derating). Mid-run payload change was not tested.
- **Incomplete runs:** the 10 in each variant reach every waypoint but overshoot the ±65° range by 5.65–6.89° (5° allowed).

**What would change the choice:** an estimator that becomes usable reliably during calibration, since the oracle shows the load law is worth about 77%. It would have to pass a newly registered held-out comparison, including payload change. No feed-forward can hold a static load above capacity.

## 4. Evidence

**Simulator.** The plant is integrated at 10 kHz, with:

- the current lag, the command delay, and CAN latency with bursts and loss;
- 14-bit encoders and multirate timing;
- current limits and voltage limiting (R 1.8 Ω ± 25%, L 0.45 mH ± 20%, K<sub>e</sub> = K<sub>t</sub>, bus down to 20 V);
- uncertainty in J ± 30%, K<sub>t</sub> ± 15%, friction, payload, coupling and latency.

| Scenario (5-seed median) | Legacy RMS | Final governed RMS | Original-request RMS | Progress | At limit | Tracked |
|---|---:|---:|---:|---:|---:|---:|
| A shaped ±45° | 3.50° | 0.68° | 0.74° | 100% | 0.0% | 5/5 |
| B yaw 1.5 Hz | 3.57° | 0.27° | 0.27° | 100% | 0.0% | 5/5 |
| C yaw 2.2 Hz | 7.64° | 0.40° | 0.40° | 97% | 0.0% | 5/5 |
| D payload ±80° | 10.23° | 4.10° | 76.7° | 83% | 1.9% | 0/5 |
| E D at 2.4 A | 12.02° | 4.25° | 78.5° | 51% | 5.5% | 0/5 |

- **Legacy** is a reconstruction; both controllers are simulated.
- **D/E:** the governed error is small only because the requests are slowed. The original-request error shows the sacrificed timing, and D/E count as not tracked (tracked = ≥ 95% progress, ≤ 2° RMS, no fault; a project threshold).

**Right answer is not to track:**

- **INF-P:** 1.2 kg at +35 mm, derated, needs 0.41 N·m of static torque against 0.336 N·m available. The nominal-model baseline cannot know this: it attempts the move, trips, catches and suspends, a reported limitation. A truth-informed governor rejects such a load outright.
- **INF-R:** roll hold under 2.8 Hz yaw. The governor reduces yaw to 0.36× and holds roll within ±1.1°.
- **Loaded M2:** trips and stays suspended rather than cycling.
- **Fallback:** local damping does not hold against gravity; a loaded 100 ms outage moves the axis up to 42°.

**Reproducibility.** A `git archive` snapshot with no git history ran all 10 steps. It matched the published evidence exactly: 0 value differences and 18/18 figures byte-identical ([R4](packets/R4.md)). 257 unit tests pass.

## 5. Testing the explanation

**Registered prediction (Calculated).** In Run B, weaken the true K<sub>t</sub> and K<sub>e</sub> by 10% with the controller unchanged. Predicted: coupling peak 0.1356 N·m (0.969 A), extra ideal current +0.108 A (±0.15 A), and an uncancelled coupling residual of +0.0136 N·m (±0.01).

**Result (Simulated: 3 seeds, feed-forward on/off pairs, 2–8 s).** A windowing and unit bug in our first analysis was found and corrected retrospectively ([R2](packets/R2.md)).

- **Consistency checks only:** the coupling and ideal-current matches re-check the model's own arithmetic.
- **Not tested:** the extra current is not separable in the total current, and its tolerance includes 0.
- **Failed:** the residual increase was −0.0007 N·m, because the causal yaw estimate's transient error (about 0.03 N·m) swamps it. With exact feed-forward (a diagnostic) the predicted +0.0123 N·m appears.
- **Supported:** feed-forward stays valuable, and removing it costs 3.5–3.9°.
- **Not uniform:** the weaker motor raised error in only 2 of 3 pairs.

**Revised explanation:** at this operating point, residual error is dominated by estimator transients; motor strength is second-order.

**Smallest justified design change: none to the frozen controller.** If hardware calibration finds that the effective K<sub>t</sub> differs from nominal, rescale the feed-forward current by K<sub>t,nom</sub>/K<sub>t,meas</sub>. That is one parameter, and it is proposed, not tested.

**Hardware confirmation:** torque-versus-current fixture identification, then the roll-held yaw sweep ([qualification plan](hardware_qualification_plan.md)).

**Risks carried to hardware:**

- the drive voltage convention (V<sub>bus</sub> vs V<sub>bus</sub>/√3);
- the loaded fallback excursion;
- the nominal-model governor admitting an unknown-load move that later trips;
- the assumed thermal thresholds.

---

## Appendix A — Plots (outside the page limit)

| What it shows | Figure |
|---|---|
| Saturation and anti-windup | ![](figs/task2_saturation.png) |
| Phase lag: loop frequency response and delay margins | ![](figs/task4b_frequency.png) |
| Delay robustness | ![](figs/task2_delay_robustness.png) |
| Fallback behaviour: feedback-loss timeline | ![](figs/task2_feedback_loss.png) |
| Fault and catch timeline | ![](figs/task4b_fault_timeline.png) |
| Generalization across chosen stress cases | ![](figs/task4b_generalization.png) |
| A request that should not be tracked | ![](figs/task4b_infeasible.png) |
| Tracking, original vs governed reference | ![](figs/task2_tracking.png) |
