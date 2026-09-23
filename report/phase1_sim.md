# Phase 1 — Simulator and Consistency With the Logs

> Historical investigation. See [Task 1](task1.md) for the current diagnosis and its uncertainty qualifications, and [the task index](README.md) for the assessment answers.

Code: `sim/`, `ctrl/interfaces.py`, `ctrl/legacy.py`, `exp/scenarios.py`, `exp/reproduce_runs.py`.
Numbers: [phase1_numbers.md](phase1_numbers.md). Figure: `figs/p1_legacy_runs.png`.

## 1. What the simulator models

| Element | Model | Source |
|---|---|---|
| Roll dynamics | J q̈ = Kt·i − b q̇ − τc tanh(q̇/0.02) − τg sin q − τ_lat cos q − τcouple + d, integrated with RK4 at 10 kHz | brief; τ_lat is the payload term |
| Payload | point mass m moved 35 mm laterally: τ_lat = m·g·0.035 and J += m·0.035² | Phase 0 inference |
| Current loop | di/dt = (i_tgt − i)/1.2 ms, **clipped to the rate that the available voltage allows**: \|R i + L di/dt + Ke q̇\| ≤ V_bus/√3 | brief; conservative SVPWM limit |
| Drive (1 kHz) | the newest command wins; clamp to the active limit (3.2 A or derated); 1 ms command delay; command timeout (10 ms) triggers local velocity damping | brief |
| Host (500 Hz by default) | receives the newest feedback, computes, and releases the command 0.5 ms later | ASSUMPTION: compute time |
| CAN | per message U(0.6, 1.8) ms in each direction; Poisson burst episodes (0.5 /s, 20–100 ms long) at about 4 ms; out-of-order delivery handled; drops and blackouts available for fault injection | brief + ASSUMPTION: burst statistics |
| Sensing | 14-bit floor-quantized roll and yaw encoders; no velocity sensor | brief |
| d(t) | band-limited (3 Hz) noise scaled so that max\|d\| = 0.05 N·m exactly | brief bound |
| Async policy | a non-blocking worker whose results become visible after 2–8 ms (for Phase 3) | brief |

Uncertainty knobs, all in `SimConfig`: J, b, τc, τg, payload mass, coupling coefficients, true Kt (motor strength), R, L, V_bus, latency distribution, derating schedule, encoder offset, d amplitude and bandwidth. A (config, seed) pair fully determines a run.

## 2. Validation: 18 tests, all passing (`python -m unittest discover -s tests`)
| Test | Checks |
|---|---|
| Small-oscillation period and frictionless energy drift (< 1e-6) | the plant and the integrator |
| Current lag reaches 63.2 % at τ; stall current = V/R; free speed = V/Ke | actuator and voltage model |
| Coupling torque peak equals the closed form | yaw coupling |
| Command step applied at exactly 0.103 s for a 0.100 s command | whole pipeline: compute 0.5 ms, CAN, drive tick, 1 ms delay |
| Newest sequence wins; burst duty ≈ rate × mean length | CAN model |
| Timeout → fallback within 15 ms, and recovery after the blackout | fault path |
| Same seed gives identical results; a different seed differs | reproducibility |
| dt = 1e-4 vs 2.5e-5 agree to < 1e-3° (24-bit encoder) | integration converged. With 14 bits, one count can flip and the runs diverge by quantization, not integration error |
| Closed-loop amplitude matches the frequency-domain model within 5 % | end-to-end gains |
| **The predicted critical gain (4.8×) is stable at 0.8× and unstable at 1.25× in the simulator** | **end-to-end delay**: the test that actually exercises latency |

Two defects were found and fixed along the way:
- The linearised analytic model had omitted gravity stiffness.
- The "clipped" metric missed commands that sat exactly at the limit.

Both current clamps (drive and controller) also turn an unstable loop into a bounded limit cycle. That is realistic, but the delay test has to lift them.

## 3. Does it reproduce Runs A–E?
The legacy controller has to be *reconstructed*, since the logs do not say what produced them. I searched about 100 candidates. The only one that reproduces D's signed bias is:
- PD (Kp = 1.5 N·m/rad, Kd = 0.05) with nominal gravity feed-forward and **no integrator**
- a lateral payload of 0.7 kg
- a sweep at 1 Hz

A PID with an integrator removes the bias (mean ≈ −0.2°).

Results for this one legacy controller, one parameter set and 5 seeds (median):

| Run | RMS obs / sim | Peak obs / sim | % limited obs / sim | Match |
|---|---|---|---|---|
| A | 2.8 / 3.5° | 6.1 / 10.6° | 0 / 0 | ✓ RMS; the peak is too high (my first 45° move is fast) |
| B | 4.2 / 3.6° | 11.5 / 6.3° | **8 / 0** | ✓ RMS; ✗ peak and clipping |
| C | 7.8 / 7.6° | 20.7 / 12.9° | **31 / 0** | ✓ RMS; ✗ peak, clipping and watchdog trip |
| D | 9.1 / 10.2°, mean \|4.6\| / \|4.9\|° | — | **38 / 0** | ✓ RMS and bias; ✗ clipping |
| E | 12.6 / 12.0° | 25.4 / 24.4° | — / 30, with 32 saturation entries | ✓ RMS, peak, and "repeatedly exits and re-enters saturation" |

The results are stable across seeds (within ±0.5° RMS). CAN bursts have no visible effect on this soft loop.

**Matched:** RMS within ±25 % in all five runs, the ordering A < B < C and D < E, the size of D's bias, and E's saturation cycling. The sign of the bias differs. The brief's sign convention for "signed mean" and the side of the shift are both unknown.

**Not matched, and not tunable away:** under the stated coupling model, *no* feedback-only controller in the search reproduces B/C's current and clipping together with their RMS error.
- Simulated current is about 40 % (B) and 80 % (C) of what was logged.
- Simulated peak/RMS is about 1.7; the logs show about 2.7.
- Coupling ×1.5 reproduces C's clipping (33 % vs 31 %) but overshoots its RMS (13° vs 7.8°), and B still never clips.
- An abrupt yaw start does not explain the gap either.

## 4. What this changes
1. **The logs contain a current demand, and an episodic error component, beyond the stated model.** The candidates are:
   - a larger or position-dependent cable torque
   - disturbance pulses above the \|d\| ≤ 0.05 N·m bound
   - a noisier or different legacy estimator

   Phase 0's decisive experiment (identify τcouple with roll held, at 1 kHz) is now *more* important, because the result decides how much feed-forward can be trusted.
2. **Robustness tests in Phases 2–4 will cover coupling ×0.5–1.5 plus disturbance pulses beyond the bound**, not the ±20 % assumed in Phase 0. The feed-forward residual estimate from Phase 0 (0.75–1.4° peak) is optimistic by up to about 2× if the coupling is wrong by 50 %.
3. **The legacy structure explains D/E without new physics.** Without an integrator or a load estimate, an unmodelled lateral load leaves a persistent bias. When the load and the sweep exceed the derated limit, the loop cycles in and out of saturation. This supports online load estimation as the one adaptive part worth considering (Phase 3).
4. The trajectory assumptions (0.35–0.5 s moves in A, a 1 Hz sweep in D/E) are the weakest inputs. Comparisons between designs will use identical scenarios, so they stay fair even if the assumptions are wrong.
