# Plan — Roll-Axis Safe Control Take-Home

> This is the historical development plan. Read [the task-organized answers](report/README.md) for the current assessment narrative. [Task 1](report/task1.md) qualifies earlier causal and feasibility claims; phase completion labels below describe recorded project progress, not a fresh verification.

**Thesis to test:** Error in the yaw runs (B/C) comes from rejecting the disturbance with feedback alone, through about 5 ms of lag. The fix is yaw feed-forward. Runs D/E are a separate problem: an unmodeled gravity load plus a real torque shortfall at 2.4 A. The right response there is to reshape or reject the request, not to track it harder.

**Stack:** Python 3 + numpy/scipy/matplotlib. `make all` (or `python run_all.py`) regenerates every number and plot.

---

## Phase 0 — Paper analysis ✅ done → `report/phase0_analysis.md`, `python analysis/phase0.py`
Result: feedback bandwidth is ~7 Hz (not 8–15). Feedback alone leaves 3–5° peak error at B/C, and yaw FF cuts that to ≤1.4°. D's signed mean error points to an asymmetric payload whose mass can't be identified. At 2.4 A, ±75° yaw is only feasible below 1.8 Hz. See §6 there for the Phase 2 inputs.
| Task | Output |
|---|---|
| Torque budget per run (gravity, Coulomb friction, coupling, d, inertia) vs 0.448 / 0.336 N·m | Table T1 |
| Delay budget (1 ms + CAN + 1.2 ms lag + ZOH + estimator) → max crossover frequency at 45° phase margin | Number: roughly 8–15 Hz |
| Voltage headroom: V = iR + L·di/dt + Ke·q̇ at 20 V, R +25% | Max speed/accel envelope |
| What the summaries can't prove + one decisive experiment | Memo §1 bullets |

**Done when:** every run has a line saying whether it is feasible, marginal, or infeasible, and why.

## Phase 1 — Simulator ✅ done → `report/phase1_sim.md`, `python exp/reproduce_runs.py`
Result: 18 validation tests pass, including a delay-sensitive stability-boundary test. One legacy PD with no integrator reproduces the A–E RMS within ±25 %, plus D's bias and E's saturation cycling. It **cannot** reproduce B/C's current or clipping → there is a demand outside the stated model, so robustness tests will use coupling ×0.5–1.5 plus pulses beyond the |d| bound.

### Original Phase 1 plan
- Plant ODE with the tanh friction term; 1st-order current lag; current clamp.
- Transport delay: 1 ms + CAN latency drawn from 0.6–1.8 ms, with random 4 ms bursts.
- 14-bit encoder quantization; multi-rate scheduler (1 kHz / controller rate / 50 Hz, with ZOH).
- Electrical model: R, L, Ke, bus voltage → voltage clamp on current.
- Uncertainties: payload (τg, center-of-mass offset), friction, R/L, V_bus, delay. Seeded RNG.

**Done when:** a feedback-only PD roughly reproduces the *shape* of Runs A–E (saturation % and error ordering). This validates the model we use as evidence.

## Phase 2 — Baseline controller ✅ done → `report/phase2_baseline.md`, `python exp/phase2_eval.py`
Result: B/C error falls from 3.6/7.6° to 0.2° RMS; Monte Carlo p95 is ≤ 2° with 0/120 fault events. The derate + coupling ×1.5 case is handled by shrinking yaw instead of tracking it. The unknown payload is handled safely but slowly (E at 48 % speed) → Phase 3 candidate: a structured payload estimate.

### Original Phase 2 plan
1. **Reference shaper:** acceleration and jerk limits derived from the available torque margin.
2. **Estimator:** position and velocity from a 2-state observer or low-pass differentiator (justify against the 0.19 rad/s quantization steps).
3. **Loop at 500 Hz:** PID with back-calculation anti-windup, plus feed-forward for Jr·q̈_ref, τg·sin q, τcouple(q̇y, q̈y), and friction (smoothed deadband).
4. **Supervisor at 1 kHz:**
   - current and thermal derating
   - tracking-error watchdog
   - stale-CAN detection → hold last command, then fall back to a safe mode
   - saturation-duration limit
5. **Request policy:** accept, reshape, derate, or reject, based on predicted torque vs available torque.

**Evidence:** loop margins (Bode / Nyquist with delay); a Monte Carlo sweep over the uncertainties; worst-case Runs B, C, E.

## Phase 3 — Learning decision
- **Default:** a bounded adaptive gravity/payload estimate at ≤50 Hz, non-blocking, with a staleness timeout. It is clamped to the physical range, and ignored when saturated or stale.
- **Test:** tune on nominal payload; evaluate on unseen center-of-mass shifts. Compare against (a) feed-forward only and (b) a retuned integrator.
- **Keep it** only if it wins on untuned cases; otherwise document why it was dropped and what would change the call.

## Phase 4 — Prediction test
1. Write the prediction first, e.g. "At 2.4 A with yaw at 2.2 Hz, feed-forward cuts RMS error from X° to Y°, and saturation drops below Z%."
2. Run one perturbation: V_bus = 20 V + R +25%, or a heavier payload.
3. Compare the result with the prediction; revise the thesis if it's wrong.
4. State the smallest design change and the hardware measurement that would confirm it.

## Phase 5 — Deliverables (`report/`)
| Item | Content |
|---|---|
| Memo (≤4 pp) | §1 Diagnosis · §2 Baseline & safety · §3 Learning call · §4 Prediction test & limits |
| Plots | Current vs limit (saturation), Bode with delay (phase lag), fault/fallback timeline, generalization sweep |
| Qualification plan (1 pp) | Instrumentation → test order (open-loop ID → static hold → sweeps → yaw disturbance → faults) → stop conditions → acceptance criteria |
| References note | Sources, reused code, AI tool use |
| README | Setup + one command |

---

## Key risks
- The simulator could be tuned to fit the summaries → keep the fit to *shape only* and state that.
- The payload mass is unknown → treat it as a parameter sweep, not a fitted value.
- Scope creep on learning → time-box Phase 3.

## Repo layout
```
sim/      plant.py  actuator.py  sensing.py  timing.py
ctrl/     shaper.py  estimator.py  baseline.py  supervisor.py  adaptive.py
exp/      reproduce_runs.py  montecarlo.py  prediction.py
report/   memo.md  qual_plan.md  references.md  figs/
run_all.py  README.md  requirements.txt
```
