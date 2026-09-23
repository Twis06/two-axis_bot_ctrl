# Phase 2 — Baseline Controller

> Historical pre-hardening design and recorded results. See [Task 2](task2.md) and [current evidence](task2_numbers.md) for the revised controller, fault handling, gains, and verification. Re-running the old phase script evaluates the current code and can therefore change its generated numbers.

Code: `ctrl/baseline.py`, `ctrl/governor.py`, `ctrl/supervisor.py`, `ctrl/loopshape.py`. Evidence: `exp/phase2_eval.py` → [phase2_numbers.md](phase2_numbers.md), `figs/p2_*.png`. Tests: `tests/test_ctrl.py` (10) plus `tests/test_sim.py` (18).

## 1. Structure

```
 planner ──request──▶ RollGovernor ──(q_c, v_c, a_c)──▶ feed-forward ─┐
 (host plan,        path time-scaling      J a + b v + 0.8τc tanh(v/.05)│
  yaw plan)         + feasibility          + τg sin q_c + τcouple(t+4ms)│
                    projection                                          ▼
 encoder (CAN) ─────────────────▶ PI × lead on (q_c − q̂) ──────────▶ Σ ─▶ clip(±limit reported by drive)
                                   back-calculation anti-windup ◀────────┘        │ CAN
                                                                                   ▼
 DRIVE SUPERVISOR (1 kHz, owns every hard limit): current clamp, I²t derating, command timeout,
 tracking watchdog (12°/40 ms, latched), over-speed 25 rad/s, over-temperature → FALLBACK = local damping.
 Re-arms only after the host realigns its reference (|q − q_ref| < 2° for 50 ms).
```

| Choice | Value | Why |
|---|---|---|
| Position loop rate | 500 Hz | At 250 Hz the delay grows by 1.5 ms. That would cost about 3° of phase margin at crossover and gain nothing. |
| Feedback | PI × lead, α = 16, ωc = 34.5 rad/s (5.5 Hz), K = 1.16 N·m/rad; Tustin | Derived by `loopshape.design` at the simulator's *measured* worst-normal pipeline delay of 7.0 ms (PM 45°, GM 11.6 dB). Checked at 11 ms, with every message in a burst (PM 37°). |
| Velocity estimate | the lead filter itself (pole at 138 rad/s) | Current jitter is about 51 mA per encoder count. A separate observer would add lag the loop cannot afford. |
| Friction FF | 0.8·τc·tanh(v_ref/0.05) on the **reference** velocity | One velocity LSB at 500 Hz is 10× the friction speed scale. Using the measured velocity would chatter at ±0.29 A. |
| Yaw FF | nominal coupling applied to the host's yaw plan, **4 ms ahead** | The disturbance is predictable, so the command-path delay can be cancelled instead of tolerated. |
| Anti-windup | back-calculation (gain ωc) against the limit **the drive reports** | This fixes the legacy loop's blindness to derating (Run E). |
| Governor | path time-scaling with look-ahead braking, plus projection onto the holdable set | Reshaping slows the path's own clock, so the reference never overshoots or leaves the requested range. Positions that can't be held are rejected. |
| Saturation back-off | sustained saturation lowers the allowed path speed (−3 /s); it recovers at +0.25 /s | Model-free protection against loads the model does not know about. |
| Yaw monitor | roll saturated > 5 % of 0.5 s while the coupling is > 25 % of capacity → yaw × 0.8 | Handles coupling that is stronger than modelled, which Phase 1 found likely. |
| Fallback | local velocity damping, 0.05 N·m·s/rad | Passive. Gravity restores toward 0 with the nominal payload. With an unknown lateral payload the head settles at that payload's equilibrium instead, which is a known limitation. |

## 2. When to reshape, derate or reject
- **Accept:** predicted torque along the path fits within 80 % of the limit, minus the |d| bound. That leaves a 20 % reserve for feedback.
- **Reshape:**
  - Roll: slow the path clock until the torque fits.
  - Yaw: shrink the amplitude to the envelope (offline admission). Example: ±75° at 2.2 Hz becomes ±54° at 2.4 A.
  - Yaw, online: ask it to shrink when roll saturates.
- **Derate:** every envelope is recomputed each tick from the drive's *reported* limit, which comes from the thermal model or a derate command.
- **Reject:** the requested position cannot be held statically. The reference holds at the edge of the feasible set, and the rejection is flagged to the planner.

## 3. Evidence
(Numbers from [phase2_numbers.md](phase2_numbers.md).)

**Runs A–E (5 seeds, median).** Baseline error is measured against the reference it chose to follow. `speed` is the fraction of the requested path speed it delivered.

| Run | legacy RMS / peak | baseline RMS / peak | speed | % at limit L / B | faults L / B |
|---|---|---|---|---|---|
| A | 3.50 / 10.6° | **0.55 / 1.8°** | 100 % | 0 / 0 | 0 / 0 |
| B | 3.57 / 6.3° | **0.20 / 0.5°** | 100 % | 0 / 0 | 0 / 0 |
| C | 7.64 / 12.9° | **0.22 / 0.5°** | 100 % | 0 / 0 | 0 / 0 |
| D | 10.2 / 17.0° | **2.85 / 6.9°** | 82 % | 0 / 2 | 0 / 0 |
| E | 12.0 / 24.4° | **2.93 / 7.8°** | 48 % | 30 / 5 | 2 trips / 0 |

In C the baseline also uses **20 % less RMS current** (1.21 vs 1.50 A): cancelling the disturbance in advance costs less than correcting it after the fact. In D/E the baseline gives up speed rather than accuracy. That is a deliberate reshape, and the saturation episodes that remain are the unknown payload (§4.1).

**Monte Carlo, 40 sampled plants per run.** Controllers see only the nominal model. Sampled ranges: coupling ×0.5–1.5, J ±30 %, τc ×0.5–2, τg ±30 %, Kt ±15 %, R, L, V_bus 20–24 V, payload up to 0.3 kg, bursts 0–2 /s.
- p95 RMS error: legacy 5.5 / 6.1 / 19.2° (A/B/C); baseline **1.6 / 1.3 / 2.0°**.
- p95 time at the current limit in C: legacy 54 %, baseline 0 %.
- Fault events: 0 of 120 baseline runs.
- Most of the baseline spread comes from coupling mismatch (`figs/p2_montecarlo.png`). This is the dependence on identifying the coupling.

**Delay, faults and electrical cases.** Yaw at 2.2 Hz unless stated.

| Case | legacy | baseline | what the baseline did |
|---|---|---|---|
| 60 ms CAN blackout | 7.6° | 0.23° | Drive timed out → damping fallback for about 50 ms → re-armed automatically |
| Burst storm (5 /s, 50–200 ms at 4 ms) | 7.8° | 0.25° | Nothing: PM ≥ 37° at 11 ms |
| 5 % message loss | 7.6° | 0.21° | Nothing: the newest command wins |
| **Derate 3.2→2.4 A mid-run, coupling ×1.5** | **26.6°, 60 % at limit** | **1.6°, 1 % at limit** | **Does not track the request: asked yaw to shrink to 0.8×** |
| 20 V bus, R +25 %, ±80° at 1.5 Hz | 64° (falls behind) | 0.83° | Reshaped to 57 % speed; never voltage-limited |

**Yaw admission** (before execution): ±75° at 2.2 Hz is accepted at 3.2 A and **reshaped to ±54° at 2.4 A**. At 3 Hz it is reshaped to ±48° at 3.2 A and ±32° at 2.4 A.

**Loop margins** (analytic model of the same loop, validated against the simulator in Phase 1):

| Case | Phase margin |
|---|---|
| Nominal delay, 6 ms | 47° |
| Design delay, 7 ms | 45° |
| All messages in a burst, 11 ms | 37° |
| J ±30 %, Kt ±15 % (7 ms) | 40–47° |
| Payload 0.7 kg (7 ms) | 46° |
| Worst corner: 11 ms, J −30 %, Kt +15 % | **24°**, GM 4.2 dB |

The loop tolerates 15–29 ms of *additional* delay before instability (7.9 ms in the worst corner). Quantization appears as about 51 mA per encoder count, visible as fine current texture in `p2_runs_compare.png`.

## 4. Known limits of the baseline (inputs to Phase 3)
1. **Unknown payload.** The governor judges feasibility with the nominal model. A constant load estimate taken from the integrator was tried and failed: a lateral payload's torque varies as cos q, so the constant estimate rejected positions that could be held. In D/E the baseline therefore relies on saturation back-off. That is safe but slow (E runs at 48 % speed) and enters saturation often in short bursts. A **structured payload estimate** (θ₁ sin q + θ₂ cos q) is the candidate adaptive part.
2. **Worst analytic corner.** With 11 ms delay (every message in a burst), J −30 % and Kt +15 %, the loop is still stable but has only 24° of phase margin. A sustained burst storm on a light head would ring. The drive watchdog and fallback are what cover that case.
3. **Yaw FF relies on the coupling model.** At coupling ×1.5 the residual is 50 %, and feedback plus the yaw monitor absorb it. That is covered, but the achieved error scales with the model error. Identifying the coupling on hardware (the Phase 0 experiment) is the highest-value measurement.
4. **Fallback with a lateral payload.** Damping-only fallback lets the head settle at the payload's passive equilibrium, which could be tens of degrees away. A drive-local gravity hold would need the payload estimate. Here that is deferred rather than guessed.
