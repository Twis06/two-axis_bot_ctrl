# Task 2 — Build a baseline

> **Status: frozen deterministic baseline (Packet 2C).** Code identity: the baseline fingerprint printed at the top of [task2_numbers.md](task2_numbers.md) and [task2_robustness.md](task2_robustness.md). It is a sha256 over the controller and simulator modules in `exp/evidence.py` `BASELINE_SOURCES`. Every number below comes from those two generated files or from a packet report named beside it. Rows of `task2_numbers.md` cite run_ids resolved in [task2_runs.json](task2_runs.json). `task2_robustness.md` carries the fingerprint and git state but no per-run manifests (2C review I-B; to be routed through the same run records in 4B). Change history and review rounds: [2A](packets/2A.md) (governor contract), [2B](packets/2B.md) (fault classes, recovery, information access), [2C](packets/2C.md) (freeze).
>
> Evidence labels: **Observed** (the A–E hardware summaries only), **Calculated**, **Simulated**, **Proposed**. Nothing here is a hardware observation. The simulator is a reconstruction whose trajectories and payload are assumptions (Task 1).

**Answer.** Keep a deterministic model-based controller:

- a 500 Hz PI × lead feedback loop with nominal-model feed-forward;
- yaw-coupling feed-forward from a causal estimate of yaw motion;
- a torque-aware reference governor that slows, restricts, suspends or rejects requests rather than moving them;
- a 1 kHz drive-local supervisor that owns the limits and recovers by fault class.

The review work (Packets 2A/2B) kept this structure but corrected what it promised:

- A held reference no longer drifts to cancel a disturbance.
- A tracking fault is no longer cleared just by re-aligning.
- Without yaw authority the axis is no longer handed to passive damping.
- The claimed mode no longer uses the future yaw plan.

This is a controller implementation and a simulation-backed proposal for hardware qualification. It is not a certified safety system, and the assessment prescribes no safety standard or tracking target.

## 1. Architecture and available information

```text
Original roll path ─> governor (PATH / JOIN / STOP / HOLD; accept, reshape, restrict,
                     suspend, reject)  ─> governed q_c, v_c, a_c
Yaw encoder in feedback ─> Kalman estimate ─> coupling predicted at t + 4 ms ─┐
Governed reference ─> nominal-model feed-forward ─────────────────────────────┤
Delayed roll encoder ─> PI × lead feedback on e = q_c − q ────────────────────┤
                                       feed-forward-priority clamp, anti-windup
                                                     │ CAN (latency, bursts, loss)
                       drive supervisor, 1 kHz: timeout, watchdog, overspeed, thermal,
                       fault classes, lockout; fallback = local velocity damping
                                                     │
                       1 ms command delay, latest-limit clamp, current lag, voltage limit
```

**What the host knows:**

- It receives timestamped, quantized roll and yaw encoder samples, measured current, the drive's active current limit, the drive mode and the drive's latched fault (name, id and lockout).
- It never reads the true, perturbed plant parameters.
- The drive's fallback uses only its local encoder.

**Yaw information: an explicit mode (Packet 2B):**

- **`estimate` (default; the claimed baseline):** a three-state constant-acceleration Kalman filter runs at the drive timestamps of the yaw encoder samples. The coupling is predicted 4 ms ahead by extrapolation. No future information is used, and a test fails if the default mode reads the yaw plan.
- **`plan` (optional):** looks ahead along the host's own yaw plan. This assumes the yaw axis follows its plan. The plan mode is tested with a yaw that lags its plan by 10 ms and moves 10% more (§6.4).

**Yaw authority is separate from yaw information.**

- With a planner handle, the host may *request* a smaller yaw amplitude.
- Without one, a request that needs yaw reduction is reported as incompatible. The roll request is braked and held, a coordinated stop is requested, and active control is kept.
- The roll axis cannot contain a continuing external yaw disturbance beyond its capacity. The system integrator must provide the coordinated stop.

## 2. Rates, sensing and estimates

| Function | Rate / setting | Rationale |
|---|---|---|
| Current command and drive supervision | 1 kHz | Given in the assessment; the local checks continue without the host |
| Position control | 500 Hz | The permitted maximum; limits sampling and hold delay |
| Plant integration (simulation) | 10 kHz RK4 | Resolves the 1.2 ms current lag; convergence is tested |
| Learned / adaptive update | none | The Task 3 decision |
| Host stale-feedback threshold | 15 ms | Declared: longer than the 11 ms all-burst delay, so feedback up to 15 ms old still produces valid commands (Packet 2B review M6) |
| Drive command timeout | 10 ms | Independent detection of missing or invalid host control |

| Quantity | Source | Estimator | Notes |
|---|---|---|---|
| Roll position | Newest delivered 14-bit sample, timestamped | none | 0.022° per count; rejected if older than 15 ms |
| Roll velocity, feedback law | not estimated separately | implicit in the lead filter (zero 7 rad/s, pole 112 rad/s) | Friction feed-forward uses the *reference* velocity: one count per 2 ms is 0.19 rad/s, ten times the friction scale |
| Roll velocity, fault recovery | roll encoder in feedback | constant-acceleration Kalman filter | Used only to plan a catch after a tracking fault (§5) |
| Roll velocity, drive | local 1 kHz encoder | first-order filter, about 36 Hz | Fallback damping and the overspeed check |
| Yaw velocity and acceleration | yaw encoder in feedback | constant-acceleration Kalman filter, q_jerk = 1e5, 12 ms warm-up, restarted after a 50 ms gap | Leaves 10–13% of the coupling RMS on the B/C sines (Simulated, 2B §3) |
| Load / payload torque | not estimated | the integrator absorbs a constant offset | A structured estimate is the Task 3 candidate |
| Winding temperature | drive thermal model | first-order, assumed constants | A simulation assumption, not a sensor |

## 3. Control law and gain selection

For the governed state (q_c, v_c, a_c), nominal-model feed-forward:

```text
tau_ff = J a_c + b v_c + 0.8 tau_c tanh(v_c / 0.05) + tau_g sin(q_c)
       + K_YV yaw_rate_est(t + 4 ms) + K_YA yaw_acc_est(t + 4 ms)
```

Friction is deliberately under-compensated (×0.8). The 4 ms look-ahead offsets the fixed command path; it does not cancel CAN jitter or estimation error.

Feedback on e = q_c − q_measured, discretized with Tustin at 500 Hz:

```text
C(s) = K (1 + omega_i / s) (1 + s/omega_z) / (1 + s/omega_p)
K = 0.7553 N·m/rad,  omega_i = 2.8 rad/s,  omega_z = 7 rad/s,  omega_p = 112 rad/s
design crossover 28 rad/s = 4.46 Hz
```

**Clamp and anti-windup (Packet 2B):**

- Feed-forward has priority in the clamp. Feed-forward is clipped to ±Kt·I_limit first, and feedback gets the headroom that remains.
- The integrator uses conditional integration: it skips any step that would push further into a clipped feedback share. The integral torque is also bounded to ±Kt·I_limit.
- The earlier back-calculation on the total clamp wound the integrator against feed-forward clipping. With the governor off, 30° in 0.1 s took 1.11 s to settle within 1°, with the integrator at its 0.448 N·m bound (Packet 2B §3, pre-2B tree). Frozen baseline: 0.60 s, max |integrator| 0.037 N·m (robustness §2b, Simulated).
- The back-calculation-on-headroom correction proposed earlier was tested and was worse (1.37 s; 2B §3).

**Gains.** The revised gain search enforces, as declared design criteria (not requirements from a standard):

- ≥ 45° phase margin and ≥ 6 dB gain margin at the nominal 7 ms condition;
- ≥ 30° / 6 dB at the combined 11 ms, J −30%, Kt +15% corner.

Neither the information mode nor the anti-windup change touches the feedback loop, so the 4.46 Hz design is kept.

| Analytic condition (Calculated) | Phase margin | Gain margin | Crossover |
|---|---:|---:|---:|
| Nominal, 6 ms delay | 51.0° | 14.5 dB | 4.45 Hz |
| Design, 7 ms | 49.4° | 13.5 dB | 4.45 Hz |
| Burst, 11 ms | 43.0° | 10.2 dB | 4.45 Hz |
| 11 ms, J −30%, Kt +15% | 32.1° | 6.1 dB | 6.80 Hz |
| 11 ms, J +30%, Kt −15% | 45.7° | 13.8 dB | 3.13 Hz |

**Linearization limits.**

- Gravity stiffness is linearized at q = 0. It varies with roll angle and payload. With a 1 kg lateral payload at its worst angle it is **negative** (−0.34 N·m/rad): the plant is locally unstable there, and the loop needs at least 0.46× of its gain (§7).
- A continuous-time delay approximation, even one cross-checked against the sampled simulator (§7), does not prove stability of the sampled, saturating, switched nonlinear system. The nonlinear sweeps in §6 are samples, not proofs.

## 4. Reference governor

The available modeled torque is `0.8 Kt I_limit − 0.05 N·m`, a 20% reserve plus the given disturbance bound. The governor ([Packet 2A](packets/2A.md)) emits exactly one reference per tick:

| Mode | Reference | Path clock |
|---|---|---|
| PATH | the admitted path on its own clock σ, slowed by a 0.4 s, 48-point torque look-ahead with planned braking | runs at s ∈ [0, 1] |
| JOIN | minimum-jerk segment between stationary holdable points; offsets ≤ 2° are blended instead | frozen, or running for blends |
| STOP | constant deceleration to rest, re-planned every tick; uses the reserve only if the budget would leave the holdable interval | frozen |
| HOLD | stationary, until the path point is holdable again, or until `replan()` when suspended | frozen |

- The emitted (q, v, a) are mutually consistent in every mode.
- There is no reference-tracking follower and no root finding.
- A requested hold is never moved to cancel a disturbance: excessive predicted coupling is reported (`over_budget`), not compensated.

| Status | Condition | Action |
|---|---|---|
| accepted | the path fits the modeled envelope | clock at 1 |
| reshaped | dynamic demand too high, positions holdable | slow the clock |
| joining | entering or re-entering the path | JOIN or blend |
| restricted | part of the request is statically infeasible or braking room was lost; or the request is **suspended** (tracking fault, incompatible yaw) | stop and hold, with the reason |
| over_budget | predicted torque at the emitted reference exceeds the budget | reported; the host asks the yaw planner for the scale that fits, or declares incompatibility (§5) |
| rejected | no holdable position, non-finite input, inconsistent plan, a state beyond capacity | latch; drive fallback until `replan()` |

**Limits.**

- Feasibility is judged with the nominal load model, so an unknown payload can exceed the prediction. Saturation lowers the permitted path rate, which cannot identify the payload.
- The plan must supply consistent (q, v, a). A zero-order-hold setpoint stream at ≥ 50 Hz is rejected; fast limit toggling makes the reference move between boundaries (2A §5).

## 5. Fault handling and recovery (by class)

| Trigger (drive latches) | Class | Re-arm condition (all also need fresh, valid commands aligned within 2° for 50 ms) |
|---|---|---|
| No fresh command for 10 ms; invalid command | comm | none: automatic once the link is back |
| Filtered speed > 25 rad/s | motion | speed < 20 rad/s |
| Assumed winding > 130 °C | thermal | < 120 °C |
| \|q − q_ref\| > 12° for 40 ms | tracking | the host's acknowledgement of this fault id **on every command of the dwell** |
| 3 tracking or motion latches within 30 s | lockout | none in this run |

**Host policy:**

| Trigger | Response | Resumes |
|---|---|---|
| Feedback missing, non-finite, stale, future-dated, or non-positive limit | Invalid zero-current command, so the drive falls back: fresh outgoing commands cannot conceal lost sensing. Path frozen. | Fresh feedback, then a comm re-arm and a re-join |
| Drive comm fault | Re-align to the measured position; the drive re-arms by itself | Automatic re-join |
| Drive tracking fault | Suspend the request. Acknowledge only while a brake-to-rest *and* hold are predicted feasible from the measured (q, v) within actuator capacity, including a coupling bound: the largest predicted coupling over the last 1 s, so a catch waits while yaw is active and becomes possible about 1 s after it goes quiet (2C review I-A). Then brake from the measured velocity and hold. | Only `replan()` |
| Drive lockout | Rejection with the reason | Not in this run |
| Coupling beyond capacity for a static hold, or over budget for 0.3 s without a yaw planner | Incompatible: suspend the roll request, request a coordinated stop, keep active control | `replan()` |
| Roll saturated with significant yaw coupling for > 5% of a complete 0.5 s window (yaw monitor) | With a planner: request 0.8× yaw. Without one: incompatible, as above. Under periodic yaw this is usually the trigger that fires, because over-budget is intermittent and the 0.3 s timer rarely completes (2C review m6) | as above |
| Coupling over budget with a yaw planner | Request the yaw scale that fits (proportional); the saturation-based yaw monitor is a backstop | Automatic |
| Non-finite command computed on the host | Latched rejection with its cause; the drive also refuses non-finite targets | `replan()` |

**Fallback.** Fallback is local velocity damping, `i = −0.05 v_est / Kt`, within the current limit. It **reduces motion. It does not hold position** against a payload or moving yaw.

Under a lateral payload, the axis moves toward its passive equilibrium (about −63° for 0.7 kg at +35 mm) until caught. In the Packet 2B outage matrix it travelled up to 36°, at up to 6.8 rad/s (Simulated).

The current *target* is re-clamped after the 1 ms delay queue, so a derate cannot be bypassed. Measured current decays toward a newly reduced target over the current-loop lag. Thermal constants, thresholds and the lockout counts are assumptions.

## 6. How the frozen baseline performs (Simulated)

All numbers below are from [task2_numbers.md](task2_numbers.md) (five seeds unless stated; run-set ids there).

- **"Tracked"** is the Packet 4A rule: net path progress ≥ 95%, request-window path RMS ≤ 2°, and no rejection, suspension or tracking fault. The thresholds are **project assumptions**, not assessment requirements.
- **"Governed" error** measures how well the chosen reference was followed. It is not delivered motion.

### 6.1 A–E reconstruction (median of 5 seeds)

| Run | Legacy RMS | Original-request RMS / peak | Governed RMS / peak | Net progress | Events | Tracked |
|---|---:|---:|---:|---:|---:|---:|
| A: shaped ±45° moves | 3.50° | 0.74° / 2.54° | 0.68° / 2.27° | 100% | 0 | 5/5 |
| B: yaw 1.5 Hz, roll held | 3.57° | 0.27° / 0.58° | 0.27° / 0.58° | 100% | 0 | 5/5 |
| C: yaw 2.2 Hz, roll held | 7.64° | 0.40° / 1.00° | 0.40° / 1.00° | 97% | 0 | 5/5 |
| D: 0.7 kg lateral, ±80° sweep | 10.23° | 76.7° / 163° | 4.10° / 9.54° | 83% | 0 | 0/5 |
| E: as D at 2.4 A | 12.02° | 78.5° / 164° | 4.25° / 9.54° | 51% | 0 | 0/5 |

- **A–C:** delivered at full or near-full path rate, with sub-degree error and no events.
- **C and the estimate mode:** in estimate mode C costs 0.15° of RMS against the plan look-ahead, and yaw is reduced once to 0.88× during the yaw ramp-up. C sits about 0.02 N·m inside its hold budget (2B §5.2).
- **D/E are not delivered as requested.** The governor slows the path to 83% and 51% of the clock, so error against the original wall clock is very large. D/E meet the tracking rule in 0/5 runs. The unmodelled payload (Task 3) is the cause, not a fault.

### 6.2 Finite motions (Packet 4A)

| Motion | Completed | Median completion vs request | Watchdog trips | Suspended |
|---|---:|---:|---:|---:|
| M1: unloaded ±45° | 5/5 | 2.09 s vs 2.10 s | 0 | 0/5 |
| M2: 0.7 kg +35 mm, 2.4 A, ±60° in 0.5 s | **0/5** | – | 5 (one per run) | 5/5 |
| M3: ±30° during yaw ±75° at 1.5 Hz | 5/5 | 2.34 s vs 2.40 s | 0 | 0/5 |

M2 is the useful-motion failure of this baseline. The nominal-model governor admits a move that the unmodelled payload makes unfollowable. The watchdog trips once, the host catches the axis and holds it, and the move never completes. Before Packet 2B the same case cycled 27 times through trip and re-arm.

### 6.3 Faults and limits (seed 7)

- **60 ms outages, feedback-only, command-only and both:** one comm fault each, automatic re-arm, governed RMS 0.62°.
  - Net progress is 92% (the path clock is frozen during the outage and the re-join), so these runs are "not tracked".
  - The yaw scale matches the no-outage run: no outage-attributable yaw request (2B §9 I3).
- **Burst storm and 5% message loss:** no events; governed RMS 0.61° and 0.52°.
- **Mid-run derate to 2.4 A with 1.5× coupling:**
  - With yaw coordination, yaw is reduced to 0.61× and governed RMS is 1.39°, with no events.
  - Without it, the request is suspended with a coordinated stop for 61% of the run, and net progress is 16%.
- **Combined corner (4 ms CAN each way, J −30%, Kt +15%):** 0.93° governed RMS, no events.
- **20 V bus, R +25%, fast sweep:** reshaped to 58% progress, 0.92° governed RMS.
- **Payload E with a 100 ms feedback loss, both CoM directions:** comm fault and re-arm only, no watchdog trip at seed 7. Governed peak 12.0° and 13.1°.
  - Across seeds 1–5, the +35 mm case trips once per run and is then caught and held (2B §5.1; before 2B it was 14–16 trips).
- **C with a 0.7 kg +35 mm payload, feedback loss, no yaw coordination:** incompatible at 0.81 s, during the yaw ramp-up and before the outage. Roll saturates under the coupling plus the unmodelled payload (yaw monitor trigger). The run is suspended with a coordinated stop for 90% of its length, reaching 10% progress.
  - The roll hold is kept near 0° until the outage.
  - It then falls in passive fallback, is re-armed, and parks at q_c = −39.6°, with q −38.9° at the end.
  - Original-request RMS after 4 s is 39.7° (rebuilt from the run's manifest).

Small governed error during fallback or suspension is not tracking.

**Current limit.** Across all baseline trials, the maximum applied-target excess above the active limit is 0 A. Measured current may briefly exceed a newly reduced limit while the current loop decays.

### 6.4 Uncertainty sweep (20 sampled plants per run; chosen stress ranges, not distributions)

| Run | Governed peak p95 | Trials with events | Watchdog trips | Suspended | Tracked |
|---|---:|---:|---:|---:|---:|
| A | 6.30° | 0/20 | 0 | 0/20 | 13/20 |
| B | 3.07° | 0/20 | 0 | 0/20 | 20/20 |
| C | 5.14° | 0/20 | 0 | 0/20 | 15/20 |
| D (0.3–1.0 kg) | 12.87° | 5/20 | 6 | 5/20 | 0/20 |
| E (0.3–1.0 kg) | 14.21° | 6/20 | 9 | 6/20 | 0/20 |

- **A and C "not tracked" cases:** the request-window path RMS is 2.0–3.2°, just over the assumed 2° threshold, for heavier or weaker sampled plants. Two C plants also reach only 93% progress. There are no events.
- **D/E:** the faulting trials are the same stressed plants as before 2B (D 5/20; E 6/20, against 7/20 before). They now end suspended after one or two trips, with no lockout in this set.

**Information-mode comparison (Packet 2B §5.4, Simulated; medians of 5 seeds, governed RMS for B / C):**

| Case | Estimate mode (the baseline) | Plan look-ahead |
|---|---|---|
| Yaw follows its plan | 0.27° / 0.40° | 0.22° / 0.25° |
| Yaw lags its plan by 10 ms and moves 10% more | 0.29° / 0.40° | 0.24° / **0.51°** |

The look-ahead is better only while yaw follows its plan closely.

## 7. Why it should remain well behaved: delay, saturation, quantization, parameters

Full evidence: [task2_robustness.md](task2_robustness.md). **Calculated** means closed-loop roots of the linearised loop with a 6th-order Padé delay. **Simulated** means the nonlinear multi-rate simulator.

**Trust in the calculation.**

- The root method reproduces the frequency-domain margins to 0.01 dB.
- Driven to instability by gain scaling, the multi-rate simulator goes unstable at 5.31× and 3.23× the design gain (1.2 ms and 4 ms CAN). The calculation, using the measured pipeline delay, predicts 5.34× and 3.24×.

**Delay.**

- **Calculated:**
  - From 4 to 14 ms of pure delay, the slowest root stays at Re ≈ −2.3 s⁻¹ with damping ratio ≥ 0.45.
  - The delay margin is 37.8 ms, against about 7 ms normal worst case and 11 ms all-burst.
  - The allowed gain increase is 13.5 dB at 7 ms and 10.2 dB at 11 ms.
- **Simulated:** burst storms, 5% loss and the combined corner stay event-free (§6.3).

**Saturation.**

- **Calculated:** a saturating clamp acts as a loop-gain reduction.
  - With nominal gravity stiffness the linear loop is stable for any reduction.
  - With a 1 kg lateral payload at ±80° (k_g = −0.32 N·m/rad), it needs at least 0.39× of its gain. Deep sustained saturation there can lose the position. That is why the governor budgets torque and why the watchdog and fallback exist.
- **Simulated:**
  - **Governed path:** a 30° / 0.1 s request (about 1.2 N·m needed, 0.448 N·m available) is reshaped. It comes within 1° at 0.17 s with 1.1° overshoot, no time at the limit and no event.
  - **Governor off:** the conditional-integration anti-windup keeps the integrator at 0.037 N·m or less and settles in 0.60 s.
  - **Governor off with the real supervisor:** no watchdog trip (pre-2B it cycled through trip and re-arm).

**Quantization.**

- **Calculated:** one count is 0.022°. The feedback's high-frequency gain turns one count into about 33 mA of current step, 1.0% of the 3.2 A limit.
- **Simulated at 24 / 14 / 12 bits:**
  - Run B RMS error is 0.30 / 0.30 / 0.35°.
  - Run B current jitter (standard deviation of the 1 ms increment) is 7.6 / 31.0 / 117 mA. Most of the growth comes from the yaw estimate differentiating a quantized yaw signal; the diagnostic plan-mode column in robustness §4 separates it out.
  - This is a cost of the causal mode: at 14 bits it is about 1% of the limit per millisecond.
  - At a 45° hold, hunting is about 0.2° peak-to-peak at every resolution. That is friction stick-slip with integral action, not quantization.

**Uncertain parameters.**

- **Calculated (108-point grid):** J −30%, nominal, +30% and +1 kg; Kt ±15%; delay 6, 7 and 11 ms; gravity stiffness +0.12, +0.02 and −0.34 N·m/rad.
  - All 108 points are stable.
  - The worst allowed gain increase is 6.1 dB, at 11 ms, and the worst damping ratio is 0.31.
  - With negative stiffness, the gain must stay above 0.46×.
- **Simulated:** the §6.4 sweeps cover coupling ×0.5–1.5, J, friction, gravity, Kt, R, L, bus 20–24 V, payload, bursts and encoder offset.

**What this does not show.**

- The calculations are local and unsaturated.
- The simulations are samples.
- Neither covers unmodelled payload geometry beyond the lateral point mass, or a real yaw axis's plan-following error.

## 8. Operating envelope and declared limitations of the frozen baseline

1. **Unmodelled payload.** The governor judges feasibility with the nominal model.
   - Loaded sweeps are slowed to 51–83% progress.
   - Loaded finite moves can be admitted and then trip once (M2), ending suspended.
   - This is the Task 3 question.
2. **Delivered motion after a tracking fault or incompatibility is zero until a replan.** Nothing in the simulator replans.
   - In the Packet 2B pre-freeze outage matrix (tracking fault; 3 directions × 5 seeds), C with 0.7 kg at +35 mm parks at a median of −43.6° from its 0° hold request.
   - Before 2B it recovered automatically at the second attempt, a policy that also cycled 14–16 times in E.
   - The frozen seed-7 row (incompatibility, §6.3) parks at −39.6°.
3. **The causal yaw estimate costs accuracy near the budget edge:** C +0.15° RMS, one yaw reduction to 0.88× during ramp-up, and more quantization jitter. Estimator uncertainty is not propagated into the governor budget; its residual shares the 0.05 N·m disturbance margin.
4. **The catch after a tracking fault is a nominal-model prediction,** using a nominal-model coupling estimate. Repeated failures are bounded by the lockout (3 within 30 s), whose thresholds are assumptions.
   - The lockout is reachable. With 1.5× the modelled coupling, a 0.7 kg payload and yaw that never stops (no planner, coordinated stop not honoured), three catches failed within 0.9 s and the drive locked out: 67.5% passive fallback, ending at −48.7° (2C review probe, Simulated).
   - In the frozen Monte Carlo, 1/5 faulted D trials and 3/6 faulted E trials trip twice; none lock out.
5. **Passive fallback is motion reduction, not a hold:** up to 36° of travel under a lateral payload in the tested outages.
6. **Without yaw authority,** coupling beyond capacity is reported and the roll request is stopped; containing the yaw disturbance itself is outside the roll axis's power. If the coordinated stop is honoured, the axis is caught and held about 1 s after yaw goes quiet (§5).
7. **Interface assumptions:**
   - Plans must be consistent (q, v, a) streams.
   - The acknowledgement has no session nonce: a real drive needs a boot counter in `fault_id` (2B review M1).
   - Feedback up to 15 ms old still produces valid commands.
8. **Assumed values:** thermal constants, the lockout counts, the 20% reserve, the tracking thresholds of §6, the A–E trajectories and the 0.7 kg / 35 mm payload.

## 9. Verification and reproduction

```bash
python3 -m unittest discover -s tests            # full suite
python3 -m exp.task2_eval                        # task2_numbers.md, task2_results.json, task2_runs.json, figures
python3 -m exp.task2_robustness                  # task2_robustness.md, figures
```

Both evaluation scripts publish through `exp.manifest.staged_publish`. Outputs appear only after the whole evaluation succeeds and no source file has changed meanwhile. Every row of `task2_numbers.md` cites a run_id or run-set id, resolved in `task2_runs.json` (the full run specification and 4A metrics per run) and `task2_results.json`. The robustness document carries only the fingerprint and git state.

The regression suites:

- `tests/test_ctrl.py`, `test_sim.py`, `test_hardening.py`: loop design, simulator, feedback and command safety, derating, the integrator bound;
- `tests/test_governor_contract.py`: the governor contract and three 2A review rounds;
- `tests/test_fault_recovery.py`: fault classes, acknowledgement and lockout, the catch, loaded outages, directional outages, no yaw authority, the monitor window, the information mode, non-finite values, anti-windup, and the 2B review reproducers;
- `tests/test_metrics.py`: the 4A scoring rules.

## 10. What is ready, and what remains uncertain

**Ready.** The baseline is frozen, with explicit and tested contracts:

- the governor never moves a request to cancel a disturbance;
- recovery cannot bypass the condition that tripped it;
- lost communication cannot be concealed;
- the claimed mode is causal.

It is the comparator for Task 3 and the controller for the Task 5 prediction.

**Uncertain before hardware.** Establish:

- the true torque/current and voltage conventions;
- command/feedback latency;
- the yaw coupling, and how closely the yaw axis follows its plan (this decides between estimate and plan modes);
- payload and thermal behaviour;
- fallback clearance and a coordinated yaw stop.

Neither threshold is specified by the assessment. The 2° request-window path RMS is a **project assumption** used by the "tracked" classifier (§6), together with ≥95% progress and no rejection, suspension or tracking fault. The 5° figure is a different project rule: the Task 3 whole-run overshoot allowance, not a peak-error criterion of this classifier. D/E deliberately trade delivered progress for staying within the *nominal-model* torque budget; the unmodelled payload still puts nominal E at the command limit 5.5% of the time (146 saturation entries).

Implementation: [controller](../ctrl/baseline.py), [governor](../ctrl/governor.py), [yaw/roll estimator](../ctrl/yaw_estimator.py), [supervisor](../ctrl/supervisor.py), [fault classes](../ctrl/interfaces.py), [gain derivation](../ctrl/loopshape.py), [drive model](../sim/drive.py), [evidence plumbing](../exp/evidence.py).
