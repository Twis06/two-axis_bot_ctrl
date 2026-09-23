# Task 2 — Build a baseline

> **Working draft — not yet accepted.** Post-test review found that the governor can drift a held reference when load exceeds its budget. Latest D/E stress results also require recovery review. See [the execution plan](../EXECUTION_PLAN.md) for the blocker, current results, and Task 2 completion gates. The implementation/evidence below is the checkpoint under review.

**Answer:** Retain a deterministic model-based controller, with a 500 Hz PI × lead feedback loop, motion/yaw feed-forward, a torque-aware reference governor, and a 1 kHz drive-local supervisor. The Task 2 review retained this structure but repaired fault recovery and limit enforcement, bounded the governor's smoothing acceleration, and reduced bandwidth to improve the combined delay/plant corner.

This is a controller implementation and simulation-backed proposal for hardware qualification. It is not a certified hardware safety system. The [assessment](../Robotics%20Controls%20Technical%20Assessment.pdf) provides constraints but does not prescribe a safety standard or numerical tracking target.

Current evidence: [regenerated results](task2_numbers.md), [machine-readable trial results](task2_results.json), and [regression tests](../tests/test_hardening.py). Earlier Phase 2 reports describe the pre-review controller and are retained as historical evidence.

## 1. Architecture and available information

```text
Original roll path ──> torque-aware governor ──> governed q, velocity, acceleration
                                                  │
Known yaw plan ──> coupling prediction ──> feed-forward + PI × lead feedback
                                                  │
Delayed encoder/current/limit feedback ────────────┘
                                                  │
                            host current clamp + anti-windup
                                                  │ CAN
                            drive supervisor at 1 kHz
                                                  │
                            command delay + latest-limit clamp
                                                  │
                            current lag / voltage limit / motor
```

The host receives timestamped, quantized encoder positions, measured current, active current limit, and drive mode. It does not read the true perturbed plant parameters. Drive fallback uses its local roll encoder and a filtered velocity estimate.

**Yaw assumptions are explicit:** predictive compensation requires a known yaw trajectory; access to measured yaw position alone does not provide future velocity and acceleration. The simulator treats yaw as a prescribed motion that perfectly follows that plan. The default controller requests fallback if the plan is missing. A deliberately configured feedback-only mode is available for comparison, with reduced disturbance-rejection capability.

A separate optional planner handle authorizes yaw amplitude reduction. If sustained yaw-associated saturation demands a reduction and that handle is absent, the host rejects the request and requests fallback. This does not physically stop the external yaw axis; the system integrator must provide a coordinated stop/replan path before hardware operation. Host-latched request rejection requires controller reset with a reviewed replacement request, not automatic resumption of the incompatible request.

## 2. Rates, sensing, and filtering

| Function | Rate / setting | Rationale |
|---|---|---|
| Current command and drive supervision | 1 kHz | Matches the assessment; local checks continue without the host |
| Position control | 500 Hz | Maximum permitted rate; limits sampling/hold delay |
| Plant integration in simulation | 10 kHz RK4 | Resolves actuator lag; existing convergence tests check numerical behavior |
| Learned/adaptive update | None in this baseline | Task 3 owns the learning decision |
| Host stale-feedback threshold | 15 ms | Explicit engineering setting, longer than modeled normal transport; tested with one-direction outages |
| Drive command timeout | 10 ms | Independent detection of missing/invalid host control |

A 14-bit encoder resolves 0.022° per count. Direct one-count velocity differences correspond to 0.192 rad/s at 500 Hz, nearly ten times the friction model's 0.02 rad/s transition scale. Consequently, friction feed-forward uses the smooth **reference velocity**, not raw differentiated encoder velocity.

The feedback lead filter supplies bounded derivative action; it is not a separate full-state observer. Its pole is 112 rad/s and its zero is 7 rad/s. The nominal continuous high-frequency proportional/lead gain corresponds to approximately 33 mA per encoder count. Drive fallback estimates local velocity with a first-order filter, approximately 36 Hz at the 1 kHz sample rate. Neither estimate supplies noiseless velocity.

### State estimates: what is estimated, and how

| Quantity | Source in the baseline | Filter / estimator | Notes |
|---|---|---|---|
| Roll position | Newest delivered encoder sample (14-bit), timestamped | None; used directly in the error | Age 0.6–4 ms plus host wait; rejected if older than 15 ms |
| Roll velocity (host) | Not estimated separately | Implicit in the lead filter on the error (zero 7, pole 112 rad/s) | Feed-forward uses the *reference* velocity instead |
| Roll velocity (drive) | Local 1 kHz encoder | First-order filter, about 36 Hz | Used only for fallback damping and the overspeed check |
| Yaw velocity / acceleration | Host yaw **plan**, evaluated 4 ms ahead | None | Assumes a known plan that yaw follows. A causal estimate from yaw feedback is not implemented (Task 2B) |
| Load / payload torque | Not estimated | The integrator absorbs a constant offset only | Structured estimate is the Task 3 candidate |
| Loop delay | Not estimated online | Fixed 4 ms yaw look-ahead; gains designed for 7 ms | Feedback timestamps are checked for staleness only |
| Winding temperature | Drive thermal model | First-order, assumed constants | Simulation assumption, not a measured sensor |

## 3. Control law and gain selection

For governed state `(q_c, v_c, a_c)`, use nominal-model feed-forward:

```text
roll feed-forward = J_nom a_c + b_nom v_c
                  + 0.8 tau_c tanh(v_c / 0.05)
                  + tau_g sin(q_c)

yaw feed-forward = 0.008 yaw_velocity(t + 4 ms)
                 + 0.0008 yaw_acceleration(t + 4 ms)
```

The 4 ms look-ahead is an assumed command-path compensation for a known plan, not access to future measurements. It does not cancel arbitrary CAN jitter or yaw tracking error. Friction is deliberately under-compensated to reduce sensitivity near reversal.

Feedback acts on `e = q_c - q_measured`:

```text
C(s) = K (1 + omega_i/s) (1 + s/omega_z) / (1 + s/omega_p)
K = 0.75533 N·m/rad
omega_i = 2.8 rad/s; omega_z = 7 rad/s; omega_p = 112 rad/s
nominal design crossover = 28 rad/s = 4.46 Hz
```

The lead uses a Tustin discretization; the integral state is updated explicitly each 2 ms. Sum feedback and feed-forward torque, divide by nominal Kt, and clamp the command to the smaller of the host maximum and drive-reported current limit. Back-calculation feeds the clamped-minus-unclamped torque difference into the integrator, with gain 28/s. The integral torque is also bounded to ±Kt·I_limit. Without that bound, an abrupt ungoverned request drove it to 6.7 N·m, which is more than the motor can produce; regression test in `tests/test_hardening.py`. The integral state resets during fallback realignment and does not accumulate against stale sensing.

**Open anti-windup finding:** back-calculation reacts to clipping of the *total* command, including feed-forward. When an inertial feed-forward term alone exceeds the limit, the integrator is driven to its bound with the wrong sign for the remaining error, then must unwind. With the governor switched off, a 30° / 0.1 s request then takes 1.12 s to come within 1°, versus 0.24 s without anti-windup (see [robustness §2b](task2_robustness.md)). The normal governed path avoids this condition. Proposed correction, not yet implemented: give feed-forward priority in the clamp and back-calculate only against the feedback headroom.

The original loop targeted approximately 5.5 Hz but had only 24° phase margin and 4.2 dB gain margin with 11 ms effective delay, inertia −30%, and motor strength +15%. The revised gain search enforces at least 45°/6 dB at the nominal design condition and 30°/6 dB at the selected combined corners. These are declared design criteria, not requirements quoted from a safety standard.

| Analytic condition | Phase margin | Gain margin |
|---|---:|---:|
| Nominal 6 ms delay | 51.0° | 14.5 dB |
| Design 7 ms delay | 49.4° | 13.5 dB |
| Burst 11 ms delay | 43.0° | 10.2 dB |
| 11 ms, inertia −30%, motor constant +15% | 32.1° | 6.1 dB |

The calculation includes the 1.2 ms current lag separately from effective pure delay. It linearizes gravity around the nominal zero-roll equilibrium. It is not a proof for arbitrary payloads, all roll angles, switched faults, or saturation. Nonlinear simulations complement the local analysis, and actual hardware delays and computation time still require measurement.

## 4. Reference governor: accept, reshape, derate, reject

Available modeled torque is `0.8 Kt I_limit - 0.05 N·m`. That is a chosen 20% reserve plus the supplied disturbance bound. The governor was rewritten in [Packet 2A](packets/2A.md); its review history and gate result are recorded there.

At each 500 Hz tick it emits exactly one of four references:

| Mode | Reference emitted | Path clock |
|---|---|---|
| PATH | The admitted path on its own clock σ, slowed by a 0.4 s, 48-point torque look-ahead with planned braking | Runs at rate s ∈ [0, 1] |
| JOIN | Minimum-jerk segment between stationary holdable points (start-up, realignment, re-joining after a jump). Offsets ≤ 2° are blended out instead. | Frozen, or running for blends |
| STOP | Constant deceleration to rest, re-planned every tick. It stays within the budget unless that would leave the holdable interval; it then uses the reserve. | Frozen |
| HOLD | Stationary at a restricted point until the path point and the path 10 ms ahead are holdable again | Frozen |

Consistency and scope:

- Velocity is analytic or integrated trapezoidally, and acceleration is the step average, so the emitted (q, v, a) are mutually consistent in every mode.
- There is no reference-tracking follower and no root finding.
- A requested hold is never moved to cancel a disturbance. Excessive predicted coupling is reported, not compensated.
- Velocity kinks and position jumps in the plan are found by the look-ahead and approached at a low crossing speed.

| Status | Condition | Action |
|---|---|---|
| accepted | The requested path fits the modeled envelope | Path clock at 1 |
| reshaped | Dynamic demand is excessive but positions are holdable | Slow the path clock |
| joining | Entering or re-entering the path | JOIN, or a start-offset blend |
| restricted | Part of the request is outside the static feasible interval, or braking room was lost after a limit drop | Stop short of it and hold, labelled `static`, `dynamic` or `braking` |
| over_budget | Predicted torque at the emitted reference exceeds the budget (e.g. yaw coupling) | Reported. The host asks the yaw planner for the scale that fits; without a planner it flags the request incompatible and keeps control. |
| rejected | No holdable position; non-finite input; inconsistent plan; a state beyond actuator capacity | Latch. The host falls back until `replan()`. |

An empty static feasible set is explicitly represented as empty. It is never silently replaced with "zero is a safe parking position". Slowing motion cannot make an excessive static load holdable. A stationary reference left outside the reserved interval by a derate returns to the boundary using the reserve (restricted). It is rejected only if that would exceed actuator capacity.

Yaw admission:

- For nominal yaw coupling with the selected reserve, the offline admission function accepts ±75° at 2.2 Hz at 3.2 A, and reduces it to about ±54° at 2.4 A.
- When predicted coupling breaks the budget, the host requests a proportional yaw reduction at once.
- The saturation-based YawMonitor (a 20% reduction after sufficient saturation occupancy in a 0.5 s window) remains as a reactive backstop. The two paths share one hold-off.

**Limits of the governor:**

- Feasibility is judged with the nominal load model and a sampled look-ahead, so an unknown payload can exceed the prediction. Saturation lowers the permitted path rate; this evidence persists across realignment. It cannot identify the payload or guarantee clearance.
- Voltage headroom is an actuator constraint in the simulator, not a governor calculation.
- The plan must supply consistent (q, v, a). A zero-order-hold setpoint staircase at ≥50 Hz is rejected as inconsistent, so it needs an interpolating front end.

## 5. Fault handling and recovery

| Trigger | Implemented response | Recovery |
|---|---|---|
| Feedback missing, nonfinite, future-dated, stale, or nonpositive reported current limit | Host sends an invalid zero-current command; freezes path and clears feedback memory | Fresh sensing is required before an aligned recovery handshake |
| Commands stale or invalid after normal operation | Drive latches fallback independently of host reference generation | Continuously fresh, valid and aligned commands plus healthy speed/temperature for 50 ms |
| Tracking error >12° for 40 ms | Drive latches tracking fault | Same recovery conditions |
| Filtered local speed >25 rad/s | Drive latches overspeed | Speed must fall below 20 rad/s as well as satisfy the other conditions |
| Assumed winding temperature >130°C | Drive latches overtemperature | Temperature below 120°C plus other recovery conditions |
| Host request rejection (no holdable position, non-finite input, inconsistent plan, state beyond capacity) | Drive fallback; host rejection remains latched with its reason | Explicit `replan()` |
| Predicted coupling over budget, yaw planner available | Governor reports `over_budget`; host requests the yaw scale that fits at once | Automatic, as the yaw blend takes effect |
| Predicted coupling over budget, no yaw planner | Reported as `incompatible` (sticky); control is kept, because passive fallback cannot resist coupling | Explicit `replan()` |

Drive reference alignment means error below 2°. During the handshake the host sends a valid, aligned reference with zero requested current while drive fallback remains authoritative. The drive does not re-enable solely because the reference was moved to the measured position; speed, temperature, freshness, and persistence must also pass.

Fallback commands local velocity damping, `i = -0.05 v_est / Kt`, subject to the current limit. It reduces motion but does **not** hold position against an unknown payload or externally moving yaw. Thermal constants and temperature thresholds are assumed simulation parameters; measured temperature protection, clearance, brakes, and a coordinated emergency stop require separate hardware design.

The active current **target** is clamped after the modeled 1 ms delay queue, so previously queued nominal commands cannot bypass a new derated limit. Measured current can still decay toward the new target over the physical current-loop time constant. The implementation does not claim an instantaneous physical current step.

## 6. How the baseline currently performs (simulated, diagnostic)

These numbers come from [task2_numbers.md](task2_numbers.md), generated by the code after [Packet 2A](packets/2A.md).

- The controller is **not frozen**, and these are not accepted final results.
- The A–E trajectories are the documented reconstruction assumptions, not hardware trajectories.
- Yaw feed-forward uses the known-plan mode.

| Run | Legacy RMS | Original-request RMS / peak | Governed RMS / peak | Path-clock rate | Events |
|---|---:|---:|---:|---:|---:|
| A: shaped ±45° moves | 3.50° | 0.74° / 2.54° | 0.68° / 2.27° | 100 % | 0 |
| B: yaw 1.5 Hz, roll held | 3.57° | 0.22° / 0.45° | 0.22° / 0.45° | 100 % | 0 |
| C: yaw 2.2 Hz, roll held | 7.64° | 0.25° / 0.49° | 0.25° / 0.49° | 100 % | 0 |
| D: 0.7 kg lateral payload, ±80° sweep | 10.23° | 76.9° / 162° | 4.06° / 9.54° | 83 % | 0 |
| E: as D at 2.4 A | 12.02° | 78.9° / 164° | 4.35° / 10.5° | 52 % | 0 |

- **A–C:** the requested motion is delivered at full path rate, with sub-degree error and no events.
- **D/E:** the request is *not* delivered as asked. The governor slows the path (83 % and 52 % clock rate), so error against the original wall clock is very large. Governed error only measures how well the slowed reference is followed. It is not completion of the original motion.
- **Stress sweeps** (20 sampled plants per run, [task2_numbers.md](task2_numbers.md)):
  - A–C: no fault events; governed p95 RMS error 1.6–2.5°.
  - D/E, with payload 0.3–1.0 kg: fault events in 5/20 and 7/20 trials; governed peak p95 about 19°; 38–51 % of time at the command limit (p95).
- **Communication faults:**
  - 60 ms outages in either direction recover, with governed RMS 0.38° and the yaw amplitude unchanged.
  - Payload E plus a feedback-only loss cycles through watchdog trip and re-arm (§8 item 2): 37 trips at seed 7, a median of 33 over seeds 1–10.

## 7. Why it should remain well behaved: delay, saturation, quantization, parameters

Full evidence: [task2_robustness.md](task2_robustness.md), regenerated by `python3 -m exp.task2_robustness`. **Calculated** means closed-loop roots of the linearised loop with a 6th-order Padé delay. **Simulated** means the nonlinear multi-rate simulator. Both describe the current, unfrozen code; neither is a hardware observation.

**Trust in the calculation.**
- **Against the frequency-domain method:** the root method reproduces the gain margins from `ctrl/loopshape.py` to 0.01 dB. Both methods give the same 37.8 ms delay margin.
- **Against the simulator:** the full multi-rate simulator (500 Hz host, 1 kHz drive, CAN both ways, 1 ms delay, current lag) is driven to instability by scaling its gain. It goes unstable at 5.31× and 3.23× the design gain, at 1.2 ms and 4 ms CAN. The calculation predicts 5.34× and 3.24× from the pipeline delay measured in the same runs.

**Delay.**
- **Calculated:** the slowest closed-loop root stays at Re ≈ −2.3 s⁻¹, with damping ratio ≥ 0.45, from 4 to 14 ms of pure delay. The loop remains stable up to 37.8 ms, against 7 ms normal worst case and 11 ms with every message in a 4 ms burst.
- **Allowed gain increase:** 13.5 dB at 7 ms and 10.2 dB at 11 ms. It falls below the 6 dB design floor only beyond about 19 ms.
- **Simulated:**
  - Burst storms (5 per second, 50–200 ms at 4 ms) and 5 % message loss leave C at 0.26° / 0.24° RMS with no events.
  - The combined corner (4 ms CAN each way, J −30 %, Kt +15 %) gives 0.58° RMS.

**Saturation.**
- **Calculated:** a saturating clamp acts as a loop-gain reduction.
  - With the nominal gravity stiffness, the linear loop is stable for *any* gain reduction, so saturation alone cannot destabilise it.
  - With a 1 kg lateral payload at ±80°, the local gravity stiffness is −0.32 N·m/rad, so the plant itself is unstable there. The loop then needs at least 0.39× of its designed gain. Deep, sustained saturation there can lose the position.
- **Mechanisms that keep saturation rare:**
  - the governor's torque budget (80 % of the reported limit minus 0.05 N·m)
  - back-calculation anti-windup, with the integrator bounded to ±Kt·I_limit
  - the drive re-clamping the delayed target to the newest limit
- **Simulated:**
  - **Governed path:** a 30° / 0.1 s request needs about 1.2 N·m against 0.448 N·m available. The governor reshapes it: the axis comes within 1° at 0.17 s, with 1.1° overshoot, no time at the limit and no events.
  - **Governor off:**
    - Anti-windup trades overshoot for a slow approach (open finding, §3).
    - With the real supervisor, the watchdog trips and re-arms 6 times. Motion stays bounded, but there is no repeated-trip lockout.
    - Derated E keeps the integrator bounded with no events (`tests/test_ctrl.py`).

**Quantization.**
- **Calculated:**
  - One count is 0.022°.
  - The feedback's high-frequency gain turns one count into about 33 mA, 1.0 % of the 3.2 A limit.
  - Differentiated velocity would be quantized at 0.19 rad/s, which is why friction compensation uses the reference velocity.
- **Simulated at 24-, 14- and 12-bit resolution:**
  - Run B RMS error is unchanged at 0.24–0.25°. Current jitter rises from 7.1 to 7.7 to 9.4 mA.
  - At a 45° hold with d(t) off, hunting is about 0.2° peak-to-peak at *every* resolution, including 24-bit. It is therefore friction stick-slip with integral action, not quantization.
  - The 14-bit encoder adds about 1 mA of current jitter at hold.

**Uncertain parameters.**
- **Calculated grid (108 points):**
  - J at −30 %, nominal, +30 %, and nominal + 1 kg payload
  - Kt at ±15 %
  - delay at 6, 7 and 11 ms
  - gravity stiffness +0.12, +0.02 and −0.34 N·m/rad, the last being a 1 kg lateral payload at its worst angle

  All 108 cases are closed-loop stable. The worst allowed gain increase is 6.1 dB, at 11 ms. The worst damping ratio is 0.31. With negative stiffness the loop also has a lower gain limit: it must keep at least 0.46× of its gain.
- **Simulated:** the stress sweeps in §6 cover coupling ×0.5–1.5, J, friction, gravity, Kt, R, L, bus 20–24 V, payload, bursts and encoder offset.

**What this does not show.**
- The calculations are local: they are linearised about an operating point, with the loop unsaturated.
- The simulations are samples, not proofs.
- Both use the known-yaw-plan mode. A causal yaw-estimate mode is untested.
- Nothing here covers the open governor defect below.

## 8. Open issues before the baseline can be frozen

**Resolved in Packet 2A: governor drift.** The former blocker was a stationary hold that drifted when yaw or load torque exceeded the budget: −0.64 rad at 0.4 N·m, a −37 rad runaway at 0.8 N·m, and 5.4 rad through the host.

- The hold now stays exactly in place and reports `over_budget` with a reason, standalone and through drive, transport and plant.
- The governor was rewritten and went through three independent review rounds. The findings of each round are regression tests in `tests/test_governor_contract.py`.
- Remaining limitations and declared tolerances are in the [Packet 2A report](packets/2A.md).

Open, in [Packet 2B](../EXECUTION_PLAN.md) scope unless noted:

1. **Anti-windup against feed-forward clipping** (§3).
2. **No repeated-trip lockout, and no recovery by fault class. This is a regression after 2A, and 2B must close it first.** An unfollowable loaded request cycles through watchdog trip and re-arm.
   - Payload E with a feedback-only loss, seeds 1–10: watchdog trips went from a median of 4 to a median of 33 (range 0–37), and fallback from about 3 % to up to 18.6 %. The governed peak is lower.
   - Monte Carlo D/E: the same trials cycle, but more often (117 and 139 trips in total, against 79 and 127).
   - Cause: after a re-arm the governor resumes the request at its nominal-budget speed.
3. **No yaw authority.** Without a yaw planner, the saturation-based yaw monitor still rejects the request into passive fallback. A 0.4 N·m reproducer runs away (skipped test M8c). Coupling beyond actuator capacity cannot be contained at the roll axis while the yaw reduction takes 0.5 s to blend in.
4. **Loaded D/E recovery:** fault events in 5–7 of 20 stress trials (see item 2).
5. **Information mode:** the yaw feed-forward assumes a known future plan. A causal mode is not yet available.
6. **Unmodelled payload** (Task 3): the governor judges feasibility with the nominal model. D/E spend 38–51% of stress-trial time (95th percentile) at the command limit.

Yaw reduced to 0.8× after brief outages is no longer observed: all three 60 ms outage cases end at yaw scale 1.00. The re-join now reserves the recent coupling peak, so the saturation that triggered the yaw monitor does not occur (Simulated).

## 9. Verification and reporting

Reproduce the tests and current Task 2 evidence:

```bash
python3 -m unittest discover -s tests -v
python3 -m exp.task2_eval
python3 -m exp.task2_robustness
```

Install the dependencies from `requirements.txt` first. `python3 run_all.py` includes this evidence generation alongside the historical analysis scripts.

The safety regressions cover:

- stale or non-finite feedback;
- feedback-only communication loss while outgoing commands remain fresh;
- timeout recovery dwell and non-finite commands;
- overspeed re-arm and mid-queue derating;
- empty feasible sets, absent yaw plans and unavailable coordination;
- the combined linear loop corner.

`tests/test_governor_contract.py` adds the Packet 2A governor contract:

- hold without drift;
- (q, v, a) consistency;
- path jumps, velocity kinks and inconsistent plans;
- fast moves, derates during motion and lost braking room;
- restricted holds and their resumption;
- non-finite inputs, and budget/capacity labelling;
- host-level yaw coordination and idle reporting.

Existing plant, numerical integration, quantization, delay, and controller tests also remain required.

The new evaluation reports A–E over five seeds, ten fault/electrical/payload cases, and twenty sampled plants for each of A–E. Unlike the earlier headline comparison, it shows both:

- **Original-request error**, evaluated at the original wall-clock time.
- **Governed-reference error**, evaluated against what the controller chose to follow.

Path-clock rate, yaw scale, current, saturation, fallback, and rejection accompany those errors. A small governed error during fallback or a slowed sweep must not be presented as completion of the original motion. Full numerical results and plots are in [task2_numbers.md](task2_numbers.md).

## 10. What is ready, and what remains uncertain

The implementation makes fault recovery, current-target limits, and request rejection explicit and testable. The local linear analysis and its simulator cross-check support stable behaviour under the stated delay, quantization and parameter ranges. It becomes a suitable frozen baseline for further comparisons and hardware qualification planning only after the §8 issues are closed.

Before hardware testing, establish the true torque/current and voltage conventions, measure command/feedback latency, identify yaw coupling, characterize the payload and thermal response, and define safe fallback clearance and coordinated yaw behavior. A ±2° RMS / ±5° peak performance target is not specified by the assessment and should not be retroactively used to label all runs successful. D/E may deliberately sacrifice substantial progress, and their unknown-load limitations motivate Task 3 rather than justify stronger safety claims here.

Implementation: [controller](../ctrl/baseline.py), [governor](../ctrl/governor.py), [supervisor](../ctrl/supervisor.py), [gain derivation](../ctrl/loopshape.py), [drive model](../sim/drive.py).
