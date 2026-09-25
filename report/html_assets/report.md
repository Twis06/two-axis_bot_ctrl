# Safe control for a coupled two-axis robot

Engineering assessment · evidence snapshot · 24 September 2026

**Recommendation:** retain the frozen deterministic controller for hardware qualification. It tracks the assumed nominal A–C scenarios well, but loaded D/E requests remain substantially reshaped and a demanding loaded finite move fails to complete. The tested adaptive load correction does not meet its adoption criteria.

This report answers the five assessment tasks using the evidence currently on disk. **Observed** means only the supplied A–E summaries; **calculated** means model-based analysis; **simulated** means project experiments; **proposed** means unperformed work. No new control experiments were run to assemble this report; all current evidence was regenerated from committed sources before the build (packet R4). Expand the evidence sections for full tables and source narratives.

## Task 1 — Understand the failure {#task1}

**Leading explanation:** B/C expose inadequate rejection of yaw-dependent torque; D/E expose changed load demand interacting with current capacity. A has tracking error despite current headroom. These are supported hypotheses, not identified hardware failure causes.

| Run | Supplied observation | Engineering implication |
|---|---|---|
| A · shaped ±45° roll | 2.8° RMS, 6.1° peak; 2.1 A peak; no fault | Observed motor torque peaks at 0.294 N·m, below the 0.448 N·m ceiling. Examine compensation, response and timing. |
| B · yaw ±75°, 1.5 Hz | 4.2° RMS, 11.5° peak; 3.1 A peak; 8% clipped | Coupling-only demand is calculated as 0.136 N·m. Clipping is not explained by coupling alone. |
| C · yaw ±75°, 2.2 Hz | 7.8° RMS, 20.7° peak; 3.2 A for 31%; one watchdog trip | Calculated coupling rises to 0.247 N·m, 1.82× B; observed RMS rises 1.86×. |
| D · COM shifted 35 mm, ±80° roll | 9.1° RMS, +4.6° mean; 38% current-limited | Bias accounts for 26% of mean-square error; removing bias alone leaves about 7.9° RMS fluctuation. |
| E · D after derating | 12.6° RMS, 25.4° peak; repeated saturation at 2.4 A | A 25% reduction in available torque accompanies a 38% increase in RMS error. |

### What the motor permits

<div class="equation">τ<sub>m,max</sub> = K<sub>t</sub>I<sub>max</sub> = 0.448 N·m nominal; 0.336 N·m derated<br>τ<sub>required</sub> = Jq̈ + bq̇ + τ<sub>c</sub>tanh(q̇/0.02) + τ<sub>g</sub>sin(q) + τ<sub>couple</sub> − d</div>

At A's 45° endpoint, nominal gravity is 0.0849 N·m. Adding 0.040 N·m friction and 0.050 N·m disturbance allowances gives 0.175 N·m, leaving 0.273 N·m for acceleration and viscous damping. The full shaped trajectory's acceleration is unspecified, so this is not a feasibility certificate.

For sinusoidal yaw of amplitude A and angular frequency ω, the velocity and acceleration terms are in quadrature:

<div class="equation">τ<sub>couple,peak</sub> = A √[(0.008ω)² + (0.0008ω²)²]</div>

| Calculated near-zero-roll budget | B · 1.5 Hz | C · 2.2 Hz |
|---|---:|---:|
| Coupling peak | 0.136 N·m | 0.247 N·m |
| Coupling-only current | 0.97 A | 1.76 A |
| Coupling + disturbance + friction allowance | 0.226 N·m | 0.337 N·m |
| Share of nominal / derated capacity | 50% / 67% | 75% / 100.3% |

C is marginal under a conservative derated reserve; exact stationary holding is not proven impossible. For D/E, a 35 mm shift gives a gravity-moment scale of **0.343m N·m**, with mass *m* in kg. Neither mass nor shift direction is supplied. Slowing helps excessive dynamic demand; it cannot solve excessive static holding torque.

**What remains unknown:** the original controller and integrator, payload geometry, reference timing, voltage utilization, thermal history, and whether clipping counts commands or measured current. E does not prove windup; the reconstructed controller has no integrator and still produces saturation cycling. B's 3.1 A peak alongside clipping requires clarification of logging conventions.

**Most decisive experiment:** hold roll near zero and sweep yaw frequency, including B/C frequencies only within the qualified envelope. Log synchronized roll/yaw motion, current commands and measurements, active limits, packet timestamps, bus voltage and duty cycle. Fit the roll torque residual against yaw velocity and acceleration, then validate at a held-out frequency. This separates physical disturbance demand from control response.

<!-- FIGURES:task1 -->
<!-- DETAIL:task1.md|Full Task 1 derivation and uncertainty analysis -->

## Task 2 — Build a baseline {#task2}

**Design:** 500 Hz PI × lead feedback, nominal motion/gravity/friction feed-forward, causal yaw-coupling compensation, a torque-aware path governor, and a 1 kHz drive-local supervisor. The baseline uses timestamped measurements, not true plant parameters or future yaw motion.

<div class="architecture">Requested path → torque-aware governor → governed q, q̇, q̈<br>Roll encoder → PI × lead feedback &nbsp; + &nbsp; nominal feed-forward<br>Yaw encoder → causal Kalman estimate → coupling feed-forward<br>↓ feed-forward-priority clamp + conditional-integration anti-windup<br>CAN → 1 kHz supervisor → 1 ms delay → latest-limit clamp → current lag → plant</div>

| Design choice | Implementation / reason |
|---|---|
| Feedback | C(s) = 0.7553(1 + 2.8/s)(1 + s/7)/(1 + s/112), Tustin at 500 Hz; 4.46 Hz design crossover. |
| State estimates | Timestamped constant-acceleration Kalman yaw estimate, extrapolated 4 ms; drive-local filtered velocity for damping and overspeed. 14-bit encoder = 0.022°/count. |
| Friction and anti-windup | 80% nominal friction compensation using reference velocity; integrator cannot push further into clipped feedback headroom. |
| Governor budget | 0.8 Kt I_limit − 0.05 N·m, with nominal load model. Explicit PATH/JOIN/STOP/HOLD states and request disposition. |
| Timing | 1 kHz drive; 500 Hz host; 15 ms host feedback-age limit; 10 ms drive command timeout. No blocking on learning. |
| Limits | Applied target is clamped after the delay queue to the latest 3.2 A / 2.4 A limit. Measured current settles with the 1.2 ms lag. |

### Stability evidence and its limits

| Calculated condition | Phase margin | Gain margin |
|---|---:|---:|
| Design, 7 ms delay | 49.4° | 13.5 dB |
| Burst, 11 ms delay | 43.0° | 10.2 dB |
| 11 ms, inertia −30%, Kt +15% | 32.1° | 6.1 dB |

The local analysis finds a 37.8 ms pure-delay margin. Nonlinear delay/loss tests and encoder-resolution sweeps support the design within tested cases. At 14 bits, Run B current-increment jitter is about 31 mA versus 7.6 mA at 24 bits; filtering matters. These results do not prove stability under arbitrary saturation, switching or unknown payload. Negative gravity stiffness under some payloads makes deep saturation especially problematic.

### When to change or stop the request

| Situation | Required behavior |
|---|---|
| Dynamic demand exceeds budget, positions remain holdable | Slow traversal along the admitted path; report reshaping and delivered progress. |
| Static feasibility or braking room is lost | Restrict, stop or reject explicitly. Never move a stationary reference to cancel disturbance. |
| Yaw coupling exceeds roll authority | Request yaw reduction when coordination exists; otherwise suspend and request coordinated stop. |
| Communication outage | Local velocity damping; re-arm only after fresh valid aligned commands persist for 50 ms. |
| Tracking error >12° for 40 ms | Latch tracking fault; host must acknowledge the fault with a feasible catch. Original motion remains suspended until replan. |
| Overspeed / thermal / repeated faults | Fault-class recovery conditions; 3 motion/tracking latches in 30 s lock out. Thermal thresholds are assumed, not hardware-qualified. |

**Accepted limitation:** fallback damping reduces motion but does not hold against gravity. Packet 4B reports up to about 42° excursion in a tested loaded outage. The nominal-model governor can admit an unknown-load move that subsequently trips. Real hardware needs verified clearance, a coordinated yaw stop and, where necessary, a brake or tighter operating envelope.

<!-- FIGURES:task2 -->
<!-- DETAIL:task2.md|Full controller design, recovery contracts and operating envelope -->
<!-- DETAIL:task2_robustness.md|Detailed delay, saturation, quantization and uncertainty evidence -->

## Task 3 — Decide whether learning belongs {#task3}

**Decision: reject the tested adaptive candidate and retain the deterministic baseline.** This is a measured failure of this candidate's adoption criteria, not a claim that all learning is useless. A diagnostic oracle demonstrates substantial achievable compensation benefit.

The candidate fits an effective residual **θs sin(q) + θc cos(q)** from delayed measured roll/yaw, measured current, active limits and health timestamps. It changes feed-forward only; it cannot expand governor capacity or change fault limits. Unknown friction, calibration error and correlated disturbance may enter the fit, so coefficients are not identified payload mass or COM.

| Guard | Frozen implementation |
|---|---|
| Training | Healthy settled yaw-stationary 250 ms dwells; ≥20 samples per aggregate; ≥30 aggregates and 60° roll span with conditioning gates. |
| Timing | ≤50 Hz worker scheduling, one pending job, modeled 2–8 ms latency, zero-order hold; discard results older than 100 ms. |
| Bounds | Coefficient norm ≤0.40 N·m; correction ≤0.20 N·m; coefficient slew ≤0.10 N·m/s. |
| Ignore / disable | Stale, invalid, faulted, saturated, mismatched, insufficiently covered or expired data; prior pending results cannot restore trust after invalidation. |

L1 has 15 focused tests; L2 contains 60 replay-like audit runs with zero structural violations; L3 has 7 focused adapter tests. The L4 packet has **357 runs: 48 tuning, 300 held-out, 9 unholdable-load checks**. These are different forms of evidence and should not be added together as independent hardware trials.

### Matched comparison

All variants receive the same calibration trajectory and time budget. The protocol, loads, seeds, metric and gate are cited in a commit made before the candidate was run; the plan file itself was first committed together with the adaptive results. The held-out matrix uses five load laws, both 3.2 A and 2.4 A limits, and seeds 101–105. The primary metric is RMS governed error during the **first 0.5 s of each test dwell**, after calibration, excluding fallback. Completion and fault counts must accompany it.

| Variant | Median primary error | Median paired reduction vs int1 | Completed | Interpretation |
|---|---:|---:|---:|---|
| int0.5 | 2.914° | −37.5% | 18/50 | Slower integral comparator. |
| int1 · frozen baseline | 2.130° | reference | 40/50 | Best tuning-selected setting meeting declared loop-margin criteria. |
| int2 | 1.157° | +42.6% | 50/50 | Faster, but fails the frozen margin criteria: 43.7° nominal PM, 28.1° corner PM / 5.7 dB GM. |
| Adaptive feed-forward | 2.088° | **0.0%** | 40/50 | Only 37/50 runs obtain a usable estimate (74% versus required 80%); active while scored in 22/50, where 8 pairs improve 42–79%. |
| Oracle feed-forward only | 0.499° | +76.9% | 50/50 | Uses true load; diagnostic and unavailable to a deployed controller. |
| Oracle with governor load access | 0.400° | +78.6% | 35/50 | Loses 15 comparator completions; a low error score alone is insufficient. |

**Why exactly 0.0%:** the candidate is `int1` plus the learned term on the same seed. In 30 of 50 pairs the correction never changed a scored command: it was never usable in 13, and first usable only at 18.3–21.6 s in 17, where bumpless transfer cancels a change at constant reference. Those runs are bit-identical to `int1`. Twenty pairs improve and none gets worse. The median of paired percentage reductions also differs from the percentage change between two aggregate medians. The machine-audited gate (`exp/task3_gate.py`) returns **fail**. The candidate loses no comparator waypoint or completion. It has no pre-clamp command above the limit and no watchdog trip, suspension, rejection or lockout in the 50 held-out cases, but it fails the ≥10% benefit and ≥80% availability criteria. The 10 incomplete int1 and adaptive runs are all at load (−0.06, +0.14): they reach every waypoint but overshoot the ±65° range by 5.65–6.89°, beyond the 5° allowance.

<!-- INTERACTIVE:learning -->

**Scope of the verdict:** the main L4 matrix has stationary yaw. Registered supplementary challenges were run with 96 runs, and none showed a limit, fault or waypoint regression:

- B- and C-yaw, a 60 ms feedback outage and a 3.2 → 2.4 A derate;
- onsets at 9 s and 12.5 s, plus a late-onset (17.5 s) amendment registered after the first results;
- loads (0, 0.18) and (0.06, 0.14), seeds 201–203.

The correction was usable after onset in only 23/48 candidate challenge runs, and never in the 9 s-onset yaw B/C sets, so most challenge runs exercise the baseline. The payload-change challenge was **not executed**, so the gate's challenge criterion is `incomplete`. Safety statements are limited to these evaluated conditions and this exact configuration. None of this changes the benefit failure.

**What would change the choice:** a redesigned estimator that acquires useful correction reliably, passes a newly registered held-out comparison and completes the remaining adaptive stress challenges. Preserve the current results; do not tune against them and relabel them unseen. No feed-forward estimator can hold a truly infeasible static load above the actuator's capacity.

<!-- DETAIL:task3_ceiling_numbers.md|All L4 tuning, held-out, per-load and unholdable-case tables -->
<!-- DETAIL:task3_challenges_numbers.md|Registered supplementary challenge results -->
<!-- DETAIL:task3_estimator_audit.md|L2 audit summary and bias / abstention results -->
<!-- DETAIL:task3.md|Full Task 3 implementation and decision narrative -->

## Task 4 — Make evidence {#task4}

**Final design equals the frozen baseline.** Learning was evaluated and not adopted. The table below uses current Task 2 generated evidence, not the earlier Phase 2 controller, and compares assumed scenarios rather than measured hardware improvements.

| Run | Legacy RMS | Final governed RMS | Final original-request RMS | Net path progress | Current RMS | At command limit | Tracked |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | 3.50° | 0.68° | 0.74° | 100% | 0.61 A | 0.0% | 5/5 |
| B | 3.57° | 0.27° | 0.27° | 100% | 0.70 A | 0.0% | 5/5 |
| C | 7.64° | 0.40° | 0.40° | 97% | 1.15 A | 0.0% | 5/5 |
| D | 10.23° | 4.10° | 76.68° | 83% | 1.46 A | 1.9% | 0/5 |
| E | 12.02° | 4.25° | 78.49° | 51% | 1.32 A | 5.5% | 0/5 |

Five-seed medians, except counts. Net progress is path-clock progress, not a measured speed ratio. “Tracked” requires ≥95% progress, ≤2° request-window path RMS, and no rejection, suspension or tracking fault; these are project thresholds. D/E's large original-time error exposes sacrificed timing. Their small governed error is not delivery of the original request. E has 146 saturation entries despite 5.5% occupancy.

### Useful motion, uncertainty and faults

Unloaded M1 and yaw-disturbed M3 complete 5/5, at median 2.09 s and 2.34 s respectively (requested 2.10 s and 2.40 s). Loaded, derated M2 completes **0/5**, trips once per run and stays suspended. Across 20 selected uncertainty trials per run, A/B/C have no events but only 13/20, 20/20 and 15/20 meet tracking criteria; D/E have events in 5/20 and 6/20 and meet tracking criteria in 0/20.

**A request that should not be tracked:** Packet 4B's INF-P load requires about 0.41 N·m at zero roll against only 0.336 N·m derated capacity. The actual nominal-model baseline initially attempts it, faults and catches at shifted positions; this limitation is reported. It is not a successful zero-degree hold. Task 3's separate unholdable case likewise has 0% completion; the truth-informed oracle rejects it, whereas baseline/adaptive suspend after faults.

The simulator includes 10 kHz RK4 plant integration, 1.2 ms current lag, a fixed 1 ms command delay, asymmetric transport effects, 14-bit position sensing, target limits and voltage limiting. Stress ranges include J ±30%, Kt ±15%, R ±25%, L ±20%, 20–24 V bus, load and coupling variation. R = 1.8 Ω, L = 0.45 mH and Ke = Kt use a matching SI convention. The assumed Vbus/√3 drive convention needs hardware verification; Packet 4B's 8–18 V probes deliberately go beyond the required 20 V floor.

<!-- FIGURES:task4 -->

### Evidence quality and reproduction status

The frozen baseline fingerprint is `7d857df507c389c9`. Task 2 publishes 177 manifested runs, Packet 4B 397, L4 357 and the Task 3 challenges 96. These sets overlap in purpose and must not be presented as independent samples. `python3 -m unittest discover -s tests` passed **255 tests** at commit `039f83b` (2026-09-24). This HTML build does not rerun the suite, and passing tests do not validate an experiment design.

**Publication and reproduction:** `python3 run_all.py` now regenerates every current evidence packet: Task 1 calculations, Task 2, Packet 4B, the Task 3 audit, L4 and challenges, and Task 5. All of them were republished from committed sources (packet R4), with Packet 4B at its reviewed round-2 state. The clean-snapshot reproduction and its tolerances are reported in packet R4. The final independent submission review (R6) remains open.

<!-- DETAIL:task2_numbers.md|All current baseline results, uncertainty ranges and run identifiers -->
<!-- DETAIL:task4b_numbers.md|Packet 4B results (reviewed round-2 evidence, republished by R4) -->
<!-- DETAIL:packets/4B.md|Packet 4B review history and round-2 findings -->
<!-- DETAIL:packets/R4.md|Publication and clean-snapshot reproduction record -->

## Task 5 — Test the explanation {#task5}

**Registered test, retrospectively corrected analysis:** weaken true Kt and Ke by 10% in nominal Run B, keep the controller unchanged, and pair feed-forward on/off across seeds 21–23. Score 2–8 s, with one window for every windowed quantity. The registration file is unchanged, but it and the original results were first committed together, so git gives no evidence of ordering. Packet R2 corrected mixed windows and a torque/current unit error in the original analysis.

| Prediction | Basis | Predicted | Measured (Simulated) | Verdict |
|---|---|---:|---:|---|
| Peak coupling torque | registered | 0.135622 N·m | 0.135622 N·m | Consistency check only (same equation) |
| Peak ideal coupling current, nominal Kt | registered | 0.9687 A | 0.9687 A | Consistency check only |
| Commanded coupling FF current | post hoc diagnostic | — | 1.160 A peak | Diagnostic: +5% sustained gain, transient peak overshoot |
| Extra ideal current at Kt ×0.90 | registered | +0.1076 A | commanded FF unchanged | **Not tested**: component not separable; ±0.15 A includes 0 |
| Coupling residual increase | post hoc operationalization | +0.0136 N·m | −0.0007 N·m | **Failed** (plan-mode diagnostic +0.0123 N·m) |
| Yaw FF remains valuable | post hoc criterion | — | FF off worse by 3.51–3.89° | Supported (3/3) |
| Weaker motor leaves more error | post hoc criterion | — | +0.054° median | Not uniform (2/3) |

Windowed governed RMS with feed-forward on is 0.328° median for the weakened motor versus 0.274° nominal, and 3.936° with feed-forward off. There are no events or clipping in any row.

<!-- INTERACTIVE:prediction -->

**Interpretation:** yaw feed-forward is useful in this controlled simulation. The registered numerical predictions are either arithmetic on the prescribed model or were not measurable as registered. The residual prediction fails under the frozen baseline's causal estimate, whose transient error dominates the residual peak, but holds with exact feed-forward. No closed-loop tracking prediction was registered; that stronger test and the hardware residual test remain open.

**Smallest justified design change: none.** Retain the frozen controller; this experiment does not justify retuning. Confirm effective torque/current calibration, the electrical convention and timestamped yaw-disturbance response before changing compensation on hardware.

<!-- DETAIL:task5.md|All 12 paired runs and prediction-versus-result discussion -->

## Hardware qualification proposal {#hardware}

This is a proposed test sequence, not completed qualification. Use the existing project's 2° RMS / 5° peak goals as provisional acceptance targets, with ≥95% admitted-path progress, no unexplained fault and no target-current limit bypass. Confirm the application-specific targets and physical clearance before testing.

| Stage | Instrumentation / procedure | Progression and stop criteria |
|---|---|---|
| 1 · Instrument and identify | Synchronized ≥1 kHz encoder, current target/measurement, active limits, bus/duty, temperature and packet timestamps; calibrated torque fixture and independent emergency stop. Identify sign, Kt/Ke, latency, offsets and load moment. | Stop on wrong sign, invalid feedback, lost timestamps or unverified current convention. Establish collision-free clearance and load containment before powered motion. |
| 2 · Static holds | Mechanically supported setup; first nominal payload, then identified loads, initially ±15° at derated 2.4 A. | Admit only poses whose conservative estimated holding demand fits 0.8 Kt I_limit − 0.05 N·m. Do not admit unknown static load from simulation alone. |
| 3 · Limited sweeps | Begin slow ±15° moves; increase toward ±45° only after each case passes. Log original and governed requests separately. | Stop on >5° tracking error, sustained ≥95% current utilization for 100 ms, or a predefined clearance/temperature boundary, whichever occurs first. These are proposed test stops, not firmware-certified limits. |
| 4 · Yaw disturbance | Roll-held low-amplitude yaw sweeps, then B/C frequencies if feasible. Fit coupling and validate at a held-out frequency. | Require ≤2° RMS / ≤5° peak in the admitted envelope; record any yaw reduction as changed delivery. Stop if coupling exceeds modeled available reserve. |
| 5 · Fault / derating tests | Protected fixture: command-only, feedback-only and bidirectional outages; 3.2→2.4 A derating; verify catch, re-arm and suspension. | No target limit bypass; communication recovery requires the 50 ms fresh aligned dwell. Tracking-fault requests stay suspended until replan. Accept only if measured fallback motion fits physical clearance. |

The simulated 25 rad/s overspeed trip and 130°C thermal trip are not initial bench operating targets. Use lower verified hardware/fixture limits where required. The loaded ≈42° fallback excursion makes an unprotected outage test inappropriate; establish containment or reduce speed first. Reconsider adaptive control only after baseline qualification and a new evidence gate.

## Sources, artifacts and remaining work {#sources}

The assessment PDF supplies the plant constants, task questions and the only hardware observations. Project Markdown reports, generated JSON manifests, code and review packets supply the calculations and simulated evidence. No outside measurements or external technical references are used; see the references note.

The implementation uses Python, NumPy, SciPy, Matplotlib and standard-library unittest. Automated coding agents contributed implementation, experiments, review and documentation, as recorded in the packet histories and Task 3 execution log. This HTML report was assembled by Codex from those artifacts; its charts reorganize stored rows and do not create new experimental evidence. The offline report builder uses Python-Markdown; HTML/CSS/JavaScript provide presentation. A repository-wide third-party reused-code/license audit has not been established by the available notes.

The HTML is a comprehensive report with appendices. The page-limited submission documents are separate: the [four-page memo](memo.pdf), the [one-page hardware qualification plan](hardware_qualification_plan.pdf) and the [references, reused-code and tools note](references_and_tools.md). The final independent review (R6) remains open. Hardware experiments above remain proposed.

<!-- SOURCES -->

## Historical figures {#history}

These figures document the earlier design and reconstruction. They are included for completeness, but their gains, controller versions and aggregate performance must not replace the frozen-baseline results above. Payload fits are assumed reconstructions, not identified hardware payloads.

<!-- FIGURES:history -->
