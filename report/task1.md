# Task 1 — Understand the failure

**Answer [Hyp]:** The evidence points to two main problems: yaw-dependent disturbance rejection in B/C, and changed payload loading aggravated by reduced torque capacity in D/E. Run A shows tracking error even with current headroom. The right first step is to establish the torque budget and identify the disturbance, before changing gains.

**Evidence boundary:** The only observed results are the summaries in the [assessment](../Robotics%20Controls%20Technical%20Assessment.pdf). The controller structure, raw time series, trajectory timing, payload mass, and temperature history are unknown. Calculations below use the supplied model. Later simulator results test possible explanations; they do not establish what happened on hardware.

**Claim labels used below:**
- **[Obs]** a number in the assessment summaries
- **[Calc]** a consequence of the supplied model under stated assumptions
- **[Hyp]** an interpretation the evidence supports but does not prove
- **[Sim]** a finding from this project's simulator, not a hardware observation

## 1. What each run tells us

| Run | Test condition | Observed tracking | Observed actuator/fault behavior | Supported interpretation |
|---|---|---|---|---|
| A | Shaped moves to ±45° roll; yaw stationary | RMS 2.8°, peak 6.1° | Peak current 2.1 A; no fault | Tracking is imperfect despite current headroom. [Calc → Hyp] Peak current is about 66% of the nominal ceiling, so the current limit is unlikely to be the cause; voltage limits and trajectory feasibility are not excluded. |
| B | Roll held at 0°; yaw sine ±75° at 1.5 Hz | RMS 4.2°, peak 11.5° | Peak current 3.1 A; 8% of samples current-clipped | Holding roll requires disturbance rejection. Clipping is already occurring. |
| C | Same as B; yaw frequency increased to 2.2 Hz | RMS 7.8°, peak 20.7° | Current at 3.2 A for 31% of samples; one tracking watchdog trip | Increased yaw excitation accompanies worse tracking, more saturation, and a fault. |
| D | Payload COM shifted 35 mm; roll sweep ±80° | RMS 9.1°; signed mean +4.6° | Current-limited for 38% of the run | A directional error bias accompanies substantial current limiting after a load change. |
| E | Same trajectory as D after thermal derating to 2.4 A | RMS 12.6°, peak 25.4° | Repeated exits from and re-entries into saturation | Reduced available torque makes the same request harder to execute (assuming E keeps D's payload shift, which the summary does not state). |

“Not reported” must not be read as zero. For example, D's peak error and E's saturation fraction are not given. B's 3.1 A peak together with clipping also means the logging definitions need checking: commanded versus measured current, rounding, and sampling could differ.

## 2. Estimate the motor's limits

### Common torque capacity

The supplied torque constant is $K_t=0.140\ \mathrm{N\,m/A}$, so:

$$
|\tau_m|\le K_t I_{\max}=
\begin{cases}
0.448\ \mathrm{N\,m}, & I_{\max}=3.2\ \mathrm A,\\
0.336\ \mathrm{N\,m}, & I_{\max}=2.4\ \mathrm A.
\end{cases}
$$

Thermal derating removes **25% of peak torque capacity**. These are current-based ceilings; voltage limits can reduce attainable torque during sufficiently fast motion or current changes.

For exact tracking, the motor must supply

$$
\tau_{\rm req}=J_r\ddot q_r+b\dot q_r+
\tau_c\tanh(\dot q_r/0.02)+\tau_g\sin q_r+
\tau_{\rm couple}-d(t),
$$

plus any payload terms missing from the nominal model. Motor torque must cover all simultaneous terms, not just roll acceleration.

### Run A: headroom exists, but trajectory feasibility is not fully known

The observed 2.1 A peak corresponds to **0.294 N·m**, about 66% of the nominal torque ceiling. Nominal gravity at 45° is

$$
0.120\sin45^\circ=0.0849\ \mathrm{N\,m}.
$$

Adding the maximum modeled Coulomb-friction magnitude and disturbance bound gives a conservative low-speed load budget of approximately

$$
0.0849+0.040+0.050=0.175\ \mathrm{N\,m}.
$$

This leaves roughly 0.273 N·m below the nominal ceiling for acceleration and viscous damping at that angle. At exactly zero speed, the supplied tanh friction term is zero; the 0.040 N·m allowance is a conservative moving-load allowance, not a measured static friction torque.

**Verdict [Calc → Hyp]:** The observed run is not exhausting the nominal current capacity. Finite feedback response, imperfect motion compensation, or timing/estimation errors are plausible. Full trajectory feasibility cannot be certified without the shaped reference's velocity and acceleration. The summaries do not prove that inertia or gravity feed-forward is absent.

### Runs B/C: compute the yaw disturbance before interpreting saturation

Assume the stated yaw sine is $q_y=A\sin\omega t$, where $A=75\pi/180=1.309\ \mathrm{rad}$. Then

$$
\dot q_y=A\omega\cos\omega t,\qquad
\ddot q_y=-A\omega^2\sin\omega t.
$$

The velocity and acceleration terms are a quarter-cycle apart, so the coupling peak is their root-sum-square, not the sum of their separate peaks:

$$
\tau_{\rm couple,pk}
=A\sqrt{(0.008\omega)^2+(0.0008\omega^2)^2}.
$$

| Quantity | B: 1.5 Hz | C: 2.2 Hz |
|---|---:|---:|
| Velocity-coupling amplitude | 0.099 N·m | 0.145 N·m |
| Acceleration-coupling amplitude | 0.093 N·m | 0.200 N·m |
| Combined coupling peak | **0.136 N·m** | **0.247 N·m** |
| Equivalent current for coupling alone | 0.97 A | 1.76 A |
| Coupling + 0.050 N·m disturbance + 0.040 N·m friction allowance | **0.226 N·m** | **0.337 N·m** |
| Conservative allowance as share of nominal capacity | 50% | 75% |
| Conservative allowance as share of derated capacity | 67% | 100.3% |

These are near-zero-roll holding budgets. They do not include substantial corrective roll acceleration or velocity once tracking has already deteriorated. For mathematically exact stationary holding, modeled friction and roll gravity are zero: coupling plus the disturbance bound is 0.186 N·m for B and 0.297 N·m for C.

**Verdict [Calc]:** Both runs have nominal-model torque headroom at 3.2 A for near-zero-roll holding. C becomes marginal at 2.4 A once correction/friction reserve is included. The 0.337 versus 0.336 N·m comparison is too close, and too conservative, to prove that exact holding is physically impossible. It does justify reshaping a C-like request when derated to 2.4 A, under a policy that requires operating reserve. (Observed C ran at 3.2 A, where the calculation shows headroom.)

**[Obs + Calc]** B→C is particularly informative: modeled coupling increases **1.82×**, observed RMS error **1.86×**, and peak error **1.80×**. **[Hyp]** This supports yaw coupling as a major contributor. It does not identify the feedback gains or establish a particular delay-induced failure.

**[Obs + Calc]** The observed current peaks exceed the coupling-only requirement considerably: B 3.1 A against 0.97 A; C is clipped at 3.2 A against 1.76 A, so its true demand is unknown and 3.2 A is only a lower bound. That gap could reflect corrective dynamics, poorly damped response, noisy control effort, additional loads, or inaccurate coupling coefficients. Comparing these peaks is not a measurement of controller efficiency because their time alignment is unknown.

### Runs D/E: load uncertainty prevents a unique torque calculation

A COM shift changes gravity torque and may change inertia. For a mass $m$ displaced by 35 mm, the scale of the gravity-moment change is

$$
mg\Delta r\approx0.343m\ \mathrm{N\,m},\quad m\text{ in kg}.
$$

Its angular dependence depends on the shift direction. A general planar gravity representation is $a\sin q_r+c\cos q_r$. A lateral component can produce a directional load over the sweep. Neither the mass nor the shift direction is supplied, so assigning a specific payload torque or mass would require an assumption.

**[Calc from Obs]** D's bias is substantial: $4.6^2/9.1^2\approx26\%$ of its mean-square error is associated with the nonzero mean. The remaining fluctuation has RMS approximately 7.9°. Thus, removing a constant bias alone would leave considerable tracking error.

**[Obs + Calc]** E reduces torque capacity by 25% while RMS error increases by **38%** relative to D. This assumes E retains D's payload shift; the summary says only "same trajectory". **[Hyp]** This is consistent with actuator limits contributing materially to the poor tracking.

**Verdict [Obs → Hyp]:** D and E are current-limited under the observed controller, and E has less physical capacity for the same motion. The summaries do not establish whether the entire trajectory is fundamentally infeasible for every controller. That requires payload torque and trajectory timing. Slow the request if dynamic torque is excessive; restrict or reject positions if their static holding demand exceeds capacity. Slowing cannot solve an excessive static load.

## 3. Leading explanation and competing causes

**Leading explanation [Hyp]:** the dominant failure is *disturbance torque that the controller does not anticipate or reject within its current limit*: yaw coupling in B/C, changed payload gravity in D/E, and less capacity in E. Run A shows that tracking is imperfect even without this. The per-run statements below are the parts of that explanation.

**[Hyp] B/C: inadequate rejection of the yaw-dependent disturbance.** Faster yaw increases the modeled disturbance, and the errors increase by a similar factor. Predictable coupling compensation is therefore a justified candidate. Feedback delay and estimator behavior may worsen rejection, but their contribution cannot be identified from aggregate errors.

**[Hyp] D/E: changed load demand interacting with the torque ceiling.** A gravity-model mismatch is a plausible explanation for D's bias. Thermal derating then leaves less corrective torque in E. Request limiting and saturation-aware control are needed even if load compensation improves.

**[Obs + Calc → Hyp] A: imperfect tracking without evidence of current exhaustion.** Motion feed-forward, feedback response, and timing deserve examination before raising the current limit or assuming a larger motor is necessary.

| Alternative | What the summaries leave open | How it changes the decision |
|---|---|---|
| Coupling model mismatch or additional cable torque | Similar B/C error scaling does not uniquely validate the two coefficients | Identify the disturbance before relying on precise feed-forward cancellation |
| Estimator noise, delay bursts, or poorly damped feedback | No spectra, timestamps, or current/error traces are available | Improve estimation or timing, or revise loop design, if these dominate current demand |
| Integrator windup | E's repeated saturation does not establish that an integrator exists | Include anti-windup if using integral action, but do not label windup as the proven cause |
| Voltage/back-EMF limits | Bus voltage, duty cycle, and roll speed are not logged | Add electrical feasibility limits if voltage is exhausted |
| Sensor/reference offset or asymmetric sampling | D's signed mean need not come exclusively from gravity | Check calibration and sweep coverage before interpreting the bias as payload identification |

**[Sim]** The existing reconstructed PD controller has **no integrator**. It was fitted by search under an assumed 0.7 kg lateral payload and a 1 Hz sweep:

- it reproduces the RMS ordering within about 25% and shows repeated saturation entries in the E condition;
- it misses the observed peaks (A 10.6° vs 6.1°, B 6.3° vs 11.5°, C 12.9° vs 20.7°), B/C/D's current limiting, and the sign of D's mean error (−4.9° vs +4.6°).

So saturation cycling does not by itself imply windup, and the summaries admit multiple explanations. This is a consistency check, not identification of the original controller. See [Task 4](task4.md).

## 4. Timing, sensing, electrical, and thermal limits

- **Timing:** Keep the fixed 1 ms command delay, CAN latency of 0.6–1.8 ms with occasional 4 ms bursts, and the 1.2 ms first-order current lag separate. Feedback transport, sampling, and command hold contribute additional loop phase lag. Neither a sinusoidal CAN-delay pattern nor a specific total observed delay is supplied.
- **Sensing:** A 14-bit encoder gives $2\pi/16384=0.0003835$ rad, or 0.022° per count. One-count finite differences correspond to 0.383 rad/s at 1 kHz and 0.192 rad/s at 500 Hz. These increments exceed the 0.02 rad/s friction scale, so raw differentiated velocity is unsuitable for abrupt friction compensation. Quantization alone does not explain multi-degree position errors, but control action can amplify its effects.
- **Electrical:** Use $V=Ri+L\,di/dt+K_e\dot q_r$, with a drive-voltage convention stated explicitly. At $R=2.25\ \Omega$, 3.2 A requires 7.2 V resistively. [Calc] A current reversal adds about 2.9 V ($L\,di/dt$). At 24 V the speed allowed at 3.2 A is about 120 rad/s with a DC-equivalent convention and 48 rad/s with $V_{bus}/\sqrt3$. Run A needs about 2.3 rad/s and C's roll error motion ≤5 rad/s ([Phase 0](phase0_numbers.md)). Under either convention, the voltage limit is therefore unlikely to bind in A–C if those speed estimates hold. D's sweep speed is unknown. This does not by itself establish that voltage limits caused, or could not have caused, the recorded errors: roll speed and duty cycle are missing. Large yaw speed is not directly the roll motor's back-EMF speed.
- **Thermal:** E explicitly establishes thermal derating. Other runs establish current loading, not temperature histories or thermal causation. Under the simplified $i^2R$ convention, the stated 31% occupancy at 3.2 A in C implies $i_{\rm RMS}\ge3.2\sqrt{0.31}=1.78$ A and resistive loss of at least 5.7 W at 1.8 Ω. This is a heating estimate, not proof of overheating; cooling, duration, and the motor's electrical convention matter.

## 5. What the summaries cannot prove

1. Whether saturation precedes the large tracking errors or follows corrective commands after errors develop.
2. The original controller structure, gains, rate, estimator, feed-forward, or anti-windup behavior.
3. Payload mass, COM shift direction, roll sweep timing, or the required inertial torque in D/E.
4. Whether current-clipping statistics refer to commanded or measured current and how samples were aligned.
5. Whether delay bursts, voltage limits, temperature changes, or external disturbance pulses coincide with failures.
6. A unique physical failure cause, or guaranteed feasibility for the complete trajectories.
7. The numerical tracking acceptance threshold; the assessment says the goal is missed but does not specify it. Any proposed 2° RMS / 5° peak target is a design assumption.

## 6. The single experiment most likely to change the diagnosis

**Repeat the roll-held yaw test at several frequencies, with synchronized current and motion logging.** Begin with an amplitude that stays within the torque envelope; increase excitation only while current and tracking stop conditions permit. Include the B/C frequencies if safe.

Log at 1 kHz: roll and yaw position, reference, commanded and measured current, active current limit, and timestamped command/feedback arrivals. Record bus voltage and drive duty/voltage utilization alongside these signals if available. Estimate velocities and accelerations offline with filtering and time alignment documented.

Using the supplied sign convention, form the roll torque residual

$$
r(t)=K_ti-J_r\ddot q_r-b\dot q_r
-\tau_c\tanh(\dot q_r/0.02)-\tau_g\sin q_r.
$$

With the nominal payload, $r(t)=\tau_{\rm couple}-d(t)$. Fit its dependence on yaw velocity and acceleration, then test the fitted model on a held-out frequency. Position-dependent residuals can indicate a missing cable/gravity term, but derivative estimates, correlated disturbance, and sinusoidal regressors limit identification. Multiple frequencies help distinguish yaw-position effects from yaw-acceleration effects.

**Decision:** If the supplied coupling model predicts measured load across frequencies and voltage has headroom, prioritize yaw feed-forward with bounded feedback correction. If the load is substantially different, identify that model first. If load demand is modest but current spikes align with estimator noise or transport events, prioritize the feedback/timing path. If the required load exceeds available torque, reshape or reject the request.

This experiment is more decisive than gain tuning because it separates the physical torque demand from the controller's response to it.

## Supporting calculations and corrections to earlier drafts

[Phase 0 calculations](phase0_numbers.md) provide additional conditional estimates; [Phase 1 results](phase1_numbers.md) show the reconstruction mismatch. Earlier phase text described these more conclusively than the summaries justify. The qualified statements above are the current diagnosis and **supersede**:

- phase0_numbers' claim that A "is what feedback-only tracking would give";
- that the evidence "favours a lateral load";
- that E's cycling is "better explained by integrator windup";
- that C "must be reshaped/rejected when derated". Analytic bandwidth calculations describe a selected controller under a delay model, not an identified bandwidth limit of the unknown original controller.
