# Hardware qualification plan: head-roll axis, frozen baseline `7d857df507c389c9`

**Status: Proposed, not performed.** Limits below are provisional project values, to be confirmed against the real application and fixture before testing.

**Instrumentation** (one timebase, ≥ 1 kHz): roll and yaw encoders; current target, pre-clamp command, measured current and active limit; bus voltage and PWM duty; winding/drive temperature; CAN send/receive timestamps; drive mode and fault state; original and governed references. Fixture: calibrated torque sensor, host-independent hard-wired e-stop, verified roll clearance with end stops or brake, load containment.

**Global stops** (any one → safe hold, end test): stale or lost feedback timestamps; wrong torque sign; unverified current/voltage convention; tracking error > 5°; ≥ 95% current utilization for > 100 ms; a latched fault the controller does not explain; temperature at the fixture limit (set below the simulated 130 °C trip); motion outside verified clearance.

| # | Test (in order; each gates the next) | Stage-specific stops | Acceptance to proceed |
|---|---|---|---|
| 1 | **Identify.** Axis blocked on the torque fixture: sign, K<sub>t</sub> (= K<sub>e</sub>), current offset, command and feedback latency distribution, electrical convention (V<sub>bus</sub> vs V<sub>bus</sub>/√3). | Any measured value outside the modelled ranges: K<sub>t</sub> ± 15%, latency > 4 ms bursts | Parameters within modelled ranges. Otherwise re-run the Task 2 margin analysis with the measured values before any motion. |
| 2 | **Static holds,** supported axis, derated 2.4 A: nominal payload, then known loads, at 0° and ±15°. | Holding current above the governor budget 0.8·K<sub>t</sub>I<sub>lim</sub> − 0.05 N·m | Hold error ≤ 0.5°. Loads whose calculated holding demand exceeds the budget are not attempted; the operator screens them, because the nominal-model governor cannot see the payload. |
| 3 | **Slow moves,** ±15° → ±45°, nominal load, then 3.2 A. | Error > 5°; sustained limit | ≤ 2° RMS and ≤ 5° peak against the governed reference; ≥ 95% path progress; no faults; original-request error reported alongside. |
| 4 | **Yaw disturbance,** roll held: low-amplitude yaw sweep, then B/C frequencies only if the fitted coupling predicts ≥ 20% current reserve. | Coupling above modelled reserve | Coupling fit validated at a held-out frequency (predicted residual within ±20%). ≤ 2° RMS in the admitted envelope; any yaw reduction recorded as changed delivery. |
| 5 | **Faults and derating** on the protected fixture: command-only, feedback-only and both-direction outages; 3.2 → 2.4 A derate mid-move; tracking-fault catch. | Fallback excursion beyond the verified clearance | No current target above the active limit. Re-arm only after the 50 ms fresh aligned dwell. Tracking-fault request stays suspended until replanned. Fallback excursion within clearance (simulation: up to 42° under load, so contain first). |
| 6 | **Payload and envelope:** the D-like shifted payload at reduced speed, then the loaded finite move. | As above | Loaded requests are reshaped or rejected as the governor reports. No request is completed by exceeding a limit. |

**Exit.** The baseline is qualified for the tested envelope only. The adaptive correction is not tested on hardware unless it first passes a newly registered simulation comparison.
