# Task 4 — Make evidence

**Status:** Simulator, reconstructed legacy controller, deterministic baseline, and recorded evaluations exist. Packet 4B's independent review is PASS WITH ISSUES with findings addressed. Task 3's registered adaptive comparison is now recorded in the Task 3 packet; the tested candidate failed its adoption gate, so the deterministic baseline remains final. The tables in the next two sections are **historical** (the Phase 1 reconstruction and the Phase 2 design), transcribed from the phase reports. Current-controller evidence is in [Task 2](task2_numbers.md), [Packet 4B](task4b_numbers.md), [Task 3](task3_ceiling_numbers.md) and [Task 5](task5.md), all republished from committed sources by R4.

> For the revised Task 2 controller, use [the regenerated evidence](task2_numbers.md). The tables below preserve the earlier investigation and are not current-controller performance claims.

## Simulator and reproducibility

Run from the repository root:

```bash
uv run python run_all.py       # or: pip install -r requirements.txt && python3 run_all.py
```

The shorter `python3 run_all.py --quick` runs tests and analytic calculations only; it does not regenerate the full experiment suite.

The simulator includes roll dynamics, first-order current lag, current-command delay, CAN transport, multirate execution, 14-bit encoders, current limits, and electrical voltage constraints. Uncertainties include inertia, payload, friction, coupling, motor strength, R/L, bus voltage, and latency. See [the simulator report](phase1_sim.md) for assumptions and [configuration](../sim/config.py) for implementation.

Electrical checks use the specified nominal R = 1.8 Ω, L = 0.45 mH, matching SI motor constants, and uncertainty ranges. The simulator uses a conservative `V_bus / sqrt(3)` voltage convention. That drive convention must be verified against hardware rather than treated as specified by the brief.

The final suite has 266 tests; the earlier `dff8efd` snapshot passed 255 (see [R4](packets/R4.md)). Passing tests show that the code does what the tests check; they do not validate an experiment design.

## Reconstruction is a consistency check, not identification

The reconstructed legacy controller was selected from a search, with assumed trajectories and payload. Its RMS agreement is therefore a fit, not independent validation of the original hardware controller.

| Run | Observed RMS | Reconstructed RMS | Observed limited time | Reconstructed limited time |
|---|---:|---:|---:|---:|
| A | 2.8° | 3.5° | Not explicitly stated; peak below nominal limit | 0% |
| B | 4.2° | 3.6° | 8% | 0% |
| C | 7.8° | 7.6° | 31% | 0% |
| D | 9.1° | 10.2° | 38% | 0% |
| E | 12.6° | 12.0° | Repeated saturation; fraction unspecified | 30% |

The model reproduces approximate RMS ordering and some payload/saturation behavior, but not B/C/D's current limiting. This mismatch leaves controller, trajectory, sensing, and load assumptions unresolved. It does not prove that the stated disturbance bound is violated. The reconstructed controller has no integrator, so E-like saturation cycling does not require windup.

Source: [reconstruction results](phase1_numbers.md).

## Baseline comparison inside the simulator (historical Phase 2 design)

The following compares two simulated controllers under the same assumed scenarios. It is not a measured hardware improvement.

| Run | Legacy RMS | Baseline RMS | Baseline delivered path speed | Time at limit: legacy / baseline |
|---|---:|---:|---:|---:|
| A | 3.50° | 0.55° | 100% | 0% / 0% |
| B | 3.57° | 0.20° | 100% | 0% / 0% |
| C | 7.64° | 0.22° | 100% | 0% / 0% |
| D | 10.23° | 2.85° | 82% | 0% / 2% |
| E | 12.02° | 2.93° | 48% | 30% / 5% |

Baseline error is measured against the governed reference. D/E trade speed for tracking. Their remaining short saturation episodes reflect a limitation of the nominal load model; E's baseline has more saturation entries despite less total saturated time. Both duration and entry count matter.

Source: [baseline results](phase2_numbers.md).

## Uncertainty, timing, and fault evidence (historical Phase 2 design unless marked current)

- Recorded Monte Carlo testing covers 40 sampled plants for each of A/B/C. Baseline p95 RMS errors are 1.6°, 1.3°, and 2.0°, with zero recorded fault events across those 120 trials. This does not establish zero fault probability, and it is not equivalent to a D/E payload generalization study.
- Fault cases include a 60 ms CAN blackout, burst storms, and 5% message loss. The blackout invokes drive-local damping and subsequent re-arm.
- At 2.4 A with coupling multiplied by 1.5, the baseline requests yaw shrinkage. Recorded RMS is 1.6° with about 1% time at the limit. This is an explicit case where it does not deliver the original requested motion.
- At 20 V, R increased by 25%, and a fast ±80° sweep, the baseline reduces path speed to 57%, with recorded RMS 0.83°. Electrical feasibility is achieved partly through reshaping.
- **Current claim (frozen baseline):** the worst analyzed timing/plant corner (11 ms, J −30%, Kt +15%) has **32.1° phase margin and 6.1 dB gain margin**; see [Task 2 §3](task2.md). *Historical:* the earlier Phase 2 design had only 24° at its worst corner. Passing sampled simulations does not remove the need to measure the real delay and margin on hardware.

## Plots supporting the claims

| Evidence | Plot |
|---|---|
| Reconstruction mismatch and current behavior | [legacy runs](figs/p1_legacy_runs.png) |
| Tracking/current comparison | [baseline runs](figs/p2_runs_compare.png) |
| Loop frequency response and delay | [Bode plot](figs/p2_bode.png) |
| Fault and derating behavior | [fault/derating plot](figs/p2_fault_derate.png) |
| Sampled generalization | [Monte Carlo plot](figs/p2_montecarlo.png) |

## Still required

- **Task 3:** the registered comparison is complete; the exact adaptive candidate failed the benefit and availability gates, so the deterministic baseline remains final. The supplementary payload-change challenge was not executed.
- **Task 5:** a prospective closed-loop prediction, committed before the runs, is supported in simulation ([task5.md](task5.md)); hardware confirmation is proposed.
- **Reproduction and review:** clean-snapshot reproduction is reported in [R4](packets/R4.md). Independent review rounds in [R6](packets/R6.md): the findings of rounds 1–3 are resolved; the final round's verdict is recorded there.
- **Reporting rule:** report original-request deviation alongside governed-reference error wherever motion changes.
