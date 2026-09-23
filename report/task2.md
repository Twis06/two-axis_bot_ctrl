# Task 2 — Build a baseline

**Status:** A deterministic baseline is implemented and evaluated in the existing simulation reports. This document summarizes that design; hardware qualification and a final submission review remain outstanding.

## Answer

Use model-based feed-forward to address predictable motion and yaw coupling, bounded feedback to correct residual error, a reference governor to enforce feasible motion, and drive-local limits and fault handling. Learning is not required for this baseline.

| Component | Implemented choice | Purpose |
|---|---|---|
| Host controller | 500 Hz PI × lead; nominal designed crossover 5.5 Hz | Correct tracking error within the modeled delay budget |
| Feed-forward | Roll inertia, viscous/friction and nominal gravity compensation; yaw coupling from the planned yaw trajectory | Supply predictable torque before tracking error develops |
| State/filter handling | Position encoder feedback with the lead filter providing filtered derivative action; friction compensation uses reference velocity | Avoid raw finite-difference velocity driving friction chatter |
| Anti-windup | Back-calculation using the current limit reported by the drive | Prevent integral accumulation against the known command limit |
| Reference governor | Path time scaling, braking look-ahead, and static feasibility projection | Slow dynamic demands and reject modeled unholdable positions |
| Drive supervisor | 1 kHz current/thermal limits, stale-command detection, tracking and speed checks | Enforce limits independently of host timing |
| Fallback | Local velocity damping, with reference realignment before re-arm | Provide bounded local action when normal tracking is unavailable |

Implementation: [baseline](../ctrl/baseline.py), [governor](../ctrl/governor.py), [supervisor](../ctrl/supervisor.py), and [loop design](../ctrl/loopshape.py). Exact gains, thresholds, and design rationale are in [the detailed baseline report](phase2_baseline.md).

## Why it should remain well behaved

The recorded analytic loop analysis includes current lag and transport delay. Phase margin is 45° at the 7 ms design delay and 37° at an 11 ms burst case. The combined corner of 11 ms delay, inertia −30%, and motor constant +15% reduces phase margin to 24° and gain margin to 4.2 dB. That corner is a limitation, not a comfortable robustness margin.

The simulator includes encoder quantization, current and voltage limits, asynchronous transport, parameter uncertainty, and fault cases. Existing tests check the pipeline, plant, actuator, and a delay-sensitive stability boundary. Recorded baseline experiments show improved tracking and fewer sustained saturation episodes; see [Task 4](task4.md). These results support simulator behavior under tested assumptions, not a hardware safety guarantee.

## Accept, reshape, derate, or reject

- **Accept** when predicted demand fits the available torque with the chosen reserve.
- **Reshape** when reducing acceleration/speed or yaw excitation restores margin.
- **Derate** when the drive lowers the active limit; recompute the envelope from that limit.
- **Reject or restrict position** when static holding demand cannot fit the limit. Slowing the trajectory does not fix this condition.

For the nominal coupling model with the selected reserve, yaw ±75° at 2.2 Hz is admitted at 3.2 A but reduced to approximately ±54° at 2.4 A. This is a policy result under a reserve requirement, not proof that ideal zero-error holding is physically impossible at 2.4 A.

The yaw adjustment assumes the roll system can request a coordinated change from the yaw planner. If that authority is unavailable, the system must report the incompatible request and use an agreed stop/degraded-motion policy; a roll controller cannot silently change another axis's command.

## Material limitations

1. Unknown payload torque can make the nominal governor optimistic. Saturation back-off reduces speed after mismatch becomes evident, but does not constitute an accurate payload feasibility estimate.
2. Damping-only fallback permits settling toward the payload's passive equilibrium, potentially far from the requested angle. Collision clearance and acceptable fallback motion need hardware qualification.
3. Planned yaw feed-forward assumes a usable yaw plan and sufficiently accurate yaw tracking. Observable yaw kinematics alone do not guarantee that future trajectory information is available.
4. Governed-reference error must be reported with delivered speed, amplitude, and any rejected motion. Small error to a modified request is not completion of the original request.

## Evidence and next work

Recorded baseline median RMS errors are 0.55°, 0.20°, 0.22°, 2.85°, and 2.93° for A–E. D and E run at 82% and 48% of requested path speed. See [full numbers](phase2_numbers.md). The next work is to resolve the learning decision, run the prospective prediction test, and specify hardware acceptance and stop criteria.
