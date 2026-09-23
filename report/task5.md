# Task 5 — Test the explanation

**Status:** Not yet completed. Existing fault and uncertainty experiments are prior evidence; they must not be relabeled as predictions written before a new test.

## Proposed test

Test the claim that yaw feed-forward performance depends on coupling-model accuracy. Use the Run B yaw trajectory and perturb **motor strength**, one of the changes permitted by the assessment. Reduce the true torque constant by 15% while keeping the controller's assumed torque constant fixed. Keep the matching SI back-EMF constant consistent with the physical motor change and document this choice.

Use identical requests, disturbance realizations, transport seeds, limits, and initial states for the nominal and weakened motor. Compare the deterministic baseline with the same controller with yaw feed-forward disabled. This ablation distinguishes feed-forward benefit from unrelated controller changes.

## Prediction to write before execution

The physical coupling torque is unchanged. Ideal compensating current scales as `1 / Kt`, so a 15% weaker motor requires `1 / 0.85 = 1.176` times the current for the same torque. Run B's coupling-only peak current therefore increases from approximately **0.97 A to 1.14 A**. An unchanged nominal feed-forward current produces only 85% of the intended cancellation torque, leaving a coupling residual of approximately **0.020 N·m peak** before feedback correction.

These are calculated component-level predictions, not predicted total measured peak current or tracking RMS. Feedback action, friction, other disturbances, and current dynamics affect those totals.

Before running, use the implemented baseline's sensitivity at 1.5 Hz to turn that residual into a numerical error prediction, define the measurement window and acceptable prediction tolerance, and record the exact configuration and seeds. This step is still pending. If code inspection shows that the proposed perturbation was already evaluated in precisely this form, choose a new held-out condition before registering the prediction.

## Required record

| Item | State |
|---|---|
| Numerical physical prediction and rationale | Component-level values above; closed-loop prediction pending |
| Frozen configuration, measurement window, seeds, and tolerance | Pending |
| New experiment | Not run |
| Predicted versus observed comparison | Pending |
| Revision of explanation, if needed | Pending |
| Smallest justified design change | Decide from result; do not assume adaptation is necessary |

A possible small change is correcting the motor torque conversion if the perturbation behaves as predicted. If delay, saturation, or voltage instead dominates the result, the design change must address that evidence. Do not change gains and compensation simultaneously and then attribute the improvement to one cause.

## Hardware confirmation

A calibrated torque-versus-current measurement is needed to establish the motor's effective torque constant in the actual drive/current convention. A timestamped roll-held yaw experiment then checks whether the predicted compensation torque and residual tracking behavior transfer to hardware. Qualification limits and stop conditions must be specified before that test.
