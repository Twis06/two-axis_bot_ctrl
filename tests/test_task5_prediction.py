"""Contract tests for the registered Task 5 prediction."""

import math
import unittest

import numpy as np


class Task5PredictionTests(unittest.TestCase):
    def test_registered_motor_prediction_uses_the_held_out_ratio(self):
        from exp.task5_prediction import prediction_registration

        registration = prediction_registration()
        self.assertEqual(registration["motor_strength_ratio"], 0.90)
        self.assertEqual(registration["seeds"], [21, 22, 23])
        self.assertTrue(registration["prior_exposure_check"]["exact_matched_condition_new"])
        self.assertAlmostEqual(registration["prediction"]["coupling_peak_current_A"], 0.9687299522, places=6)
        self.assertAlmostEqual(registration["prediction"]["extra_current_A"], 0.1076366614, places=6)
        self.assertAlmostEqual(registration["prediction"]["residual_torque_Nm"], 0.0135622193, places=6)


def _window_log():
    """Startup (0-2 s) with large error and current; small inside the 2-8 s window."""
    from tests.test_metrics import synth
    t = np.arange(8000) / 1000.0
    early = t < 2.0
    q = np.where(early, math.radians(10.0), math.radians(0.1))
    log = synth(q=q, T=8.0, tele=dict(q_c=0.0, gov_s=1.0, gov_lag=0.0, gov_rejected=0.0, tau_cpl=np.where(early, 0.5, 0.1)))
    log.err[:] = q
    log.i[:] = np.where(early, 3.0, 0.5)
    log.tau_couple[:] = np.where(early, 0.4, 0.12)
    return log


class Task5WindowAndUnitTests(unittest.TestCase):
    """R2 regressions: one declared window for every windowed quantity; torque is not current."""

    def _row(self):
        from exp.task5_prediction import _row
        from sim.metrics import summarize
        log = _window_log()
        return _row(log, summarize(log), seed=0, motor_ratio=1.0, use_yaw_ff=True)

    def test_tracking_and_current_use_the_same_registered_window(self):
        row = self._row()
        self.assertAlmostEqual(row["measured_peak_current_A"], 0.5)
        self.assertAlmostEqual(row["governed_rms_deg"], 0.1, places=6)      # not the 10 deg startup
        self.assertAlmostEqual(row["original_rms_deg"], 0.1, places=6)
        self.assertEqual(row["window_s"], [2.0, 8.0])

    def test_controller_coupling_is_reported_as_torque_and_converted_explicitly(self):
        from sim import params as P
        row = self._row()
        self.assertNotIn("predicted_coupling_peak_current_A", row)          # old mislabeled field
        self.assertAlmostEqual(row["controller_coupling_ff_peak_torque_Nm"], 0.1)
        self.assertAlmostEqual(row["controller_coupling_ff_peak_current_A"], 0.1 / P.K_T)

    def test_whole_run_safety_evidence_is_kept_separately(self):
        row = self._row()
        self.assertIn("whole_run", row)
        self.assertAlmostEqual(row["whole_run"]["measured_peak_current_A"], 3.0)


if __name__ == "__main__":
    unittest.main()
