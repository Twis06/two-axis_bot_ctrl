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


class Task5DiscriminatingTests(unittest.TestCase):
    """R2 review I4: each of the reviewer's surviving mutants must fail one of these."""

    def _log(self):
        log = _window_log()
        early = log.t < 2.0
        log.clipped[:] = early.astype(float)                  # clipping only during startup
        log["c_gov_s"] = np.where(early, 0.0, 1.0)            # clock frozen only during startup
        return log

    def test_clip_speed_and_peak_are_windowed(self):
        from exp.task5_prediction import _row
        from sim.metrics import summarize
        log = self._log()
        row = _row(log, summarize(log), seed=0, motor_ratio=1.0, use_yaw_ff=True)
        self.assertEqual(row["clip_pct"], 0.0)
        self.assertEqual(row["path_speed"], 1.0)
        self.assertAlmostEqual(row["governed_peak_deg"], 0.1, places=6)

    def test_true_kt_scaling_in_ideal_current_and_residual(self):
        from exp.task5_prediction import _row
        from sim import params as P
        from sim.metrics import summarize
        log = self._log()
        log["c_tau_cpl"] = log.tau_couple.copy()               # FF exactly equal to the true coupling
        row = _row(log, summarize(log), seed=0, motor_ratio=0.9, use_yaw_ff=True)
        self.assertAlmostEqual(row["ideal_true_coupling_peak_current_A"], 0.12 / (0.9 * P.K_T))
        self.assertAlmostEqual(row["controller_coupling_ff_peak_current_A"], 0.12 / P.K_T)
        self.assertAlmostEqual(row["coupling_residual_peak_Nm"], 0.1 * 0.12, places=9)
        for ms in ("0", "2", "4", "6"):
            self.assertAlmostEqual(row["coupling_residual_peak_by_shift_ms"][ms], 0.1 * 0.12, places=9)

    def _result(self, residual_increase):
        from exp.task5_prediction import prediction_registration
        reg = prediction_registration()
        rows = []
        for seed in (1, 2, 3):
            for ratio in (1.0, 0.9):
                for ff in (True, False):
                    for mode in (("estimate", "plan") if ff else ("estimate",)):
                        res = 0.03 + (residual_increase if ratio == 0.9 else 0.0)
                        rows.append(dict(seed=seed, motor_strength_ratio=ratio, use_yaw_ff=ff, yaw_info=mode,
                                         true_coupling_peak_torque_Nm=0.1356, ideal_true_coupling_peak_current_A=0.97 / ratio,
                                         controller_coupling_ff_peak_current_A=0.97, coupling_estimate_ls_gain=1.0,
                                         coupling_estimate_peak_ratio=1.0, coupling_residual_peak_Nm=res,
                                         coupling_residual_rms_Nm=res / 3,
                                         coupling_residual_peak_by_shift_ms={m: res for m in ("0", "2", "4", "6")},
                                         governed_rms_deg=(0.3 if ff else 3.5) + (0.05 if ratio == 0.9 else 0.0)))
        return dict(registration=reg, rows=rows)

    def test_residual_verdict_follows_the_registered_tolerance(self):
        from exp.task5_prediction import prediction_matrix
        verdict = lambda inc: next(m["verdict"] for m in prediction_matrix(self._result(inc))
                                   if m["prediction"].startswith("coupling residual"))
        self.assertEqual(verdict(0.0136), "supported")
        self.assertEqual(verdict(0.0), "not resolved by this post hoc peak metric")
        self.assertEqual(verdict(0.03), "not resolved by this post hoc peak metric")

    def test_post_hoc_rows_are_labelled(self):
        from exp.task5_prediction import prediction_matrix
        m = {r["prediction"]: r for r in prediction_matrix(self._result(0.0))}
        self.assertTrue(all(r["basis"] in ("registered", "post hoc", "post hoc operationalization",
                                           "post hoc criterion") for r in m.values()))
        self.assertEqual(sum(r["basis"] == "registered" for r in m.values()), 3)


if __name__ == "__main__":
    unittest.main()
