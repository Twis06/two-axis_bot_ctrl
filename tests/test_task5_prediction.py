"""Contract tests for the registered Task 5 prediction."""

import unittest


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


if __name__ == "__main__":
    unittest.main()
