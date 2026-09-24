"""Contract tests for the standalone Task 3 L2 estimator audit."""

import unittest


class Task3AuditTests(unittest.TestCase):
    def test_ideal_and_narrow_cases_report_structural_outcomes(self):
        from exp.task3_estimator_audit import run_case

        ideal = run_case("ideal", seed=201, quick=True)
        self.assertEqual(ideal["case"], "ideal")
        self.assertEqual(ideal["structural_violations"], [])
        self.assertTrue(ideal["usable_seen"])
        self.assertGreaterEqual(ideal["modeled_delay_ms"]["min"], 2.0)
        self.assertLessEqual(ideal["modeled_delay_ms"]["max"], 8.0)

        narrow = run_case("narrow_coverage", seed=201, quick=True)
        self.assertEqual(narrow["structural_violations"], [])
        self.assertFalse(narrow["usable_seen"])
        self.assertIn(narrow["final_reason"], {"awaiting_estimate", "insufficient_coverage"})


if __name__ == "__main__":
    unittest.main()
