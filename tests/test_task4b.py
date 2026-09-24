"""Packet 4B evaluation helpers: the scenarios, emulations and rules behind
report/task4b_numbers.md do what the report says they do.

    python -m unittest tests.test_task4b -v
"""
import json
import math
import os
import sys
import unittest
from dataclasses import replace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from exp import manifest as MF  # noqa: E402
from exp import scenarios as S  # noqa: E402
from exp import task4b_eval as T  # noqa: E402
from exp.evidence import RunBook  # noqa: E402
from sim.config import SimConfig  # noqa: E402
from sim.engine import Log  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestScenarios(unittest.TestCase):
    def test_a_finite_keeps_task2_request_and_config(self):
        base = SimConfig()
        a, af = S.run_a(base), T.a_finite(base)
        self.assertEqual(af.cfg, a.cfg)
        self.assertEqual(af.roll.eval(1.23), a.roll.eval(1.23))
        self.assertEqual(len(af.waypoints), 5)
        self.assertAlmostEqual(af.waypoints[-1], 0.0)
        self.assertAlmostEqual(af.t_request, 4.2)

    def test_drop_windows_hit_exactly_the_stated_share_in_one_direction(self):
        rng = np.random.default_rng(0)
        for period, offset in ((1e-3, 0.0), (2e-3, 0.5e-3)):
            w = T.drop_windows(8.0, 0.1, period, offset, rng)
            sends = np.arange(int(round(8.0 / period))) * period + offset
            lost = sum(any(a <= s < b for a, b in w) for s in sends[::1])
            self.assertEqual(lost, int(round(0.1 * len(sends))))
        sc = T.stress_cases(SimConfig())
        fb = [c for c in sc if c[0].startswith("Feedback-only loss 10%")][0][1]
        self.assertEqual(fb.cfg.timing.command_blackout, ())
        self.assertTrue(len(fb.cfg.timing.feedback_blackout) > 0)

    def test_monte_carlo_regeneration_matches_task2_draws(self):
        mc = T.mc_de()
        self.assertEqual(len(mc), 40)
        with open(os.path.join(ROOT, "report", "task2_runs.json")) as f:
            runs = json.load(f)["runs"]
        pub = {(v["run"]["scenario"], v["run"]["seed"]): v["run"]["config"] for v in runs.values()
               if v["run"]["scenario"] in ("D", "E") and v["run"]["seed"] >= 100}
        for name, k, sc in mc:
            self.assertEqual(MF.jsonable(sc.cfg), pub[(name, 100 + k)], (name, k))

    def test_freq_amplitude_stays_inside_budget(self):
        from sim import params as P
        for f in T.FREQS:
            a = math.radians(T.freq_amp(f))
            self.assertLessEqual(P.J_R * a * (2 * math.pi * f) ** 2, 0.2 + 1e-12)


class TestRules(unittest.TestCase):
    def test_classify_picks_largest_unmodelled_term_and_limit_flag(self):
        p = dict(terms=dict(payload=0.3, gravity_model=0.01, coupling_total=0.9, coupling_error=0.05,
                            inertia_error=0.02, kt_error=0.04), at_limit_pct=60.0)
        self.assertEqual(T.classify(p), ("unmodelled payload torque, limit-bound", "payload"))
        p["terms"]["coupling_error"] = 0.5
        p["at_limit_pct"] = 10.0
        self.assertEqual(T.classify(p)[1], "coupling_error")   # total coupling is modelled, never the cause

    def test_calculated_tracking_is_unity_at_low_frequency(self):
        g, ph = T.calc_tracking(0.01, 4e-3, 2e-3)
        self.assertAlmostEqual(g[0], 1.0, places=3)
        self.assertAlmostEqual(ph[0], 0.0, places=1)

    def test_fallback_excursion_ignores_startup_and_measures_drift(self):
        n = 1000
        t = np.arange(n) / 1000.0
        q = np.zeros(n)
        mode = np.zeros(n)
        mode[:5] = 1                                  # start-up fallback: ignored
        mode[500:600] = 1
        q[500:600] = np.linspace(0, math.radians(7), 100)
        log = Log(t=t, q=q, mode=mode)
        self.assertAlmostEqual(T.fallback_excursion(log), 7.0, places=6)


class TestMismatchRunner(unittest.TestCase):
    def test_mismatch_is_part_of_run_id_and_changes_the_plant_yaw(self):
        c = S.run_c(SimConfig())
        c = replace(c, cfg=replace(c.cfg, duration=1.5))
        from unittest import mock
        book = RunBook(T.ENTRY)
        spec = T.Spec("stress", "x", c, "baseline", 1, mismatch=(0.010, 1.1))
        # Other tests in this process import modules outside 4B's declared set;
        # declaration is tested in test_metrics, not here.
        with mock.patch.object(MF, "loaded_sources", lambda root=MF.ROOT: []):
            log_m, _, rid_m, _ = T._one(book, spec)
            log_p, _, rid_p, _ = T._one(book, spec._replace(mismatch=None))
        self.assertNotEqual(rid_m, rid_p)
        self.assertEqual(book.runs[rid_m]["run"]["extra"]["yaw_mismatch"]["lag"], 0.010)
        k = 1200
        self.assertAlmostEqual(log_m.qy[k], 1.1 * log_m.qy_plan[k - 10], places=3)


if __name__ == "__main__":
    unittest.main()
