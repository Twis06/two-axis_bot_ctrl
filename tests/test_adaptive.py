"""Task 3 L3 contract: feed-forward-only integration of the payload estimator
(docs/plans/task3-learning.md §7)."""
import math
import unittest
from dataclasses import replace

import numpy as np

from ctrl.adaptive import AdaptiveController
from ctrl.baseline import BaselineController
from ctrl.interfaces import Context, Feedback
from exp import scenarios as S
from exp.common import run
from sim.config import SimConfig
from sim.trajectories import Hold, MinJerkSequence

R = math.radians
TS = 0.002


def loaded_moves(duration=6.0):
    cfg = replace(SimConfig(), duration=duration).with_(plant=dict(m_payload=0.5))
    roll = MinJerkSequence(0.0, [(R(-50), 1.0, 1.0), (0.0, 1.0, 1.0), (R(50), 1.0, 1.0)])
    return S.Scenario("L3", cfg, roll, Hold(0.0), "")


class TestL3Contract(unittest.TestCase):
    def test_disabled_reproduces_the_frozen_baseline(self):
        sc = loaded_moves(3.0)
        a, _ = run(sc, BaselineController, seed=4)
        b, _ = run(sc, lambda: AdaptiveController(enabled=False), seed=4)
        for k in ("q", "i", "i_tgt", "mode"):
            self.assertTrue(np.array_equal(a[k], b[k]), k)

    def test_unusable_estimator_applies_nothing(self):
        # Before the estimator has enough dwell coverage it must be the baseline.
        sc = loaded_moves(1.5)
        a, _ = run(sc, BaselineController, seed=4)
        b, _ = run(sc, AdaptiveController, seed=4)
        self.assertEqual(float(np.nanmax(b.c_learn_usable)), 0.0)
        self.assertTrue(np.array_equal(a.q, b.q))

    def _forced(self, value):
        """Adaptive controller whose estimator reports a fixed usable model."""
        c = AdaptiveController()
        c.reset(SimConfig())
        c.adapter.torque = lambda q: value
        c.adapter.coefficients = lambda: (0.0, value)
        c.adapter.observe = lambda t, fb, sat: setattr(c.adapter, "snap", type(
            "S", (), dict(usable=True, theta_s=0.0, theta_c=value, coefficients=(0.0, value)))())
        return c

    def test_correction_never_reaches_the_governor(self):
        c = self._forced(0.2)
        seen = []
        step = c.gov.step

        def spy(*a, **k):
            seen.append(a[5] if len(a) > 5 else k.get("extra_fn"))
            return step(*a, **k)
        c.gov.step = spy
        b = BaselineController()
        b.reset(SimConfig())
        ref = Hold(R(30))
        for k in range(100):
            fb = Feedback(k, k * TS, 0.0, 0.0, 0.0, 3.2, 0)
            ctx = Context(k * TS, ref.eval(k * TS), (0, 0, 0), ref.eval, Hold(0).eval)
            c.update(ctx, fb)
            b.update(ctx, fb)
        # The wrapped step receives the model but forwards None to the governor;
        # admission (the governed reference) is exactly the baseline's.
        self.assertTrue(seen)
        self.assertEqual(c.telemetry["q_c"], b.telemetry["q_c"])

    def test_bumpless_transfer_at_a_model_update(self):
        a = self._forced(0.0)
        b = self._forced(0.0)
        ref = Hold(0.0)
        for k in range(60):
            if k == 49:                          # model update: 0.1 N m of learned cos load
                b.adapter.torque = lambda q: 0.1 * math.cos(q)
                b.adapter.coefficients = lambda: (0.0, 0.1)
            fb = Feedback(k, k * TS, R(1), 0.0, 0.0, 3.2, 0)
            ctx = Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, Hold(0).eval)
            ca, cb = a.update(ctx, fb), b.update(ctx, fb)
            if k == 49:
                # The bump is evaluated at the previous tick's reference angle, so the
                # transfer is exact only to ~1e-7 N m here (declared approximation).
                self.assertAlmostEqual(ca.i_cmd, cb.i_cmd, places=5)
        self.assertAlmostEqual(a.integ - b.integ, 0.1, places=5)   # load moved from integrator to FF

    def test_nonfinite_learned_value_is_ignored(self):
        a = BaselineController()
        a.reset(SimConfig())
        b = self._forced(0.0)
        b.adapter.torque = lambda q: float("nan")
        ref = Hold(R(20))
        for k in range(50):
            fb = Feedback(k, k * TS, 0.0, 0.0, 0.0, 3.2, 0)
            ctx = Context(k * TS, ref.eval(k * TS), (0, 0, 0), ref.eval, Hold(0).eval)
            ca, cb = a.update(ctx, fb), b.update(ctx, fb)
            self.assertTrue(math.isfinite(cb.i_cmd))

    @staticmethod
    def _tuning_run(**timing):
        # Candidate B (10-count dwell window) on a TUNING load of the registered protocol.
        from exp import task3_l4 as T
        sc, _ = T.scenario(T.TUNING_LOADS[1], 3.2, "tune")
        if timing:
            sc = replace(sc, cfg=sc.cfg.with_(timing=timing))
        return run(sc, lambda: AdaptiveController(stationary_counts=10), seed=11)[0]

    def test_learning_engages_and_stays_bounded_and_within_limits(self):
        log = self._tuning_run()
        self.assertGreater(float(np.nanmax(log.c_learn_usable)), 0.5)       # it does learn here
        self.assertLessEqual(float(np.max(np.abs(log.i_tgt) - log.i_lim)), 1e-9)
        theta = np.hypot(np.nan_to_num(log.c_learn_theta_s), np.nan_to_num(log.c_learn_theta_c))
        self.assertLessEqual(float(np.max(theta)), 0.40 + 1e-9)

    def test_health_invalidated_during_a_feedback_outage(self):
        log = self._tuning_run(feedback_blackout=((14.0, 14.1),))
        usable = np.nan_to_num(log.c_learn_usable)
        self.assertGreater(usable[(log.t > 13.5) & (log.t < 14.0)].max(), 0.5)
        self.assertEqual(usable[(log.t > 14.03) & (log.t < 14.1)].max(), 0.0)


if __name__ == "__main__":
    unittest.main()
