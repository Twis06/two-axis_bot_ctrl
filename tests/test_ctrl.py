"""Unit tests for the baseline's safety-relevant parts.

    python -m unittest discover -s tests -v
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ctrl import loopshape as LS  # noqa: E402
from ctrl.baseline import BaselineController  # noqa: E402
from ctrl.governor import RollGovernor, admit_yaw_sine, yaw_envelope  # noqa: E402
from ctrl.interfaces import Command  # noqa: E402
from ctrl.supervisor import DriveSupervisor  # noqa: E402
from exp.common import run  # noqa: E402
from exp import scenarios as S  # noqa: E402
from sim import params as P  # noqa: E402
from sim.config import SimConfig  # noqa: E402
from sim.trajectories import GovernedYaw, Hold, RampedSine  # noqa: E402

R = math.radians


class TestGovernor(unittest.TestCase):
    def _drive(self, ref, i_lim, T=6.0, extra_fn=None):
        g = RollGovernor()
        g.reset(0.0)
        qs, ss = [], []
        for k in range(int(T / 0.002)):
            q, v, a = g.step(0.002, ref.eval, i_lim, extra_fn=extra_fn)
            qs.append(q)
            ss.append(g.s)
        return np.array(qs), np.array(ss), g

    def test_never_leaves_requested_range(self):
        ref = RampedSine(R(80), 1.5, t_ramp=0.5)          # too fast for the torque available
        qs, ss, _ = self._drive(ref, 2.4)
        self.assertLess(np.max(np.abs(qs)), R(80.5))
        self.assertLess(ss.min(), 0.9)                    # it did have to slow down

    def test_feasible_request_is_untouched(self):
        ref = RampedSine(R(20), 0.2, t_ramp=0.5)
        qs, ss, _ = self._drive(ref, 3.2)
        self.assertGreater(ss.min(), 0.999)
        ts = np.arange(len(qs)) * 0.002
        want = np.array([ref.eval(t)[0] for t in ts])
        self.assertLess(np.max(np.abs(qs - want)), R(0.2))

    def test_rejects_unholdable_positions(self):
        # Constant extra load 0.1 N m: at 2.4 A (0.219 N m available) q > ~41 deg
        # cannot be held (0.12 sin q + 0.1 + 0.04 > 0.219)
        extra = lambda q: 0.1
        tau_av = RollGovernor().available(2.4)
        ref = Hold(R(60))
        qs, _, g = self._drive(ref, 2.4, T=2.0, extra_fn=extra)
        hold = abs(P.TAU_G * math.sin(qs[-1]) + extra(qs[-1])) + P.TAU_C
        self.assertLessEqual(hold, tau_av + 1e-3)
        self.assertTrue(g.rejected)

    def test_yaw_admission_matches_phase0_envelope(self):
        self.assertEqual(admit_yaw_sine(R(75), 2.2, 3.2)[0], "accept")
        dec, A = admit_yaw_sine(R(75), 2.2, 2.4)
        self.assertEqual(dec, "reshape")
        self.assertAlmostEqual(A * 180 / math.pi, 54, delta=1.0)   # Phase 0: +/-54 deg


class TestGovernedYaw(unittest.TestCase):
    def test_scale_blend_is_smooth_and_only_shrinks(self):
        y = GovernedYaw(RampedSine(R(75), 2.2, t_ramp=0.5))
        y.request_scale(2.0, 0.5)
        y.request_scale(3.0, 0.9)                          # increase ignored
        self.assertAlmostEqual(y.scale(4.0), 0.5)
        ts = np.arange(1.9, 2.7, 1e-4)
        qdd = np.array([y.eval(t)[2] for t in ts])
        self.assertLess(np.max(np.abs(np.diff(qdd))), 1.0)  # no acceleration steps


class TestSupervisor(unittest.TestCase):
    def test_watchdog_latches_and_rearms_only_when_realigned(self):
        s = DriveSupervisor(thermal=False)
        s.reset()
        t, dt = 0.0, 1e-3
        mode = None
        for _ in range(100):                          # 15 deg error for 100 ms
            _, mode = s.decide(t, R(15), Command(t, 1.0, 0.0), 0.002, dt)
            t += dt
        self.assertEqual(mode, 1)
        self.assertEqual(s.fault, "watchdog_trip")
        for _ in range(100):                          # still misaligned: stays latched
            _, mode = s.decide(t, R(15), Command(t, 1.0, R(10)), 0.002, dt)
            t += dt
        self.assertEqual(mode, 1)
        for _ in range(100):                          # host re-aligned q_ref
            _, mode = s.decide(t, R(15), Command(t, 1.0, R(15)), 0.002, dt)
            t += dt
        self.assertEqual(mode, 0)

    def test_thermal_derating_reduces_limit(self):
        s = DriveSupervisor()
        s.reset()
        for _ in range(int(120 / 1e-3 / 50)):         # 120 s at 3.2 A, coarse steps
            s.thermal_step(3.2, 50e-3)
        self.assertLess(s.current_limit(), 3.2)
        self.assertGreaterEqual(s.current_limit(), 2.4)


class TestBaselineLoop(unittest.TestCase):
    def test_design_margins(self):
        c = BaselineController()
        pm, gm, _ = LS.margins(c.K, c.wc, 7e-3, c.alpha)
        self.assertGreaterEqual(pm, 44.9)
        self.assertGreaterEqual(gm, 6.0)
        pm_b, _, _ = LS.margins(c.K, c.wc, 11e-3, c.alpha)
        self.assertGreaterEqual(pm_b, 30.0)

    def test_integrator_bounded_under_derating(self):
        # Anti-windup against the drive-reported limit: the integrator must stay
        # bounded when the drive derates and the demand becomes infeasible.
        sc = S.run_e(SimConfig())
        lg, s = run(sc, BaselineController, seed=3)
        self.assertLess(np.nanmax(np.abs(lg.c_integ)), 0.5)
        self.assertEqual(s["events"], 0)

    def test_cmd_timeout_then_recovery(self):
        sc = S.run_b(SimConfig())
        cfg = sc.cfg.with_(timing=dict(blackout=((3.0, 3.1),)))
        lg, s = run(sc, BaselineController, cfg=cfg)
        fb = (lg.mode > 0) & (lg.t > 0.1)
        self.assertTrue(fb.any())
        self.assertLess(lg.t[fb][-1], 3.1 + 0.2)          # back in control quickly
        late = lg.t > 4.0
        self.assertLess(np.max(np.abs(lg.err[late])) * 180 / math.pi, 1.5)


if __name__ == "__main__":
    unittest.main()
