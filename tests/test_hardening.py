"""Safety regressions found in the Task 2 review (real controller/drive paths)."""
import math
import unittest
from dataclasses import replace

import numpy as np

from ctrl.baseline import BaselineController
from ctrl import loopshape
from ctrl.governor import RollGovernor
from ctrl.interfaces import Command, Context, Feedback
from ctrl.supervisor import DriveSupervisor
from sim.config import SimConfig
from sim.drive import Drive, DriveSafety
from exp.common import run
from exp import scenarios as S
from sim.trajectories import Hold


class TestFeedbackSafety(unittest.TestCase):
    def setUp(self):
        self.c = BaselineController()
        self.c.reset(SimConfig())
        self.ref = Hold(0.0)

    def context(self, t, yaw_at=None):
        return Context(t, self.ref.eval(t), (0, 0, 0), self.ref.eval,
                       self.ref.eval if yaw_at is None else yaw_at)

    def test_stale_feedback_invalidates_command_and_freezes_path(self):
        self.c.update(self.context(0), Feedback(0, 0, 0, 0, 0, 3.2, 0))
        clock = self.c.gov.sigma
        for t in (.1, .2):
            cmd = self.c.update(self.context(t), Feedback(0, 0, .1, 0, 0, 3.2, 0))
            self.assertFalse(cmd.valid)
            self.assertEqual(self.c.gov.sigma, clock)
        fresh = self.c.update(self.context(.202), Feedback(1, .202, .1, 0, 0, 3.2, 1))
        self.assertTrue(fresh.valid)  # aligned handshake; drive owns re-arm
        self.assertEqual(self.c.gov.sigma, clock)
        self.assertAlmostEqual(fresh.q_ref, .1)

    def test_nonfinite_feedback_cannot_generate_valid_command(self):
        for field in ('q', 't_meas', 'i_limit', 'i_meas'):
            fb = replace(Feedback(0, 0, 0, 0, 0, 3.2, 0), **{field: float('nan')})
            with self.subTest(field=field):
                self.assertFalse(self.c.update(self.context(0), fb).valid)

    def test_missing_yaw_plan_requires_explicit_feedback_only_mode(self):
        ctx = replace(self.context(0), yaw_at=None)
        self.assertFalse(self.c.update(ctx, Feedback(0, 0, 0, 0, 0, 3.2, 0)).valid)
        fb_only = BaselineController(use_yaw_ff=False)
        fb_only.reset(SimConfig())
        self.assertTrue(fb_only.update(ctx, Feedback(0, 0, 0, 0, 0, 3.2, 0)).valid)

    def test_uncoordinated_yaw_saturation_rejects_request(self):
        # Known yaw acceleration demands 0.8 Nm: cannot be rejected with 0.448 Nm.
        ctx = self.context(0, lambda t: (0, 0, 1000.0))
        valid = []
        for k in range(350):
            t = k*.002
            cmd = self.c.update(replace(ctx, t=t), Feedback(k, t, 0, 0, 0, 3.2, 0))
            valid.append(cmd.valid)
        self.assertFalse(valid[-1])
        self.assertTrue(self.c.telemetry.get('request_rejected', False))


class TestLoopCorner(unittest.TestCase):
    def test_combined_delay_and_plant_corner_has_reserved_margins(self):
        c = BaselineController()
        pm, gm, _ = loopshape.margins(c.K*1.15, c.wc, .011, c.alpha, J=.0028)
        self.assertGreaterEqual(pm, 30)
        self.assertGreaterEqual(gm, 6)


class TestLimitSafety(unittest.TestCase):
    def test_derate_reclamps_delayed_target(self):
        cfg = SimConfig().with_(drive=dict(derate_schedule=((.001, 2.4),)))
        drive = Drive(cfg.drive, DriveSafety(), .001, 1000)
        drive.tick(0, 0, (0, 0, 0, Command(0, 3.2)))
        tick = drive.tick(.001, 0, (1, .001, .001, Command(.001, 3.2)))
        self.assertLessEqual(abs(tick['applied']), 2.4)

    def test_empty_feasible_set_is_rejected_even_for_zero_request(self):
        gov = RollGovernor(); gov.reset(0)
        gov.step(.002, Hold(0).eval, 2.4, tau_extra=1.0)
        self.assertTrue(gov.rejected)
        self.assertIsNone(gov.feasible_interval(gov.available(2.4), 1.0))

    def test_governor_step_does_not_command_unbudgeted_acceleration(self):
        gov = RollGovernor(); gov.reset(0)
        q, v, a = gov.step(.002, Hold(math.radians(80)).eval, 2.4)
        # 0.336*0.8 - 0.05 = 0.2188 Nm, including moving friction allowance.
        demand = abs(.004*a + .012*v + .12*math.sin(q) + .04*math.tanh(v/.02))
        self.assertLessEqual(demand, .2188 + 1e-9)

    def test_overspeed_cannot_rearm_until_speed_clears(self):
        supervisor = DriveSupervisor(thermal=False); supervisor.reset()
        # A real encoder ramp keeps the local velocity estimate above 25 rad/s.
        for k in range(200):
            t = k*.001; q = 30*t
            _, mode = supervisor.decide(t, q, Command(t, 0, q), 0, .001)
            if k > 25:
                self.assertEqual(mode, 1)
        self.assertEqual(supervisor.fault, 'overspeed')
        for k in range(200, 500):
            t = k*.001; q = 30*.199
            _, mode = supervisor.decide(t, q, Command(t, 0, q), 0, .001)
        self.assertEqual(mode, 0)


class TestRecoveryAndTransport(unittest.TestCase):
    def test_feedback_only_loss_falls_back_while_commands_stay_fresh(self):
        sc = S.run_b(SimConfig())
        timing = replace(sc.cfg.timing, feedback_blackout=((1.0, 1.1),))
        cfg = replace(sc.cfg, timing=timing, duration=1.5)
        log, _ = run(sc, BaselineController, cfg=cfg)
        during = (log.t > 1.025) & (log.t < 1.1)
        self.assertTrue(np.all(log.mode[during] == 1))
        self.assertLess(np.max(log.cmd_age[during]), .01)
        self.assertTrue(np.all(log.mode[log.t > 1.3] == 0))

    def test_timeout_recovery_requires_continuously_fresh_commands(self):
        s = DriveSupervisor(thermal=False); s.reset()
        s.decide(0, 0, Command(0, 0), 0, .001)
        _, mode = s.decide(.02, 0, Command(0, 0), .02, .001)
        self.assertEqual(mode, 1)
        _, mode = s.decide(.021, 0, Command(.021, 0), 0, .001)
        self.assertEqual(mode, 1)
        for k in range(22, 100):
            _, mode = s.decide(k*.001, 0, Command(k*.001, 0), 0, .001)
        self.assertEqual(mode, 0)

    def test_nonfinite_command_is_rejected_locally(self):
        s = DriveSupervisor(thermal=False); s.reset()
        current, mode = s.decide(0, 0, Command(0, float('nan')), 0, .001)
        self.assertEqual(mode, 1)
        self.assertTrue(math.isfinite(current))


class TestIntegratorBound(unittest.TestCase):
    def test_ungoverned_abrupt_request_cannot_wind_integrator(self):
        # Regression (found in Task 2 robustness work): back-calculation also reacts
        # to feed-forward clipping. 60 deg in 20 ms asks ~60 N m of inertial
        # feed-forward; before the bound the integrator reached ~5 N m (>10x the
        # 0.448 N m the motor can produce).
        from exp.common import run
        from exp import scenarios as S
        from sim.trajectories import MinJerkSequence
        cfg = SimConfig(duration=1.0)
        sc = S.Scenario('step', cfg, MinJerkSequence(0.0, [(math.radians(60), 0.02, 5.0)]),
                        Hold(0.0), 'abrupt')
        log, _ = run(sc, lambda: BaselineController(use_governor=False, use_yaw_monitor=False),
                     seed=1, supervisor=DriveSafety(cmd_timeout=0.01))
        self.assertLessEqual(np.nanmax(np.abs(log.c_integ)), 0.14 * 3.2 + 1e-9)


if __name__ == '__main__':
    unittest.main()
