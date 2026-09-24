"""Packet 2B regressions: fault classes, recovery, information access
(EXECUTION_PLAN.md §5, Packet 2B). Written before the corrections.

Recovery must not bypass the condition that caused a fault: a tracking fault is
not cleared by moving the reference to the measured position. Communication
loss cannot be concealed by fresh outgoing commands. The claimed baseline uses
only information the host has at that instant.
"""
import math
import unittest
from dataclasses import replace

import numpy as np

from ctrl.baseline import BaselineController
from ctrl.governor import RollGovernor, YawMonitor
from ctrl.interfaces import Command, Context, Feedback
from ctrl.supervisor import DriveSupervisor
from exp import scenarios as S
from exp.common import run
from sim.config import SimConfig
from sim.drive import Drive, DriveSafety
from sim.trajectories import Hold, MinJerkSequence

R = math.radians
TS = 0.002
LSB = 2 * math.pi / 2 ** 14


def trip_watchdog(s, t0=0.0, q=0.3):
    """Hold the axis 0.3 rad (17 deg) from a fresh reference until the watchdog latches."""
    t = t0
    while s.fault is None:
        s.decide(t, q, Command(t, 0.0, 0.0), 0.0, 0.001)
        t += 0.001
    return t


def dwell(s, t0, T, q=lambda t: 0.3, ack=0, q_ref=None):
    """Aligned, fresh commands for T seconds; returns the last drive mode."""
    mode, t = None, t0
    while t < t0 + T:
        qe = q(t)
        _, mode = s.decide(t, qe, Command(t, 0.0, qe if q_ref is None else q_ref, ack=ack),
                           0.0, 0.001)
        t += 0.001
    return mode, t


class TestSupervisorFaultClasses(unittest.TestCase):
    def setUp(self):
        self.s = DriveSupervisor(thermal=False)
        self.s.reset()

    def test_tracking_fault_is_not_cleared_by_alignment_alone(self):
        t = trip_watchdog(self.s)
        self.assertEqual(self.s.fault, "watchdog_trip")
        self.assertEqual(self.s.fault_class, "tracking")
        mode, t = dwell(self.s, t, 0.5)              # aligned, at rest, no acknowledgement
        self.assertEqual(mode, 1)
        mode, t = dwell(self.s, t, 0.1, ack=self.s.fault_id)
        self.assertEqual(mode, 0)

    def test_moving_tracking_fault_rearms_only_on_acknowledgement(self):
        # First draft required rest at the drive; under continuing yaw coupling the
        # damped axis never came to rest (Packet 2B report). The feasibility decision
        # is the host's acknowledgement, bounded by the lockout.
        t = trip_watchdog(self.s)
        t0 = t
        mode, t = dwell(self.s, t, 0.5, q=lambda t: 0.3 + 1.0 * (t - t0))
        self.assertEqual(mode, 1)                    # aligned and moving, no acknowledgement
        mode, t = dwell(self.s, t, 0.1, q=lambda t: 0.3 + 1.0 * (t - t0), ack=self.s.fault_id)
        self.assertEqual(mode, 0)

    def test_stale_acknowledgement_does_not_rearm_a_new_fault(self):
        t = trip_watchdog(self.s)
        first = self.s.fault_id
        _, t = dwell(self.s, t, 0.1, ack=first)
        self.assertIsNone(self.s.fault)
        t = trip_watchdog(self.s, t)
        mode, t = dwell(self.s, t, 0.5, ack=first)
        self.assertEqual(mode, 1)

    def test_repeated_tracking_faults_lock_out(self):
        t = 0.0
        for _ in range(3):
            t = trip_watchdog(self.s, t)
            _, t = dwell(self.s, t, 0.1, ack=self.s.fault_id)
        self.assertTrue(self.s.locked)
        self.assertIn("lockout", [e[1] for e in self.s.events])
        mode, t = dwell(self.s, t, 0.5, ack=self.s.fault_id)
        self.assertEqual(mode, 1)

    def test_communication_fault_rearms_without_acknowledgement(self):
        self.s.decide(0, 0, Command(0, 0), 0, .001)
        self.s.decide(.02, 0, Command(0, 0), .02, .001)
        self.assertEqual(self.s.fault_class, "comm")
        mode, _ = dwell(self.s, .021, 0.1, q=lambda t: 0.0)
        self.assertEqual(mode, 0)

    def test_invalid_command_cause_is_reported(self):
        self.s.decide(0, 0, Command(0, 0), 0, .001)
        _, mode = self.s.decide(.001, 0, Command(float("nan"), 0.0), float("nan"), .001)
        self.assertEqual((mode, self.s.fault), (1, "invalid_command"))


def fb(k, q, mode=0, fault="", fault_id=0, i_limit=3.2, locked=False):
    return Feedback(k, k * TS, q, 0.0, 0.0, i_limit, mode, fault=fault, fault_id=fault_id,
                    locked=locked)


class TestHostRecoveryPolicy(unittest.TestCase):
    def setUp(self):
        self.c = BaselineController()
        self.c.reset(SimConfig())
        self.ref = Hold(R(30))
        self.k = 0

    def step(self, **kw):
        t = self.k * TS
        cmd = self.c.update(Context(t, self.ref.eval(t), (0, 0, 0), self.ref.eval,
                                    Hold(0).eval), fb(self.k, **kw))
        self.k += 1
        return cmd

    def test_request_is_not_resumed_after_a_tracking_fault(self):
        for _ in range(50):
            self.step(q=0.0)
        for _ in range(100):
            cmd = self.step(q=R(10), mode=1, fault="watchdog_trip", fault_id=1)
        self.assertTrue(cmd.valid)
        self.assertEqual(cmd.ack, 1)                 # holdable here: acknowledged
        refs = [self.step(q=R(10)).q_ref for _ in range(500)]
        self.assertLess(np.max(np.abs(np.array(refs) - R(10))), 1e-9)
        self.assertTrue(self.c.suspended)
        self.assertEqual(self.c.telemetry["gov_status"], RollGovernor.STATUS.index("restricted"))
        self.c.replan()
        refs = [self.step(q=R(10)).q_ref for _ in range(300)]
        self.assertGreater(refs[-1], R(11))          # explicit replan resumes the request

    def test_unholdable_position_is_not_acknowledged(self):
        for _ in range(20):
            cmd = self.step(q=R(85), mode=1, fault="watchdog_trip", fault_id=1, i_limit=1.0)
        self.assertEqual(cmd.ack, 0)

    def test_catch_is_acknowledged_only_if_the_stop_fits(self):
        # At 1.0 A the holdable interval is about +/-10 deg; at 5 rad/s the budgeted
        # stop needs ~2 rad: no acknowledgement.
        for k in range(60):
            cmd = self.step(q=-0.15 + 5.0 * k * TS, mode=1, fault="watchdog_trip", fault_id=1,
                            i_limit=1.0)
        self.assertEqual(cmd.ack, 0)
        # Slow enough to brake within the budget: acknowledged, and the reference
        # starts at the measured velocity, then brakes to a hold.
        self.setUp()
        for k in range(60):
            cmd = self.step(q=R(10) + 0.5 * k * TS, mode=1, fault="watchdog_trip", fault_id=1)
        self.assertEqual(cmd.ack, 1)
        q0 = R(10) + 0.5 * 59 * TS
        refs = [self.step(q=q0 + 0.5 * TS * k).q_ref for k in range(1, 200)]
        dq = np.diff(refs) / TS
        self.assertGreater(dq[0], 0.1)                 # starts moving with the axis
        self.assertLess(abs(dq[-1]), 1e-9)             # and comes to rest
        self.assertLess(max(refs) - q0, R(3))

    def test_communication_fault_is_acknowledged_and_not_suspended(self):
        for _ in range(20):
            cmd = self.step(q=0.0, mode=1, fault="cmd_timeout", fault_id=1)
        self.assertEqual(cmd.ack, 1)
        self.assertFalse(self.c.suspended)

    def test_drive_lockout_is_reported_as_rejection(self):
        for _ in range(5):
            self.step(q=0.0, mode=1, fault="watchdog_trip", fault_id=3, locked=True)
        self.assertTrue(self.c.request_blocked)
        self.assertIn("lockout", self.c.reject_reason)


class TestLoadedOutage(unittest.TestCase):
    """2A declared regression: payload E + 100 ms feedback loss cycled through
    watchdog trip and automatic re-arm (median 33 trips over seeds 1-10)."""

    def test_tracking_fault_does_not_cycle_through_rearm(self):
        e = S.run_e(SimConfig())
        cfg = replace(e.cfg, duration=7.0).with_(timing=dict(feedback_blackout=((3, 3.1),)))
        for seed in (1, 2):
            with self.subTest(seed=seed):
                c = BaselineController()
                log, _ = run(replace(e, cfg=cfg), lambda: c, seed=seed)
                trips = [ev for ev in log.events if ev[1] == "watchdog_trip"]
                self.assertLessEqual(len(trips), 1)
                if trips:
                    self.assertTrue(c.suspended)
                    after = log.t > trips[0][0] + 1.5
                    self.assertLess(np.max(np.abs(np.diff(log.c_q_c[after]))), 1e-9)
                    # Caught by a braking reference, not left to fall to the passive
                    # rest (first draft, re-arm only at rest: 51-96 deg of travel).
                    k = log.t >= trips[0][0]
                    self.assertLess(np.ptp(log.q[k & (log.t < trips[0][0] + 0.3)]), R(20))

    def test_continuing_coupling_does_not_keep_the_axis_in_passive_fallback(self):
        # No yaw planner, 0.7 kg +35 mm, C yaw: with re-arm only at rest the damped
        # axis kept moving under the coupling and stayed in fallback (97.6 %).
        c_ = S.run_c(SimConfig())
        cfg = replace(c_.cfg, duration=6.0).with_(plant=dict(m_payload=0.7, s_lat=0.035),
                                                    timing=dict(feedback_blackout=((3, 3.1),)))
        c = BaselineController()
        log, _ = run(replace(c_, cfg=cfg), lambda: c, seed=1, governed_yaw=False)
        self.assertLess(np.mean(log.mode[log.t > 3.0] > 0), 0.10)
        # Under active control but not tracking: the request is suspended (coordinated
        # stop), reported, and the reference is stationary where the catch ended.
        self.assertTrue(c.suspended and c.coord_stop)
        late = log.t > 4.0
        self.assertLess(np.ptp(log.c_q_c[late]), 1e-9)
        self.assertTrue(np.all(log.c_request_rejected[late] + log.c_suspended[late] > 0.5))


class TestDirectionalOutages(unittest.TestCase):
    def _run(self, key):
        sc = S.run_c(SimConfig())
        cfg = replace(sc.cfg, duration=4.0)
        if key:
            cfg = cfg.with_(timing={key: ((3.0, 3.06),)})
        c = BaselineController()
        log, s = run(replace(sc, cfg=cfg), lambda: c, seed=7)
        return log, s, c

    def test_command_only_and_bidirectional_loss_recover_automatically(self):
        ref, _, _ = self._run(None)
        for key in ("command_blackout", "blackout", "feedback_blackout"):
            with self.subTest(outage=key):
                log, s, c = self._run(key)
                self.assertTrue(np.any(log.mode[(log.t > 3.0) & (log.t < 3.1)] == 1))
                self.assertTrue(np.all(log.mode[log.t > 3.4] == 0))
                self.assertFalse(c.suspended)
                # 2A-era finding: brief outages shrank yaw to 0.8x. No yaw request
                # may be attributable to the outage (matched run without it).
                self.assertEqual(log.meta["yaw_scale"], ref.meta["yaw_scale"])


class TestYawMonitorWindow(unittest.TestCase):
    def test_incomplete_window_does_not_decide(self):
        m = YawMonitor()
        m.reset(dt=TS)
        out = [m.update(t, True) for t in np.arange(0, 0.2, TS)]
        out += [m.update(t, True) for t in np.arange(0.45, 0.7, TS)]   # 0.25 s gap
        self.assertTrue(all(o is None for o in out))

    def test_host_counts_every_valid_sample_and_clears_on_idle(self):
        c = BaselineController(yaw_info="plan")
        c.reset(SimConfig())
        ref = Hold(0.0)
        for k in range(200):
            # Roll error saturates the command; yaw coupling is small (not the cause).
            c.update(Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, lambda t: (0, 0, 10.0)),
                     Feedback(k, k * TS, R(40) if k % 2 else 0.0, 0.0, 0.0, 3.2, 0))
        self.assertEqual(len(c.mon.hist), 200)
        c.update(Context(0.5, ref.eval(0), (0, 0, 0), ref.eval, Hold(0).eval),
                 Feedback(0, 0.0, 0.0, 0.0, 0.0, 3.2, 0))                # stale
        self.assertEqual(len(c.mon.hist), 0)


class TestInformationMode(unittest.TestCase):
    def test_default_mode_never_reads_the_yaw_plan(self):
        def forbidden(t):
            raise AssertionError("estimate mode read the yaw plan")
        c = BaselineController()
        c.reset(SimConfig())
        self.assertEqual(c.yaw_info, "estimate")
        ref = Hold(0.0)
        for k in range(50):
            cmd = c.update(Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, forbidden),
                           Feedback(k, k * TS, 0.0, 0.01 * k, 0.0, 3.2, 0))
        self.assertTrue(cmd.valid)

    def test_estimated_coupling_follows_true_coupling(self):
        sc = S.run_c(SimConfig())
        log, _ = run(replace(sc, cfg=replace(sc.cfg, duration=5.0)), BaselineController, seed=3)
        k = int(round(BaselineController().T_act * 1000))
        est, true = log.c_tau_cpl[:-k], log.tau_couple[k:]
        m = log.t[:-k] > 2.0
        rms = lambda x: float(np.sqrt(np.mean(x ** 2)))
        self.assertLess(rms(est[m] - true[m]), 0.25 * rms(true[m]))

    def test_unknown_information_mode_is_refused(self):
        with self.assertRaises(ValueError):
            BaselineController(yaw_info="oracle")


class TestNonFinite(unittest.TestCase):
    def test_nonfinite_yaw_plan_never_becomes_a_limit_command(self):
        c = BaselineController(yaw_info="plan")
        c.reset(SimConfig())
        ref = Hold(0.0)
        nan = float("nan")
        for k in range(20):
            cmd = c.update(Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, lambda t: (0, 0, nan)),
                           Feedback(k, k * TS, 0.0, 0.0, 0.0, 3.2, 0))
            self.assertTrue(math.isfinite(cmd.i_cmd))
            self.assertLess(abs(cmd.i_cmd), 1.0)
        self.assertTrue(any("yaw" in n[2] for n in c.notices))

    def test_nonfinite_load_model_never_becomes_a_limit_command(self):
        class NaNLoad:
            def reset(self): pass
            def update(self, *a, **k): pass
            def torque(self, q): return float("nan")
        c = BaselineController(load_model=NaNLoad())
        c.reset(SimConfig())
        ref = Hold(0.0)
        for k in range(20):
            cmd = c.update(Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, Hold(0).eval),
                           Feedback(k, k * TS, 0.0, 0.0, 0.0, 3.2, 0))
            self.assertTrue(math.isfinite(cmd.i_cmd))
            self.assertTrue(not cmd.valid or abs(cmd.i_cmd) < 1.0)

    def test_drive_never_turns_a_nonfinite_target_into_the_limit(self):
        drive = Drive(SimConfig().drive, DriveSafety(), .001, 1000)
        for k in range(3):
            tick = drive.tick(k * .001, 0, (k, k * .001, k * .001, Command(k * .001, float("nan"))))
        self.assertEqual(tick["applied"], 0.0)
        self.assertEqual(tick["mode"], 1)


class TestAntiWindupPriority(unittest.TestCase):
    def test_feedforward_clipping_does_not_wind_the_integrator(self):
        # 30 deg in 0.1 s asks ~1.2 N m of inertial feed-forward (capacity 0.448).
        # Back-calculating against the total clamp drove the integrator to its
        # bound; settling within 1 deg took 1.12 s. The remaining approach time is
        # the ~2 deg Coulomb-friction band removed by the integrator (time constant
        # ~0.36 s), not windup: the first-draft 0.5 s threshold assumed otherwise.
        cfg = SimConfig(duration=3.0)
        sc = S.Scenario("step", cfg, MinJerkSequence(0.0, [(R(30), 0.1, 5.0)]), Hold(0.0), "")
        log, _ = run(sc, lambda: BaselineController(use_governor=False, use_yaw_monitor=False),
                     seed=1, supervisor=DriveSafety(cmd_timeout=0.010))
        outside = np.where(np.abs(log.err) > R(1))[0]
        self.assertLess(log.t[outside[-1]], 0.8)
        self.assertLess(np.nanmax(np.abs(log.c_integ)), 0.1)     # was at its 0.448 bound


# ---------------------------------------------------------------------------
# Independent 2B review (scratchpad/review2B). Reproducers first.
# ---------------------------------------------------------------------------
class TestReview2B(unittest.TestCase):
    def _host(self):
        c = BaselineController()
        c.reset(SimConfig())
        return c

    def test_I1_acknowledgement_is_withdrawn_when_the_catch_becomes_infeasible(self):
        c, ref, q = self._host(), Hold(0.0), 0.0
        for k in range(60):
            v = 0.0 if k < 30 else 5.0                     # at rest, then 5 rad/s at 1.0 A
            q += v * TS
            cmd = c.update(Context(k * TS, ref.eval(0), (0, 0, 0), ref.eval, Hold(0).eval),
                           Feedback(k, k * TS, q, 0.0, 0.0, 1.0, 1, fault="watchdog_trip",
                                    fault_id=1))
            if k == 29:
                self.assertEqual(cmd.ack, 1)
        self.assertEqual(cmd.ack, 0)

    def test_I2_catch_requires_the_hold_to_carry_the_coupling(self):
        g = RollGovernor()
        # Static hold at -49 deg (0.131 N m with friction) + 0.3 N m coupling > capacity.
        for v in (0.0, 1e-10, 0.01, -0.01):
            with self.subTest(v=v):
                self.assertIsNone(g.catch_plan(R(-49), v, 3.2, tau_couple=0.3))
        # Within capacity at the hold: acknowledged for any small velocity.
        for v in (0.0, 1e-10, 0.01, -0.01):
            with self.subTest(v=v):
                self.assertIsNotNone(g.catch_plan(R(-10), v, 3.2, tau_couple=0.25))

    def test_I3_brief_outages_cause_no_yaw_reduction_or_incompatibility(self):
        def go(key, L, seed, planner):
            sc = S.run_c(SimConfig())
            cfg = replace(sc.cfg, duration=4.2)
            if L:
                cfg = cfg.with_(timing={key: ((3.0, 3.0 + L),)})
            c = BaselineController()
            log, _ = run(replace(sc, cfg=cfg), lambda: c, seed=seed, governed_yaw=planner)
            return log.meta["yaw_scale"], bool(c.incompatible)
        for planner in (True, False):
            for seed in (1, 3):
                ref = go(None, 0, seed, planner)
                for key, L in (("feedback_blackout", 0.1), ("command_blackout", 0.02),
                               ("blackout", 0.06)):
                    with self.subTest(planner=planner, seed=seed, outage=key, L=L):
                        self.assertEqual(go(key, L, seed, planner), ref)

    def test_I5_path_speed_reads_zero_while_the_clock_is_frozen(self):
        sc = S.run_e(SimConfig())
        cfg = replace(sc.cfg, duration=3.0).with_(plant=dict(m_payload=1.0, s_lat=0.035))
        log, _ = run(replace(sc, cfg=cfg), BaselineController, seed=1)
        k = np.nan_to_num(log.c_suspended) > 0.5
        self.assertTrue(np.any(k))
        self.assertEqual(float(np.max(log.c_gov_s[k])), 0.0)


if __name__ == "__main__":
    unittest.main()
