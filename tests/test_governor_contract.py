"""Packet 2A regressions: the governor's reference contract (EXECUTION_PLAN.md §5).

The governor may slow traversal of an admitted path, or explicitly restrict or
reject a request. It must not create a different path to use motion as
disturbance cancellation. Emitted (q, v, a) must be mutually consistent, and the
request disposition must be observable with a reason.
"""
import math
import unittest

import numpy as np

from ctrl.baseline import BaselineController
from ctrl.governor import RollGovernor
from ctrl.interfaces import Context, Feedback
from sim.config import SimConfig
from sim.trajectories import Hold, MinJerkSequence, RampedSine

R = math.radians
TS = 0.002


def drive(gov, ref, i_lim, n, **kw):
    """Step a governor n times; return arrays of q, v, a and the status per step."""
    out = []
    for _ in range(n):
        q, v, a = gov.step(TS, ref.eval, i_lim, **kw)
        out.append((q, v, a, gov.status))
    q, v, a = (np.array([o[i] for o in out]) for i in range(3))
    return q, v, a, [o[3] for o in out]


class TestHoldDoesNotDrift(unittest.TestCase):
    """Reviewer reproducer: excessive coupling must not move a requested hold."""

    def _hold(self, tau_couple):
        g = RollGovernor()
        g.reset(0.0)
        return g, drive(g, Hold(0.0), 3.2, 1000, tau_couple=tau_couple)

    def test_hold_stays_stationary_when_coupling_exceeds_budget(self):
        for c in (0.4, 0.8):
            with self.subTest(tau_couple=c):
                g, (q, v, a, st) = self._hold(c)
                self.assertLess(np.max(np.abs(q)), 1e-12)
                self.assertLess(np.max(np.abs(v)), 1e-12)
                self.assertEqual(st[-1], "over_budget")
                self.assertTrue(g.reason)

    def test_hold_within_budget_is_accepted(self):
        g, (q, v, a, st) = self._hold(0.1)
        self.assertLess(np.max(np.abs(q)), 1e-12)
        self.assertEqual(st[-1], "accepted")


class TestKinematicConsistency(unittest.TestCase):
    def _check(self, q, v, a, tol_v=2e-3, tol_a=0.6):
        # Position vs velocity: trapezoidal integration reproduces q.
        dq = np.diff(q)
        self.assertLess(np.max(np.abs(dq - 0.5 * (v[1:] + v[:-1]) * TS)), tol_v * TS + 1e-9)
        # Velocity vs acceleration: the emitted a is consistent with v over each
        # step, either as the step average (path) or trapezoidally (smooth joins).
        dv = np.diff(v)
        err = np.minimum(np.abs(dv - a[1:] * TS), np.abs(dv - 0.5 * (a[1:] + a[:-1]) * TS))
        self.assertLess(np.max(err), tol_a * TS)

    def test_reshaped_sweep_is_consistent(self):
        g = RollGovernor()
        g.reset(0.0)
        q, v, a, st = drive(g, RampedSine(R(80), 1.5, t_ramp=0.5), 2.4, 3000)
        self.assertIn("reshaped", st)
        self._check(q, v, a)

    def test_join_after_abrupt_request_is_consistent_and_budgeted(self):
        g = RollGovernor()
        g.reset(0.0)
        q, v, a, st = drive(g, Hold(R(60)), 3.2, 1500)
        self._check(q, v, a)
        self.assertAlmostEqual(q[-1], R(60), places=6)
        budget = g.available(3.2)
        demand = (g.J * np.abs(a) + g.b * np.abs(v) + g.tau_g * np.abs(np.sin(q)) + g.tau_c)
        self.assertLessEqual(np.max(demand), budget + 1e-9)

    def test_speed_limit_respected(self):
        g = RollGovernor(v_max=2.0)
        g.reset(0.0)
        q, v, a, st = drive(g, RampedSine(R(80), 1.5, t_ramp=0.5), 3.2, 3000)
        self.assertLessEqual(np.max(np.abs(v)), 2.0 + 1e-9)


class TestFeasibleSetBoundaries(unittest.TestCase):
    EXTRA = staticmethod(lambda q: 0.1)   # makes q > ~41 deg unholdable at 2.4 A

    def test_path_stops_before_unholdable_region(self):
        g = RollGovernor()
        g.reset(0.0)
        q, v, a, st = drive(g, RampedSine(R(80), 0.2, t_ramp=0.5), 2.4, 4000,
                            extra_fn=self.EXTRA)
        lo, hi = g.feasible_interval(g.available(2.4), 0.0, self.EXTRA)
        # A reference creeping into the boundary may stop up to one encoder count
        # beyond it (budgeted STOP) and then returns to it.
        from sim.params import ENC_LSB
        self.assertLessEqual(np.max(q), hi + ENC_LSB)
        self.assertLessEqual(q[-1], hi + 1e-9)
        self.assertEqual(st[-1], "restricted")
        self.assertLess(abs(v[-1]), 1e-9)
        self.assertTrue(g.rejected and g.reason)

    def test_unholdable_initial_state_is_explicitly_rejected(self):
        # Beyond actuator capacity even without the reserve: explicit rejection.
        g = RollGovernor()
        g.reset(R(70))
        q, v, a, st = drive(g, Hold(R(70)), 2.4, 50, extra_fn=lambda q: 0.15)
        self.assertEqual(st[-1], "rejected")
        self.assertLess(np.max(np.abs(v)), 1e-12)   # no invented motion

    def test_initial_state_outside_reserve_but_within_capacity_returns_labelled(self):
        # Outside the reserved interval but holdable with the reserve (second 2A
        # review, m3): return to the boundary, restricted, instead of passive fallback.
        g = RollGovernor()
        g.reset(R(70))
        q, v, a, st = drive(g, Hold(R(70)), 2.4, 1000, extra_fn=self.EXTRA)
        lo, hi = g.feasible_interval(g.available(2.4), 0.0, self.EXTRA)
        self.assertEqual(st[-1], "restricted")
        self.assertAlmostEqual(q[-1], hi, places=9)
        consistent(self, q, v, a)

    def test_derate_during_motion_keeps_reference_continuous(self):
        g = RollGovernor()
        g.reset(0.0)
        ref = RampedSine(R(80), 0.5, t_ramp=0.5)
        q1, v1, a1, _ = drive(g, ref, 3.2, 1000)
        q2, v2, a2, st = drive(g, ref, 2.4, 1500)
        q = np.concatenate([q1, q2])
        self.assertLess(np.max(np.abs(np.diff(q))), 20.0 * TS + 1e-9)   # no jump
        self.assertLess(np.max(np.abs(q)), R(80) + 1e-9)


class TestSyncDoesNotStallFeasiblePath(unittest.TestCase):
    """2A evaluation regression: syncing onto a path that is already (slowly)
    moving restarted the clock at s = 0 and ramped it at s_rate, so run A lost
    0.30 s of path time on a request the governor accepts in full."""

    def test_sync_onto_moving_feasible_path_keeps_clock(self):
        ref = MinJerkSequence(0.0, [(R(45), 0.35, 0.5), (R(-45), 0.5, 0.5)])
        g = RollGovernor()
        g.reset(0.0, t=TS)                  # host initialises on its first tick
        q, v, a, st = drive(g, ref, 3.2, 600)
        self.assertLess(TS + 600 * TS - g.sigma, 0.01)
        TestKinematicConsistency._check(self, q, v, a)
        budget = g.available(3.2)
        demand = np.abs(g.J * a + g.b * v + g.tau_g * np.sin(q)) + g.tau_c
        self.assertLessEqual(np.max(demand), budget + 1e-9)

    def test_one_count_realign_offset_does_not_stall_clock(self):
        # Start-up realign to a quantized measurement one count off the path
        # (run A, seeds 2-5: a 0.05 s join froze the clock on a moving path).
        from sim.params import ENC_LSB
        ref = MinJerkSequence(0.0, [(R(45), 0.35, 0.5)])
        g = RollGovernor()
        g.reset(-ENC_LSB, t=2 * TS)
        q, v, a, st = drive(g, ref, 3.2, 300)
        self.assertLess(2 * TS + 300 * TS - g.sigma, 0.005)
        self.assertLessEqual(abs(q[0] - ref.eval(g.sigma - 299 * TS)[0]), ENC_LSB)

    def test_small_start_offset_is_blended_without_stopping_the_clock(self):
        # After-2A evaluation (Monte Carlo A, encoder offset up to 0.01 rad): the
        # start-up join froze the clock for 0.05 s -> ~6.5 deg wall-clock RMS.
        ref = MinJerkSequence(0.0, [(R(45), 0.35, 0.5), (R(-45), 0.5, 0.5)])
        for off in (0.006, -0.01, R(2)):
            with self.subTest(offset=off):
                g = RollGovernor()
                g.reset(off, t=TS)
                q, v, a, st = drive(g, ref, 3.2, 600)
                self.assertLess(TS + 600 * TS - g.sigma, 0.005)
                consistent(self, q, v, a)
                self.assertAlmostEqual(q[-1], ref.eval(g.sigma)[0], places=9)
                budget = g.available(3.2)
                demand = np.abs(g.J * a + g.b * v + g.tau_g * np.sin(q)) + g.tau_c
                self.assertLessEqual(np.max(demand), budget + 1e-9)

    def test_saturation_evidence_survives_realignment(self):
        # After-2A evaluation (payload E + feedback loss): kappa reset to 1 at every
        # re-arm, the clock resumed at full speed and the watchdog tripped 33 times.
        g = RollGovernor()
        g.reset(0.0)
        for _ in range(100):
            g.step(TS, Hold(0.0).eval, 3.2, saturated=True)
        k = g.kappa
        self.assertLess(k, 0.9)
        g.reset(0.1, t=g.sigma)
        self.assertEqual(g.kappa, k)
        c = BaselineController()
        c.gov.kappa = 0.2
        c.reset(SimConfig())
        self.assertEqual(c.gov.kappa, 1.0)

    def test_sync_onto_fast_path_starts_only_as_fast_as_budget_allows(self):
        g = RollGovernor()
        g.reset(0.0, t=0.25)                # mid-sweep: path moving at ~7 rad/s
        ref = RampedSine(R(80), 1.5, t_ramp=0.0)
        q, v, a, st = drive(g, ref, 3.2, 5)
        budget = g.available(3.2)
        demand = np.abs(g.J * a + g.b * v + g.tau_g * np.sin(q)) + g.tau_c
        self.assertLessEqual(np.max(demand), budget + 1e-9)


class TestJoinReservesCoupling(unittest.TestCase):
    """2A evaluation regression: after an outage the re-join was planned without
    the yaw coupling, its feed-forward clipped at the current limit and the
    back-calculation wound the integrator against the error (C, feedback loss:
    ~5 deg held for 0.4 s after re-arm)."""

    def test_rejoin_under_coupling_stays_within_budget(self):
        g = RollGovernor()
        g.reset(0.0)
        w, amp = 2 * math.pi * 2.2, 0.2
        cpl = lambda k: amp * math.sin(w * k * TS)
        for k in range(500):                           # coupling seen before the outage
            g.step(TS, Hold(0.0).eval, 3.2, tau_couple=cpl(k))
        g.reset(R(-5), t=g.sigma)                     # host realign after fallback
        budget = g.available(3.2)
        worst = 0.0
        for k in range(500, 700):
            q, v, a = g.step(TS, Hold(0.0).eval, 3.2, tau_couple=cpl(k))
            worst = max(worst, g._demand(q, v, a, 0.0, None, cpl(k)))
        self.assertAlmostEqual(g.q, 0.0, places=9)    # join completed
        self.assertLessEqual(worst, budget + 1e-9)


class TestHostIntegration(unittest.TestCase):
    def test_host_hold_reference_does_not_drift_under_excess_yaw_coupling(self):
        c = planned()
        c.reset(SimConfig())
        ref = Hold(0.0)
        yaw = lambda t: (0.0, 0.0, 1000.0)       # 0.8 N m of predicted coupling
        qc = []
        for k in range(300):
            t = k * TS
            ctx = Context(t, ref.eval(t), (0, 0, 0), ref.eval, yaw)
            c.update(ctx, Feedback(k, t, 0.0, 0.0, 0.0, 3.2, 0))
            qc.append(c.telemetry["q_c"])
        self.assertLess(np.max(np.abs(qc)), 1e-9)


class TestSimIntegration(unittest.TestCase):
    """Same reproducer through drive, transport and plant: yaw coupling larger
    than the roll budget (|tau_cpl| ~ 0.42 N m > 0.27 N m) with roll held at 0."""

    @staticmethod
    def _run(governed_yaw):
        from dataclasses import replace
        from exp.common import run
        from exp.scenarios import Scenario
        cfg = replace(SimConfig(), duration=1.5)
        sc = Scenario("hold", cfg, Hold(0.0), RampedSine(R(75), 3.0, t_ramp=0.3), "")
        log, _ = run(sc, BaselineController, seed=3, governed_yaw=governed_yaw)
        return log

    # The reference starts at the measured (quantized) position and joins the
    # hold, so the only allowed excursion is one encoder count; the pre-2A
    # failure drifted 5.4 rad.
    LSB = 2 * math.pi / 2 ** 14 + 1e-9

    def test_coordinated_yaw_hold_reference_stays_put(self):
        log = self._run(True)
        active = np.asarray(log.c_host_fallback) < 0.5
        self.assertLessEqual(np.max(np.abs(np.asarray(log.c_q_c)[active])), self.LSB)
        self.assertGreater(np.nanmax(log.c_gov_over_budget), 0.5)   # disposition visible

    def test_uncoordinated_yaw_changes_decision_not_reference(self):
        log = self._run(False)
        active = np.asarray(log.c_host_fallback) < 0.5
        self.assertLessEqual(np.max(np.abs(np.asarray(log.c_q_c)[active])), self.LSB)
        flagged = np.fmax(np.asarray(log.c_request_rejected), np.asarray(log.c_gov_incompatible))
        self.assertGreater(np.nanmax(flagged), 0.5)


# ---------------------------------------------------------------------------
# Independent 2A review findings (scratchpad/review2A). Each test is the
# reviewer's reproducer, written before the correction.
# ---------------------------------------------------------------------------
def consistent(tc, q, v, a, tol_v=2e-3, tol_a=0.6):
    TestKinematicConsistency._check(tc, q, v, a, tol_v, tol_a)


class TestReviewGovernor(unittest.TestCase):
    EXTRA = staticmethod(lambda q: 0.1)

    def test_M1_path_jump_is_not_passed_through(self):
        ref = lambda t: ((0.0 if t < 1.0 else R(30)), 0.0, 0.0)
        g = RollGovernor()
        g.reset(0.0)
        q, v, a, st = drive(g, type("J", (), {"eval": staticmethod(ref)}), 3.2, 1500)
        consistent(self, q, v, a)
        self.assertAlmostEqual(q[-1], R(30), places=6)
        self.assertLessEqual(np.max(np.abs(v)), g.v_max + 1e-9)

    def test_M2_fast_feasible_move_completes(self):
        for T in (0.05, 0.02):
            for il in (3.2, 2.4):
                with self.subTest(T=T, i_lim=il):
                    g = RollGovernor()
                    g.reset(0.0)
                    q, v, a, st = drive(g, MinJerkSequence(0.0, [(R(90), T, 2.0)]), il, 2500)
                    self.assertAlmostEqual(q[-1], R(90), places=4)
                    # Trapezoid error of a sampled path is ~ts^3 * jerk / 12; a 90 deg
                    # move in <= 0.05 s has <= 25 samples (error ~5e-6 rad), hence the
                    # wider velocity tolerance for these.
                    consistent(self, q, v, a, tol_v=2e-3 if T > 0.05 else 1e-2)

    def test_M4_braking_room_loss_is_consistent_and_surfaced(self):
        ref = RampedSine(R(80), 0.5, t_ramp=0.5)
        for kd in (120, 140, 156, 1100):
            with self.subTest(derate_step=kd):
                g = RollGovernor()
                g.reset(0.0)
                out, flags = [], set()
                for k in range(2000):
                    qq, vv, aa = g.step(TS, ref.eval, 3.2 if k < kd else 2.4,
                                        extra_fn=self.EXTRA)
                    out.append((qq, vv, aa))
                    flags.add((g.status, g.infeasibility))
                q, v, a = (np.array(x) for x in zip(*out))
                consistent(self, q, v, a)
                self.assertLessEqual(np.max(np.abs(np.diff(v))), 0.5)   # no velocity step
                lo, hi = g.feasible_interval(g.available(2.4), 0.0, self.EXTRA)
                self.assertLessEqual(q[-1], hi + 1e-9)
                if kd < 200:      # derate while moving towards the boundary
                    self.assertTrue({("restricted", "braking"), ("rejected", "braking")} & flags)

    def test_M5_join_rechecked_after_derate(self):
        g = RollGovernor()
        g.reset(0.0)
        out = []
        for k in range(600):
            qq, vv, aa = g.step(TS, Hold(R(60)).eval, 3.2 if k < 20 else 2.4,
                                extra_fn=self.EXTRA)
            out.append((qq, vv, aa))
            lo, hi = g.feasible_interval(g.available(2.4 if k >= 20 else 3.2), 0.0, self.EXTRA)
            if k > 20 + 150:          # after at most a braking segment
                self.assertLessEqual(qq, hi + 1e-9)
        q, v, a = (np.array(x) for x in zip(*out))
        consistent(self, q, v, a)
        self.assertEqual(g.status, "restricted")

    def test_M6_restricted_target_never_moves_away_from_request(self):
        tav = RollGovernor().available(2.4)
        c = tav - 0.04 - 0.12 * math.sin(R(41)) - 1e-5
        for q0 in (40.9, 40.5, 40.0):
            with self.subTest(q0=q0):
                g = RollGovernor()
                g.reset(R(q0))
                q, v, a, st = drive(g, Hold(R(50)), 2.4, 800, tau_extra=c)
                self.assertGreaterEqual(np.min(q), R(q0) - 1e-9)
                self.assertEqual(st[-1], "restricted")
                self.assertNotIn("braking", g.reason)

    def test_M7_nonfinite_inputs_are_rejected_not_projected(self):
        nan = float("nan")
        cases = [
            (lambda t: (nan, nan, nan) if t < 1e-9 else (0.0, 0.0, 0.0), {}),
            (lambda t: (0.0, nan, 0.0), {}),
            (lambda t: (0.0, 0.0, 0.0), dict(tau_couple=nan)),
            (lambda t: (0.0, 0.0, 0.0), dict(tau_couple=float("inf"))),
        ]
        for i, (ref, kw) in enumerate(cases):
            with self.subTest(case=i):
                g = RollGovernor()
                g.reset(0.0)
                qs = []
                for _ in range(100):
                    qs.append(g.step(TS, ref, 3.2, **kw)[0])
                self.assertEqual(g.status, "rejected")
                self.assertIn("non-finite", g.reason)
                self.assertLess(np.max(np.abs(qs)), 1e-12)

    def test_m2_restricted_hold_reports_restriction_not_braking(self):
        g = RollGovernor()
        g.reset(0.0)
        q, v, a, st = drive(g, Hold(R(95)), 3.2, 1000)
        self.assertAlmostEqual(q[-1], R(90), places=6)
        self.assertEqual(st[-1], "restricted")
        self.assertIn("outside holdable", g.reason)

    def test_m7_interval_cache_sees_load_model_change(self):
        g = RollGovernor()
        tav = g.available(2.4)
        f2 = lambda q: 0.2 if 0.3 < q < 0.9 else 0.0
        g.feasible_interval(tav, 0.0, lambda q: 0.0)
        self.assertEqual(g.feasible_interval(tav, 0.0, f2),
                         RollGovernor().feasible_interval(tav, 0.0, f2))

    def test_m8_invalid_configuration_is_refused(self):
        for kw in (dict(n_look=0), dict(horizon=0.0), dict(v_max=0.0), dict(s_rate=0.0)):
            with self.subTest(**kw):
                with self.assertRaises(ValueError):
                    RollGovernor(**kw)
        g = RollGovernor()
        g.reset(0.0)
        with self.assertRaises(ValueError):
            g.step(0.0, Hold(0.0).eval, 3.2)


class Ramp:
    """Constant yaw rate: coupling k_yv * w with no acceleration term."""
    def __init__(self, w):
        self.w = w

    def eval(self, t):
        return (self.w * t, self.w, 0.0)


class ConstAcc:
    def __init__(self, acc):
        self.acc = acc

    def eval(self, t):
        return (0.0, 0.0, self.acc)


def planned():
    """Plan-look-ahead host (Packet 2B yaw_info="plan"). The synthetic ConstAcc /
    lambda yaw inputs carry acceleration without the matching position, so only the
    plan mode sees their coupling; they exercise the predicted-coupling logic."""
    return BaselineController(yaw_info="plan")


def sim(yaw, roll=Hold(0.0), dur=1.5, ctrl=None, cfg=None, seed=1):
    from dataclasses import replace
    from ctrl.supervisor import DriveSupervisor
    from sim.engine import simulate
    cfg = replace(cfg or SimConfig(), duration=dur)
    c = ctrl or BaselineController()
    return simulate(cfg, c, roll, yaw, seed=seed, safety=DriveSupervisor()), c


class TestReviewHost(unittest.TestCase):
    def test_M8a_no_planner_overbudget_keeps_control_and_reports(self):
        # 45 rad/s yaw: 0.36 N m nominal coupling, over the 0.308 budget but
        # within 0.448 N m capacity. Rejection to passive fallback ran away (728 deg).
        log, c = sim(Ramp(45.0), dur=2.0)
        self.assertLess(np.max(np.abs(log.q)), R(5))
        self.assertGreater(np.nanmax(log.c_gov_incompatible), 0.5)

    def test_M8b_coordinated_overbudget_shrinks_yaw_before_drive_trips(self):
        from sim.trajectories import GovernedYaw
        yaw = GovernedYaw(ConstAcc(1000.0))           # 0.8 N m predicted coupling
        log, c = sim(yaw, dur=1.0, ctrl=planned())
        shrinks = [t for t, s in yaw.scale_log[1:]]
        self.assertTrue(shrinks)
        first_event = min([e[0] for e in log.events] or [1e9])
        self.assertLess(shrinks[0], first_event)
        self.assertLess(shrinks[0], 0.02)

    def test_M8c_no_planner_saturation_does_not_hand_over_to_passive_fallback(self):
        # 0.4 N m constant coupling (inside the 0.448 N m capacity): the YawMonitor
        # path rejected -> passive damping -> runaway (~800 deg). Closed in 2B.
        log, c = sim(ConstAcc(500.0), dur=1.5, ctrl=planned())
        self.assertFalse(c.request_blocked)
        self.assertLess(np.max(np.abs(log.q)), R(10))

    def test_M8d_coupling_beyond_capacity_is_reported_before_the_drive_trips(self):
        log, c = sim(ConstAcc(1000.0), dur=0.5, ctrl=planned())  # 0.8 N m > capacity
        inc = np.asarray(log.c_gov_incompatible)
        t_inc = log.t[np.argmax(np.nan_to_num(inc) > 0.5)]
        first_event = min([e[0] for e in log.events] or [1e9])
        self.assertGreater(np.nanmax(inc), 0.5)
        self.assertLess(t_inc, first_event)

    def test_no_yaw_shrink_when_coupling_is_not_the_cause(self):
        # After-2A evaluation (payload E + feedback loss): in drive fallback at a
        # loaded angle the hold alone exceeds the budget (room < 0) with zero yaw
        # coupling, and the proportional factor drove the yaw scale to 0.
        from sim.trajectories import GovernedYaw
        for mode in (1, 0):
            with self.subTest(drive_mode=mode):
                yaw = GovernedYaw(Hold(0.0))
                c = BaselineController()
                c.reset(SimConfig())
                for k in range(100):
                    t = k * TS
                    ctx = Context(t, (R(80), 0, 0), (0, 0, 0), Hold(R(80)).eval, yaw.eval, yaw)
                    c.update(ctx, Feedback(k, t, R(80), 0.0, 0.0, 1.0, mode))
                self.assertEqual(yaw.scale(1.0), 1.0)

    def test_m1_idle_telemetry_reports_rejection(self):
        nan = float("nan")
        roll = type("N", (), {"eval": staticmethod(lambda t: (nan if t > 0.2 else 0.0, 0.0, 0.0))})()
        log, c = sim(Hold(0.0), roll=roll, dur=0.5)
        idx = RollGovernor.STATUS.index("rejected")
        self.assertTrue(c.request_blocked)
        self.assertEqual(np.asarray(log.c_gov_status)[-1], idx)

    def test_m3_over_budget_timer_restarts_after_idle(self):
        # 38 rad/s: 0.304 N m, over budget but within capacity, so only the escalation
        # timer runs; the blackout is short enough that the coupling cannot push the
        # passive axis to where holding exceeds capacity. (45 rad/s is incompatible at
        # once, which since Packet 2B suspends the request and stops the timer.)
        cfg = SimConfig().with_(timing=dict(feedback_blackout=((0.10, 0.14),)))
        c = BaselineController()
        log, c = sim(Ramp(38.0), dur=0.35, ctrl=c, cfg=cfg)
        self.assertFalse(c.incompatible)
        self.assertGreaterEqual(c.over_since or 0.0, 0.14)


# ---------------------------------------------------------------------------
# Second independent 2A review (scratchpad/review2A_r2). Reproducers first.
# ---------------------------------------------------------------------------
def triangle(A, slope):
    P = 4 * A / slope

    def f(t):
        x = t % P
        if x < P / 4:
            return (slope * x, slope, 0.0)
        if x < 3 * P / 4:
            return (A - slope * (x - P / 4), -slope, 0.0)
        return (-A + slope * (x - 3 * P / 4), slope, 0.0)
    return f


def jumped_move(EX_shift=R(5.8)):
    mj = MinJerkSequence(0.0, [(R(-60), 0.9, 2.0)])
    return lambda t: (mj.eval(t)[0] + (EX_shift if t >= 0.30 else 0.0),) + mj.eval(t)[1:]


class TestReview2Governor(unittest.TestCase):
    EX = staticmethod(lambda q: 0.1)

    def _stop_run(self, ilim):
        EX = lambda q: 0.085
        ref = jumped_move()
        g = RollGovernor()
        g.reset(0.0)
        rows, kstop = [], None
        for k in range(1500):
            il = ilim(k, kstop)
            q, v, a = g.step(TS, ref, il, extra_fn=EX)
            if g.mode == "stop" and kstop is None:
                kstop = k
            rows.append((q, v, a, il, g.mode))
        return g, rows, kstop, EX

    def test_N1_stop_replans_when_limit_rises(self):
        # A limit drop while moving (not anticipated by the look-ahead) starts a
        # STOP; restoring the limit 10 ms later must shorten it (a_b re-planned).
        def run(restore):
            ref = RampedSine(R(80), 0.5, t_ramp=0.5)
            g = RollGovernor()
            g.reset(0.0)
            ks, n = None, 0
            for k in range(1500):
                il = 3.2 if k < 140 else 2.4
                if restore and ks is not None and k >= ks + 5:
                    il = 3.2
                g.step(TS, ref.eval, il, extra_fn=self.EX)
                if g.mode == "stop":
                    ks = k if ks is None else ks
                    n += 1
            return n
        n0, n1 = run(False), run(True)
        self.assertGreater(n0, 5)
        self.assertLess(n1, n0)

    def test_N1_stop_replans_when_limit_falls(self):
        g, rows, ks, EX = self._stop_run(lambda k, ks: 3.2 if ks is None or k < ks + 2 else 1.6)
        for q, v, a, il, mode in rows:
            if mode == "stop":
                self.assertLessEqual(g._demand(q, v, a, 0.0, EX, 0.0),
                                     g._capacity(g.available(il)) + 1e-9)

    def test_N3_kink_near_travel_limit_stays_in_range_and_is_not_rejected(self):
        for A, sl in ((R(88), 3.0), (R(80), 3.0)):
            with self.subTest(apex=math.degrees(A)):
                g = RollGovernor()
                g.reset(0.0)
                qs = [g.step(TS, triangle(A, sl), 3.2)[0] for _ in range(2000)]
                self.assertLessEqual(max(abs(x) for x in qs), R(90) + 1e-9)
                self.assertNotEqual(g.status, "rejected")

    def test_N2_restricted_hold_resumes_after_limit_restored(self):
        for amp, f in ((80, 0.5), (60, 0.2), (45, 0.5)):
            with self.subTest(amp=amp, f=f):
                ref = RampedSine(R(amp), f, t_ramp=0.5)
                g = RollGovernor()
                g.reset(0.0)
                sig = []
                for k in range(4000):
                    g.step(TS, ref.eval, 2.4 if k < 2500 else 3.2, extra_fn=self.EX)
                    sig.append(g.sigma)
                self.assertGreater(sig[-1] - sig[2600], 2.0)     # clock runs again

    def test_hold_entered_after_limit_restored_still_resumes(self):
        # Fuzz seed 42 (second review harness): the braking-room return join ended
        # after the limit was already back, and the hold never resumed.
        ref = RampedSine(R(80), 0.5, t_ramp=0.5)
        g = RollGovernor()
        g.reset(0.0)
        modes = []
        for k in range(3000):
            il = 2.4 if 140 <= k < 175 else 3.2       # derate while moving, restored mid-return
            g.step(TS, ref.eval, il, extra_fn=self.EX)
            modes.append(g.mode)
        self.assertIn("stop", modes)
        self.assertEqual(g.mode, "path")
        self.assertGreater(g.sigma, 5.0)

    def test_plan_with_inconsistent_position_and_velocity_is_contained(self):
        # Fuzz seeds 40/214: a plan clipped at 89 deg (q flat while v says moving)
        # was met at speed (reference to 92.6 deg) or looped stop/join forever.
        sine = RampedSine(R(120), 0.5, t_ramp=0.0)
        ref = lambda t: (max(-R(89), min(R(89), sine.eval(t)[0])),) + sine.eval(t)[1:]
        g = RollGovernor()
        g.reset(0.0)
        qs = []
        for _ in range(3000):
            qs.append(g.step(TS, ref, 3.2)[0])
            if g.latched:
                break
        self.assertLessEqual(max(abs(x) for x in qs), R(90) + 1e-9)
        self.assertEqual(g.status, "rejected")
        self.assertIn("inconsistent", g.reason)

    def test_M3_resync_onto_moving_accepted_path_loses_little_time(self):
        ref = RampedSine(R(45), 0.5, t_ramp=0.0)
        g = RollGovernor()
        g.reset(0.0)
        rows = [(g.step(TS, ref.eval, 3.2), g.sigma)[1] for _ in range(1000)]
        k0 = 300
        g2 = RollGovernor()
        g2.reset(ref.eval(rows[k0])[0], t=rows[k0])
        for _ in range(500):
            g2.step(TS, ref.eval, 3.2)
        lost = (rows[k0] + 500 * TS) - g2.sigma
        self.assertLess(lost, 0.03)

    def test_m1_capacity_return_join_is_labelled_restricted_braking(self):
        ref = RampedSine(R(80), 0.5, t_ramp=0.5)
        g = RollGovernor()
        g.reset(0.0)
        seen = set()
        for k in range(2000):
            g.step(TS, ref.eval, 3.2 if k < 140 else 2.4, extra_fn=self.EX)
            if g.mode == "join" and g.seg.get("label") == "braking":
                seen.add((g.status, g.infeasibility))
        self.assertTrue(seen)
        self.assertEqual(seen, {("restricted", "braking")})

    def test_m2_creeping_into_boundary_without_limit_change_is_not_braking(self):
        g = RollGovernor(horizon=0.05)
        g.reset(0.0)
        labels = set()
        for _ in range(4000):
            g.step(TS, RampedSine(R(80), 0.2, t_ramp=0.5).eval, 2.4, extra_fn=self.EX)
            labels.add(g.infeasibility)
        self.assertNotIn("braking", labels)

    def test_m3_stationary_hold_outside_after_derate_returns_like_a_moving_one(self):
        g = RollGovernor()
        g.reset(0.0)
        for k in range(1500):
            g.step(TS, Hold(R(60)).eval, 3.2 if k < 1000 else 2.4, extra_fn=self.EX)
        lo, hi = g.feasible_interval(g.available(2.4), 0.0, self.EX)
        self.assertEqual(g.status, "restricted")
        self.assertAlmostEqual(g.q, hi, places=9)

    def test_m3_reset_one_count_beyond_boundary_is_restricted_not_rejected(self):
        from sim.params import ENC_LSB
        g = RollGovernor()
        lo, hi = g.feasible_interval(g.available(2.4), 0.0, self.EX)
        g.reset(hi + ENC_LSB)
        for _ in range(200):
            g.step(TS, Hold(R(60)).eval, 2.4, extra_fn=self.EX)
        self.assertEqual(g.status, "restricted")

    def test_m5_start_offset_blend_is_reported(self):
        g = RollGovernor()
        g.reset(R(1.0))
        g.step(TS, Hold(0.0).eval, 3.2)
        self.assertEqual(g.status, "joining")
        self.assertIn("blend", g.reason)

    def test_m8_coarse_step_smooth_path_is_not_a_discontinuity(self):
        g = RollGovernor()
        g.reset(0.0)
        for _ in range(200):
            g.step(0.02, RampedSine(R(60), 0.5, t_ramp=0.0).eval, 3.2)   # C-infinity
            self.assertNotIn("discontinuity", g.reason)


class TestReview2Host(unittest.TestCase):
    def test_M8b_outcome_coordinated_coupling_within_capacity_is_contained(self):
        from sim.trajectories import GovernedYaw
        yaw = GovernedYaw(ConstAcc(500.0))            # 0.4 N m: over budget, within capacity
        log, c = sim(yaw, dur=1.5, ctrl=planned())
        self.assertLess(np.max(np.abs(log.q)), R(2))
        self.assertFalse(log.events)
        self.assertFalse(c.request_blocked)

    def test_m4_coupling_on_an_over_budget_reference_still_shrinks_yaw(self):
        from sim.trajectories import GovernedYaw
        yaw = GovernedYaw(Hold(0.0))
        c = planned()
        c.reset(SimConfig())
        big = lambda t: (0.0, 0.0, 300.0)             # 0.24 N m of coupling
        for k in range(50):
            t = k * TS
            ctx = Context(t, (R(80), 0, 0), (0, 0, 0), Hold(R(80)).eval, big, yaw)
            c.update(ctx, Feedback(k, t, R(80), 0.0, 0.0, 1.5, 0))
        self.assertLess(yaw.scale(1.0), 1.0)

    def test_m6_replan_clears_incompatibility(self):
        log, c = sim(Ramp(45.0), dur=1.0)
        self.assertTrue(c.incompatible)
        c.replan()
        self.assertFalse(c.incompatible or c.request_blocked)

    def test_m2_idle_is_not_reported_as_accepted(self):
        cfg = SimConfig().with_(timing=dict(feedback_blackout=((0.10, 0.20),)))
        log, c = sim(Hold(0.0), dur=0.3, cfg=cfg)
        acc = RollGovernor.STATUS.index("accepted")
        idle = np.asarray(log.c_host_fallback) > 0.5
        self.assertFalse(np.any(np.asarray(log.c_gov_status)[idle] == acc))


# ---------------------------------------------------------------------------
# Third independent 2A review (scratchpad/review2A_r3). Reproducers first.
# ---------------------------------------------------------------------------
def signed_demand(g, q, v, a, extra_fn=None, tau_couple=0.0):
    return g._demand(q, v, a, 0.0, extra_fn, tau_couple)


class TestReview3(unittest.TestCase):
    def test_R3M1_first_path_step_after_resync_stays_within_budget(self):
        ref = RampedSine(R(45), 0.5, t_ramp=0.0)             # constant-amplitude, moving
        for t0 in (0.3, 0.6, 1.1):
            with self.subTest(t0=t0):
                g = RollGovernor()
                g.reset(ref.eval(t0)[0], t=t0)
                budget = g.available(3.2)
                for _ in range(60):
                    q, v, a = g.step(TS, ref.eval, 3.2)
                    self.assertLessEqual(signed_demand(g, q, v, a), budget + 1e-9)

    def test_R3M2_kinks_are_crossed_slowly_within_capacity_without_overshoot(self):
        for A, sl in ((R(45), 6.0), (R(88), 3.0), (R(60), 3.0)):
            with self.subTest(apex=math.degrees(A), slope=sl):
                g = RollGovernor()
                g.reset(0.0)
                cap = g._capacity(g.available(3.2))
                qs = []
                for _ in range(3000):
                    q, v, a = g.step(TS, triangle(A, sl), 3.2)
                    qs.append(q)
                    self.assertLessEqual(signed_demand(g, q, v, a), cap + 1e-9)
                    self.assertNotIn("discontinuity", g.reason)
                self.assertLessEqual(max(abs(x) for x in qs), A + R(0.01))

    def test_R3m1_stop_steps_over_capacity_are_labelled_over_budget(self):
        ref = RampedSine(R(60), 1.0, t_ramp=0.3)
        cpl = lambda k: 0.25 * math.sin(2 * math.pi * 1.7 * k * TS)
        g = RollGovernor()
        g.reset(0.0)
        stops = 0
        for k in range(3000):
            il = 1.6 if (k // 150) % 2 else 3.2              # repeated limit drops while moving
            q, v, a = g.step(TS, ref.eval, il, tau_couple=cpl(k))
            if g.mode == "stop":
                stops += 1
                if signed_demand(g, q, v, a, None, cpl(k)) > g._capacity(g.available(il)) + 1e-9:
                    self.assertEqual(g.status, "over_budget")
        self.assertGreater(stops, 0)

    def test_R3d_single_jump_is_not_rejected_while_saturation_backoff_is_at_floor(self):
        # Delta check of round 3: with kappa at its 0.1 floor the jump was crossed
        # at s ~ 0.001 and the q/v persistence check (wall time) rejected the plan.
        mj = MinJerkSequence(0.0, [(R(30), 1.0, 5.0)])
        jump = lambda t: (mj.eval(t)[0] + (R(5) if t >= 0.5 else 0.0),) + mj.eval(t)[1:]
        for ref in (jump, Hold(R(20)).eval):
            g = RollGovernor()
            g.reset(0.0)
            for _ in range(10000):
                g.step(TS, ref, 3.2, saturated=True)          # kappa pinned at its floor
                self.assertNotEqual(g.status, "rejected", g.reason)
            self.assertAlmostEqual(g.q, ref(g.sigma)[0], places=6)


if __name__ == "__main__":
    unittest.main()
