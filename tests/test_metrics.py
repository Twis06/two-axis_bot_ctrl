"""Packet 4A: metrics cannot reward a stationary/rejecting controller; strata
partition time; manifests identify runs; staged publishing is all-or-nothing.

    python -m unittest tests.test_metrics -v
"""
import json
import math
import os
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ctrl.baseline import BaselineController  # noqa: E402
from exp import manifest as MF  # noqa: E402
from exp import motions as M  # noqa: E402
from exp.common import run  # noqa: E402
from exp.metrics4a import RejectingDiagnostic, StationaryDiagnostic  # noqa: E402
from sim import metrics as SM  # noqa: E402
from sim.config import SimConfig  # noqa: E402
from sim.engine import Log  # noqa: E402
from sim.trajectories import MinJerkSequence, RampedSine  # noqa: E402

R = math.radians
PATH = MinJerkSequence(0.0, [(R(45), 0.5, 0.3), (R(-45), 0.5, 0.3), (0.0, 0.5, 0.3)])
WAYPOINTS = [R(45), R(-45), 0.0]
T_REQ = 2.1


def synth(q=None, tele=None, T=4.0, mode=None, events=(), yaw_scale=None, qy=None, cfg=None):
    """Hand-built Log with the engine's keys. q defaults to stationary at 0."""
    t = np.arange(int(T * 1000)) / 1000.0
    n = len(t)
    ref = np.array([PATH.eval(x)[0] for x in t])
    q = np.zeros(n) if q is None else np.asarray(q, float)
    z = np.zeros(n)
    log = Log(t=t, q=q, qd=np.gradient(q, t), i=z.copy(), i_raw=z.copy(), i_tgt=z.copy(), clipped=z.copy(),
              vlim=z.copy(), i_lim=np.full(n, 3.2), mode=z.copy() if mode is None else np.asarray(mode, float),
              q_ref=ref, qd_ref=z.copy(), qdd_ref=z.copy(), err=q - ref, q_enc=q.copy(),
              qy=z.copy() if qy is None else qy, qyd=z.copy(), qydd=z.copy(), tau_couple=z.copy(), d=z.copy(),
              cmd_age=np.full(n, 0.003), fb_age=np.full(n, 0.003), burst=z.copy())
    for k, v in (tele or {}).items():
        log["c_" + k] = np.broadcast_to(np.asarray(v, float), (n,)).copy()
    log.events = list(events)
    log.meta = dict(seed=0, controller="synthetic", wd_trips=0, cfg=cfg or SimConfig(),
                    T_winding=None, yaw_scale=yaw_scale)
    return log


def rejected_stationary():
    n = 4000
    t = np.arange(n) / 1000.0
    return synth(tele=dict(q_c=0.0, gov_lag=t - (t % 0.002), gov_status=5, request_rejected=1,
                           host_fallback=1, gov_s=0.0, gov_rejected=1))


def claims_perfect_stationary():
    return synth(tele=dict(q_c=0.0, gov_lag=0.0, gov_status=0, request_rejected=0, host_fallback=0,
                           gov_s=1.0, gov_rejected=0))


def perfect_tracker():
    t = np.arange(4000) / 1000.0
    q = np.array([PATH.eval(x)[0] for x in t])
    return synth(q=q, tele=dict(q_c=q, gov_lag=0.0, gov_status=0, request_rejected=0, host_fallback=0,
                                gov_s=1.0, gov_rejected=0))


def ev(log):
    return SM.evaluate(log, ref=PATH, T_request=T_REQ, waypoints=WAYPOINTS)


class TestNoCreditForStandingStill(unittest.TestCase):
    def test_positive_control_perfect_tracker_passes(self):
        e = ev(perfect_tracker())
        self.assertTrue(e["tracked"], e["not_tracked_because"])
        self.assertTrue(e["completion"]["completed"])
        self.assertLess(abs(e["completion"]["t_complete"] - T_REQ), 0.2)

    def test_rejected_stationary_is_not_success(self):
        e = ev(rejected_stationary())
        self.assertLess(e["errors"]["gov_rms"], 1e-9)          # the trap: governed error is zero
        self.assertLess(e["errors"]["path_rms"], 1e-3)         # and so is error at the frozen sigma
        self.assertFalse(e["tracked"])
        self.assertFalse(e["completion"]["completed"])
        self.assertEqual(e["completion"]["time_ratio"], 0.0)
        self.assertEqual(e["progress"]["progress"], 0.0)
        self.assertAlmostEqual(e["strata"]["rejected"]["share_pct"], 100.0)

    def test_stationary_claiming_accepted_wall_clock_is_not_success(self):
        """Telemetry claims accepted tracking with zero lag: progress looks full,
        but the original path at that sigma exposes the plant standing still."""
        e = ev(claims_perfect_stationary())
        self.assertLess(e["errors"]["gov_rms"], 1e-9)
        self.assertGreater(e["errors"]["path_rms"], 10.0)
        self.assertFalse(e["tracked"])
        self.assertFalse(e["completion"]["completed"])

    def test_ungoverned_stationary_is_not_success(self):
        e = ev(synth())            # no telemetry at all: sigma = wall clock
        self.assertFalse(e["tracked"])
        self.assertFalse(e["completion"]["completed"])

    def test_completion_needs_waypoints_in_order_and_healthy_settle(self):
        t = np.arange(4000) / 1000.0
        q = np.array([PATH.eval(x)[0] for x in t])
        skip = q.copy()
        skip[(t > 0.2) & (t < 1.0)] = R(10)     # never reaches +45 deg
        self.assertFalse(SM.completion(synth(q=skip), WAYPOINTS, T_REQ)["completed"])
        mode = (t > 1.5).astype(float)          # drive fallback before the final settle
        self.assertFalse(SM.completion(synth(q=q, mode=mode), WAYPOINTS, T_REQ)["completed"])
        self.assertTrue(SM.completion(synth(q=q), WAYPOINTS, T_REQ)["completed"])

    def test_real_run_diagnostic_comparators_fail(self):
        sc = M.m1(SimConfig())
        sc = replace(sc, cfg=replace(sc.cfg, duration=3.0))
        for make in (StationaryDiagnostic, RejectingDiagnostic):
            log, _ = run(sc, make, seed=1)
            e = SM.evaluate(log, ref=sc.roll, T_request=sc.t_request, waypoints=sc.waypoints)
            self.assertFalse(e["tracked"], make.name)
            self.assertFalse(e["completion"]["completed"], make.name)


class TestClockAndStrata(unittest.TestCase):
    def test_path_clock_uses_host_tick_time(self):
        """div = 2: telemetry forward-filled on the odd drive tick refers to the
        even host tick, so sigma is exact on every sample."""
        t = np.arange(1000) / 1000.0
        th = t - (np.arange(1000) % 2) / 1000.0
        sigma_true = 0.5 * th
        log = synth(T=1.0, tele=dict(gov_lag=th - sigma_true, host_fallback=0, q_c=0.0))
        np.testing.assert_allclose(SM.path_clock(log), sigma_true, atol=1e-12)

    def test_clock_jump_is_not_progress(self):
        t = np.arange(2000) / 1000.0
        th = t - (np.arange(2000) % 2) / 1000.0
        sigma = np.where(th < 1.0, 0.1 * th, th)          # frozen-ish, then jumps to wall clock
        log = synth(T=2.0, tele=dict(gov_lag=th - sigma, host_fallback=0, q_c=0.0))
        p = SM.progress(log, 2.0)
        self.assertEqual(p["clock_jumps"], 1)
        self.assertAlmostEqual(p["path_time"], 0.1 + 1.0, delta=0.01)

    def test_strata_partition_synthetic(self):
        t = np.arange(4000) / 1000.0
        status = np.where(t < 1, 0, np.where(t < 2, 1, 0))
        mode = ((t >= 2) & (t < 2.5)).astype(float)
        rej = (t >= 3.5).astype(float)
        log = synth(tele=dict(gov_status=status, host_fallback=rej, request_rejected=rej, gov_lag=0.0, q_c=0.0),
                    mode=mode)
        st = SM.stratified(log, ref=PATH)
        self.assertAlmostEqual(sum(v["share_pct"] for v in st.values()), 100.0, places=9)
        self.assertAlmostEqual(st["reshaping"]["share_pct"], 25.0, places=6)
        self.assertAlmostEqual(st["fallback"]["share_pct"], 12.5, places=6)
        self.assertAlmostEqual(st["recovery"]["share_pct"], 12.5, places=6)   # 2.5-3.0 s
        self.assertAlmostEqual(st["rejected"]["share_pct"], 12.5, places=6)

    def test_strata_partition_real_run_with_fault(self):
        sc = M.m3(SimConfig())
        cfg = replace(sc.cfg, duration=3.0).with_(timing=dict(command_blackout=((1.5, 1.56),)))
        log, _ = run(replace(sc, cfg=cfg), BaselineController, seed=1)
        st = SM.stratified(log, ref=sc.roll)
        self.assertAlmostEqual(sum(v["share_pct"] for v in st.values()), 100.0, places=9)
        self.assertGreater(st["fallback"]["time_s"], 0.0)
        self.assertGreater(st["recovery"]["time_s"], 0.0)
        f = SM.fault_timeline(log)
        self.assertIn("cmd_timeout", f["counts"])
        self.assertGreaterEqual(f["drive_fallback"]["count"], 1)


class TestCurrentFaultYaw(unittest.TestCase):
    def test_saturation_and_limit_changes(self):
        t = np.arange(1000) / 1000.0
        log = synth(T=1.0)
        log["clipped"][(t >= .1) & (t < .2)] = 1
        log["clipped"][(t >= .5) & (t < .55)] = 1
        log["i_lim"][t >= .6] = 2.4
        log["i"][:] = 2.5                       # measured current above the new limit
        c = SM.current_report(log)
        self.assertEqual(c["sat_entries"], 2)
        self.assertAlmostEqual(c["sat_s"], 0.15, places=6)
        self.assertEqual(c["limit_changes"], [(0.6, 2.4)])
        self.assertAlmostEqual(c["measured_over_limit_s"], 0.4, places=6)

    def test_fault_latch_rearm_pairing(self):
        log = synth(events=[(1.0, "watchdog_trip"), (1.3, "rearm_after_watchdog_trip"), (2.0, "cmd_timeout")])
        f = SM.fault_timeline(log)
        self.assertEqual(f["rearms"], 1)
        self.assertAlmostEqual(f["latches"][0]["down_s"], 0.3)
        self.assertIsNone(f["latches"][1]["rearm_t"])

    def test_yaw_original_vs_delivered(self):
        base = RampedSine(R(75), 1.5, t_ramp=0.0)
        t = np.arange(4000) / 1000.0
        qy = 0.5 * np.array([base.eval(x)[0] for x in t])
        y = SM.yaw_report(synth(qy=qy, yaw_scale=[(0.0, 1.0), (0.5, 0.5)]), yaw_request=base)
        self.assertAlmostEqual(y["amp_ratio_end"], 0.5, places=3)
        self.assertAlmostEqual(y["delivered_freq"], 1.5, delta=0.02)
        self.assertEqual(y["n_scale_requests"], 1)

    def test_summarize_keys_unchanged(self):
        s = SM.summarize(perfect_tracker())
        self.assertEqual(set(s), {"rms", "peak", "mean", "i_peak", "i_rms", "clip_pct", "vlim_pct", "fallback_pct",
                                  "wd_trips", "heat_J", "cmd_age_p50", "cmd_age_max", "sat_entries", "rms_gov",
                                  "peak_gov", "speed", "rejected_pct", "lag_end", "yaw_scale", "events"})


class TestManifest(unittest.TestCase):
    def test_config_round_trip(self):
        cfg = SimConfig().with_(timing=dict(blackout=((3, 3.06),), burst_len_s=(0.01, 0.2)),
                                drive=dict(derate_schedule=((3, 2.4),)), plant=dict(m_payload=0.7))
        d = MF.jsonable(cfg)
        self.assertEqual(MF.config_from_dict(d), cfg)
        self.assertEqual(MF.config_from_dict(json.loads(MF.canonical(d))), cfg)

    def test_manifest_round_trip_and_stable_id(self):
        sc = M.m1(SimConfig())
        a = MF.make_manifest(sc, BaselineController(), seed=3)
        b = MF.make_manifest(sc, BaselineController(), seed=3)
        self.assertEqual(a["run_id"], b["run_id"])
        self.assertNotEqual(a["run_id"], MF.make_manifest(sc, BaselineController(), seed=4)["run_id"])
        self.assertNotEqual(a["run_id"], MF.make_manifest(sc, BaselineController(use_governor=False), seed=3)["run_id"])
        with tempfile.TemporaryDirectory() as d:
            MF.write_json(a, Path(d) / "m.json")
            self.assertEqual(MF.read_json(Path(d) / "m.json"), MF.jsonable(a))
        self.assertIn("ctrl/baseline.py", a["code"]["sources"])
        self.assertIn("sim/engine.py", a["code"]["sources"])
        self.assertEqual(a["run"]["controller"]["options"]["use_governor"], True)

    def test_source_change_changes_hash(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "ctrl" / "x.py"
            p.parent.mkdir()
            p.write_text("K = 1\n")
            h1 = MF.source_hashes([p], root=d)
            man = dict(code=dict(sources=h1))
            self.assertEqual(MF.changed_sources(man, root=d), [])
            p.write_text("K = 2\n")
            h2 = MF.source_hashes([p], root=d)
            self.assertNotEqual(MF.code_hash(h1), MF.code_hash(h2))
            self.assertEqual(MF.changed_sources(man, root=d), ["ctrl/x.py"])


class TestStagedPublish(unittest.TestCase):
    def _target(self, d):
        tgt = Path(d) / "pub"
        tgt.mkdir()
        (tgt / "table.md").write_text("old")
        return tgt

    def test_failure_leaves_target_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            tgt = self._target(d)
            with self.assertRaises(RuntimeError):
                with MF.staged_publish(tgt) as stage:
                    (stage / "table.md").write_text("partial")
                    (stage / "new.json").write_text("{}")
                    raise RuntimeError("evaluation failed mid-way")
            self.assertEqual((tgt / "table.md").read_text(), "old")
            self.assertFalse((tgt / "new.json").exists())
            stages = list(Path(d).glob(".stage-*"))
            self.assertEqual(len(stages), 1)
            self.assertTrue((stages[0] / "FAILED").exists())

    def test_failed_check_leaves_target_untouched(self):
        def check(stage):
            raise RuntimeError("source changed")
        with tempfile.TemporaryDirectory() as d:
            tgt = self._target(d)
            with self.assertRaises(RuntimeError):
                with MF.staged_publish(tgt, check=check) as stage:
                    (stage / "table.md").write_text("new")
            self.assertEqual((tgt / "table.md").read_text(), "old")

    def test_success_publishes_and_cleans_stage(self):
        with tempfile.TemporaryDirectory() as d:
            tgt = self._target(d)
            with MF.staged_publish(tgt) as stage:
                (stage / "table.md").write_text("new")
                (stage / "runs").mkdir()
                (stage / "runs" / "r.json").write_text("{}")
            self.assertEqual((tgt / "table.md").read_text(), "new")
            self.assertTrue((tgt / "runs" / "r.json").exists())
            self.assertEqual(list(Path(d).glob(".stage-*")), [])
            self.assertEqual(list(tgt.rglob(".*publishing")), [])


# ---------------------------------------------------------------------------
# Review round 1 regressions (written before the fixes; see report/packets/4A.md)
# ---------------------------------------------------------------------------
def _host_time(n):
    t = np.arange(n) / 1000.0
    return t, t - (np.arange(n) % 2) / 1000.0


def late_fault_suspended_periodic():
    """Perfect tracking of a periodic sine, then a watchdog trip, 55 ms drive
    fallback, re-arm and a 2B-style SUSPENDED hold for the last 4.4 % of the run."""
    ref = RampedSine(R(30), 1.0, t_ramp=0.5)
    t, th = _host_time(10000)
    tf = 9.56
    sigma = np.minimum(th, tf)
    q = np.array([ref.eval(s)[0] for s in sigma])
    mode = ((t >= tf) & (t < tf + 0.055)).astype(float)
    susp = (t >= tf).astype(float)
    log = synth(q=q, T=10.0, mode=mode, events=[(tf, "watchdog_trip"), (tf + 0.056, "rearm_after_watchdog_trip")],
                tele=dict(q_c=q, gov_lag=th - sigma, gov_sigma=sigma, gov_status=np.where(t >= tf, 3, 0),
                          request_rejected=0.0, host_fallback=mode, gov_s=1 - susp, gov_rejected=0.0,
                          suspended=susp))
    qr = np.array([ref.eval(x)[0] for x in t])
    log["q_ref"], log["err"] = qr, q - qr
    log.meta["notices"] = [(tf + 0.056, "suspended", "tracking fault (watchdog_trip): request suspended")]
    return log, ref


def late_fault_suspended_finite(tf=2.025):
    t, th = _host_time(4000)
    sigma = np.minimum(th, tf)
    q = np.array([PATH.eval(s)[0] for s in sigma])
    mode = ((t >= tf) & (t < tf + 0.055)).astype(float)
    susp = (t >= tf).astype(float)
    return synth(q=q, mode=mode, events=[(tf, "watchdog_trip"), (tf + 0.056, "rearm_after_watchdog_trip")],
                 tele=dict(q_c=q, gov_lag=th - sigma, gov_sigma=sigma, gov_status=np.where(t >= tf, 3, 0),
                           request_rejected=0.0, host_fallback=mode, gov_s=1 - susp, gov_rejected=0.0,
                           suspended=susp))


class TestReviewRound1(unittest.TestCase):
    # I1 -------------------------------------------------------------------
    def test_I1_late_tracking_fault_and_suspension_is_not_tracked(self):
        log, ref = late_fault_suspended_periodic()
        e = SM.evaluate(log, ref=ref)
        self.assertFalse(e["tracked"])
        why = " ".join(e["not_tracked_because"])
        self.assertIn("watchdog_trip", why)
        self.assertIn("suspended", why)

    def test_I1_suspended_hold_does_not_complete(self):
        e = ev(late_fault_suspended_finite())
        self.assertFalse(e["completion"]["completed"])
        self.assertFalse(e["tracked"])

    def test_I1_waypoint_visit_must_be_healthy(self):
        t = np.arange(4000) / 1000.0
        q = np.array([PATH.eval(x)[0] for x in t])
        mode = ((t > 1.0) & (t < 1.8)).astype(float)      # -45 deg (1.3-1.6 s dwell) only in drive fallback
        self.assertFalse(SM.completion(synth(q=q, mode=mode), WAYPOINTS, T_REQ)["completed"])

    def test_I1_M6_rejected_status_and_coord_stop_block_completion(self):
        t = np.arange(4000) / 1000.0
        q = np.array([PATH.eval(x)[0] for x in t])
        base = dict(q_c=q, gov_lag=0.0, request_rejected=0.0, host_fallback=0.0, gov_s=1.0, gov_rejected=0.0)
        rej = synth(q=q, tele=dict(base, gov_status=np.where(t > 2.2, 5, 0)))
        self.assertFalse(SM.completion(rej, WAYPOINTS, T_REQ)["completed"])
        stop = synth(q=q, tele=dict(base, gov_status=0, coord_stop=(t > 2.2).astype(float)))
        self.assertFalse(SM.completion(stop, WAYPOINTS, T_REQ)["completed"])
        self.assertFalse(ev(stop)["tracked"])

    def test_I1_settle_must_persist_to_end_of_run(self):
        t = np.arange(4000) / 1000.0
        q = np.array([PATH.eval(x)[0] for x in t])
        q[t > 3.0] = R(20)                                  # leaves the final band after settling
        self.assertFalse(SM.completion(synth(q=q), WAYPOINTS, T_REQ)["completed"])

    def test_I1_brief_velocity_blip_after_arrival_does_not_move_t_complete(self):
        t = np.arange(4000) / 1000.0
        q = np.array([PATH.eval(x)[0] for x in t])
        log = synth(q=q)
        t0 = SM.completion(log, WAYPOINTS, T_REQ)["t_complete"]
        log["qd"][3300:3303] = 0.2                           # 3 ms disturbance, position still in band
        self.assertEqual(SM.completion(log, WAYPOINTS, T_REQ)["t_complete"], t0)

    # I2 -------------------------------------------------------------------
    def test_I2_suspended_stratum_timeline_and_replan_recovery(self):
        log, ref = late_fault_suspended_periodic()
        st = SM.stratified(log, ref=ref)
        self.assertAlmostEqual(sum(v["share_pct"] for v in st.values()), 100.0, places=9)
        self.assertAlmostEqual(st["suspended"]["time_s"], 10.0 - 9.56 - 0.055, delta=0.002)
        self.assertAlmostEqual(st["fallback"]["time_s"], 0.055, delta=0.002)
        f = SM.fault_timeline(log)
        self.assertEqual(f["suspended"]["count"], 1)
        self.assertIn("watchdog_trip", f["suspended"]["reasons"][0])
        # replan at 5 s (counter increments) opens a recovery window
        t = log.t
        log2 = synth(T=10.0, tele=dict(q_c=0.0, gov_lag=0.0, gov_status=0, host_fallback=0.0, request_rejected=0.0,
                                       replans=(t >= 5.0).astype(float)))
        lab = SM.strata(log2)
        self.assertTrue(np.all(lab[(t >= 5.0) & (t < 5.0 + SM.RECOVERY_S)] == SM.STRATA.index("recovery")))
        self.assertTrue(np.all(lab[(t > 1.0) & (t < 5.0)] == SM.STRATA.index("normal")))

    # I3 -------------------------------------------------------------------
    def test_I3_progress_is_net_and_backsteps_fail_tracking(self):
        ref = RampedSine(R(30), 1.0, t_ramp=1.0)
        t, th = _host_time(10000)
        hk = (np.arange(10000) // 2) % 25
        sigma = np.minimum(hk, 24) * 0.002                  # saw-tooth 0..48 ms, stationary plant
        log = synth(T=10.0, tele=dict(q_c=0.0, gov_lag=th - sigma, gov_status=0, request_rejected=0,
                                      host_fallback=0, gov_s=1.0, gov_rejected=0))
        qr = np.array([ref.eval(x)[0] for x in t])
        log["q_ref"], log["err"] = qr, -qr
        e = SM.evaluate(log, ref=ref)
        self.assertLess(e["progress"]["progress"], 0.01)
        self.assertFalse(e["tracked"])
        alt = synth(T=10.0, tele=dict(q_c=0.0, gov_lag=th - (hk % 2) * 0.002, host_fallback=0))
        p = SM.progress(alt, 2.0)["progress"]
        self.assertLessEqual(p, 1.0)
        self.assertLess(p, 0.01)

    # I4 -------------------------------------------------------------------
    def test_I4_explicit_gov_sigma_preferred_and_gov_active_zero_means_wall_clock(self):
        t, th = _host_time(4000)
        q = np.array([PATH.eval(x)[0] for x in t])
        tl = dict(q_c=q, gov_status=0, host_fallback=0.0, request_rejected=0.0, gov_s=1.0, gov_rejected=0.0)
        # telemetry lag is garbage, explicit sigma is the truth
        log = synth(q=q, tele=dict(tl, gov_lag=123.0, gov_sigma=th))
        np.testing.assert_allclose(SM.path_clock(log), th, atol=1e-12)
        # ungoverned controller still emitting a frozen governor lag (probe_ungov)
        ung = synth(q=q, tele=dict(tl, gov_lag=th, gov_active=0.0))
        np.testing.assert_allclose(SM.path_clock(ung), t, atol=1e-12)
        e = ev(ung)
        self.assertTrue(e["tracked"], e["not_tracked_because"])
        # without the declaration the frozen lag is (conservatively) believed ...
        legacy = synth(q=q, tele=dict(tl, gov_lag=th))
        self.assertLess(ev(legacy)["progress"]["progress"], 0.1)
        # ... unless the caller declares the run ungoverned
        self.assertTrue(SM.evaluate(legacy, ref=PATH, T_request=T_REQ, waypoints=WAYPOINTS, governed=False)["tracked"])

    # I5 -------------------------------------------------------------------
    def test_I5_run_id_independent_of_import_history(self):
        import types
        from unittest import mock
        sc = M.m1(SimConfig())
        a = MF.make_manifest(sc, BaselineController(), seed=1)
        fake = types.ModuleType("fake_unrelated")
        fake.__file__ = str(MF.ROOT / "exp" / "task2_robustness.py")
        with mock.patch.dict(sys.modules, {"fake_unrelated": fake}):
            b = MF.make_manifest(sc, BaselineController(), seed=1)
        self.assertEqual(a["run_id"], b["run_id"])
        self.assertEqual(a["code"]["sources"], b["code"]["sources"])
        for rel in ("ctrl/governor.py", "sim/engine.py", "sim/metrics.py", "exp/common.py", "exp/manifest.py"):
            self.assertIn(rel, a["code"]["sources"])

    # Minor ----------------------------------------------------------------
    def test_M1_sub_object_options_change_run_id(self):
        sc = M.m1(SimConfig())
        c = BaselineController()
        c.gov.v_max *= 0.5
        self.assertNotEqual(MF.make_manifest(sc, BaselineController(), 1)["run_id"], MF.make_manifest(sc, c, 1)["run_id"])

    def test_M2_nonfinite_config_round_trips_and_note_not_in_run_id(self):
        cfg = SimConfig().with_(drive=dict(cmd_timeout=float("inf")), sensor=dict(enc_offset=float("nan")))
        back = MF.config_from_dict(json.loads(MF.canonical(MF.jsonable(cfg, tag_nonfinite=True))))
        self.assertEqual(back.drive.cmd_timeout, float("inf"))
        self.assertTrue(math.isnan(back.sensor.enc_offset))
        sc = M.m1(SimConfig())
        self.assertEqual(MF.make_manifest(sc, BaselineController(), 1)["run_id"],
                         MF.make_manifest(replace(sc, note="reworded"), BaselineController(), 1)["run_id"])

    def test_M3_failure_in_rename_phase_rolls_back(self):
        import os as _os
        from unittest import mock
        with tempfile.TemporaryDirectory() as d:
            tgt = Path(d) / "pub"
            tgt.mkdir()
            for n in ("a.md", "b.md"):
                (tgt / n).write_text("old")
            real, calls = _os.replace, []

            def flaky(a, b):
                calls.append(b)
                if len(calls) == 2:
                    raise OSError("disk full")
                return real(a, b)
            with self.assertRaises(OSError):
                with mock.patch.object(MF.os, "replace", flaky):
                    with MF.staged_publish(tgt) as st:
                        (st / "a.md").write_text("new")
                        (st / "b.md").write_text("new")
            self.assertEqual({p.name: p.read_text() for p in tgt.iterdir()}, {"a.md": "old", "b.md": "old"})
            self.assertTrue((next(Path(d).glob(".stage-*")) / "FAILED").exists())

    def test_M4_run_rebuilds_from_record_alone(self):
        from ctrl.supervisor import DriveSupervisor
        for sc, ctrl in ((M.m1(SimConfig()), BaselineController(use_governor=False)),
                         (M.m3(SimConfig()), StationaryDiagnostic())):
            sup = DriveSupervisor(rearm_time=0.2)
            a = MF.make_manifest(sc, ctrl, 5, supervisor=sup, governed_yaw=False)
            rec = json.loads(MF.canonical(MF.jsonable(a)))
            sc2, c2, s2, gy, seed = MF.rebuild_run(rec["run"])
            b = MF.make_manifest(sc2, c2, seed, supervisor=s2, governed_yaw=gy)
            self.assertEqual(a["run_id"], b["run_id"])
            self.assertEqual(sc2.roll.eval(1.0), sc.roll.eval(1.0))
            self.assertEqual(s2.rearm_time, 0.2)

    def test_M5_rejection_at_startup_is_rejected_not_startup(self):
        t, th = _host_time(4000)
        sigma = np.maximum(th - 0.15, 0.0)
        q = np.array([PATH.eval(s)[0] for s in sigma])
        rej = (t < 0.15).astype(float)
        log = synth(q=q, tele=dict(q_c=q, gov_lag=th - sigma, gov_status=np.where(t < 0.15, 5, 0),
                                   request_rejected=rej, host_fallback=rej, gov_s=1 - rej, gov_rejected=rej))
        e = ev(log)
        self.assertAlmostEqual(e["strata"]["rejected"]["time_s"], 0.15, delta=0.003)
        self.assertFalse(e["tracked"])

    def test_M7_slack_does_not_dilute_finite_path_error(self):
        t = np.arange(6400) / 1000.0
        q = np.array([PATH.eval(max(x - 0.045, 0))[0] for x in t])       # 45 ms late throughout
        log = synth(q=q, T=6.4, tele=dict(q_c=q, gov_lag=0.0, gov_status=0, request_rejected=0, host_fallback=0,
                                          gov_s=1.0, gov_rejected=0))
        e = ev(log)
        self.assertGreater(e["errors"]["path_rms_request"], 5.0)
        self.assertFalse(e["tracked"])


# ---------------------------------------------------------------------------
# Review round 2 (delta) regressions
# ---------------------------------------------------------------------------
def settled_with_fault(t0, t1, events, suspended=False):
    """Perfect M1 tracker; drive + host fallback over [t0, t1) after arrival."""
    t = np.arange(4000) / 1000.0
    q = np.array([PATH.eval(x)[0] for x in t])
    fb = ((t >= t0) & (t < t1)).astype(float)
    tele = dict(q_c=q, gov_lag=0.0, gov_status=0, request_rejected=0.0, host_fallback=fb, gov_s=1.0,
                gov_rejected=0.0)
    if suspended:
        tele["suspended"] = (t >= t1).astype(float)
    return synth(q=q, mode=fb, events=events, tele=tele)


class TestReviewRound2(unittest.TestCase):
    def test_N1_transient_comm_fault_after_arrival_keeps_t_complete(self):
        t_ref = SM.completion(settled_with_fault(9, 9, []), WAYPOINTS, T_REQ)["t_complete"]
        for t0 in (3.0, 3.9):                  # mid-hold, and one ending with the run
            ev_ = [(t0 + .009, "cmd_timeout"), (t0 + .1, "rearm_after_cmd_timeout")]
            c = SM.completion(settled_with_fault(t0, min(t0 + .1, 4.0), ev_), WAYPOINTS, T_REQ)
            self.assertTrue(c["completed"], t0)
            self.assertEqual(c["t_complete"], t_ref)
            self.assertEqual([e["name"] for e in c["post_arrival_transient"]["events"]], ["cmd_timeout"])
            self.assertGreater(c["post_arrival_transient"]["fallback_s"], 0.05)

    def test_N1_latched_fault_suspension_or_leaving_band_after_arrival_still_void(self):
        wd = [(3.0, "watchdog_trip"), (3.06, "rearm_after_watchdog_trip")]
        self.assertFalse(SM.completion(settled_with_fault(3.0, 3.06, wd), WAYPOINTS, T_REQ)["completed"])
        tr = [(3.009, "cmd_timeout"), (3.1, "rearm_after_cmd_timeout")]
        self.assertFalse(SM.completion(settled_with_fault(3.0, 3.1, tr, suspended=True), WAYPOINTS,
                                       T_REQ)["completed"])
        log = settled_with_fault(3.0, 3.1, tr)
        log["q"][(log.t >= 3.02) & (log.t < 3.05)] = R(10)     # leaves the band during the outage
        c = SM.completion(log, WAYPOINTS, T_REQ)
        self.assertGreaterEqual(c["t_complete"], 3.1)            # the earlier arrival no longer counts
        wd_c = SM.completion(settled_with_fault(3.0, 3.06, wd), WAYPOINTS, T_REQ)
        self.assertAlmostEqual(wd_c["voided_at"], 3.0)

    def test_N1_real_run_command_blackout_after_arrival(self):
        sc = M.m1(SimConfig())
        base, _ = run(sc, BaselineController, seed=1)
        bl, _ = run(replace(sc, cfg=sc.cfg.with_(timing=dict(command_blackout=((4.0, 4.06),)))),
                    BaselineController, seed=1)
        a = SM.completion(base, list(sc.waypoints), sc.t_request)
        b = SM.completion(bl, list(sc.waypoints), sc.t_request)
        self.assertTrue(b["completed"])
        self.assertAlmostEqual(b["t_complete"], a["t_complete"], delta=0.01)

    def test_N2_load_model_controller_rebuilds_or_says_why(self):
        from ctrl.yaw_estimator import KinematicEstimator
        sc = M.m1(SimConfig())
        a = MF.make_manifest(sc, BaselineController(load_model=KinematicEstimator()), 1)
        rec = json.loads(MF.canonical(MF.jsonable(a)))["run"]
        try:
            sc2, c2, s2, gy, seed = MF.rebuild_run(rec)
        except MF.NotRebuildable as e:
            self.assertIn("load_model", str(e))
        else:
            self.assertEqual(MF.make_manifest(sc2, c2, seed, supervisor=s2, governed_yaw=gy)["run_id"], a["run_id"])

    def test_N3_orphan_prev_with_missing_destination_is_restored(self):
        with tempfile.TemporaryDirectory() as d:
            tgt = Path(d) / "pub"
            tgt.mkdir()
            (tgt / ".old.md.prev").write_text("only copy")
            (tgt / "x.md").write_text("old x")
            (tgt / ".x.md.prev").write_text("older x")           # destination exists and is replaced
            with MF.staged_publish(tgt) as st:
                (st / "new.md").write_text("new")
                (st / "x.md").write_text("new x")
            names = sorted(p.name for p in tgt.iterdir())
            self.assertEqual((tgt / "old.md").read_text(), "only copy")
            self.assertEqual(names, ["new.md", "old.md", "x.md"])

    def test_N4_undeclared_imports_can_be_required_empty(self):
        import types
        from unittest import mock
        sc = M.m1(SimConfig())
        # The imports made by other tests in this process are not this test's
        # subject: pin the loaded set to the declared set, then add one module.
        declared = MF.declared_sources("exp.metrics4a")
        undeclared = (MF.ROOT / "exp" / "task2_robustness.py").resolve()
        with mock.patch.object(MF, "loaded_sources", lambda root=MF.ROOT: list(declared)):
            clean = MF.make_manifest(sc, BaselineController(), 1, entry="exp.metrics4a")
        MF.require_declared(clean)                                # no exception
        with mock.patch.object(MF, "loaded_sources", lambda root=MF.ROOT: list(declared) + [undeclared]):
            dirty = MF.make_manifest(sc, BaselineController(), 1, entry="exp.metrics4a")
            with self.assertRaises(MF.UndeclaredSources):
                MF.make_manifest(sc, BaselineController(), 1, entry="exp.metrics4a", require_declared=True)
        self.assertEqual(clean["run_id"], dirty["run_id"])
        with self.assertRaises(MF.UndeclaredSources):
            MF.require_declared(dirty)


if __name__ == "__main__":
    unittest.main()
