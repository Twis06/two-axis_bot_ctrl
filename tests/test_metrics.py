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


if __name__ == "__main__":
    unittest.main()
