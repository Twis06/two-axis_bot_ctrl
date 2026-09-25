"""Prospective Task 5 packet: scoring and registration contracts on synthetic inputs only.
None of these tests simulates the registered condition."""
import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from exp import task5_prospective as T

W = 2 * math.pi * 3.0
t = np.arange(0.0, 12.0, 1e-3)


class TestPhasor(unittest.TestCase):
    def test_known_transfer(self):
        H = T.transfer(t, 2 * np.sin(W * t + math.pi / 6), np.sin(W * t), 3.0, [4.0, 12.0])
        self.assertAlmostEqual(H.real, math.sqrt(3), places=9)
        self.assertAlmostEqual(H.imag, 1.0, places=9)

    def test_lag_gives_negative_phase_and_offset_is_ignored(self):
        H = T.transfer(t, 0.5 + np.sin(W * t - 0.3), np.sin(W * t), 3.0, [4.0, 12.0])
        self.assertAlmostEqual(math.atan2(H.imag, H.real), -0.3, places=9)

    def test_window_boundaries_are_exact(self):
        x = np.sin(W * t)
        x[t < 4.0] = 99.0                      # garbage strictly before the window
        x[t >= 12.0] = 99.0
        H = T.transfer(t, x, np.sin(W * t), 3.0, [4.0, 12.0])
        self.assertAlmostEqual(abs(H - 1), 0.0, places=9)
        y = np.sin(W * t).copy()
        y[np.argmin(np.abs(t - 4.0))] = 99.0   # the first in-window sample matters
        self.assertGreater(abs(T.transfer(t, y, np.sin(W * t), 3.0, [4.0, 12.0]) - 1), 1e-6)

    def test_invalid_data_is_none_not_zero(self):
        x = np.sin(W * t); x[5000] = np.nan
        self.assertIsNone(T.transfer(t, x, np.sin(W * t), 3.0, [4.0, 12.0]))
        self.assertIsNone(T.transfer(t, np.sin(W * t), 1e-6 * np.sin(W * t), 3.0, [4.0, 12.0]))
        self.assertIsNone(T.transfer(t, np.sin(W * t), np.sin(W * t), 3.0, [20.0, 30.0]))


def row(seed, d, H=1.0 + 0j, **kw):
    r = dict(seed=seed, cmd_delay_s=d, run_id=f"{seed}-{d}", valid=True,
             H=dict(re=H.real, im=H.imag, gain=abs(H), phase_deg=0.0), events=[], suspended=False,
             rejected=False, fallback_pct=0.0)
    r.update(kw)
    return r


REG = dict(prediction=dict(acceptance=dict(center=dict(re=0.07, im=-0.02), radius=0.02)))


def grid(dH=0.07 - 0.02j):
    out = []
    for s in T.PROTOCOL["seeds"]:
        out += [row(s, T.PROTOCOL["cmd_delay_s"][0], 1.0 + 0j), row(s, T.PROTOCOL["cmd_delay_s"][1], 1.0 + dH)]
    return out


class TestOutcome(unittest.TestCase):
    def test_inside_is_supported_outside_is_contradicted(self):
        self.assertEqual(T.outcome(grid(), REG)["label"], "supported")
        self.assertEqual(T.outcome(grid(0.0j), REG)["label"], "contradicted")
        self.assertEqual(T.outcome(grid(0.07 + 0.03j), REG)["label"], "contradicted")

    def test_missing_duplicate_or_unregistered_cells_are_inconclusive(self):
        rows = grid()
        self.assertEqual(T.outcome(rows[:-1], REG)["label"], "inconclusive")
        self.assertEqual(T.outcome(rows + [rows[0]], REG)["label"], "inconclusive")
        self.assertEqual(T.outcome(rows + [row(999, 1e-3)], REG)["label"], "inconclusive")

    def test_invalid_or_faulted_runs_are_inconclusive(self):
        for kw in (dict(valid=False, H=None), dict(events=["watchdog_trip"]), dict(suspended=True),
                   dict(rejected=True), dict(fallback_pct=0.5)):
            rows = grid()
            rows[3].update(kw)
            self.assertEqual(T.outcome(rows, REG)["label"], "inconclusive", kw)


class _Log:
    events = []

    def __init__(self, mode):
        self.t = t
        self.q = np.sin(W * t); self.c_q_c = np.sin(W * t); self.q_ref = np.sin(W * t)
        self.c_gov_limited = np.zeros_like(t); self.clipped = np.zeros(t.size, bool); self.i = 0.5 * np.ones_like(t)
        self.c_suspended = np.zeros_like(t); self.c_request_rejected = np.zeros_like(t); self.mode = mode


class TestStartupFallback(unittest.TestCase):
    """Pre-run review B1: the drive's start-up fallback ticks are not a fault."""

    def test_startup_fallback_is_not_counted(self):
        mode = np.zeros(t.size, int); mode[:3] = 1                 # first ticks before the first command
        self.assertEqual(T.score_run(_Log(mode), 301, 1e-3, "x")["fallback_pct"], 0.0)

    def test_fallback_inside_the_window_is_counted(self):
        mode = np.zeros(t.size, int); mode[6000:6060] = 1          # 60 ms outage inside [4, 12) s
        r = T.score_run(_Log(mode), 301, 1e-3, "x")
        self.assertGreater(r["fallback_pct"], 0.0)
        rows = grid(); rows[0].update(fallback_pct=r["fallback_pct"])
        self.assertEqual(T.outcome(rows, REG)["label"], "inconclusive")


class TestRegistration(unittest.TestCase):
    def test_registration_is_self_consistent_and_discriminating(self):
        p = T.prediction()
        self.assertFalse(p["acceptance"]["includes_zero"])
        self.assertGreater(p["dH"]["mag"], p["acceptance"]["radius"])

    def test_altered_registration_is_refused(self):
        reg = T.registration()
        for mutate in (lambda r: r["protocol"].__setitem__("seeds", [301, 302, 303, 304, 306]),
                       lambda r: r["protocol"].__setitem__("window_s", [3.0, 12.0]),
                       lambda r: r["protocol"].__setitem__("freq_hz", 2.9),
                       lambda r: r["prediction"]["dH"].__setitem__("re", r["prediction"]["dH"]["re"] + 1e-6),
                       lambda r: r["prediction"]["acceptance"].__setitem__("radius", 0.5),
                       lambda r: r.__setitem__("baseline_fingerprint", "0" * 64),
                       lambda r: r.__setitem__("scorer_sha256", "0" * 64)):
            bad = copy.deepcopy(reg)
            mutate(bad)
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "reg.json"
                p.write_text(json.dumps(bad))
                with mock.patch.object(T, "REG_PATH", p), self.assertRaises(SystemExit):
                    T.check_registration()

    def test_missing_registration_is_refused(self):
        with mock.patch.object(T, "REG_PATH", Path("/nonexistent/reg.json")), self.assertRaises(SystemExit):
            T.check_registration()


if __name__ == "__main__":
    unittest.main()
