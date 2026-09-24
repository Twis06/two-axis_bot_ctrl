"""R1: calibration cannot earn finite test-motion credit."""
import math
import unittest
from dataclasses import replace

import numpy as np

from tests.test_metrics import synth
from sim import metrics as SM
from exp import manifest as MF
from exp.task3_l4 import scenario
from exp.motions import FiniteMotion
from ctrl.baseline import BaselineController

R = math.radians
WP = [R(30), R(-30), R(65), R(-65), 0.0]
WINDOWS = ((9., 11.), (11., 13.), (13., 15.), (15., 17.), (17., None))


def trace(delay=0., omit=()):
    t = np.arange(26000) / 1000
    sig = np.maximum(0., t - delay)
    q = np.zeros(len(t))
    q[(sig >= 5) & (sig < 6)] = WP[0]  # calibration-only +30 crossing
    for k, (a, b) in enumerate(WINDOWS):
        if k not in omit:
            q[(sig >= a + .5) & (sig < (b if b is not None else 100))] = WP[k]
    return synth(q=q, T=26., tele=dict(gov_sigma=sig))


def score(log):
    return SM.completion(log, WP, 18., waypoint_windows=WINDOWS)


class TestPhaseCompletion(unittest.TestCase):
    def test_calibration_only_crossing_is_not_test_completion(self):
        log = trace(omit=(0,))
        self.assertTrue(SM.completion(log, WP, 18.)['completed'])
        c = score(log)
        self.assertFalse(c['completed'])
        self.assertEqual(c['missing_waypoints'], [0])
        self.assertIsNone(c['waypoint_visits'][0]['wall_t'])

    def test_delayed_calibration_excluded_and_elapsed_budget_retained(self):
        self.assertFalse(score(trace(delay=6., omit=(0,)))['completed'])
        c = score(trace(delay=6.))
        self.assertTrue(c['completed'])
        self.assertAlmostEqual(c['waypoint_visits'][0]['wall_t'], 15.5)
        self.assertAlmostEqual(c['waypoint_visits'][0]['path_t'], 9.5)
        self.assertGreater(c['t_complete'], 23.)
        self.assertAlmostEqual(c['time_ratio'], 18. / c['t_complete'])

    def test_later_crossing_cannot_replace_missing_intended_visit(self):
        log = trace(omit=(0,))
        log.q[(log.t >= 12) & (log.t < 12.2)] = WP[0]
        self.assertFalse(score(log)['completed'])

    def test_reordered_and_omitted_targets_fail(self):
        log = trace()
        log.q[(log.t >= 9) & (log.t < 11)] = WP[1]
        log.q[(log.t >= 11) & (log.t < 13)] = WP[0]
        self.assertFalse(score(log)['completed'])
        self.assertEqual(score(trace(omit=(2,)))['missing_waypoints'], [2])

    def test_valid_visits_include_final_target(self):
        c = score(trace())
        self.assertTrue(c['completed'])
        self.assertEqual(c['missing_waypoints'], [])
        self.assertEqual(len(c['waypoint_visits']), 5)
        self.assertEqual(c['reached'], 5)
        self.assertGreaterEqual(c['waypoint_visits'][-1]['path_t'], 17.)

    def test_final_settles_after_path_end_using_slack(self):
        log = trace()
        log.q[(log.t >= 17) & (log.t < 20)] = R(10)
        log.qd[:] = np.gradient(log.q, log.t)
        c = score(log)
        self.assertTrue(c['completed'])
        self.assertGreater(c['t_complete'], 19.)

    def test_post_arrival_hard_status_voids_completion(self):
        for field in ('c_suspended', 'c_request_rejected', 'c_coord_stop'):
            log = trace()
            log[field] = (log.t >= 22).astype(float)
            self.assertFalse(score(log)['completed'], field)

    def test_post_arrival_latched_fault_voids_completion(self):
        log = trace()
        log.events = [(22., 'watchdog')]
        self.assertFalse(score(log)['completed'])

    def test_post_arrival_transient_excused_only_while_holding(self):
        log = trace()
        log.mode[(log.t >= 22) & (log.t < 22.06)] = 1
        log.events = [(22., 'cmd_timeout')]
        self.assertTrue(score(log)['completed'])
        log.q[log.t >= 22] = R(10)
        self.assertFalse(score(log)['completed'])

    def test_explicit_ungoverned_override_uses_wall_time(self):
        log = trace()
        log.c_gov_sigma[:] = 5.
        c = SM.evaluate(log, T_request=18., waypoints=WP,
                        waypoint_windows=WINDOWS, governed=False)['completion']
        self.assertTrue(c['completed'])

    def test_invalid_protocols_rejected(self):
        bad = (WINDOWS[:-1], ((float('nan'), 11.),) + WINDOWS[1:],
               ((11., 9.),) + WINDOWS[1:], ((9., 12.),) + WINDOWS[1:],
               WINDOWS[:-1] + ((17., 19.),))
        for windows in bad:
            with self.subTest(windows=windows), self.assertRaises(ValueError):
                SM.completion(trace(), WP, 18., waypoint_windows=windows)

    def test_frozen_calibration_clock_cannot_credit_wall_time_test(self):
        log = trace()
        log.c_gov_sigma[:] = 5.
        c = score(log)
        self.assertFalse(c['completed'])
        self.assertEqual(c['missing_waypoints'], [0, 1, 2, 3, 4])

    def test_recovered_prearrival_fault_does_not_void_completion(self):
        log = trace()
        log.events = [(2., 'watchdog'), (2.1, 'rearm_after_watchdog')]
        self.assertTrue(score(log)['completed'])
        self.assertEqual(len(SM.fault_timeline(log)['tracking_faults']), 1)

    def test_protocol_record_round_trip_and_identity(self):
        sc, wp = scenario((0., .1), 3.2, 'test')
        fm = FiniteMotion(sc.name, sc.cfg, sc.roll, sc.yaw, sc.note, wp, 18., WINDOWS)
        m = MF.make_manifest(fm, BaselineController(), 101)
        rebuilt, ctrl, sup, yaw, seed = MF.rebuild_run(m['run'])
        again = MF.make_manifest(rebuilt, ctrl, seed, supervisor=sup, governed_yaw=yaw)
        self.assertEqual(again['run_id'], m['run_id'])
        self.assertEqual(tuple(map(tuple, rebuilt.waypoint_windows)), WINDOWS)
        changed = MF.make_manifest(replace(fm, waypoint_windows=((8., 11.),) + WINDOWS[1:]), BaselineController(), 101)
        self.assertNotEqual(m['run_id'], changed['run_id'])


if __name__ == '__main__':
    unittest.main()
