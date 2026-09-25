"""R3: the Task 3 adoption gate is complete and cannot be passed by hiding a failure."""
import copy
import unittest

from exp.task3_gate import REQUIRED_CHALLENGES, adoption_gate


def row(variant, seed, primary, **kw):
    r = dict(variant=variant, load=[0.0, 0.1], limit=3.2, seed=seed, run_id=f"{variant}{seed}",
             primary_deg=primary, completed=True, missing_waypoints=[], progress=1.0, wd_trips=0,
             suspended=False, rejected=False, lockout=False, command_over_limit=-0.1, t_complete=18.5,
             learn_usable_max=1.0, applied_while_unusable=0, events=[], event_counts={})
    r.update(kw)
    return r


N = 10
KEYS = [((0.0, 0.1), 3.2, s) for s in range(N)]


def good_heldout(n=N):
    return ([row("int1", s, 2.0) for s in range(n)] +
            [row("adaptive", s, 1.0) for s in range(n)])


CH_KEYS = [(g, [0.0, 0.1], 3.2, s) for g in REQUIRED_CHALLENGES for s in range(2)]


def gate(rows, ch="good", **kw):
    ch = good_challenges() if ch == "good" else ch
    kw.setdefault("expected_keys", KEYS)
    kw.setdefault("expected_challenge_keys", CH_KEYS if ch is not None else None)
    return adoption_gate(rows, "int1", "adaptive", ch, **kw)


def good_challenges():
    out = []
    for g in REQUIRED_CHALLENGES:
        for s in range(2):
            out.append(dict(row("int1", s, 2.0), challenge=g))
            out.append(dict(row("adaptive", s, 1.0), challenge=g, learn_usable_during_challenge=0.9))
    return out


def status(gate, prefix):
    return next(c["status"] for c in gate["criteria"] if c["name"].startswith(prefix))


class TestAdoptionGate(unittest.TestCase):
    def test_a_clean_candidate_passes_every_criterion(self):
        g = gate(good_heldout())
        self.assertEqual(g["overall"], "pass", [c for c in g["criteria"] if c["status"] != "pass"])

    def test_hidden_lost_waypoint_fails_even_if_completion_flags_agree(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, missing_waypoints=[2])    # still flagged completed
        g = gate(rows)
        self.assertEqual(status(g, "no individual comparator waypoint"), "fail")
        self.assertEqual(g["overall"], "fail")

    def test_single_severe_regression_is_not_masked_by_the_median(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 3.0)                            # +50 % in one case
        g = gate(rows)
        self.assertEqual(status(g, "median paired"), "pass")          # the median still looks good
        self.assertEqual(status(g, "no ordinary feasible case"), "fail")
        self.assertEqual(g["overall"], "fail")

    def test_unavailable_metrics_are_incomplete_not_pass(self):
        rows = good_heldout()
        for r in rows:
            r.pop("applied_while_unusable")
            r.pop("command_over_limit")
        g = gate(rows)
        self.assertEqual(status(g, "host current command"), "incomplete")
        self.assertEqual(status(g, "no learned correction applied"), "incomplete")
        self.assertEqual(g["overall"], "incomplete")

    def test_target_limit_violation_fails(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, command_over_limit=0.01)
        self.assertEqual(gate(rows)["overall"], "fail")

    def test_missing_or_unexercised_challenges_are_incomplete(self):
        self.assertEqual(gate(good_heldout(), ch=None)["overall"], "incomplete")
        partial = [r for r in good_challenges() if r["challenge"] != "payload_change"]
        self.assertEqual(gate(good_heldout(), ch=partial)["overall"], "incomplete")
        idle = copy.deepcopy(good_challenges())
        for r in idle:
            r["learn_usable_during_challenge"] = 0.0
        self.assertEqual(gate(good_heldout(), ch=idle)["overall"], "incomplete")

    def test_a_new_fault_in_a_challenge_fails(self):
        ch = good_challenges()
        for r in ch:
            if r["variant"] == "adaptive" and r["challenge"] == "derate":
                r["wd_trips"] = 1
        self.assertEqual(gate(good_heldout(), ch=ch)["overall"], "fail")

    def test_benefit_from_slower_motion_is_not_credited(self):
        rows = good_heldout()
        for r in rows:
            if r["variant"] == "adaptive":
                r["progress"] = 0.9
        g = gate(rows)
        self.assertEqual(status(g, "no benefit credited"), "fail")

    def test_insufficient_benefit_and_availability_fail(self):
        rows = [row("int1", s, 2.0) for s in range(10)] + \
               [row("adaptive", s, 1.95, learn_usable_max=1.0 if s < 7 else 0.0) for s in range(10)]
        g = gate(rows)
        self.assertEqual(status(g, "median paired"), "fail")
        self.assertEqual(status(g, ">= 80 %"), "fail")


class TestReviewR3Adversarial(unittest.TestCase):
    """R3 review cases A-H: none may pass."""

    def test_A_missing_registered_cells_are_incomplete(self):
        rows = [r for r in good_heldout() if r["seed"] == 0]       # one pair only
        self.assertEqual(gate(rows)["overall"], "incomplete")
        ch = [r for r in good_challenges() if r["seed"] == 0]
        self.assertEqual(gate(good_heldout(), ch=ch)["overall"], "incomplete")
        self.assertEqual(adoption_gate(good_heldout(), "int1", "adaptive", good_challenges())["overall"],
                         "incomplete")                            # grid not supplied

    def test_E_duplicate_rows_are_incomplete(self):
        rows = good_heldout() + [row("adaptive", 0, 1.0)]
        self.assertEqual(gate(rows)["overall"], "incomplete")

    def test_B_nan_candidate_primary_is_not_dropped(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, float("nan"))
        self.assertNotEqual(gate(rows)["overall"], "pass")

    def test_C_G_regression_where_the_comparator_did_not_complete_still_counts(self):
        rows = [row("int1", s, 2.0, completed=False) for s in range(N)] + \
               [row("adaptive", s, 1.0 if s else 10.0, completed=False) for s in range(N)]
        g = gate(rows)
        self.assertEqual(status(g, "no ordinary feasible case"), "fail")

    def test_D_lockout_field_is_read(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, lockout=True, events=["a", "b", "c", "d", "e", "f"])
        self.assertEqual(gate(rows)["overall"], "fail")

    def test_F_slower_completion_is_not_credited(self):
        rows = good_heldout()
        for r in rows:
            if r["variant"] == "adaptive":
                r["t_complete"] = 21.0
        self.assertEqual(status(gate(rows), "no benefit credited"), "fail")

    def test_H_missing_learning_telemetry_is_incomplete(self):
        rows = good_heldout()
        for r in rows:
            if r["variant"] == "adaptive":
                r["applied_while_unusable"] = None
        self.assertEqual(status(gate(rows), "no learned correction applied"), "incomplete")

    def test_late_amendment_rows_count_towards_exercise(self):
        ch = good_challenges()
        for r in ch:
            if r["challenge"] == "yaw_B":
                r["learn_usable_during_challenge"] = 0.0
        late = [dict(r, challenge="yaw_B@late", learn_usable_during_challenge=0.8)
                for r in good_challenges() if r["challenge"] == "yaw_B"]
        keys = CH_KEYS + [("yaw_B@late", [0.0, 0.1], 3.2, s) for s in range(2)]
        self.assertEqual(gate(good_heldout(), ch=ch + late, expected_challenge_keys=keys)["overall"], "pass")


class TestReviewR6(unittest.TestCase):
    """R6 I1/I2: every fault type counts, and challenge exercise is reported per onset set."""

    def test_new_overspeed_or_overtemp_fault_fails(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, events=["overspeed", "rearm_after_overspeed", "overtemp"],
                       event_counts={"overspeed": 1, "rearm_after_overspeed": 1, "overtemp": 1})
        g = gate(rows)
        self.assertEqual(status(g, "no new fault"), "fail")
        self.assertEqual(g["overall"], "fail")

    def test_equal_fault_counts_in_both_variants_pass(self):
        rows = good_heldout()
        for r in rows:
            if r["seed"] == 0:
                r["event_counts"] = {"invalid_command": 1, "rearm_after_invalid_command": 1}
        self.assertEqual(status(gate(rows), "no new fault"), "pass")

    def test_missing_event_counts_are_incomplete(self):
        rows = good_heldout()
        rows[-1].pop("event_counts")
        self.assertEqual(status(gate(rows), "no new fault"), "incomplete")

    def test_exercise_is_reported_per_sub_challenge(self):
        ch = good_challenges()
        for r in ch:
            if r["challenge"] == "yaw_B":
                r["learn_usable_during_challenge"] = 0.0
        late = [dict(r, challenge="yaw_B@late", learn_usable_during_challenge=0.8)
                for r in good_challenges() if r["challenge"] == "yaw_B"]
        keys = CH_KEYS + [("yaw_B@late", [0.0, 0.1], 3.2, s) for s in range(2)]
        g = gate(good_heldout(), ch=ch + late, expected_challenge_keys=keys)
        crit = next(c for c in g["criteria"] if c["name"].startswith("required supplementary"))
        exer = {e["challenge"]: e["correction_active"] for e in crit["evidence"][3]["exercise"]}
        self.assertEqual((exer["yaw_B"], exer["yaw_B@late"]), (0, 2))


class TestReviewR6Round2(unittest.TestCase):
    """R6 round-2 minors r2-1 and r2-2."""

    def test_unregistered_challenge_cells_are_incomplete(self):
        extra = [dict(row("adaptive", 0, 1.0, event_counts={"overspeed": 3}), challenge="yaw_C@late",
                      learn_usable_during_challenge=0.9)]
        g = gate(good_heldout(), ch=good_challenges() + extra)
        self.assertEqual(status(g, "registered grid"), "incomplete")
        self.assertNotEqual(g["overall"], "pass")

    def test_new_fault_fails_even_when_another_pair_lacks_counts(self):
        rows = good_heldout()
        rows[N].pop("event_counts")                                   # adaptive seed 0: no counts
        rows[-1] = row("adaptive", 9, 1.0, event_counts={"overtemp": 1})
        self.assertEqual(status(gate(rows), "no new fault"), "fail")


class TestReviewR6Round3(unittest.TestCase):
    """R6 round-3 m3: non-finite or inconsistent evidence is never a pass."""

    def test_nan_fault_or_limit_evidence_is_incomplete(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, wd_trips=float("nan"))
        self.assertEqual(status(gate(rows), "no new fault"), "incomplete")
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, command_over_limit=float("nan"))
        self.assertEqual(status(gate(rows), "host current command"), "incomplete")

    def test_listed_event_missing_from_counts_is_incomplete(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, events=["overtemp"], event_counts={})
        self.assertEqual(status(gate(rows), "no new fault"), "incomplete")
        self.assertNotEqual(gate(rows)["overall"], "pass")


class TestGateEvidence(unittest.TestCase):
    def _log(self):
        import numpy as np
        from tests.test_metrics import synth
        log = synth(T=2.0, tele=dict(gov_sigma=0.0))
        log.i_lim[:] = np.where(log.t < 1.0, 3.2, 2.4)            # derate at 1.0 s
        log.i_raw[:] = 2.0
        return log

    def test_command_above_a_new_limit_is_allowed_only_within_the_grace(self):
        from exp.task3_l4 import gate_evidence
        log = self._log()
        log.i_raw[(log.t >= 1.0) & (log.t < 1.015)] = 3.0         # host has not yet seen the derate
        self.assertLessEqual(gate_evidence(log, [])["command_over_limit"], 0.0)
        log.i_raw[(log.t >= 1.5) & (log.t < 1.51)] = 3.0          # long after the change: a violation
        self.assertGreater(gate_evidence(log, [])["command_over_limit"], 0.0)

    def test_event_counts_are_untruncated(self):
        from exp.task3_l4 import gate_evidence
        log = self._log()
        log.events = [(0.1 * k, "invalid_command") for k in range(9)] + [(1.0, "overspeed")]
        ev = gate_evidence(log, [])["event_counts"]
        self.assertEqual(ev, {"invalid_command": 9, "overspeed": 1})

    def test_missing_learning_telemetry_gives_none(self):
        from exp.task3_l4 import gate_evidence
        ev = gate_evidence(self._log(), [])
        self.assertIsNone(ev["applied_while_unusable"])
        self.assertIsNone(ev["learn_usable_during_challenge"])


if __name__ == "__main__":
    unittest.main()
