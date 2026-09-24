"""R3: the Task 3 adoption gate is complete and cannot be passed by hiding a failure."""
import copy
import unittest

from exp.task3_gate import REQUIRED_CHALLENGES, adoption_gate


def row(variant, seed, primary, **kw):
    r = dict(variant=variant, load=[0.0, 0.1], limit=3.2, seed=seed, run_id=f"{variant}{seed}",
             primary_deg=primary, completed=True, missing_waypoints=[], progress=1.0, wd_trips=0,
             suspended=False, rejected=False, target_over_limit=-0.1, learn_usable_max=1.0,
             applied_while_unusable=0, events=[])
    r.update(kw)
    return r


def good_heldout(n=10):
    return ([row("int1", s, 2.0) for s in range(n)] +
            [row("adaptive", s, 1.0) for s in range(n)])


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
        g = adoption_gate(good_heldout(), "int1", "adaptive", good_challenges())
        self.assertEqual(g["overall"], "pass", [c for c in g["criteria"] if c["status"] != "pass"])

    def test_hidden_lost_waypoint_fails_even_if_completion_flags_agree(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, missing_waypoints=[2])    # still flagged completed
        g = adoption_gate(rows, "int1", "adaptive", good_challenges())
        self.assertEqual(status(g, "no individual comparator waypoint"), "fail")
        self.assertEqual(g["overall"], "fail")

    def test_single_severe_regression_is_not_masked_by_the_median(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 3.0)                            # +50 % in one case
        g = adoption_gate(rows, "int1", "adaptive", good_challenges())
        self.assertEqual(status(g, "median paired"), "pass")          # the median still looks good
        self.assertEqual(status(g, "no ordinary feasible case"), "fail")
        self.assertEqual(g["overall"], "fail")

    def test_unavailable_metrics_are_incomplete_not_pass(self):
        rows = good_heldout()
        for r in rows:
            r.pop("applied_while_unusable")
            r.pop("target_over_limit")
        g = adoption_gate(rows, "int1", "adaptive", good_challenges())
        self.assertEqual(status(g, "applied current target"), "incomplete")
        self.assertEqual(status(g, "no learned correction applied"), "incomplete")
        self.assertEqual(g["overall"], "incomplete")

    def test_target_limit_violation_fails(self):
        rows = good_heldout()
        rows[-1] = row("adaptive", 9, 1.0, target_over_limit=0.01)
        self.assertEqual(adoption_gate(rows, "int1", "adaptive", good_challenges())["overall"], "fail")

    def test_missing_or_unexercised_challenges_are_incomplete(self):
        self.assertEqual(adoption_gate(good_heldout(), "int1", "adaptive", None)["overall"], "incomplete")
        partial = [r for r in good_challenges() if r["challenge"] != "payload_change"]
        self.assertEqual(adoption_gate(good_heldout(), "int1", "adaptive", partial)["overall"], "incomplete")
        idle = copy.deepcopy(good_challenges())
        for r in idle:
            r["learn_usable_during_challenge"] = 0.0
        self.assertEqual(adoption_gate(good_heldout(), "int1", "adaptive", idle)["overall"], "incomplete")

    def test_a_new_fault_in_a_challenge_fails(self):
        ch = good_challenges()
        for r in ch:
            if r["variant"] == "adaptive" and r["challenge"] == "derate":
                r["wd_trips"] = 1
        self.assertEqual(adoption_gate(good_heldout(), "int1", "adaptive", ch)["overall"], "fail")

    def test_benefit_from_slower_motion_is_not_credited(self):
        rows = good_heldout()
        for r in rows:
            if r["variant"] == "adaptive":
                r["progress"] = 0.9
        g = adoption_gate(rows, "int1", "adaptive", good_challenges())
        self.assertEqual(status(g, "no benefit credited"), "fail")

    def test_insufficient_benefit_and_availability_fail(self):
        rows = [row("int1", s, 2.0) for s in range(10)] + \
               [row("adaptive", s, 1.95, learn_usable_max=1.0 if s < 7 else 0.0) for s in range(10)]
        g = adoption_gate(rows, "int1", "adaptive", good_challenges())
        self.assertEqual(status(g, "median paired"), "fail")
        self.assertEqual(status(g, ">= 80 %"), "fail")


if __name__ == "__main__":
    unittest.main()
