"""Machine-auditable Task 3 adoption gate (EXECUTION_PLAN.md R3).

Every registered criterion of docs/plans/task3-learning.md §8 is evaluated per
pair and returned with its evidence. A criterion is "pass", "fail" or
"incomplete". Missing metrics, missing rows or missing required challenge groups
give "incomplete", never "pass". The overall verdict is "fail" if any criterion
fails, else "incomplete" if any is incomplete, else "pass".

Rows are dicts as produced by exp/task3_l4.py (and exp/task3_challenges.py):
variant, load, limit, seed, primary_deg, completed, missing_waypoints,
waypoint_visits, progress, wd_trips, suspended, rejected, target_over_limit,
learn_usable_max, applied_while_unusable, events.
"""
import math

BENEFIT_MIN = 0.10            # median paired reduction of the primary metric
AVAILABILITY_MIN = 0.80       # share of held-out runs that ever obtain a usable estimate
DETERIORATION_MAX = 0.20      # worst allowed per-case primary increase (ordinary feasible cases)
PROGRESS_SLACK = 0.01         # progress the candidate may lose where a benefit is credited
LIMIT_TOL = 1e-9
REQUIRED_CHALLENGES = ("yaw_B", "yaw_C", "feedback_outage", "derate", "payload_change")


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def _key(r):
    return (tuple(r["load"]), r["limit"], r["seed"])


def _pairs(rows, comparator, candidate):
    ref = {_key(r): r for r in rows if r["variant"] == comparator}
    cand = {_key(r): r for r in rows if r["variant"] == candidate}
    return [(k, ref[k], cand[k]) for k in sorted(set(ref) | set(cand)) if k in ref and k in cand], \
        sorted(set(ref) ^ set(cand))


def _crit(name, status, value=None, threshold=None, evidence=None):
    return dict(name=name, status=status, value=value, threshold=threshold, evidence=evidence or [])


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    return float("nan") if not n else (xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2]))


def benefit(pairs):
    red, bad = [], []
    for k, b, c in pairs:
        if _finite(b.get("primary_deg")) and _finite(c.get("primary_deg")) and b["primary_deg"] > 0:
            red.append(1 - c["primary_deg"] / b["primary_deg"])
        else:
            bad.append(list(k))
    if not pairs or len(red) < AVAILABILITY_MIN * len(pairs):
        return _crit("median paired primary reduction >= 10 %", "incomplete", evidence=bad)
    m = _median(red)
    return _crit("median paired primary reduction >= 10 %", "pass" if m >= BENEFIT_MIN else "fail",
                 m, BENEFIT_MIN, [dict(unscored_pairs=bad)] if bad else [])


def availability(pairs):
    vals = [c.get("learn_usable_max") for _, _, c in pairs]
    if not pairs or any(v is None for v in vals):
        return _crit(">= 80 % of runs ever obtain a usable estimate", "incomplete")
    share = sum(v > 0.5 for v in vals) / len(vals)
    return _crit(">= 80 % of runs ever obtain a usable estimate", "pass" if share >= AVAILABILITY_MIN else "fail",
                 share, AVAILABILITY_MIN)


def waypoints(pairs):
    lost, missing = [], []
    for k, b, c in pairs:
        if b.get("missing_waypoints") is None or c.get("missing_waypoints") is None:
            missing.append(list(k))
            continue
        extra = sorted(set(c["missing_waypoints"]) - set(b["missing_waypoints"]))
        if extra:
            lost.append(dict(pair=list(k), lost_waypoints=extra))
        if b.get("completed") and not c.get("completed"):
            lost.append(dict(pair=list(k), lost_completion=True))
    if missing:
        return _crit("no individual comparator waypoint or completion lost", "incomplete", evidence=missing)
    return _crit("no individual comparator waypoint or completion lost", "fail" if lost else "pass",
                 len(lost), 0, lost)


def deterioration(pairs):
    worst, bad, unscored = 0.0, [], []
    for k, b, c in pairs:
        ordinary = b.get("completed") and _finite(b.get("primary_deg")) and b["primary_deg"] > 0
        if not ordinary:
            continue
        if not _finite(c.get("primary_deg")):
            unscored.append(list(k))
            continue
        inc = c["primary_deg"] / b["primary_deg"] - 1
        worst = max(worst, inc)
        if inc > DETERIORATION_MAX:
            bad.append(dict(pair=list(k), increase=inc))
    if unscored:
        return _crit("no ordinary feasible case > 20 % worse", "fail", worst, DETERIORATION_MAX,
                     [dict(unscored_candidate=unscored)] + bad)
    return _crit("no ordinary feasible case > 20 % worse", "fail" if bad else "pass", worst,
                 DETERIORATION_MAX, bad)


def current_limit(rows):
    vals = [r.get("target_over_limit") for r in rows]
    if not rows or any(v is None for v in vals):
        return _crit("applied current target never above the active limit", "incomplete")
    bad = [dict(run=r.get("run_id"), excess=r["target_over_limit"]) for r in rows if r["target_over_limit"] > LIMIT_TOL]
    return _crit("applied current target never above the active limit", "fail" if bad else "pass",
                 max(vals), LIMIT_TOL, bad)


def safety(pairs):
    new = []
    for k, b, c in pairs:
        for f in ("wd_trips", "suspended", "rejected"):
            if c.get(f) is None or b.get(f) is None:
                return _crit("no new fault, suspension, rejection or lockout", "incomplete", evidence=[list(k)])
            if float(c[f]) > float(b[f]):
                new.append(dict(pair=list(k), field=f, comparator=b[f], candidate=c[f]))
        if "lockout" in (c.get("events") or []) and "lockout" not in (b.get("events") or []):
            new.append(dict(pair=list(k), field="lockout"))
    return _crit("no new fault, suspension, rejection or lockout", "fail" if new else "pass", len(new), 0, new)


def stale_application(rows):
    vals = [r.get("applied_while_unusable") for r in rows]
    if not rows or any(v is None for v in vals):
        return _crit("no learned correction applied while the estimator is unusable", "incomplete")
    bad = [dict(run=r.get("run_id"), samples=r["applied_while_unusable"]) for r in rows if r["applied_while_unusable"]]
    return _crit("no learned correction applied while the estimator is unusable", "fail" if bad else "pass",
                 sum(vals), 0, bad)


def no_slower_credit(pairs):
    bad = []
    for k, b, c in pairs:
        if not (_finite(b.get("primary_deg")) and _finite(c.get("primary_deg"))):
            continue
        if c["primary_deg"] < b["primary_deg"]:                     # a credited improvement
            if not (_finite(b.get("progress")) and _finite(c.get("progress"))):
                return _crit("no benefit credited to slower motion or reduced range", "incomplete",
                             evidence=[list(k)])
            if c["progress"] < b["progress"] - PROGRESS_SLACK or (b.get("completed") and not c.get("completed")):
                bad.append(dict(pair=list(k), progress=(b["progress"], c["progress"])))
    return _crit("no benefit credited to slower motion or reduced range", "fail" if bad else "pass",
                 len(bad), 0, bad)


def challenges(challenge_rows, comparator, candidate):
    """Required supplementary groups present, exercised, and without safety failures."""
    if challenge_rows is None:
        return _crit("required supplementary challenges present and passed", "incomplete",
                     evidence=[dict(missing=list(REQUIRED_CHALLENGES))])
    groups = {r.get("challenge") for r in challenge_rows}
    missing = [g for g in REQUIRED_CHALLENGES if g not in groups]
    unexercised, failures = [], []
    for g in REQUIRED_CHALLENGES:
        rows = [r for r in challenge_rows if r.get("challenge") == g]
        if not rows:
            continue
        pairs, _ = _pairs(rows, comparator, candidate)
        cand = [c for _, _, c in pairs]
        if not any((c.get("learn_usable_during_challenge") or 0) > 0 for c in cand):
            unexercised.append(g)
        for crit in (current_limit(cand), safety(pairs), stale_application(cand), waypoints(pairs)):
            if crit["status"] == "fail":
                failures.append(dict(challenge=g, criterion=crit["name"], evidence=crit["evidence"]))
            elif crit["status"] == "incomplete":
                missing.append(f"{g}: {crit['name']}")
    ev = [dict(missing=missing), dict(unexercised=unexercised), dict(failures=failures)]
    if failures:
        return _crit("required supplementary challenges present and passed", "fail", evidence=ev)
    if missing or unexercised:
        return _crit("required supplementary challenges present and passed", "incomplete", evidence=ev)
    return _crit("required supplementary challenges present and passed", "pass", evidence=ev)


def adoption_gate(heldout_rows, comparator, candidate, challenge_rows=None):
    pairs, unpaired = _pairs(heldout_rows, comparator, candidate)
    cand_rows = [c for _, _, c in pairs]
    crits = [benefit(pairs), availability(pairs), waypoints(pairs), deterioration(pairs),
             current_limit(cand_rows), safety(pairs), stale_application(cand_rows), no_slower_credit(pairs),
             challenges(challenge_rows, comparator, candidate)]
    if unpaired:
        crits.append(_crit("every held-out case paired", "incomplete", evidence=[list(k) for k in unpaired]))
    status = [c["status"] for c in crits]
    overall = "fail" if "fail" in status else ("incomplete" if "incomplete" in status else "pass")
    return dict(overall=overall, comparator=comparator, candidate=candidate, pairs=len(pairs), criteria=crits)
