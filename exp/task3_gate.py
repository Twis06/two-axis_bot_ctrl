"""Machine-auditable Task 3 adoption gate (EXECUTION_PLAN.md R3).

Every registered criterion of docs/plans/task3-learning.md §8 is evaluated per
pair and returned with its evidence. A criterion is "pass", "fail" or
"incomplete". Missing metrics, missing rows or missing required challenge groups
give "incomplete", never "pass". The overall verdict is "fail" if any criterion
fails, else "incomplete" if any is incomplete, else "pass".

Rows are dicts as produced by exp/task3_l4.py (and exp/task3_challenges.py):
variant, load, limit, seed, primary_deg, completed, missing_waypoints,
waypoint_visits, progress, t_complete, wd_trips, suspended, rejected, lockout,
command_over_limit, learn_usable_max, applied_while_unusable, events.
"Ordinary feasible" = every registered held-out pair; the ">= 80 % usable" rule is
applied as a fail (the registration made the comparison "conditional"; this is
stricter and declared).
"""
import math

BENEFIT_MIN = 0.10            # median paired reduction of the primary metric
AVAILABILITY_MIN = 0.80       # share of held-out runs that ever obtain a usable estimate
DETERIORATION_MAX = 0.20      # worst allowed per-case primary increase (ordinary feasible cases)
PROGRESS_SLACK = 0.01         # progress the candidate may lose where a benefit is credited
TIME_SLACK_S = 0.05           # completion time the candidate may lose where a benefit is credited
LIMIT_GRACE_S = 0.02          # after a limit change (declared; see current_limit)
LIMIT_TOL = 1e-9
REQUIRED_CHALLENGES = ("yaw_B", "yaw_C", "feedback_outage", "derate", "payload_change")


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def _key(r):
    return (tuple(r["load"]), r["limit"], r["seed"])


def _pairs(rows, comparator, candidate):
    ref, cand = {}, {}
    for r in rows:
        d = ref if r["variant"] == comparator else cand if r["variant"] == candidate else None
        if d is not None:
            d.setdefault(_key(r), []).append(r)
    dup = sorted(k for d in (ref, cand) for k, v in d.items() if len(v) > 1)
    ref = {k: v[0] for k, v in ref.items()}
    cand = {k: v[0] for k, v in cand.items()}
    pairs = [(k, ref[k], cand[k]) for k in sorted(set(ref) | set(cand)) if k in ref and k in cand]
    return pairs, sorted(set(ref) ^ set(cand)), dup


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
    if not pairs or bad:
        # Every registered pair must be scored; an unscored pair cannot be dropped from the median.
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
    """Ordinary feasible = every registered held-out pair (the unholdable-load checks are
    a separate split), defined independently of either controller's outcome."""
    worst, bad, unscored = 0.0, [], []
    if not pairs:
        return _crit("no ordinary feasible case > 20 % worse", "incomplete")
    for k, b, c in pairs:
        if not (_finite(b.get("primary_deg")) and b["primary_deg"] > 0 and _finite(c.get("primary_deg"))):
            unscored.append(list(k))
            continue
        inc = c["primary_deg"] / b["primary_deg"] - 1
        worst = max(worst, inc)
        if inc > DETERIORATION_MAX:
            bad.append(dict(pair=list(k), increase=inc))
    if bad:
        return _crit("no ordinary feasible case > 20 % worse", "fail", worst, DETERIORATION_MAX,
                     bad + ([dict(unscored=unscored)] if unscored else []))
    if unscored:
        return _crit("no ordinary feasible case > 20 % worse", "incomplete", worst, DETERIORATION_MAX,
                     [dict(unscored=unscored)])
    return _crit("no ordinary feasible case > 20 % worse", "pass", worst, DETERIORATION_MAX)


def current_limit(rows):
    """The drive clamps the APPLIED target by construction, so that is only a simulator
    consistency check. The criterion uses the pre-clamp command the drive received
    (i_raw) against the active limit, outside LIMIT_GRACE_S after a limit change
    (the host learns a new limit through delayed feedback)."""
    name = "host current command within the active limit (pre-clamp)"
    vals = [r.get("command_over_limit") for r in rows]
    if not rows or not all(_finite(v) for v in vals):             # None or NaN is missing evidence
        return _crit(name, "incomplete")
    bad = [dict(run=r.get("run_id"), excess=r["command_over_limit"]) for r in rows if r["command_over_limit"] > LIMIT_TOL]
    return _crit(name, "fail" if bad else "pass", max(vals), LIMIT_TOL, bad)


def safety(pairs):
    """Every latched fault and re-arm is compared per pair from the untruncated event
    counts (any event type the candidate has more often is new), plus suspension,
    rejection and lockout. Every pair is scanned: any new fault is a fail even if other
    pairs lack evidence; otherwise missing evidence is incomplete, never pass."""
    new, missing = [], []

    def counts_ok(r):     # a count table that is present, numeric and covers every listed event
        ec = r.get("event_counts")
        return (isinstance(ec, dict) and all(_finite(v) for v in ec.values())
                and set(r.get("events") or []) <= set(ec))

    for k, b, c in pairs:
        if not counts_ok(c) or not counts_ok(b):
            missing.append(dict(pair=list(k), missing="event_counts (absent, non-numeric or inconsistent with events)"))
        else:
            for ev in sorted(set(c["event_counts"]) | set(b["event_counts"])):
                nc, nb = c["event_counts"].get(ev, 0), b["event_counts"].get(ev, 0)
                if nc > nb:
                    new.append(dict(pair=list(k), field=ev, comparator=nb, candidate=nc))
        for f in ("wd_trips", "suspended", "rejected"):
            if not (_finite(c.get(f)) or isinstance(c.get(f), bool)) or not (_finite(b.get(f)) or isinstance(b.get(f), bool)):
                missing.append(dict(pair=list(k), missing=f))
            elif float(c[f]) > float(b[f]):
                new.append(dict(pair=list(k), field=f, comparator=b[f], candidate=c[f]))
        lock = lambda r: bool(r.get("lockout")) or "lockout" in (r.get("events") or [])
        if lock(c) and not lock(b):
            new.append(dict(pair=list(k), field="lockout"))
    status = "fail" if new else ("incomplete" if missing else "pass")
    return _crit("no new fault, suspension, rejection or lockout", status, len(new), 0, new + missing)


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
            later = (_finite(b.get("t_complete")) and _finite(c.get("t_complete"))
                     and c["t_complete"] > b["t_complete"] + TIME_SLACK_S)
            if c["progress"] < b["progress"] - PROGRESS_SLACK or (b.get("completed") and not c.get("completed")) or later:
                bad.append(dict(pair=list(k), progress=(b["progress"], c["progress"]),
                                t_complete=(b.get("t_complete"), c.get("t_complete"))))
    return _crit("no benefit credited to slower motion or reduced range", "fail" if bad else "pass",
                 len(bad), 0, bad)


def challenges(challenge_rows, comparator, candidate):
    """Required supplementary groups present, exercised, and without safety failures."""
    if challenge_rows is None:
        return _crit("required supplementary challenges present and passed", "incomplete",
                     evidence=[dict(missing=list(REQUIRED_CHALLENGES))])
    base = lambda r: (r.get("challenge") or "").split("@")[0]      # "yaw_B@late" counts for yaw_B
    groups = {base(r) for r in challenge_rows}
    missing = [g for g in REQUIRED_CHALLENGES if g not in groups]
    unexercised, failures, exercise = [], [], []
    for g in REQUIRED_CHALLENGES:
        rows = [r for r in challenge_rows if base(r) == g]
        if not rows:
            continue
        pairs = []
        for sub in sorted({r["challenge"] for r in rows}):             # pair within each onset set
            sub_pairs = _pairs([r for r in rows if r["challenge"] == sub], comparator, candidate)[0]
            pairs += sub_pairs
            active = sum((c.get("learn_usable_during_challenge") or 0) > 0 for _, _, c in sub_pairs)
            exercise.append(dict(challenge=sub, candidate_runs=len(sub_pairs), correction_active=active))
        cand = [c for _, _, c in pairs]
        if not any((c.get("learn_usable_during_challenge") or 0) > 0 for c in cand):
            unexercised.append(g)
        for crit in (current_limit(cand), safety(pairs), stale_application(cand), waypoints(pairs)):
            if crit["status"] == "fail":
                failures.append(dict(challenge=g, criterion=crit["name"], evidence=crit["evidence"]))
            elif crit["status"] == "incomplete":
                missing.append(f"{g}: {crit['name']}")
    ev = [dict(missing=missing), dict(unexercised=unexercised), dict(failures=failures), dict(exercise=exercise)]
    if failures:
        return _crit("required supplementary challenges present and passed", "fail", evidence=ev)
    if missing or unexercised:
        return _crit("required supplementary challenges present and passed", "incomplete", evidence=ev)
    return _crit("required supplementary challenges present and passed", "pass", evidence=ev)


def adoption_gate(heldout_rows, comparator, candidate, challenge_rows=None, expected_keys=None,
                  expected_challenge_keys=None):
    """expected_keys: registered (load, limit, seed) held-out cells; expected_challenge_keys:
    registered (challenge, load, limit, seed) cells. Any missing or duplicated cell is incomplete."""
    pairs, unpaired, dup = _pairs(heldout_rows, comparator, candidate)
    cand_rows = [c for _, _, c in pairs]
    crits = [benefit(pairs), availability(pairs), waypoints(pairs), deterioration(pairs),
             current_limit(cand_rows), safety(pairs), stale_application(cand_rows), no_slower_credit(pairs),
             challenges(challenge_rows, comparator, candidate)]
    grid = []
    if expected_keys is None:
        grid.append("registered held-out grid not supplied")
    else:
        have = {k for k, _, _ in pairs}
        grid += [f"missing held-out cell {list(k)}" for k in sorted(set(map(lambda k: (tuple(k[0]), k[1], k[2]), expected_keys)) - have)]
        grid += [f"unregistered held-out cell {list(k)}" for k in sorted(have - set(map(lambda k: (tuple(k[0]), k[1], k[2]), expected_keys)))]
    if challenge_rows is not None:
        if expected_challenge_keys is None:
            grid.append("registered challenge grid not supplied")
        else:
            for v in (comparator, candidate):
                seen = [(r["challenge"], tuple(r["load"]), r["limit"], r["seed"]) for r in challenge_rows
                        if r["variant"] == v]
                exp = {(g, tuple(l), lim, sd) for g, l, lim, sd in expected_challenge_keys}
                grid += [f"{v}: missing challenge cell {list(k)}" for k in sorted(exp - set(seen))]
                grid += [f"{v}: unregistered challenge cell {list(k)}" for k in sorted(set(seen) - exp)]
                grid += [f"{v}: duplicate challenge cell {list(k)}" for k in sorted({x for x in seen if seen.count(x) > 1})]
    grid += [f"unpaired held-out cell {list(k)}" for k in unpaired]
    grid += [f"duplicate held-out cell {list(k)}" for k in dup]
    crits.append(_crit("registered grid complete, every cell paired exactly once",
                       "incomplete" if grid else "pass", len(grid), 0, grid))
    status = [c["status"] for c in crits]
    overall = "fail" if "fail" in status else ("incomplete" if "incomplete" in status else "pass")
    return dict(overall=overall, comparator=comparator, candidate=candidate, pairs=len(pairs), criteria=crits)
