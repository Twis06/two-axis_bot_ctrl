"""Task 3 decision input: the known-load ceiling under the pre-registered protocol.

    python -m exp.task3_l4          # publishes report/task3_ceiling_*.{md,json}

docs/plans/task3-learning.md §8 pre-registered an L4 experiment (comparators,
loads, sequences, seeds, primary metric, >= 10 % adoption gate). This module runs
the frozen deterministic comparators, the adaptive
candidate, and the diagnostic oracle:

  1. frozen deterministic baseline (fingerprint 7d857df507c389c9)
  2. integral-rate variants 0.5x / 1x / 2x, the best selected on the TUNING
     loads and seeds only (loop margins checked against the declared criteria)
  3. adaptive feed-forward-only candidate using the exact frozen L1 estimator
     configuration through the L3 adapter
  4. DIAGNOSTIC ORACLE: the true extra load theta_s sin q + theta_c cos q given
     to the controller. Unavailable to a deployable controller; a ceiling.
     Two forms: in feed-forward and governor feasibility (the existing
     load-model hook), and in feed-forward only (governor nominal).

Logic of the decision: an adaptive estimator of the same two-parameter law can
at best approach the oracle. If the oracle does not beat the stronger
deterministic comparator by the registered 10 % on the registered primary
metric, no estimator of that law can pass the gate, and rejection is
evidence-based. If it does, rejection without the matched adaptive candidate is
premature.

Primary metric (registered): governed-reference RMS during the first 0.5 s of
each TEST dwell, dwell located on the governor's path clock (sigma). Everything
is Simulated; the oracle is labelled as such in every table.
"""
import argparse
import math
import os
import tempfile
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from ctrl import loopshape as LS
from ctrl.adaptive import AdaptiveController
from ctrl.baseline import BaselineController
from exp import manifest as MF
from exp import scenarios as S
from exp.evidence import RunBook, baseline_fingerprint, run_set_id
from sim import params as P
from sim.config import SimConfig
from sim.trajectories import Hold, MinJerkSequence

ROOT = Path(__file__).resolve().parents[1]
ENTRY = "exp.task3_l4"
PROVENANCE_SOURCES = ("exp/evidence.py",)
FROZEN = "7d857df507c389c9"
R, DEG = math.radians, 180 / math.pi

# Registered protocol (docs/plans/task3-learning.md §8)
TUNING_LOADS = ((0.0, 0.08), (0.04, 0.12))
TUNING_SEEDS = (11, 12, 13)
HELDOUT_LOADS = ((0.0, 0.10), (0.0, 0.18), (0.0, -0.18), (0.06, 0.14), (-0.06, 0.14))
HELDOUT_SEEDS = (101, 102, 103, 104, 105)
LIMITS = (3.2, 2.4)
UNHOLDABLE = (0.0, 0.40)          # rejection check at 2.4 A, not a performance case
CALIB = [(R(-50), 1.0, 1.0), (0.0, 1.0, 1.0), (R(50), 1.0, 1.0), (0.0, 1.0, 1.0)]
TEST = [(R(30), 1.0, 1.0), (R(-30), 1.0, 1.0), (R(65), 1.0, 1.0), (R(-65), 1.0, 1.0), (0.0, 1.0, 1.0)]
WI = (0.5, 1.0, 2.0)              # x the frozen integral rate (wi_ratio 0.1)
SLACK = 3.0


class OracleLoad:
    """DIAGNOSTIC ORACLE: the true extra load law. Not deployable."""
    diagnostic_oracle = True

    def __init__(self, theta_s, theta_c):
        self.theta_s, self.theta_c = theta_s, theta_c

    def reset(self):
        pass

    def update(self, *a, **k):
        pass

    def torque(self, q):
        return self.theta_s * math.sin(q) + self.theta_c * math.cos(q)


class OracleFFOnly(BaselineController):
    """Oracle load in feed-forward only; governor feasibility stays nominal."""
    name = "baseline+oracle_ff_only"

    def reset(self, cfg):
        super().reset(cfg)
        step = self.gov.step

        def nominal_step(ts, ref_at, i_limit, tau_extra=0.0, tau_couple=0.0, extra_fn=None, **kw):
            return step(ts, ref_at, i_limit, tau_extra, tau_couple, None, **kw)
        self.gov.step = nominal_step


def sequence():
    segs = [(0.0, 1.0, 0.0)] + CALIB + TEST            # 1 s initial dwell, then calibration, test
    roll = MinJerkSequence(0.0, segs)
    test_dwells = [(t0 + T, t0 + T + 1.0) for (t0, T, q0, q1) in roll.segs[-len(TEST):]]
    return roll, test_dwells


def scenario(load, i_limit, name):
    """Load moments as physics: theta_s adds to gravity; theta_c is a lateral
    point mass at +/-35 mm (sign by shift direction, never a negative mass)."""
    ts, tc = load
    s_lat = 0.035 if tc >= 0 else -0.035
    m = abs(tc) / (P.G * 0.035)
    roll, _ = sequence()
    cfg = replace(SimConfig(), duration=roll.t_end + SLACK).with_(
        plant=dict(tau_g=P.TAU_G + ts, m_payload=m, s_lat=s_lat), drive=dict(i_limit=i_limit))
    sc = S.Scenario(name, cfg, roll, Hold(0.0),
                    f"Task 3 L4 protocol: load (theta_s, theta_c) = ({ts:+.2f}, {tc:+.2f}) N m, "
                    f"{i_limit} A, calibration then test sequence")
    # Completion / waypoint scoring on the TEST waypoints only.
    test_wp = tuple(q for q, _, _ in TEST)
    return sc, test_wp


def make(variant, load, seed=None):
    if variant.startswith("int"):
        k = float(variant[3:])
        return BaselineController(wi_ratio=0.1 * k)
    if variant == "adaptive":
        # The L4 candidate uses the exact frozen L1 stationarity gate. The
        # widened 10-count helper is reserved for its separate tuning test.
        return AdaptiveController(est_seed=0 if seed is None else int(seed))
    if variant == "oracle":
        return BaselineController(load_model=OracleLoad(*load))
    if variant == "oracle_ff":
        return OracleFFOnly(load_model=OracleLoad(*load))
    raise ValueError(variant)


_BOOK = None


def primary(log, dwells):
    """RMS of q - q_c over the first 0.5 s of each test dwell, located on sigma."""
    sig = np.asarray(log.c_gov_sigma)
    e = np.asarray(log.q - log.c_q_c)
    m = np.zeros(len(sig), bool)
    for d0, _ in dwells:
        m |= (sig >= d0) & (sig < d0 + 0.5) & np.isfinite(sig)
    # Frozen-clock samples (sigma held in the window during a stop/hold) are included
    # only while the drive is normal; fallback is excluded and reported separately.
    m &= np.asarray(log.mode) == 0
    return (float(np.sqrt(np.mean(e[m] ** 2)) * DEG) if m.any() else float("nan")), int(m.sum())


def _work(job):
    global _BOOK
    if _BOOK is None:
        _BOOK = RunBook(ENTRY)
    variant, load, lim, seed, split = job
    sc, test_wp = scenario(load, lim, f"L4 {split} {load} {lim}A")
    before = set(_BOOK.runs)
    # evaluate() scoring on test waypoints: pass them via a FiniteMotion-like wrapper
    from exp.motions import FiniteMotion
    t_req = sequence()[0].segs[-1][0] + 1.0
    fm = FiniteMotion(sc.name, sc.cfg, sc.roll, sc.yaw, sc.note, test_wp, t_req)
    log, stats, rid, ev = _BOOK.run(fm, lambda: make(variant, load, seed), seed=seed)
    _, dwells = sequence()
    prim, n = primary(log, dwells)
    comp = ev.get("completion") or {}
    fb = float(np.mean(log.mode > 0))
    learn = np.asarray(getattr(log, "c_learn_usable", []), dtype=float)
    learn_ok = np.isfinite(learn) & (learn > 0.5)
    learn_max = float(np.max(learn)) if learn.size else 0.0
    learn_frac = float(np.mean(learn_ok)) if learn.size else 0.0
    learn_first = float(log.t[int(np.flatnonzero(learn_ok)[0])]) if np.any(learn_ok) else None
    row = dict(variant=variant, load=list(load), limit=lim, seed=seed, split=split, run_id=rid,
               primary_deg=prim, primary_samples=n, completed=bool(comp.get("completed")),
               reached=comp.get("reached"), t_complete=comp.get("t_complete"),
               wd_trips=stats["wd_trips"], suspended=bool(np.nanmax(log.c_suspended) > 0.5),
               rejected=bool(np.nanmax(log.c_request_rejected) > 0.5), fallback_frac=fb,
               progress=float(ev["progress"]["progress"]), rms_gov=stats["rms_gov"],
               clip_pct=stats["clip_pct"], i2=float(np.sum(log.i ** 2) * 1e-3),
               learn_usable_max=learn_max, learn_usable_fraction=learn_frac,
               learn_first_usable_s=learn_first,
               events=[e[1] for e in log.events][:6])
    new = {k: _BOOK.runs[k] for k in _BOOK.runs if k not in before}
    return dict(row=row, records=new, code=_BOOK.code, env=_BOOK.env, defs=_BOOK.defs)


def _work_entry(job):
    from exp import task3_l4 as T
    return T._work(job)


def margins_for(k):
    c = BaselineController()
    rows = []
    for name, T, J, kt in (("design 7 ms", .007, P.J_R, 1.0), ("11 ms, J -30 %, Kt +15 %", .011, .0028, 1.15)):
        pm, gm, _ = LS.margins(c.K * kt, c.wc, T, c.alpha, wi_ratio=0.1 * k, J=J)
        rows.append((name, pm, gm))
    return rows


def med(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return float(np.median(xs)) if xs else float("nan")


def table(h, rows):
    return "\n".join(["| " + " | ".join(h) + " |", "|" + "---|" * len(h)]
                     + ["| " + " | ".join(str(c) for c in r) + " |" for r in rows])


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args(argv)
    fp = baseline_fingerprint()[0][:16]
    if fp != FROZEN:
        raise SystemExit(f"baseline fingerprint {fp} is not the frozen {FROZEN}")
    variants = tuple(f"int{k:g}" for k in WI) + ("adaptive", "oracle", "oracle_ff")
    jobs = [(v, ld, lim, sd, "tuning") for v in variants[:4] for ld in TUNING_LOADS for lim in LIMITS
            for sd in TUNING_SEEDS]
    jobs += [(v, ld, lim, sd, "heldout") for v in variants for ld in HELDOUT_LOADS for lim in LIMITS
             for sd in HELDOUT_SEEDS]
    jobs += [(v, UNHOLDABLE, 2.4, sd, "unholdable") for v in ("int1", "adaptive", "oracle")
             for sd in HELDOUT_SEEDS[:3]]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        res = list(ex.map(_work_entry, jobs, chunksize=1))
    if len({r["code"]["code_hash"] for r in res}) != 1:
        raise RuntimeError("workers saw different code hashes")
    book = RunBook(ENTRY)
    book.code, book.env, book.defs = res[0]["code"], res[0]["env"], res[0]["defs"]
    for r in res:
        book.runs.update(r["records"])
    rows = [r["row"] for r in res]

    # --- tuning: choose the integral rate (median primary over tuning runs) ---
    tune = {v: med([r["primary_deg"] for r in rows if r["split"] == "tuning" and r["variant"] == v])
            for v in variants[:3]}
    marg = {k: margins_for(k) for k in WI}
    meets = {v: marg[float(v[3:])][0][1] >= 45 and marg[float(v[3:])][0][2] >= 6
             and marg[float(v[3:])][1][1] >= 30 and marg[float(v[3:])][1][2] >= 6 for v in variants[:3]}
    # The comparator must meet the declared loop criteria ("verifying applicable
    # loop/fault constraints"); a variant that fails them is reported, not selected.
    best = min((v for v in variants[:3] if meets[v]), key=tune.get)

    # --- held-out: paired by (load, limit, seed) ---
    H = [r for r in rows if r["split"] == "heldout"]
    key = lambda r: (tuple(r["load"]), r["limit"], r["seed"])
    by = {v: {key(r): r for r in H if r["variant"] == v} for v in variants}
    ref = best                                                 # stronger deterministic comparator
    def paired(v):
        red = []
        for k, r in by[v].items():
            b = by[ref][k]
            if math.isfinite(r["primary_deg"]) and math.isfinite(b["primary_deg"]) and b["primary_deg"] > 0:
                red.append(1 - r["primary_deg"] / b["primary_deg"])
        return red
    out_rows = []
    for v in variants:
        rs = list(by[v].values())
        red = paired(v) if v != ref else [0.0] * len(rs)
        lost = sum(1 for k, r in by[v].items() if by[ref][k]["completed"] and not r["completed"])
        usable = sum(1 for r in rs if r.get("learn_usable_max", 0.0) > 0.5)
        out_rows.append([v + (" (DIAGNOSTIC ORACLE)" if v.startswith("oracle") else "") + (" ← comparator" if v == ref else ""),
                         f"{med([r['primary_deg'] for r in rs]):.3f}",
                         "—" if v == ref else f"{100 * med(red):+.1f} %",
                         "—" if v == ref else f"{100 * min(red):+.1f} / {100 * max(red):+.1f} %",
                         f"{sum(r['completed'] for r in rs)}/{len(rs)}", str(lost),
                         f"{usable}/{len(rs)}" if v == "adaptive" else "—",
                         f"{sum(r['wd_trips'] for r in rs)}", f"{sum(r['suspended'] for r in rs)}",
                         f"{med([r['clip_pct'] for r in rs]):.1f} %",
                         f"`{run_set_id([r['run_id'] for r in rs])}`"])
    per_case = []
    for ld in HELDOUT_LOADS:
        for lim in LIMITS:
            cells = [f"({ld[0]:+.2f}, {ld[1]:+.2f})", f"{lim}"]
            for v in variants:
                rs = [by[v][(ld, lim, sd)] for sd in HELDOUT_SEEDS]
                cells.append(f"{med([r['primary_deg'] for r in rs]):.3f} ({sum(r['completed'] for r in rs)}/5)")
            per_case.append(cells)
    U = [r for r in rows if r["split"] == "unholdable"]
    urows = [[r["variant"] + (" (ORACLE)" if r["variant"] == "oracle" else ""), r["seed"],
              "yes" if r["rejected"] else "no", "yes" if r["suspended"] else "no",
              f"{r['fallback_frac']:.0%}", f"{r['progress']:.0%}", "yes" if r["completed"] else "no",
              ", ".join(r["events"]) or "none", f"`{r['run_id']}`"] for r in U]

    orc = med(paired("oracle")); orf = med(paired("oracle_ff"))
    adaptive_rows = list(by["adaptive"].values())
    adaptive_red = med(paired("adaptive"))
    adaptive_usable = sum(1 for r in adaptive_rows if r.get("learn_usable_max", 0.0) > 0.5)
    adaptive_lost = sum(1 for k, r in by["adaptive"].items()
                        if by[ref][k]["completed"] and not r["completed"])
    adaptive_wd = sum(r["wd_trips"] for r in adaptive_rows)
    adaptive_suspended = sum(r["suspended"] for r in adaptive_rows)
    adaptive_rejected = sum(r["rejected"] for r in adaptive_rows)
    gate = 0.10
    ceiling_verdict = ("**The ceiling does not reach the registered gate.** Even the oracle, which knows the true "
               "two-parameter load, improves the registered primary metric by less than 10 % (median, paired) "
               "over the stronger deterministic comparator; an estimator of the same law can at best approach it. "
               "Rejecting the adaptive feature is therefore supported by this evidence, not only by its absence."
               if max(orc, orf) < gate else
               "**The ceiling exceeds the registered gate.** The oracle improves the registered primary metric by "
               f"at least 10 % (median, paired) over the stronger deterministic comparator. This establishes "
               "potential headroom but is not deployable controller evidence.")
    adaptive_gate = (adaptive_red >= gate and adaptive_usable >= 0.8 * len(adaptive_rows)
                     and adaptive_lost == 0 and adaptive_wd == 0 and adaptive_suspended == 0
                     and adaptive_rejected == 0)
    adaptive_verdict = (f"**The adaptive candidate {'passes' if adaptive_gate else 'fails'} the registered gate.** "
                        f"Its median paired reduction is {100 * adaptive_red:+.1f} % against `int1`; "
                        f"{adaptive_usable}/{len(adaptive_rows)} held-out runs obtain a usable estimate "
                        f"(required ≥80 %), it loses {adaptive_lost} comparator-completed sequences, and it "
                        f"records {adaptive_wd} watchdog trips, {adaptive_suspended} suspensions, and "
                        f"{adaptive_rejected} request rejections. "
                        + ("The candidate is eligible for adoption review." if adaptive_gate else
                           "The deterministic baseline remains the current controller; a redesigned candidate "
                           "would require a new registered comparison."))
    verdict = ceiling_verdict + "\n\n" + adaptive_verdict
    fp_full, _ = baseline_fingerprint()
    g = MF.git_state(MF.ROOT, [])
    head = ["# Task 3 decision input: known-load ceiling (generated)", "",
            "Generated by `python -m exp.task3_l4`. **Simulated.** Protocol, loads, sequences, seeds, primary "
            "metric and the 10 % gate are those pre-registered in `docs/plans/task3-learning.md` §8; the adaptive "
            "candidate uses the exact frozen L1 estimator configuration through the L3 adapter. Oracle "
            "rows use the true load and are **not deployable**.", "",
            f"Frozen baseline fingerprint `{fp_full[:16]}`; git `{(g or {}).get('commit', '?')[:12]}`; "
            f"{len(book.runs)} runs, rows cite run-set ids in [task3_ceiling_runs.json](task3_ceiling_runs.json).", "",
            "## Integral-rate selection (tuning loads and seeds only)", "",
            table(["Variant", "Median primary ° (tuning)", "PM / GM design 7 ms", "PM / GM 11 ms corner",
                   "Meets declared criteria"],
                  [[v, f"{tune[v]:.3f}", f"{marg[float(v[3:])][0][1]:.1f}° / {marg[float(v[3:])][0][2]:.1f} dB",
                    f"{marg[float(v[3:])][1][1]:.1f}° / {marg[float(v[3:])][1][2]:.1f} dB",
                    "yes" if meets[v] else "**no** (not eligible as comparator)"] for v in variants[:3]]), "",
            f"Selected stronger deterministic comparator: **{ref}** (declared criteria: PM ≥ 45° / GM ≥ 6 dB at "
            "7 ms and PM ≥ 30° / GM ≥ 6 dB at the corner; a variant failing them is reported, not excluded "
            "silently).", "",
            "## Held-out result (5 loads × 2 limits × seeds 101–105, paired)", "",
            table(["Controller", "Median primary °", "Median paired reduction vs comparator",
                   "Min / max paired reduction", "Test sequence completed", "Completions lost vs comparator",
                   "Usable estimate",
                   "WD trips", "Runs suspended", "At limit (median)", "Run set"], out_rows), "",
            verdict, "",
            "## Per case: median primary ° (completed / 5)", "",
            table(["Load (θs, θc) N·m", "Limit A"] + list(variants), per_case), "",
            "## Unholdable load (0, +0.40) N·m at 2.4 A: rejection check", "",
            table(["Controller", "Seed", "Rejected", "Suspended", "Fallback", "Net progress", "Completed",
                   "Events", "run_id"], urows), "",
            "The primary metric excludes samples in drive fallback; faults, suspensions and completions are "
            "reported beside it so a stopped controller cannot look good on it."]
    results = dict(tuning=tune, comparator=ref, meets_criteria=meets,
                   adaptive_configuration=dict(estimator="PayloadEstimator", stationarity_counts=2,
                                               worker_delay_ms=[2.0, 8.0], feed_forward_only=True),
                   margins={str(k): v for k, v in marg.items()}, rows=rows,
                   adaptive_median_reduction=adaptive_red,
                   adaptive_usable_heldout=adaptive_usable,
                   adaptive_gate_passed=adaptive_gate,
                   adaptive_lost_vs_comparator=adaptive_lost,
                   adaptive_watchdog_trips=adaptive_wd,
                   adaptive_suspended=adaptive_suspended,
                   adaptive_rejected=adaptive_rejected,
                   oracle_median_reduction=orc, oracle_ff_median_reduction=orf, gate=gate)

    def check(stage):
        book.check_unchanged()
        if baseline_fingerprint()[0][:16] != FROZEN:
            raise RuntimeError("baseline changed during evaluation")
    with MF.staged_publish(ROOT / "report", staging_root=Path(tempfile.gettempdir()) / "task3_ceiling_stage",
                           check=check) as stage:
        (stage / "task3_ceiling_numbers.md").write_text("\n".join(head) + "\n")
        MF.write_json(results, stage / "task3_ceiling_results.json")
        MF.write_json(book.record(), stage / "task3_ceiling_runs.json")
    print("oracle median reduction %.3f, oracle_ff %.3f, comparator %s" % (orc, orf, ref), flush=True)


if __name__ == "__main__":
    main()
