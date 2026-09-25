"""Task 3 R3: supplementary adaptive challenge matrix (registered before execution).

    python -m exp.task3_challenges --register    # writes the registration only
    python -m exp.task3_challenges --out DIR      # runs it (refuses without the registration)

These are SUPPLEMENTARY VALIDATION runs, not original held-out discovery data.
They pair the exact frozen L4 candidate (AdaptiveController with the frozen L1
estimator, est_seed = seed, default stationarity gate) against int1, the frozen
baseline, on the registered L4 calibration + test sequence. Each perturbation
starts in the TEST phase, after calibration, so an estimator that trained during
calibration is exercised while it may be applying a correction. If it is not
usable when a challenge starts, the challenge is reported as unexercised for
the learned correction, never as a pass of its active behaviour. No gate is
widened to force engagement.
"""
import argparse
import json
import math
import os
import tempfile
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from exp import manifest as MF
from exp import task3_l4 as T
from exp.evidence import RunBook, baseline_fingerprint, run_set_id
from exp.motions import FiniteMotion
from sim.trajectories import RampedSine

ROOT = Path(__file__).resolve().parents[1]
ENTRY = "exp.task3_challenges"
PROVENANCE_SOURCES = ("exp/evidence.py", "exp/task3_l4.py", "exp/task3_gate.py")
REG_PATH = ROOT / "report" / "task3_challenges_registration.json"
R = math.radians
TEST_START = 9.0                   # requested path time of the first test move

REGISTRATION = dict(
    packet="R3 supplementary adaptive challenges",
    label="supplementary validation, not held-out discovery data",
    candidate=dict(variant="adaptive", controller="ctrl.adaptive.AdaptiveController",
                   estimator="ctrl.payload_estimator.PayloadEstimator (frozen L1 settings)",
                   est_seed="= run seed", stationarity="frozen default (2 encoder counts)"),
    comparator="int1 (frozen baseline, wi_ratio 0.1)",
    sequence="docs/plans/task3-learning.md §8 calibration + test sequence (exp.task3_l4.sequence)",
    loads=[[0.0, 0.18], [0.06, 0.14]], limit_A=3.2, seeds=[201, 202, 203],
    challenges={
        "yaw_B": "yaw 75 deg at 1.5 Hz, ramped over 1 s from t = 9 s (test phase only)",
        "yaw_C": "yaw 75 deg at 2.2 Hz, ramped over 1 s from t = 9 s (test phase only)",
        "feedback_outage": "feedback-only loss for 60 ms at t = [12.5, 12.56) s (second test dwell)",
        "derate": "current limit 3.2 -> 2.4 A at t = 12.5 s",
        "payload_change": "NOT EXECUTABLE in the frozen simulator (no time-varying plant parameters); "
                          "reported as missing, so the gate is incomplete",
    },
    scoring=dict(primary="governed RMS over the first 0.5 s of each test dwell (as L4)",
                 completion="R1 phase-correct test waypoints", onset_s=dict(yaw_B=9.0, yaw_C=9.0,
                 feedback_outage=12.5, derate=12.5),
                 observations=["learned correction usable at onset and during the challenge",
                               "correction disable / re-enable times", "applied correction bound",
                               "current-target compliance", "fault timeline", "delivered progress"]),
)
EXECUTABLE = ("yaw_B", "yaw_C", "feedback_outage", "derate")

# R3 review M1: the registered onsets (9 s, 12.5 s) precede the frozen estimator's typical
# first usable time (L4 median 16.6 s), so the correction was rarely active. This AMENDMENT
# is registered separately, before it runs, and repeats the same challenges starting at
# 17.5 s (during the final test move). It supplements; it does not replace the original.
AMEND_PATH = ROOT / "report" / "task3_challenges_late_registration.json"
AMENDMENT = dict(
    packet="R3 supplementary adaptive challenges, late-onset amendment",
    label="supplementary validation, not held-out discovery data; registered after the original "
          "challenge results (onset chosen from the L4 first-usable times), before this amendment ran",
    amends="report/task3_challenges_registration.json",
    reason="original onsets precede the candidate's typical first usable time (median 16.6 s over the "
           "37 ever-usable L4 runs), leaving yaw B/C unexercised",
    candidate=REGISTRATION["candidate"], comparator=REGISTRATION["comparator"],
    loads=REGISTRATION["loads"], limit_A=REGISTRATION["limit_A"], seeds=REGISTRATION["seeds"],
    challenges={
        "yaw_B@late": "yaw 75 deg at 1.5 Hz, ramped over 1 s from t = 17.5 s",
        "yaw_C@late": "yaw 75 deg at 2.2 Hz, ramped over 1 s from t = 17.5 s",
        "feedback_outage@late": "feedback-only loss for 60 ms at t = [17.5, 17.56) s",
        "derate@late": "current limit 3.2 -> 2.4 A at t = 17.5 s",
    },
    scoring=dict(same_as="original registration", onset_s={k: 17.5 for k in
                 ("yaw_B@late", "yaw_C@late", "feedback_outage@late", "derate@late")}),
)
LATE = 17.5


def challenge_scenario(name, load, lim):
    sc, test_wp = T.scenario(load, lim, f"R3 {name} {load} {lim}A")
    cfg, yaw = sc.cfg, sc.yaw
    base, late = name.split("@")[0], name.endswith("@late")
    t_yaw, t_evt = (LATE, LATE) if late else (TEST_START, 12.5)
    if base == "yaw_B":
        yaw = RampedSine(R(75), 1.5, t_ramp=1.0, t0=t_yaw)
    elif base == "yaw_C":
        yaw = RampedSine(R(75), 2.2, t_ramp=1.0, t0=t_yaw)
    elif base == "feedback_outage":
        cfg = cfg.with_(timing=dict(feedback_blackout=((t_evt, t_evt + 0.06),)))
    elif base == "derate":
        cfg = cfg.with_(drive=dict(derate_schedule=((t_evt, 2.4),)))
    else:
        raise ValueError(name)
    segs = sc.roll.segs[-len(test_wp):]
    windows = tuple((sg[0], segs[k + 1][0] if k + 1 < len(segs) else None) for k, sg in enumerate(segs))
    t_req = T.sequence()[0].segs[-1][0] + 1.0
    return FiniteMotion(sc.name, cfg, sc.roll, yaw, sc.note + f"; R3 challenge {name}", test_wp, t_req, windows)


_BOOK = None


def fnum(x, nd):
    return "—" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{nd}f}"


def _transitions(log, t_from):
    u = np.nan_to_num(np.asarray(log.get("c_learn_usable", np.zeros(len(log.t))), float)) > 0.5
    t = np.asarray(log.t)
    d = np.flatnonzero(np.diff(u.astype(int)) != 0)
    return [(float(t[k + 1]), "enabled" if u[k + 1] else "disabled") for k in d if t[k + 1] >= t_from - 1.0][:8]


def _work(job):
    global _BOOK
    if _BOOK is None:
        _BOOK = RunBook(ENTRY)
    name, variant, load, seed = job
    fm = challenge_scenario(name, load, REGISTRATION["limit_A"])
    before = set(_BOOK.runs)
    log, stats, rid, ev = _BOOK.run(fm, lambda: T.make(variant, load, seed), seed=seed)
    _, dwells = T.sequence()
    prim, n = T.primary(log, dwells)
    comp = ev.get("completion") or {}
    onset = {**REGISTRATION["scoring"]["onset_s"], **AMENDMENT["scoring"]["onset_s"]}[name]
    learn = np.nan_to_num(np.asarray(log.get("c_learn_usable", np.zeros(len(log.t))), float))
    k0 = min(int(np.searchsorted(log.t, onset)), len(log.t) - 1)
    row = dict(challenge=name, variant=variant, load=list(load), limit=REGISTRATION["limit_A"], seed=seed,
               run_id=rid, primary_deg=prim, primary_samples=n, completed=bool(comp.get("completed")),
               missing_waypoints=comp.get("missing_waypoints"), waypoint_visits=comp.get("waypoint_visits"),
               t_complete=comp.get("t_complete"), progress=float(ev["progress"]["progress"]),
               wd_trips=stats["wd_trips"], suspended=bool(np.nanmax(log.c_suspended) > 0.5),
               rejected=bool(np.nanmax(log.c_request_rejected) > 0.5),
               fallback_frac=float(np.mean(log.mode > 0)), learn_usable_max=float(learn.max()),
               learn_usable_at_onset=bool(learn[k0] > 0.5), transitions=_transitions(log, onset),
               events=[e[1] for e in log.events][:8], **T.gate_evidence(log, dwells, t_from=onset))
    new = {k: _BOOK.runs[k] for k in _BOOK.runs if k not in before}
    return dict(row=row, records=new, code=_BOOK.code, env=_BOOK.env, defs=_BOOK.defs)


def _work_entry(job):
    from exp import task3_challenges as C
    return C._work(job)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--register", action="store_true")
    ap.add_argument("--register-amendment", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args(argv)
    if args.register:
        if REG_PATH.exists():
            raise SystemExit(f"{REG_PATH} exists: a registration is never regenerated")
        MF.write_json(REGISTRATION, REG_PATH)
        print(f"registered {REG_PATH}")
        return
    if args.register_amendment:
        if AMEND_PATH.exists():
            raise SystemExit(f"{AMEND_PATH} exists: a registration is never regenerated")
        MF.write_json(AMENDMENT, AMEND_PATH)
        print(f"registered {AMEND_PATH}")
        return
    for path, reg in ((REG_PATH, REGISTRATION), (AMEND_PATH, AMENDMENT)):
        if not path.exists() or MF.read_json(path) != json.loads(json.dumps(MF.jsonable(reg))):
            raise SystemExit(f"{path.name} missing or differs from the code: refusing to run")
    if baseline_fingerprint()[0][:16] != T.FROZEN:
        raise SystemExit("baseline fingerprint is not the frozen one")
    names = EXECUTABLE + tuple(f"{c}@late" for c in EXECUTABLE)
    jobs = [(c, v, tuple(ld), sd) for c in names for v in ("int1", "adaptive")
            for ld in REGISTRATION["loads"] for sd in REGISTRATION["seeds"]]
    expected = [(c, list(ld), REGISTRATION["limit_A"], sd) for c in names
                for ld in REGISTRATION["loads"] for sd in REGISTRATION["seeds"]]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        res = list(ex.map(_work_entry, jobs, chunksize=1))
    if len({r["code"]["code_hash"] for r in res}) != 1:
        raise RuntimeError("workers saw different code hashes")
    book = RunBook(ENTRY)
    book.code, book.env, book.defs = res[0]["code"], res[0]["env"], res[0]["defs"]
    for r in res:
        book.runs.update(r["records"])
    rows = [r["row"] for r in res]
    out = Path(args.out).resolve() if args.out else ROOT / "report" / "staged" / "R3"
    rel = lambda p: Path(os.path.relpath(p, out)).as_posix()       # links valid from the output directory
    lines = ["# Task 3 R3 — supplementary adaptive challenges (generated)", "",
             "Generated by `python -m exp.task3_challenges`. **Simulated.** Registered in "
             f"[task3_challenges_registration.json]({rel(REG_PATH)}) before execution. "
             "The late-onset amendment is registered separately in "
             f"[task3_challenges_late_registration.json]({rel(AMEND_PATH)}). "
             "Supplementary validation, not held-out discovery data.", "",
             f"Baseline fingerprint `{baseline_fingerprint()[0][:16]}`; {len(book.runs)} runs.", "",
             T.table(["Challenge", "Variant", "Median primary °", "Completed", "WD trips", "Suspended",
                      "Usable at onset", "Usable share after onset", "Max applied correction N·m",
                      "Applied while unusable", "Max host command over limit A (pre-clamp)", "Run set"],
                     [[c, v, f"{T.med([r['primary_deg'] for r in rs]):.3f}",
                       f"{sum(r['completed'] for r in rs)}/{len(rs)}", str(sum(r['wd_trips'] for r in rs)),
                       str(sum(r['suspended'] for r in rs)),
                       f"{sum(r['learn_usable_at_onset'] for r in rs)}/{len(rs)}",
                       fnum(T.med([r['learn_usable_during_challenge'] for r in rs
                                   if r['learn_usable_during_challenge'] is not None]), 2),
                       fnum(max([r['learn_applied_max_Nm'] for r in rs if r['learn_applied_max_Nm'] is not None],
                                default=None), 3),
                       "—" if all(r['applied_while_unusable'] is None for r in rs)
                       else str(sum(r['applied_while_unusable'] or 0 for r in rs)),
                       f"{max(r['command_over_limit'] for r in rs):.3g}",
                       f"`{run_set_id([r['run_id'] for r in rs])}`"]
                      for c in names for v in ("int1", "adaptive")
                      for rs in [[r for r in rows if r['challenge'] == c and r['variant'] == v]]]), "",
             "Payload change: registered as not executable in the frozen simulator; no rows. "
             "The adoption gate therefore stays `incomplete` for challenges regardless of these results."]

    def check(stage):
        book.check_unchanged()
    with MF.staged_publish(out, staging_root=Path(tempfile.gettempdir()) / "task3_ch_stage", check=check) as st:
        (st / "task3_challenges_numbers.md").write_text("\n".join(lines) + "\n")
        MF.write_json(dict(registration=REGISTRATION, amendment=AMENDMENT, expected_keys=expected, rows=rows),
                      st / "task3_challenges_results.json")
        MF.write_json(book.record(), st / "task3_challenges_runs.json")
    print(f"published {len(rows)} rows to {out}")


if __name__ == "__main__":
    main()
