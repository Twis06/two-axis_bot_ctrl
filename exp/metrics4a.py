"""Packet 4A demonstration: new metrics + manifests + staged publish.

    python -m exp.metrics4a --out <dir>

Runs a few scenarios with the CURRENT working controller (not a frozen
baseline) and two diagnostic comparators that do nothing useful, writes one
JSON record (metrics + manifest) per run and a markdown table into a staging
directory, and publishes them to <dir> only if every run succeeded, every
run was reproduced FROM ITS RECORD ALONE (config, requests, controller and
supervisor options, seed rebuilt by exp.manifest.rebuild_run) with an identical
run_id and byte-identical metrics, and no source file changed meanwhile.
Deliberately not wired into task2_eval.py; it never writes under report/.
"""
import argparse
import math
from dataclasses import replace
from pathlib import Path

from ctrl.baseline import BaselineController
from ctrl.interfaces import Command, Controller
from ctrl.supervisor import DriveSupervisor
from exp import manifest as MF
from exp import motions as M
from exp import scenarios as S
from exp.common import run
from sim import metrics as SM
from sim.config import SimConfig

# exp modules this entry point uses beyond exp.manifest.EXP_CORE (closed source set).
PROVENANCE_SOURCES = ()


class StationaryDiagnostic(Controller):
    """Diagnostic comparator, never a design: holds zero current and sends the
    measured position as its reference, so the drive watchdog never trips. It
    reports telemetry that CLAIMS perfect accepted tracking on the wall clock
    (q_c = measured q, lag 0). Governed error is then ~0; the 4A metrics must
    still score it as not tracking and not completing."""
    name = "stationary_diagnostic"

    def update(self, ctx, fb):
        q = fb.q if fb is not None else 0.0
        self.telemetry = dict(q_c=q, gov_s=1.0, gov_lag=0.0, gov_status=0.0, gov_rejected=0.0,
                              request_rejected=0.0, host_fallback=0.0, tau_ff=0.0, tau_fb=0.0, sat=0.0)
        return Command(ctx.t, 0.0, q)


class RejectingDiagnostic(Controller):
    """Diagnostic comparator: rejects every request (explicit, truthful
    telemetry: rejected, host fallback, frozen path clock) and asks the drive
    for fallback. Correct fault handling, zero useful motion."""
    name = "rejecting_diagnostic"

    def reset(self, cfg):
        self.telemetry = {}
        self.t0 = None

    def update(self, ctx, fb):
        q = fb.q if fb is not None else 0.0
        self.t0 = ctx.t if self.t0 is None else self.t0
        self.telemetry = dict(q_c=q, gov_s=0.0, gov_lag=ctx.t - self.t0, gov_status=5.0, gov_rejected=1.0,
                              request_rejected=1.0, host_fallback=1.0, tau_ff=0.0, tau_fb=0.0, sat=0.0)
        return Command(ctx.t, 0.0, q, valid=False)


def no_coordination_derate(base):
    """Task 2 fault case 'Same derate without yaw coordination' (run with
    governed_yaw=False): the host rejects and latches the request."""
    c = S.run_c(base)
    return replace(c, name="C-derate-nocoord",
                   cfg=c.cfg.with_(drive=dict(derate_schedule=((3, 2.4),)), plant=dict(k_yv=.012, k_ya=.0012)),
                   note="C, derate to 2.4 A at 3 s, coupling x1.5, no yaw coordination")


def cases(base):
    a, e = S.run_a(base), S.run_e(base)
    m1, m2, m3 = M.all_motions(base)
    B = BaselineController
    return [  # (scenario, controller factory, governed_yaw, seed)
        (a, B, True, 1), (e, B, True, 1), (no_coordination_derate(base), B, False, 7),
        (m1, B, True, 1), (m2, B, True, 1), (m3, B, True, 1),
        (m1, lambda: B(use_governor=False), True, 1),
        (m1, StationaryDiagnostic, True, 1), (m1, RejectingDiagnostic, True, 1),
        (a, StationaryDiagnostic, True, 1),
    ]


def run_one(sc, make, governed_yaw, seed, sup=None):
    ctrl = make() if callable(make) else make
    sup = sup or DriveSupervisor()            # both described before reset() mutates them
    man = MF.make_manifest(sc, ctrl, seed, supervisor=sup, governed_yaw=governed_yaw, entry=__spec__.name)
    log, _ = run(sc, lambda: ctrl, seed=seed, supervisor=sup, governed_yaw=governed_yaw)
    ev = SM.evaluate(log, ref=sc.roll, yaw_request=sc.yaw, T_request=getattr(sc, "t_request", None),
                     waypoints=getattr(sc, "waypoints", None))
    return man, MF.jsonable(ev)


def fmt(x, nd=2, pct=False):
    if isinstance(x, dict):           # tagged non-finite float from the JSON record
        x = MF._untag(x)
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "–"
    return f"{100 * x:.0f}%" if pct else f"{x:.{nd}f}"


def yaw(y):
    if not (y.get("original_amp") or 0) > 0:
        return "no yaw"
    return (f"{y['original_amp_end']:.0f} → {y['delivered_amp_end']:.0f} ({y['n_scale_requests']})"
            if y["coordination_available"] else f"{y['original_amp_end']:.0f} → {y['delivered_amp_end']:.0f} (no planner)")


def row(man, ev):
    e, p, st, c, f = ev["errors"], ev["progress"], ev["strata"], ev["current"], ev["faults"]
    comp = ev.get("completion")
    ctime = "–" if comp is None else (f"{comp['t_complete']:.2f} / {comp['t_request']:.2f}"
                                      if comp["completed"] else f"not completed ({comp['reached']}/{comp['waypoints']})")
    strata = " / ".join(f"{st[k]['share_pct']:.0f}" for k in
                        ("startup", "normal", "reshaping", "recovery", "suspended", "fallback", "rejected"))
    opts = man["run"]["controller"]["options"]
    name = man["run"]["controller"]["name"] + ("" if opts.get("use_governor", True) else " (use_governor=False)")
    return [man["run"]["scenario"], name, str(man["run"]["seed"]),
            f"{fmt(e['orig_rms'])} / {fmt(e['gov_rms'])} / {fmt(e['path_rms'])} / {fmt(e['geo_rms'])}",
            fmt(e["path_rms_request"]),
            fmt(p["progress"], pct=True), ctime, strata,
            f"{fmt(c['sat_pct'], 1)}% / {c['sat_entries']}", f"{f['counts'].get('watchdog_trip', 0)} / {f['rearms']}",
            yaw(ev["yaw"]),
            "yes" if ev["tracked"] else "no: " + "; ".join(ev["not_tracked_because"]), f"`{man['run_id']}`"]


HEADERS = ["Case", "Controller", "Seed", "RMS ° orig / gov / path / geo (whole run)", "Path RMS ° request window",
           "Net progress", "Complete s (actual / requested)",
           "Time % startup / normal / reshape / recovery / suspended / fallback / rejected", "At limit / entries",
           "WD trips / re-arms",
           "Yaw amp ° requested → delivered (scale requests)",
           "Tracked", "run_id"]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", required=True, help="publish directory (outside report/)")
    ap.add_argument("--note", default="", help="provenance note for the table header (e.g. snapshot origin)")
    args = ap.parse_args(argv)
    out = Path(args.out).resolve()
    if (MF.ROOT / "report") in out.parents or out == MF.ROOT / "report":
        raise SystemExit("4A demo output must not go under report/ (evidence is regenerated after the freeze)")
    base = SimConfig()
    first, hashes = None, set()

    def unchanged(stage):
        bad = MF.changed_sources(first)
        if bad or len(hashes) != 1:
            raise RuntimeError(f"source files changed during evaluation: {bad or sorted(hashes)}")

    with MF.staged_publish(out, check=unchanged) as stage:
        rows, ids = [], []
        for sc, make, gy, seed in cases(base):
            man, ev = run_one(sc, make, gy, seed)
            first = first or man
            hashes.add(man["code"]["code_hash"])
            MF.write_json(dict(manifest=man, metrics=ev), stage / "runs" / f"{man['run_id']}.json")
            rows.append(row(man, ev))
            ids.append(man["run_id"])
            print("done", sc.name, man["run"]["controller"]["name"], flush=True)
        # Reproducibility from the record alone: rebuild scenario, controller,
        # supervisor, yaw coordination and seed from runs/<id>.json, re-run, and
        # require the same run_id and byte-identical metrics.
        reproduced = 0
        for rid in ids:
            rec = MF.read_json(stage / "runs" / f"{rid}.json")
            sc2, ctrl2, sup2, gy2, seed2 = MF.rebuild_run(rec["manifest"]["run"])
            man2, ev2 = run_one(sc2, ctrl2, gy2, seed2, sup=sup2)
            if man2["run_id"] != rid or MF.canonical(ev2) != MF.canonical(rec["metrics"]):
                raise RuntimeError(f"run {rid} was not reproduced from its record")
            reproduced += 1
        g = first["code"]["git"]
        lines = ["# Packet 4A demo (Simulated, pre-freeze working controller; not final evidence)", "",
                 f"Command: `python -m exp.metrics4a --out {out}`. Metrics version {SM.METRICS_VERSION}; "
                 f"code hash `{first['code']['code_hash'][:16]}`; git {g['commit'] and g['commit'][:10]} "
                 f"dirty={g['dirty']} (dirty sources: {', '.join(g['dirty_sources'] or []) or 'none'}).", "",
                 *([args.note, ""] if args.note else []),
                 "| " + " | ".join(HEADERS) + " |", "|" + "---|" * len(HEADERS)]
        lines += ["| " + " | ".join(r) + " |" for r in rows]
        lines += ["", f"Reproduced from the record alone (exp.manifest.rebuild_run: config, requests, controller "
                  f"and supervisor options, seed) with identical run_id and byte-identical metrics: "
                  f"{reproduced}/{len(ids)} runs. Publication aborts on any mismatch.",
                  "Each run_id names `runs/<run_id>.json` (manifest + full metrics). Definitions: "
                  "`sim.metrics.METRIC_DEFS`. Thresholds are project assumptions."]
        (stage / "4A_demo.md").write_text("\n".join(lines) + "\n")
    print("published", out)


if __name__ == "__main__":
    main()
