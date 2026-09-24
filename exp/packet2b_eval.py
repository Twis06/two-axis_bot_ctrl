"""Packet 2B before/after evidence on matched seeds (diagnostic, pre-freeze).

Run from the project root against any code tree on PYTHONPATH:
    PYTHONPATH=<tree> python3 exp/packet2b_eval.py <out.json> [section ...]
Sections: outages, ae, ae_plan, montecarlo, info, sat, noauth. The same file runs against
the pre-2B snapshot (git archive a03bb1b) and the working tree; attributes that
only exist after 2B are read with getattr defaults.
"""
import json
import math
import sys
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace

import numpy as np

from ctrl.baseline import BaselineController
from ctrl.supervisor import DriveSupervisor
from exp import scenarios as S
from exp.common import run
from sim.config import SimConfig
from sim.engine import simulate
from sim.trajectories import GovernedYaw, Hold, MinJerkSequence

DEG = 180 / math.pi
R = math.radians
SEEDS = (1, 2, 3, 4, 5)


def ctrl(mode=None):
    if mode is None:
        return BaselineController()
    return BaselineController(yaw_info=mode)


def fault_stats(log, c, t0):
    """Fault sequence and loaded fallback behaviour after an outage starting at t0."""
    fbk = log.mode > 0
    after = log.t >= t0
    ev = [(round(t, 3), n) for t, n in log.events if t >= t0 - 1e-9]
    disp = 0.0
    k = np.where(fbk & after)[0]
    if k.size:
        # Largest excursion from the position at each fallback entry.
        entries = [i for i in k if i == 0 or not fbk[i - 1]]
        for i in entries:
            j = i
            while j < len(fbk) and fbk[j]:
                j += 1
            disp = max(disp, float(np.max(np.abs(log.q[i:j] - log.q[i]))))
    sig_end = log.t[-1] - float(log.c_gov_lag[-1])
    return dict(
        trips=sum(n == "watchdog_trip" for _, n in ev),
        rearms=sum(n.startswith("rearm") for _, n in ev),
        lockout=any(n == "lockout" for _, n in ev),
        events=ev[:12],
        fallback_pct=100 * float(np.mean(fbk[after])),
        fallback_disp_deg=disp * DEG,
        qd_peak_fallback=float(np.max(np.abs(log.qd[fbk & after]))) if np.any(fbk & after) else 0.0,
        i_peak=float(np.max(np.abs(log.i[after]))),
        q_peak_deg=float(np.max(np.abs(log.q[after]))) * DEG,
        peak_gov_deg=float(np.nanmax(np.abs(log.q[after] - log.c_q_c[after]))) * DEG,
        progress=sig_end / log.t[-1],
        # Original-request error well after the event, and where the axis ends up:
        # a suspended hold far from the request must not hide behind low trip counts.
        orig_rms_late_deg=float(np.sqrt(np.mean(log.err[log.t > t0 + 1.0] ** 2))) * DEG,
        q_final_deg=float(log.q[-1]) * DEG,
        suspended=bool(getattr(c, "suspended", "")),
        rejected=bool(c.request_blocked),
        incompatible=bool(getattr(c, "incompatible", "")),
    )


def outage_case(args):
    motion, payload, outage, seed = args
    base = SimConfig()
    sc = S.run_c(base) if motion == "C hold" else (S.run_e(base) if motion == "E sweep" else S.run_d(base))
    m, s_lat = payload
    cfg = replace(sc.cfg, duration=7.0).with_(plant=dict(m_payload=m, s_lat=s_lat))
    if outage:
        cfg = cfg.with_(timing={outage: ((3.0, 3.1),)})
    c = ctrl()
    log, _ = run(replace(sc, cfg=cfg), lambda: c, seed=seed)
    return dict(motion=motion, payload=payload, outage=outage or "none", seed=seed,
                **fault_stats(log, c, 3.0))


def outages():
    jobs = [(mo, pl, ou, sd) for mo, pls in (("C hold", ((0.0, 0.035), (0.7, 0.035), (0.7, -0.035))),
                                             ("D sweep", ((0.7, 0.035), (0.7, -0.035))),
                                             ("E sweep", ((0.7, 0.035), (0.7, -0.035))))
            for pl in pls for ou in ("feedback_blackout", "command_blackout", "blackout", None)
            for sd in SEEDS]
    with ProcessPoolExecutor() as ex:
        return list(ex.map(outage_case, jobs))


def ae_case(args):
    name, seed, mode = args
    sc = {s.name: s for s in S.all_runs()}[name]
    c = ctrl(mode)
    log, s = run(sc, lambda: c, seed=seed)
    return dict(run=name, seed=seed, mode=mode or "default", rms_gov=s["rms_gov"], peak_gov=s["peak_gov"],
                rms=s["rms"], speed=s["speed"], clip_pct=s["clip_pct"], events=s["events"],
                wd_trips=s["wd_trips"], yaw_scale=s["yaw_scale"],
                suspended=bool(getattr(c, "suspended", "")))


def ae(modes=(None,)):
    jobs = [(n, sd, m) for n in "ABCDE" for sd in SEEDS for m in modes]
    with ProcessPoolExecutor() as ex:
        return list(ex.map(ae_case, jobs))


def mc_case(args):
    from exp.phase2_eval import sample_cfg
    name, k = args
    rng = np.random.default_rng(2024)
    # Same sampling order as exp/task2_eval.py (five runs x twenty draws).
    for mk in (S.run_a, S.run_b, S.run_c, S.run_d, S.run_e):
        for j in range(20):
            base, _ = sample_cfg(rng, SimConfig())
            base = base.with_(plant=dict(k_e=base.plant.k_t))
            sc = mk(base)
            if sc.name in ("D", "E"):
                sc = replace(sc, cfg=sc.cfg.with_(plant=dict(m_payload=float(rng.uniform(.3, 1.0)))))
            sc = replace(sc, cfg=replace(sc.cfg, duration=6))
            if sc.name == name and j == k:
                c = ctrl()
                log, s = run(sc, lambda: c, seed=100 + k)
                return dict(run=name, k=k, peak_gov=s["peak_gov"], rms_gov=s["rms_gov"],
                            clip_pct=s["clip_pct"], events=s["events"], wd_trips=s["wd_trips"],
                            speed=s["speed"], suspended=bool(getattr(c, "suspended", "")))


def montecarlo():
    jobs = [(n, k) for n in "ABCDE" for k in range(20)]
    with ProcessPoolExecutor() as ex:
        return list(ex.map(mc_case, jobs))


def info_case(args):
    name, mode, mismatch, seed = args
    try:
        from sim.trajectories import FollowingYaw
    except ImportError:
        FollowingYaw = None
    sc = {s.name: s for s in S.all_runs()}[name]
    plan = GovernedYaw(sc.yaw)
    actual = FollowingYaw(plan, lag=0.010, gain=1.1) if mismatch else plan
    c = ctrl(mode)
    kw = dict(yaw_plan=plan) if mismatch else {}
    log = simulate(sc.cfg, c, sc.roll, actual, seed=seed, safety=DriveSupervisor(), **kw)
    from sim.metrics import summarize
    s = summarize(log)
    return dict(run=name, mode=mode, mismatch=mismatch, seed=seed, rms_gov=s["rms_gov"],
                peak_gov=s["peak_gov"], events=s["events"], yaw_scale=s["yaw_scale"],
                clip_pct=s["clip_pct"])


def info():
    jobs = [(n, m, mm, sd) for n in "BC" for m in ("estimate", "plan") for mm in (False, True)
            for sd in SEEDS]
    with ProcessPoolExecutor() as ex:
        return list(ex.map(info_case, jobs))


def sat():
    from sim.drive import DriveSafety
    sc = S.Scenario("step", SimConfig(duration=3.0), MinJerkSequence(0.0, [(R(30), 0.1, 5.0)]),
                    Hold(0.0), "30 deg in 0.1 s")
    out = []
    for name, make in (("AW on", lambda: BaselineController(use_governor=False, use_yaw_monitor=False)),
                       ("AW off", lambda: BaselineController(use_governor=False, use_yaw_monitor=False,
                                                         anti_windup=False))):
        log, s = run(sc, make, seed=1, supervisor=DriveSafety(cmd_timeout=0.010))
        outside = np.where(np.abs(log.err) > R(1))[0]
        out.append(dict(case=name, settle=float(log.t[outside[-1]]),
                        overshoot_deg=max(0.0, float(np.max(log.q) - R(30)) * DEG),
                        integ_max=float(np.nanmax(np.abs(log.c_integ)))))
    return out


def noauth():
    """No yaw authority. 2A M8c reproducer (plan mode, synthetic yaw with no position:
    0.4 N m predicted coupling), then the default estimate mode with physically
    consistent constant-rate yaw (50 rad/s = 0.40 N m, 60 rad/s = 0.48 N m, above
    capacity) and C yaw with a 0.7 kg +35 mm payload, all without a planner."""
    out = [dict(case="M8c plan mode, 0.4 N m", **_noauth_plan())]
    class Ramp:
        def __init__(self, w):
            self.w = w
        def eval(self, t):
            return (self.w * t, self.w, 0.0)
    for w in (50.0, 60.0):
        c = BaselineController()
        log = simulate(replace(SimConfig(), duration=1.5), c, Hold(0.0), Ramp(w), seed=1,
                       safety=DriveSupervisor())
        out.append(dict(case="estimate mode, %g rad/s" % w,
                        q_peak_deg=float(np.max(np.abs(log.q))) * DEG,
                        events=[e[1] for e in log.events][:5], rejected=bool(c.request_blocked),
                        incompatible=bool(getattr(c, "incompatible", ""))))
    sc = S.run_c(SimConfig())
    cfg = replace(sc.cfg, duration=6.0).with_(plant=dict(m_payload=0.7, s_lat=0.035))
    c = BaselineController()
    log, _ = run(replace(sc, cfg=cfg), lambda: c, seed=1, governed_yaw=False)
    out.append(dict(case="C yaw, 0.7 kg +35 mm, no planner", q_peak_deg=float(np.max(np.abs(log.q))) * DEG,
                    events=[e[1] for e in log.events][:5], rejected=bool(c.request_blocked),
                    incompatible=bool(getattr(c, "incompatible", ""))))
    return out


def _noauth_plan():
    class ConstAcc:
        def eval(self, t):
            return (0.0, 0.0, 500.0)
    try:
        c = BaselineController(yaw_info="plan")
    except TypeError:
        c = BaselineController()
    log = simulate(replace(SimConfig(), duration=1.5), c, Hold(0.0), ConstAcc(), seed=1,
                   safety=DriveSupervisor())
    return dict(q_peak_deg=float(np.max(np.abs(log.q))) * DEG, events=[e[1] for e in log.events][:5],
                rejected=bool(c.request_blocked), incompatible=bool(getattr(c, "incompatible", "")))


SECTIONS = dict(outages=outages, ae=ae, ae_plan=lambda: ae(("plan",)), montecarlo=montecarlo,
                info=info, sat=sat, noauth=noauth)

if __name__ == "__main__":
    out_path, names = sys.argv[1], sys.argv[2:] or list(SECTIONS)
    res = {}
    for n in names:
        if n in ("info", "ae_plan"):
            try:
                BaselineController(yaw_info="plan")
            except TypeError:
                continue            # pre-2B tree has no information modes
        res[n] = SECTIONS[n]()
        print("done", n, flush=True)
    with open(out_path, "w") as f:
        json.dump(res, f, indent=1)
