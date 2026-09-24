"""Packet 4B: matched validation of the frozen deterministic baseline.

    python -m exp.task4b_eval [--jobs N] [--quick]

Writes report/task4b_numbers.md, report/task4b_results.json,
report/task4b_runs.json and report/figs/task4b_*.png through
exp.manifest.staged_publish, only if the whole evaluation succeeded, the frozen
baseline fingerprint is 7d857df507c389c9 and no source file changed meanwhile.
Every simulated run goes through exp.evidence.RunBook (or run_mismatch below,
which makes the same manifest plus the plan-mismatch description in `extra`),
so every table row cites a run_id or a run-set id.

Contents (EXECUTION_PLAN.md §7, Packet 4B):
  1. matched variants on identical scenarios and seeds: frozen baseline
     (yaw estimate), plan look-ahead (optional, stronger information), legacy
     comparator; A-E x 5 seeds, finite motions A (scored as finite) and M1-M3
  2. stress set: yaw plan mismatch, asymmetric packet loss (emulated), mid-run
     derating, low bus / high resistance, loaded recovery, quantization near
     rest, combined parameter/delay corners
  3. the Task 2 D/E Monte Carlo trials regenerated and every faulted trial
     classified by the torque terms acting before its first trip
  4. explicit infeasible cases (physical, and by request policy)
  5. frequency response, generalization sweeps, saturation / fault timelines
  6. manifested re-runs of the Task 2 robustness simulations (review 2C I-B)

All results are Simulated unless labelled Calculated. Thresholds are project
assumptions (sim.metrics).
"""
import argparse
import math
import os
import tempfile
from collections import namedtuple
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
from pathlib import Path

import numpy as np

from ctrl.baseline import BaselineController
from ctrl.supervisor import DriveSupervisor
from exp import manifest as MF
from exp import motions as M
from exp import scenarios as S
from exp.evidence import KEEP, RunBook, _supervisor, baseline_fingerprint, run_set_id
from exp.phase2_eval import legacy, sample_cfg
from sim import metrics as SM
from sim import params as P
from sim.config import SimConfig
from sim.drive import DriveSafety
from sim.engine import simulate
from sim.metrics import summarize
from sim.trajectories import FollowingYaw, GovernedYaw, Hold, MinJerkSequence, RampedSine

ROOT = Path(__file__).resolve().parents[1]
ENTRY = "exp.task4b_eval"
PROVENANCE_SOURCES = ("exp/evidence.py", "exp/phase2_eval.py", "exp/task2_robustness.py")
FROZEN = "7d857df507c389c9"
SEEDS = (1, 2, 3, 4, 5)
STRESS_SEEDS = (1, 2, 3)
DEG = 180 / math.pi
R = math.radians

# ---------------------------------------------------------------------------
# Run specifications (picklable; controllers and supervisors are built in the
# worker so every run gets fresh objects)
# ---------------------------------------------------------------------------
Spec = namedtuple("Spec", "group key sc ctrl seed sup gy mismatch post")
Spec.__new__.__defaults__ = ("design", True, None, None)

VARIANTS = ("baseline", "plan", "legacy")
VARIANT_LABEL = {"baseline": "frozen baseline (yaw estimate)",
                 "plan": "plan look-ahead (optional, stronger information)",
                 "legacy": "legacy comparator"}


def make_ctrl(desc):
    """desc = name or (name, kwargs, K_scale[, attribute overrides]). Overrides
    are recorded by the manifest (controller options), so they are in run_id."""
    name, kw, g, attrs = ((desc, {}, 1.0, {}) if isinstance(desc, str) else (tuple(desc) + ({},))[:4])
    if name == "legacy":
        return legacy()
    if name == "bench":
        name = "baseline"
    if name == "plan":
        kw = dict(kw, yaw_info="plan")
    c = BaselineController(**kw)
    if g != 1.0:
        c.K *= g
    for k, v in attrs.items():
        setattr(c, k, v)
    return c


def make_sup(desc):
    if isinstance(desc, tuple):        # ("safety", kwargs): a plain DriveSafety
        return DriveSafety(**desc[1])
    return _supervisor(desc)


def variant_spec(group, key, sc, variant, seed, **kw):
    if variant == "legacy":
        return Spec(group, key, sc, "legacy", seed, "legacy", False, **kw)
    return Spec(group, key, sc, variant, seed, **kw)


# ---------------------------------------------------------------------------
# Worker
# ---------------------------------------------------------------------------
_BOOK = None


def run_mismatch(book, sc, ctrl, seed, sup, lag, gain):
    """RunBook.run with an actual yaw that follows the host's plan imperfectly
    (sim.trajectories.FollowingYaw). Same manifest, same scoring; the mismatch
    is recorded in the manifest's `extra`, so it is part of run_id."""
    extra = dict(yaw_mismatch=dict(actual="sim.trajectories.FollowingYaw(plan, lag, gain)",
                                   plan="sim.trajectories.GovernedYaw(scenario.yaw)", lag=lag, gain=gain))
    man = MF.make_manifest(sc, ctrl, seed, supervisor=sup, governed_yaw=True, entry=book.entry,
                           require_declared=True, extra=extra)
    if book.code is None:
        book.code, book.env, book.defs = man["code"], man["env"], man["metric_defs"]
    elif man["code"]["code_hash"] != book.code["code_hash"]:
        raise RuntimeError("code hash changed during the evaluation")
    plan = GovernedYaw(sc.yaw)
    log = simulate(sc.cfg, ctrl, sc.roll, FollowingYaw(plan, lag=lag, gain=gain), seed=seed, safety=sup,
                   yaw_plan=plan)
    ev = SM.evaluate(log, ref=sc.roll, yaw_request=sc.yaw, T_request=getattr(sc, "t_request", None),
                     waypoints=getattr(sc, "waypoints", None))
    rid = man["run_id"]
    book.runs[rid] = dict(run=man["run"], metrics={k: ev.get(k) for k in KEEP})
    return log, summarize(log), rid, ev


def _one(book, spec, ctrl_desc=None):
    ctrl = make_ctrl(ctrl_desc or spec.ctrl)
    sup = make_sup(spec.sup)
    if spec.mismatch:
        return run_mismatch(book, spec.sc, ctrl, spec.seed, sup, *spec.mismatch)
    return book.run(spec.sc, lambda: ctrl, seed=spec.seed, supervisor=sup, governed_yaw=spec.gy)


def fallback_excursion(log):
    """Largest |q - q(entry)| during any drive/host fallback span after start-up
    (deg): how far the axis moves while only local damping acts."""
    t = np.asarray(log.t, float)
    bad = (np.asarray(log.mode) > 0) | (np.asarray(log.get("c_host_fallback", np.zeros(len(t)))) == 1)
    worst, n0 = 0.0, int(np.argmax(~bad)) if (~bad).any() else len(t)
    for a, b in SM._span_idx(bad):
        if a < n0:
            continue
        worst = max(worst, float(np.max(np.abs(log.q[a:b] - log.q[a]))) * DEG)
    return worst


HOLD_MODE = BaselineController.MODES.index("hold")
GOV_V_MAX = 20.0          # rad/s, RollGovernor.v_max (the governor's own speed cap)


def controller_contracts(log):
    """Controller-side contracts checked from the log (not the simulator clamp
    and not the scorer's definitions). Returns counts and worst values:

    hold   between faults, a suspended hold's reference is stationary: over
           samples that are suspended, drive normal, host active and governor
           mode HOLD (the braking segment after a catch is mode STOP), q_c is
           constant within each fault epoch (epochs split at non-transient
           fault events). A new hold position after a new fault is a re-base,
           reported separately, not a stationary hold.
    ack    every rearm_after_<tracking fault> is preceded, after that fault, by
           host telemetry acknowledging the drive's fault_id (c_fault_ack).
    resume the request never resumes (c_suspended 1 -> 0) without a replan()
           since the suspension began (c_replans increments).
    reason every suspended / restricted / rejected sample is covered by a
           controller notice (log.meta['notices']) with a non-empty reason.
    jump   between consecutive host ticks with the host active, |dq_c| / ts
           stays within the governor's speed cap (no position jump).
    """
    t = np.asarray(log.t, float)
    out = dict(hold_samples=0, hold_drift_deg=0.0, hold_positions_deg=[], rebases=0,
               rearms=0, rearms_unacked=0, resumes=0, resumes_without_replan=0,
               reason_samples=0, reason_missing=0, reason_unknown=False, jump_max_rad_s=0.0, jumps=0)
    if "c_q_c" not in log or "c_gov_mode" not in log:
        return out
    qc = np.asarray(log.c_q_c, float)
    susp = np.asarray(log.get("c_suspended", np.zeros(len(t))), float) == 1
    idle = np.asarray(log.get("c_host_fallback", np.zeros(len(t))), float) == 1
    mode = np.asarray(log.mode)
    gmode = np.asarray(log.c_gov_mode, float)
    faults = sorted(float(a) for a, b in log.events if b not in SM.TRANSIENT_FAULTS and not b.startswith("rearm_after_"))
    edges = [-1.0] + faults + [float("inf")]
    hold = susp & (mode == 0) & ~idle & (gmode == HOLD_MODE) & np.isfinite(qc)
    for a, b in zip(edges[:-1], edges[1:]):
        m = hold & (t >= a) & (t < b)
        if m.any():
            v = qc[m]
            out["hold_samples"] += int(m.sum())
            out["hold_drift_deg"] = max(out["hold_drift_deg"], float(np.ptp(v)) * DEG)
            out["hold_positions_deg"].append(round(float(v[0]) * DEG, 2))
    out["rebases"] = max(0, len(out["hold_positions_deg"]) - 1)
    ack = np.asarray(log.get("c_fault_ack", np.full(len(t), np.nan)), float)
    fid = np.asarray(log.get("fault_id", np.full(len(t), np.nan)), float)
    for te, name in log.events:
        if not name.startswith("rearm_after_") or name[len("rearm_after_"):] in SM.TRANSIENT_FAULTS:
            continue
        out["rearms"] += 1
        k = int(round(te / (t[1] - t[0])))
        f_now = fid[max(0, k - 1)]
        t_f = max([f for f in faults if f <= te] or [0.0])
        w = (t >= t_f) & (t <= te)
        if not np.any(ack[w] == f_now):
            out["rearms_unacked"] += 1
    rp = np.asarray(log.get("c_replans", np.zeros(len(t))), float)
    ends = np.where(susp[:-1] & ~susp[1:])[0]
    starts = np.where(~susp[:-1] & susp[1:])[0]
    for e in ends:
        out["resumes"] += 1
        s0 = starts[starts < e]
        s0 = s0[-1] if s0.size else 0
        if not np.nanmax(rp[e + 1:e + 3]) > np.nanmin(rp[s0:e + 1]):
            out["resumes_without_replan"] += 1
    notices = getattr(log, "meta", {}).get("notices") or []
    nt = np.array([n[0] if n[0] is not None else -1.0 for n in notices], float)
    st = np.asarray(log.get("c_gov_status", np.full(len(t), np.nan)), float)
    need = susp | np.isin(st, [3, 5])
    div = T_DIV
    th = t - (np.arange(len(t)) % div) * (t[1] - t[0])
    idx = np.searchsorted(nt, th + 1e-9, side="right") - 1
    out["reason_samples"] = int(need.sum())
    if len(notices) >= 2000 and nt.size and nt[0] > 0:
        out["reason_unknown"] = True       # notice log truncated at its cap
    good = np.array([i >= 0 and bool(notices[i][2]) for i in idx]) if len(notices) else np.zeros(len(t), bool)
    out["reason_missing"] = int(np.sum(need & ~good))
    k = np.arange(0, len(t), div)
    act = ~idle[k] & np.isfinite(qc[k])
    dq = np.abs(np.diff(qc[k])) / (div * (t[1] - t[0]))
    pair = act[1:] & act[:-1]
    if pair.any():
        out["jump_max_rad_s"] = float(np.max(dq[pair]))
        out["jumps"] = int(np.sum(dq[pair] > GOV_V_MAX * 1.001))
    return out


T_DIV = 2    # drive ticks per host tick in every 4B configuration (1 kHz / 500 Hz)


def fallback_detail(log, spec):
    """For the fallback span with the largest excursion (after start-up): entry
    speed, fallback current, share at the limit, and the payload torque's sign
    relative to braking (> 0: the payload pushes along the motion, i.e. opposes
    braking). Calculated from the logged state and the true plant."""
    t = np.asarray(log.t, float)
    bad = (np.asarray(log.mode) > 0) | (np.asarray(log.get("c_host_fallback", np.zeros(len(t)))) == 1)
    n0 = int(np.argmax(~bad)) if (~bad).any() else len(t)
    best = None
    for a, b in SM._span_idx(bad):
        if a < n0:
            continue
        exc = float(np.max(np.abs(log.q[a:b] - log.q[a]))) * DEG
        if best is None or exc > best[0]:
            best = (exc, a, b)
    if best is None:
        return None
    exc, a, b = best
    pc = spec.sc.cfg.plant
    qd0 = float(log.qd[a])
    pay = -pc.tau_lat * np.cos(log.q[a:b])        # payload torque on the axis (sim.plant sign)
    return dict(t_entry=float(t[a]), duration_s=float(t[b - 1] - t[a] + (t[1] - t[0])), excursion_deg=exc,
                qd_entry=qd0, i_fallback_max=float(np.max(np.abs(log.i_tgt[a:b]))),
                at_limit_pct=100 * float(np.mean(np.abs(log.i_tgt[a:b]) >= 0.999 * log.i_lim[a:b])),
                i_lim=float(np.min(log.i_lim[a:b])),
                payload_along_motion_Nm=float(np.mean(pay) * np.sign(qd0)))


def row_stats(log, stats, rid, ev, spec=None):
    """Per-run numbers used by the tables (full evaluation is in task4b_runs.json)."""
    comp = ev.get("completion") or {}
    st = ev["strata"]
    y = ev["yaw"]
    col = lambda k: np.asarray(log[k], float) if k in log else np.zeros(len(log.t))
    return dict(
        run_id=rid, tracked=bool(ev["tracked"]), why=ev["not_tracked_because"],
        net_progress=float(ev["progress"]["progress"]), rms=stats["rms"], peak=stats["peak"],
        rms_gov=stats["rms_gov"], peak_gov=stats["peak_gov"], path_rms=ev["errors"]["path_rms_request"],
        geo_rms=ev["errors"]["geo_rms"], clip_pct=stats["clip_pct"], sat_entries=stats["sat_entries"],
        i_peak=stats["i_peak"], i_rms=stats["i_rms"], wd_trips=int(stats["wd_trips"]),
        events=sorted({e[1] for e in log.events}), n_events=len(log.events),
        completed=comp.get("completed"), t_complete=comp.get("t_complete"), t_request=comp.get("t_request"),
        voided_at=comp.get("voided_at"),
        suspended_pct=st["suspended"]["share_pct"], fallback_pct=st["fallback"]["share_pct"],
        rejected_pct=st["rejected"]["share_pct"], reshaping_pct=st["reshaping"]["share_pct"],
        coord_stop_pct=float(y.get("coord_stop_pct", 0.0)), yaw_scale=float(y.get("scale_end", 1.0) or 1.0),
        yaw_ratio=y.get("amp_ratio_end"),
        target_over_limit=float(np.max(np.abs(log.i_tgt) - log.i_lim)),
        current_over_limit=float(np.max(np.abs(log.i) - log.i_lim)),
        q_end=float(log.q[-1] * DEG), q_min=float(np.min(log.q) * DEG), q_max=float(np.max(log.q) * DEG),
        qd_peak=float(np.max(np.abs(log.qd))), fb_age_max_ms=1e3 * float(np.nanmax(log.fb_age)),
        lockout=bool(np.any(col("locked") == 1)), fb_excursion=fallback_excursion(log),
        vlim_pct=stats["vlim_pct"], contracts=controller_contracts(log),
        fallback=fallback_detail(log, spec) if spec is not None else None,
        reasons=sorted({part.strip() for key in ("suspended", "coord_stop", "rejected")
                        for r in (ev["faults"][key].get("reasons") or []) if r
                        for part in r.split(";") if part.strip()}))


# -- post-processors (return small, picklable dicts) ---------------------------
def post_mc(log, ev, spec):
    """Torque terms over the 100 ms before the first trip (Calculated from the
    logged true state and the true plant config; diagnostic only)."""
    trips = [t for t, n in log.events if n not in SM.TRANSIENT_FAULTS and not n.startswith("rearm_after_")]
    pc = spec.sc.cfg.plant
    if not trips:
        return dict(faulted=False, m_payload=pc.m_payload)
    t0 = trips[0]
    m = (log.t >= t0 - 0.1) & (log.t < t0)
    q = log.q[m]
    kt = pc.k_t
    cap = kt * log.i_lim[m]
    payload = pc.tau_lat * np.cos(q)                               # unmodelled by the host
    grav_err = (pc.tau_g - P.TAU_G) * np.sin(q)                    # gravity-model error
    cpl_true = log.tau_couple[m]
    cpl_pred = log.c_tau_cpl[m] if "c_tau_cpl" in log else np.zeros(q.size)
    inertia_err = (pc.J_total - P.J_R) * np.gradient(log.qd[m], log.t[m]) if q.size > 2 else np.zeros(q.size)
    kt_err = (kt - P.K_T) * log.i[m]                               # torque not credited by the host
    qd = log.qd[m]
    fric_err = (pc.tau_c - P.TAU_C) * np.tanh(qd / pc.v_fric)       # Coulomb friction error
    visc_err = (pc.b - P.B_VISC) * qd                               # viscous damping error
    dist = log.d[m]                                                 # bounded disturbance d(t)
    a = lambda x: float(np.mean(np.abs(x))) if x.size else float("nan")
    terms = dict(payload=a(payload), gravity_model=a(grav_err), coupling_total=a(cpl_true),
                 coupling_error=a(cpl_true - cpl_pred), inertia_error=a(inertia_err), kt_error=a(kt_err),
                 friction_error=a(fric_err), viscous_error=a(visc_err), disturbance=a(dist))
    return dict(faulted=True, t_trip=float(t0), n_trips=len(trips), at_limit_pct=100 * float(np.mean(log.clipped[m])),
                static_over_capacity=bool(pc.tau_lat > pc.k_t * float(np.min(log.i_lim))),
                tau_lat=pc.tau_lat, capacity_true=pc.k_t * float(np.min(log.i_lim)),
                capacity=a(cap), terms=terms, q_trip=float(np.degrees(np.mean(q))) if q.size else float("nan"),
                m_payload=pc.m_payload, coupling_scale=pc.k_yv / P.K_YV, kt_ratio=kt / P.K_T,
                J_ratio=pc.J / P.J_R, tau_g_ratio=pc.tau_g / P.TAU_G)


def post_freq(log, ev, spec):
    """Least-squares fit of q(t) and the original request to sin/cos at the
    request frequency over the steady window (after the 1 s ramp + 1 s)."""
    f = spec.sc.roll.w / (2 * math.pi)
    m = log.t >= 2.0
    t = log.t[m]
    X = np.column_stack([np.sin(2 * math.pi * f * t), np.cos(2 * math.pi * f * t), np.ones(t.size)])
    cq = np.linalg.lstsq(X, log.q[m], rcond=None)[0]
    cr = np.linalg.lstsq(X, log.q_ref[m], rcond=None)[0]
    hq, hr = complex(cq[0], cq[1]), complex(cr[0], cr[1])
    ratio = hq / hr
    return dict(f=f, gain=abs(ratio), phase_deg=math.degrees(math.atan2(ratio.imag, ratio.real)),
                clip_pct=100 * float(np.mean(log.clipped)),
                T_ff=float(np.nanmean(log.cmd_age[m]) + P.T_CMD_DELAY + 0.5e-3),
                T_fb_age=float(np.nanmean(log.fb_age[m])),
                gov_lag=float(np.nanmean(log.c_gov_lag[m])) if "c_gov_lag" in log else 0.0)


def _decimate(log, keys, k=4, t0=None, t1=None):
    m = np.ones(len(log.t), bool)
    if t0 is not None:
        m &= log.t >= t0
    if t1 is not None:
        m &= log.t <= t1
    idx = np.where(m)[0][::k]
    return {key: np.asarray(log[key], float)[idx] for key in keys if key in log}


def post_trace(log, ev, spec):
    keys = ("t", "q", "q_ref", "c_q_c", "i", "i_tgt", "i_lim", "c_i_unsat", "clipped", "mode", "c_suspended",
            "c_host_fallback", "qy", "qy_plan", "c_gov_sigma", "fb_age", "cmd_age")
    return dict(trace=_decimate(log, keys, k=2), events=list(log.events))


def post_step(log, ev, spec):
    """Task 2 robustness §2b metrics for the 30 deg step (same definitions)."""
    final = R(30)
    e = log.err * DEG
    outside = np.where(np.abs(e) > 1.0)[0]
    reach = np.where(log.q >= final - R(1))[0]
    return dict(rise=float(log.t[reach[0]]) if reach.size else float("nan"),
                over=max(0.0, float((np.max(log.q) - final) * DEG)),
                settle=float(log.t[outside[-1]]) if outside.size else 0.0,
                clip_pct=100 * float(np.mean(log.clipped)),
                integ=float(np.nanmax(np.abs(log.c_integ))), events=len(log.events))


def post_quant(log, ev, spec):
    t_from = 2.0 if spec.key.startswith("B") else 3.0
    m = log.t > t_from
    return dict(jitter_mA=1e3 * float(np.std(np.diff(log.i[m]))),
                rms=float(np.sqrt(np.mean((log.err[m] * DEG) ** 2))),
                hunt=float(np.ptp(log.q_enc[m]) * DEG))


def post_infeasible(log, ev, spec):
    """Timing and distances for the infeasible cases, from the log."""
    t = np.asarray(log.t, float)
    trips = [float(a) for a, b in log.events if b not in SM.TRANSIENT_FAULTS and not b.startswith("rearm_after_")]
    susp = np.asarray(log.get("c_suspended", np.zeros(len(t))), float) == 1
    last = t >= t[-1] - 1.0
    out = dict(trips=trips, t_suspended=float(t[np.argmax(susp)]) if susp.any() else None,
               q_end=float(log.q[-1] * DEG), q_req_end=float(log.q_ref[-1] * DEG),
               dist_end=float(abs(log.q[-1] - log.q_ref[-1]) * DEG),
               t_req_settled=None, clipped_pct=100 * float(np.mean(log.clipped)),
               i_mean_last_s=float(np.mean(np.abs(log.i[last]))))
    ref = np.asarray(log.q_ref, float)
    moving = np.where(np.abs(np.diff(ref)) > 1e-12)[0]
    out["t_req_settled"] = float(t[moving[-1] + 1]) if moving.size else 0.0
    if spec.seed == 1:
        out.update(post_trace(log, ev, spec))
    return out


POST = dict(mc=post_mc, freq=post_freq, trace=post_trace, step=post_step, quant=post_quant, infeasible=post_infeasible)


def _growth(book, spec, g):
    ctrl = ("baseline", dict(i_max=1e6, use_yaw_monitor=False), g)
    log, stats, rid, ev = _one(book, spec, ctrl)
    e = log.err
    n = len(e)
    early, late = np.ptp(e[n // 4: n // 2]), np.ptp(e[3 * n // 4:])
    return late / max(early, 1e-12), log, rid


def bisect_gain(book, spec):
    """Task 2 robustness §1b: simulated critical feedback-gain multiplier
    (9-step bisection on a 2 deg offset) plus the measured pipeline delay."""
    _, log, rid0 = _growth(book, spec, 1.0)
    m = log.t > 0.2
    T = float(np.nanmean(log.fb_age[m]) + np.nanmean(log.cmd_age[m]) + P.T_CMD_DELAY + 0.5e-3)
    lo, hi, steps = 1.0, 20.0, []
    for _ in range(9):
        mid = math.sqrt(lo * hi)
        r, lg, rid = _growth(book, spec, mid)
        stable = r < 0.9 and np.max(np.abs(lg.err)) < R(10)
        steps.append(dict(g=mid, ratio=float(r), stable=bool(stable), run_id=rid))
        lo, hi = (mid, hi) if stable else (lo, mid)
    return dict(T=T, g_sim=math.sqrt(lo * hi), steps=steps, run_id=rid0)


def _work(spec):
    global _BOOK
    if _BOOK is None:
        _BOOK = RunBook(ENTRY)
    before = set(_BOOK.runs)
    if spec.post == "bisect":
        post = bisect_gain(_BOOK, spec)
        row, rid = None, post["run_id"]
    else:
        log, stats, rid, ev = _one(_BOOK, spec)
        row = row_stats(log, stats, rid, ev, spec)
        post = POST[spec.post](log, ev, spec) if spec.post else None
    new = {k: _BOOK.runs[k] for k in _BOOK.runs if k not in before}
    if rid not in new:
        new[rid] = _BOOK.runs[rid]
    return dict(group=spec.group, key=spec.key, variant=spec.ctrl if isinstance(spec.ctrl, str) else spec.ctrl[0],
                seed=spec.seed, run_id=rid, row=row, post=post, records=new,
                code=_BOOK.code, env=_BOOK.env, defs=_BOOK.defs)


# ---------------------------------------------------------------------------
# Scenarios
# ---------------------------------------------------------------------------
def a_finite(base):
    """Run A scored as a finite motion (4A review N5): same config, request and
    seed as Task 2's A; waypoints and t_request taken from its MinJerkSequence."""
    a = S.run_a(base)
    segs = a.roll.segs
    return M.FiniteMotion("A", a.cfg, a.roll, a.yaw, a.note + " (finite scoring)",
                          tuple(q1 for _, _, _, q1 in segs), segs[-1][0] + segs[-1][1])


def drop_windows(duration, frac, period, offset, rng):
    """Emulated one-direction packet loss: blackout windows of +/-0.25 ms around
    a random `frac` of the send instants k*period + offset (exactly that share
    of messages is lost, independently). sim.config has no per-direction drop
    probability; windows reach the same effect without touching sim/."""
    n = int(round(duration / period))
    ks = np.sort(rng.choice(n, size=int(round(frac * n)), replace=False))
    return tuple((round(k * period + offset - 2.5e-4, 7), round(k * period + offset + 2.5e-4, 7)) for k in ks)


def with_cfg(sc, name=None, note=None, **groups):
    kw = dict(cfg=sc.cfg.with_(**groups))
    if name:
        kw["name"] = name
    if note:
        kw["note"] = note
    return replace(sc, **kw)


def stress_cases(base):
    """(label, scenario, mismatch or None, governed_yaw, trace?)"""
    a, b, c, d, e = (f(base) for f in (a_finite, S.run_b, S.run_c, S.run_d, S.run_e))
    m1 = M.m1(base)
    rng = np.random.default_rng(4242)
    tc = base.timing
    cases = [
        ("Yaw plan mismatch: C, actual yaw 10 ms late, x1.10", c, (0.010, 1.10)),
        ("Yaw plan mismatch: C, actual yaw 25 ms late, x0.85", c, (0.025, 0.85)),
        ("Yaw plan mismatch: B, actual yaw 10 ms late, x1.10", b, (0.010, 1.10)),
    ]
    for frac in (0.10, 0.30):
        fb = drop_windows(c.cfg.duration, frac, 1.0 / tc.f_drive, 0.0, rng)
        cmd = drop_windows(c.cfg.duration, frac, 1.0 / tc.f_ctrl, tc.t_compute, rng)
        cases += [(f"Feedback-only loss {frac:.0%} (emulated): C", with_cfg(c, timing=dict(feedback_blackout=fb)), None),
                  (f"Command-only loss {frac:.0%} (emulated): C", with_cfg(c, timing=dict(command_blackout=cmd)), None)]
    cases += [
        ("Mid-run derate 3.2 -> 2.4 A at 5 s: D", with_cfg(d, drive=dict(derate_schedule=((5.0, 2.4),))), None),
        ("Mid-run derate 3.2 -> 2.4 A at 2 s: A (finite)", with_cfg(a, drive=dict(derate_schedule=((2.0, 2.4),))), None),
        ("18 V bus, R +25 %: B", with_cfg(b, plant=dict(v_bus=18.0, R=2.25)), None),
        ("18 V bus, R +25 %: D", with_cfg(d, plant=dict(v_bus=18.0, R=2.25)), None),
        ("18 V bus, R +25 %: M1", with_cfg(m1, plant=dict(v_bus=18.0, R=2.25)), None),
        ("10 V bus, R +25 %: E (voltage limit binds)", with_cfg(e, plant=dict(v_bus=10.0, R=2.25)), None),
        ("8 V bus, R +25 %: E (voltage limit binds)", with_cfg(e, plant=dict(v_bus=8.0, R=2.25)), None),
    ]
    for s_lat in (0.035, -0.035):
        for kind in ("command_blackout", "blackout"):
            lab = "command-only" if kind == "command_blackout" else "both-direction"
            cases.append((f"Loaded recovery: D, CoM {1e3 * s_lat:+.0f} mm, {lab} loss 100 ms at 4 s",
                          with_cfg(d, plant=dict(s_lat=s_lat), timing={kind: ((4.0, 4.1),)}), None))
    hold45 = S.Scenario("H45", replace(base, duration=6.0).with_(plant=dict(d_amp=0.0), sensor=dict(enc_bits=12)),
                        MinJerkSequence(0.0, [(R(45), 0.8, 10.0)]), Hold(0.0), "hold 45 deg, d(t) off, 12-bit")
    cases += [
        ("Quantization near rest: M1 with 12-bit encoder", with_cfg(m1, sensor=dict(enc_bits=12)), None),
        ("Quantization near rest: hold 45 deg, 12-bit, d(t) off", hold45, None),
        ("Corner: A (finite), 4 ms CAN, J -30 %, Kt +15 %",
         with_cfg(a, timing=dict(can_min=.004, can_max=.004, burst_rate_hz=0), plant=dict(J=.0028, k_t=.161, k_e=.161)), None),
        ("Corner: C, 4 ms CAN, J -30 %, Kt +15 %",
         with_cfg(c, timing=dict(can_min=.004, can_max=.004, burst_rate_hz=0), plant=dict(J=.0028, k_t=.161, k_e=.161)), None),
        ("Corner: D, 4 ms CAN, J +30 %, Kt -15 %",
         with_cfg(d, timing=dict(can_min=.004, can_max=.004, burst_rate_hz=0), plant=dict(J=.0052, k_t=.119, k_e=.119)), None),
    ]
    return cases


def mc_de():
    """Task 2's uncertainty trials for D and E, regenerated with the same draws:
    rng(2024) is consumed for A-C exactly as exp.task2_eval does (no runs)."""
    rng = np.random.default_rng(2024)
    out = []
    for mk in (S.run_a, S.run_b, S.run_c, S.run_d, S.run_e):
        for k in range(20):
            b, _ = sample_cfg(rng, SimConfig())
            b = b.with_(plant=dict(k_e=b.plant.k_t))
            sc = mk(b)
            if sc.name in ("D", "E"):
                sc = replace(sc, cfg=sc.cfg.with_(plant=dict(m_payload=float(rng.uniform(.3, 1.0)))))
            sc = replace(sc, cfg=replace(sc.cfg, duration=6))
            if sc.name in ("D", "E"):
                out.append((sc.name, k, sc))
    return out


def infeasible_cases(base):
    physical = M.finite("INF-P", base, 0.0, [(R(30), 0.8, 0.5), (0.0, 0.8, 0.5)],
                        note="1.2 kg at +35 mm, derated 2.4 A: static load at 0 deg (0.41 N m) exceeds "
                             "capacity (0.336 N m); load unknown to the host",
                        plant=dict(m_payload=1.2), drive=dict(i_limit=2.4))
    c = S.run_c(base)
    policy = replace(c, name="INF-R", yaw=RampedSine(R(75), 2.8, t_ramp=1.0), cfg=c.cfg.with_(drive=dict(i_limit=2.4)),
                     note="roll hold under yaw +/-75 deg @ 2.8 Hz at 2.4 A: predicted coupling exceeds capacity")
    return physical, policy


def freq_amp(f):
    """Amplitude (deg) whose inertial torque J A w^2 stays <= 0.2 N m (inside the
    governor's budget, so the request is not reshaped), capped at 10 deg."""
    return min(10.0, math.degrees(0.2 / (P.J_R * (2 * math.pi * f) ** 2)))


def freq_scenario(base, f, bench=False):
    T = 2.0 + max(3.0, 6.0 / f)
    plant = dict(d_amp=0.0, tau_c=0.0) if bench else dict(d_amp=0.0)
    amp = freq_amp(f)
    return S.Scenario(f"F{f:g}{'-bench' if bench else ''}", replace(base, duration=T).with_(plant=plant),
                      RampedSine(R(amp), f, t_ramp=1.0), Hold(0.0),
                      f"roll sine {amp:.1f} deg at {f:g} Hz, yaw still, d(t) off"
                      + (", Coulomb friction off in plant and host (linear bench)" if bench else ""))


FREQS = (0.5, 1.0, 2.0, 4.0, 6.0)
BENCH_EXTRA = (2.2, 8.0, 10.0)     # bench-only points (review 4B I2)
PAYLOADS = (0.0, 0.3, 0.5, 0.7, 0.9, 1.2)
COUPLING = (0.5, 1.0, 1.5, 2.0, 2.5)


def all_specs(base, quick=False):
    seeds = (1,) if quick else SEEDS
    sseeds = (1,) if quick else STRESS_SEEDS
    sp = []
    for sc in S.all_runs(base):
        for v in VARIANTS:
            for s in seeds:
                post = "trace" if (sc.name == "E" and s == 1 and v != "plan") else None
                sp.append(variant_spec("matched", sc.name, sc, v, s, post=post))
    for mo in [a_finite(base)] + M.all_motions(base):
        for v in VARIANTS:
            for s in seeds:
                sp.append(variant_spec("finite", mo.name, mo, v, s))
    for label, sc, mis in stress_cases(base):
        for v in ("baseline", "plan"):
            for s in sseeds:
                sp.append(Spec("stress", label, sc, v, s, mismatch=mis))
    for name, k, sc in (mc_de()[:4] if quick else mc_de()):
        sp.append(Spec("mc", f"{name}{k:02d}", sc, "baseline", 100 + k, post="mc"))
    phys, pol = infeasible_cases(base)
    for s in sseeds:
        sp.append(Spec("infeasible", "INF-P", phys, "baseline", s, post="infeasible"))
        sp.append(variant_spec("infeasible", "INF-P", phys, "legacy", s, post="infeasible"))
        sp.append(Spec("infeasible", "INF-R no coordination", pol, "baseline", s, gy=False, post="infeasible"))
        sp.append(Spec("infeasible", "INF-R with coordination", pol, "baseline", s, post="infeasible"))
    for f in FREQS:
        for v in ("baseline", "legacy"):
            sp.append(variant_spec("freq", f"{f:g}", freq_scenario(base, f), v, 1, post="freq"))
    for f in FREQS + BENCH_EXTRA:
        sp.append(Spec("freq", f"{f:g}", freq_scenario(base, f, bench=True), ("bench", {}, 1.0, dict(tau_c=0.0)),
                       1, post="freq"))
    for mpay in PAYLOADS:
        for mk, lab in ((lambda b: S.run_d(b, m=mpay), "D"), (lambda b: M.m2(b, m=mpay), "M2")):
            sp.append(Spec("gen_payload", f"{lab} {mpay:.1f} kg", mk(base), "baseline", 1))
    for cs in COUPLING:
        sc = with_cfg(S.run_c(base), plant=dict(k_yv=P.K_YV * cs, k_ya=P.K_YA * cs))
        for v in ("baseline", "plan"):
            sp.append(Spec("gen_coupling", f"C x{cs:g}", sc, v, 1))
    e = S.run_e(base)
    for seed in (1, 7):     # seed 1 trips after the outage, seed 7 does not (Task 2's row)
        sp.append(Spec("timeline", "E, CoM +35 mm, feedback-only loss 100 ms at 3 s",
                       with_cfg(e, timing=dict(feedback_blackout=((3, 3.1),))), "baseline", seed, post="trace"))
    # Task 2 robustness re-runs with manifests (review 2C I-B)
    from exp.task2_robustness import linear_cfg
    for can in (1.2e-3, 4.0e-3):
        sp.append(Spec("rob_delay", f"{can * 1e3:.1f} ms", S.Scenario("LIN", linear_cfg(can), Hold(0.0), Hold(0.0),
                       "delay-only bench"), "baseline", 0, ("safety", dict(cmd_timeout=0.05)), False, post="bisect"))
    step = S.Scenario("step", SimConfig(duration=3.0), MinJerkSequence(0.0, [(R(30), 0.1, 5.0)]), Hold(0.0),
                      "30 deg in 0.1 s")
    no_wd = ("safety", dict(cmd_timeout=0.010))
    off = dict(use_governor=False, use_yaw_monitor=False)
    for key, ctrl, sup in (("A. governor ON, real supervisor (the baseline)", "baseline", "design"),
                           ("B. governor OFF, anti-windup ON, no watchdog", ("baseline", off, 1.0), no_wd),
                           ("C. governor OFF, anti-windup OFF, no watchdog", ("baseline", dict(off, anti_windup=False), 1.0), no_wd),
                           ("D. governor OFF, anti-windup ON, real supervisor", ("baseline", off, 1.0), "design")):
        sp.append(Spec("rob_step", key, step, ctrl, 1, sup, True, post="step"))
    for bits in (24, 14, 12):
        cfg = SimConfig().with_(sensor=dict(enc_bits=bits))
        h = S.Scenario("H", replace(cfg, duration=6.0).with_(plant=dict(d_amp=0.0)),
                       MinJerkSequence(0.0, [(R(45), 0.8, 10.0)]), Hold(0.0), "hold 45")
        sp.append(Spec("rob_quant", f"B {bits}", S.run_b(cfg), "baseline", 1, post="quant"))
        sp.append(Spec("rob_quant", f"H {bits}", h, "baseline", 1, post="quant"))
        sp.append(Spec("rob_quant", f"B {bits} plan", S.run_b(cfg), "plan", 1, post="quant"))
    return sp


# ---------------------------------------------------------------------------
# Calculated reference: linear tracking response of the baseline
# ---------------------------------------------------------------------------
def calc_tracking(f, T_ff, T_fb_age, lag=0.0, ctl=None):
    """q/r of the linearised baseline at q = 0 (Calculated): feed-forward
    F = J s^2 + b s + tau_g and PI-lead C both act on the host-time reference;
    the error uses feedback T_fb_age old; every torque arrives T_ff later
    through the current lag; `lag` is the governor path-clock lag (the governed
    reference runs that far behind the request on an admitted path).
    q/r = e^{-s lag} P G_ff (F + C) / (1 + P G_ff C e^{-s T_fb_age})."""
    from ctrl import loopshape as LS
    ctl = ctl or BaselineController()
    w = 2 * math.pi * np.atleast_1d(np.asarray(f, float))
    C, _, Pl = LS.freq_response(w, ctl.K, ctl.wc, 0.0, ctl.alpha, ctl.wi_ratio)
    s = 1j * w
    G = np.exp(-s * T_ff) / (P.TAU_I * s + 1)
    F = P.J_R * s ** 2 + P.B_VISC * s + P.TAU_G
    H = Pl * G * (F + C) / (1 + Pl * G * C * np.exp(-s * T_fb_age)) * np.exp(-s * lag)
    return np.abs(H), np.degrees(np.angle(H))


# ---------------------------------------------------------------------------
# Evaluation driver
# ---------------------------------------------------------------------------
def evaluate_all(specs, jobs):
    """Run every spec (in parallel), merge the per-worker RunBooks into BOOK in
    spec order, and require one code hash across all workers."""
    book = RunBook(ENTRY)
    if jobs > 1:
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            res = list(ex.map(_work_entry, specs, chunksize=1))
    else:
        res = [_work_entry(sp) for sp in specs]
    hashes = {r["code"]["code_hash"] for r in res}
    if len(hashes) != 1:
        raise RuntimeError(f"workers saw different code hashes: {sorted(hashes)}")
    book.code, book.env, book.defs = res[0]["code"], res[0]["env"], res[0]["defs"]
    for r in res:
        book.runs.update(r["records"])
    return book, res


def _work_entry(spec):
    # Always execute through the importable module, never __main__, so class
    # paths and the declared source set are the same in every process.
    from exp import task4b_eval as T
    return T._work(spec)


def _m(x):
    x = [v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))]
    return float(np.median(x)) if x else float("nan")


def _rng(x, nd=2):
    x = [v for v in x if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not x:
        return "–"
    lo, hi, md = min(x), max(x), float(np.median(x))
    return f"{md:.{nd}f} [{lo:.{nd}f}–{hi:.{nd}f}]" if len(x) > 1 else f"{md:.{nd}f}"


def _pct_rng(x):
    return _rng([100 * v for v in x], 0).replace("[", "[").replace("]", "]") + " %"


def table(headers, rows):
    out = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def ids_cell(ids):
    return f"`{run_set_id(ids)}`" if len(ids) > 1 else f"`{ids[0]}`"


def group(res, g):
    out = {}
    for r in res:
        if r["group"] == g:
            out.setdefault((r["key"], r["variant"]), []).append(r)
    return out


# ---------------------------------------------------------------------------
# Sections (each returns markdown lines; numbers come only from `res`)
# ---------------------------------------------------------------------------
def sec_matched(res):
    g = group(res, "matched")
    rows, per_seed = [], []
    for name in "ABCDE":
        for v in VARIANTS:
            rr = g.get((name, v), [])
            if not rr:
                continue
            x = [r["row"] for r in rr]
            rows.append([name, v, _rng([s["path_rms"] for s in x]), _rng([s["rms_gov"] for s in x]),
                         _rng([s["peak_gov"] for s in x], 1), _pct_rng([s["net_progress"] for s in x]),
                         f"{sum(s['tracked'] for s in x)}/{len(x)}", _rng([s["clip_pct"] for s in x], 1),
                         str(sum(s["wd_trips"] for s in x)), f"{sum(s['suspended_pct'] > 0 for s in x)}/{len(x)}",
                         ids_cell([r["run_id"] for r in rr])])
    return ["## 1. Matched variants: A–E, identical scenarios and seeds (Simulated)", "",
            "Median [min–max] over seeds 1–5. Same SimConfig, request and seed for every variant. "
            "*baseline* = frozen `BaselineController()` (causal yaw estimate); *plan* = `yaw_info=\"plan\"` "
            "(optional mode, assumes yaw follows its plan: stronger information); *legacy* = reconstructed legacy "
            "PID with the legacy watchdog (no governor, so its governed error is its wall-clock error). "
            "Path RMS is over the request window at the governor's path time (4A.3). A–E are periodic requests "
            "except A, which §2 also scores as a finite motion.", "",
            table(["Run", "Variant", "Path RMS ° (request window)", "Governed RMS °", "Governed peak °",
                   "Net progress", "Tracked (4A)", "At command limit %", "WD trips (total)", "Runs suspended",
                   "Run set"], rows), ""]


def sec_finite(res):
    g = group(res, "finite")
    rows = []
    for name in ("A", "M1", "M2", "M3"):
        for v in VARIANTS:
            rr = g.get((name, v), [])
            if not rr:
                continue
            x = [r["row"] for r in rr]
            done = [s for s in x if s["completed"]]
            rows.append([name, v, f"{len(done)}/{len(x)}",
                         _rng([s["t_complete"] for s in done]) if done else "–", f"{x[0]['t_request']:.2f}",
                         _rng([s["path_rms"] for s in x]), f"{sum(s['tracked'] for s in x)}/{len(x)}",
                         str(sum(s["wd_trips"] for s in x)), f"{sum(s['suspended_pct'] > 0 for s in x)}/{len(x)}",
                         ids_cell([r["run_id"] for r in rr])])
    return ["## 2. Finite motions: completion time (Simulated)", "",
            "A is Task 2's run A scored as a finite motion (waypoints ±45°, t_request = end of its last move, "
            "4A review N5); M1–M3 are the Packet 4A sequences. Completion = every waypoint visited and the final "
            "target reached at rest while healthy, then held to the end of the run (4A.3). Median [min–max] "
            "completion time over completed seeds.", "",
            table(["Motion", "Variant", "Completed", "Completion s", "Requested s", "Path RMS ° (request window)",
                   "Tracked (4A)", "WD trips (total)", "Runs suspended", "Run set"], rows), ""]


def sec_regressions(res):
    """Per-(case, seed) differences against the frozen baseline."""
    by = {}
    for r in res:
        if r["group"] in ("matched", "finite", "stress", "gen_coupling"):
            by[(r["group"], r["key"], r["seed"], r["variant"])] = r
    rows = []
    for (grp, key, seed, v), r in sorted(by.items(), key=lambda kv: (kv[0][0], str(kv[0][1]), kv[0][2], kv[0][3])):
        if v == "baseline":
            continue
        b = by.get((grp, key, seed, "baseline"))
        if b is None:
            continue
        x, y = b["row"], r["row"]
        diffs = []
        if x["tracked"] != y["tracked"]:
            diffs.append(f"tracked {'yes' if x['tracked'] else 'no'} → {'yes' if y['tracked'] else 'no'}")
        if x["completed"] is not None and x["completed"] != y["completed"]:
            diffs.append(f"completed {x['completed']} → {y['completed']}")
        if x["wd_trips"] != y["wd_trips"]:
            diffs.append(f"WD trips {x['wd_trips']} → {y['wd_trips']}")
        if (x["suspended_pct"] > 0) != (y["suspended_pct"] > 0):
            diffs.append(f"suspended {x['suspended_pct']:.0f} % → {y['suspended_pct']:.0f} %")
        if diffs:
            rows.append([grp, key, seed, v, "; ".join(diffs), b["run_id"], r["run_id"]])
    merged = {}
    for grp, key, seed, v, d, rb, rv in rows:
        merged.setdefault((grp, key, v), []).append((seed, d, rb, rv))
    n_rows = len(rows)
    rows = [[grp, key, v, "; ".join(f"s{seed}: {d}" for seed, d, _, _ in items),
             f"`{run_set_id([x[2] for x in items])}` / `{run_set_id([x[3] for x in items])}`"]
            for (grp, key, v), items in merged.items()]
    return ["## 3. Per-case differences against the frozen baseline (Simulated)", "",
            "Every (case, seed) where a variant's verdict differs from the baseline's on the same inputs: "
            "tracked, completed, watchdog-trip count or suspension, listed per seed (baseline → variant). "
            "Run sets list the member runs in task4b_results.json.", "",
            table(["Set", "Case", "Variant", "Change vs baseline, per seed", "Run sets (baseline / variant)"], rows)
            if rows else "No differences.", ""], n_rows


def _fb_cell(x):
    f = [s["fallback"] for s in x if s.get("fallback")]
    if not f:
        return "–"
    w = max(f, key=lambda d: d["excursion_deg"])
    return (f"{w['qd_entry']:+.1f} / {w['i_fallback_max']:.1f} ({w['at_limit_pct']:.0f} %) / "
            f"{w['payload_along_motion_Nm']:+.2f}")


def sec_stress(res):
    g = group(res, "stress")
    keys = []
    for r in res:
        if r["group"] == "stress" and r["key"] not in keys:
            keys.append(r["key"])
    rows = []
    for key in keys:
        for v in ("baseline", "plan"):
            rr = g.get((key, v), [])
            if not rr:
                continue
            x = [r["row"] for r in rr]
            comp = [s["completed"] for s in x if s["completed"] is not None]
            ev = sorted({e for s in x for e in s["events"]})
            rows.append([key, v, f"{sum(s['tracked'] for s in x)}/{len(x)}",
                         f"{sum(comp)}/{len(comp)}" if comp else "–", _pct_rng([s["net_progress"] for s in x]),
                         _rng([s["path_rms"] for s in x]), f"{max(s['peak_gov'] for s in x):.1f}",
                         f"{max(s['fallback_pct'] for s in x):.1f}", f"{max(s['fb_excursion'] for s in x):.1f}",
                         _fb_cell(x), f"{max(s['vlim_pct'] for s in x):.1f}",
                         f"{sum(s['suspended_pct'] > 0 for s in x)}/{len(x)}",
                         f"{min(s['yaw_scale'] for s in x):.2f}", ", ".join(ev) or "none",
                         ids_cell([r["run_id"] for r in rr])])
    return ["## 4. Stress set: baseline and plan look-ahead, seeds 1–3 (Simulated)", "",
            "Yaw mismatch: the plant's yaw is `FollowingYaw(plan, lag, gain)` while the host sees the plan "
            "(`simulate(..., yaw_plan=plan)`); the estimate-mode baseline measures yaw and does not read the plan. "
            "Asymmetric loss is emulated with ±0.25 ms blackout windows on exactly the stated share of messages in one "
            "direction (rng 4242); sim.config has no per-direction drop probability. Loaded recovery uses D's "
            "0.7 kg payload at CoM ±35 mm. Corners combine the fixed 4 ms CAN latency with J/Kt extremes.", "",
            table(["Case", "Variant", "Tracked", "Completed", "Net progress", "Path RMS °", "Worst governed peak °",
                   "Worst fallback %", "Worst fallback excursion °",
                   "Worst-excursion fallback: entry q̇ rad/s / fallback current A (at limit %) / payload torque along motion N·m",
                   "Voltage-limited %", "Runs suspended", "Min yaw scale", "Events", "Run set"], rows), "",
            "Fallback excursion = largest |q − q(entry)| while only the drive's local damping acts (after start-up): "
            "fallback is motion reduction, not a position hold. For the fallback span with the largest excursion, the "
            "entry speed, the largest fallback current target and its share at the active limit, and the mean payload "
            "torque along the direction of motion (> 0: the payload opposes braking) are listed (Calculated from the "
            "log and the true plant). Fallback current is −c·v/Kt with c = 0.05 N·m·s/rad, so entering faster than "
            "Kt·I_limit/c ≈ 9 rad/s asks more than the limit and is clamped: the excursion is then a current-limited "
            "stopping distance. Voltage-limited % = share of samples where the supply voltage limited the current loop.", ""]


def classify(p):
    """Declared rule (project assumption): the cause is the largest unmodelled
    torque term over the 100 ms before the first trip; 'limit-bound' if the
    command was at the current limit for >= 50 % of that window."""
    unmod = {k: v for k, v in p["terms"].items() if k != "coupling_total"}
    cause = max(unmod, key=unmod.get)
    label = {"payload": "unmodelled payload torque", "gravity_model": "gravity-model error",
             "coupling_error": "coupling prediction error", "inertia_error": "inertia error",
             "kt_error": "motor-constant error", "friction_error": "Coulomb-friction error",
             "viscous_error": "viscous-damping error", "disturbance": "bounded disturbance d(t)"}[cause]
    if p.get("static_over_capacity"):
        label += ", physically infeasible (static payload torque > capacity)"
    elif p["at_limit_pct"] >= 50:
        label += ", limit-bound"
    return label, cause


def sec_mc(res, published):
    rows, counts, xcheck = [], {}, []
    mc = [r for r in res if r["group"] == "mc"]
    for r in mc:
        name, k = r["key"][0], int(r["key"][1:])
        pub = (published or {}).get(name, [])
        if k < len(pub):
            xcheck.append(pub[k]["wd_trips"] == r["row"]["wd_trips"]
                          and abs(pub[k]["rms_gov"] - r["row"]["rms_gov"]) < 1e-9)
        p = r["post"]
        if not p["faulted"]:
            continue
        label, cause = classify(p)
        counts[label] = counts.get(label, 0) + 1
        t = p["terms"]
        s = r["row"]
        rows.append([r["key"], f"{p['m_payload']:.2f}", f"{p['coupling_scale']:.2f}", f"{p['kt_ratio']:.2f}",
                     f"{p['J_ratio']:.2f}", f"{p['t_trip']:.2f}", str(p["n_trips"]), f"{p['q_trip']:.0f}",
                     f"{p['at_limit_pct']:.0f}", f"{p['capacity']:.3f}", f"{t['payload']:.3f}",
                     f"{t['gravity_model']:.3f}", f"{t['coupling_error']:.3f}", f"{t['inertia_error']:.3f}",
                     f"{t['kt_error']:.3f}", f"{t['friction_error']:.3f}", f"{t['viscous_error']:.3f}",
                     f"{t['disturbance']:.3f}", f"{p['tau_lat']:.3f} / {p['capacity_true']:.3f}", label,
                     f"suspended {s['suspended_pct']:.0f} %{', LOCKOUT' if s['lockout'] else ''}",
                     f"`{r['run_id']}`"])
    n_f = len(rows)
    same = f"{sum(xcheck)}/{len(xcheck)}" if xcheck else "not checked"
    fpay = [r["post"]["m_payload"] for r in mc if r["post"]["faulted"]]
    nf = [r for r in mc if not r["post"]["faulted"]]
    lo = min(fpay) if fpay else float("nan")
    similar = [r for r in nf if r["post"]["m_payload"] >= lo]
    sim_list = ", ".join(f"{r['key']} {r['post']['m_payload']:.2f} kg" for r in similar)
    nonf_line = (f"**The rule attributes; it does not discriminate.** Payload dominates by construction in payload "
                 f"scenarios: every D/E trial carries 0.3–1.0 kg. Faulted payloads span {lo:.2f}–{max(fpay):.2f} kg; "
                 f"{len(similar)} non-faulted trials also carry at least {lo:.2f} kg ({sim_list}), so payload mass "
                 "alone does not predict the fault. The rule says which unmodelled torque was largest when the trip "
                 "came, not why this trial tripped and a similar one did not.") if fpay else ""
    return ["## 5. D/E Monte Carlo faulted trials, classified (Simulated; torque terms Calculated from logged state)", "",
            f"The 40 D/E uncertainty trials of Task 2 were regenerated with the same draws (rng 2024, seeds 100+k). "
            f"Cross-check against `task2_results.json`: watchdog trips and governed RMS identical in {same} trials. "
            f"{n_f} of {len(mc)} trials fault (a non-transient drive event). None is excluded. For each, the mean "
            "|torque| of every term the host does not model is computed over the 100 ms before the first trip from "
            "the logged true state and the trial's true parameters: payload τ_lat cos q; gravity-model error "
            "(τ_g − τ_g,nom) sin q; coupling prediction error (true − host-predicted coupling); inertia error "
            "(J − J_nom)·q̈; motor-constant error (Kt − Kt_nom)·i; Coulomb-friction error (τ_c − τ_c,nom)·tanh(q̇/v_f); "
            "viscous error (b − b_nom)·q̇; the bounded disturbance d(t). Capacity is Kt·I_limit (true Kt in the "
            "static column). Rule (project assumption): cause = largest unmodelled term; *physically infeasible* if "
            "the static payload torque τ_lat alone exceeds the true capacity; otherwise *limit-bound* if the command "
            "sat at the current limit ≥ 50 % of the window.", "",
            "Counts: " + (", ".join(f"{k}: {v}" for k, v in sorted(counts.items())) or "none"), "",
            table(["Trial", "Payload kg", "Coupling ×", "Kt ×", "J ×", "First trip s", "Trips", "q at trip °",
                   "At limit % (100 ms)", "Capacity N·m", "Payload N·m", "Gravity err N·m", "Coupling err N·m",
                   "Inertia err N·m", "Kt err N·m", "Friction err N·m", "Viscous err N·m", "d(t) N·m",
                   "Static payload / capacity N·m", "Cause (rule)", "Outcome", "run_id"], rows) if rows else "No faulted trials.",
            "", nonf_line, ""], counts, n_f


def sec_infeasible(res):
    rows, rows2 = [], []
    for r in res:
        if r["group"] != "infeasible":
            continue
        s = r["row"]
        p = r["post"] or {}
        c = s["contracts"]
        rows2.append([r["key"], r["variant"], str(r["seed"]),
                      ", ".join(f"{x:.3f}" for x in p.get("trips", [])) or "none",
                      "–" if p.get("t_suspended") is None else f"{p['t_suspended']:.3f}",
                      " → ".join(f"{h:.1f}" for h in c["hold_positions_deg"]) or "–",
                      f"{p.get('q_req_end', float('nan')):.1f} (from {p.get('t_req_settled', 0):.1f} s)",
                      f"{p.get('q_end', float('nan')):.1f}", f"{p.get('dist_end', float('nan')):.1f}",
                      f"{p.get('clipped_pct', float('nan')):.1f}", f"{p.get('i_mean_last_s', float('nan')):.2f}"])
        rows.append([r["key"], r["variant"], str(r["seed"]), "yes" if s["tracked"] else "no",
                     "–" if s["completed"] is None else ("yes" if s["completed"] else "no"),
                     f"{s['net_progress']:.0%}", f"{s['suspended_pct']:.0f} / {s['coord_stop_pct']:.0f} / "
                     f"{s['fallback_pct']:.0f} / {s['rejected_pct']:.0f}",
                     ", ".join(s["events"]) or "none", f"{s['q_min']:.1f} … {s['q_max']:.1f}", f"{s['q_end']:.1f}",
                     f"{s['fb_excursion']:.1f}",
                     f"{s['qd_peak']:.2f}", f"{s['yaw_scale']:.2f}", "; ".join(s["reasons"]) or "–",
                     f"`{r['run_id']}`"])
    return ["## 6. Explicit infeasible cases (Simulated)", "",
            "*INF-P (physical):* 1.2 kg at +35 mm, derated 2.4 A, finite move 0 → 30° → 0. The static load at 0° "
            "(τ_lat = 0.41 N·m) exceeds the actuator capacity (Kt·2.4 A = 0.336 N·m) and the host does not know "
            "the payload. *INF-R (request policy):* roll hold under yaw ±75° at 2.8 Hz at 2.4 A; the predicted "
            "coupling exceeds capacity and the host knows it. Run with and without yaw coordination. "
            "Neither can be tracked; what matters is the disposition and the actual motion afterwards.", "",
            table(["Case", "Variant", "Seed", "Tracked", "Completed", "Net progress",
                   "Time % suspended / coord. stop / fallback / rejected", "Events", "Roll range °", "Final roll °",
                   "Fallback excursion °",
                   "Peak |q̇| rad/s", "Final yaw scale", "Disposition reasons (controller notices)", "run_id"], rows), "",
            "Timing and distances (from the logs). Hold positions are the suspended-hold references per fault epoch: "
            "a second value is a new catch after a new fault, not a drifting hold. Distance is to the *current* request "
            "at the end of the run.", "",
            table(["Case", "Variant", "Seed", "Tracking trips s", "Suspended from s", "Hold position(s) °",
                   "Current request °", "Final roll °", "Final distance to request °", "At command limit %",
                   "Mean |i| last 1 s A"], rows2), ""]


def sec_freq(res):
    g = group(res, "freq")
    tff, tfb, lags = [], [], []
    for r in res:
        if r["group"] == "freq" and r["variant"] == "baseline":
            tff.append(r["post"]["T_ff"]); tfb.append(r["post"]["T_fb_age"]); lags.append(r["post"]["gov_lag"])
    T_ff, T_fb, lag = float(np.mean(tff)), float(np.mean(tfb)), float(np.mean(lags))
    rows = []
    for f in sorted(FREQS + BENCH_EXTRA):
        b, l, n = (g.get((f"{f:g}", v), [None])[0] for v in ("baseline", "legacy", "bench"))
        if b is None:
            gc, pc = calc_tracking(f, T_ff, T_fb, lag)
            rows.append([f"{f:g}", f"{freq_amp(f):.1f}", f"{gc[0]:.3f} / {pc[0]:+.1f}",
                         f"{n['post']['gain']:.3f} / {n['post']['phase_deg']:+.1f}", "–", "–",
                         f"{n['row']['reshaping_pct']:.0f}", f"`{n['run_id']}`"])
            continue
        gc, pc = calc_tracking(f, T_ff, T_fb, lag)
        rows.append([f"{f:g}", f"{freq_amp(f):.1f}", f"{gc[0]:.3f} / {pc[0]:+.1f}",
                     f"{n['post']['gain']:.3f} / {n['post']['phase_deg']:+.1f}",
                     f"{b['post']['gain']:.3f} / {b['post']['phase_deg']:+.1f}",
                     f"{l['post']['gain']:.3f} / {l['post']['phase_deg']:+.1f}",
                     f"{max(b['row']['reshaping_pct'], n['row']['reshaping_pct']):.0f}",
                     f"`{n['run_id']}` / `{b['run_id']}` / `{l['run_id']}`"])
    return ["## 7. Tracking frequency response (Simulated vs Calculated)", "",
            "Roll sines with yaw still, seed 1, bounded disturbance d(t) off (at ±0.05 N·m it can move the axis by "
            "≈ 0.05/K ≈ 3.8° and would swamp small sines). Amplitude is chosen so the inertial torque stays ≤ 0.2 N·m, "
            "inside the governor budget (capped at 10°). Gain/phase of q relative to the original request from a "
            "least-squares sin/cos fit after 2 s. *Calculated:* linearised baseline at q = 0; feed-forward and "
            f"feedback act on the host-time reference; torque delay T_ff = {1e3 * T_ff:.2f} ms and feedback age "
            f"{1e3 * T_fb:.2f} ms and governor path-clock lag {1e3 * lag:.2f} ms (the start-up sync leaves the governed "
            "reference that far behind the wall-clock request on an admitted path; means measured in these runs); "
            "current lag 1.2 ms; no friction, quantization or "
            "d(t). *Bench:* the same simulated baseline with Coulomb friction removed from the plant and from the "
            "host's friction compensation (diagnostic only), which isolates the linear part the calculation models.", "",
            table(["f Hz", "Amplitude °", "Calculated gain / phase °", "Bench sim gain / phase °",
                   "Baseline sim gain / phase °", "Legacy sim gain / phase °", "Reshaping % (max)",
                   "run_ids (bench / baseline / legacy)"], rows), "", band_text(T_ff, T_fb, lag), ""], (T_ff, T_fb, lag)


def over_tracking_band(T_ff, T_fb, lag):
    """Calculated gain on a dense 0.1-30 Hz grid: bands where gain > 1.05 and
    > 1.10, the peak, and the gain at 1, 1.5 and 2.2 Hz."""
    f = np.logspace(-1, np.log10(30), 6000)
    g, _ = calc_tracking(f, T_ff, T_fb, lag)
    band = lambda thr: (float(f[g > thr][0]), float(f[g > thr][-1])) if np.any(g > thr) else None
    k = int(np.argmax(g))
    at = lambda x: float(calc_tracking(x, T_ff, T_fb, lag)[0][0])
    return dict(b105=band(1.05), b110=band(1.10), peak=float(g[k]), f_peak=float(f[k]),
                g10=at(1.0), g15=at(1.5), g22=at(2.2))


def band_text(T_ff, T_fb, lag):
    b = over_tracking_band(T_ff, T_fb, lag)
    fmt = lambda x: "none" if x is None else f"{x[0]:.2f}–{x[1]:.1f} Hz"
    return (f"**Over-tracking band (Calculated, dense grid 0.1–30 Hz):** gain > 1.05 for {fmt(b['b105'])}; "
            f"gain > 1.10 for {fmt(b['b110'])}; peak {b['peak']:.3f} at {b['f_peak']:.2f} Hz. The Task 2 requests "
            f"lie at the lower edge: gain {b['g10']:.3f} at 1 Hz (D/E roll sweeps), {b['g15']:.3f} at 1.5 Hz, "
            f"{b['g22']:.3f} at 2.2 Hz (C's yaw frequency; roll in B/C is a hold). The bench points at 2.2, 8 and "
            "10 Hz check the calculation inside the band.")


def sec_general(res):
    rows = []
    for r in res:
        if r["group"] == "gen_payload":
            s = r["row"]
            rows.append([r["key"], f"{s['net_progress']:.0%}", f"{s['path_rms']:.2f}", f"{s['rms_gov']:.2f}",
                         "–" if s["completed"] is None else ("yes" if s["completed"] else "no"),
                         f"{s['wd_trips']}", f"{s['suspended_pct']:.0f}", "yes" if s["tracked"] else "no",
                         f"`{r['run_id']}`"])
    rows2 = []
    g = group(res, "gen_coupling")
    for cs in COUPLING:
        cells = [f"C ×{cs:g}"]
        ids = []
        for v in ("baseline", "plan"):
            r = g.get((f"C x{cs:g}", v), [None])[0]
            s = r["row"]
            cells += [f"{s['net_progress']:.0%}", f"{s['yaw_scale']:.2f}", f"{s['path_rms']:.2f}",
                      "yes" if s["tracked"] else "no"]
            ids.append(r["run_id"])
        rows2.append(cells + [f"`{ids[0]}` / `{ids[1]}`"])
    return ["## 8. Generalization sweeps, seed 1 (Simulated)", "",
            "Payload at +35 mm on D (periodic sweep) and M2 (finite ±60° moves at 2.4 A); coupling scale on C "
            "(both k_yv and k_ya scaled; the host keeps the nominal coupling model).", "",
            table(["Case", "Net progress", "Path RMS °", "Governed RMS °", "Completed", "WD trips", "Suspended %",
                   "Tracked", "run_id"], rows), "",
            table(["Coupling", "Baseline progress", "Baseline yaw scale", "Baseline path RMS °", "Baseline tracked",
                   "Plan progress", "Plan yaw scale", "Plan path RMS °", "Plan tracked", "run_ids"], rows2), ""]


def sec_robust(res):
    from exp.task2_robustness import gain_range
    ctl = BaselineController()
    pub_delay = {"1.2 ms": (6.00, 5.34, 5.31), "4.0 ms": (11.00, 3.24, 3.23)}
    rows = []
    for r in res:
        if r["group"] == "rob_delay":
            p = r["post"]
            g_an = gain_range(ctl, p["T"])[1]
            pd = pub_delay.get(r["key"])
            mine = (round(1e3 * p["T"], 2), round(g_an, 2), round(p["g_sim"], 2))
            rows.append([r["key"], f"{1e3 * p['T']:.2f}", f"{g_an:.2f}", f"{p['g_sim']:.2f}",
                         f"{pd[0]:.2f} / {pd[1]:.2f} / {pd[2]:.2f}" if pd else "–",
                         "same" if pd and mine == tuple(pd) else "DIFFERENT",
                         f"`{p['run_id']}`, bisection {run_set_id([st['run_id'] for st in p['steps']])}"])
    pub_step = {"A": ("0.17", "1.1", "0.22", "0.0", "0.017", "0"), "B": ("0.60", "0.6", "0.60", "2.0", "0.037", "0"),
                "C": ("0.18", "1.5", "1.61", "1.8", "0.100", "0"), "D": ("0.60", "0.6", "0.60", "2.0", "0.037", "0")}
    rows2 = []
    for r in res:
        if r["group"] == "rob_step":
            p = r["post"]
            mine = (f"{p['rise']:.2f}", f"{p['over']:.1f}", f"{p['settle']:.2f}", f"{p['clip_pct']:.1f}",
                    f"{p['integ']:.3f}", str(p["events"]))
            pub = pub_step[r["key"][0]]
            rows2.append([r["key"], " / ".join(mine), " / ".join(pub), "same" if mine == pub else "DIFFERENT",
                          f"`{r['run_id']}`"])
    pub_q = {24: (0.303, 7.6, 7.1, 0.0779, 0.0, 0.217), 14: (0.303, 31.0, 7.7, 0.0806, 1.1, 0.220),
             12: (0.351, 117.3, 9.4, 0.0939, 4.6, 0.176)}
    q = {r["key"]: r for r in res if r["group"] == "rob_quant"}
    rows3 = []
    for bits in (24, 14, 12):
        b, h, pl = q[f"B {bits}"], q[f"H {bits}"], q[f"B {bits} plan"]
        mine = (round(b["post"]["rms"], 3), round(b["post"]["jitter_mA"], 1), round(pl["post"]["jitter_mA"], 1),
                round(h["post"]["rms"], 4), round(h["post"]["jitter_mA"], 1), round(h["post"]["hunt"], 3))
        rows3.append([f"{bits}-bit", " / ".join(map(str, mine)), " / ".join(map(str, pub_q[bits])),
                      "same" if mine == pub_q[bits] else "DIFFERENT",
                      ids_cell([b["run_id"], h["run_id"], pl["run_id"]])])
    return ["## 9. Task 2 robustness simulations re-run with manifests (review 2C I-B)", "",
            "The simulated rows of `task2_robustness.md` §1b, §2b and §4b, re-run through RunBook with the same "
            "configurations, controllers, supervisors and seeds, so each now has a run_id. "
            "Published values are copied from `task2_robustness.md` for comparison.", "",
            table(["CAN latency", "Pipeline delay ms", "Analytic critical gain × (Calculated)",
                   "Simulated critical gain ×", "Published delay / analytic / simulated", "Match", "run_ids"], rows), "",
            table(["Saturation step case", "This run: rise / overshoot / settle / % at limit / max|integ| / events",
                   "Published", "Match", "run_id"], rows2), "",
            table(["Encoder", "This run: B RMS / B jitter mA / B plan jitter / hold RMS / hold jitter / hunting",
                   "Published", "Match", "Run set (B, hold, B plan)"], rows3), ""]


def sec_contracts(res):
    mine = [r for r in res if r["row"] is not None and r["variant"] in ("baseline", "plan")]
    n = len(mine)
    tgt = max(r["row"]["target_over_limit"] for r in mine)
    cur = max(mine, key=lambda r: r["row"]["current_over_limit"])
    locks = [r for r in mine if r["row"]["lockout"]]
    bad_track = [r for r in mine if r["row"]["tracked"] and (r["row"]["suspended_pct"] > 0 or r["row"]["rejected_pct"] > 0)]
    bad_comp = [r for r in mine if r["row"]["completed"] and r["row"]["voided_at"] is not None]
    fb_track = [r for r in mine if r["row"]["tracked"] and r["row"]["fallback_pct"] > 0.5]
    c = [r["row"]["contracts"] for r in mine]
    lst = lambda rr: ", ".join(f"{r['key']} ({r['variant']}, s{r['seed']})" for r in rr[:5])
    drift = max(x["hold_drift_deg"] for x in c)
    rebased = [r for r in mine if r["row"]["contracts"]["rebases"]]
    unacked = [r for r in mine if r["row"]["contracts"]["rearms_unacked"]]
    noreplan = [r for r in mine if r["row"]["contracts"]["resumes_without_replan"]]
    noreason = [r for r in mine if r["row"]["contracts"]["reason_missing"]]
    unknown = [r for r in mine if r["row"]["contracts"]["reason_unknown"]]
    jumps = [r for r in mine if r["row"]["contracts"]["jumps"]]
    jmax = max(x["jump_max_rad_s"] for x in c)
    ctrl_rows = [
        ["(a) A suspended hold's reference is stationary between faults (suspended, drive normal, host active, governor HOLD)",
         f"{sum(x['hold_samples'] for x in c)} hold samples in {sum(1 for x in c if x['hold_samples'])} runs; max drift "
         f"within a fault epoch {drift:.3g}°. Re-based by a new catch after a new fault: {len(rebased)} runs"
         + (f", e.g. {lst(rebased)}" if rebased else ""), "PASS" if drift < 1e-6 else "FAIL"],
        ["(b) Every re-arm after a tracking fault is preceded by host commands acknowledging that fault_id",
         f"{sum(x['rearms'] for x in c)} tracking-fault re-arms; {len(unacked)} runs with an unacknowledged re-arm",
         "PASS" if not unacked else "FAIL"],
        ["(b) A suspended request never resumes without replan()",
         f"{sum(x['resumes'] for x in c)} resumptions; {len(noreplan)} runs resumed without a replan",
         "PASS" if not noreplan else "FAIL"],
        ["(c) Every suspended / restricted / rejected sample carries a non-empty controller reason",
         f"{sum(x['reason_samples'] for x in c)} samples; {sum(x['reason_missing'] for x in c)} without a reason"
         + (f"; notice log truncated in {len(unknown)} runs" if unknown else ""),
         "PASS" if not noreason and not unknown else ("FAIL" if noreason else "UNKNOWN")],
        ["(d) The governed reference never jumps in position (|Δq_c|/ts ≤ governor speed cap 20 rad/s, host active)",
         f"max {jmax:.2f} rad/s; {len(jumps)} runs with a jump", "PASS" if not jumps else "FAIL"],
        ["No drive lockout", f"{len(locks)} runs" + (f": {lst(locks)}" if locks else ""),
         "PASS" if not locks else "lockout outside the modelled envelope (declared)"],
    ]
    sim_rows = [
        ["Applied current target never above the active limit (the drive clamps it: a simulator property)",
         f"max excess {max(0.0, tgt):.3g} A", "consistent" if tgt < 1e-9 else "INCONSISTENT"],
        ["Measured current above a newly reduced limit (current cannot step)",
         f"max {cur['row']['current_over_limit']:.3f} A ({cur['key']}, `{cur['run_id']}`)",
         "never above" if cur["row"]["current_over_limit"] <= 1e-9 else "transient (declared)"],
        ["Suspended or rejected time never scored as tracking (scorer definition)", f"{len(bad_track)} violations",
         "consistent" if not bad_track else "INCONSISTENT"],
        ["Fallback > 0.5 % of a run never scored as tracking", f"{len(fb_track)} runs" + (f": {lst(fb_track)}" if fb_track else ""),
         "consistent" if not fb_track else "review"],
        ["Completion never survives a post-arrival latched fault (scorer definition)", f"{len(bad_comp)} violations",
         "consistent" if not bad_comp else "INCONSISTENT"],
    ]
    return ["## 0. Contract checks over every baseline and plan-mode run", "",
            f"**0a. Controller contracts, computed from the logs of all {n} baseline and plan-mode runs (Simulated).** "
            "These test the frozen controller's behaviour, not the simulator or the scorer.", "",
            table(["Contract", "Result", "Verdict"], ctrl_rows), "",
            "**0b. Simulator and scorer consistency checks.** These hold by construction (the drive model clamps the "
            "target; the 4A scorer defines tracked/completed this way). They check that the evidence pipeline is "
            "consistent, not that the controller is correct.", "",
            table(["Check", "Result", "Verdict"], sim_rows), ""], dict(controller=ctrl_rows, consistency=sim_rows)


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
def _find(res, grp, key=None, variant=None, seed=None):
    for r in res:
        if r["group"] == grp and (key is None or r["key"] == key) and (variant is None or r["variant"] == variant) \
                and (seed is None or r["seed"] == seed):
            return r
    return None


def figures(res, figdir, T):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    figdir.mkdir(parents=True, exist_ok=True)
    checks = {}

    # saturation: run E seed 1, baseline vs legacy
    fig, axes = plt.subplots(2, 2, figsize=(12, 6.5), sharex=True)
    for col, v in enumerate(("baseline", "legacy")):
        r = _find(res, "matched", "E", v, 1)
        tr = r["post"]["trace"]
        ax = axes[0, col]
        ax.plot(tr["t"], tr["q_ref"] * DEG, lw=.8, alpha=.6, label="original request")
        if "c_q_c" in tr:
            ax.plot(tr["t"], tr["c_q_c"] * DEG, lw=.9, label="governed reference")
        ax.plot(tr["t"], tr["q"] * DEG, lw=.9, label="roll")
        ax.set_ylabel("roll (deg)"); ax.set_title(f"E seed 1: {VARIANT_LABEL[v]}", fontsize=9)
        ax = axes[1, col]
        if "c_i_unsat" in tr:
            ax.plot(tr["t"], tr["c_i_unsat"], lw=.6, alpha=.7, label="host demand before clamp")
        ax.plot(tr["t"], tr["i"], lw=.7, label="measured current")
        ax.plot(tr["t"], tr["i_lim"], "k--", lw=.8, label="active limit"); ax.plot(tr["t"], -tr["i_lim"], "k--", lw=.8)
        ax.fill_between(tr["t"], -3.5, 3.5, where=tr["clipped"] > 0, color="red", alpha=.12, label="at limit")
        ax.set_ylim(-3.6, 3.6); ax.set_ylabel("current (A)"); ax.set_xlabel("time (s)")
        checks[f"sat_{v}"] = dict(at_limit_pct=100 * float(np.mean(tr["clipped"])),
                                  max_abs_i=float(np.max(np.abs(tr["i"]))), lim=float(np.min(tr["i_lim"])))
    for ax in axes.flat:
        ax.grid(alpha=.2); ax.legend(fontsize=7, loc="upper right")
    fig.suptitle("Saturation: E (0.7 kg, 2.4 A) — the baseline reshapes, the legacy loop saturates (Simulated)")
    fig.tight_layout(); fig.savefig(figdir / "task4b_saturation.png", dpi=130); plt.close(fig)

    # frequency response
    T_ff, T_fb, lag = T
    ff = np.logspace(np.log10(0.3), np.log10(20), 300)
    gc, pc = calc_tracking(ff, T_ff, T_fb, lag)
    fig, ax = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    ax[0].semilogx(ff, 20 * np.log10(gc), "k-", lw=1, label="baseline, Calculated (linear)")
    ax[1].semilogx(ff, pc, "k-", lw=1)
    for v, mk in (("bench", "^"), ("baseline", "o"), ("legacy", "s")):
        pts = sorted((r["post"]["f"], r["post"]["gain"], r["post"]["phase_deg"]) for r in res
                     if r["group"] == "freq" and r["variant"] == v)
        f_, g_, p_ = zip(*pts)
        ax[0].semilogx(f_, 20 * np.log10(g_), mk, label=f"{v}, Simulated")
        ax[1].semilogx(f_, p_, mk)
        checks[f"freq_{v}"] = dict(zip(map(str, f_), zip(g_, p_)))
    ax[0].set_ylabel("|q / q_req| (dB)"); ax[1].set_ylabel("phase (deg)"); ax[1].set_xlabel("frequency (Hz)")
    for a in ax:
        a.grid(alpha=.25, which="both")
    ax[0].legend(fontsize=8)
    fig.suptitle("Roll tracking response, yaw still, d(t) off (amplitude 10° down to 2° at 6 Hz)"); fig.tight_layout()
    fig.savefig(figdir / "task4b_frequency.png", dpi=130); plt.close(fig)

    # fault / recovery timeline
    r = _find(res, "timeline", seed=1)
    tr, ev = r["post"]["trace"], r["post"]["events"]
    m = (tr["t"] >= 2.8) & (tr["t"] <= 4.5)
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(tr["t"][m], tr["q_ref"][m] * DEG, lw=.8, alpha=.6, label="original request")
    axes[0].plot(tr["t"][m], tr["c_q_c"][m] * DEG, lw=.9, label="governed reference")
    axes[0].plot(tr["t"][m], tr["q"][m] * DEG, lw=.9, label="roll"); axes[0].set_ylabel("roll (deg)")
    axes[1].step(tr["t"][m], tr["mode"][m], where="post", label="drive fallback")
    axes[1].step(tr["t"][m], tr["c_host_fallback"][m] + 0.02, where="post", label="host idle")
    axes[1].step(tr["t"][m], tr["c_suspended"][m] - 0.02, where="post", label="request suspended")
    axes[1].set_ylabel("flag")
    axes[2].plot(tr["t"][m], tr["i"][m], lw=.7, label="measured current")
    axes[2].plot(tr["t"][m], tr["i_lim"][m], "k--", lw=.8); axes[2].plot(tr["t"][m], -tr["i_lim"][m], "k--", lw=.8)
    axes[2].set_ylabel("current (A)"); axes[2].set_xlabel("time (s)")
    for a in axes:
        a.axvspan(3.0, 3.1, color="orange", alpha=.15)
        for te, name in ev:
            if 2.8 <= te <= 4.5:
                a.axvline(te, color="r" if not name.startswith("rearm") else "g", lw=.6, alpha=.6)
        a.grid(alpha=.2); a.legend(fontsize=7, loc="upper right")
    seq = " → ".join("re-arm" if n.startswith("rearm_after_") else n for _, n in ev if 2.8 <= _ <= 4.5)
    if any(v == 1 for v in tr["c_suspended"][m]):
        seq += " → request suspended (held)"
    fig.suptitle(f"E, CoM +35 mm, seed 1, 100 ms feedback loss (orange):\n{seq} (Simulated)", fontsize=10)
    fig.tight_layout(); fig.savefig(figdir / "task4b_fault_timeline.png", dpi=130); plt.close(fig)
    checks["timeline_events"] = [(round(a, 3), b) for a, b in ev]

    # generalization
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.2))
    d = sorted((float(r["key"].split()[1]), r["row"]) for r in res if r["group"] == "gen_payload" and r["key"].startswith("D"))
    m2 = sorted((float(r["key"].split()[1]), r["row"]) for r in res if r["group"] == "gen_payload" and r["key"].startswith("M2"))
    ax[0].plot([x for x, _ in d], [100 * s["net_progress"] for _, s in d], "o-", label="D net progress %")
    ax[0].plot([x for x, _ in m2], [100 * s["net_progress"] for _, s in m2], "s-", label="M2 net progress %")
    for x, s in m2:
        ax[0].annotate("done" if s["completed"] else "not done", (x, 100 * s["net_progress"]), fontsize=7,
                       textcoords="offset points", xytext=(0, 6))
    ax2 = ax[0].twinx()
    ax2.plot([x for x, _ in d], [s["path_rms"] for _, s in d], "o--", color="gray", label="D path RMS °")
    ax2.set_ylabel("D path RMS (deg)")
    ax[0].set_xlabel("payload at +35 mm (kg)"); ax[0].set_ylabel("net progress (%)"); ax[0].legend(fontsize=8, loc="center left")
    for v, mk in (("baseline", "o-"), ("plan", "s--")):
        pts = sorted((float(r["key"][3:]), r["row"]) for r in res if r["group"] == "gen_coupling" and r["variant"] == v)
        ax[1].plot([x for x, _ in pts], [100 * s["net_progress"] for _, s in pts], mk, label=f"{v}: net progress %")
        ax[1].plot([x for x, _ in pts], [100 * s["yaw_scale"] for _, s in pts], mk, alpha=.5, label=f"{v}: final yaw scale %")
    ax[1].set_xlabel("true coupling / nominal (C)"); ax[1].set_ylabel("%"); ax[1].legend(fontsize=8)
    for a in ax:
        a.grid(alpha=.2)
    fig.suptitle("Generalization, seed 1 (Simulated)"); fig.tight_layout()
    fig.savefig(figdir / "task4b_generalization.png", dpi=130); plt.close(fig)

    # infeasible
    fig, axes = plt.subplots(2, 2, figsize=(12, 6), sharex="col")
    for col, key in enumerate(("INF-P", "INF-R no coordination")):
        r = _find(res, "infeasible", key, "baseline", 1)
        tr = r["post"]["trace"]
        axes[0, col].plot(tr["t"], tr["q_ref"] * DEG, lw=.8, alpha=.6, label="original request")
        axes[0, col].plot(tr["t"], tr["c_q_c"] * DEG, lw=.9, label="governed reference")
        axes[0, col].plot(tr["t"], tr["q"] * DEG, lw=.9, label="roll")
        axes[0, col].set_title(key, fontsize=9); axes[0, col].set_ylabel("roll (deg)")
        axes[1, col].step(tr["t"], tr["mode"], where="post", label="drive fallback")
        axes[1, col].step(tr["t"], tr["c_suspended"] + 0.03, where="post", label="suspended")
        axes[1, col].set_ylabel("flag"); axes[1, col].set_xlabel("time (s)")
        for a in axes[:, col]:
            a.grid(alpha=.2); a.legend(fontsize=7)
        checks[f"inf_{key}"] = dict(q_end=float(tr["q"][-1] * DEG), susp_share=float(np.mean(tr["c_suspended"])))
    fig.suptitle("Infeasible requests: disposition and actual motion (Simulated)"); fig.tight_layout()
    fig.savefig(figdir / "task4b_infeasible.png", dpi=130); plt.close(fig)
    return checks


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description="Packet 4B matched baseline validation")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--quick", action="store_true", help="one seed per case; requires --out outside report/")
    ap.add_argument("--out", default=None, help="publish directory (default: report/)")
    args = ap.parse_args(argv)
    out = Path(args.out).resolve() if args.out else ROOT / "report"
    if args.quick and out == ROOT / "report":
        raise SystemExit("--quick output must not be published to report/")
    fp0 = baseline_fingerprint()[0][:16]
    if fp0 != FROZEN:
        raise SystemExit(f"baseline fingerprint {fp0} is not the frozen {FROZEN}: refusing to evaluate")
    import json
    published = json.loads((ROOT / "report/task2_results.json").read_text()).get("montecarlo", {})
    specs = all_specs(SimConfig(), quick=args.quick)
    print(f"Packet 4B: {len(specs)} specs, {args.jobs} jobs, baseline {fp0}", flush=True)
    book, res = evaluate_all(specs, args.jobs)
    print(f"{len(book.runs)} manifested runs", flush=True)

    lines = []
    c_lines, c_rows = sec_contracts(res); lines += c_lines
    lines += sec_matched(res)
    lines += sec_finite(res)
    reg_lines, n_reg = sec_regressions(res); lines += reg_lines
    lines += sec_stress(res)
    mc_lines, mc_counts, n_faulted = sec_mc(res, published); lines += mc_lines
    lines += sec_infeasible(res)
    f_lines, T = sec_freq(res); lines += f_lines
    lines += sec_general(res)
    lines += sec_robust(res)
    lines += ["Figures: [saturation](figs/task4b_saturation.png), [frequency response](figs/task4b_frequency.png), "
              "[fault timeline](figs/task4b_fault_timeline.png), [generalization](figs/task4b_generalization.png), "
              "[infeasible cases](figs/task4b_infeasible.png)."]

    def check(stage):
        book.check_unchanged(stage)
        if baseline_fingerprint()[0][:16] != FROZEN:
            raise RuntimeError("baseline fingerprint changed during the evaluation")

    fp = book.fingerprint()
    git = fp["git"] or {}
    head = ["# Packet 4B — matched baseline validation (generated)", "",
            "Generated by `python -m exp.task4b_eval`. **Simulated** unless marked **Calculated**; no hardware "
            "observation. Thresholds (4A tracked/completed rules, fault-cause rule) are project assumptions."
            + (" **QUICK MODE: one seed per case — not evidence.**" if args.quick else ""), "",
            f"**Frozen baseline fingerprint:** `{fp['baseline'][:16]}` (required `{FROZEN}`; exp/evidence.py "
            f"BASELINE_SOURCES), git `{(git.get('commit') or '?')[:12]}`, baseline sources "
            f"{'clean' if not git.get('dirty_sources') else 'MODIFIED: ' + ', '.join(git['dirty_sources'])}. "
            f"Run manifests: declared-set code hash `{fp['code_hash'][:16]}` (per-file sha256 in "
            f"[task4b_runs.json](task4b_runs.json); uncommitted files in the set: "
            f"{', '.join(fp['run_set_git'].get('dirty_sources') or []) or 'none'}); metrics version "
            f"{fp['metrics_version']}. {len(book.runs)} runs; every row cites a run_id or a run-set id "
            "(member run_ids per sample in [task4b_results.json](task4b_results.json)).", "",
            "**Run-id caveat (review 4B I5):** run_ids hash the declared source set, which includes every file under "
            "ctrl/ and sim/ — also the Task 3 co-worker's uncommitted, still-changing `ctrl/payload_estimator.py`, "
            "which these runs never import. Re-running later reproduces the *metrics* exactly (the baseline sources are "
            "fingerprinted and unchanged), but re-derives the same *run_ids* only with the code hash recorded in "
            "task4b_runs.json. `exp.manifest.rebuild_run` also ignores the manifest's `extra` field, so the yaw-mismatch "
            "runs are rebuilt with `exp.task4b_eval.run_mismatch`.", ""]
    results = dict(fingerprint=fp, contracts=c_rows, over_tracking=over_tracking_band(*T), n_regressions=n_reg, mc_counts=mc_counts, n_faulted=n_faulted,
                   T_ff=T[0], T_fb_age=T[1], gov_lag=T[2],
                   runs=[dict(group=r["group"], key=r["key"], variant=r["variant"], seed=r["seed"],
                              run_id=r["run_id"], row=r["row"],
                              post={k: v for k, v in (r["post"] or {}).items() if k != "trace"})
                         for r in res])
    with MF.staged_publish(out, staging_root=Path(tempfile.gettempdir()) / "task4b_stage", check=check) as stage:
        results["figure_checks"] = figures(res, stage / "figs", T)
        (stage / "task4b_numbers.md").write_text("\n".join(head + lines) + "\n")
        MF.write_json(results, stage / "task4b_results.json")
        MF.write_json(book.record(), stage / "task4b_runs.json")
    print(f"Published task4b_numbers.md, task4b_results.json, task4b_runs.json and figures to {out}", flush=True)


if __name__ == "__main__":
    main()
