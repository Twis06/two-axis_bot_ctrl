"""Scalar summaries of a Log, matching the quantities reported for Runs A-E."""
import math

import numpy as np

DEG = 180 / math.pi


def summarize(log, t_skip=0.0, R=1.8):
    m = log.t >= t_skip
    e = log.err[m] * DEG
    i = log.i[m]
    dt = log.t[1] - log.t[0]
    age = log.cmd_age[m]
    age = age[np.isfinite(age)]
    out = dict(
        rms=float(np.sqrt(np.mean(e ** 2))),
        peak=float(np.max(np.abs(e))),
        mean=float(np.mean(e)),
        i_peak=float(np.max(np.abs(i))),
        i_rms=float(np.sqrt(np.mean(i ** 2))),
        clip_pct=100 * float(np.mean(log.clipped[m])),
        vlim_pct=100 * float(np.mean(log.vlim[m] > 0)),
        fallback_pct=100 * float(np.mean(log.mode[m] > 0)),
        wd_trips=log.meta.get("wd_trips", 0),
        heat_J=float(np.sum(i ** 2) * R * dt),
        cmd_age_p50=1e3 * float(np.median(age)) if age.size else float("nan"),
        cmd_age_max=1e3 * float(np.max(age)) if age.size else float("nan"),
        # Number of transitions into saturation - "exits and re-enters"
        sat_entries=int(np.sum(np.diff(log.clipped[m].astype(int)) == 1)),
    )
    if "c_q_c" in log:     # governed controllers: tracking of what was actually commanded
        eg = (log.q[m] - log.c_q_c[m]) * DEG
        out.update(rms_gov=float(np.sqrt(np.nanmean(eg ** 2))), peak_gov=float(np.nanmax(np.abs(eg))),
                   speed=float(np.nanmean(log.c_gov_s[m])), rejected_pct=100 * float(np.nanmean(log.c_gov_rejected[m])),
                   lag_end=float(log.c_gov_lag[-1]))
    else:
        out.update(rms_gov=out["rms"], peak_gov=out["peak"], speed=1.0, rejected_pct=0.0, lag_end=0.0)
    ys = log.meta.get("yaw_scale")
    out["yaw_scale"] = float(ys[-1][1]) if ys else 1.0
    out["events"] = len(getattr(log, "events", []))
    return out


def fmt_row(name, s, keys=("rms", "peak", "mean", "i_peak", "clip_pct", "sat_entries", "wd_trips")):
    return [name] + [f"{s[k]:.2f}" if isinstance(s[k], float) else str(s[k]) for k in keys]


# ---------------------------------------------------------------------------
# Packet 4A metrics. Additive: summarize()/fmt_row() above are unchanged.
#
# Why separate measures: a governor may slow, restrict or reject a request, and
# a host in fallback re-aligns its reference to the measured position. Error
# against that governed reference then shrinks exactly when the axis is not
# doing what was asked. So every run reports (a) error against the ORIGINAL
# wall-clock request, (b) error against the GOVERNED reference, (c) error
# against the original PATH at the governor's path time sigma, (d) how much of
# the path was actually DELIVERED, and, for finite motions, (e) whether and
# when the plant physically COMPLETED it. Success needs (c)-(e), never (b) alone.
# ---------------------------------------------------------------------------
from sim import params as _P

METRICS_VERSION = "4A.4"

# Declared thresholds. PROJECT DESIGN ASSUMPTIONS, not assessment requirements.
TRACK_PROGRESS_MIN = 0.95     # net delivered path time / requested path time
TRACK_PATH_RMS_DEG = 2.0      # RMS error vs the original path at sigma, request window
COMPLETE_TOL_DEG = 2.0        # waypoint / final-position tolerance
COMPLETE_SETTLE_S = 0.2       # minimum length of the final settled interval
COMPLETE_V_TOL = 0.1          # rad/s, |qd| bound while settled
COMPLETE_OVERSHOOT_DEG = 5.0  # max excursion beyond the finite path's swept range
RECOVERY_S = 0.5              # window after a fallback/suspension/rejection ends, a re-arm or a replan
GEO_WINDOW_S = 0.25           # +/- path-time window for the geometric distance
# Communication faults that re-arm automatically once fresh traffic resumes.
# Every other drive event (watchdog_trip, overspeed, overtemp, lockout,
# nonfinite_target, anything unknown) is a latched fault that fails `tracked`.
TRANSIENT_FAULTS = ("cmd_timeout", "invalid_command")

# Index order = label values. Priority (highest first): rejected > fallback >
# suspended > recovery > reshaping > normal; startup only labels pure-fallback
# samples before the first active one. Rationale: a rejection is the request's
# disposition, whatever the axis does; fallback means the drive is not running
# host commands at all (motion reduction only); a suspended request is under
# closed-loop control but paused until replan(), so it outranks recovery and
# reshaping, where the request is still being executed; suspension during
# drive fallback is reported as fallback, the physical state.
STRATA = ("startup", "normal", "reshaping", "recovery", "suspended", "fallback", "rejected")
_GOV_STATUS = ("accepted", "reshaped", "joining", "restricted", "over_budget", "rejected")

METRIC_DEFS = {
    "version": METRICS_VERSION,
    "phase_completion": "optional own-segment [start,end) governor path-time waypoint visits; final end=None permits slack; elapsed completion includes calibration; whole-run overshoot retained",
    "orig": "q(t) - q_req(t): original request on the wall clock (log.err)",
    "gov": "q(t) - c_q_c(t): governed reference actually commanded (host rate, forward-filled)",
    "sigma": "c_gov_sigma if logged (forward-filled over uninitialised NaN), else host tick time - c_gov_lag; "
             "sigma = t where c_gov_active == 0, where evaluate(governed=False), or without governor telemetry. "
             "sigma is held over the half host period between host ticks (known alignment artefact: "
             "~0.07 deg RMS for a perfect tracker of M1)",
    "path": "q(t) - q_req(sigma(t))",
    "geo": f"distance of q(t) from [min, max] of q_req over sigma(t) +/- {GEO_WINDOW_S} s",
    "request_window": "samples with sigma <= T_request (the requested motion, excluding end slack); "
                      "*_request error keys and the tracking threshold use it",
    "progress": "net: increments of the running max of sigma clipped to [sigma_start, T_request], counted "
                "only when <= one host period and while healthy at both ends; divided by T_request (<= 1). "
                "Forward clock jumps and backsteps are counted separately, never as progress",
    "healthy": "drive mode 0, host not in fallback, request not rejected (c_request_rejected or status "
               "rejected), not suspended (c_suspended) and no coordinated stop (c_coord_stop)",
    "tracked": f"progress >= {TRACK_PROGRESS_MIN}; request-window path RMS <= {TRACK_PATH_RMS_DEG} deg; no "
               "rejected, suspended or coordinated-stop sample; no latched fault event other than "
               f"{', '.join(TRANSIENT_FAULTS)} (and their re-arms); no path-clock backstep; finite motions completed",
    "completed": f"plant visits every waypoint in order within {COMPLETE_TOL_DEG} deg on a healthy sample; "
                 f"arrives: {COMPLETE_SETTLE_S} s of consecutive healthy samples within {COMPLETE_TOL_DEG} deg of "
                 f"the final target with |qd| <= {COMPLETE_V_TOL} rad/s (t_complete = its start); then stays "
                 f"within {COMPLETE_TOL_DEG} deg to the END of the run, healthy except for fallback caused by "
                 f"{', '.join(TRANSIENT_FAULTS)} (reported as post_arrival_transient); leaving the band restarts the "
                 "search for arrival; a latched fault, suspension, coordinated stop or rejection after the first "
                 "arrival voids completion (voided_at); excursion beyond "
                 f"the path range <= {COMPLETE_OVERSHOOT_DEG} deg",
    "strata": "priority rejected > fallback > suspended > recovery > reshaping > normal; startup = pure "
              "fallback samples before the first active one (rejected/suspended samples are never startup); "
              "reshaping = governor status reshaped/joining/restricted/over_budget; fallback = drive mode 1 or "
              f"host fallback; recovery = {RECOVERY_S} s after a fallback/suspended/rejected span ends (except "
              "the startup span), after a rearm_after_* event, or after a replan (c_replans increments)",
    "host_demand": "c_i_unsat if logged, else (c_tau_ff + c_tau_fb) / K_T nominal: host current demand "
                   "before its own clamp",
}


def _col(log, key):
    return np.asarray(log[key], float) if key in log else np.full(len(log.t), np.nan)


def _flag(log, key):
    return _col(log, key) == 1


def _div(log):
    cfg = getattr(log, "meta", {}).get("cfg")
    return int(round(cfg.timing.f_drive / cfg.timing.f_ctrl)) if cfg is not None else 1


def _rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def _peak(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.max(np.abs(x))) if x.size else float("nan")


def spans(mask, t, dt):
    """[(t_start, t_end)] of consecutive True runs; t_end is exclusive."""
    m = np.concatenate(([0], np.asarray(mask, int), [0]))
    d = np.diff(m)
    return [(float(t[a]), float(t[b - 1] + dt)) for a, b in zip(np.where(d == 1)[0], np.where(d == -1)[0])]


def _span_idx(mask):
    m = np.concatenate(([0], np.asarray(mask, int), [0]))
    d = np.diff(m)
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0]))


def _ffill(x):
    fin = np.isfinite(x)
    if not fin.any():
        return np.full(len(x), np.nan)
    idx = np.where(fin, np.arange(len(x)), 0)
    np.maximum.accumulate(idx, out=idx)
    out = x[idx]
    out[:np.argmax(fin)] = np.nan
    return out


def _host_idle(log):
    return _flag(log, "c_host_fallback")


def _rejected(log):
    return _flag(log, "c_request_rejected") | (_col(log, "c_gov_status") == _GOV_STATUS.index("rejected"))


def _suspended(log):
    return _flag(log, "c_suspended") | _flag(log, "c_coord_stop")


def _fallback(log):
    return (np.asarray(log.mode) > 0) | _host_idle(log)


def healthy(log):
    """Per-sample: the request is being executed (METRIC_DEFS['healthy'])."""
    return ~(_fallback(log) | _rejected(log) | _suspended(log))


def _governed(log, governed=None):
    """Per-sample bool: is sigma the governor's path time (else the wall clock)?"""
    n = len(log.t)
    if governed is not None:
        return np.full(n, bool(governed))
    if "c_gov_active" in log:
        a = _ffill(_col(log, "c_gov_active"))
        fin = np.isfinite(a)
        if fin.any():
            a[~fin] = a[fin][0]           # before the first host tick: the run's first declaration
        return ~(a == 0)
    return np.full(n, "c_gov_sigma" in log or "c_gov_lag" in log)


def path_clock(log, governed=None):
    """Governor path time sigma(t) at every drive tick (NaN until known).

    Preferred source is the explicit c_gov_sigma (NaN while the controller is
    uninitialised; held over those samples). Without it, sigma is rebuilt from
    c_gov_lag, referring forward-filled telemetry to the host tick that produced
    it (t - (k mod div) dt); a lag of exactly 0 while the host idles is the
    uninitialised marker and is held too. Ungoverned samples use sigma = t.
    """
    t = np.asarray(log.t, float)
    gov = _governed(log, governed)
    if not gov.any():
        return t.copy()
    if "c_gov_sigma" in log:
        sig = _ffill(_col(log, "c_gov_sigma"))
    else:
        dt = float(t[1] - t[0])
        t_host = t - (np.arange(len(t)) % _div(log)) * dt
        lag = _col(log, "c_gov_lag")
        sig = t_host - lag
        sig[_host_idle(log) & (lag == 0)] = np.nan
        sig = _ffill(sig)
    return np.where(gov, sig, t)


def progress(log, T_request, governed=None):
    """Net delivered path time and its fraction of the request (METRIC_DEFS)."""
    sig = path_clock(log, governed)
    div = _div(log)
    dt = float(log.t[1] - log.t[0])
    k = np.arange(0, len(sig), div)
    ok = healthy(log)[k]
    s = sig[k]
    fin = np.isfinite(s)
    out = dict(path_time=0.0, T_request=float(T_request), progress=0.0, sigma_start=float("nan"),
               sigma_end=float("nan"), clock_jumps=0, clock_backsteps=0)
    if not fin.any() or T_request <= 0:
        return out
    s0 = float(s[fin][0])
    rm = np.maximum.accumulate(np.where(fin, np.clip(s, s0, max(T_request, s0)), s0))
    inc = np.diff(rm)
    step = div * dt * (1 + 1e-6) + 1e-12
    pair = fin[1:] & fin[:-1]
    good = pair & ok[1:] & ok[:-1] & (inc <= step)
    ds = np.diff(s)
    delivered = float(np.sum(inc[good]))
    out.update(path_time=delivered, progress=min(1.0, delivered / T_request), sigma_start=s0,
               sigma_end=float(s[fin][-1]), clock_jumps=int(np.sum(pair & (ds > step))),
               clock_backsteps=int(np.sum(pair & (ds < -1e-9))))
    return out


def path_errors(log, ref=None, window=GEO_WINDOW_S, governed=None):
    """Per-sample error arrays (rad): orig, gov, path, geo, plus sigma. `ref` is
    the original roll request (object with eval(t)); without it path/geo are NaN."""
    from scipy.ndimage import maximum_filter1d, minimum_filter1d
    q = np.asarray(log.q, float)
    sig = path_clock(log, governed)
    out = dict(orig=np.asarray(log.err, float),
               gov=q - _col(log, "c_q_c") if "c_q_c" in log else np.asarray(log.err, float), sigma=sig)
    if ref is None:
        out["path"] = out["geo"] = np.full(len(q), np.nan)
        return out
    fin = np.isfinite(sig)
    qp = np.full(len(q), np.nan)
    qp[fin] = [ref.eval(s)[0] for s in sig[fin]]
    out["path"] = q - qp
    dt = float(log.t[1] - log.t[0])
    s_max = float(np.nanmax(sig)) if fin.any() else 0.0
    grid = np.arange(0.0, max(s_max, 0.0) + window + 2 * dt, dt)
    qg = np.array([ref.eval(s)[0] for s in grid])
    n = 2 * int(round(window / dt)) + 1
    lo, hi = minimum_filter1d(qg, n, mode="nearest"), maximum_filter1d(qg, n, mode="nearest")
    geo = np.full(len(q), np.nan)
    j = np.clip(np.round(sig[fin] / dt).astype(int), 0, len(grid) - 1)
    geo[fin] = np.maximum.reduce([lo[j] - q[fin], q[fin] - hi[j], np.zeros(j.size)])
    out["geo"] = geo
    return out


def strata(log, recovery_s=RECOVERY_S):
    """Integer label per sample, indexing STRATA (priority in METRIC_DEFS)."""
    t = np.asarray(log.t, float)
    lab = np.full(len(t), STRATA.index("normal"))
    lab[np.isin(_col(log, "c_gov_status"), [1, 2, 3, 4])] = STRATA.index("reshaping")
    rejected, fallback, susp = _rejected(log), _fallback(log), _suspended(log)
    bad = rejected | fallback | susp
    n0 = int(np.argmax(~bad)) if (~bad).any() else 0
    starts = []
    for a, b in _span_idx(bad):
        if a == 0 and b == n0 and not (rejected[a:b] | susp[a:b]).any():
            continue                      # pure startup span: nothing to recover from
        if b < len(t):
            starts.append(t[b])
    starts += [te for te, name in getattr(log, "events", []) if name.startswith("rearm_after_")]
    rp = _ffill(_col(log, "c_replans"))
    starts += list(t[1:][np.nan_to_num(np.diff(rp)) > 0])
    rec = np.zeros(len(t), bool)
    for t1 in starts:
        rec |= (t >= t1) & (t < t1 + recovery_s)
    lab[rec & ~bad] = STRATA.index("recovery")
    lab[susp] = STRATA.index("suspended")
    lab[fallback] = STRATA.index("fallback")
    lab[rejected] = STRATA.index("rejected")
    pre = np.zeros(len(t), bool)
    pre[:n0] = True
    lab[pre & ~rejected & ~susp] = STRATA.index("startup")
    return lab


def stratified(log, errs=None, ref=None, mask=None, recovery_s=RECOVERY_S):
    """Time share (%) and error per stratum; shares sum to 100 over the mask."""
    errs = errs if errs is not None else path_errors(log, ref)
    lab = strata(log, recovery_s)
    mask = np.ones(len(lab), bool) if mask is None else mask
    n = max(int(np.sum(mask)), 1)
    dt = float(log.t[1] - log.t[0])
    out = {}
    for i, name in enumerate(STRATA):
        m = mask & (lab == i)
        row = dict(share_pct=100.0 * float(np.sum(m)) / n, time_s=float(np.sum(m)) * dt)
        for key in ("orig", "gov", "path", "geo"):
            row[f"{key}_rms"] = _rms(errs[key][m] * DEG)
            row[f"{key}_peak"] = _peak(errs[key][m] * DEG)
        out[name] = row
    return out


def current_report(log, mask=None):
    """Commands at each stage (host demand -> drive target -> applied) and
    measured current against the active limit; saturation, voltage, thermal."""
    t = np.asarray(log.t, float)
    dt = float(t[1] - t[0])
    m = np.ones(len(t), bool) if mask is None else mask
    lim, i, raw, tgt = (np.asarray(log[k], float)[m] for k in ("i_lim", "i", "i_raw", "i_tgt"))
    demand = (_col(log, "c_i_unsat") if "c_i_unsat" in log
              else (_col(log, "c_tau_ff") + _col(log, "c_tau_fb")) / _P.K_T)[m]
    dfin = np.isfinite(demand)
    clip = np.asarray(log.clipped, bool)[m]
    vl = np.asarray(log.vlim, float)[m] > 0
    sat = spans(clip, t[m], dt)
    lim_all = np.asarray(log.i_lim, float)
    ch = np.where(np.diff(lim_all) != 0)[0] + 1
    Tw = _col(log, "T_wind")[m]
    T_end = getattr(log, "meta", {}).get("T_winding")
    over = np.abs(i) - lim
    return dict(
        host_demand_source="c_i_unsat" if "c_i_unsat" in log else "(c_tau_ff+c_tau_fb)/K_T",
        host_demand_peak=_peak(demand),
        host_demand_over_limit_pct=100 * float(np.mean(np.abs(demand[dfin]) > lim[dfin])) if dfin.any()
        else float("nan"),
        host_sat_pct=100 * float(np.nanmean(_col(log, "c_sat")[m])) if "c_sat" in log else float("nan"),
        drive_raw_peak=_peak(raw), target_peak=_peak(tgt), measured_peak=_peak(i), measured_rms=_rms(i),
        target_over_limit_max=float(np.max(np.abs(tgt) - lim)),
        measured_over_limit_max=float(np.max(over)), measured_over_limit_s=float(np.sum(over > 1e-9)) * dt,
        i_lim_min=float(np.min(lim)), i_lim_max=float(np.max(lim)),
        limit_changes=[(float(t[k]), float(lim_all[k])) for k in ch],
        sat_pct=100 * float(np.mean(clip)), sat_s=float(np.sum(clip)) * dt, sat_entries=len(sat),
        sat_longest_s=max((b - a for a, b in sat), default=0.0),
        vlim_pct=100 * float(np.mean(vl)), vlim_s=float(np.sum(vl)) * dt, vlim_entries=len(spans(vl, t[m], dt)),
        T_winding_max=float(np.nanmax(Tw)) if np.isfinite(Tw).any() else float("nan"),
        T_winding_end=float(T_end) if T_end is not None else float("nan"),
    )


def latched_fault_mask(log):
    """Per-sample: a non-transient fault (tracking_faults) is latched, i.e.
    from its event to the matching rearm_after_* event (or the end of the run)."""
    t = np.asarray(log.t, float)
    ev = sorted((float(a), b) for a, b in getattr(log, "events", []))
    m = np.zeros(len(t), bool)
    for k, (te, name) in enumerate(ev):
        if name in TRANSIENT_FAULTS or name.startswith("rearm_after_"):
            continue
        re = next((a for a, b in ev[k + 1:] if b == f"rearm_after_{name}"), float("inf"))
        m |= (t >= te) & (t < re)
    return m


def tracking_faults(log):
    """Latched fault events that are not automatic communication recoveries."""
    return [(float(a), b) for a, b in getattr(log, "events", [])
            if b not in TRANSIENT_FAULTS and not b.startswith("rearm_after_")]


def fault_timeline(log):
    """Event sequence, time to re-arm per latched fault, and spans of drive
    fallback, host fallback, rejection, suspension (with the controller's
    reasons from log.meta['notices'] when present), coordinated stop, lockout."""
    t = np.asarray(log.t, float)
    dt = float(t[1] - t[0])
    ev = sorted((float(a), b) for a, b in getattr(log, "events", []))
    counts = {}
    for _, name in ev:
        counts[name] = counts.get(name, 0) + 1
    latches = []
    for k, (te, name) in enumerate(ev):
        if name.startswith("rearm_after_"):
            continue
        re = next((a for a, b in ev[k + 1:] if b == f"rearm_after_{name}"), None)
        latches.append(dict(t=te, fault=name, rearm_t=re, down_s=None if re is None else re - te))
    notices = [(float(n[0]), n[1], n[2]) for n in getattr(log, "meta", {}).get("notices", []) or []
               if n[0] is not None]

    def summ(mask, statuses=()):
        sp = spans(mask, t, dt)
        out = dict(count=len(sp), total_s=float(sum(b - a for a, b in sp)),
                   longest_s=max((b - a for a, b in sp), default=0.0), spans=sp)
        if statuses:   # reason = latest matching notice issued up to 50 ms before/while the span runs
            out["reasons"] = [next((r for tn, s, r in reversed(notices) if s in statuses and a - 0.05 <= tn < b),
                                   None) for a, b in sp]
        return out
    rp = _col(log, "c_replans")
    return dict(events=[dict(t=a, name=b) for a, b in ev], counts=counts,
                rearms=sum(v for k, v in counts.items() if k.startswith("rearm_after_")),
                latches=latches, tracking_faults=[dict(t=a, name=b) for a, b in tracking_faults(log)],
                drive_fallback=summ(np.asarray(log.mode) > 0), host_fallback=summ(_host_idle(log)),
                rejected=summ(_rejected(log), ("rejected",)),
                suspended=summ(_flag(log, "c_suspended"), ("suspended",)),
                coord_stop=summ(_flag(log, "c_coord_stop"), ("incompatible",)),
                locked=summ(_col(log, "locked") == 1),
                replans=int(np.nanmax(rp) - np.nanmin(rp)) if np.isfinite(rp).any() else 0,
                notices_logged=len(notices))


def _amp_freq(x, t):
    """Half peak-to-peak amplitude and frequency from the spacing of zero
    crossings about the mid-range: (n - 1) / (2 (t_last - t_first))."""
    if x.size < 3:
        return float("nan"), float("nan")
    amp = 0.5 * float(np.max(x) - np.min(x))
    if amp <= 1e-9:
        return amp, 0.0
    y = x - 0.5 * (np.max(x) + np.min(x))
    keep = np.abs(y) > 1e-3 * amp
    y, tt = y[keep], t[keep]
    zc = np.where(np.diff(np.sign(y)) != 0)[0]
    if zc.size < 2:
        return amp, float("nan")
    return amp, float((zc.size - 1) / (2 * (tt[zc[-1]] - tt[zc[0]])))


def yaw_report(log, yaw_request=None, t_skip=1.0, t_end_window=1.0):
    """Original (requested) vs planned (host plan, if logged as qy_plan) vs
    delivered yaw amplitude/frequency, and coordination requests. Amplitude is
    half peak-to-peak (deg) over t >= t_skip and over the final t_end_window s."""
    t = np.asarray(log.t, float)
    m, me = t >= t_skip, t >= t[-1] - t_end_window
    out = {}
    series = [("delivered", np.asarray(log.qy, float))]
    if "qy_plan" in log:
        series.append(("planned", np.asarray(log.qy_plan, float)))
    if yaw_request is not None:
        series.append(("original", np.array([yaw_request.eval(x)[0] for x in t])))
    for name, y in series:
        a, f = _amp_freq(y[m] * DEG, t[m])
        out.update({f"{name}_amp": a, f"{name}_freq": f, f"{name}_amp_end": _amp_freq(y[me] * DEG, t[me])[0]})
    if "original_amp_end" in out:
        a0e = out["original_amp_end"]
        out["amp_ratio_end"] = out["delivered_amp_end"] / a0e if a0e > 1e-9 else float("nan")
    ys = getattr(log, "meta", {}).get("yaw_scale") or []
    reqs = [(float(a), float(b)) for a, b in ys[1:]]
    inc = _flag(log, "c_gov_incompatible")
    stop = _flag(log, "c_coord_stop")
    out.update(scale_requests=reqs, n_scale_requests=len(reqs),
               scale_end=float(ys[-1][1]) if ys else float("nan"),
               scale_min=float(min(b for _, b in ys)) if ys else float("nan"),
               coordination_available=bool(ys),
               incompatible_pct=100 * float(np.mean(inc)),
               incompatible_first_t=float(t[np.argmax(inc)]) if inc.any() else None,
               coord_stop_pct=100 * float(np.mean(stop)),
               coord_stop_first_t=float(t[np.argmax(stop)]) if stop.any() else None)
    return out


def completion(log, waypoints, t_request, tol_deg=COMPLETE_TOL_DEG, settle_s=COMPLETE_SETTLE_S,
               v_tol=COMPLETE_V_TOL, overshoot_deg=COMPLETE_OVERSHOOT_DEG,
               waypoint_windows=None, governed=None):
    """Finite-motion completion (METRIC_DEFS['completed']). A controller that
    rejects, falls back, is suspended, or stays put never completes."""
    t, q, qd = (np.asarray(log[k], float) for k in ("t", "q", "qd"))
    dt, tol = float(t[1] - t[0]), math.radians(tol_deg)
    ok = healthy(log)
    out = dict(completed=False, t_complete=float("nan"), t_request=float(t_request), time_ratio=0.0,
               waypoints=len(waypoints), reached=0, visit_t=[], settled_s=0.0)
    lo, hi = min(waypoints + [q[0]]), max(waypoints + [q[0]])
    out["overshoot_deg"] = float(max(0.0, np.max(q) - hi, lo - np.min(q))) * DEG
    if waypoint_windows is not None:
        return _phase_completion(log, waypoints, t_request, waypoint_windows,
                                 tol_deg, settle_s, v_tol, overshoot_deg, governed)
    j = 0
    for w in waypoints[:-1]:
        hit = np.where((np.abs(q[j:] - w) <= tol) & ok[j:])[0]
        if not hit.size:
            return out
        j += int(hit[0])
        out["visit_t"].append(float(t[j]))
        out["reached"] += 1
    n = int(round(settle_s / dt))
    in_band = np.abs(q - waypoints[-1]) <= tol
    # After arrival the plant must stay in the band to the end of the run. A
    # transient communication fallback (TRANSIENT_FAULTS) is excused there; a
    # latched fault, suspension, coordinated stop or rejection is not.
    hard = _rejected(log) | _suspended(log) | latched_fault_mask(log)
    excused = _fallback(log) & ~hard
    stay = in_band & (ok | excused)
    # Arrival: n consecutive healthy, slow, in-band samples (at rest, not passing through).
    arrive = (ok & in_band & (np.abs(qd) <= v_tol)).astype(int)

    def first_arrival(k0):
        run = np.convolve(arrive[k0:], np.ones(n, int), "valid") if len(t) - k0 >= n else np.array([], int)
        hit = np.where(run == n)[0]
        return k0 + int(hit[0]) if hit.size else None
    ja = first_arrival(j)
    if ja is not None and hard[ja:].any():
        # A latched fault, suspension, coordinated stop or rejection after the
        # motion arrived: the axis did not hold the completed motion. Void.
        k = ja + int(np.argmax(hard[ja:]))
        out["voided_at"] = float(t[k])
        return out
    miss = np.where(~stay[j:])[0]
    j0 = j + (int(miss[-1]) + 1 if miss.size else 0)       # in band from j0 to the end of the run
    jc = first_arrival(j0)
    if jc is None:
        out["settled_s"] = (len(t) - j0) * dt
        return out
    out["settled_s"] = (len(t) - jc) * dt
    out["reached"] += 1
    tc = float(t[jc])
    after = t >= tc
    out["post_arrival_transient"] = dict(
        events=[dict(t=float(a), name=b) for a, b in getattr(log, "events", []) if a >= tc and b in TRANSIENT_FAULTS],
        fallback_s=float(np.sum(after & ~ok)) * dt, spans=spans(after & ~ok, t, dt))
    out.update(t_complete=tc, time_ratio=t_request / tc if tc > 0 else float("nan"),
               completed=out["overshoot_deg"] <= overshoot_deg)
    return out


def _phase_completion(log, waypoints, t_request, windows, tol_deg, settle_s, v_tol, overshoot_deg, governed):
    """Own-segment [start, end) path-time visits; None end permits final slack.

    Calibration cannot earn a visit. Intermediate visits remain crossings,
    while the final target requires the legacy healthy settling/hold rule.
    Absolute timestamps preserve calibration cost in completion time.
    """
    if len(windows) != len(waypoints) or not windows:
        raise ValueError("one path-time window required per waypoint")
    if windows[-1][1] is not None:
        raise ValueError("final waypoint window must be unbounded for settling/hold")
    for k, (a, b) in enumerate(windows):
        if not math.isfinite(a) or a < 0 or (b is not None and (not math.isfinite(b) or b <= a)):
            raise ValueError("invalid waypoint window")
        if k and (windows[k - 1][1] is None or a < windows[k - 1][1]):
            raise ValueError("waypoint windows must be ordered and nonoverlapping")
    t, q = np.asarray(log.t), np.asarray(log.q)
    sig, ok = path_clock(log, governed), healthy(log)
    visits = [dict(index=k, target_rad=float(w), path_window=list(win),
                   wall_t=None, path_t=None, missing=True) for k, (w, win) in enumerate(zip(waypoints, windows))]
    out = dict(completed=False, t_complete=float("nan"), t_request=float(t_request), time_ratio=0.0,
               waypoints=len(waypoints), reached=0, visit_t=[], settled_s=0.0,
               scoring="own_segment_path_time_v1", waypoint_visits=visits,
               missing_waypoints=list(range(len(waypoints))))
    lo, hi = min(list(waypoints) + [q[0]]), max(list(waypoints) + [q[0]])
    # Keep whole-run excursion checking and safety evidence, including calibration.
    out['overshoot_deg'] = float(max(0.0, np.max(q) - hi, lo - np.min(q))) * DEG
    last = -1
    masks = []
    for a, b in windows:
        masks.append(np.isfinite(sig) & (sig >= a) & (True if b is None else sig < b))
    for k, w in enumerate(waypoints[:-1]):
        hit = np.flatnonzero(masks[k] & ok & (np.abs(q - w) <= math.radians(tol_deg)) & (np.arange(len(t)) > last))
        if hit.size:
            last = int(hit[0])
            visits[k].update(wall_t=float(t[last]), path_t=float(sig[last]), missing=False)
    final = np.flatnonzero(masks[-1] & (np.arange(len(t)) > last))
    if final.size:
        # Slice only the final phase, retaining absolute time and all event history.
        # The existing scorer supplies settling and post-arrival fault semantics.
        start = int(final[0])
        if len(t) - start >= 2:
            tail = type(log)({k: np.asarray(v)[start:].copy() for k, v in log.items()})
            tail.events, tail.meta = log.events, log.meta
            # Out-of-window samples cannot establish healthy arrival.
            idle = np.asarray(tail.get('c_host_fallback', np.zeros(len(tail.t)))).copy()
            idle[~masks[-1][start:]] = 1
            tail['c_host_fallback'] = idle
            c = completion(tail, [waypoints[-1]], t_request, tol_deg, settle_s, v_tol, float('inf'))
            for key in ('settled_s', 'voided_at', 'post_arrival_transient'):
                if key in c:
                    out[key] = c[key]
            if c['completed']:
                tc = c['t_complete']
                idx = int(np.searchsorted(t, tc))
                visits[-1].update(wall_t=tc, path_t=float(sig[idx]), missing=False)
                out.update(t_complete=tc, time_ratio=c['time_ratio'])
    out['missing_waypoints'] = [v['index'] for v in visits if v['missing']]
    out['reached'] = len(visits) - len(out['missing_waypoints'])
    out['visit_t'] = [v['wall_t'] for v in visits if not v['missing']]
    out['completed'] = not out['missing_waypoints'] and out['overshoot_deg'] <= overshoot_deg
    if not out['completed']:
        out.update(t_complete=float('nan'), time_ratio=0.0)
    return out


def evaluate(log, ref=None, yaw_request=None, T_request=None, waypoints=None, t_skip=0.0, governed=None, waypoint_windows=None):
    """All Packet 4A metrics for one run as a JSON-ready dict (NaN kept as float).

    ref: original roll request; T_request: requested path duration (finite
    motion end, or the run duration for periodic requests); waypoints: ordered
    targets of a finite motion (rad) to score completion; governed: override
    the telemetry's governor-in-use declaration (None = from c_gov_active).
    """
    t = np.asarray(log.t, float)
    mask = t >= t_skip
    T_request = float(T_request if T_request is not None else t[-1] + (t[1] - t[0]))
    errs = path_errors(log, ref, governed=governed)
    req = mask & (errs["sigma"] <= T_request + 1e-9)
    e = {}
    for k in ("orig", "gov", "path", "geo"):
        e.update({f"{k}_rms": _rms(errs[k][mask] * DEG), f"{k}_peak": _peak(errs[k][mask] * DEG),
                  f"{k}_rms_request": _rms(errs[k][req] * DEG), f"{k}_peak_request": _peak(errs[k][req] * DEG)})
    prog = progress(log, T_request, governed)
    st = stratified(log, errs, mask=mask)
    ft = fault_timeline(log)
    out = dict(metrics_version=METRICS_VERSION, t_skip=float(t_skip), errors=e, progress=prog, strata=st,
               current=current_report(log, mask), faults=ft,
               yaw=yaw_report(log, yaw_request), summary=summarize(log, t_skip))
    reasons = []
    if not prog["progress"] >= TRACK_PROGRESS_MIN:
        reasons.append(f"progress {prog['progress']:.2f} < {TRACK_PROGRESS_MIN}")
    if not e["path_rms_request"] <= TRACK_PATH_RMS_DEG:
        reasons.append(f"request-window path RMS {e['path_rms_request']:.2f} deg > {TRACK_PATH_RMS_DEG}"
                       if ref is not None else "no original request supplied: path error unknown")
    if prog["clock_backsteps"]:
        reasons.append(f"path clock stepped back {prog['clock_backsteps']} times")
    if np.any(_rejected(log)[mask]):
        reasons.append(f"rejected {100 * float(np.mean(_rejected(log)[mask])):.1f}% of time")
    for key in ("suspended", "coord_stop"):
        if ft[key]["count"]:
            reasons.append(f"{key} {ft[key]['total_s']:.2f} s ({ft[key]['count']} spans)")
    if ft["tracking_faults"]:
        names = sorted({f["name"] for f in ft["tracking_faults"]})
        reasons.append(f"latched faults: {', '.join(names)} (first at {ft['tracking_faults'][0]['t']:.3f} s)")
    if waypoints is not None:
        out["completion"] = completion(log, list(waypoints), T_request, waypoint_windows=waypoint_windows, governed=governed)
        if not out["completion"]["completed"]:
            reasons.append("finite motion not completed")
    out["tracked"] = not reasons
    out["not_tracked_because"] = reasons
    return out
