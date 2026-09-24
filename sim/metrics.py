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

METRICS_VERSION = "4A.1"

# Declared thresholds. PROJECT DESIGN ASSUMPTIONS, not assessment requirements.
TRACK_PROGRESS_MIN = 0.95     # delivered path time / requested path time
TRACK_PATH_RMS_DEG = 2.0      # RMS error vs the original path at sigma
COMPLETE_TOL_DEG = 2.0        # waypoint / final-position tolerance
COMPLETE_SETTLE_S = 0.2       # stay in the final band this long
COMPLETE_V_TOL = 0.1          # rad/s, |qd| bound while settled
COMPLETE_OVERSHOOT_DEG = 5.0  # max excursion beyond the finite path's swept range
RECOVERY_S = 0.5              # window after a fallback/rejection span ends
GEO_WINDOW_S = 0.25           # +/- path-time window for the geometric distance

STRATA = ("startup", "normal", "reshaping", "fallback", "recovery", "rejected")
_GOV_STATUS = ("accepted", "reshaped", "joining", "restricted", "over_budget", "rejected")

METRIC_DEFS = {
    "version": METRICS_VERSION,
    "orig": "q(t) - q_req(t): original request on the wall clock (log.err)",
    "gov": "q(t) - c_q_c(t): governed reference actually commanded (host rate, forward-filled)",
    "path": "q(t) - q_req(sigma(t)); sigma = host tick time - c_gov_lag; sigma = t if ungoverned",
    "geo": f"distance of q(t) from [min, max] of q_req over sigma(t) +/- {GEO_WINDOW_S} s",
    "progress": "sum of path-clock increments 0 <= dsigma <= host period, sigma clipped to the "
                "request duration, counted only while host and drive are both in normal mode; "
                "forward clock jumps (replan/realign) are counted separately, never as progress",
    "tracked": f"progress >= {TRACK_PROGRESS_MIN}, path RMS <= {TRACK_PATH_RMS_DEG} deg, no rejected "
               "time, and (finite motions) completed",
    "completed": f"plant visits every waypoint in order within {COMPLETE_TOL_DEG} deg, then stays within "
                 f"{COMPLETE_TOL_DEG} deg of the final target with |qd| <= {COMPLETE_V_TOL} rad/s for "
                 f"{COMPLETE_SETTLE_S} s while drive normal, host not in fallback and request not rejected; "
                 f"excursion beyond the path range <= {COMPLETE_OVERSHOOT_DEG} deg",
    "strata": "priority startup > rejected > fallback > recovery > reshaping > normal; startup = "
              "before the first sample with drive normal and host active; "
              "reshaping = governor status reshaped/joining/restricted/over_budget; fallback = drive "
              f"mode 1 or host fallback; recovery = {RECOVERY_S} s after a fallback/rejected span ends",
    "host_demand": "(c_tau_ff + c_tau_fb) / K_T nominal: host current demand before its own clamp",
}


def _col(log, key):
    return np.asarray(log[key], float) if key in log else np.full(len(log.t), np.nan)


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


def _ffill(x):
    fin = np.isfinite(x)
    if not fin.any():
        return np.full(len(x), np.nan)
    idx = np.where(fin, np.arange(len(x)), 0)
    np.maximum.accumulate(idx, out=idx)
    out = x[idx]
    out[:np.argmax(fin)] = np.nan
    return out


def _governed(log):
    return "c_gov_lag" in log


def _host_idle(log):
    return _col(log, "c_host_fallback") == 1


def path_clock(log):
    """Governor path time sigma(t) at every drive tick (NaN until known).

    Telemetry is forward-filled from host ticks, so the lag is referred to the
    host tick that produced it (t - (k mod div) dt), not the drive tick.
    The controller reports lag = 0 while uninitialised (before first valid
    feedback, after replan()); those samples carry no path time and are held.
    Ungoverned controllers follow the wall clock by construction: sigma = t.
    """
    t = np.asarray(log.t, float)
    if not _governed(log):
        return t.copy()
    dt = float(t[1] - t[0])
    t_host = t - (np.arange(len(t)) % _div(log)) * dt
    lag = _col(log, "c_gov_lag")
    sig = t_host - lag
    sig[_host_idle(log) & (lag == 0)] = np.nan
    return _ffill(sig)


def progress(log, T_request):
    """Delivered path time and its fraction of the request (see METRIC_DEFS)."""
    sig = path_clock(log)
    div = _div(log)
    dt = float(log.t[1] - log.t[0])
    k = np.arange(0, len(sig), div)
    ok = ~_host_idle(log)[k] & (np.asarray(log.mode)[k] == 0)
    s = np.minimum(sig[k], T_request)
    ds = np.diff(s)
    step = div * dt * (1 + 1e-6) + 1e-12
    fin = np.isfinite(ds)
    good = fin & ok[1:] & ok[:-1] & (ds >= -1e-12) & (ds <= step)
    delivered = float(np.sum(ds[good]))
    last = sig[np.isfinite(sig)]
    return dict(path_time=delivered, T_request=float(T_request),
                progress=delivered / T_request if T_request > 0 else float("nan"),
                sigma_end=float(last[-1]) if last.size else float("nan"),
                clock_jumps=int(np.sum(fin & (ds > step))),
                clock_backsteps=int(np.sum(fin & (ds < -1e-9))))


def path_errors(log, ref=None, window=GEO_WINDOW_S):
    """Per-sample error arrays (rad): orig, gov, path, geo. `ref` is the
    original roll request (object with eval(t)); without it path/geo are NaN."""
    from scipy.ndimage import maximum_filter1d, minimum_filter1d
    q = np.asarray(log.q, float)
    out = dict(orig=np.asarray(log.err, float),
               gov=q - _col(log, "c_q_c") if _governed(log) else np.asarray(log.err, float))
    if ref is None:
        out["path"] = out["geo"] = np.full(len(q), np.nan)
        return out
    sig = path_clock(log)
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
    dt = float(t[1] - t[0])
    lab = np.full(len(t), STRATA.index("normal"))
    st = _col(log, "c_gov_status")
    lab[np.isin(st, [1, 2, 3, 4])] = STRATA.index("reshaping")
    rejected = (_col(log, "c_request_rejected") == 1) | (st == 5)
    fallback = (np.asarray(log.mode) > 0) | _host_idle(log)
    bad = rejected | fallback
    # Startup: before the first sample with drive and host both active (no command
    # has reached the drive yet). A run that never becomes active has no startup
    # stratum: all of it is fallback/rejected.
    n0 = int(np.argmax(~bad)) if (~bad).any() else 0
    rec = np.zeros(len(t), bool)
    for t0, t1 in spans(bad, t, dt):
        if t0 >= t[n0]:
            rec |= (t >= t1) & (t < t1 + recovery_s)
    for te, name in getattr(log, "events", []):
        if name.startswith("rearm_after_"):
            rec |= (t >= te) & (t < te + recovery_s)
    lab[rec & ~bad] = STRATA.index("recovery")
    lab[fallback] = STRATA.index("fallback")
    lab[rejected] = STRATA.index("rejected")
    lab[:n0] = STRATA.index("startup")
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
    measured current against the active limit; saturation and voltage activity."""
    t = np.asarray(log.t, float)
    dt = float(t[1] - t[0])
    m = np.ones(len(t), bool) if mask is None else mask
    lim, i, raw, tgt = (np.asarray(log[k], float)[m] for k in ("i_lim", "i", "i_raw", "i_tgt"))
    demand = (_col(log, "c_tau_ff") + _col(log, "c_tau_fb"))[m] / _P.K_T
    clip = np.asarray(log.clipped, bool)[m]
    vl = np.asarray(log.vlim, float)[m] > 0
    sat = spans(clip, t[m], dt)
    lim_all = np.asarray(log.i_lim, float)
    ch = np.where(np.diff(lim_all) != 0)[0] + 1
    T = getattr(log, "meta", {}).get("T_winding")
    over = np.abs(i) - lim
    return dict(
        host_demand_peak=_peak(demand),
        host_demand_over_limit_pct=100 * float(np.mean(np.abs(demand[np.isfinite(demand)]) > lim[np.isfinite(demand)]))
        if np.isfinite(demand).any() else float("nan"),
        host_sat_pct=100 * float(np.nanmean(_col(log, "c_sat")[m])) if "c_sat" in log else float("nan"),
        drive_raw_peak=_peak(raw), target_peak=_peak(tgt), measured_peak=_peak(i), measured_rms=_rms(i),
        target_over_limit_max=float(np.max(np.abs(tgt) - lim)),
        measured_over_limit_max=float(np.max(over)), measured_over_limit_s=float(np.sum(over > 1e-9)) * dt,
        i_lim_min=float(np.min(lim)), i_lim_max=float(np.max(lim)),
        limit_changes=[(float(t[k]), float(lim_all[k])) for k in ch],
        sat_pct=100 * float(np.mean(clip)), sat_s=float(np.sum(clip)) * dt, sat_entries=len(sat),
        sat_longest_s=max((b - a for a, b in sat), default=0.0),
        vlim_pct=100 * float(np.mean(vl)), vlim_s=float(np.sum(vl)) * dt, vlim_entries=len(spans(vl, t[m], dt)),
        T_winding_end=float(T) if T is not None else float("nan"),
    )


def fault_timeline(log):
    """Event sequence, time to re-arm per latched fault, fallback/rejection spans."""
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

    def summ(mask):
        sp = spans(mask, t, dt)
        return dict(count=len(sp), total_s=float(sum(b - a for a, b in sp)),
                    longest_s=max((b - a for a, b in sp), default=0.0), spans=sp)
    return dict(events=[dict(t=a, name=b) for a, b in ev], counts=counts,
                rearms=sum(v for k, v in counts.items() if k.startswith("rearm_after_")),
                latches=latches, drive_fallback=summ(np.asarray(log.mode) > 0),
                host_fallback=summ(_host_idle(log)), rejected=summ(_col(log, "c_request_rejected") == 1))


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
    """Original vs delivered yaw amplitude/frequency and coordination requests.
    Amplitude is half peak-to-peak (deg) over t >= t_skip and over the final
    t_end_window seconds; frequency from zero crossings."""
    t = np.asarray(log.t, float)
    m, me = t >= t_skip, t >= t[-1] - t_end_window
    qy = np.asarray(log.qy, float)
    out = {}
    a, f = _amp_freq(qy[m] * DEG, t[m])
    out.update(delivered_amp=a, delivered_freq=f, delivered_amp_end=_amp_freq(qy[me] * DEG, t[me])[0])
    if yaw_request is not None:
        qo = np.array([yaw_request.eval(x)[0] for x in t])
        a0, f0 = _amp_freq(qo[m] * DEG, t[m])
        a0e = _amp_freq(qo[me] * DEG, t[me])[0]
        out.update(original_amp=a0, original_freq=f0, original_amp_end=a0e,
                   amp_ratio_end=out["delivered_amp_end"] / a0e if a0e > 1e-9 else float("nan"))
    ys = getattr(log, "meta", {}).get("yaw_scale") or []
    reqs = [(float(a), float(b)) for a, b in ys[1:]]
    inc = _col(log, "c_gov_incompatible") == 1
    out.update(scale_requests=reqs, n_scale_requests=len(reqs),
               scale_end=float(ys[-1][1]) if ys else float("nan"),
               scale_min=float(min(b for _, b in ys)) if ys else float("nan"),
               coordination_available=bool(ys),
               incompatible_pct=100 * float(np.mean(inc)),
               incompatible_first_t=float(t[np.argmax(inc)]) if inc.any() else None)
    return out


def completion(log, waypoints, t_request, tol_deg=COMPLETE_TOL_DEG, settle_s=COMPLETE_SETTLE_S,
               v_tol=COMPLETE_V_TOL, overshoot_deg=COMPLETE_OVERSHOOT_DEG):
    """Finite-motion completion (see METRIC_DEFS['completed']). A controller that
    rejects, falls back, or stays put never completes, whatever its error."""
    t, q, qd = (np.asarray(log[k], float) for k in ("t", "q", "qd"))
    dt, tol = float(t[1] - t[0]), math.radians(tol_deg)
    out = dict(completed=False, t_complete=float("nan"), t_request=float(t_request), time_ratio=0.0,
               waypoints=len(waypoints), reached=0, visit_t=[])
    lo, hi = min(waypoints + [q[0]]), max(waypoints + [q[0]])
    out["overshoot_deg"] = float(max(0.0, np.max(q) - hi, lo - np.min(q))) * DEG
    j = 0
    for w in waypoints[:-1]:
        hit = np.where(np.abs(q[j:] - w) <= tol)[0]
        if not hit.size:
            return out
        j += int(hit[0])
        out["visit_t"].append(float(t[j]))
        out["reached"] += 1
    healthy = (np.asarray(log.mode) == 0) & ~_host_idle(log) & ~(_col(log, "c_request_rejected") == 1)
    band = (np.abs(q - waypoints[-1]) <= tol) & (np.abs(qd) <= v_tol) & healthy
    n = int(round(settle_s / dt))
    run = np.convolve(band[j:].astype(int), np.ones(n, int), "valid") if len(band) - j >= n else np.array([])
    ok = np.where(run == n)[0]
    if not ok.size:
        return out
    out["reached"] += 1
    tc = float(t[j + int(ok[0])])
    out.update(t_complete=tc, time_ratio=t_request / tc if tc > 0 else float("nan"),
               completed=out["overshoot_deg"] <= overshoot_deg)
    return out


def evaluate(log, ref=None, yaw_request=None, T_request=None, waypoints=None, t_skip=0.0):
    """All Packet 4A metrics for one run as a JSON-ready dict (NaN kept as float).

    ref: original roll request; T_request: requested path duration (finite
    motion end, or the run duration for periodic requests); waypoints: ordered
    targets of a finite motion (rad) to score completion.
    """
    t = np.asarray(log.t, float)
    mask = t >= t_skip
    T_request = float(T_request if T_request is not None else t[-1] + (t[1] - t[0]))
    errs = path_errors(log, ref)
    e = {f"{k}_{s}": f(errs[k][mask] * DEG) for k in errs for s, f in (("rms", _rms), ("peak", _peak))}
    prog = progress(log, T_request)
    st = stratified(log, errs, mask=mask)
    out = dict(metrics_version=METRICS_VERSION, t_skip=float(t_skip), errors=e, progress=prog, strata=st,
               current=current_report(log, mask), faults=fault_timeline(log),
               yaw=yaw_report(log, yaw_request), summary=summarize(log, t_skip))
    reasons = []
    if not prog["progress"] >= TRACK_PROGRESS_MIN:
        reasons.append(f"progress {prog['progress']:.2f} < {TRACK_PROGRESS_MIN}")
    if not e["path_rms"] <= TRACK_PATH_RMS_DEG:
        reasons.append(f"path RMS {e['path_rms']:.2f} deg > {TRACK_PATH_RMS_DEG}" if ref is not None
                       else "no original request supplied: path error unknown")
    if st["rejected"]["share_pct"] > 0:
        reasons.append(f"rejected {st['rejected']['share_pct']:.1f}% of time")
    if waypoints is not None:
        out["completion"] = completion(log, list(waypoints), T_request)
        if not out["completion"]["completed"]:
            reasons.append("finite motion not completed")
    out["tracked"] = not reasons
    out["not_tracked_because"] = reasons
    return out
