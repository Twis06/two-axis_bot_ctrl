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
