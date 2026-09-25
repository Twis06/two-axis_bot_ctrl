"""Task 5, prospective: a numerical closed-loop prediction registered before the test.

The brief asks for a numerical prediction written *before* a new test. The historical
motor-strength study (exp/task5_prediction.py) has no ordering evidence, so this packet
adds one genuinely new experiment whose prediction is computed analytically, written to
report/task5_prospective_registration.json and committed before any run of the condition.

Experiment (frozen): 5 deg roll sine at 3 Hz (1 s ramp, 12 s), yaw held at 0, nominal
plant with d(t) = 0, frozen BaselineController() and DriveSupervisor(). Current-command
delay 1 ms (nominal) against 5 ms, seeds 301-305 matched within each pair.

Primary quantity: the paired change of the fundamental tracking transfer
    dH = H(5 ms) - H(1 ms),  H = actual roll / governed roll reference,
each H from a sin/cos/intercept least-squares fit over [4, 12) s (24 cycles); the
primary aggregate is the arithmetic mean of the five paired complex changes.

Prediction (Calculated; no new-run data): linearised plant at q = 0 with the Coulomb
friction replaced by its describing function at the predicted velocity amplitude; the
controller's discrete PI x lead (Tustin, 500 Hz) and feed-forward (inertia, viscous,
gravity, 80 % friction on the governed reference); the 1.2 ms current lag; and command /
feedback timing derived from the configured scheduling and transport (not measured).

Usage:
    python -m exp.task5_prospective --register   # once, before any run; never overwrites
    python -m exp.task5_prospective              # run, score and publish (refuses if the
                                                 # registration differs from this code)
"""
import argparse
import cmath
import hashlib
import itertools
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

from ctrl.baseline import BaselineController
from exp import manifest as MF
from exp import scenarios as S
from exp.evidence import RunBook, baseline_fingerprint
from sim import params as P
from sim.config import SimConfig
from sim.trajectories import Hold, RampedSine

ROOT = Path(__file__).resolve().parents[1]
REG_PATH = ROOT / "report" / "task5_prospective_registration.json"
ENTRY = "exp.task5_prospective"

PROTOCOL = dict(
    freq_hz=3.0, amp_deg=5.0, ramp_s=1.0, duration_s=12.0, window_s=[4.0, 12.0],
    cmd_delay_s=[1.0e-3, 5.0e-3], seeds=[301, 302, 303, 304, 305], d_amp=0.0,
    controller="BaselineController() defaults (estimate mode, governor and yaw monitor on)",
    supervisor="DriveSupervisor() defaults", yaw="Hold(0)",
    primary="mean over the five seeds of H(5 ms) - H(1 ms); H = fundamental of actual roll q "
            "divided by fundamental of governed reference c_q_c, sin/cos/intercept fit over the window",
)
MEASUREMENT_FLOOR = 0.01      # |dH| allowance for effects the linear model omits entirely (declared)


# ---------------------------------------------------------------------------
# Calculated prediction
# ---------------------------------------------------------------------------
def timing(cmd_delay_s, cfg=None, base_shift=0.0, fb_shift=0.0, burst=True):
    """Command-path delay T_ff (host sample -> mean torque application, before the current
    lag) and feedback age T_fb, from the configured schedule, not from any log.
    T_ff = compute + mean CAN + mean wait for the drive tick + FIFO + half the host hold."""
    tc = (cfg or SimConfig()).timing
    can = 0.5 * (tc.can_min + tc.can_max)
    if burst:        # expected share of time in burst episodes at can_burst
        share = min(1.0, tc.burst_rate_hz * 0.5 * (tc.burst_len_s[0] + tc.burst_len_s[1]))
        can += share * (tc.can_burst - can)
    fifo = round(cmd_delay_s * tc.f_drive) / tc.f_drive
    t_ff = tc.t_compute + can + 0.5 / tc.f_drive + fifo + 0.5 / tc.f_ctrl + base_shift
    t_fb = can + 0.5 / tc.f_drive + fb_shift
    return t_ff, t_fb


def controller_response(w, ctl, discrete=True):
    """Feedback C and the controller's linear feed-forward pieces at angular frequency w."""
    if discrete:
        b0, b1, a1 = ctl.lead
        z1 = cmath.exp(-1j * w * ctl.ts)                    # z^-1
        lead = (b0 + b1 * z1) / (1 + a1 * z1)
        integ = ctl.ts * ctl.wi * z1 / (1 - z1)             # integ updated after use
        return ctl.K * lead * (1 + integ)
    s = 1j * w
    wz, wp = ctl.wc / math.sqrt(ctl.alpha), ctl.wc * math.sqrt(ctl.alpha)
    return ctl.K * (1 + ctl.wi / s) * (1 + s / wz) / (1 + s / wp)


_CTL = None


def _controller():
    """The frozen BaselineController() with reset(SimConfig()) applied (discrete lead
    coefficients and sample time, exactly as at run start). Built once: its design is costly."""
    global _CTL
    if _CTL is None:
        _CTL = BaselineController()
        _CTL.reset(SimConfig())
    return _CTL


def predict_H(cmd_delay_s, fric_scale=1.0, discrete=True, base_shift=0.0, fb_shift=0.0, burst=True):
    """Closed-loop H = q / q_c at the registered frequency (Calculated).
    q = P * G * e^{-s T_ff} * [F q_c + C (q_c - e^{-s T_fb} q)], so
    H = P G e^{-sT_ff} (F + C) / (1 + P G C e^{-s (T_ff + T_fb)})."""
    ctl = _controller()
    f, amp = PROTOCOL["freq_hz"], math.radians(PROTOCOL["amp_deg"])
    w = 2 * math.pi * f
    s = 1j * w
    t_ff, t_fb = timing(cmd_delay_s, base_shift=base_shift, fb_shift=fb_shift, burst=burst)
    G = cmath.exp(-s * t_ff) / (P.TAU_I * s + 1)
    C = controller_response(w, ctl, discrete)
    # Describing functions (fundamental, in phase with velocity) of sign(v) at amplitude V:
    df = lambda V: 4.0 / (math.pi * V)
    v_ref = amp * w
    F = ctl.J * s ** 2 + ctl.b * s + ctl.tau_g + fric_scale * 0.8 * ctl.tau_c * df(v_ref) * s
    H = 1.0 + 0j
    for _ in range(50):        # plant friction depends on the actual velocity amplitude
        Pl = 1.0 / (P.J_R * s ** 2 + (P.B_VISC + fric_scale * P.TAU_C * df(abs(H) * v_ref)) * s + P.TAU_G)
        H_new = Pl * G * (F + C) / (1 + Pl * G * C * cmath.exp(-s * t_fb))
        if abs(H_new - H) < 1e-14:
            break
        H = H_new
    return H_new, dict(t_ff_s=t_ff, t_fb_s=t_fb)


def prediction():
    """Nominal prediction and its declared acceptance region (all analytical)."""
    d0, d1 = PROTOCOL["cmd_delay_s"]
    H0, tm0 = predict_H(d0)
    H1, tm1 = predict_H(d1)
    dH = H1 - H0
    # Approximation bound: vary every modelling assumption over a declared range.
    grid = dict(fric_scale=(0.5, 1.0, 1.5), discrete=(True, False),
                base_shift=(-0.5e-3, 0.0, 0.5e-3), fb_shift=(-0.5e-3, 0.0, 0.5e-3), burst=(True, False))
    devs = []
    for vals in itertools.product(*grid.values()):
        kw = dict(zip(grid, vals))
        devs.append(abs((predict_H(d1, **kw)[0] - predict_H(d0, **kw)[0]) - dH))
    radius = max(devs) + MEASUREMENT_FLOOR
    c = lambda z: dict(re=z.real, im=z.imag, mag=abs(z), phase_deg=math.degrees(cmath.phase(z)))
    return dict(
        H_nominal=c(H0), H_delayed=c(H1), dH=c(dH),
        gain_change_db=20 * math.log10(abs(H1) / abs(H0)),
        phase_change_deg=math.degrees(cmath.phase(H1 / H0)),
        timing_nominal=tm0, timing_delayed=tm1,
        acceptance=dict(kind="disc in the complex plane", center=c(dH), radius=radius,
                        model_bound=max(devs), measurement_floor=MEASUREMENT_FLOOR,
                        grid={k: list(v) for k, v in grid.items()},
                        includes_zero=abs(dH) <= radius),
        rationale=("The extra 4 ms delays both the feed-forward and the feedback torque. The feed-forward alone "
                   "would add about 4.3 deg of phase lag at 3 Hz, but the loop is closed with a 4.46 Hz "
                   "crossover: the added delay removes about 6 deg of phase margin, which raises the closed-loop "
                   "peaking near 3 Hz. The model therefore predicts mainly a gain increase (about +0.5 dB) with "
                   "only about -1 deg of extra phase, i.e. dH points mostly along +Re. The region covers +-0.5 ms "
                   "in the command-path and feedback-age accounting, CAN bursts on/off, a continuous versus "
                   "discrete controller, and friction describing-function gain x0.5-1.5, plus a fixed "
                   f"{MEASUREMENT_FLOOR} floor for effects the linear model omits (friction harmonics, "
                   "quantization, integrator transients)."),
    )


def registration():
    return dict(schema=1, protocol=PROTOCOL, prediction=prediction(),
                baseline_fingerprint=baseline_fingerprint()[0],
                statement="Written and committed before any simulation of this condition. Outcome labels: "
                          "supported (measured mean dH inside the disc), contradicted (outside), "
                          "inconclusive (any pair invalid, faulted, suspended or rejected).")


def _canon(d):
    return json.dumps(d, sort_keys=True, separators=(",", ":"))


def check_registration():
    """The committed registration must equal what this code computes (protocol and numbers)."""
    if not REG_PATH.exists():
        raise SystemExit(f"{REG_PATH} missing: register before running")
    frozen = json.loads(REG_PATH.read_text())
    now = registration()
    if frozen["protocol"] != now["protocol"] or frozen["baseline_fingerprint"] != now["baseline_fingerprint"]:
        raise SystemExit("registration protocol or baseline differs from the code: refusing to run")
    a, b = frozen["prediction"], now["prediction"]
    for k in ("dH", "H_nominal", "H_delayed"):
        for part in ("re", "im"):
            if abs(a[k][part] - b[k][part]) > 1e-12:
                raise SystemExit(f"registered prediction {k}.{part} differs from the code: refusing to run")
    if abs(a["acceptance"]["radius"] - b["acceptance"]["radius"]) > 1e-12:
        raise SystemExit("registered acceptance region differs from the code: refusing to run")
    return frozen, hashlib.sha256(REG_PATH.read_bytes()).hexdigest()


# ---------------------------------------------------------------------------
# Scoring
# ---------------------------------------------------------------------------
def phasor(t, x, f, window):
    """Fundamental of x over [t0, t1) as a + j b from the fit x = a sin(wt) + b cos(wt) + c.
    For x = A sin(wt + phi) this is A e^{j phi}, the same convention as a transfer function
    evaluated at s = j w. None if the window is incomplete or non-finite."""
    t, x = np.asarray(t, float), np.asarray(x, float)
    m = (t >= window[0]) & (t < window[1])
    if m.sum() < 10 or not np.all(np.isfinite(x[m])):
        return None
    w = 2 * math.pi * f
    X = np.column_stack([np.sin(w * t[m]), np.cos(w * t[m]), np.ones(int(m.sum()))])
    c = np.linalg.lstsq(X, x[m], rcond=None)[0]
    return complex(c[0], c[1])


def transfer(t, q, r, f, window, min_ref=1e-3):
    """H = fundamental(q) / fundamental(r); None (invalid, never zero) if r is negligible."""
    pq, pr = phasor(t, q, f, window), phasor(t, r, f, window)
    if pq is None or pr is None or abs(pr) < min_ref:
        return None
    return pq / pr


def paired_grid(rows):
    """Exactly one run per (seed, delay) for every registered seed; else incomplete."""
    want = {(s, d) for s in PROTOCOL["seeds"] for d in PROTOCOL["cmd_delay_s"]}
    have = [(r["seed"], r["cmd_delay_s"]) for r in rows]
    missing = sorted(want - set(have))
    dup = sorted({k for k in have if have.count(k) > 1})
    extra = sorted(set(have) - want)
    return dict(complete=not (missing or dup or extra), missing=missing, duplicate=dup, unregistered=extra)


def scenario(cmd_delay_s):
    p = PROTOCOL
    cfg = replace(SimConfig(), duration=p["duration_s"]).with_(
        plant=dict(d_amp=p["d_amp"]), timing=dict(t_cmd_delay=cmd_delay_s))
    return S.Scenario(f"T5P-{cmd_delay_s * 1e3:.0f}ms", cfg,
                      RampedSine(math.radians(p["amp_deg"]), p["freq_hz"], t_ramp=p["ramp_s"]), Hold(0.0),
                      f"roll sine {p['amp_deg']} deg at {p['freq_hz']} Hz, yaw still, d(t) off, "
                      f"current-command delay {cmd_delay_s * 1e3:.0f} ms")


def score_run(log, seed, cmd_delay_s, rid):
    p = PROTOCOL
    H = transfer(log.t, log.q, log.c_q_c, p["freq_hz"], p["window_s"])
    Hr = transfer(log.t, log.q, log.q_ref, p["freq_hz"], p["window_s"])
    m = (log.t >= p["window_s"][0]) & (log.t < p["window_s"][1])
    events = [e[1] for e in log.events]
    return dict(
        seed=seed, cmd_delay_s=cmd_delay_s, run_id=rid, valid=H is not None,
        H=None if H is None else dict(re=H.real, im=H.imag, gain=abs(H), phase_deg=math.degrees(cmath.phase(H))),
        H_vs_request=None if Hr is None else dict(re=Hr.real, im=Hr.imag, gain=abs(Hr),
                                                    phase_deg=math.degrees(cmath.phase(Hr))),
        governed_rms_deg=float(math.degrees(np.sqrt(np.mean((log.q[m] - log.c_q_c[m]) ** 2)))),
        request_rms_deg=float(math.degrees(np.sqrt(np.mean((log.q[m] - log.q_ref[m]) ** 2)))),
        gov_limited_pct=100 * float(np.nanmean(log.c_gov_limited[m])),
        clip_pct=100 * float(np.mean(log.clipped[m])),
        i_peak_A=float(np.max(np.abs(log.i))),
        suspended=bool(np.nanmax(log.c_suspended) > 0.5),
        rejected=bool(np.nanmax(log.c_request_rejected) > 0.5),
        fallback_pct=100 * float(np.mean(log.mode > 0)),
        events=events,
    )


def outcome(rows, reg):
    grid = paired_grid(rows)
    by = {(r["seed"], r["cmd_delay_s"]): r for r in rows}
    d0, d1 = PROTOCOL["cmd_delay_s"]
    pairs, problems = [], []
    for s in PROTOCOL["seeds"]:
        a, b = by.get((s, d0)), by.get((s, d1))
        if a is None or b is None:
            continue
        for r in (a, b):
            if not r["valid"]:
                problems.append(f"seed {s}, {r['cmd_delay_s'] * 1e3:.0f} ms: invalid transfer")
            if r["events"] or r["suspended"] or r["rejected"] or r["fallback_pct"] > 0:
                problems.append(f"seed {s}, {r['cmd_delay_s'] * 1e3:.0f} ms: fault/suspension/rejection/fallback")
        if a["valid"] and b["valid"]:
            pairs.append(dict(seed=s, dH=dict(re=b["H"]["re"] - a["H"]["re"], im=b["H"]["im"] - a["H"]["im"])))
    acc = reg["prediction"]["acceptance"]
    center = complex(acc["center"]["re"], acc["center"]["im"])
    res = dict(grid=grid, pairs=pairs, problems=problems)
    if not grid["complete"] or problems or len(pairs) != len(PROTOCOL["seeds"]):
        res.update(label="inconclusive", mean_dH=None, distance=None)
        return res
    mean = complex(np.mean([p["dH"]["re"] for p in pairs]), np.mean([p["dH"]["im"] for p in pairs]))
    dist = abs(mean - center)
    res.update(mean_dH=dict(re=mean.real, im=mean.imag, mag=abs(mean), phase_deg=math.degrees(cmath.phase(mean))),
               distance=dist, radius=acc["radius"],
               label="supported" if dist <= acc["radius"] else "contradicted")
    return res


# ---------------------------------------------------------------------------
# Execution and publication
# ---------------------------------------------------------------------------
def _figure(result, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    reg, out = result["registration"]["prediction"], result["outcome"]
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.2))
    acc = reg["acceptance"]
    ax[0].add_patch(plt.Circle((acc["center"]["re"], acc["center"]["im"]), acc["radius"], color="#4c72b0",
                               alpha=0.18, label="registered acceptance region"))
    ax[0].plot(acc["center"]["re"], acc["center"]["im"], "o", color="#4c72b0", label="predicted ΔH")
    for p in out["pairs"]:
        ax[0].plot(p["dH"]["re"], p["dH"]["im"], "x", color="#555555")
    if out["mean_dH"]:
        ax[0].plot(out["mean_dH"]["re"], out["mean_dH"]["im"], "D", color="#c44e52", label="measured mean ΔH")
    ax[0].plot(0, 0, "+", color="k", ms=12, label="no change")
    ax[0].set_aspect("equal", "datalim")
    ax[0].set_xlabel("Re ΔH")
    ax[0].set_ylabel("Im ΔH")
    ax[0].set_title("Paired change in tracking transfer at 3 Hz")
    ax[0].legend(fontsize=8, loc="best")
    ax[0].grid(alpha=0.3)
    rows = result["rows"]
    for d, col in zip(PROTOCOL["cmd_delay_s"], ("#4c72b0", "#c44e52")):
        g = [r["H"]["gain"] for r in rows if r["cmd_delay_s"] == d and r["H"]]
        ph = [r["H"]["phase_deg"] for r in rows if r["cmd_delay_s"] == d and r["H"]]
        ax[1].plot(ph, g, "o", color=col, label=f"measured, {d * 1e3:.0f} ms")
    for key, col in (("H_nominal", "#4c72b0"), ("H_delayed", "#c44e52")):
        ax[1].plot(reg[key]["phase_deg"], reg[key]["mag"], "*", ms=14, mfc="none", color=col,
                   label=f"predicted, {'1' if key == 'H_nominal' else '5'} ms")
    ax[1].set_xlabel("phase of H (deg)")
    ax[1].set_ylabel("|H|")
    ax[1].set_title("Tracking gain and phase per run")
    ax[1].legend(fontsize=8)
    ax[1].grid(alpha=0.3)
    fig.suptitle(f"Prospective Task 5 test: prediction registered before the runs — outcome: "
                 f"{out['label'].upper()}", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=150, metadata={"Software": None})
    plt.close(fig)


def _numbers(result, reg_sha):
    reg, out = result["registration"]["prediction"], result["outcome"]
    f = lambda z: f"{z['re']:+.4f} {z['im']:+.4f}j (|·| {abs(complex(z['re'], z['im'])):.4f})"
    lines = ["# Task 5 — prospective closed-loop prediction (generated)", "",
             "Generated by `python -m exp.task5_prospective`. **Prediction: Calculated, registered before any run "
             f"of this condition** (`report/task5_prospective_registration.json`, sha256 `{reg_sha[:16]}`). "
             "**Result: Simulated.**", "",
             f"Condition: {PROTOCOL['amp_deg']}° roll sine at {PROTOCOL['freq_hz']} Hz, yaw still, d(t) off, frozen "
             f"baseline `{result['registration']['baseline_fingerprint'][:16]}`; current-command delay 1 ms vs 5 ms; "
             f"seeds {PROTOCOL['seeds'][0]}–{PROTOCOL['seeds'][-1]}; H = actual roll / governed reference, "
             f"fundamental over [{PROTOCOL['window_s'][0]:g}, {PROTOCOL['window_s'][1]:g}) s.", "",
             "## Registered prediction", "",
             f"- H(1 ms) = {f(reg['H_nominal'])}; H(5 ms) = {f(reg['H_delayed'])}",
             f"- **Predicted ΔH = {f(reg['dH'])}**: gain change {reg['gain_change_db']:+.2f} dB, phase change "
             f"{reg['phase_change_deg']:+.2f}°",
             f"- Acceptance: a disc of radius {reg['acceptance']['radius']:.4f} around the prediction "
             f"(model bound {reg['acceptance']['model_bound']:.4f} + floor {reg['acceptance']['measurement_floor']}); "
             f"it {'includes' if reg['acceptance']['includes_zero'] else 'excludes'} ΔH = 0.",
             f"- Timing used (from configuration): T_ff {reg['timing_nominal']['t_ff_s'] * 1e3:.2f} → "
             f"{reg['timing_delayed']['t_ff_s'] * 1e3:.2f} ms, feedback age {reg['timing_nominal']['t_fb_s'] * 1e3:.2f} ms.",
             "", "## Result", "",
             "| Seed | Delay | gain | phase ° | governed RMS ° | request RMS ° | governor-limited % | clipped % | "
             "peak A | events | run_id |", "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(result["rows"], key=lambda r: (r["seed"], r["cmd_delay_s"])):
        h = r["H"]
        lines.append(f"| {r['seed']} | {r['cmd_delay_s'] * 1e3:.0f} ms | "
                     + (f"{h['gain']:.4f} | {h['phase_deg']:+.2f}" if h else "invalid | invalid")
                     + f" | {r['governed_rms_deg']:.3f} | {r['request_rms_deg']:.3f} | {r['gov_limited_pct']:.1f} | "
                     f"{r['clip_pct']:.1f} | {r['i_peak_A']:.2f} | {', '.join(r['events']) or 'none'} | `{r['run_id']}` |")
    lines += ["", "| Seed | measured ΔH |", "|---|---|"]
    lines += [f"| {p['seed']} | {f(p['dH'])} |" for p in out["pairs"]]
    lines += [""]
    if out["mean_dH"]:
        lines.append(f"**Measured mean ΔH = {f(out['mean_dH'])}; distance from the prediction {out['distance']:.4f} "
                     f"against the registered radius {out['radius']:.4f}.**")
    lines += ["", f"**Outcome: {out['label']}.** "
              + ("; ".join(out["problems"]) if out["problems"] else "All ten runs valid; no fault, suspension, "
                 "rejection or fallback in any run."), "",
              "Every registered run is listed; nothing was added, removed or re-scored after the runs."]
    return "\n".join(lines) + "\n"


def run(out_dir):
    reg, reg_sha = check_registration()
    book = RunBook(ENTRY)
    rows = []
    for seed in PROTOCOL["seeds"]:
        for d in PROTOCOL["cmd_delay_s"]:
            log, _stats, rid, _ev = book.run(scenario(d), BaselineController, seed=seed)
            rows.append(score_run(log, seed, d, rid))
            print(f"run seed {seed}, delay {d * 1e3:.0f} ms done", flush=True)
    result = dict(registration=reg, registration_sha256=reg_sha, rows=rows, outcome=outcome(rows, reg))

    def check(stage):
        book.check_unchanged()
    import tempfile
    out = Path(out_dir)
    with MF.staged_publish(out, staging_root=Path(tempfile.gettempdir()) / "task5p_stage", check=check) as st:
        MF.write_json(result, st / "task5_prospective_results.json")
        MF.write_json(book.record(), st / "task5_prospective_runs.json")
        (st / "task5_prospective_numbers.md").write_text(_numbers(result, reg_sha))
        (st / "figs").mkdir(exist_ok=True)
        _figure(result, st / "figs" / "task5_prospective.png")
    print(f"outcome: {result['outcome']['label']}; published to {out}")
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--register", action="store_true")
    ap.add_argument("--out", default=str(ROOT / "report"))
    a = ap.parse_args(argv)
    if a.register:
        if REG_PATH.exists():
            raise SystemExit(f"{REG_PATH} exists: a registration is never regenerated")
        MF.write_json(registration(), REG_PATH)
        print(f"registered {REG_PATH}")
        return 0
    run(a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
