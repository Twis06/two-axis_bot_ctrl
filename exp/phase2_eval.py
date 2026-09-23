"""Phase 2 evidence: baseline vs legacy on Runs A-E, Monte Carlo robustness,
fault/timing injection, and analytic loop margins.

    python exp/phase2_eval.py [--quick]
"""
import math
import os
import sys
from dataclasses import replace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from ctrl import loopshape as LS  # noqa: E402
from ctrl.baseline import BaselineController  # noqa: E402
from ctrl.governor import admit_yaw_sine, yaw_envelope  # noqa: E402
from ctrl.legacy import LegacyPID  # noqa: E402
from exp import scenarios as S  # noqa: E402
from exp.common import run  # noqa: E402
from sim import params as P  # noqa: E402
from sim.config import SimConfig  # noqa: E402
from sim.trajectories import Hold, RampedSine  # noqa: E402

FIGS = os.path.join(ROOT, "report", "figs")
DEG = 180 / math.pi
QUICK = "--quick" in sys.argv
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def legacy():
    return LegacyPID(**S.LEGACY)


def run_pair(sc, seed, cfg=None):
    lg_l, s_l = run(sc, legacy, seed=seed, supervisor="legacy", governed_yaw=False, cfg=cfg)
    lg_b, s_b = run(sc, BaselineController, seed=seed, cfg=cfg)
    return (lg_l, s_l), (lg_b, s_b)


def table(headers, rows):
    out = ["| " + " | ".join(headers) + " |", "|" + "---|" * len(headers)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def style(ax):
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ax.grid(True, color=GRID, lw=0.6)
    ax.tick_params(labelsize=8, colors=INK2)


# ---------------------------------------------------------------------------
def section_runs(out):
    seeds = (1, 2) if QUICK else (1, 2, 3, 4, 5)
    rows, keep = [], {}
    for sc in S.all_runs():
        res = [run_pair(sc, sd) for sd in seeds]
        keep[sc.name] = res[0]
        L = [r[0][1] for r in res]
        B = [r[1][1] for r in res]

        def med(xs, k):
            return float(np.median([x[k] for x in xs]))
        rows.append([sc.name, sc.note,
                     f"{med(L, 'rms'):.2f} / {med(L, 'peak'):.1f}",
                     f"{med(B, 'rms_gov'):.2f} / {med(B, 'peak_gov'):.1f}",
                     f"{med(B, 'speed'):.0%}",
                     f"{med(L, 'clip_pct'):.0f} / {med(B, 'clip_pct'):.0f}",
                     f"{med(L, 'sat_entries'):.0f} / {med(B, 'sat_entries'):.0f}",
                     f"{med(L, 'i_rms'):.2f} / {med(B, 'i_rms'):.2f}",
                     f"{med(L, 'wd_trips'):.0f} / {med(B, 'events'):.0f}"])
    out += ["## 1. Runs A–E: legacy vs baseline (median of %d seeds)" % len(seeds), "",
            "Baseline error is measured against the *governed* reference it chose to follow; "
            "`speed` is the fraction of requested path speed delivered (1 = unmodified).", "",
            table(["Run", "request", "legacy RMS/peak °", "baseline RMS/peak °", "speed",
                   "% limited L/B", "sat. entries L/B", "i RMS A L/B", "wd trips L / B events"],
                  rows), ""]
    return keep


def fig_runs(keep):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    names = ["B", "C", "E"]
    fig, axes = plt.subplots(len(names), 2, figsize=(10, 7.5))
    for r, n in enumerate(names):
        (lg_l, s_l), (lg_b, s_b) = keep[n]
        ax = axes[r, 0]
        ax.plot(lg_l.t, lg_l.err * DEG, color=C2, lw=1, label="legacy (vs request)")
        eb = (lg_b.q - lg_b.c_q_c) * DEG
        ax.plot(lg_b.t, eb, color=C1, lw=1, label="baseline (vs governed ref)")
        ax.set_ylabel(f"Run {n}\nerror °", fontsize=9, color=INK2)
        style(ax)
        ax = axes[r, 1]
        ax.plot(lg_l.t, lg_l.i, color=C2, lw=0.8, label="legacy")
        ax.plot(lg_b.t, lg_b.i, color=C1, lw=0.8, label="baseline")
        ax.plot(lg_b.t, lg_b.i_lim, color=INK2, ls="--", lw=1, label="limit")
        ax.plot(lg_b.t, -lg_b.i_lim, color=INK2, ls="--", lw=1)
        ax.set_ylim(-3.5, 3.5)
        ax.set_ylabel("current A", fontsize=9, color=INK2)
        style(ax)
        if r == 0:
            axes[r, 0].legend(fontsize=7, frameon=False, loc="upper right")
            ax.legend(fontsize=7, frameon=False, loc="lower right", ncol=3)
    axes[-1, 0].set_xlabel("time s", color=INK2)
    axes[-1, 1].set_xlabel("time s", color=INK2)
    fig.suptitle("Legacy vs baseline: yaw disturbance (B, C) and derated payload sweep (E)",
                 x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p2_runs_compare.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
def sample_cfg(rng, base):
    """Uncertainty ranges: brief-given where stated, Phase 1 widened where not."""
    cs = rng.uniform(0.5, 1.5)
    return base.with_(
        plant=dict(J=P.J_R * rng.uniform(0.7, 1.3), tau_c=P.TAU_C * rng.uniform(0.5, 2.0),
                   b=P.B_VISC * rng.uniform(0.5, 1.5), tau_g=P.TAU_G * rng.uniform(0.7, 1.3),
                   k_t=P.K_T * rng.uniform(0.85, 1.15), k_yv=P.K_YV * cs, k_ya=P.K_YA * cs,
                   R=P.R_NOM * rng.uniform(0.75, 1.25), L=P.L_NOM * rng.uniform(0.8, 1.2),
                   v_bus=rng.uniform(20.0, 24.0), m_payload=rng.uniform(0.0, 0.3)),
        timing=dict(burst_rate_hz=rng.uniform(0.0, 2.0)),
        sensor=dict(enc_offset=rng.uniform(-0.01, 0.01))), cs


def section_montecarlo(out):
    n = 8 if QUICK else 40
    rng = np.random.default_rng(2024)
    scen = [S.run_a, S.run_b, S.run_c]
    rows, data = [], {sc(SimConfig()).name: {"L": [], "B": [], "cs": []} for sc in scen}
    for k in range(n):
        base, cs = sample_cfg(rng, SimConfig())
        for mk in scen:
            sc = mk(base)
            sc = replace(sc, cfg=replace(sc.cfg, duration=min(sc.cfg.duration, 6.0)))
            (_, s_l), (_, s_b) = run_pair(sc, seed=100 + k)
            d = data[sc.name]
            d["L"].append(s_l)
            d["B"].append(s_b)
            d["cs"].append(cs)
    for name, d in data.items():
        def pct(xs, key, q):
            return float(np.percentile([x[key] for x in xs], q))
        rows.append([name,
                     f"{pct(d['L'], 'rms', 50):.2f} / {pct(d['L'], 'rms', 95):.2f}",
                     f"{pct(d['B'], 'rms_gov', 50):.2f} / {pct(d['B'], 'rms_gov', 95):.2f}",
                     f"{pct(d['B'], 'peak_gov', 95):.1f}",
                     f"{pct(d['L'], 'clip_pct', 95):.0f} / {pct(d['B'], 'clip_pct', 95):.0f}",
                     f"{pct(d['B'], 'speed', 5):.0%}",
                     f"{sum(x['events'] > 0 for x in d['B'])}/{len(d['B'])}"])
    out += [f"## 2. Monte Carlo robustness ({n} plants per run)", "",
            "Sampled: J ±30 %, τc ×0.5–2, b ±50 %, τg ±30 %, Kt ±15 %, yaw coupling ×0.5–1.5, "
            "R ±25 %, L ±20 %, V_bus 20–24 V, payload 0–0.3 kg, CAN burst rate 0–2 /s, "
            "encoder offset ±0.6°. Controllers use nominal parameters only.", "",
            table(["Run", "legacy RMS ° p50/p95", "baseline RMS ° p50/p95", "baseline peak ° p95",
                   "% limited p95 L/B", "baseline speed p5", "baseline runs with fault events"], rows),
            ""]
    return data


def fig_montecarlo(data):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(10, 3.2), sharey=False)
    for ax, (name, d) in zip(axes, data.items()):
        cs = np.array(d["cs"])
        ax.scatter(cs, [x["rms"] for x in d["L"]], s=14, color=C2, label="legacy")
        ax.scatter(cs, [x["rms_gov"] for x in d["B"]], s=14, color=C1, label="baseline")
        ax.set_yscale("log")
        ax.set_title(f"Run {name}", fontsize=10, loc="left")
        ax.set_xlabel("true / modelled yaw coupling", fontsize=8, color=INK2)
        style(ax)
    axes[0].set_ylabel("RMS error ° (log)", fontsize=9, color=INK2)
    axes[0].legend(fontsize=7, frameon=False)
    fig.suptitle("Generalisation over sampled plants: error vs coupling mismatch", x=0.01,
                 ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p2_montecarlo.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
def section_faults(out):
    base = SimConfig()
    cases = []
    c = S.run_c(base)
    cases.append(("CAN blackout 60 ms @3 s (yaw 2.2 Hz)",
                  replace(c, cfg=c.cfg.with_(timing=dict(blackout=((3.0, 3.06),))))))
    cases.append(("burst storm: 5 /s, 50–200 ms (yaw 2.2 Hz)",
                  replace(c, cfg=c.cfg.with_(timing=dict(burst_rate_hz=5.0, burst_len_s=(0.05, 0.2))))))
    cases.append(("5 % message loss (yaw 2.2 Hz)",
                  replace(c, cfg=c.cfg.with_(timing=dict(drop_prob=0.05)))))
    cases.append(("derate to 2.4 A @3 s, coupling ×1.5 (yaw 2.2 Hz)",
                  replace(c, cfg=c.cfg.with_(drive=dict(derate_schedule=((3.0, 2.4),)),
                                             plant=dict(k_yv=P.K_YV * 1.5, k_ya=P.K_YA * 1.5)))))
    v_cfg = replace(base, duration=8.0).with_(plant=dict(v_bus=20.0, R=P.R_NOM * 1.25))
    cases.append(("bus 20 V, R +25 %, fast ±80° 1.5 Hz sweep",
                  S.Scenario("V", v_cfg, RampedSine(math.radians(80), 1.5, t_ramp=1.0), Hold(0.0),
                             "voltage")))
    rows, keep = [], {}
    for name, sc in cases:
        (lg_l, s_l), (lg_b, s_b) = run_pair(sc, seed=7)
        keep[name] = (lg_l, lg_b)
        ev = sorted({e[1] for e in lg_b.events})
        rows.append([name, f"{s_l['rms']:.2f} / {s_l['peak']:.1f}",
                     f"{s_b['rms_gov']:.2f} / {s_b['peak_gov']:.1f}", f"{s_b['speed']:.0%}",
                     f"{s_l['clip_pct']:.0f} / {s_b['clip_pct']:.0f}", f"{s_b['vlim_pct']:.1f}",
                     f"{s_b['fallback_pct']:.1f}", f"{s_b['yaw_scale']:.2f}",
                     ", ".join(ev) if ev else "—"])
    out += ["## 3. Fault, timing and electrical cases (seed 7)", "",
            table(["Case", "legacy RMS/peak °", "baseline RMS/peak °", "speed", "% limited L/B",
                   "% voltage-limited B", "% fallback B", "yaw scale B", "baseline events"], rows),
            ""]
    # Offline admission (the planner's decision before a request is executed)
    rows = []
    for f in (1.5, 2.2, 3.0):
        for lim in (3.2, 2.4):
            dec, A = admit_yaw_sine(math.radians(75), f, lim)
            rows.append([f"±75° @ {f} Hz", f"{lim} A", dec, f"±{A * DEG:.0f}°"])
    out += ["### Yaw admission (planner, before execution; nominal model, 20 % reserve)", "",
            table(["request", "limit", "decision", "allowed amplitude"], rows), ""]
    return keep


def fig_fault(keep):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    name = [k for k in keep if k.startswith("derate")][0]
    lg_l, lg_b = keep[name]
    fig, axes = plt.subplots(4, 1, figsize=(9, 8), sharex=True)
    ax = axes[0]
    ax.plot(lg_b.t, lg_b.qy * DEG, color=C3, lw=1, label="yaw (baseline asks it to shrink)")
    ax.plot(lg_l.t, lg_l.qy * DEG, color=INK2, lw=0.6, alpha=0.6, label="yaw as requested (legacy)")
    ax.set_ylabel("yaw °", fontsize=9, color=INK2)
    ax.legend(fontsize=7, frameon=False, loc="upper right")
    ax = axes[1]
    ax.plot(lg_l.t, lg_l.err * DEG, color=C2, lw=0.8, label="legacy")
    ax.plot(lg_b.t, lg_b.err * DEG, color=C1, lw=0.8, label="baseline")
    ax.set_ylabel("roll error °", fontsize=9, color=INK2)
    ax.legend(fontsize=7, frameon=False, loc="upper right")
    ax = axes[2]
    ax.plot(lg_l.t, lg_l.i, color=C2, lw=0.6, label="legacy")
    ax.plot(lg_b.t, lg_b.i, color=C1, lw=0.6, label="baseline")
    ax.plot(lg_b.t, lg_b.i_lim, color=INK2, ls="--", lw=1, label="drive limit")
    ax.plot(lg_b.t, -lg_b.i_lim, color=INK2, ls="--", lw=1)
    ax.set_ylabel("current A", fontsize=9, color=INK2)
    ax.legend(fontsize=7, frameon=False, loc="upper right", ncol=3)
    ax = axes[3]
    ax.step(lg_l.t, lg_l.clipped, color=C2, lw=0.8, label="legacy at limit")
    ax.step(lg_b.t, lg_b.clipped + 1.2, color=C1, lw=0.8, label="baseline at limit")
    ax.set_yticks([])
    ax.set_xlabel("time s", color=INK2)
    ax.legend(fontsize=7, frameon=False, loc="upper right")
    for a in axes:
        style(a)
        a.axvline(3.0, color=INK2, lw=0.8, ls=":")
    axes[0].text(3.05, axes[0].get_ylim()[1] * 0.8, "derate 3.2→2.4 A", fontsize=8, color=INK2)
    fig.suptitle("Not tracking is the right answer: derated drive + stronger-than-modelled yaw coupling",
                 x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p2_fault_derate.png"), dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
def section_margins(out):
    ctl = BaselineController()
    wc, K, a = ctl.wc, ctl.K, ctl.alpha
    rows = []
    for name, T, J, kt in [("nominal delay 6.0 ms", 6.0e-3, P.J_R, 1.0),
                           ("design: worst-normal 7.0 ms", 7.0e-3, P.J_R, 1.0),
                           ("every message in a burst 11 ms", 11e-3, P.J_R, 1.0),
                           ("7 ms, J +30 %", 7e-3, P.J_R * 1.3, 1.0),
                           ("7 ms, payload 0.7 kg (J +22 %)", 7e-3, P.J_R + 0.7 * 0.035 ** 2, 1.0),
                           ("7 ms, J −30 %", 7e-3, P.J_R * 0.7, 1.0),
                           ("7 ms, Kt +15 %", 7e-3, P.J_R, 1.15),
                           ("7 ms, Kt −15 %", 7e-3, P.J_R, 0.85),
                           ("11 ms, J −30 %, Kt +15 % (worst corner)", 11e-3, P.J_R * 0.7, 1.15)]:
        pm, gm, w = LS.margins(K * kt, wc, T, a, J=J)
        dm = (pm / DEG) / w * 1e3
        rows.append([name, f"{pm:.0f}°", f"{gm:.1f} dB", f"{w / 2 / math.pi:.1f} Hz", f"{dm:.1f} ms"])
    out += ["## 4. Loop margins of the baseline (analytic, same loop as the simulator)", "",
            f"Design: α = {a}, ωc = {wc:.1f} rad/s, K = {K:.3f} N·m/rad, integral at 0.1 ωc; "
            f"current per encoder count ≈ {K * a * P.ENC_LSB / P.K_T * 1e3:.0f} mA.", "",
            table(["case", "PM", "GM", "crossover", "extra delay tolerated"], rows), ""]


def fig_bode():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    ctl = BaselineController()
    w = np.logspace(0, 3.3, 800)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    for T, col, lab in ((6e-3, C1, "6 ms (nominal)"), (11e-3, C2, "11 ms (all bursts)")):
        C, G, Pl = LS.freq_response(w, ctl.K, ctl.wc, T, ctl.alpha)
        L = C * G * Pl
        a1.semilogx(w / 2 / math.pi, 20 * np.log10(np.abs(L)), color=col, lw=2, label=lab)
        a2.semilogx(w / 2 / math.pi, LS.phase(w, ctl.wc, T, ctl.alpha) * DEG, color=col, lw=2)
    a1.axhline(0, color=INK2, lw=0.8)
    a2.axhline(-180, color=INK2, lw=0.8)
    for f, lab in ((1.5, "B"), (2.2, "C")):
        for a in (a1, a2):
            a.axvline(f, color=INK2, ls=":", lw=0.8)
        a1.text(f, 42, f"yaw {lab}", fontsize=7, color=INK2, ha="center")
    a1.set_ylabel("|L| dB", color=INK2)
    a2.set_ylabel("phase °", color=INK2)
    a2.set_xlabel("frequency Hz", color=INK2)
    a2.set_ylim(-360, -60)
    a1.set_ylim(-40, 50)
    a1.legend(fontsize=8, frameon=False)
    for a in (a1, a2):
        style(a)
    fig.suptitle("Baseline open loop with delay: phase lag sets the ~5 Hz crossover", x=0.01,
                 ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p2_bode.png"), dpi=140)
    plt.close(fig)


def main():
    os.makedirs(FIGS, exist_ok=True)
    out = ["# Phase 2 numbers (generated by exp/phase2_eval.py)", ""]
    keep = section_runs(out)
    fig_runs(keep)
    section_margins(out)
    fig_bode()
    faults = section_faults(out)
    fig_fault(faults)
    mc = section_montecarlo(out)
    fig_montecarlo(mc)
    text = "\n".join(out)
    with open(os.path.join(ROOT, "report", "phase2_numbers.md"), "w") as f:
        f.write(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
