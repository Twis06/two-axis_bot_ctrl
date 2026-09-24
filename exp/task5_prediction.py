"""Registered prospective Task 5 motor-strength explanation test.

The exact 15% motor-constant perturbation proposed in the old note appears in
the prior uncertainty/corner evidence.  This packet therefore registers a new
held-out matched condition: Run B with a 10% true motor-strength reduction,
matching back-EMF reduction, and seeds 21--23.  The controller still uses the
nominal motor constant.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from statistics import median
from typing import Optional, Sequence

from ctrl.baseline import BaselineController
from exp.common import run
from exp import scenarios as S
from sim import metrics as SM
from sim import params as P
from sim.config import SimConfig
from sim.trajectories import RampedSine


ROOT = Path(__file__).resolve().parents[1]
SEEDS = (21, 22, 23)
MOTOR_STRENGTH_RATIO = 0.90
WINDOW = (2.0, 8.0)


def _coupling_peak_current() -> float:
    yaw = RampedSine(math.radians(75.0), 1.5, t_ramp=1.0)
    peak = 0.0
    t = WINDOW[0]
    while t < WINDOW[1]:
        _, qyd, qydd = yaw.eval(t)
        peak = max(peak, abs(P.K_YV * qyd + P.K_YA * qydd))
        t += 0.001
    return peak / P.K_T


def prediction_registration() -> dict:
    """Return the frozen prediction and protocol before any new run is made."""
    coupling_current = _coupling_peak_current()
    extra_current = coupling_current * (1.0 / MOTOR_STRENGTH_RATIO - 1.0)
    peak_tau = coupling_current * P.K_T
    residual_tau = (1.0 - MOTOR_STRENGTH_RATIO) * peak_tau
    return {
        "claim": "With yaw feed-forward unchanged, a weaker true motor produces a predictable coupling-torque residual and requires proportionally more current.",
        "condition": "Run B nominal yaw trajectory, true Kt and Ke multiplied by 0.90, controller parameters unchanged",
        "motor_strength_ratio": MOTOR_STRENGTH_RATIO,
        "back_emf_ratio": MOTOR_STRENGTH_RATIO,
        "seeds": list(SEEDS),
        "window_s": list(WINDOW),
        "primary_component_metric": "peak ideal yaw-coupling current from the registered 1.5 Hz trajectory",
        "secondary_metrics": ["measured peak/RMS current", "governed RMS error", "path/original RMS error", "yaw scale", "fault events"],
        "prior_exposure_check": {
            "exact_matched_condition_new": True,
            "note": "Prior evidence contains Kt ±15% random/corner cases, including combined corners, but not this nominal Run B, Kt×0.90 paired intervention with seeds 21–23.",
        },
        "prediction": {
            "coupling_peak_torque_Nm": peak_tau,
            "coupling_peak_current_A": coupling_current,
            "extra_current_A": extra_current,
            "residual_torque_Nm": residual_tau,
            "component_current_tolerance_A": 0.15,
            "residual_torque_tolerance_Nm": 0.01,
            "rationale": "The plant coupling torque is unchanged. Ideal current is inverse in true Kt; the unchanged nominal feed-forward therefore delivers 0.90 of the intended torque, leaving 0.10 of the peak coupling torque as residual.",
        },
    }


def _source_hashes():
    paths = (
        "ctrl/baseline.py", "ctrl/governor.py", "ctrl/supervisor.py",
        "sim/config.py", "sim/engine.py", "sim/params.py", "sim/plant.py",
        "sim/trajectories.py", "exp/common.py", "exp/scenarios.py",
        "exp/task5_prediction.py",
    )
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}


DEG = 180.0 / math.pi
# R2 metric definitions (retrospective correction; see report/packets/R2.md).
METRIC_DEFS = {
    "window": "every windowed quantity uses t in [2, 8) s (the registered window)",
    "governed_rms_deg": "RMS of q - c_q_c over the window (governed reference actually commanded)",
    "original_rms_deg": "RMS of q - q_req(t) over the window (original request, wall clock)",
    "controller_coupling_ff_peak_torque_Nm": "max |c_tau_cpl| over the window: the controller's estimated "
                                             "coupling feed-forward torque, N m",
    "controller_coupling_ff_peak_current_A": "that torque / nominal Kt: the commanded coupling FF current, A",
    "ideal_true_coupling_peak_current_A": "max |tau_couple| / true Kt: current an ideal controller would need "
                                          "for the true coupling in this plant, A",
    "coupling_residual_peak_Nm": "max over the window of |tau_couple(t + T_act) - (Kt_true/Kt_nom) c_tau_cpl(t)|: "
                                 "coupling torque the unchanged feed-forward leaves for feedback, N m (FF on only)",
    "coupling_estimate_ls_gain": "least-squares gain of c_tau_cpl on the aligned true coupling over the window",
    "coupling_estimate_peak_ratio": "max|c_tau_cpl| / max|true coupling| over the window (a peak statistic)",
    "coupling_residual_peak_by_shift_ms": "residual peak for alignments of 0, 2, 4, 6 ms (sensitivity)",
    "yaw_scale_end_of_run": "yaw planner scale at the end of the run (not windowed)",
    "whole_run": "whole-run (0-8 s) safety evidence kept separately: events, trips, clipping, peak current",
}


def prediction_matrix(result: dict) -> list:
    """Each registered prediction against the quantity actually measured (R2).

    `basis` says whether the comparison was registered or chosen afterwards
    (post hoc); post-hoc rows are diagnostics and do not validate or refute the
    registration on their own."""
    p = result["registration"]["prediction"]
    rows = result["rows"]

    def pick(ratio, ff, mode="estimate"):
        return {r["seed"]: r for r in rows if r["motor_strength_ratio"] == ratio and r["use_yaw_ff"] == ff
                and r.get("yaw_info", "estimate") == mode}
    nom_on, weak_on, weak_off = pick(1.0, True), pick(MOTOR_STRENGTH_RATIO, True), pick(MOTOR_STRENGTH_RATIO, False)
    nom_plan, weak_plan = pick(1.0, True, "plan"), pick(MOTOR_STRENGTH_RATIO, True, "plan")
    med = lambda xs: float(median(xs))
    within = lambda x, c, tol: abs(x - c) <= tol
    true_tau = med([r["true_coupling_peak_torque_Nm"] for r in weak_on.values()])
    ideal_nom = med([r["ideal_true_coupling_peak_current_A"] for r in nom_on.values()])
    ff_cur = med([r["controller_coupling_ff_peak_current_A"] for r in nom_on.values()])
    gains = [r["coupling_estimate_ls_gain"] for r in nom_on.values()]
    ratios = [r["coupling_estimate_peak_ratio"] for r in nom_on.values()]
    ideal_extra = med([weak_on[s]["ideal_true_coupling_peak_current_A"] - nom_on[s]["ideal_true_coupling_peak_current_A"]
                       for s in weak_on])
    ff_extra = med([weak_on[s]["controller_coupling_ff_peak_current_A"] - nom_on[s]["controller_coupling_ff_peak_current_A"]
                    for s in weak_on])
    res_inc = [weak_on[s]["coupling_residual_peak_Nm"] - nom_on[s]["coupling_residual_peak_Nm"] for s in weak_on]
    rms_inc = [weak_on[s]["coupling_residual_rms_Nm"] - nom_on[s]["coupling_residual_rms_Nm"] for s in weak_on]
    shift = {ms: med([weak_on[s]["coupling_residual_peak_by_shift_ms"][ms] - nom_on[s]["coupling_residual_peak_by_shift_ms"][ms]
                      for s in weak_on]) for ms in ("0", "2", "4", "6")}
    plan_inc = [weak_plan[s]["coupling_residual_peak_Nm"] - nom_plan[s]["coupling_residual_peak_Nm"] for s in weak_plan] \
        if weak_plan and nom_plan else []
    d_gov = [weak_on[s]["governed_rms_deg"] - nom_on[s]["governed_rms_deg"] for s in weak_on]
    off_minus_on = [weak_off[s]["governed_rms_deg"] - weak_on[s]["governed_rms_deg"] for s in weak_on]
    tol_i, tol_r = p["component_current_tolerance_A"], p["residual_torque_tolerance_Nm"]
    out = [
        dict(prediction="peak coupling torque of the registered yaw trajectory", basis="registered",
             predicted=p["coupling_peak_torque_Nm"], unit="N m",
             measured="logged true coupling peak, weakened runs (median)", value=true_tau, window="[2, 8) s",
             tolerance="none registered", verdict="consistency check only",
             note="the simulator's coupling uses the same equation; this checks arithmetic, not the hardware explanation"),
        dict(prediction="peak ideal coupling current at nominal Kt (registered primary component metric)",
             basis="registered", predicted=p["coupling_peak_current_A"], unit="A",
             measured="ideal true-plant coupling current peak, nominal motor (median)", value=ideal_nom,
             window="[2, 8) s", tolerance=f"+/-{tol_i} A", verdict="consistency check only",
             note="the like-for-like quantity is again the prescribed model's arithmetic"),
        dict(prediction="DIAGNOSTIC (post hoc): commanded coupling FF current of the frozen baseline",
             basis="post hoc", predicted=p["coupling_peak_current_A"], unit="A",
             measured="controller's commanded coupling FF current peak, nominal motor (median)", value=ff_cur,
             window="[2, 8) s", tolerance=f"+/-{tol_i} A applied for reference only",
             verdict="diagnostic (peak outside tolerance)" if not within(ff_cur, p["coupling_peak_current_A"], tol_i)
             else "diagnostic (within tolerance)",
             note=f"the causal estimate's sustained gain is {min(gains):.3f}-{max(gains):.3f} (least squares, "
                  f"about +5 %) while its peak ratio is {min(ratios):.3f}-{max(ratios):.3f}: the +20 % is transient "
                  "overshoot at the coupling extrema, not a sustained gain error"),
        dict(prediction="extra ideal coupling current at Kt x 0.90", basis="registered", predicted=p["extra_current_A"],
             unit="A", measured="ideal true-plant coupling current, weak minus nominal (median, paired)", value=ideal_extra,
             window="[2, 8) s", tolerance=f"+/-{tol_i} A registered; it includes 0 A, so it cannot discriminate the "
             "predicted change from no change", verdict="not tested",
             note=f"the ideal value is the prescribed model's arithmetic; the commanded coupling FF current does not "
                  f"rise (paired change {ff_extra:+.4f} A, nominal FF unchanged), and the extra current feedback "
                  "supplies for the coupling component is not separable in the total current"),
        dict(prediction="coupling residual left by unchanged nominal FF", basis="post hoc operationalization",
             predicted=p["residual_torque_Nm"], unit="N m",
             measured="increase of the measured coupling-residual PEAK, weak minus nominal, FF on (median, paired); "
                      "metric chosen after the results", value=med(res_inc), window="[2, 8) s",
             tolerance=f"+/-{tol_r} N m (registered)",
             verdict="failed" if not within(med(res_inc), p["residual_torque_Nm"], tol_r) else "supported",
             note=f"paired range {min(res_inc):+.4f} to {max(res_inc):+.4f} N m; by alignment 0/2/4/6 ms: "
                  + ", ".join(f"{shift[m]:+.4f}" for m in ("0", "2", "4", "6"))
                  + f" (fails at every alignment); RMS residual change {med(rms_inc):+.4f} N m. The residual PEAK is "
                  "dominated by the estimator's transient error (about 0.03 N m), so a difference of peaks cannot "
                  "resolve an added 0.0136 N m component. DIAGNOSTIC plan mode (exact FF): "
                  + (f"{med(plan_inc):+.4f} N m, within tolerance: the prediction holds when FF equals the true "
                     "coupling" if plan_inc and within(med(plan_inc), p["residual_torque_Nm"], tol_r)
                     else (f"{med(plan_inc):+.4f} N m" if plan_inc else "not run"))),
        dict(prediction="qualitative: yaw feed-forward remains valuable with the weaker motor", basis="post hoc criterion",
             predicted=None, unit="deg", measured="governed RMS, FF off minus FF on, weak motor (median, paired)",
             value=med(off_minus_on), window="[2, 8) s", tolerance="sign in every pair (criterion chosen afterwards)",
             verdict="supported" if min(off_minus_on) > 0 else "failed",
             note=f"FF off is worse by {min(off_minus_on):+.3f} to {max(off_minus_on):+.3f} deg in the "
                  f"{len(off_minus_on)} pairs"),
        dict(prediction="qualitative: a weaker motor leaves more tracking error for feedback", basis="post hoc criterion",
             predicted=None, unit="deg", measured="governed RMS, weak minus nominal motor, FF on (median, paired)",
             value=med(d_gov), window="[2, 8) s", tolerance="sign in every pair (criterion chosen afterwards)",
             verdict="supported" if min(d_gov) > 0 else "not uniform",
             note=f"{sum(d > 0 for d in d_gov)} of {len(d_gov)} pairs increase; paired range {min(d_gov):+.3f} to "
                  f"{max(d_gov):+.3f} deg (the original analysis had the same negative pair)"),
    ]
    return out


def _rms(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(math.sqrt(float((x ** 2).mean()))) if len(x) else float("nan")


def _row(log, summary, seed: int, motor_ratio: float, use_yaw_ff: bool, info_mode: str = "estimate") -> dict:
    mask = (log.t >= WINDOW[0]) & (log.t < WINDOW[1])
    c_tau = log.get("c_tau_cpl")
    gov_err = (log.q - log.c_q_c)[mask] * DEG if "c_q_c" in log else log.err[mask] * DEG
    ff_tau = float(max(abs(c_tau[mask]))) if c_tau is not None and mask.any() else 0.0
    true_tau = float(max(abs(log.tau_couple[mask]))) if mask.any() else None
    k_true = P.K_T * motor_ratio
    # Coupling residual actually left for feedback: true coupling minus the torque the
    # true motor delivers for the commanded coupling FF current (c_tau_cpl is the
    # prediction for t + T_act, so compare it with the true coupling T_act later).
    res_peak = res_rms = gain = peak_ratio = float("nan")
    by_shift = {}
    if c_tau is not None and use_yaw_ff:
        dt = float(log.t[1] - log.t[0])
        k0 = int(round(BaselineController().T_act / dt))

        def aligned(k):
            n = len(log.t)
            est, true = np.asarray(c_tau)[:n - k], np.asarray(log.tau_couple)[k:]
            return est, true, mask[:n - k]
        for ms in (0, 2, 4, 6):                       # alignment sensitivity (R2 review I3)
            e, tr, m = aligned(int(round(ms * 1e-3 / dt)))
            r = tr[m] - motor_ratio * e[m]
            by_shift[str(ms)] = float(np.max(np.abs(r))) if r.size else float("nan")
        est, true, m = aligned(max(k0, 0))
        res = true[m] - motor_ratio * est[m]
        if res.size:
            res_peak, res_rms = float(np.max(np.abs(res))), _rms(res)
            # Sustained estimator gain (least squares) vs its peak ratio (R2 review I1).
            gain = float(np.dot(est[m], true[m]) / np.dot(true[m], true[m]))
            peak_ratio = float(np.max(np.abs(est[m])) / np.max(np.abs(true[m])))
    return {
        "seed": seed,
        "motor_strength_ratio": motor_ratio,
        "use_yaw_ff": use_yaw_ff,
        "yaw_info": info_mode,
        "window_s": list(WINDOW),
        "measured_peak_current_A": float(max(abs(log.i[mask]))) if mask.any() else None,
        "measured_rms_current_A": _rms(log.i[mask]) if mask.any() else None,
        "controller_coupling_ff_peak_torque_Nm": ff_tau,
        "controller_coupling_ff_peak_current_A": ff_tau / P.K_T,
        "true_coupling_peak_torque_Nm": true_tau,
        "ideal_true_coupling_peak_current_A": None if true_tau is None else true_tau / k_true,
        "coupling_residual_peak_Nm": res_peak,
        "coupling_residual_rms_Nm": res_rms,
        "coupling_residual_peak_by_shift_ms": by_shift,
        "coupling_estimate_ls_gain": gain,
        "coupling_estimate_peak_ratio": peak_ratio,
        "governed_rms_deg": _rms(gov_err),
        "governed_peak_deg": float(max(abs(gov_err))) if len(gov_err) else float("nan"),
        "original_rms_deg": _rms(log.err[mask] * DEG),
        "path_speed": float(log.c_gov_s[mask].mean()) if "c_gov_s" in log else float("nan"),
        "yaw_scale_end_of_run": float(summary["yaw_scale"]),
        "clip_pct": 100.0 * float(log.clipped[mask].mean()) if mask.any() else float("nan"),
        "whole_run": {
            "measured_peak_current_A": float(max(abs(log.i))),
            "clip_pct": float(summary["clip_pct"]),
            "watchdog_trips": int(summary["wd_trips"]),
            "events": list(log.events),
            "governed_rms_deg": float(summary["rms_gov"]),
        },
        "watchdog_trips": int(summary["wd_trips"]),
        "events": list(log.events),
    }


REGISTRATION_FILE = ROOT / "report" / "task5_registration.json"


def registration_audit(frozen: dict, regenerated: dict) -> dict:
    """Differences between the frozen registration file and what the code regenerates."""
    def flat(d, p=""):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, p + k + "."))
            else:
                out[p + k] = v
        return out
    f, g = flat(frozen), flat(regenerated)
    diffs = []
    for k in sorted(set(f) | set(g)):
        a, b = f.get(k, "<absent>"), g.get(k, "<absent>")
        if a != b:
            num = isinstance(a, float) and isinstance(b, float)
            diffs.append(dict(key=k, frozen=a, regenerated=b,
                              kind="numeric, relative %.1e" % (abs(a - b) / max(abs(a), 1e-300)) if num else "structural"))
    return dict(frozen_file="report/task5_registration.json", differences=diffs,
                ordering_evidence="registration and original results were first committed together (f19fff2); "
                                  "git gives no evidence that the registration preceded the run")


def run_experiment(quick: bool = False) -> dict:
    regenerated = prediction_registration()
    registration = json.loads(REGISTRATION_FILE.read_text()) if REGISTRATION_FILE.exists() else regenerated
    seeds = (SEEDS[0],) if quick else SEEDS
    scenario = S.run_b(SimConfig())
    rows = []
    for seed in seeds:
        for motor_ratio, cfg in ((1.0, scenario.cfg),
                                 (MOTOR_STRENGTH_RATIO,
                                  scenario.cfg.with_(plant=dict(
                                      k_t=P.K_T * MOTOR_STRENGTH_RATIO,
                                      k_e=P.K_E * MOTOR_STRENGTH_RATIO))),):
            for use_yaw_ff in (True, False):
                log, summary = run(
                    scenario,
                    lambda use=use_yaw_ff: BaselineController(use_yaw_ff=use),
                    seed=seed,
                    cfg=cfg,
                    governed_yaw=True,
                )
                rows.append(_row(log, summary, seed, motor_ratio, use_yaw_ff))
            if not quick:
                # DIAGNOSTIC (post hoc, not the frozen baseline's mode): exact plan look-ahead FF.
                log, summary = run(scenario, lambda: BaselineController(yaw_info="plan"), seed=seed,
                                   cfg=cfg, governed_yaw=True)
                rows.append(_row(log, summary, seed, motor_ratio, True, info_mode="plan"))
    result = {
        "registration": registration,
        "registration_audit": registration_audit(registration, regenerated),
        "metric_defs": METRIC_DEFS,
        "analysis": "R2 retrospective correction: windows, units and prediction-to-measurement mapping",
        "source_hashes": _source_hashes(),
        "scenario": {"name": scenario.name, "note": scenario.note, "duration_s": scenario.cfg.duration},
        "rows": rows,
        "quick": quick,
    }
    if not quick:
        result["prediction_matrix"] = prediction_matrix(result)
    return result


def _markdown(result: dict) -> str:
    reg = result["registration"]
    p = reg["prediction"]
    lines = [
        "# Task 5 — motor-strength explanation test (R2 retrospective correction of the analysis)",
        "",
        "Prediction registration precedes the new experiment. The exact nominal Run B, Kt×0.90 paired intervention was not present in the prior matched evidence; earlier Kt±15% random/corner cases remain prior context and are not relabeled.",
        "",
        f"Claim: {reg['claim']}",
        "",
        f"Frozen protocol: {reg['condition']}; seeds {reg['seeds']}; window {reg['window_s'][0]}–{reg['window_s'][1]} s; controller uses nominal Kt and Ke.",
        "",
        "## Registered prediction",
        "",
        f"The requested 1.5 Hz yaw trajectory has a calculated peak coupling torque of **{p['coupling_peak_torque_Nm']:.6f} N·m**, equivalent to **{p['coupling_peak_current_A']:.6f} A** at nominal Kt. With true Kt×0.90 and unchanged nominal feed-forward, the predicted extra ideal current is **{p['extra_current_A']:.6f} A** and the uncancelled coupling residual is **{p['residual_torque_Nm']:.6f} N·m**. Registered tolerances are ±{p['component_current_tolerance_A']:.2f} A for the component-current check and ±{p['residual_torque_tolerance_Nm']:.2f} N·m for the residual calculation.",
        "",
        "These are component-level predictions. Measured total current and tracking error also include feedback, friction, transport, voltage, and governor effects.",
        "",
        "## Matched results",
        "",
        "| Kt ratio | Yaw FF | Yaw info | Seed | Peak current A [2,8) s | RMS current A [2,8) s | Governed RMS ° [2,8) s | Original RMS ° [2,8) s | Yaw scale | Clipped % [2,8) s | Events (whole run) |",
        "|---:|:---:|---|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in result["rows"]:
        lines.append(
            f"| {row['motor_strength_ratio']:.2f} | {'on' if row['use_yaw_ff'] else 'off'} | {row.get('yaw_info', 'estimate')}{' (diagnostic)' if row.get('yaw_info') == 'plan' else ''} | {row['seed']} | {row['measured_peak_current_A']:.3f} | {row['measured_rms_current_A']:.3f} | {row['governed_rms_deg']:.3f} | {row['original_rms_deg']:.3f} | {row['yaw_scale_end_of_run']:.3f} | {row['clip_pct']:.2f} | {', '.join(e[1] for e in row['events']) or 'none'} |"
        )
    if not result["quick"]:
        fmt = lambda x: "—" if x is None else (f"{x:.6f}" if isinstance(x, float) else str(x))
        lines.extend([
            "",
            "## Prediction versus measurement (R2 retrospective correction)",
            "",
            "Rows above and below use one window, [2, 8) s, for every windowed quantity; whole-run safety events are in "
            "the JSON `whole_run` field. This is a **retrospective correction** of the original analysis (same frozen "
            "controller, conditions and seeds, replayed), not a new prospective experiment. The registration is unchanged.",
            "",
            "| Prediction | Basis | Predicted | Unit | Quantity actually measured | Measured | Tolerance | Verdict |",
            "|---|---|---:|---|---|---:|---|---|",
        ])
        for m in result["prediction_matrix"]:
            lines.append(f"| {m['prediction']} | {m['basis']} | {fmt(m['predicted'])} | {m['unit']} | {m['measured']} | "
                         f"{fmt(m['value'])} | {m['tolerance']} | **{m['verdict']}** |")
        lines.append("")
        for m in result["prediction_matrix"]:
            lines.append(f"- *{m['prediction']}:* {m['note']}.")
        lines.extend([
            "",
            "What this does and does not show: yaw feed-forward matters (every pair). The registered numerical "
            "predictions are arithmetic on the prescribed model (consistency checks) or were not measurable (extra "
            "current). Under the one operationalization of the residual chosen afterwards, the frozen baseline's "
            "residual increase is not resolved: the causal estimate's transient peak error dominates the residual peak, "
            "while the exact-FF plan-mode diagnostic shows the predicted increase. The weaker motor does not raise "
            "tracking error in every pair. The registration audit is in the JSON (`registration_audit`). The registered extra-current prediction was not tested as a "
            "measurable quantity, and its tolerance could not have discriminated it from zero. No closed-loop tracking-error "
            "prediction was registered, so none is claimed; a new prospective one would need its own registration first.",
        ])
    lines.extend([
        "",
        "The paired rows preserve the same scenario, seed, request, disturbance realization, transport draw, and initial state. The weakened-motor totals are reported separately from the registered component prediction; no controller retuning or design change was made.",
        "",
        "## Hardware follow-up",
        "",
        "Hardware confirmation still requires calibrated torque-versus-current identification and a timestamped roll-held yaw experiment with the same stop limits. This simulated result does not certify the drive convention or hardware Kt.",
    ])
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true")
    parser.add_argument("--out", type=Path, default=Path("report"))
    args = parser.parse_args(argv)
    result = run_experiment(quick=args.quick)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "task5_results.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.out / "task5.md").write_text(_markdown(result))
    print(json.dumps({"rows": len(result["rows"]), "quick": result["quick"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
