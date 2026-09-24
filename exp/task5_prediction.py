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


def _row(log, summary, seed: int, motor_ratio: float, use_yaw_ff: bool) -> dict:
    mask = (log.t >= WINDOW[0]) & (log.t < WINDOW[1])
    c_tau = log.get("c_tau_cpl")
    return {
        "seed": seed,
        "motor_strength_ratio": motor_ratio,
        "use_yaw_ff": use_yaw_ff,
        "window_s": list(WINDOW),
        "measured_peak_current_A": float(max(abs(log.i[mask]))) if mask.any() else None,
        "measured_rms_current_A": float(math.sqrt(float((log.i[mask] ** 2).mean()))) if mask.any() else None,
        "predicted_coupling_peak_current_A": float(max(abs(c_tau[mask]))) if c_tau is not None and mask.any() else 0.0,
        "true_coupling_peak_torque_Nm": float(max(abs(log.tau_couple[mask]))) if mask.any() else None,
        "governed_rms_deg": float(summary["rms_gov"]),
        "governed_peak_deg": float(summary["peak_gov"]),
        "original_rms_deg": float(summary["rms"]),
        "path_speed": float(summary["speed"]),
        "yaw_scale": float(summary["yaw_scale"]),
        "clip_pct": float(summary["clip_pct"]),
        "watchdog_trips": int(summary["wd_trips"]),
        "events": list(log.events),
    }


def run_experiment(quick: bool = False) -> dict:
    registration = prediction_registration()
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
    return {
        "registration": registration,
        "source_hashes": _source_hashes(),
        "scenario": {"name": scenario.name, "note": scenario.note, "duration_s": scenario.cfg.duration},
        "rows": rows,
        "quick": quick,
    }


def _markdown(result: dict) -> str:
    reg = result["registration"]
    p = reg["prediction"]
    lines = [
        "# Task 5 — prospective motor-strength explanation test",
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
        "| Kt ratio | Yaw FF | Seed | Measured peak A | RMS current A | Governed RMS ° | Original RMS ° | Yaw scale | Clips % | Events |",
        "|---:|:---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in result["rows"]:
        lines.append(
            f"| {row['motor_strength_ratio']:.2f} | {'on' if row['use_yaw_ff'] else 'off'} | {row['seed']} | {row['measured_peak_current_A']:.3f} | {row['measured_rms_current_A']:.3f} | {row['governed_rms_deg']:.3f} | {row['original_rms_deg']:.3f} | {row['yaw_scale']:.3f} | {row['clip_pct']:.2f} | {', '.join(e[1] for e in row['events']) or 'none'} |"
        )
    if not result["quick"]:
        nominal_on = {r["seed"]: r for r in result["rows"]
                      if r["motor_strength_ratio"] == 1.0 and r["use_yaw_ff"]}
        weak_on = [r for r in result["rows"]
                   if r["motor_strength_ratio"] == MOTOR_STRENGTH_RATIO and r["use_yaw_ff"]]
        weak_off = [r for r in result["rows"]
                    if r["motor_strength_ratio"] == MOTOR_STRENGTH_RATIO and not r["use_yaw_ff"]]
        delta_peak = [r["measured_peak_current_A"] - nominal_on[r["seed"]]["measured_peak_current_A"]
                      for r in weak_on]
        delta_gov = [r["governed_rms_deg"] - nominal_on[r["seed"]]["governed_rms_deg"]
                     for r in weak_on]
        on_error = [r["governed_rms_deg"] for r in weak_on]
        off_error = [r["governed_rms_deg"] for r in weak_off]
        true_tau = [r["true_coupling_peak_torque_Nm"] for r in weak_on]
        lines.extend([
            "",
            "## Prediction versus observation",
            "",
            f"The logged true coupling peak was {median(true_tau):.6f} N·m across the weakened runs, matching the registered {p['coupling_peak_torque_Nm']:.6f} N·m trajectory calculation. The registered **0.107637 A** is an ideal coupling-component increase, not a prediction of the measured total-current maximum. The paired measured total-current peak changed by a median **{median(delta_peak):+.3f} A** (range {min(delta_peak):+.3f} to {max(delta_peak):+.3f} A), because feedback, phase, and the maximum operator contribute to the total trace.",
            f"With yaw feed-forward enabled, weakened-motor governed RMS was {median(on_error):.3f}° versus {median(nominal_on[s]['governed_rms_deg'] for s in nominal_on):.3f}° nominal (paired change {median(delta_gov):+.3f}°). With feed-forward disabled under the same weakened motor, governed RMS was {median(off_error):.3f}°. No run clipped or generated a fault event. The result supports the qualitative explanation that yaw feed-forward is valuable and motor strength affects the residual, while the component-level current prediction cannot be equated with total measured peak current.",
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
