"""Standalone Task 3 L2 audit for the bounded payload estimator.

This module deliberately uses only ``PayloadEstimator``'s public observation,
tick, snapshot, and correction API.  The cases are replay-like synthetic
sensor streams; none is a plant simulation or a closed-loop benefit claim.

Run from the repository root with::

    python -m exp.task3_estimator_audit

``--quick`` uses a 250 Hz event stream for smoke checks.  The published audit
uses the frozen 500 Hz protocol and seeds 201--205.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import time
from pathlib import Path
from statistics import median
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from ctrl.payload_estimator import PayloadEstimator, PayloadObservation


COUNT = PayloadEstimator.ENCODER_COUNT
DEG = math.pi / 180.0
BASE_COEFFICIENTS = (0.08, 0.16)
UNSEEN_ANGLES = tuple(x * DEG for x in (-65.0, -30.0, 30.0, 65.0))
AUDIT_SEEDS = (201, 202, 203, 204, 205)


def _minimum_jerk(q0: float, q1: float, x: float) -> float:
    x = max(0.0, min(1.0, x))
    p = 10.0 * x ** 3 - 15.0 * x ** 4 + 6.0 * x ** 5
    return q0 + (q1 - q0) * p


def _route_profile(amplitude_deg: float = 50.0, dwell: float = 1.0):
    """Return q(t), yaw(t), stationary(t) for the frozen finite route."""
    targets = (0.0, -amplitude_deg * DEG, 0.0, amplitude_deg * DEG, 0.0)
    t = dwell
    segments = []
    q0 = targets[0]
    for q1 in targets[1:]:
        segments.append((t, t + 1.0, q0, q1))
        t += 1.0
        segments.append((t, t + dwell, q1, q1))
        t += dwell
        q0 = q1

    def profile(t_now: float) -> Tuple[float, float, bool]:
        if t_now < dwell:
            return targets[0], 0.0, True
        for start, end, qa, qb in segments:
            if start <= t_now < end:
                if qa == qb:
                    return qa, 0.0, True
                return _minimum_jerk(qa, qb, (t_now - start) / (end - start)), 0.0, False
        return targets[-1], 0.0, True

    return profile, t


def _moving_profile(t_now: float, start: float):
    phase = max(0.0, t_now - start)
    q = 0.35 * math.sin(0.7 * phase)
    qy = 0.22 * math.sin(0.5 * phase + 0.4)
    return q, qy, False


def _case_setup(name: str, quick: bool = False):
    dwell = 0.70 if quick else 1.0
    route, route_end = _route_profile(dwell=dwell)
    if name in {
        "ideal", "bounded_disturbance", "constant_disturbance", "current_bias",
        "duplicates", "pending_fault", "feedback_gap", "payload_change",
    }:
        duration = route_end + 0.05
        q_profile = route
    elif name == "narrow_coverage":
        q_profile, duration = _route_profile(amplitude_deg=15.0, dwell=dwell)
        duration += 0.05
    elif name == "short_dwell":
        q_profile, duration = _route_profile(amplitude_deg=50.0, dwell=0.15)
        duration += 0.05
    elif name in {"moving", "expiry"}:
        duration = route_end + (1.0 if name == "moving" else 31.5)

        def q_profile(t_now: float):
            if t_now < route_end:
                return route(t_now)
            return _moving_profile(t_now, route_end)
    else:
        raise ValueError(f"unknown audit case: {name}")

    def coefficients(t_now: float):
        if name == "payload_change" and t_now >= 5.2:
            return (-0.04, 0.04)
        return BASE_COEFFICIENTS

    def disturbance(t_now: float, seed: int):
        if name == "bounded_disturbance":
            phase = (seed % 5) * 0.37
            return 0.05 * math.sin(2.0 * math.pi * 0.4 * t_now + phase)
        if name == "constant_disturbance":
            return 0.05
        return 0.0

    def current_bias(t_now: float):
        if name != "current_bias":
            return 0.0
        return 0.10 if t_now < route_end / 2.0 else -0.10

    return q_profile, duration, coefficients, disturbance, current_bias, route_end


def _observation(t_meas: float, t_received: float, q: float, qy: float,
                 coefficients: Tuple[float, float], disturbance: float,
                 current_bias: float, *, mode: object = 0,
                 saturated: bool = False, i_limit: float = 3.2,
                 arbitrary_current: Optional[float] = None) -> PayloadObservation:
    if arbitrary_current is None:
        theta_s, theta_c = coefficients
        tau = (PayloadEstimator.NOMINAL_GRAVITY * math.sin(q) +
               theta_s * math.sin(q) + theta_c * math.cos(q) - disturbance)
        i_meas = tau / PayloadEstimator.CURRENT_TO_TORQUE + current_bias
    else:
        i_meas = arbitrary_current
    return PayloadObservation(
        t_meas=t_meas, t_received=t_received, q=q, qy=qy,
        i_meas=i_meas, i_limit=i_limit, mode=mode, saturated=saturated,
    )


def _record_snapshot(records: List[dict], snapshot, t_now: float,
                     estimator: PayloadEstimator, q: float):
    correction = estimator.correction(q, t_now)
    records.append({
        "t": t_now,
        "usable": bool(snapshot.usable),
        "reason": snapshot.reason,
        "accepted_sample_count": snapshot.accepted_sample_count,
        "last_training_time": snapshot.last_training_time,
        "last_publication_time": snapshot.last_publication_time,
        "pending": bool(snapshot.pending),
        "pending_ready_time": snapshot.pending_ready_time,
        "last_submission_time": snapshot.last_submission_time,
        "theta_s": snapshot.theta_s,
        "theta_c": snapshot.theta_c,
        "full_theta_s": snapshot.full_fit_coefficients[0],
        "full_theta_c": snapshot.full_fit_coefficients[1],
        "correction": correction,
        "coefficient_norm": snapshot.coefficient_norm,
        "health_generation": snapshot.health_generation,
        "worker_submission_count": snapshot.worker_submission_count,
    })


def _run_stream(name: str, seed: int, quick: bool = False) -> dict:
    q_profile, duration, coefficient_fn, disturbance_fn, bias_fn, route_end = _case_setup(name, quick)
    hz = 250.0 if quick else 500.0
    dt = 1.0 / hz
    rng = random.Random(seed)
    estimator = PayloadEstimator(seed=seed)
    events = []
    n = int(math.ceil(duration / dt))
    for index in range(n):
        t_meas = index * dt
        q, qy, stationary = q_profile(t_meas)
        delay = rng.uniform(0.0006, 0.0018)
        t_received = t_meas + delay
        obs = _observation(
            t_meas, t_received, q, qy, coefficient_fn(t_meas),
            disturbance_fn(t_meas, seed), bias_fn(t_meas),
        )
        events.append((t_received, t_meas, q, qy, stationary, obs))

    # Deliberate replay packets have fresh receipt times but the same physical
    # timestamp.  They test that receipt freshness cannot create training weight.
    if name == "duplicates":
        for t_received, t_meas, q, qy, stationary, obs in list(events):
            if t_meas >= route_end - 1.0 and int(round(t_meas / dt)) % 50 == 0:
                replay = PayloadObservation(
                    t_meas=t_meas, t_received=t_received + 0.010,
                    q=q, qy=qy, i_meas=obs.i_meas, i_limit=obs.i_limit,
                    mode=obs.mode, saturated=obs.saturated,
                )
                events.append((replay.t_received, replay.t_meas, q, qy, stationary, replay))
    events.sort(key=lambda item: item[0])

    # The gap is intentionally inserted while an early fit can be pending.
    gap = (5.50, 5.56) if name == "feedback_gap" else None
    records: List[dict] = []
    reason_transitions = []
    delays_ms = []
    structural: List[str] = []
    usable_seen = False
    first_usable_time = None
    training_while_moving = 0
    previous_count = 0
    previous_reason = None
    injected_fault = False
    gap_tick_done = False
    t_wall_start = time.perf_counter()

    for t_received, t_meas, q, qy, stationary, observation in events:
        if gap and not gap_tick_done and t_received >= gap[0]:
            gap_snapshot = estimator.tick(gap[1])
            _record_snapshot(records, gap_snapshot, gap[1], estimator, q)
            gap_tick_done = True
        if gap and gap[0] <= t_meas < gap[1]:
            continue

        estimator.observe(observation)
        snapshot = estimator.tick(t_received)
        if snapshot.pending and snapshot.pending_ready_time is not None:
            delay_ms = 1000.0 * (snapshot.pending_ready_time - snapshot.last_submission_time)
            delays_ms.append(delay_ms)

        # A fault packet deliberately reuses the current physical timestamp so
        # it can only be handled correctly if hard status checks precede replay
        # filtering.  This does not use private estimator state.
        if name == "pending_fault" and snapshot.pending and not injected_fault:
            bad = PayloadObservation(
                t_meas=observation.t_meas,
                t_received=observation.t_received + 0.0001,
                q=observation.q, qy=observation.qy, i_meas=observation.i_meas,
                i_limit=observation.i_limit, mode=1, saturated=False,
            )
            estimator.observe(bad)
            snapshot = estimator.tick(bad.t_received)
            injected_fault = True

        if snapshot.pending and snapshot.pending_ready_time is not None:
            delay_ms = 1000.0 * (snapshot.pending_ready_time - snapshot.last_submission_time)
            delays_ms.append(delay_ms)
        if (name in {"moving", "expiry"} and
                snapshot.accepted_sample_count > previous_count and not stationary):
            # Give a 100 ms transition allowance when a finite route hands off
            # to a moving test segment; after that, no dynamic packet may train.
            if not (name in {"moving", "expiry"} and t_meas < route_end + 0.10):
                training_while_moving += snapshot.accepted_sample_count - previous_count
        previous_count = snapshot.accepted_sample_count

        if snapshot.reason != previous_reason:
            reason_transitions.append({"t": t_received, "reason": snapshot.reason})
            previous_reason = snapshot.reason
        if snapshot.usable and first_usable_time is None:
            first_usable_time = t_received
        usable_seen = usable_seen or snapshot.usable
        if not math.isfinite(snapshot.coefficient_norm) or snapshot.coefficient_norm > 0.40 + 1e-10:
            structural.append("coefficient_norm_exceeded")
        if not math.isfinite(estimator.correction(q, t_received)) or abs(estimator.correction(q, t_received)) > 0.20 + 1e-10:
            structural.append("application_bound_exceeded")
        if snapshot.usable and snapshot.last_training_time is not None and t_received - snapshot.last_training_time > 30.000001:
            structural.append("expired_estimate_remained_usable")
        _record_snapshot(records, snapshot, t_received, estimator, q)

    final_t = events[-1][0] if events else duration
    final_snapshot = estimator.snapshot(final_t)
    _record_snapshot(records, final_snapshot, final_t, estimator, 0.0)
    wall_ms = 1000.0 * (time.perf_counter() - t_wall_start)

    full_errors = []
    prediction_errors = []
    applied_prediction_errors = []
    for rec in records:
        if rec["last_training_time"] is None:
            continue
        truth = coefficient_fn(rec["last_training_time"])
        full_errors.append(math.hypot(rec["full_theta_s"] - truth[0], rec["full_theta_c"] - truth[1]))
        if rec["usable"]:
            full_prediction_errors = [
                abs(rec["full_theta_s"] * math.sin(q) + rec["full_theta_c"] * math.cos(q) -
                    (truth[0] * math.sin(q) + truth[1] * math.cos(q)))
                for q in UNSEEN_ANGLES
            ]
            applied_prediction = [
                abs(rec["theta_s"] * math.sin(q) + rec["theta_c"] * math.cos(q) -
                    (truth[0] * math.sin(q) + truth[1] * math.cos(q)))
                for q in UNSEEN_ANGLES
            ]
            prediction_errors.extend(full_prediction_errors)
            applied_prediction_errors.extend(applied_prediction)

    # The gap must have zero usable correction at its explicit stale tick.
    if gap:
        gap_records = [r for r in records if abs(r["t"] - gap[1]) < 1e-9]
        if not gap_records or gap_records[0]["usable"] or abs(gap_records[0]["correction"]) > 1e-12:
            structural.append("feedback_gap_did_not_disable")
    if name == "pending_fault" and not injected_fault:
        structural.append("pending_fault_not_injected")
    if training_while_moving:
        structural.append("dynamic_observation_trained")

    return {
        "case": name,
        "seed": seed,
        "sample_rate_hz": hz,
        "duration_s": duration,
        "source_hash": hashlib.sha256(
            (Path(__file__).resolve().parents[1] / "ctrl" / "payload_estimator.py").read_bytes()
        ).hexdigest(),
        "usable_seen": usable_seen,
        "first_usable_time_s": first_usable_time,
        "final_reason": final_snapshot.reason,
        "final_usable": bool(final_snapshot.usable),
        "final_accepted_sample_count": final_snapshot.accepted_sample_count,
        "first_fit_time_s": min((r["last_publication_time"] for r in records if r["last_publication_time"] is not None), default=None),
        "disabled_duration_s": sum(
            max(0.0, b["t"] - a["t"])
            for a, b in zip(records, records[1:]) if not a["usable"]
        ),
        "coefficient_error_norm_max": max(full_errors, default=None),
        "unseen_angle_prediction_error_max_nm": max(prediction_errors, default=None),
        "applied_prediction_error_max_nm": max(applied_prediction_errors, default=None),
        "modeled_delay_ms": {
            "min": min(delays_ms, default=None),
            "max": max(delays_ms, default=None),
            "count": len(delays_ms),
        },
        "actual_calculation_wall_ms": wall_ms,
        "training_while_moving": training_while_moving,
        "fault_injected": injected_fault,
        "reason_transitions": reason_transitions,
        "structural_violations": sorted(set(structural)),
    }


def run_case(name: str, seed: int, quick: bool = False) -> dict:
    """Run one named L2 case and return a JSON-serializable record."""
    return _run_stream(name, seed, quick=quick)


CASES = (
    "ideal", "bounded_disturbance", "constant_disturbance", "current_bias",
    "narrow_coverage", "short_dwell", "moving", "feedback_gap", "duplicates",
    "payload_change", "pending_fault", "expiry",
)


def audit_all(seeds: Sequence[int] = AUDIT_SEEDS, quick: bool = False) -> dict:
    started = time.perf_counter()
    rows = [run_case(case, seed, quick=quick) for case in CASES for seed in seeds]
    return {
        "protocol": {
            "sample_rate_hz": 250.0 if quick else 500.0,
            "seeds": list(seeds),
            "cases": list(CASES),
            "base_coefficients_nm": list(BASE_COEFFICIENTS),
            "worker_delay_ms": [2.0, 8.0],
            "estimate_age_s": 30.0,
            "application_torque_cap_nm": 0.20,
            "quick": quick,
        },
        "source_hash": rows[0]["source_hash"] if rows else None,
        "rows": rows,
        "summary": {
            "run_count": len(rows),
            "structural_violation_count": sum(bool(row["structural_violations"]) for row in rows),
            "wall_time_s": time.perf_counter() - started,
        },
    }


def _summary_markdown(result: dict) -> str:
    rows = result["rows"]
    lines = [
        "# Task 3 L2 — standalone estimator audit",
        "",
        "This audit uses synthetic/replay-like timestamped observations through the public estimator API. It is not a plant simulation, hardware observation, or closed-loop benefit result.",
        "",
        f"Protocol: {result['protocol']['sample_rate_hz']:.0f} Hz, seeds {result['protocol']['seeds']}, modeled worker delay 2–8 ms, source hash `{result['source_hash']}`.",
        "",
        "## Case summary",
        "",
        "| Case | Usable seeds | Final reasons | Max coefficient error (N·m) | Max unseen prediction error (N·m) | Structural violations |",
        "|---|---:|---|---:|---:|---:|",
    ]
    for case in CASES:
        group = [r for r in rows if r["case"] == case]
        usable = sum(r["usable_seen"] for r in group)
        reasons = ", ".join(sorted({r["final_reason"] for r in group}))
        coeff = max((r["coefficient_error_norm_max"] for r in group if r["coefficient_error_norm_max"] is not None), default=None)
        pred = max((r["unseen_angle_prediction_error_max_nm"] for r in group if r["unseen_angle_prediction_error_max_nm"] is not None), default=None)
        violations = sum(bool(r["structural_violations"]) for r in group)
        lines.append(f"| {case} | {usable}/{len(group)} | {reasons} | {coeff if coeff is not None else '—'} | {pred if pred is not None else '—'} | {violations} |")

    lines.extend([
        "",
        "## Interpretation",
        "",
        "- Ideal quantized sensing checks convergence and unseen-angle prediction under the fixed gravity-like law.",
        "- Sinusoidal and constant disturbance, current-offset, and payload-change cases are bias/identifiability checks. Their errors are reported as limitations, not converted into confidence intervals.",
        "- Narrow coverage and short dwells are required abstention cases.",
        "- Moving, feedback-gap, duplicate, and pending-fault cases check that dynamic or stale data cannot train or re-enable an old result.",
        "- Expiry is based on the newest training measurement; fresh moving packets do not refresh an old model.",
        "",
        "No controller integration, matched comparator, or adoption claim is made by this packet.",
    ])
    return "\n".join(lines) + "\n"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quick", action="store_true", help="use the 250 Hz smoke protocol")
    parser.add_argument("--out", type=Path, default=Path("report"), help="report output directory")
    args = parser.parse_args(argv)
    result = audit_all(quick=args.quick)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "task3_estimator_audit.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    (args.out / "task3_estimator_audit.md").write_text(_summary_markdown(result))
    print(json.dumps(result["summary"], sort_keys=True))
    return 0 if result["summary"]["structural_violation_count"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
