"""Compare a reproduced report/ directory with the published one.

Usage:
    uv run python tools/compare_evidence.py <reproduced report dir> [--published report]

What is compared (the method used for the R4 reproduction numbers):
  * Result files: task2_results, task4b_results, task3_ceiling_results,
    task3_challenges_results, task3_estimator_audit, task5_results (.json).
    Every leaf is flattened; keys naming provenance (run_id, code, git, env, hashes,
    timestamps) and the L2 wall-clock field actual_calculation_wall_ms are skipped.
    Strings and booleans must be equal. Floats are counted as bit-identical or not,
    and must agree within |a - b| <= 1e-12 + 1e-9 * max(|a|, |b|).
  * run_ids: per *_runs.json, the sets are compared; in task2/task4b, differing ids are
    classified as Monte Carlo (seed >= 100, the sampled plants) or not.
  * Figures: report/figs/*.png byte equality, and if bytes differ the share of
    pixels that differ (needs Pillow, which Matplotlib already installs).
Exit status 0 when no string/boolean differs and every float is within tolerance.
"""
import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

RESULTS = ("task2_results", "task4b_results", "task3_ceiling_results", "task3_challenges_results",
           "task3_estimator_audit", "task5_results")
RUNS = ("task2_runs", "task4b_runs", "task3_ceiling_runs", "task3_challenges_runs")
MONTE_CARLO = ("task2_runs", "task4b_runs")
SKIP = ("run_id", "run_ids", "code", "git", "env", "hash", "sha", "manifest", "time", "date",
        "source_hashes", "figure_checks", "stage", "wall_ms")
RTOL, ATOL = 1e-9, 1e-12


def flat(x, p=""):
    out = {}
    if isinstance(x, dict):
        if set(x) == {"$float"}:                       # manifest encoding of nan/inf
            return {p: float(x["$float"])}
        for k, v in x.items():
            if not any(s in k.lower() for s in SKIP):
                out.update(flat(v, f"{p}.{k}"))
    elif isinstance(x, list):
        for i, v in enumerate(x):
            out.update(flat(v, f"{p}[{i}]"))
    else:
        out[p] = float(x) if isinstance(x, int) and not isinstance(x, bool) else x
    return out


def compare_results(pub, rep):
    ok, total_f, not_bit, worst = True, 0, 0, (0.0, None)
    for name in RESULTS:
        a = flat(json.loads((pub / f"{name}.json").read_text()))
        b = flat(json.loads((rep / f"{name}.json").read_text()))
        bad = []
        for k in sorted(set(a) | set(b)):
            x, y = a.get(k), b.get(k)
            if isinstance(x, float) and isinstance(y, float):
                total_f += 1
                if math.isnan(x) and math.isnan(y) or x == y:
                    continue
                not_bit += 1
                rel = abs(x - y) / max(abs(x), abs(y))
                if rel > worst[0]:
                    worst = (rel, f"{name}{k}")
                if abs(x - y) > ATOL + RTOL * max(abs(x), abs(y)):
                    bad.append((k, x, y))
            elif x != y:
                bad.append((k, x, y))
        ok &= not bad
        print(f"{name}: {len(a)} leaves, {len(bad)} outside tolerance" + "".join(f"\n    {d}" for d in bad[:5]))
    print(f"floats: {total_f}; not bit-identical: {not_bit}; worst relative difference: {worst[0]:.2e} {worst[1] or ''}")
    return ok


def compare_runs(pub, rep):
    for name in RUNS:
        a = json.loads((pub / f"{name}.json").read_text())["runs"]
        b = json.loads((rep / f"{name}.json").read_text())["runs"]
        only = set(a) - set(b)
        line = f"{name}: {len(a)} runs; run_ids differing: {len(only)}"
        if name in MONTE_CARLO:            # seeds >= 100 are the sampled plants only in these packets
            line += f" (Monte Carlo: {sum(1 for r in only if (a[r]['run'].get('seed') or 0) >= 100)})"
        print(line)


def compare_figures(pub, rep):
    same, diff = 0, []
    for f in sorted((pub / "figs").glob("*.png")):
        g = rep / "figs" / f.name
        if not g.exists():
            diff.append((f.name, "missing"))
        elif hashlib.sha256(f.read_bytes()).digest() == hashlib.sha256(g.read_bytes()).digest():
            same += 1
        else:
            try:
                import numpy as np
                from PIL import Image
                x = np.asarray(Image.open(f).convert("RGB"), float)
                y = np.asarray(Image.open(g).convert("RGB"), float)
                share = "shape differs" if x.shape != y.shape else f"{100 * np.mean(np.abs(x - y).max(axis=2) > 0):.3f}% pixels differ"
            except ImportError:
                share = "bytes differ"
            diff.append((f.name, share))
    print(f"figures: {same} byte-identical" + "".join(f"\n    {n}: {s}" for n, s in diff))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("reproduced", type=Path)
    ap.add_argument("--published", type=Path, default=Path(__file__).resolve().parents[1] / "report")
    a = ap.parse_args(argv)
    ok = compare_results(a.published, a.reproduced)
    compare_runs(a.published, a.reproduced)
    compare_figures(a.published, a.reproduced)
    print("RESULT:", "agree within tolerance" if ok else "DIFFERENCES OUTSIDE TOLERANCE")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
