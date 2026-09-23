"""One-command reproduction: tests, then every analysis and experiment.

    python run_all.py            # full
    python run_all.py --quick    # tests + Phase 0 only
"""
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
STEPS = [
    ("validation tests", [sys.executable, "-m", "unittest", "discover", "-s", "tests"]),
    ("phase 0 analysis", [sys.executable, "analysis/phase0.py"]),
    ("phase 1 reproduce runs A-E", [sys.executable, "exp/reproduce_runs.py"]),
    ("phase 2 baseline evaluation", [sys.executable, "exp/phase2_eval.py"]),
    ("task 2 hardened baseline evidence", [sys.executable, "-m", "exp.task2_eval"]),
]
QUICK = 2


def main():
    steps = STEPS[:QUICK] if "--quick" in sys.argv else STEPS
    for name, cmd in steps:
        t0 = time.time()
        print(f"=== {name} ===", flush=True)
        out = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.DEVNULL if "analysis" in cmd[-1]
                             or "exp" in cmd[-1] else None)
        if out.returncode:
            sys.exit(f"FAILED: {name}")
        print(f"    ok ({time.time() - t0:.0f} s)")
    print("Outputs: report/*.md, report/figs/*.png")


if __name__ == "__main__":
    main()
