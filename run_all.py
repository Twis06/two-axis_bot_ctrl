"""One-command reproduction of the current evidence (EXECUTION_PLAN.md R4).

    python run_all.py              # full: tests + every current analysis/experiment, published to report/
    python run_all.py --quick      # tests + Phase 0 only; does NOT reproduce the evidence (no gate)
    python run_all.py --historical # also regenerate the superseded Phase 2 reports

Every step's stdout/stderr is appended to report/reproduction_log.txt with the
command, exit code, duration and the source snapshot (git commit, baseline
fingerprint, Python and library versions). A failed step stops the run and is
recorded as a failure. Registrations (report/task5_registration.json,
report/task3_challenges_registration.json) are inputs: they are never
regenerated here.
"""
import os
import platform
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
FULL = [
    ("validation tests", [PY, "-m", "unittest", "discover", "-s", "tests"]),
    ("Task 1: phase 0 analysis", [PY, "analysis/phase0.py"]),
    ("Task 1: phase 1 reproduction of runs A-E", [PY, "exp/reproduce_runs.py"]),
    ("Task 2: frozen baseline evidence", [PY, "-m", "exp.task2_eval"]),
    ("Task 2: robustness evidence", [PY, "-m", "exp.task2_robustness"]),
    ("Task 3 L2: estimator audit", [PY, "-m", "exp.task3_estimator_audit"]),
    ("Task 3 R3: supplementary challenges (registered)", [PY, "-m", "exp.task3_challenges", "--out", "report"]),
    ("Task 3 L4: matched comparison and adoption gate",
     [PY, "-m", "exp.task3_l4", "--challenges", "report/task3_challenges_results.json"]),
    ("Task 4 Packet 4B: matched baseline validation", [PY, "-m", "exp.task4b_eval"]),
    ("Task 5: registered motor-strength test (analysis corrected in R2)", [PY, "-m", "exp.task5_prediction"]),
    ("Task 5: prospective closed-loop prediction (registered before the runs)", [PY, "-m", "exp.task5_prospective"]),
    ("Task 5: answer overview (prospective test + historical study)", [PY, "-m", "exp.task5_overview"]),
    ("Memo Figure 8: calculated static-holdability diagram", [PY, "report/html_assets/holdability_figure.py"]),
]
HISTORICAL = [("historical: phase 2 baseline evaluation", [PY, "exp/phase2_eval.py"])]
QUICK = FULL[:2]
LOG = os.path.join(ROOT, "report", "reproduction_log.txt")


def snapshot():
    def git(*a):
        r = subprocess.run(["git", *a], cwd=ROOT, capture_output=True, text=True)
        return r.stdout.strip() if r.returncode == 0 else "unavailable"
    fp = subprocess.run([PY, "-c", "from exp.evidence import baseline_fingerprint as b; print(b()[0])"],
                        cwd=ROOT, capture_output=True, text=True).stdout.strip()
    libs = subprocess.run([PY, "-c", "import numpy, scipy, matplotlib; print(numpy.__version__, "
                           "scipy.__version__, matplotlib.__version__)"], cwd=ROOT, capture_output=True,
                          text=True).stdout.strip()
    snap = os.path.join(ROOT, "SNAPSHOT_COMMIT")        # written when a delivery archive is exported
    snap = open(snap).read().strip() if os.path.exists(snap) else "none"
    return [f"git commit: {git('rev-parse', 'HEAD')}", f"snapshot commit file: {snap}", f"git status: {git('status', '--porcelain') or 'clean'}",
            f"baseline fingerprint: {fp}", f"python: {platform.python_version()} ({PY})",
            f"numpy scipy matplotlib: {libs}", f"platform: {platform.platform()}"]


def main():
    steps = QUICK if "--quick" in sys.argv else FULL + (HISTORICAL if "--historical" in sys.argv else [])
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    with open(LOG, "a") as log:
        log.write(f"\n==== run_all.py {' '.join(sys.argv[1:]) or '(full)'} at {time.strftime('%Y-%m-%d %H:%M:%S %Z')}\n")
        log.write("\n".join(snapshot()) + "\n")
        if "--quick" in sys.argv:
            log.write("QUICK MODE: tests and Phase 0 only; the current evidence is NOT reproduced.\n")
        for name, cmd in steps:
            t0 = time.time()
            print(f"=== {name} ===", flush=True)
            log.write(f"\n--- {name}: {' '.join(cmd[1:])}\n")
            log.flush()
            r = subprocess.run(cmd, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            dt = time.time() - t0
            log.write(f"--- exit {r.returncode} after {dt:.0f} s\n")
            log.flush()
            if r.returncode:
                print(f"FAILED: {name} (see report/reproduction_log.txt)")
                sys.exit(1)
            print(f"    ok ({dt:.0f} s)", flush=True)
    print("Done. Outputs in report/; log in report/reproduction_log.txt")


if __name__ == "__main__":
    main()
