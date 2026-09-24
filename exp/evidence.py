"""Evidence plumbing for the frozen-baseline tables (Packet 2C, reused by 4B).

Every simulated run that feeds a published table goes through `RunBook.run`:
it is described by an exp.manifest manifest before it runs (run_id = hash of the
result-determining fields), scored with sim.metrics.evaluate (Packet 4A), and
recorded. `RunBook.publish` then writes the tables, figures and a single
`<name>_runs.json` (shared code/environment block once, one entry per run_id)
through exp.manifest.staged_publish, after checking that no source file changed
while the evaluation ran. Tables cite run_ids, so every row traces to a run.
"""
import hashlib

from ctrl.supervisor import DriveSupervisor
from exp import manifest as MF
from exp.common import run
from sim import metrics as SM
from sim.drive import DriveSafety
from exp import scenarios as S

# Fields of sim.metrics.evaluate kept per run in the published record (the full
# evaluation is reproducible from the manifest; these are what tables use).
KEEP = ("tracked", "not_tracked_because", "progress", "errors", "completion", "summary")


def _supervisor(kind):
    if kind == "design":
        return DriveSupervisor()
    if kind == "legacy":
        return DriveSafety(**S.LEGACY_WATCHDOG)
    return kind


# The frozen deterministic baseline: every module BaselineController and the
# simulator import (not the Task 3 estimator or the legacy comparator).
BASELINE_SOURCES = ("ctrl/__init__.py", "ctrl/baseline.py", "ctrl/governor.py", "ctrl/interfaces.py",
                    "ctrl/loopshape.py", "ctrl/supervisor.py", "ctrl/yaw_estimator.py",
                    "sim/__init__.py", "sim/config.py", "sim/drive.py", "sim/engine.py", "sim/metrics.py",
                    "sim/params.py", "sim/plant.py", "sim/sensing.py", "sim/timing.py",
                    "sim/trajectories.py")


def baseline_fingerprint():
    """(hash, per-file sha256) of BASELINE_SOURCES: the identity of the frozen baseline."""
    hashes = MF.source_hashes([MF.ROOT / p for p in BASELINE_SOURCES])
    return MF.code_hash(hashes), hashes


def run_set_id(ids):
    """Short id for a set of runs behind one table row (order-independent)."""
    return hashlib.sha256(",".join(sorted(ids)).encode()).hexdigest()[:10]


class RunBook:
    def __init__(self, entry):
        self.entry, self.runs, self.code, self.env, self.defs = entry, {}, None, None, None

    def run(self, sc, make_ctrl, seed=1, supervisor="design", governed_yaw=True):
        """Run one case; returns (log, summarize() stats, run_id, 4A evaluation)."""
        ctrl = make_ctrl()
        sup = _supervisor(supervisor)
        man = MF.make_manifest(sc, ctrl, seed, supervisor=sup, governed_yaw=governed_yaw,
                               entry=self.entry)
        if self.code is None:
            self.code, self.env, self.defs = man["code"], man["env"], man["metric_defs"]
        elif man["code"]["code_hash"] != self.code["code_hash"]:
            raise RuntimeError("code hash changed during the evaluation")
        log, stats = run(sc, lambda: ctrl, seed=seed, supervisor=sup, governed_yaw=governed_yaw)
        ev = SM.evaluate(log, ref=sc.roll, yaw_request=sc.yaw,
                         T_request=getattr(sc, "t_request", None),
                         waypoints=getattr(sc, "waypoints", None))
        rid = man["run_id"]
        self.runs[rid] = dict(run=man["run"], metrics={k: ev.get(k) for k in KEEP})
        return log, stats, rid, ev

    def check_unchanged(self, stage=None):
        bad = MF.changed_sources(dict(code=self.code))
        if bad:
            raise RuntimeError(f"source files changed during evaluation: {bad}")

    def record(self):
        return dict(manifest_version=MF.MANIFEST_VERSION, entry=self.entry, code=self.code,
                    env=self.env, metrics_version=SM.METRICS_VERSION, metric_defs=self.defs,
                    runs=self.runs)

    def fingerprint(self):
        return dict(baseline=baseline_fingerprint()[0], code_hash=self.code["code_hash"],
                    git=self.code["git"], metrics_version=SM.METRICS_VERSION)
