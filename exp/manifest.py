"""Run provenance and staged publishing (Packet 4A).

A report row is only evidence if it can be traced to the exact run that made
it. A manifest records what determines a simulated run - source-file hashes,
the full nested SimConfig, the requests, seed, time window, controller options,
supervisor, metric definitions - plus the environment and git state, which do
not determine the result but explain a mismatch when one appears.

run_id hashes only the result-determining fields (code hash, run spec, metric
version). Git commit, dirty flag and library versions are recorded beside it.

staged_publish() writes outputs to a private staging directory and copies them
to the target only after the whole evaluation (and an optional check, e.g. that
no source file changed mid-run) succeeded, so a partial or failed run never
overwrites a published table.
"""
import dataclasses
import hashlib
import json
import math
import os
import platform
import shutil
import subprocess
import sys
import tempfile
import traceback
from contextlib import contextmanager
from pathlib import Path

import numpy as np

from sim import metrics as SM
from sim.config import DriveConfig, PlantConfig, SensorConfig, SimConfig, TimingConfig

ROOT = Path(__file__).resolve().parents[1]
MANIFEST_VERSION = "4A.1"
SOURCE_DIRS = ("ctrl", "sim", "exp")


# -- JSON-safe conversion ------------------------------------------------------
def jsonable(x, depth=0):
    """Plain JSON types. NaN/inf -> None (strict JSON); dataclasses -> dicts;
    other objects -> describe(). Deterministic for equal inputs."""
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: jsonable(getattr(x, f.name), depth) for f in dataclasses.fields(x)}
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        return float(x) if math.isfinite(x) else None
    if isinstance(x, str) or x is None:
        return x
    if isinstance(x, dict):
        return {str(k): jsonable(v, depth) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [jsonable(v, depth) for v in x]
    return describe(x, depth + 1)


def describe(obj, depth=0):
    """Class plus public and private data attributes (trajectories, supervisors).
    Callables are skipped; nesting stops at depth 3."""
    if depth > 3:
        return f"<{type(obj).__name__}>"
    attrs = {k: jsonable(v, depth) for k, v in sorted(vars(obj).items())
             if not callable(v)} if hasattr(obj, "__dict__") else repr(obj)
    return {"class": f"{type(obj).__module__}.{type(obj).__qualname__}", "attrs": attrs}


def controller_options(ctrl):
    """Name plus every scalar/tuple option of a constructed controller (before
    reset), e.g. use_governor, alpha, fb_stale, and derived gains K, wc."""
    simple = (bool, int, float, str, type(None), np.floating, np.integer)
    opts = {k: jsonable(v) for k, v in sorted(vars(ctrl).items())
            if isinstance(v, simple) or (isinstance(v, tuple) and all(isinstance(e, simple) for e in v))}
    return {"name": getattr(ctrl, "name", type(ctrl).__name__),
            "class": f"{type(ctrl).__module__}.{type(ctrl).__qualname__}", "options": opts,
            "components": {k: type(v).__qualname__ for k, v in sorted(vars(ctrl).items())
                           if hasattr(v, "__dict__") and not isinstance(v, type)}}


def canonical(d):
    return json.dumps(d, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(d):
    return hashlib.sha256(canonical(d).encode()).hexdigest()


# -- configuration round trip ------------------------------------------------
_GROUPS = dict(plant=PlantConfig, timing=TimingConfig, drive=DriveConfig, sensor=SensorConfig)


def _tuples(v):
    return tuple(_tuples(e) for e in v) if isinstance(v, list) else v


def config_from_dict(d):
    """Inverse of jsonable(SimConfig): lists back to the tuples the config uses."""
    kw = {k: v for k, v in d.items() if k not in _GROUPS}
    for name, cls in _GROUPS.items():
        kw[name] = cls(**{k: _tuples(v) for k, v in d[name].items()})
    return SimConfig(**kw)


# -- code identity -------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def source_hashes(paths, root=ROOT):
    root = Path(root)
    return {str(Path(p).resolve().relative_to(root.resolve())): sha256_file(p) for p in sorted(map(str, paths))}


def code_hash(hashes):
    return hashlib.sha256("".join(f"{k}:{v}\n" for k, v in sorted(hashes.items())).encode()).hexdigest()


def loaded_sources(root=ROOT, dirs=SOURCE_DIRS):
    """Project .py files imported by this process under ctrl/, sim/, exp/: the
    code that can have affected a run (files never imported cannot)."""
    root = Path(root).resolve()
    out = set()
    for mod in list(sys.modules.values()):
        f = getattr(mod, "__file__", None)
        if not f or not f.endswith(".py"):
            continue
        p = Path(f).resolve()
        try:
            rel = p.relative_to(root)
        except ValueError:
            continue
        if rel.parts and rel.parts[0] in dirs:
            out.add(p)
    return sorted(out)


def git_state(root=ROOT, rel_paths=()):
    """Read-only git query (no index refresh). None fields if git is unavailable."""
    def git(*args):
        r = subprocess.run(["git", "--no-optional-locks", "-C", str(root), *args],
                           capture_output=True, text=True, timeout=20)
        if r.returncode:
            raise RuntimeError(r.stderr.strip())
        return r.stdout
    try:
        commit = git("rev-parse", "HEAD").strip()
        dirty_all = bool(git("status", "--porcelain").strip())
        lines = git("status", "--porcelain", "--", *rel_paths).splitlines() if rel_paths else []
        dirty_src = sorted(line[3:] for line in lines)
    except (OSError, RuntimeError, subprocess.SubprocessError):
        return dict(commit=None, dirty=None, dirty_sources=None)
    return dict(commit=commit, dirty=dirty_all, dirty_sources=dirty_src)


def environment():
    import scipy
    return dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
                platform=platform.platform(), executable=sys.executable)


# -- manifest ------------------------------------------------------------------
def make_manifest(scenario, controller, seed, t_window=None, supervisor=None, governed_yaw=True,
                  sources=None, extra=None, root=ROOT):
    """Manifest for one run. `controller` is the constructed controller instance
    (describe it before the run: reset() mutates it); `scenario` is a Scenario or
    FiniteMotion; t_window = (t_skip, t_end) used for the metrics."""
    cfg = scenario.cfg
    run = dict(scenario=scenario.name, note=getattr(scenario, "note", ""),
               config=jsonable(cfg), roll_request=describe(scenario.roll), yaw_request=describe(scenario.yaw),
               governed_yaw=bool(governed_yaw), seed=int(seed),
               t_window=list(t_window) if t_window is not None else [0.0, float(cfg.duration)],
               controller=controller_options(controller),
               supervisor=describe(supervisor) if supervisor is not None else None)
    for k in ("waypoints", "t_request"):
        if hasattr(scenario, k):
            run[k] = jsonable(getattr(scenario, k))
    if extra:
        run["extra"] = jsonable(extra)
    paths = sources if sources is not None else loaded_sources(root)
    hashes = source_hashes(paths, root)
    code = dict(sources=hashes, code_hash=code_hash(hashes), git=git_state(root, list(hashes)))
    ident = dict(run=run, code_hash=code["code_hash"], metrics_version=SM.METRICS_VERSION)
    return dict(manifest_version=MANIFEST_VERSION, run_id=digest(ident)[:16],
                metrics_version=SM.METRICS_VERSION, metric_defs=SM.METRIC_DEFS,
                run=run, code=code, env=environment())


def write_json(obj, path):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(jsonable(obj), indent=1, sort_keys=True, allow_nan=False) + "\n")


def read_json(path):
    return json.loads(Path(path).read_text())


def changed_sources(manifest, root=ROOT):
    """Files whose content differs from (or disappeared since) the manifest."""
    out = []
    for rel, h in manifest["code"]["sources"].items():
        p = Path(root) / rel
        if not p.exists() or sha256_file(p) != h:
            out.append(rel)
    return out


# -- staged publish ------------------------------------------------------------
@contextmanager
def staged_publish(target, staging_root=None, check=None):
    """with staged_publish(target) as stage: write files into `stage`.

    On normal exit (and if check(stage) does not raise) every staged file is
    copied beside its destination and then renamed over it (os.replace), so no
    destination is replaced until every copy has succeeded. On any exception
    the target is not touched and the stage is kept with a FAILED note."""
    target = Path(target)
    root = Path(staging_root or target.parent)
    root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".stage-", dir=root))
    try:
        yield stage
        if check is not None:
            check(stage)
    except BaseException:
        (stage / "FAILED").write_text(traceback.format_exc())
        raise
    files = [p for p in sorted(stage.rglob("*")) if p.is_file()]
    tmp = []
    try:
        for p in files:
            dst = target / p.relative_to(stage)
            dst.parent.mkdir(parents=True, exist_ok=True)
            t = dst.with_name(f".{dst.name}.publishing")
            shutil.copy2(p, t)
            tmp.append((t, dst))
    except BaseException:
        for t, _ in tmp:
            t.unlink(missing_ok=True)
        (stage / "FAILED").write_text(traceback.format_exc())
        raise
    for t, dst in tmp:
        os.replace(t, dst)
    shutil.rmtree(stage)
