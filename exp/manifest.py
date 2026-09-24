"""Run provenance and staged publishing (Packet 4A).

A report row is only evidence if it can be traced to the exact run that made
it. A manifest records what determines a simulated run - source-file hashes,
the full nested SimConfig, the requests, seed, time window, controller options,
supervisor, metric definitions - plus the environment and git state, which do
not determine the result but explain a mismatch when one appears.

run_id hashes only the result-determining fields (code hash, run spec without
its prose note, metric version). Git commit, dirty flag, library versions and
the list of project modules actually imported are recorded beside it.

Code identity is a DECLARED CLOSED SET, independent of what else the process
happened to import: every .py under ctrl/ and sim/, the evaluation plumbing in
exp/ (EXP_CORE), the entry module, and any exp modules the entry module lists
in its PROVENANCE_SOURCES. This is conservative: an edit to an unused ctrl/ or
sim/ file changes run_id. Imported project modules outside the set are listed
in code.undeclared_imports so an undeclared dependency is visible.

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
MANIFEST_VERSION = "4A.2"
SOURCE_DIRS = ("ctrl", "sim", "exp")
CLOSED_DIRS = ("ctrl", "sim")
EXP_CORE = ("exp/__init__.py", "exp/common.py", "exp/scenarios.py", "exp/motions.py", "exp/manifest.py")
_NONFINITE = {"inf": float("inf"), "-inf": float("-inf"), "nan": float("nan")}


# -- JSON-safe conversion ------------------------------------------------------
def jsonable(x, depth=0, tag_nonfinite=True):
    """Plain JSON types; dataclasses -> dicts; other objects -> describe().
    Non-finite floats -> {"$float": "inf"|"-inf"|"nan"} (strict JSON, lossless,
    so configs round-trip and inf stays distinct from NaN); tag_nonfinite=False
    gives None instead. Deterministic for equal inputs."""
    kw = dict(tag_nonfinite=tag_nonfinite)
    if dataclasses.is_dataclass(x) and not isinstance(x, type):
        return {f.name: jsonable(getattr(x, f.name), depth, **kw) for f in dataclasses.fields(x)}
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    if isinstance(x, (float, np.floating)):
        if math.isfinite(x):
            return float(x)
        return {"$float": "nan" if math.isnan(x) else ("inf" if x > 0 else "-inf")} if tag_nonfinite else None
    if isinstance(x, str) or x is None:
        return x
    if isinstance(x, dict):
        return {str(k): jsonable(v, depth, **kw) for k, v in x.items()}
    if isinstance(x, (list, tuple, np.ndarray)):
        return [jsonable(v, depth, **kw) for v in x]
    return describe(x, depth + 1, tag_nonfinite)


def class_path(cls):
    """module.qualname; a class defined in a module run as __main__ (python -m)
    is recorded under its import name so the record can be rebuilt."""
    mod = cls.__module__
    if mod == "__main__":
        spec = getattr(sys.modules.get("__main__"), "__spec__", None)
        mod = spec.name if spec is not None else mod
    return f"{mod}.{cls.__qualname__}"


def describe(obj, depth=0, tag_nonfinite=True):
    """Class plus public and private data attributes (trajectories, supervisors).
    Callables are skipped; nesting stops at depth 3."""
    if depth > 3:
        return f"<{type(obj).__name__}>"
    attrs = {k: jsonable(v, depth, tag_nonfinite) for k, v in sorted(vars(obj).items())
             if not callable(v)} if hasattr(obj, "__dict__") else repr(obj)
    return {"class": class_path(type(obj)), "attrs": attrs}


_SIMPLE = (bool, int, float, str, type(None), np.floating, np.integer)


def _scalars(obj):
    return {k: jsonable(v, tag_nonfinite=True) for k, v in sorted(vars(obj).items())
            if isinstance(v, _SIMPLE) or (isinstance(v, tuple) and all(isinstance(e, _SIMPLE) for e in v))}


def controller_options(ctrl):
    """Name plus every scalar/tuple option of a constructed controller (before
    reset), e.g. use_governor, alpha, fb_stale, derived gains K, wc, and the
    scalar parameters of each sub-object (governor, monitor, estimators,
    load model), e.g. gov.v_max."""
    return {"name": getattr(ctrl, "name", type(ctrl).__name__),
            "class": class_path(type(ctrl)), "options": _scalars(ctrl),
            "components": {k: {"class": class_path(type(v)), "options": _scalars(v)}
                           for k, v in sorted(vars(ctrl).items())
                           if hasattr(v, "__dict__") and not isinstance(v, type) and not callable(v)}}


def rebuild(desc):
    """Object from a describe() record: import the class, bypass __init__, set
    the recorded attributes (lists back to tuples only where the record came
    from tuples cannot be known, so trajectories keep lists - they only iterate).
    For request objects (Hold, RampedSine, MinJerkSequence) whose state is data."""
    import importlib
    mod, _, name = desc["class"].rpartition(".")
    cls = importlib.import_module(mod)
    for part in name.split("."):
        cls = getattr(cls, part)
    obj = cls.__new__(cls)
    for k, v in desc["attrs"].items():
        setattr(obj, k, _untag(v))
    return obj


def construct(desc):
    """Instance of a recorded controller/supervisor class built with its default
    constructor. The caller must compare its description with the record: a
    non-default option shows up as a mismatch instead of passing silently."""
    import importlib
    mod, _, name = desc["class"].rpartition(".")
    cls = importlib.import_module(mod)
    for part in name.split("."):
        cls = getattr(cls, part)
    return cls()


class NotRebuildable(RuntimeError):
    """A record names an object rebuild_run cannot re-create from its record."""


class UndeclaredSources(RuntimeError):
    """Project modules outside the declared closed source set were imported."""


def assert_declared(manifest):
    """Raise UndeclaredSources if the run imported project modules outside its
    declared source set (code.undeclared_imports). For evidence scripts that
    must not depend on undeclared code; run_id semantics are unchanged."""
    extra = manifest["code"].get("undeclared_imports") or []
    if extra:
        raise UndeclaredSources(f"imported project modules outside the declared source set: {extra}; "
                                "declare them in the entry module's PROVENANCE_SOURCES")


require_declared = assert_declared   # alias used by evidence scripts


def _apply(obj, opts):
    for k, v in opts.items():
        v = _untag(v)
        setattr(obj, k, _tuples(v) if isinstance(v, list) else v)   # options only record tuples
    return obj


def rebuild_run(run):
    """Everything needed to re-run a manifest's `run` section, from the record
    alone: scenario (config, requests, waypoints), controller and supervisor
    (default-constructed, then the recorded options applied), yaw coordination
    and seed. Returns (scenario, controller, supervisor, governed_yaw, seed);
    the caller re-describes them and must get the same run_id."""
    from types import SimpleNamespace
    sc = SimpleNamespace(name=run["scenario"], note=run.get("note", ""), cfg=config_from_dict(run["config"]),
                         roll=rebuild(run["roll_request"]), yaw=rebuild(run["yaw_request"]))
    for k in ("waypoints", "t_request", "waypoint_windows"):
        if k in run:
            setattr(sc, k, _untag(run[k]))
    rc = run["controller"]
    ctrl = _apply(construct(rc), rc["options"])
    for name, comp in rc["components"].items():
        obj = getattr(ctrl, name, None)
        if obj is None or class_path(type(obj)) != comp["class"]:
            # e.g. a load_model passed to the constructor: build the recorded class
            try:
                obj = construct(comp)
            except Exception as e:
                raise NotRebuildable(f"component {name} ({comp['class']}) cannot be default-constructed: "
                                     f"{type(e).__name__}: {e}") from e
            setattr(ctrl, name, obj)
        _apply(obj, comp["options"])
    sup = None
    if run.get("supervisor"):
        sup = _apply(construct(run["supervisor"]), run["supervisor"]["attrs"])
    return sc, ctrl, sup, run["governed_yaw"], run["seed"]


def _untag(v):
    if isinstance(v, dict):
        if set(v) == {"$float"}:
            return _NONFINITE[v["$float"]]
        return {k: _untag(e) for k, e in v.items()}
    if isinstance(v, list):
        return [_untag(e) for e in v]
    return v


def canonical(d):
    return json.dumps(d, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(d):
    return hashlib.sha256(canonical(d).encode()).hexdigest()


# -- configuration round trip ------------------------------------------------
_GROUPS = dict(plant=PlantConfig, timing=TimingConfig, drive=DriveConfig, sensor=SensorConfig)


def _tuples(v):
    v = _untag(v)
    return tuple(_tuples(e) for e in v) if isinstance(v, list) else v


def config_from_dict(d):
    """Inverse of jsonable(SimConfig, tag_nonfinite=True): lists back to the
    tuples the config uses, tagged non-finite floats back to inf/nan."""
    kw = {k: _untag(v) for k, v in d.items() if k not in _GROUPS}
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


def declared_sources(entry=None, root=ROOT):
    """The closed source set of a run (module docstring): all .py under ctrl/
    and sim/, EXP_CORE, the entry module (dotted name or path) and the exp
    modules it lists in PROVENANCE_SOURCES (repo-relative paths)."""
    import importlib
    root = Path(root).resolve()
    out = {p.resolve() for d in CLOSED_DIRS for p in (root / d).rglob("*.py") if "__pycache__" not in p.parts}
    out |= {(root / rel).resolve() for rel in EXP_CORE}
    if entry is not None:
        mod = importlib.import_module(entry) if isinstance(entry, str) else entry
        out.add(Path(mod.__file__).resolve())
        out |= {(root / rel).resolve() for rel in getattr(mod, "PROVENANCE_SOURCES", ())}
    return sorted(p for p in out if p.exists())


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
                  sources=None, extra=None, root=ROOT, entry=None, require_declared=False):
    """Manifest for one run. `controller` is the constructed controller instance
    (describe it before the run: reset() mutates it); `scenario` is a Scenario or
    FiniteMotion; t_window = (t_skip, t_end) used for the metrics; entry = the
    evaluation module (dotted name) whose source and PROVENANCE_SOURCES join
    the closed source set; require_declared=True raises UndeclaredSources if the
    process imported project modules outside that set."""
    cfg = scenario.cfg
    run = dict(scenario=scenario.name, note=getattr(scenario, "note", ""),
               config=jsonable(cfg, tag_nonfinite=True), roll_request=describe(scenario.roll),
               yaw_request=describe(scenario.yaw),
               governed_yaw=bool(governed_yaw), seed=int(seed),
               t_window=list(t_window) if t_window is not None else [0.0, float(cfg.duration)],
               controller=controller_options(controller),
               supervisor=describe(supervisor) if supervisor is not None else None)
    for k in ("waypoints", "t_request", "waypoint_windows"):
        if hasattr(scenario, k) and (k != "waypoint_windows" or scenario.waypoint_windows is not None):
            run[k] = jsonable(getattr(scenario, k))
    if extra:
        run["extra"] = jsonable(extra)
    paths = sources if sources is not None else declared_sources(entry, root)
    hashes = source_hashes(paths, root)
    rootr = Path(root).resolve()
    undeclared = sorted(str(p.relative_to(rootr)) for p in loaded_sources(root) if p not in set(map(Path, paths)))
    code = dict(sources=hashes, code_hash=code_hash(hashes), git=git_state(root, list(hashes)),
                source_set="declared: ctrl/**, sim/**, EXP_CORE, entry + PROVENANCE_SOURCES",
                entry=entry if isinstance(entry, str) or entry is None else entry.__name__,
                undeclared_imports=undeclared)
    ident = dict(run={k: v for k, v in run.items() if k != "note"}, code_hash=code["code_hash"],
                 metrics_version=SM.METRICS_VERSION)
    if require_declared:
        assert_declared(dict(code=code))
    return dict(manifest_version=MANIFEST_VERSION, run_id=digest(ident)[:16],
                metrics_version=SM.METRICS_VERSION, metric_defs=SM.METRIC_DEFS,
                run=run, code=code, env=environment())


def write_json(obj, path):
    """Strict JSON; non-finite floats tagged as {"$float": ...} (see jsonable)."""
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

    Phase 1 (evaluation): the body and check(stage) run; any exception leaves the
    target untouched and keeps the stage with a FAILED note.
    Phase 2 (publish): every staged file is first copied beside its destination
    (.name.publishing); then each existing destination is moved aside
    (.name.prev) and the copy renamed over it. If anything in phase 2 raises,
    already-replaced files are restored from .prev, new files are removed, all
    temporaries are deleted, and FAILED records the error: the target returns to
    its previous state. Only a crash of the process itself (power loss, kill)
    during phase 2 can leave a mix; orphaned .publishing/.prev files from such a
    crash are handled at the start of the next phase 2: a .X.prev whose X is
    missing is the only surviving copy and is renamed back to X; other orphans
    are left alone unless this publish replaces X (then they are overwritten
    and removed). Files in the target that the stage does not contain are left
    alone.
    """
    target = Path(target)
    root = Path(staging_root or target.parent)
    root.mkdir(parents=True, exist_ok=True)
    stage = Path(tempfile.mkdtemp(prefix=".stage-", dir=root))
    try:
        yield stage
        if check is not None:
            check(stage)
    except BaseException:
        (stage / "FAILED").write_text("phase 1 (evaluation/check) failed; target untouched\n"
                                      + traceback.format_exc())
        raise
    files = [p for p in sorted(stage.rglob("*")) if p.is_file()]
    if target.exists():
        for prev in sorted(target.rglob(".*.prev")):          # crash leftovers: restore lost originals
            orig = prev.with_name(prev.name[1:-len(".prev")])
            if not orig.exists():
                os.replace(prev, orig)
    tmp, moved, created = [], [], []
    try:
        for p in files:
            dst = target / p.relative_to(stage)
            dst.parent.mkdir(parents=True, exist_ok=True)
            t = dst.with_name(f".{dst.name}.publishing")
            shutil.copy2(p, t)
            tmp.append((t, dst))
        for t, dst in tmp:
            if dst.exists():
                prev = dst.with_name(f".{dst.name}.prev")
                os.replace(dst, prev)
                moved.append((prev, dst))
            else:
                created.append(dst)
            os.replace(t, dst)
    except BaseException:
        errors = []
        for dst in created:
            try:
                dst.unlink(missing_ok=True)
            except OSError as e:
                errors.append(f"remove {dst}: {e}")
        for prev, dst in moved:
            try:
                os.replace(prev, dst)
            except OSError as e:
                errors.append(f"restore {dst}: {e}")
        for t, _ in tmp:
            t.unlink(missing_ok=True)
        (stage / "FAILED").write_text("phase 2 (publish) failed; target rolled back"
                                      + (" INCOMPLETELY:\n" + "\n".join(errors) if errors else "") + "\n"
                                      + traceback.format_exc())
        raise
    for prev, _ in moved:
        prev.unlink(missing_ok=True)
    shutil.rmtree(stage)
