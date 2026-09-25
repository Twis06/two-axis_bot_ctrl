# Safe Control for a Coupled Two-Axis Robot

A roll-axis simulator, diagnosis, and controller design for the Robotics Controls take-home.

## Reproduce

With [uv](https://docs.astral.sh/uv/) (recommended; installs the exact locked versions and Python 3.9):

```bash
uv run python run_all.py            # tests + every current evidence packet (~12 min on a 12-core Mac, ~8 min on 112 cores)
```

Without uv, in a Python 3.9 virtual environment:

```bash
python3 -m pip install -r requirements.txt   # exact pins exported from uv.lock
python3 run_all.py
```

- **Locked versions:** Python 3.9, NumPy 2.0.2, SciPy 1.13.1, Matplotlib 3.9.4 (`pyproject.toml`, `uv.lock`). The published evidence was produced with these, and other versions may change the last floating-point digits.
- **Log:** `run_all.py` logs each step, its exit code and the environment to `report/reproduction_log.txt`.
- **Parallelism:** experiments run in a process pool sized to the machine; results do not depend on the pool size.
- **`--quick`:** runs the tests and calculations only, and does not reproduce the evidence.
- **Report documents:** `uv run --group report python report/html_assets/build_report.py` rebuilds the HTML report. `report/html_assets/render_pdf.py` renders the memo PDFs and also needs Google Chrome.

## Layout
| Path | Contents |
|---|---|
| `sim/` | plant, current and voltage model, CAN latency, encoder, drive firmware, engine, metrics |
| `ctrl/` | `baseline.py` (PI×lead + feed-forward), `governor.py` (reshape/derate/reject), `supervisor.py` (drive-side faults, thermal), `loopshape.py` (gain derivation), `legacy.py` (reconstructed "before" controller) |
| `analysis/phase0.py` | closed-form torque, delay, bandwidth, voltage and thermal budgets |
| `exp/` | scenarios for Runs A–E; experiment scripts |
| `tests/` | simulator validation and controller safety/regression tests |
| `report/` | analyses (`phase*_*.md`), generated numbers, figures |

## Assessment answers

Start with [Task 1 — Understand the failure](report/task1.md). The [task index](report/README.md) maps Tasks 1–5 to the current answers, supporting evidence, and unfinished work. The phase reports retain the development history.

## Status
- ✅ Phase 0: paper analysis
- ✅ Phase 1: simulator + consistency with the logs (a fit, not identification)
- ✅ Task 2: deterministic baseline frozen (fingerprint `7d857df507c389c9`)
- ✅ Task 3: the tested adaptive candidate fails its registered adoption gate (benefit, availability); baseline retained. The payload-change challenge was not executed.
- ◐ Task 5: registered prediction tested in simulation and retrospectively corrected (R2); the closed-loop prediction is untested, and hardware confirmation is pending
- ◐ Corrective packets R1–R5 done ([report/packets](report/packets)); independent final review (R6) pending
- ✅ Submission documents: [memo](report/memo.pdf), [hardware qualification plan](report/hardware_qualification_plan.pdf), [references and tools](report/references_and_tools.md)
