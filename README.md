# Safe Control for a Coupled Two-Axis Robot

A roll-axis simulator, diagnosis, and controller design for the Robotics Controls take-home.

## Reproduce
```bash
python3 -m pip install -r requirements.txt   # numpy, scipy, matplotlib
python3 run_all.py                            # tests + all analyses
```
Tested with Python 3.9, numpy 2.0, scipy 1.13, matplotlib 3.9. There are no other dependencies. The tests use the standard-library `unittest`.

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
- ◐ Corrective packets R1–R4 done ([report/packets](report/packets)); final review (R6) pending
- ☐ Four-page memo, one-page hardware qualification plan, references note
