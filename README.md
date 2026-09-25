# Safe Control for a Coupled Two-Axis Robot

A roll-axis simulator, failure diagnosis, deterministic baseline controller and evidence packets for the Robotics Controls take-home assessment.

**Recommendation:** take the frozen deterministic baseline (fingerprint `7d857df507c389c9`) to hardware qualification. Do not deploy the tested adaptive load correction: it failed its pre-registered adoption gate. All results are simulations or calculations; nothing has been run on hardware.

## Submission deliverables

| # | Required by the brief | Deliverable | Notes |
|---|---|---|---|
| 1 | Concise technical memo, at most 4 pages excluding plots and appendices | [`report/memo.pdf`](report/memo.pdf) (source [`report/memo.md`](report/memo.md)) | The body is exactly 4 pages. The plot appendix (Figures 1–10) follows from page 5. |
| 2 | Runnable code with a short README and one-command reproduction | This repository; run `uv run python run_all.py` | See [Reproduce](#reproduce). Code is in `sim/`, `ctrl/` and `exp/`. |
| 3 | Plots showing saturation, phase lag, fallback behavior and generalization, not only aggregate reward | Memo appendix, Figures 1–10; image files in [`report/figs/`](report/figs/) | Mapped plot by plot [below](#plots-required-by-the-brief). |
| 4 | One-page hardware qualification plan: instrumentation, test order, stop conditions, acceptance criteria | [`report/hardware_qualification_plan.pdf`](report/hardware_qualification_plan.pdf) (source [`.md`](report/hardware_qualification_plan.md)) | One page. It covers instrumentation, 7 ordered stages, global and stage-specific stops, and acceptance criteria. |
| 5 | Short note of outside references, reused code and automated tools | [`report/references_and_tools.md`](report/references_and_tools.md) | No outside references cited and no copied code. It lists the libraries and discloses the AI coding agents used. |

**Supporting material:**

- **Longer answers:** the full answer to each task is in [`report/task1.md`](report/task1.md) … [`report/task5.md`](report/task5.md), indexed in [`report/README.md`](report/README.md).
- **HTML report:** a self-contained interactive report, [`report/assessment_report.html`](report/assessment_report.html). Download it and open it in a browser; it needs no network access.

### Plots required by the brief

All figures are simulated unless marked Calculated.

| Requirement | Memo figure | File |
|---|---|---|
| **Saturation** | Fig. 2: saturation and anti-windup (with the governor-disabled diagnostic) | [`task2_saturation.png`](report/figs/task2_saturation.png); saturation duration and entries: [`task4b_saturation.png`](report/figs/task4b_saturation.png) |
| **Phase lag** | Fig. 3: loop frequency response and margins | [`task4b_frequency.png`](report/figs/task4b_frequency.png) |
| | Fig. 4: delay robustness | [`task2_delay_robustness.png`](report/figs/task2_delay_robustness.png) |
| | Fig. 10: prospective Task 5 test (measured phase and gain change with added delay) | [`task5_prospective.png`](report/figs/task5_prospective.png) |
| **Fallback behavior** | Fig. 5: feedback loss and recovery | [`task2_feedback_loss.png`](report/figs/task2_feedback_loss.png) |
| | Fig. 6: fault, catch and suspension timeline | [`task4b_fault_timeline.png`](report/figs/task4b_fault_timeline.png) |
| **Generalization** | Fig. 7: sampled plants and chosen stress cases | [`task4b_generalization.png`](report/figs/task4b_generalization.png) |
| **Also shown** | Fig. 1: tracking and delivered motion against original and governed references | [`task2_tracking.png`](report/figs/task2_tracking.png) |
| | Fig. 8: static holdability (Calculated) | [`static_holdability.png`](report/figs/static_holdability.png) |
| | Fig. 9: a request that should not be tracked | [`task4b_infeasible.png`](report/figs/task4b_infeasible.png) |

The `p0_*`, `p1_*` and `p2_*` figures are historical (early analysis and the reconstructed legacy controller). They appear in the HTML report's history section, not as current results.

## Reproduce

With [uv](https://docs.astral.sh/uv/) (recommended; it installs Python 3.9 and the exact locked versions):

```bash
uv run python run_all.py      # all tests + every evidence packet: about 10 min on a 12-core laptop
```

Without uv, in a Python 3.9 virtual environment:

```bash
python3 -m pip install -r requirements.txt   # exact pins exported from uv.lock
python3 run_all.py
```

- **What it runs:** 278 unit tests, then every analysis and experiment, regenerating all numbers, JSON evidence and figures in `report/`. Each step, its exit code and the environment are logged to `report/reproduction_log.txt`.
- **Versions:** Python 3.9, NumPy 2.0.2, SciPy 1.13.1 and Matplotlib 3.9.4, from `pyproject.toml` and `uv.lock`. Other versions may change the last floating-point digits.
- **Checking a reproduction against the published evidence:**
  1. Reproduce in a separate copy: `git archive HEAD | tar -x -C /tmp/copy`, then `uv run python run_all.py` inside it.
  2. From this checkout, run `uv run python tools/compare_evidence.py /tmp/copy/report`.
  3. On macOS arm64, expect exact agreement. On x86-64, expect values within about 1e-11 and different Monte Carlo run IDs, because NumPy rounds random draws differently across CPUs ([`report/packets/R4.md`](report/packets/R4.md)).
- **Rebuilding the documents:**
  - HTML report: `uv run --group report python report/html_assets/build_report.py`.
  - PDFs: `uv run --group report python report/html_assets/render_pdf.py report/memo.md report/memo.pdf`, and the same for the plan. This also needs Google Chrome.
- **`--quick`:** runs only the tests and calculations, and does not reproduce the evidence.

## Repository layout

| Path | Contents |
|---|---|
| `sim/` | Plant, current and voltage model, CAN latency, encoder, drive firmware, engine, metrics |
| `ctrl/` | `baseline.py` (PI × lead + feed-forward), `governor.py` (reshape, derate, reject), `supervisor.py` (drive-side faults, thermal), `loopshape.py` (gain derivation), `legacy.py` (reconstructed "before" controller), `adaptive.py` and `payload_estimator.py` (the rejected learning candidate) |
| `analysis/phase0.py` | Closed-form torque, delay, bandwidth, voltage and thermal budgets |
| `exp/` | Scenarios for Runs A–E and the experiment scripts behind every evidence packet |
| `tests/` | Simulator validation, controller safety and regression tests, gate and scoring tests |
| `tools/compare_evidence.py` | Compares a reproduced `report/` with the published evidence |
| `report/` | Deliverables, task answers, generated numbers, JSON evidence and run manifests, figures, HTML report |
| `report/packets/` | Change, review and verification records (2A–4B, corrective R1–R6, Task 5 prospective review) |
| `docs/plans/` | Protocols and registered plans (e.g. the [Task 5 prospective protocol](docs/plans/task5-prospective-protocol.md)) |
| `docs/process/` | Development and execution plans and the planner review record (process history, not deliverables) |
| `docs/COMMIT_MAP.md` | Maps commit IDs cited in older provenance to the current history |

## Status and declared limitations

- **Task 1:** diagnosis from the five run summaries, with observations, calculations and hypotheses kept separate.
- **Task 2:** deterministic baseline frozen (fingerprint `7d857df507c389c9`), with governor, fault-class recovery and causal yaw compensation.
- **Task 3:** learning was evaluated and rejected. The tested adaptive candidate fails its registered gate on benefit (0.0% median) and availability (74%). No safety criterion failed. The payload-change challenge was not executed.
- **Task 4:** the simulator and matched evidence reproduce with one command, verified on macOS arm64 (exact) and Linux x86-64 (within 3e-11).
- **Task 5:** a numerical closed-loop prediction was committed before the test and is **supported**: command delay 1 → 5 ms at 3 Hz. The earlier motor-strength study is kept as a qualified secondary result.
- **Reviews:** independent review rounds and their resolutions are in [`report/packets/R6.md`](report/packets/R6.md) and [`report/packets/task5-prospective-review.md`](report/packets/task5-prospective-review.md).
- **Not done:** hardware qualification is proposed, not performed. One exploratory availability replay is unscripted and is labelled as such.
