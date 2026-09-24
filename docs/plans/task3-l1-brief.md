# L1 brief — standalone payload estimator

Read `docs/plans/task3-learning.md` sections 3–5 and 10 as the requirements. These sections are the single source for numerical settings, API, safety/causality behavior and tests. Read section 1 for the scientific question; later integration/comparison packets are not your assignment.

Only create/edit:
- `ctrl/payload_estimator.py`
- `tests/test_payload_estimator.py`
- `report/task3_estimator_report.md`

The baseline is under concurrent work. Do not modify or integrate with it. No governor, plant, dependency, existing test, report index or Task 3 answer changes. No subagents. No broad simulator performance experiments. No claim that learning improves closed-loop tracking.

Use test-first development. Keep the code small, explicit and bounded; the 2x2 fit needs no ML framework. All estimator observations must be causal, with measurement age separate from receipt and publication age. Nonstationary healthy input may preserve application but not create training samples. Fault/invalid/stale/saturation input invalidates application and pending worker results.

Before coding, resolve any ambiguous requirement in your report with a conservative choice; ask the planner if the ambiguity affects the scientific comparison or safety contract. Do not broaden the statistical validity claim beyond the plan.

Report exact changes, tests/commands/results, limitations, and source hashes of files you wrote. Use `uv run --quiet --with numpy python -m unittest tests.test_payload_estimator -v` where normal Python lacks dependencies; also run `uv run --quiet --with numpy python -m unittest discover -s tests -v`. Baseline failures from concurrent work must be reported separately, not repaired here. This folder has no Git repository: do not initialize one or claim commits.

Return a short status with the report path, test summary and concerns. The full record belongs in `report/task3_estimator_report.md`.
