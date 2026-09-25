# Task 5 prospective test — reviews, reproduction and release scope

**Scope.** The prospective closed-loop test that closes the brief's Task 5 requirement: a numerical prediction written and committed before a new test. It covers the protocol, registration and results, their integration into the submission, and the release checks. Plan: [`docs/plans/2026-09-25-submission-readiness.md`](../../docs/plans/2026-09-25-submission-readiness.md). Protocol: [`docs/plans/task5-prospective-protocol.md`](../../docs/plans/task5-prospective-protocol.md).

## Chronology (git, 2026-09-25)

| Commit | Time | Content |
|---|---|---|
| `c0b065f` | 10:20 | Registration v1: predictor, scorer, synthetic tests, `task5_prospective_registration.json`; no results |
| `7452ea8` | 10:28 | Registration v2 after the pre-run review (below); central prediction unchanged |
| `62a9ac6` | 10:29 | Declare `exp/evidence.py` for manifests. The first execution attempt at `7452ea8` stopped in `make_manifest` before any simulation |
| `1101240` | 10:30 | Results of the ten registered runs, executed 10:29:49–10:30:09 from `62a9ac6` |
| `82ceda0` → release | — | Integration (overview, memo, HTML, status), republication, rebuilds and review fixes |

- **No early results:** `git log --all` shows no result artifact for this condition before `1101240`.
- **Execution commit:** the runs were executed from `62a9ac6`, as recorded in `1101240`'s run manifests. Later republications record the current commit instead. The results JSON does not carry start/end times; the times above come from the execution log of this session and the commit timestamps.

## Pre-run independent review (before any run)

- **Verdict on v1 (`c0b065f`): DO NOT RUN.** One blocking defect: the scorer counted drive fallback over the whole log. The drive's normal start-up ticks would have made every run inconclusive.
- **Fixed in v2 (`7452ea8`):**
  - fallback is counted inside the scoring window, with a test;
  - friction uncertainty is plant-only;
  - the registration binds `scorer_sha256`.
- **Verdict on v2:** **CLEAR TO RUN.**
- **Confirmed by the reviewer:**
  - prediction independence (configuration-derived timing only);
  - the model and discrete controller;
  - the phasor convention;
  - discrimination (|ΔH| = 2.6 × radius; the "feed-forward lag only" alternative lies far outside).
- **Disclosures raised by the reviewer**, recorded in the protocol:
  - the absolute |H| from the model may be about 0.02 low;
  - the 0.5 ms reference-sampling advance;
  - the 0.010 floor is a judgment;
  - governor limiting and clipping are recorded but are not outcome rules.

## Result

The measured mean ΔH is +0.0698 − 0.0238j against the predicted +0.0667 − 0.0199j, a distance of 0.0050 inside the radius 0.0264. **Supported.**

- **Run quality:** all ten runs are valid, with no fault events, and no fallback, clipping or governor limiting inside the window.
- **Consistency:** the pairs agree within about 0.0002.
- **Robustness to the pre-run revision:** the outcome would be the same under v1's radius (0.0217), and even under the 0.010 floor alone.

## Final independent review (release candidate `a377d89`): PASS WITH ISSUES

**No Critical or Important findings.**

**Independently verified:**

- **Order:** the registration precedes the results, and v1, v2 and HEAD carry an identical central prediction.
- **Re-execution:** the module re-executed into a scratch directory gives 0 of 209 floats different, identical run_ids and a byte-identical figure. The same holds from a clean archive.
- **Scoring:** H for seed 303 was recomputed with an independent DFT and matches to 2e-16.
- **Outcome:** it follows the registered rule exactly.
- **Builds:** 278 tests pass; the memo body is 4 pages; the plan is 1 page.

**Its 13 minor findings, and what was done:**

| # | Finding | Resolution |
|---|---|---|
| 1 | Stale test count (266) | 278 in the memo, Task 4, HTML and backlog |
| 2 | Stale "no prospective prediction" statements (plan status row, HTML, backlog) | Updated |
| 3 | "confirmed / holds quantitatively" without scope | "supported quantitatively at this one condition (3 Hz, +4 ms, simulated)"; other conditions untested |
| 4 | "ΔH is unaffected" by the absolute-gain offset | "moves ΔH only by about 0.002 per ms, below the radius" |
| 5 | "no fallback" was unscoped | "no fallback … inside the scoring window" |
| 6 | Overview prose was hard-coded to the outcome | `exp/task5_overview.py` now refuses to write it unless the outcome is *supported* with no problems |
| 7 | Protocol still said "not yet run" | Dated post-run pointer added; registered content unchanged |
| 8 | This review packet was missing | This file |
| 9 | Report index listed only the historical Task 5 record | Prospective protocol, registration, results, runs, numbers and this packet added |
| 10 | Hardware confirmation not in the qualification plan | Added to stage 4 (plan still one page); HTML table in step |
| 11 | Figure 10's five pair markers are hidden under the mean | Caption says so |
| 12 | Execution commit and times not in the result packet | Disclosed above (chronology table) |
| 13 | Configuration timing (4.28/1.78 ms) is close to 4B's log-measured values, which were available | Disclosed in the protocol's prior knowledge. ΔH depends on the exact 4 ms FIFO change |

## Reproduction of `a377d89`

Both checks used `tools/compare_evidence.py`.

- **Linux x86-64 (luna1), `git archive` + `uv sync --frozen`:** 13/13 steps.
  - The prospective outcome is **supported**, with the same ΔH to about 1e-11 and identical run_ids.
  - Across all packets: 5,541 of 46,489 floats differ in the last digits (worst 3.1e-11), 0 outside tolerance, and the only differing run_ids are the known Monte Carlo ones (70 in Task 2, 30 in 4B).
- **macOS arm64, fresh `git archive` + `uv sync --frozen`:** recorded in [R4](R4.md).

## Release scope

The release commit differs from the reviewed candidate `a377d89` only in the documentation and generator-text fixes listed above:

- `exp/task5_overview.py` (text and the outcome guard);
- the memo, HTML source, plan, indexes and protocol pointer.

The rebuilt HTML and PDFs follow from those. No experiment source, registration or result changed.
