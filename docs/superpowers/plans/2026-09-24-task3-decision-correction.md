# Task 3 Decision Correction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Correct Task 3 documentation so it reflects the evidence at each gate and ends with a scoped keep/drop decision after the matched comparison.

**Architecture:** Keep the deterministic baseline as the current submitted controller. Preserve the accepted L1/L2 estimator evidence and the known-load ceiling as separate evidence layers. Record L3/L4 adaptive integration and matched comparison as pending because no validated adaptive held-out result exists yet.

**Tech Stack:** Markdown reports, JSON run manifests, Python unittest verification.

**Spec:** `docs/plans/task3-learning.md` sections 2, 6–9.

## Global Constraints

- L2 establishes estimator behavior only; it does not establish closed-loop benefit.
- The diagnostic oracle may access truth only as a ceiling, never as deployable controller evidence.
- Adoption requires the preregistered paired improvement, waypoint, and safety gates.
- Do not claim an adaptive result until an independently reviewed L3 adapter and matched L4 run exist.
- Preserve the frozen deterministic baseline fingerprint `7d857df507c389c9`.

## Review Focus

- Distinguish “baseline retained for this submission” from “learning rejected”; verify every index and plan status uses the same wording.
- Preserve the oracle result as evidence of potential headroom without presenting it as deployable performance.
- Keep the physical-capacity case scoped to infeasible loads; do not use it to reject feasible-load adaptation.
- Identify the new L3 adapter as unverified unless its tests and review are explicitly recorded.
- Ensure no document says L4 was completed when the adaptive candidate is absent from the run manifest.

### Task 1: Correct the decision records

**Files:**
- Modify: `report/task3.md`
- Modify: `docs/plans/task3-execution-log.md`
- Modify: `EXECUTION_PLAN.md`
- Modify: `report/README.md`
- Modify: `report/task4.md`

**Interfaces:**
- Consumes: `report/task3_ceiling_numbers.md`, `report/task3_estimator_audit.md`, and `docs/plans/task3-learning.md`.
- Produces: consistent conditional Task 3 status and an explicit L3/L4 evidence gap.

- [x] Replace premature final-rejection language with conditional non-adoption language while the matched comparison is pending.
- [x] Cite the 76.9% oracle feed-forward-only median reduction and 50/50 completion result.
- [x] State that the 0.41 N·m infeasible case is a capacity boundary, not a general efficacy result.
- [x] Mark L3/L4 as pending and preserve the baseline as the current controller.

### Task 2: Verify the correction

**Files:**
- Test: `report/task3.md`, `report/task3_ceiling_numbers.md`, `EXECUTION_PLAN.md`, and the changed index/log documents.

- [x] Search for stale claims that Task 3 is finally rejected or that the adaptive comparison was completed.
- [x] Run `git diff --check` on the affected files; unrelated pre-existing whitespace remains in `note.md`.
- [x] Confirm no source or run artifact was changed.

## Follow-up outcome

The subsequent registered L3/L4 run resolved the pending state: the exact adaptive candidate produced 0.0% median paired improvement and 37/50 usable held-out estimates, so the scoped rejection is now evidence-based. The oracle ceiling remains positive and is retained as the trigger for any future redesign.
