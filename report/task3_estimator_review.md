# Task 3 L1 independent review

## Critical — duplicate or receipt-out-of-order fault status can leave correction live

`ctrl/payload_estimator.py:247-268` returns for a duplicate physical timestamp
(and `:252-254` for an out-of-order receipt) before it evaluates positive
current limit, drive mode, or saturation. This contradicts the fail-closed
health contract in `docs/plans/task3-learning.md:87-93`: a received fault,
saturation, or invalid input must disable correction and invalidate pending
work even when the packet is ineligible for training.

Minimal focused reproducer: after learning the supplied synthetic model, a
pending fit was submitted at `t=3.337500` and ready at `3.344805`. Supplying a
fresh-arrival duplicate (`t_meas=3.337500`, `t_received=3.340500`) with
`mode=1` returned `False`, but the snapshot remained `(usable=True,
pending=True, health_generation=0)`. Ticking at `3.344905` completed the
pre-fault job and remained usable. The same result occurred for
`saturated=True` and `i_limit=0` duplicates.

Expected behavior is immediate zero correction, cleared pending work, and a
new health generation. The chronology policy should be conservative: hard
status/validity failures take precedence over deduplication; otherwise a
duplicate must remain ignored and never refresh health. Add public behavior
tests for duplicate and receipt-out-of-order fault/saturation/invalid-limit
packets while a fit is pending. The execution report's statement that fault
and saturation packets always disable application
(`report/task3_estimator_report.md:30-33`) is therefore inaccurate.

## Important — raw dwell storage and ingress work are unbounded

`ctrl/payload_estimator.py:191` creates an unbounded dwell deque. The only
retention rule is its 250 ms time span (`:373-390`), while every observation
also materializes and scans the whole deque (`:378-384`); aggregate creation
scans it again (`:475-493`). The public API has no maximum observation rate.

As a focused probe, 3,000 valid stationary observations at 1 microsecond
physical intervals (all within 3 ms and each received 2 ms later) were all
accepted. Before any aggregate was due, the raw dwell held 3,000 samples and
the 3,000 `observe` calls took 0.272 s on this host. The exact runtime is not
a portability claim; the cardinality and per-call work grow without a bound as
the input rate increases.

This conflicts with the review brief's bounded-history/work requirement. The
fixed plan caps the *aggregate* buffer at 600
(`docs/plans/task3-learning.md:72-73`) but does not specify a raw-dwell
cardinality or an assumed maximum feedback rate, so the exact capacity is a
plan gap as well as an implementation defect. Freeze a raw-sample cap or an
over-rate abstention/reset policy, then add a test that feeds more than that
cap within one dwell window and proves bounded storage and bounded per-call
work.

## Important — a rejected nonfinite packet does not reserve its timestamp

At `ctrl/payload_estimator.py:232-258`, all fields are checked for finiteness
before `_last_seen_measurement_time` is updated. A nonfinite packet therefore
invalidates the estimator but leaves its finite `t_meas` eligible for a later
replacement packet. This violates the strict physical-timestamp rule in
`docs/plans/task3-learning.md:67` and permits an invalid physical sample to be
turned into recovery/training input.

Focused sequence: accept `t_meas=0.000`; reject a packet at `t_meas=0.005`
with `q=NaN`; then send a finite packet with the same `t_meas=0.005`. The
replacement was accepted and, after otherwise healthy samples through
`t_meas=0.260`, contributed an aggregate (`accepted_sample_count=1`).

Reserve a finite, temporally admissible physical timestamp before rejecting
its non-status payload fields, while retaining the Critical finding's rule
that observed hard status still invalidates immediately. Add a regression test
showing that nonfinite/future/stale packet timestamps cannot later be replayed
as valid input. If corrected retransmissions are meant to be allowed, that
would require an explicit change to the current strict-timestamp contract.

## Checks performed during the initial review

- Ran `uv run --quiet --with numpy python -m unittest
  tests.test_payload_estimator -v`: 12 tests passed.
- Confirmed the report's implementation and test SHA-256 values match the
  current sources.
- Inspected the measured-current target, received-before-submission snapshot,
  worker generation/late-result checks, post-projection residual check,
  freshness/estimate-age gates, recovery ramp reset, and output cap. Those
  paths appear aligned with the L1 plan on inspection.

At that review point, no broad simulation or repository-wide suite was rerun.
The supplied initial tests did not cover the three counterexamples; their
fixed 5 ms feed helper also could not expose unbounded raw-dwell behavior.

## Resolution record

The three findings above were fixed in the authorized L1 files and covered by
new regression tests:

1. Hard mode, saturation, and nonpositive-limit failures are evaluated before
   duplicate/out-of-order returns, so a repeated bad-status packet clears
   application and pending work.
2. Raw dwell storage is capped at 256 samples. Overflow enters
   `input_overload`, clears the dwell, and requires normal recovery.
3. A finite, temporally admissible physical timestamp is reserved before a
   nonfinite payload is rejected, so a later finite replay cannot become a new
   training sample.

Fresh verification after the fixes: `python3 -m unittest
tests.test_payload_estimator -v` ran 15 tests with 0 failures; the full
repository suite ran 196 tests with 0 failures. The subsequent L2 audit ran 60
500 Hz seeded cases and reported zero structural violations. These results
close the review findings for L1; they do not constitute a closed-loop benefit
claim.
