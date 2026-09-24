"""Behavioral tests for the bounded Task 3 payload-estimator kernel."""

from dataclasses import FrozenInstanceError
import math
import unittest

from ctrl.payload_estimator import PayloadEstimator, PayloadObservation


COUNT = 2.0 * math.pi / 16384.0
NOMINAL_GRAVITY = 0.120
CURRENT_TO_TORQUE = 0.140


class PayloadEstimatorTests(unittest.TestCase):
    """Tests use only observable estimator behavior and synthetic sensor packets."""

    def _observation(self, t_meas, q, qy=0.0, coefficients=(0.08, 0.16),
                     i_limit=3.2, mode=0, saturated=False, receipt_delay=0.002,
                     current=None):
        if current is None:
            theta_s, theta_c = coefficients
            extra = theta_s * math.sin(q) + theta_c * math.cos(q)
            current = (NOMINAL_GRAVITY * math.sin(q) + extra) / CURRENT_TO_TORQUE
        return PayloadObservation(
            t_meas=t_meas,
            t_received=t_meas + receipt_delay,
            q=q,
            qy=qy,
            i_meas=current,
            i_limit=i_limit,
            mode=mode,
            saturated=saturated,
        )

    def _feed(self, estimator, t_start, duration, q, qy=0.0,
              coefficients=(0.08, 0.16), i_limit=3.2, mode=0,
              saturated=False, current=None, dt=0.005):
        """Feed fresh packets and tick at their receipt time; return next time/now."""
        steps = int(math.ceil(duration / dt))
        now = t_start
        for index in range(steps):
            t_meas = t_start + index * dt
            roll = q(t_meas, index) if callable(q) else q
            yaw = qy(t_meas, index) if callable(qy) else qy
            if callable(current):
                i_meas = current(t_meas, roll, yaw, index)
            else:
                i_meas = current
            observation = self._observation(
                t_meas, roll, yaw, coefficients=coefficients, i_limit=i_limit,
                mode=mode, saturated=saturated, current=i_meas,
            )
            estimator.observe(observation)
            now = observation.t_received
            estimator.tick(now)
        return t_start + steps * dt, now

    def _learn(self, coefficients=(0.08, 0.16), i_limit=3.2):
        """Provide informative settled dwells, then enough fresh time for the ramp."""
        estimator = PayloadEstimator(seed=7)
        t = 0.0
        t, _ = self._feed(estimator, t, 0.34, math.radians(-50),
                          coefficients=coefficients, i_limit=i_limit)
        t, _ = self._feed(estimator, t, 0.34, 0.0, coefficients=coefficients,
                          i_limit=i_limit)
        # The first usable full fit appears after the third dwell.  Keep fresh
        # packets flowing for the required 0.10 N m/s publication ramp as well.
        t, now = self._feed(estimator, t, 2.65, math.radians(50),
                            coefficients=coefficients, i_limit=i_limit)
        # Flush the final modeled result with one new, still-settled packet.
        # Its receipt is 8.5 ms after the prior tick (beyond the worker's 8 ms
        # maximum) but below the 15 ms physical-gap and freshness limits.  No
        # second aggregate may be scheduled because learning cadence is 20 ms.
        flush_t = t + 0.0035
        flush = self._observation(flush_t, math.radians(50), coefficients=coefficients,
                                  i_limit=i_limit)
        estimator.observe(flush)
        now = flush.t_received
        estimator.tick(now)
        # Follow-up packets must be causally after the final worker-poll time.
        return estimator, flush_t + 0.001, now

    def _advance_until_pending(self, estimator, t_start,
                               coefficients=(0.08, 0.16)):
        """Find a real pending modeled job without reaching into estimator state."""
        t = t_start
        for _ in range(80):
            observation = self._observation(t, math.radians(50),
                                            coefficients=coefficients)
            estimator.observe(observation)
            estimator.tick(observation.t_received)
            snapshot = estimator.snapshot(observation.t_received)
            if snapshot.pending:
                return t, observation.t_received, snapshot
            t += 0.001
        self.fail("estimator did not schedule a pending job from fresh dwell data")

    def test_observation_is_immutable(self):
        """Changing sensor values after receipt must not change estimator input."""
        observation = self._observation(0.0, 0.0)
        with self.assertRaises(FrozenInstanceError):
            observation.q = 1.0

    def test_settled_three_angle_data_converges_after_worker_and_publication_ramp(self):
        """A correct two-term settled residual reaches the known coefficients."""
        estimator, _, now = self._learn()
        snapshot = estimator.snapshot(now)

        self.assertTrue(snapshot.usable, snapshot)
        self.assertAlmostEqual(snapshot.theta_s, 0.08, delta=0.01)
        self.assertAlmostEqual(snapshot.theta_c, 0.16, delta=0.01)
        self.assertAlmostEqual(
            estimator.correction(math.radians(20), now),
            0.08 * math.sin(math.radians(20)) + 0.16 * math.cos(math.radians(20)),
            delta=0.015,
        )

    def test_single_angle_hold_abstains_even_after_many_aggregates(self):
        """Removing angular coverage must make a two-parameter fit unusable."""
        estimator = PayloadEstimator(seed=2)
        _, now = self._feed(estimator, 0.0, 4.0, 0.0)
        snapshot = estimator.snapshot(now)

        self.assertGreaterEqual(snapshot.accepted_sample_count, 30)
        self.assertFalse(snapshot.usable, snapshot)
        self.assertEqual(estimator.correction(0.0, now), 0.0)
        self.assertLess(snapshot.roll_span, math.radians(60))

    def test_motion_is_not_training_but_two_count_quantized_dwell_jitter_is(self):
        """Motion must not leak dynamic residuals into dwell aggregates."""
        estimator = PayloadEstimator(seed=3)
        t, now = self._feed(
            estimator, 0.0, 0.6,
            lambda current_t, _: -0.4 + 0.18 * current_t,
            qy=lambda current_t, _: 0.12 * current_t,
        )
        moving = estimator.snapshot(now)
        self.assertEqual(moving.accepted_sample_count, 0)

        t, now = self._feed(
            estimator, t, 0.32,
            lambda _, index: 0.20 + (-COUNT if index % 3 == 0 else
                                      COUNT if index % 3 == 1 else 0.0),
            qy=lambda _, index: -0.10 + (-COUNT if index % 3 == 0 else
                                          COUNT if index % 3 == 1 else 0.0),
        )
        jittered = estimator.snapshot(now)
        self.assertGreaterEqual(jittered.accepted_sample_count, 1)
        self.assertFalse(jittered.usable, "one stationary angle remains unidentifiable")

    def test_duplicate_old_future_and_nonfinite_packets_neither_train_nor_refresh_health(self):
        """Rejected packets cannot become a new causal/healthy sensor sample."""
        estimator = PayloadEstimator(seed=4)
        _, now = self._feed(estimator, 0.0, 0.32, 0.0)
        before = estimator.snapshot(now)
        self.assertGreaterEqual(before.accepted_sample_count, 1)
        self.assertAlmostEqual(before.last_observation_time, 0.315, places=9)

        # Each has a timestamp that cannot be accepted as a newly received
        # physical sample.  The duplicate/old packets use otherwise fresh
        # arrival times to make a false health refresh observable.
        estimator.observe(self._observation(0.315, 0.0, receipt_delay=0.010))
        estimator.observe(self._observation(0.310, 0.0, receipt_delay=0.010))
        estimator.observe(PayloadObservation(
            t_meas=0.340, t_received=0.330, q=0.0, qy=0.0,
            i_meas=0.16 / CURRENT_TO_TORQUE, i_limit=3.2, mode=0, saturated=False,
        ))
        estimator.observe(PayloadObservation(
            t_meas=0.341, t_received=0.343, q=math.nan, qy=0.0,
            i_meas=0.0, i_limit=3.2, mode=0, saturated=False,
        ))
        after = estimator.snapshot(now)

        self.assertEqual(after.accepted_sample_count, before.accepted_sample_count)
        self.assertAlmostEqual(after.last_observation_time, before.last_observation_time)
        self.assertFalse(after.usable)

    def test_saturation_fault_and_staleness_disable_correction_and_kill_prefault_job(self):
        """A pending pre-fault result cannot re-arm correction after bad health."""
        estimator, t, now = self._learn()
        self.assertTrue(estimator.snapshot(now).usable)

        t_pending, _, pending = self._advance_until_pending(estimator, t)
        self.assertTrue(pending.pending)
        estimator.observe(self._observation(
            t_pending + 0.0001, math.radians(50), saturated=True,
        ))
        estimator.tick(t_pending + 0.030)
        after_saturation = estimator.snapshot(t_pending + 0.030)
        self.assertFalse(after_saturation.pending)
        self.assertFalse(after_saturation.usable)
        self.assertEqual(estimator.correction(math.radians(50), t_pending + 0.030), 0.0)

        # Fault and stale input are independently terminal for application.
        estimator.observe(self._observation(t_pending + 0.031, math.radians(50), mode=1))
        self.assertFalse(estimator.snapshot(t_pending + 0.033).usable)
        estimator.observe(self._observation(t_pending + 0.040, math.radians(50)))
        self.assertEqual(estimator.correction(math.radians(50), t_pending + 0.056), 0.0)

    def test_worker_delay_cadence_and_duplicate_ticks_are_bounded(self):
        """Modeled fitting has 2--8 ms latency and cannot schedule twice per tick."""
        estimator, t, now = self._learn()
        _, submitted_at, pending = self._advance_until_pending(estimator, t)
        self.assertTrue(pending.pending)
        self.assertGreaterEqual(pending.pending_ready_time - submitted_at, 0.002)
        self.assertLessEqual(pending.pending_ready_time - submitted_at, 0.008)
        submissions = pending.worker_submission_count

        estimator.tick(submitted_at)
        duplicate = estimator.snapshot(submitted_at)
        self.assertEqual(duplicate.worker_submission_count, submissions)
        self.assertTrue(duplicate.pending)

        estimator.tick(submitted_at + 0.001)
        self.assertTrue(estimator.snapshot(submitted_at + 0.001).pending)
        estimator.tick(pending.pending_ready_time + 0.0001)
        complete = estimator.snapshot(pending.pending_ready_time + 0.0001)
        self.assertGreaterEqual(complete.last_publication_time, pending.pending_ready_time)

        # Fresh samples continue, but the next modeled job can only be submitted
        # at least 20 ms after the previous submission.
        t2 = t + 0.030
        observation = self._observation(t2, math.radians(50))
        estimator.observe(observation)
        estimator.tick(observation.t_received)
        next_snapshot = estimator.snapshot(observation.t_received)
        if next_snapshot.worker_submission_count > submissions:
            self.assertGreaterEqual(next_snapshot.last_submission_time - submitted_at, 0.020 - 1e-12)

    def test_parameter_projection_torque_cap_and_publication_rate_hold_under_adversarial_data(self):
        """Neither excessive measured current nor a good fit can bypass bounds."""
        adversarial, _, adversarial_now = self._learn(coefficients=(2.0, 2.0),
                                                       i_limit=100.0)
        bad = adversarial.snapshot(adversarial_now)
        self.assertLessEqual(math.hypot(bad.theta_s, bad.theta_c), 0.40 + 1e-12)
        self.assertLessEqual(abs(adversarial.correction(0.0, adversarial_now)), 0.20 + 1e-12)
        self.assertFalse(bad.usable, "projection must not manufacture a consistent fit")

        estimator = PayloadEstimator(seed=5)
        t = 0.0
        for roll in (math.radians(-50), 0.0, math.radians(50)):
            t, now = self._feed(estimator, t, 0.70, roll, coefficients=(0.0, 0.35))
        # First completion is intentionally only a small step from zero.
        estimator.tick(now + 0.009)
        first = estimator.snapshot(now + 0.009)
        self.assertGreater(first.first_eligible_fit_submission, 0.0)
        self.assertLessEqual(
            math.hypot(first.theta_s, first.theta_c),
            0.10 * (first.last_publication_time - first.first_eligible_fit_submission) + 1e-9,
        )

        t, now = self._feed(estimator, t, 3.8, math.radians(50), coefficients=(0.0, 0.35))
        estimator.tick(now + 0.009)
        learned = estimator.snapshot(now + 0.009)
        self.assertTrue(learned.usable, learned)
        raw = estimator.raw_prediction(0.0, now + 0.009)
        self.assertGreater(raw, 0.20)
        self.assertAlmostEqual(estimator.correction(0.0, now + 0.009), 0.20, delta=1e-12)
        self.assertLessEqual(math.hypot(learned.theta_s, learned.theta_c), 0.40 + 1e-12)

    def test_healthy_motion_can_apply_existing_model_and_training_data_expire(self):
        """Dynamic packets preserve an estimate but cannot train it; age still wins."""
        estimator, t, now = self._learn()
        before = estimator.snapshot(now)
        self.assertTrue(before.usable)

        t, now = self._feed(
            estimator, t, 0.12,
            lambda current_t, _: math.radians(50) + 0.40 * (current_t - t),
            current=lambda _, __, ___, index: 0.15 + 0.01 * index,
        )
        moving = estimator.snapshot(now)
        self.assertTrue(moving.usable, moving)
        self.assertEqual(moving.accepted_sample_count, before.accepted_sample_count)
        self.assertNotEqual(estimator.correction(math.radians(40), now), 0.0)

        # A fresh packet after the training timestamp is 30 s old keeps sensor
        # health current but cannot make old training look newly published.
        # A modeled result submitted before motion may complete during the
        # moving packets, so expiration is measured from that accepted job's
        # newest training sample rather than an older snapshot.
        expiry_time = moving.last_training_time + 30.001
        fresh = self._observation(expiry_time, math.radians(40))
        estimator.observe(fresh)
        expired = estimator.snapshot(fresh.t_received)
        self.assertFalse(expired.usable)
        self.assertEqual(estimator.correction(math.radians(40), fresh.t_received), 0.0)

    def test_recovery_needs_a_complete_new_dwell_not_one_fresh_packet(self):
        """Old coefficients remain diagnostic only until a post-fault dwell refits."""
        estimator, t, now = self._learn()
        estimator.observe(self._observation(t, math.radians(50), saturated=True))
        estimator.observe(self._observation(t + 0.005, math.radians(50)))
        estimator.tick(t + 0.007)
        self.assertFalse(estimator.snapshot(t + 0.007).usable)

        t, now = self._feed(estimator, t + 0.010, 0.34, math.radians(50))
        estimator.tick(now + 0.009)
        recovered = estimator.snapshot(now + 0.009)
        self.assertTrue(recovered.usable, recovered)

    def test_recovery_restarts_the_applied_coefficient_ramp_from_zero(self):
        """Disabled time cannot buy slew credit for a diagnostic pre-fault fit."""
        estimator, t, now = self._learn()
        self.assertGreater(abs(estimator.correction(math.radians(50), now)), 0.10)

        estimator.observe(self._observation(t, math.radians(50), saturated=True))
        # Resume one second later with the same physical load.  Retained full-fit
        # diagnostics may validate this dwell, but must not restore applied torque
        # in one step because correction was disabled for the intervening second.
        t, now = self._feed(estimator, t + 1.0, 0.34, math.radians(50))
        estimator.tick(now + 0.009)
        recovered = estimator.snapshot(now + 0.009)

        self.assertTrue(recovered.usable, recovered)
        self.assertGreater(recovered.first_eligible_fit_submission, t - 0.34)
        self.assertLess(math.hypot(recovered.theta_s, recovered.theta_c), 0.012)
        self.assertLess(abs(estimator.correction(math.radians(50), now + 0.009)), 0.012)

    def test_residual_inconsistent_fresh_dwell_invalidates_without_clipping_or_zero_fill(self):
        """Changed or invalid current must reject the learned correction, not hide it."""
        estimator, t, _ = self._learn()
        t, now = self._feed(estimator, t, 0.34, math.radians(50), coefficients=(0.0, -0.16))
        mismatch = estimator.snapshot(now)
        self.assertFalse(mismatch.usable)
        self.assertEqual(mismatch.reason, "residual_mismatch")
        self.assertEqual(estimator.correction(math.radians(50), now), 0.0)

        invalid = PayloadEstimator(seed=6)
        # NaN measurements repeatedly clear the dwell; turning them into zeros
        # would create a false, apparently consistent training target.
        t = 0.0
        for _ in range(80):
            observation = PayloadObservation(
                t_meas=t, t_received=t + 0.002, q=0.0, qy=0.0,
                i_meas=math.nan, i_limit=3.2, mode=0, saturated=False,
            )
            invalid.observe(observation)
            invalid.tick(observation.t_received)
            t += 0.005
        snapshot = invalid.snapshot(t)
        self.assertEqual(snapshot.accepted_sample_count, 0)
        self.assertFalse(snapshot.usable)

    def test_duplicate_or_old_bad_health_packet_invalidates_pending_fit(self):
        """Hard health failures take precedence over timestamp de-duplication."""
        for bad_status in (
            {"mode": 1},
            {"saturated": True},
            {"i_limit": 0.0},
        ):
            with self.subTest(bad_status=bad_status):
                estimator, t, now = self._learn()
                t_pending, _, pending = self._advance_until_pending(estimator, t)
                self.assertTrue(pending.pending)
                before = estimator.snapshot(now)

                duplicate = self._observation(
                    t_pending, math.radians(50), **bad_status,
                )
                self.assertFalse(estimator.observe(duplicate))
                after = estimator.snapshot(duplicate.t_received)
                self.assertFalse(after.usable)
                self.assertFalse(after.pending)
                self.assertGreater(after.health_generation, before.health_generation)
                self.assertEqual(
                    estimator.correction(math.radians(50), duplicate.t_received), 0.0,
                )

        estimator, t, now = self._learn()
        t_pending, _, pending = self._advance_until_pending(estimator, t)
        self.assertTrue(pending.pending)
        before = estimator.snapshot(now)
        old_receipt_fault = PayloadObservation(
            t_meas=t_pending - 0.001,
            t_received=t_pending + 0.003,
            q=math.radians(50), qy=0.0,
            i_meas=0.0, i_limit=3.2, mode=1, saturated=False,
        )
        self.assertFalse(estimator.observe(old_receipt_fault))
        after = estimator.snapshot(old_receipt_fault.t_received)
        self.assertFalse(after.usable)
        self.assertFalse(after.pending)
        self.assertGreater(after.health_generation, before.health_generation)

    def test_raw_dwell_overflow_abstains_and_clears_the_window(self):
        """An unbounded ingress rate cannot grow the raw dwell indefinitely."""
        estimator = PayloadEstimator(seed=8)
        for index in range(estimator.MAX_DWELL_SAMPLES + 40):
            t_meas = index * 1.0e-6
            observation = self._observation(t_meas, 0.0, receipt_delay=0.002)
            accepted = estimator.observe(observation)
            if not accepted:
                break

        snapshot = estimator.snapshot(observation.t_received)
        self.assertEqual(snapshot.reason, "input_overload")
        self.assertFalse(snapshot.usable)
        self.assertFalse(snapshot.pending)
        self.assertEqual(len(estimator._dwell), 0)

    def test_nonfinite_payload_reserves_a_finite_physical_timestamp(self):
        """A rejected payload cannot be replayed later as a new physical sample."""
        estimator = PayloadEstimator(seed=9)
        estimator.observe(self._observation(0.0, 0.0))
        invalid = PayloadObservation(
            t_meas=0.005, t_received=0.007, q=math.nan, qy=0.0,
            i_meas=0.0, i_limit=3.2, mode=0, saturated=False,
        )
        self.assertFalse(estimator.observe(invalid))

        replacement = self._observation(0.005, 0.0)
        self.assertFalse(estimator.observe(replacement))


if __name__ == "__main__":
    unittest.main()
