"""Bounded, standalone gravity-residual estimator for Task 3 L1.

The estimator deliberately knows nothing about the simulator, controller state,
reference, or plant.  It only consumes timestamped measured feedback.  A
modeled worker makes a fit visible after a deterministic seeded 2--8 ms delay;
no call sleeps or waits for that result.
"""

from collections import deque
from dataclasses import dataclass
import math
import random
from typing import Deque, Optional, Sequence, Tuple


# Public status strings.  They make health and abstention readable in telemetry
# without binding callers to private numeric state.
REASON_AWAITING_OBSERVATION = "awaiting_observation"
REASON_AWAITING_ESTIMATE = "awaiting_estimate"
REASON_RECOVERY_DWELL_REQUIRED = "recovery_dwell_required"
REASON_USABLE = "usable"
REASON_INVALID_OBSERVATION = "invalid_observation"
REASON_FUTURE_TIMESTAMP = "future_timestamp"
REASON_OUT_OF_ORDER = "out_of_order"
REASON_MEASUREMENT_STALE = "measurement_stale"
REASON_RECEIPT_STALE = "receipt_stale"
REASON_FAULT = "fault_mode"
REASON_SATURATED = "saturated"
REASON_NEAR_CURRENT_LIMIT = "near_current_limit"
REASON_INSUFFICIENT_COVERAGE = "insufficient_coverage"
REASON_RESIDUAL_INCONSISTENT = "residual_inconsistent"
REASON_RESIDUAL_MISMATCH = "residual_mismatch"
REASON_LATE_WORKER_RESULT = "late_worker_result"
REASON_ESTIMATE_EXPIRED = "estimate_expired"
REASON_FUTURE_APPLICATION_TIME = "future_application_time"
REASON_INPUT_OVERLOAD = "input_overload"


@dataclass(frozen=True)
class PayloadObservation:
    """One delayed physical feedback sample available to the host."""

    t_meas: float
    t_received: float
    q: float
    qy: float
    i_meas: float
    i_limit: float
    mode: object
    saturated: bool


@dataclass(frozen=True)
class PayloadConditioning:
    """Coverage/conditioning information for the current training buffer."""

    roll_span: float
    cosine_span: float
    min_eigenvalue: float
    max_eigenvalue: float
    eigenvalue_ratio: float
    rms_residual: Optional[float]


@dataclass(frozen=True)
class PayloadSnapshot:
    """Immutable diagnostic state; coefficients are only feed-forward data."""

    coefficients: Tuple[float, float]
    theta_s: float
    theta_c: float
    usable: bool
    reason: str
    last_training_time: Optional[float]
    last_publication_time: Optional[float]
    last_observation_time: Optional[float]
    last_receipt_time: Optional[float]
    accepted_sample_count: int
    conditioning: PayloadConditioning
    full_fit_coefficients: Tuple[float, float]
    pending: bool
    pending_ready_time: Optional[float]
    last_submission_time: Optional[float]
    first_eligible_fit_submission: Optional[float]
    worker_submission_count: int
    publication_count: int
    health_generation: int
    last_fit_reason: str
    coefficient_norm: float
    application_torque_bound: float

    @property
    def roll_span(self) -> float:
        return self.conditioning.roll_span

    @property
    def cosine_span(self) -> float:
        return self.conditioning.cosine_span

    @property
    def min_eigenvalue(self) -> float:
        return self.conditioning.min_eigenvalue

    @property
    def max_eigenvalue(self) -> float:
        return self.conditioning.max_eigenvalue

    @property
    def eigenvalue_ratio(self) -> float:
        return self.conditioning.eigenvalue_ratio

    @property
    def rms_residual(self) -> Optional[float]:
        return self.conditioning.rms_residual


@dataclass(frozen=True)
class _DwellSample:
    t_meas: float
    t_received: float
    q: float
    qy: float
    i_meas: float


@dataclass(frozen=True)
class _Aggregate:
    t_meas: float
    t_received: float
    q: float
    feature_s: float
    feature_c: float
    target: float


@dataclass(frozen=True)
class _PendingFit:
    submitted_at: float
    ready_at: float
    generation: int
    newest_training_time: float
    aggregates: Tuple[_Aggregate, ...]


class PayloadEstimator:
    """Learn a bounded ``theta_s*sin(q) + theta_c*cos(q)`` residual.

    The numerical settings are intentionally fixed for the L1 experiment.  A
    caller should invoke :meth:`observe` for received feedback and :meth:`tick`
    from its normal event loop; neither operation blocks.
    """

    NOMINAL_GRAVITY = 0.120
    CURRENT_TO_TORQUE = 0.140
    ENCODER_COUNT = 2.0 * math.pi / 16384.0
    NORMAL_DRIVE_MODE = 0
    NORMAL_DRIVE_NAME = "normal"

    MAX_ARRIVAL_AGE = 0.015
    MAX_APPLICATION_MEASUREMENT_AGE = 0.015
    MAX_APPLICATION_RECEIPT_AGE = 0.020
    DWELL_SECONDS = 0.250
    MIN_DWELL_SAMPLES = 20
    MAX_DWELL_GAP = 0.015
    MAX_DWELL_SAMPLES = 256
    MAX_STATIONARY_RANGE = 2.0 * ENCODER_COUNT
    NEAR_LIMIT_FRACTION = 0.95
    UPDATE_PERIOD = 0.020
    BUFFER_SIZE = 600
    MIN_AGGREGATES = 30
    MIN_ROLL_SPAN = math.radians(60.0)
    MIN_COSINE_SPAN = 0.25
    MIN_GRAM_EIGENVALUE = 0.01
    MIN_GRAM_RATIO = 0.01
    RIDGE = 1.0e-6
    MAX_COEFFICIENT_NORM = 0.40
    MAX_RMS_RESIDUAL = 0.065
    MIN_WORKER_DELAY = 0.002
    MAX_WORKER_DELAY = 0.008
    MAX_WORKER_LATENCY = 0.100
    MAX_ESTIMATE_AGE = 30.0
    MAX_APPLICATION_TORQUE = 0.20
    MAX_COEFFICIENT_RATE = 0.10

    def __init__(self, seed: int = 0):
        self._seed = seed
        self.reset()

    def reset(self) -> None:
        """Clear observations, jobs, learned coefficients, and health state."""
        self._rng = random.Random(self._seed)
        self._dwell: Deque[_DwellSample] = deque()
        self._training: Deque[_Aggregate] = deque(maxlen=self.BUFFER_SIZE)
        self._pending: Optional[_PendingFit] = None
        self._coefficients = (0.0, 0.0)
        # ``_full_fit_coefficients`` is the physically projected fit.  It is
        # separate from ``_coefficients``, which may still be in its slew ramp.
        self._full_fit_coefficients = (0.0, 0.0)
        self._consistency_coefficients: Optional[Tuple[float, float]] = None
        self._model_valid = False
        self._sensor_ok = False
        self._recovery_required = False
        self._health_reason = REASON_AWAITING_OBSERVATION
        self._last_fit_reason = REASON_AWAITING_ESTIMATE
        self._health_generation = 0
        self._last_seen_measurement_time: Optional[float] = None
        self._last_valid_measurement_time: Optional[float] = None
        self._last_valid_receipt_time: Optional[float] = None
        self._last_training_time: Optional[float] = None
        self._last_publication_time: Optional[float] = None
        # Rate state is deliberately distinct from diagnostic publication time.
        # A fault clears this state so disabled time cannot buy slew credit.
        self._rate_last_publication_time: Optional[float] = None
        self._last_submission_time: Optional[float] = None
        self._first_eligible_fit_submission: Optional[float] = None
        self._last_tick_time: Optional[float] = None
        self._last_aggregate_tick_time: Optional[float] = None
        self._last_aggregate_measurement_time: Optional[float] = None
        self._worker_submission_count = 0
        self._publication_count = 0
        self._conditioning = self._empty_conditioning()

    def observe(self, observation: PayloadObservation) -> bool:
        """Ingest one physical sample without fitting or waiting for a worker.

        ``True`` means the packet was a newly timestamped, causally valid sensor
        packet.  It may still be ineligible for learning when it is near the
        current limit or moving.  Rejected packets never refresh health.
        """
        if not isinstance(observation, PayloadObservation):
            self._invalidate(REASON_INVALID_OBSERVATION)
            return False

        timestamp_numeric = (observation.t_meas, observation.t_received)
        if not all(self._is_finite(value) for value in timestamp_numeric):
            self._invalidate(REASON_INVALID_OBSERVATION)
            return False

        if observation.t_meas > observation.t_received:
            self._invalidate(REASON_FUTURE_TIMESTAMP)
            return False
        arrival_age = observation.t_received - observation.t_meas
        if arrival_age > self.MAX_ARRIVAL_AGE:
            self._invalidate(REASON_MEASUREMENT_STALE)
            return False

        # Hard drive-state failures take precedence over timestamp
        # de-duplication.  A repeated physical sample that explicitly reports
        # a fault or saturation must still fail closed and kill pending work.
        hard_reason = None
        if self._is_finite(observation.i_limit) and observation.i_limit <= 0.0:
            hard_reason = REASON_INVALID_OBSERVATION
        elif not self._is_normal_mode(observation.mode):
            hard_reason = REASON_FAULT
        elif bool(observation.saturated):
            hard_reason = REASON_SATURATED
        if hard_reason is not None:
            is_new_timestamp = (
                self._last_seen_measurement_time is None or
                observation.t_meas > self._last_seen_measurement_time
            )
            receipt_is_ordered = (
                self._last_valid_receipt_time is None or
                observation.t_received >= self._last_valid_receipt_time
            )
            if is_new_timestamp and receipt_is_ordered:
                self._last_seen_measurement_time = observation.t_meas
            self._invalidate(hard_reason)
            return False

        if self._last_seen_measurement_time is not None:
            if observation.t_meas <= self._last_seen_measurement_time:
                # A delayed replay is ignored, rather than allowed to refresh
                # the receipt timestamp or invalidate an otherwise fresh stream.
                return False
        if (self._last_valid_receipt_time is not None and
                observation.t_received < self._last_valid_receipt_time):
            return False

        # This is a new physical timestamp even if the drive state makes it
        # unusable.  Recording it prevents a later replay of the same sample.
        self._last_seen_measurement_time = observation.t_meas

        payload_numeric = (observation.q, observation.qy, observation.i_meas,
                           observation.i_limit)
        if not all(self._is_finite(value) for value in payload_numeric):
            self._invalidate(REASON_INVALID_OBSERVATION)
            return False

        self._sensor_ok = True
        self._health_reason = REASON_USABLE
        self._last_valid_measurement_time = observation.t_meas
        self._last_valid_receipt_time = observation.t_received

        # Near-limit feedback is healthy enough to keep an existing correction
        # applied, but it cannot enter an unsaturated dwell fit.
        if abs(observation.i_meas) >= self.NEAR_LIMIT_FRACTION * observation.i_limit:
            self._clear_dwell()
            self._last_fit_reason = REASON_NEAR_CURRENT_LIMIT
            return True

        appended = self._append_dwell(_DwellSample(
            t_meas=observation.t_meas,
            t_received=observation.t_received,
            q=observation.q,
            qy=observation.qy,
            i_meas=observation.i_meas,
        ))
        return appended

    def tick(self, t_now: float) -> PayloadSnapshot:
        """Poll one modeled result and schedule at most one new modeled fit."""
        if not self._is_finite(t_now):
            return self._snapshot_for_invalid_time()
        if self._last_tick_time is not None and t_now <= self._last_tick_time:
            return self.snapshot(t_now)
        self._last_tick_time = t_now

        sensor_reason = self._sensor_health_reason(t_now, invalidate=True)
        if sensor_reason is None:
            if self._pending is not None and t_now >= self._pending.ready_at:
                self._complete_pending(t_now)

            aggregate = self._new_aggregate(t_now)
            if aggregate is not None:
                if self._fresh_dwell_mismatches(aggregate):
                    # A changed load must not be hidden by a long buffer of old
                    # observations.  Retain diagnostics, but restart training.
                    self._invalidate(REASON_RESIDUAL_MISMATCH, clear_training=True)
                    self._consistency_coefficients = None
                else:
                    self._training.append(aggregate)
                    self._conditioning = self._conditioning_for(self._training)
                    self._schedule_if_allowed(t_now)

        return self.snapshot(t_now)

    def snapshot(self, t_now: float) -> PayloadSnapshot:
        """Return immutable diagnostic state; this never schedules a worker."""
        usable, reason = self._application_state(t_now, invalidate=True)
        pending = self._pending
        return PayloadSnapshot(
            coefficients=self._coefficients,
            theta_s=self._coefficients[0],
            theta_c=self._coefficients[1],
            usable=usable,
            reason=reason,
            last_training_time=self._last_training_time,
            last_publication_time=self._last_publication_time,
            last_observation_time=self._last_valid_measurement_time,
            last_receipt_time=self._last_valid_receipt_time,
            accepted_sample_count=len(self._training),
            conditioning=self._conditioning,
            full_fit_coefficients=self._full_fit_coefficients,
            pending=pending is not None,
            pending_ready_time=None if pending is None else pending.ready_at,
            last_submission_time=self._last_submission_time,
            first_eligible_fit_submission=self._first_eligible_fit_submission,
            worker_submission_count=self._worker_submission_count,
            publication_count=self._publication_count,
            health_generation=self._health_generation,
            last_fit_reason=self._last_fit_reason,
            coefficient_norm=math.hypot(*self._coefficients),
            application_torque_bound=self.MAX_APPLICATION_TORQUE,
        )

    def raw_prediction(self, q: float, t_now: float) -> float:
        """Return the usable, unclipped model torque for clipping diagnostics."""
        usable, _ = self._application_state(t_now, invalidate=True)
        if not usable or not self._is_finite(q):
            return 0.0
        value = self._coefficients[0] * math.sin(q) + self._coefficients[1] * math.cos(q)
        return value if self._is_finite(value) else 0.0

    def correction(self, q: float, t_now: float) -> float:
        """Return finite, bounded feed-forward correction; zero when unusable."""
        raw = self.raw_prediction(q, t_now)
        return max(-self.MAX_APPLICATION_TORQUE,
                   min(self.MAX_APPLICATION_TORQUE, raw))

    # -- observation and health -------------------------------------------------

    @staticmethod
    def _is_finite(value: object) -> bool:
        try:
            return math.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    def _is_normal_mode(self, mode: object) -> bool:
        return mode == self.NORMAL_DRIVE_MODE or mode == self.NORMAL_DRIVE_NAME

    def _append_dwell(self, sample: _DwellSample) -> bool:
        if self._dwell and sample.t_meas - self._dwell[-1].t_meas > self.MAX_DWELL_GAP:
            self._clear_dwell()
        if len(self._dwell) >= self.MAX_DWELL_SAMPLES:
            self._invalidate(REASON_INPUT_OVERLOAD)
            return False
        self._dwell.append(sample)

        if self._dwell:
            q_values = [entry.q for entry in self._dwell]
            qy_values = [entry.qy for entry in self._dwell]
            if (max(q_values) - min(q_values) > self.MAX_STATIONARY_RANGE + 1e-15 or
                    max(qy_values) - min(qy_values) > self.MAX_STATIONARY_RANGE + 1e-15):
                self._clear_dwell()
                self._dwell.append(sample)

        # A sliding 250 ms window provides one new aggregate per allowed update
        # without counting samples from an earlier settled pose.
        while (len(self._dwell) > 1 and
               self._dwell[-1].t_meas - self._dwell[1].t_meas >= self.DWELL_SECONDS):
            self._dwell.popleft()
        return True

    def _clear_dwell(self) -> None:
        self._dwell.clear()
        self._last_aggregate_measurement_time = None

    def _invalidate(self, reason: str, clear_training: bool = False) -> None:
        """Disable application immediately and invalidate any pending result."""
        already_invalid = (not self._sensor_ok and self._health_reason == reason and
                           self._pending is None and not self._model_valid)
        self._sensor_ok = False
        self._model_valid = False
        self._recovery_required = True
        self._health_reason = reason
        # Correction is forced to zero by health.  Clear the applied state too,
        # so a later recovery must ramp from zero even if the diagnostic full fit
        # and training buffer are retained.
        self._coefficients = (0.0, 0.0)
        self._rate_last_publication_time = None
        self._first_eligible_fit_submission = None
        self._pending = None
        self._clear_dwell()
        self._last_fit_reason = reason
        if clear_training:
            self._training.clear()
            self._conditioning = self._empty_conditioning()
            self._last_training_time = None
        if not already_invalid:
            self._health_generation += 1

    def _sensor_health_reason(self, t_now: float, invalidate: bool) -> Optional[str]:
        if not self._sensor_ok:
            return self._health_reason
        if self._last_valid_measurement_time is None or self._last_valid_receipt_time is None:
            return REASON_AWAITING_OBSERVATION
        if t_now < self._last_valid_measurement_time or t_now < self._last_valid_receipt_time:
            return REASON_FUTURE_APPLICATION_TIME
        if t_now - self._last_valid_measurement_time > self.MAX_APPLICATION_MEASUREMENT_AGE:
            if invalidate:
                self._invalidate(REASON_MEASUREMENT_STALE)
            return REASON_MEASUREMENT_STALE
        if t_now - self._last_valid_receipt_time > self.MAX_APPLICATION_RECEIPT_AGE:
            if invalidate:
                self._invalidate(REASON_RECEIPT_STALE)
            return REASON_RECEIPT_STALE
        return None

    def _application_state(self, t_now: float, invalidate: bool) -> Tuple[bool, str]:
        if not self._is_finite(t_now):
            return False, REASON_INVALID_OBSERVATION
        sensor_reason = self._sensor_health_reason(t_now, invalidate)
        if sensor_reason is not None:
            return False, sensor_reason
        if self._model_valid and self._last_training_time is not None:
            if t_now - self._last_training_time > self.MAX_ESTIMATE_AGE:
                if invalidate:
                    self._invalidate(REASON_ESTIMATE_EXPIRED, clear_training=True)
                return False, REASON_ESTIMATE_EXPIRED
        if not self._model_valid:
            if self._health_reason == REASON_RESIDUAL_MISMATCH:
                return False, REASON_RESIDUAL_MISMATCH
            if self._recovery_required:
                if self._last_fit_reason == REASON_RESIDUAL_MISMATCH:
                    return False, REASON_RESIDUAL_MISMATCH
                return False, REASON_RECOVERY_DWELL_REQUIRED
            if self._last_fit_reason in (REASON_INSUFFICIENT_COVERAGE,
                                         REASON_RESIDUAL_INCONSISTENT):
                return False, self._last_fit_reason
            return False, REASON_AWAITING_ESTIMATE
        return True, REASON_USABLE

    # -- dwell aggregation and modeled worker ----------------------------------

    def _new_aggregate(self, t_now: float) -> Optional[_Aggregate]:
        if self._last_aggregate_tick_time is not None:
            if t_now - self._last_aggregate_tick_time < self.UPDATE_PERIOD:
                return None
        if len(self._dwell) < self.MIN_DWELL_SAMPLES:
            return None
        first, last = self._dwell[0], self._dwell[-1]
        if last.t_meas - first.t_meas < self.DWELL_SECONDS:
            return None
        if (self._last_aggregate_measurement_time is not None and
                last.t_meas <= self._last_aggregate_measurement_time):
            return None
        # Dwell integrity is maintained in _append_dwell; re-check it here so
        # no aggregation can depend on an accidental caller-side mutation.
        for older, newer in zip(self._dwell, tuple(self._dwell)[1:]):
            if newer.t_meas - older.t_meas > self.MAX_DWELL_GAP:
                return None
        q_values = tuple(entry.q for entry in self._dwell)
        qy_values = tuple(entry.qy for entry in self._dwell)
        if (max(q_values) - min(q_values) > self.MAX_STATIONARY_RANGE + 1e-15 or
                max(qy_values) - min(qy_values) > self.MAX_STATIONARY_RANGE + 1e-15):
            return None

        count = len(self._dwell)
        feature_s = math.fsum(math.sin(entry.q) for entry in self._dwell) / count
        feature_c = math.fsum(math.cos(entry.q) for entry in self._dwell) / count
        target = math.fsum(
            self.CURRENT_TO_TORQUE * entry.i_meas -
            self.NOMINAL_GRAVITY * math.sin(entry.q)
            for entry in self._dwell
        ) / count
        aggregate = _Aggregate(
            t_meas=last.t_meas,
            t_received=last.t_received,
            q=math.fsum(q_values) / count,
            feature_s=feature_s,
            feature_c=feature_c,
            target=target,
        )
        self._last_aggregate_tick_time = t_now
        self._last_aggregate_measurement_time = last.t_meas
        return aggregate

    def _fresh_dwell_mismatches(self, aggregate: _Aggregate) -> bool:
        if self._consistency_coefficients is None:
            return False
        theta_s, theta_c = self._consistency_coefficients
        raw_prediction = theta_s * aggregate.feature_s + theta_c * aggregate.feature_c
        return abs(aggregate.target - raw_prediction) > self.MAX_RMS_RESIDUAL

    def _schedule_if_allowed(self, t_now: float) -> None:
        if self._pending is not None:
            return
        if self._last_submission_time is not None:
            if t_now - self._last_submission_time < self.UPDATE_PERIOD:
                return
        aggregates = tuple(
            aggregate for aggregate in self._training
            if aggregate.t_received <= t_now
        )
        if not aggregates:
            return
        conditioning = self._conditioning_for(aggregates)
        # Do not create a worker result that can only report insufficient
        # coverage.  This keeps a mismatch/fault cause observable throughout
        # recovery and enforces the coverage gate before any new fit is enabled.
        if not self._coverage_is_sufficient(len(aggregates), conditioning):
            return
        if self._first_eligible_fit_submission is None:
            self._first_eligible_fit_submission = t_now
        delay = self._rng.uniform(self.MIN_WORKER_DELAY, self.MAX_WORKER_DELAY)
        self._pending = _PendingFit(
            submitted_at=t_now,
            ready_at=t_now + delay,
            generation=self._health_generation,
            newest_training_time=max(aggregate.t_meas for aggregate in aggregates),
            aggregates=aggregates,
        )
        self._last_submission_time = t_now
        self._worker_submission_count += 1

    def _complete_pending(self, t_now: float) -> None:
        pending = self._pending
        self._pending = None
        if pending is None:
            return
        if pending.generation != self._health_generation:
            self._last_fit_reason = REASON_INVALID_OBSERVATION
            return
        if t_now - pending.submitted_at > self.MAX_WORKER_LATENCY:
            self._last_fit_reason = REASON_LATE_WORKER_RESULT
            return
        if t_now - pending.newest_training_time > self.MAX_ESTIMATE_AGE:
            self._invalidate(REASON_ESTIMATE_EXPIRED, clear_training=True)
            return

        coefficients, conditioning, reason = self._fit(pending.aggregates)
        self._conditioning = conditioning
        self._last_fit_reason = reason
        if coefficients is None:
            return

        self._full_fit_coefficients = coefficients
        self._consistency_coefficients = coefficients
        self._coefficients = self._rate_limited_coefficients(coefficients, t_now)
        self._model_valid = True
        self._recovery_required = False
        self._health_reason = REASON_USABLE
        self._last_training_time = pending.newest_training_time
        self._last_publication_time = t_now
        self._rate_last_publication_time = t_now
        self._publication_count += 1
        self._last_fit_reason = REASON_USABLE

    # -- bounded 2x2 ridge fit --------------------------------------------------

    @classmethod
    def _empty_conditioning(cls) -> PayloadConditioning:
        return PayloadConditioning(0.0, 0.0, 0.0, 0.0, 0.0, None)

    def _conditioning_for(self, aggregates: Sequence[_Aggregate]) -> PayloadConditioning:
        if not aggregates:
            return self._empty_conditioning()
        count = len(aggregates)
        roll_span = max(item.q for item in aggregates) - min(item.q for item in aggregates)
        cosine_span = (max(item.feature_c for item in aggregates) -
                       min(item.feature_c for item in aggregates))
        g_ss = math.fsum(item.feature_s * item.feature_s for item in aggregates) / count
        g_sc = math.fsum(item.feature_s * item.feature_c for item in aggregates) / count
        g_cc = math.fsum(item.feature_c * item.feature_c for item in aggregates) / count
        trace = g_ss + g_cc
        discriminant = max(0.0, (g_ss - g_cc) ** 2 + 4.0 * g_sc ** 2)
        root = math.sqrt(discriminant)
        min_eigenvalue = 0.5 * (trace - root)
        max_eigenvalue = 0.5 * (trace + root)
        ratio = (min_eigenvalue / max_eigenvalue
                 if max_eigenvalue > 0.0 else 0.0)
        return PayloadConditioning(
            roll_span=roll_span,
            cosine_span=cosine_span,
            min_eigenvalue=min_eigenvalue,
            max_eigenvalue=max_eigenvalue,
            eigenvalue_ratio=ratio,
            rms_residual=None,
        )

    def _coverage_is_sufficient(self, count: int, conditioning: PayloadConditioning) -> bool:
        return (
            count >= self.MIN_AGGREGATES and
            conditioning.roll_span >= self.MIN_ROLL_SPAN and
            conditioning.cosine_span >= self.MIN_COSINE_SPAN and
            conditioning.min_eigenvalue >= self.MIN_GRAM_EIGENVALUE and
            conditioning.eigenvalue_ratio >= self.MIN_GRAM_RATIO
        )

    def _fit(self, aggregates: Sequence[_Aggregate]) -> Tuple[
            Optional[Tuple[float, float]], PayloadConditioning, str]:
        conditioning = self._conditioning_for(aggregates)
        count = len(aggregates)
        if not self._coverage_is_sufficient(count, conditioning):
            return None, conditioning, REASON_INSUFFICIENT_COVERAGE

        g_ss = math.fsum(item.feature_s * item.feature_s for item in aggregates) / count
        g_sc = math.fsum(item.feature_s * item.feature_c for item in aggregates) / count
        g_cc = math.fsum(item.feature_c * item.feature_c for item in aggregates) / count
        b_s = math.fsum(item.feature_s * item.target for item in aggregates) / count
        b_c = math.fsum(item.feature_c * item.target for item in aggregates) / count
        a_ss = g_ss + self.RIDGE
        a_cc = g_cc + self.RIDGE
        determinant = a_ss * a_cc - g_sc * g_sc
        if not self._is_finite(determinant) or determinant <= 0.0:
            return None, conditioning, REASON_INSUFFICIENT_COVERAGE
        theta_s = (a_cc * b_s - g_sc * b_c) / determinant
        theta_c = (a_ss * b_c - g_sc * b_s) / determinant
        if not self._is_finite(theta_s) or not self._is_finite(theta_c):
            return None, conditioning, REASON_RESIDUAL_INCONSISTENT

        norm = math.hypot(theta_s, theta_c)
        if norm > self.MAX_COEFFICIENT_NORM:
            scale = self.MAX_COEFFICIENT_NORM / norm
            theta_s *= scale
            theta_c *= scale
        rms = math.sqrt(math.fsum(
            (item.target - theta_s * item.feature_s - theta_c * item.feature_c) ** 2
            for item in aggregates
        ) / count)
        conditioning = PayloadConditioning(
            roll_span=conditioning.roll_span,
            cosine_span=conditioning.cosine_span,
            min_eigenvalue=conditioning.min_eigenvalue,
            max_eigenvalue=conditioning.max_eigenvalue,
            eigenvalue_ratio=conditioning.eigenvalue_ratio,
            rms_residual=rms,
        )
        if not self._is_finite(rms) or rms > self.MAX_RMS_RESIDUAL:
            return None, conditioning, REASON_RESIDUAL_INCONSISTENT
        return (theta_s, theta_c), conditioning, REASON_USABLE

    def _rate_limited_coefficients(self, target: Tuple[float, float], t_now: float) -> Tuple[float, float]:
        if self._rate_last_publication_time is None:
            start = self._first_eligible_fit_submission
            elapsed = 0.0 if start is None else max(0.0, t_now - start)
        else:
            elapsed = max(0.0, t_now - self._rate_last_publication_time)
        allowed_change = self.MAX_COEFFICIENT_RATE * elapsed
        delta_s = target[0] - self._coefficients[0]
        delta_c = target[1] - self._coefficients[1]
        delta_norm = math.hypot(delta_s, delta_c)
        if delta_norm <= allowed_change or delta_norm == 0.0:
            return target
        scale = allowed_change / delta_norm
        return (
            self._coefficients[0] + scale * delta_s,
            self._coefficients[1] + scale * delta_c,
        )

    def _snapshot_for_invalid_time(self) -> PayloadSnapshot:
        """Avoid mutating state for a malformed tick timestamp."""
        pending = self._pending
        return PayloadSnapshot(
            coefficients=self._coefficients,
            theta_s=self._coefficients[0],
            theta_c=self._coefficients[1],
            usable=False,
            reason=REASON_INVALID_OBSERVATION,
            last_training_time=self._last_training_time,
            last_publication_time=self._last_publication_time,
            last_observation_time=self._last_valid_measurement_time,
            last_receipt_time=self._last_valid_receipt_time,
            accepted_sample_count=len(self._training),
            conditioning=self._conditioning,
            full_fit_coefficients=self._full_fit_coefficients,
            pending=pending is not None,
            pending_ready_time=None if pending is None else pending.ready_at,
            last_submission_time=self._last_submission_time,
            first_eligible_fit_submission=self._first_eligible_fit_submission,
            worker_submission_count=self._worker_submission_count,
            publication_count=self._publication_count,
            health_generation=self._health_generation,
            last_fit_reason=self._last_fit_reason,
            coefficient_norm=math.hypot(*self._coefficients),
            application_torque_bound=self.MAX_APPLICATION_TORQUE,
        )
