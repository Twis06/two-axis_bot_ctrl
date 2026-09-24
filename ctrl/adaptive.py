"""Task 3 L3: feed-forward-only integration of the bounded payload estimator.

The estimator (ctrl/payload_estimator.py, Task 3 L1/L2, unchanged here) learns
theta_s sin q + theta_c cos q from delayed measured feedback only. This adapter:

  * feeds every received Feedback to the estimator and ticks it on every host
    update, including updates that return early for stale feedback or fallback,
    so health invalidation is never skipped;
  * exposes the bounded correction as the baseline's load model, and keeps it out
    of the governor, fault catch and budget checks (feed-forward only);
  * computes the bumpless-transfer term: the part of the correction change due
    to a coefficient update at the current reference angle, which the host
    removes from its integrator so the total command is continuous;
  * returns zero whenever the estimator reports itself unusable.

With AdaptiveController(enabled=False) the host is the frozen baseline.
"""
import math

from ctrl.baseline import BaselineController
from ctrl.payload_estimator import PayloadEstimator, PayloadObservation


def estimator_class(stationary_counts=None):
    """The L1 estimator unchanged, or (Task 3 L4 candidate B) with only its dwell
    stationarity window widened to `stationary_counts` encoder counts."""
    if stationary_counts is None:
        return PayloadEstimator
    return type(f"PayloadEstimatorStat{stationary_counts}", (PayloadEstimator,),
                {"MAX_STATIONARY_RANGE": stationary_counts * PayloadEstimator.ENCODER_COUNT})


class PayloadFFAdapter:
    """Load-model object for BaselineController (torque(q) = learned correction).
    AdaptiveController keeps it out of every feasibility decision (see below)."""

    def __init__(self, seed=0, stationary_counts=None):
        self.stationary_counts = stationary_counts
        self.est = estimator_class(stationary_counts)(seed=seed)
        self.reset()

    def reset(self):
        self.est.reset()
        self.coef = (0.0, 0.0)          # coefficients behind the last applied correction
        self.snap, self.t = None, 0.0

    def update(self, *a, **k):
        pass                            # the estimator is fed from observe()

    def observe(self, t, fb, saturated):
        self.t = t
        if fb is not None:
            self.est.observe(PayloadObservation(t_meas=fb.t_meas, t_received=t, q=fb.q, qy=fb.qy,
                                                i_meas=fb.i_meas, i_limit=fb.i_limit, mode=fb.mode,
                                                saturated=bool(saturated)))
        self.snap = self.est.tick(t)

    def coefficients(self):
        return self.snap.coefficients if (self.snap is not None and self.snap.usable) else (0.0, 0.0)

    @staticmethod
    def at(k, q):
        b = PayloadEstimator.MAX_APPLICATION_TORQUE
        return max(-b, min(b, k[0] * math.sin(q) + k[1] * math.cos(q)))

    def bump(self, q):
        """Change of the applied correction at angle q caused by a model update
        (coefficients now vs at the last host tick); moves to or from the integrator."""
        new = self.coefficients()
        d = self.at(new, q) - self.at(self.coef, q)
        self.coef = new
        return d if math.isfinite(d) else 0.0

    def torque(self, q):
        c = self.est.correction(q, self.t)
        return c if math.isfinite(c) else 0.0


class AdaptiveController(BaselineController):
    """Frozen baseline + feed-forward-only learned payload correction (candidate).

    The baseline's load-model hook reaches the governor, the fault catch and the
    budget checks as well as feed-forward. Here the governor methods the host
    calls with that model are wrapped to drop it, so the learned correction
    enters feed-forward only and never changes admission or catch decisions
    (docs/plans/task3-learning.md §7). The frozen baseline source is untouched.
    Bumpless transfer: before each update the change caused by a model update,
    evaluated at the previous reference angle, is removed from the integrator.
    """
    name = "baseline+payload_ff"

    def __init__(self, enabled=True, est_seed=0, stationary_counts=None, **kw):
        self.adapter = PayloadFFAdapter(est_seed, stationary_counts) if enabled else None
        super().__init__(load_model=self.adapter, **kw)

    def reset(self, cfg):
        super().reset(cfg)
        if self.adapter is None:
            return
        g = self.gov
        step, catch, hold, demand = g.step, g.catch_plan, g._hold, g._demand

        def step_nominal(ts, ref_at, i_limit, tau_extra=0.0, tau_couple=0.0, extra_fn=None, **k):
            return step(ts, ref_at, i_limit, tau_extra, tau_couple, None, **k)

        def catch_nominal(q, v, i_limit, tau_couple=0.0, extra_fn=None):
            return catch(q, v, i_limit, tau_couple, None)

        g.step, g.catch_plan = step_nominal, catch_nominal
        g._hold = lambda q, tau_extra, extra_fn=None: hold(q, tau_extra, None)
        g._demand = lambda q, v, a, tau_extra, extra_fn, tau_couple: demand(q, v, a, tau_extra, None,
                                                                            tau_couple)

    def update(self, ctx, fb):
        if self.adapter is None:
            return super().update(ctx, fb)
        self.adapter.observe(ctx.t, fb, self.was_saturated)
        if self.initialised:
            self.integ -= self.adapter.bump(self.gov.q)
        else:
            self.adapter.coef = self.adapter.coefficients()
        cmd = super().update(ctx, fb)
        sn = self.adapter.snap
        self.telemetry.update(learn_usable=float(sn.usable), learn_theta_s=sn.theta_s,
                              learn_theta_c=sn.theta_c)
        return cmd
