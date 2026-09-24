"""Request handling: what to do when a request exceeds what the axis can do.

RollGovernor   online reference governor for roll (500 Hz, host side)
admit_yaw_sine offline admission of a yaw sine request against the roll envelope
YawMonitor     online yaw derate trigger (roll saturating => ask yaw to shrink)

Decision rule, used throughout:
  accept   predicted demand <= available torque x (1 - reserve)
  reshape  the request is feasible in position but not at the requested
           speed/acceleration -> slow its clock
  restrict part of the request cannot be held -> stop short of it, report it
  reject   no holdable position, the current reference cannot be held, or an
           input is invalid -> hold the reference, latch, caller falls back
"""
import math
from collections import deque

from sim import params as P


def static_load(q, tau_g, tau_extra, extra_fn=None):
    """Torque needed to hold q: nominal gravity + optional extra load, either a
    constant or a position-dependent model extra_fn(q) (Phase 3 estimator)."""
    return tau_g * math.sin(q) + tau_extra + (extra_fn(q) if extra_fn else 0.0)


def _finite(*xs):
    return all(math.isfinite(x) for x in xs)


class RollGovernor:
    """Reference governor: path time-scaling with explicit request disposition.

    Contract (EXECUTION_PLAN.md, Packet 2A). Each step emits exactly one of
      PATH  the admitted path on its own clock sigma: q_p(sigma), v_p s, and the
            step-average acceleration (v - v_prev)/ts
      JOIN  minimum-jerk segment between two stationary holdable points
      STOP  constant deceleration from the current state to rest, used when the
            path jumps, a join loses feasibility or braking room runs out
      HOLD  stationary at a restricted point until the path point is holdable
    so the emitted (q, v, a) are mutually consistent. The clock is frozen
    outside PATH. The governor never moves a reference to cancel a disturbance:
    predicted over-budget torque is reported (over_budget), not compensated.

    status        accepted | reshaped | joining | restricted | over_budget | rejected
    infeasibility "" | "static" (a position cannot be held) | "dynamic" (the path
                  cannot be traversed within budget at any speed) | "braking"
                  (a limit drop left no room to stop inside the holdable set)
    There is no reference-tracking follower and no root finding.
    """

    STATUS = ("accepted", "reshaped", "joining", "restricted", "over_budget", "rejected")

    def __init__(self, J=P.J_R, b=P.B_VISC, tau_c=P.TAU_C, tau_g=P.TAU_G, k_t=P.K_T,
                 reserve=0.20, d_margin=P.D_MAX, v_max=20.0,
                 q_range=math.radians(90), s_rate=2.0, horizon=0.4, n_look=48,
                 kappa_drop=3.0, kappa_rise=0.25, kappa_min=0.1, align_tol=1.5 * P.ENC_LSB,
                 cpl_tau=1.0, jump_tol=1e-4, stall_time=0.5, blend_max=math.radians(2)):
        if not (n_look >= 1 and horizon > 0 and v_max > 0 and s_rate > 0):
            raise ValueError("RollGovernor needs n_look >= 1 and positive horizon, v_max, s_rate")
        self.J, self.b, self.tau_c, self.tau_g, self.k_t = J, b, tau_c, tau_g, k_t
        self.reserve, self.d_margin = reserve, d_margin
        self.v_max, self.q_range = v_max, q_range
        self.s_rate, self.horizon, self.n_look = s_rate, horizon, n_look
        # Saturation back-off (model-free): sustained saturation means the model
        # is missing load, so the allowed path speed shrinks; it recovers slowly.
        self.kappa_drop, self.kappa_rise, self.kappa_min = kappa_drop, kappa_rise, kappa_min
        # Realign offsets up to one encoder count (plus half a count for path motion
        # since the realign) are snapped, not joined: the feedback sees a one-count
        # step at every count change (~33 mA), whereas a join freezes the path clock
        # for >= 0.05 s. Declared position tolerance at sync.
        self.align_tol = align_tol
        # Larger start offsets up to blend_max (the drive's 2 deg re-arm window) are
        # blended out while the clock runs: q = q_p(sigma) + D (1 - p(t/T_b)), with
        # T_b sized so the blend's own torque stays within 10 % of the budget, which
        # the path reserves meanwhile. Beyond blend_max: JOIN (clock frozen).
        self.blend_max = blend_max
        # A path step whose position change disagrees with its velocity by more
        # than jump_tol is a discontinuity (smooth paths: O(ts^3), ~1e-6 rad).
        self.jump_tol = jump_tol
        # Clock held near zero on a moving, holdable path for stall_time =>
        # the path is reported dynamically infeasible (restricted).
        self.stall_time = stall_time
        # Recent peak |tau_couple| (decays with cpl_tau). Kept across reset() so a
        # re-join after an outage reserves the coupling it will have to carry.
        self.cpl_tau, self.cpl_peak = cpl_tau, 0.0
        self._grid = [math.radians(x) for x in range(-90, 91)]
        self._probe = [math.radians(x) for x in range(-90, 91, 5)]
        self._cache = None
        # Saturation back-off is evidence of unmodelled load: kept across reset()
        # (realignment); only a new run clears it.
        self.kappa = 1.0
        # A suspended request (host: tracking fault, incompatible yaw) is brought to
        # rest and held; kept across reset() (realignment) until resume().
        self.suspended = ""
        self.reset(0.0)           # defined state before the host's first reset()

    def reset(self, q, v=0.0, t=0.0):
        """(Re)start at measured position q with the path clock at t. If q is not
        on the path, the first step plans a join. The clock then (re)starts at the
        fastest speed whose one-step velocity change fits the budget (_s_start)."""
        # A moving start (v != 0) only exists for a suspended governor catching the
        # axis after a tracking fault: it brakes from (q, v). Otherwise from rest.
        self.q, self.v, self.a = q, (v if self.suspended else 0.0), 0.0
        self.sigma, self.s = t, 1.0
        self.blend = None         # start-offset blend while on the path
        self.pq, self.pv = q, 0.0  # last path-only position/velocity (jump detection)
        self.mode = "sync"        # sync | path | join | stop | hold
        self.fresh = True         # first sync after reset(): offsets <= align_tol snap
        self.seg = None           # active JOIN / STOP segment
        self.s_free = False       # next path step may choose s afresh (after sync/join)
        self.latched = False      # a rejection freezes the governor until reset()
        self.stall = 0.0
        self.hold_iv = None       # interval at which a restricted hold was entered
        self.jumps = []           # clock positions of recent path discontinuities
        self.incons = 0.0         # path time traversed while the plan's q and v disagree
        self.sig_seen = t         # clock position at the previous path step
        self.t_int, self.t_drop, self.tau_prev = 0.0, -math.inf, None
        self.status, self.reason, self.infeasibility = "accepted", "", ""
        self.braking_short = False
        self.speed_capped = False
        self._set_flags()

    def suspend(self, reason):
        """Stop the request's traversal and hold (status restricted, with reason).
        The clock stays frozen; resume() re-joins the path from the hold."""
        self.suspended = reason or "request suspended"

    def resume(self):
        self.suspended = ""

    def catch_plan(self, q, v, i_limit, tau_couple=0.0, extra_fn=None):
        """Predicted rest position of the brake-to-rest a suspended governor would
        run from the measured state (q, v), or None if it does not fit. tau_couple is
        a coupling *bound* (the caller passes the recent peak, not the instantaneous
        value, so the verdict does not follow the coupling's phase). Both the braking
        and the final hold are judged against actuator capacity: the alternative to
        a catch is passive damping, which resists neither load nor coupling. The
        final hold must also lie inside the governor's holdable interval. A model
        prediction, not a guarantee; the drive's lockout bounds repeated failures."""
        tau_av = self.available(i_limit)
        cap, cpl = self._capacity(tau_av), abs(tau_couple)
        if not _finite(q, v, tau_av, cpl):
            return None
        iv = self.feasible_interval(tau_av, 0.0, extra_fn)
        if iv is None:
            return None
        q_end = q
        if abs(v) > 1e-9:
            saved = self.q, self.v
            self.q, self.v = q, v
            try:
                a_b = self._brake_decel(cap, 0.0, extra_fn, cpl)
            finally:
                self.q, self.v = saved
            if a_b is None:
                return None
            q_end = q + math.copysign(v * v / (2 * a_b), v)
        if not iv[0] <= q_end <= iv[1] or self._hold(q_end, 0.0, extra_fn) + cpl > cap:
            return None
        return q_end

    def _set_flags(self):
        st = self.status
        self.blocked = st == "rejected"                    # caller must fall back
        self.rejected = st in ("restricted", "rejected")   # request not fully honoured
        self.limited = st != "accepted"
        self.over_budget = st == "over_budget"

    def available(self, i_limit):
        return self.k_t * i_limit * (1 - self.reserve) - self.d_margin

    def feasible_interval(self, tau_av, tau_extra, extra_fn=None):
        """Largest interval of 1-deg grid points around 0 that the axis can hold,
        or None when no position is holdable (never replaced by a default).
        Cached on the budget and on the load model sampled every 5 deg."""
        probe = tuple(round(extra_fn(q), 4) for q in self._probe) if extra_fn else ()
        key = (round(tau_av, 4), round(tau_extra, 4), probe)
        if self._cache and self._cache[0] == key:
            return self._cache[1]
        ok = [abs(static_load(q, self.tau_g, tau_extra, extra_fn)) + self.tau_c <= tau_av
              and abs(q) <= self.q_range for q in self._grid]
        i0 = min(range(len(ok)), key=lambda i: (not ok[i], abs(self._grid[i])))
        if not ok[i0]:
            iv = None
        else:
            lo = hi = i0
            while lo > 0 and ok[lo - 1]:
                lo -= 1
            while hi < len(ok) - 1 and ok[hi + 1]:
                hi += 1
            iv = (self._grid[lo], self._grid[hi])
        self._cache = (key, iv)
        return iv

    # -- torque model ---------------------------------------------------------
    def _hold(self, q, tau_extra, extra_fn):
        return abs(static_load(q, self.tau_g, tau_extra, extra_fn)) + self.tau_c

    def _s_max(self, q, v, a, tau_av, tau_extra, extra_fn, tau_couple):
        """Largest path speed s in [0, 1] at which this plan point is feasible:
        J|a| s^2 + (b + J s_rate/2)|v| s <= tau_av - hold - |tau_couple|.
        The s_rate term reserves torque for changing s by up to half its rate
        relative to s (J v_p s_dot with |s_dot| <= s s_rate/2), so a feasible
        s > 0 exists wherever the point can be held. Closed form."""
        m = tau_av - self._hold(q, tau_extra, extra_fn) - abs(tau_couple)
        if m <= 0:
            return 0.0
        A, B = self.J * abs(a), (self.b + 0.5 * self.J * self.s_rate) * abs(v)
        if A < 1e-12:
            s = 1.0 if B < 1e-12 else m / B
        else:
            s = (-B + math.sqrt(B * B + 4 * A * m)) / (2 * A)
        if abs(v) > 1e-9:
            s = min(s, self.v_max / abs(v))
        return max(0.0, min(1.0, s))

    def _s_start(self, ts, qp, vp, tau_av, tau_extra, extra_fn, tau_couple, margin=0.9):
        """Largest s in [0, 1] whose step from the current velocity to v_p s keeps
        |J (v_p s - v)/ts + b v_p s + static + couple| + tau_c within margin * budget.
        The emitted acceleration is this step average, so the bound is exact to
        first order in the clock advance."""
        c0 = (static_load(qp, self.tau_g, tau_extra, extra_fn) + tau_couple
              - self.J * self.v / ts)
        k = vp * (self.J / ts + self.b)
        m = margin * tau_av - self.tau_c
        if abs(c0) > m:
            return 0.0
        if abs(k) < 1e-12:
            return 1.0
        s = (m - math.copysign(1.0, k) * c0) / abs(k)
        if abs(vp) > 1e-9:
            s = min(s, self.v_max / abs(vp))
        return max(0.0, min(1.0, s))

    def _demand(self, q, v, a, tau_extra, extra_fn, tau_couple):
        """Nominal motor torque the reference state asks for (worst-sign friction)."""
        return (abs(self.J * a + self.b * v + static_load(q, self.tau_g, tau_extra, extra_fn)
                    + tau_couple) + self.tau_c)

    def _demand_bound(self, q, v, a, tau_extra, extra_fn):
        """Conservative bound (no sign cancellation) used to plan segments."""
        return self.J * abs(a) + self.b * abs(v) + self._hold(q, tau_extra, extra_fn)

    # -- segments ---------------------------------------------------------------
    @staticmethod
    def _mj(x):
        return (10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5, 30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4,
                60 * x - 180 * x ** 2 + 120 * x ** 3)

    def _seg_state(self, seg, x):
        """(q, v, a) of a JOIN segment at normalised time x in [0, 1]."""
        p, pd, pdd = self._mj(x)
        T = seg["T"]
        D = seg["q1"] - seg["q0"]
        return seg["q0"] + D * p, D * pd / T, D * pdd / T ** 2

    def _seg_excess(self, seg, x0, tau_av, tau_extra, extra_fn, n=40):
        """Largest demand-bound excess over tau_av on the segment from x0 to 1."""
        worst = -math.inf
        for k in range(n + 1):
            q, v, a = self._seg_state(seg, x0 + (1 - x0) * k / n)
            if abs(q) > max(self.q_range, abs(seg["q0"])) + 1e-9:
                return math.inf                  # moving further beyond the travel range
            worst = max(worst, self._demand_bound(q, v, a, tau_extra, extra_fn) - tau_av)
        return worst

    def _seg_time(self, seg, tau_av, tau_extra, extra_fn, T0, T_max):
        """Shortest T in [T0, T_max] (x1.15 steps) whose segment fits tau_av less the
        recent peak coupling, else without that reservation. Returns (T, fits); when
        nothing fits, the T with the smallest excess and fits = False."""
        best = (math.inf, T0)
        for reserve in (self.cpl_peak, 0.0):
            T = T0
            while T <= T_max * (1 + 1e-9):
                seg["T"] = T
                ex = self._seg_excess(seg, 0.0, tau_av - reserve, tau_extra, extra_fn)
                if ex <= 0:
                    return T, True
                best = min(best, (ex, T))
                T *= 1.15
        return best[1], False

    def _plan_join(self, q0, q1, tau_av, tau_extra, extra_fn):
        """Shortest join duration (0.05 s .. 10 s) within budget, None if none fits."""
        seg = dict(kind="join", q0=q0, q1=q1, T=1.0)
        T, fits = self._seg_time(seg, tau_av, tau_extra, extra_fn,
                                 max(0.05, 1.875 * abs(q1 - q0) / self.v_max), 10.0)
        return T if fits else None

    def _finish(self, status, reason, q, v, a, infeasibility=""):
        if status == "rejected":
            self.latched = True
        self.q, self.v, self.a = q, v, a
        self.status, self.reason, self.infeasibility = status, reason, infeasibility
        self._set_flags()
        return q, v, a

    def _reject(self, reason, infeasibility=""):
        return self._finish("rejected", reason, self.q, 0.0, 0.0, infeasibility)

    # -- main step ----------------------------------------------------------------
    def step(self, ts, ref_at, i_limit, tau_extra=0.0, tau_couple=0.0, extra_fn=None,
             saturated=False):
        if not (math.isfinite(ts) and ts > 0):
            raise ValueError("ts must be positive and finite")
        self.braking_short = self.speed_capped = False
        if self.latched:
            return self._finish("rejected", self.reason, self.q, 0.0, 0.0, self.infeasibility)
        if not _finite(i_limit, tau_extra, tau_couple) or (
                extra_fn is not None and not _finite(extra_fn(self.q))):
            return self._reject("non-finite limit, load or coupling input")
        tau_av = self.available(i_limit)
        self.t_int += ts
        if self.tau_prev is not None and tau_av < self.tau_prev - 1e-9:
            self.t_drop = self.t_int          # the limit fell: braking room may be short
        self.tau_prev = tau_av
        self.cpl_peak = max(abs(tau_couple), self.cpl_peak * math.exp(-ts / self.cpl_tau))
        if saturated:
            self.kappa = max(self.kappa_min, self.kappa - self.kappa_drop * ts)
        else:
            self.kappa = min(1.0, self.kappa + self.kappa_rise * ts)

        interval = self.feasible_interval(tau_av, tau_extra, extra_fn)
        if interval is None:
            return self._reject("no holdable position at the active limit", "static")
        lo, hi = interval
        ctx = (ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        inside = lo - 1e-9 <= self.q <= hi + 1e-9
        if self.suspended:
            return self._step_suspended(ctx, inside)
        if self.mode == "stop":
            return self._step_stop(*ctx)
        if self.mode == "join":
            return self._step_join(*ctx)
        if not inside:
            if abs(self.v) > 1e-9:
                return self._begin_stop(ctx, "restricted", "reference outside holdable "
                                        "interval, braking", self._room_label())
            return self._return_to_boundary(ctx, "current reference position outside the "
                                            "holdable interval")
        if self.mode == "hold":
            return self._step_hold(*ctx)
        if self.mode == "sync":
            return self._sync(*ctx)
        return self._step_path(*ctx)

    # -- modes ----------------------------------------------------------------------
    def _step_suspended(self, ctx, inside):
        """Suspended: brake to rest (STOP), then hold where it stops. A stationary
        reference outside the holdable interval returns to its boundary as usual."""
        ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple = ctx
        if self.mode == "stop":
            self.seg["then"] = "hold"
            return self._step_stop(*ctx)
        if self.mode == "join" and self.seg is not None and self.seg.get("cap"):
            return self._step_join(*ctx)          # return-to-boundary in progress
        if abs(self.v) > 1e-9:
            return self._begin_stop(ctx, "restricted", self.suspended,
                                    "" if inside else self._room_label())
        if not inside:
            return self._return_to_boundary(ctx, self.suspended + "; reference outside "
                                            "the holdable interval")
        self._enter_hold(lo, hi)
        over = self._demand(self.q, 0.0, 0.0, tau_extra, extra_fn, tau_couple) > tau_av
        return self._finish("over_budget" if over else "restricted", self.suspended,
                            self.q, 0.0, 0.0, "")

    def _room_label(self):
        """'braking' only when a recent limit drop took the room away; otherwise the
        reference simply met a static boundary."""
        return "braking" if self.t_int - self.t_drop < 1.0 else "static"

    def _enter_hold(self, lo, hi):
        self.mode, self.hold_iv, self.seg, self.blend = "hold", (lo, hi), None, None

    def _return_to_boundary(self, ctx, why):
        """Stationary outside the reserved interval: return to its boundary using
        the reserve (restricted, labelled); reject only if even that does not fit."""
        ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple = ctx
        label = self._room_label()
        qb = hi if self.q > hi else lo
        T = self._plan_join(self.q, qb, self._capacity(tau_av), tau_extra, extra_fn)
        self.v = 0.0
        if T is None:
            return self._reject(why + "; not recoverable within actuator capacity", label)
        self.seg = dict(kind="join", t=0.0, T=T, q0=self.q, q1=qb, restricted=True,
                        tau_av=tau_av, cap=True, label=label,
                        why=why + ", returning to its boundary using the reserve",
                        done="restricted at the holdable boundary")
        self.mode = "join"
        return self._step_join(*ctx)

    def _sync(self, ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple):
        """Enter the path at the current clock from rest (start-up, realignment,
        after a stop). Offsets within align_tol are snapped; otherwise JOIN."""
        ctx = (ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        qp, vp, ap = ref_at(self.sigma)
        if not _finite(qp, vp, ap):
            return self._reject("non-finite plan sample")
        q1 = min(hi, max(lo, qp))
        restricted = q1 != qp
        # Snapping is only for a restart from a measurement; after an internal stop
        # the reference is exact and any offset is joined.
        fresh = self.fresh
        tol, self.fresh = (self.align_tol if fresh else 1e-12), False
        self.blend = None
        if fresh and not restricted and tol < abs(q1 - self.q) <= self.blend_max:
            D = self.q - q1
            A, B = 5.77 * self.J * abs(D), 1.875 * self.b * abs(D)
            x = (-B + math.sqrt(B * B + 0.4 * A * tau_av)) / (2 * A)
            self.blend = dict(D=D, t=0.0, T=max(0.05, 1.0 / x))
            self.mode, self.s, self.s_free = "path", 0.0, True
            return self._step_path(*ctx)
        if abs(q1 - self.q) <= tol:
            if restricted:
                self._enter_hold(lo, hi)
                return self._finish("restricted", "path point outside holdable interval",
                                    self.q, 0.0, 0.0, "static")
            self.mode, self.s, self.s_free = "path", 0.0, True
            return self._step_path(*ctx)
        T = self._plan_join(self.q, q1, tau_av, tau_extra, extra_fn)
        if restricted:
            # A target on the holdable boundary has no static margin for motion:
            # step it back towards the current position (never past it), 1 deg at a
            # time, until a join of <= 1 s fits. If none does, hold where we are.
            d = math.radians(1) * (1 if self.q > q1 else -1)
            for _ in range(10):
                if T is not None and T <= 1.0:
                    break
                q1 += d
                if (q1 - self.q) * d >= 0:
                    T = None
                    break
                T = self._plan_join(self.q, q1, tau_av, tau_extra, extra_fn)
            if T is None or T > 1.0:
                self._enter_hold(lo, hi)
                return self._finish("restricted", "path point outside holdable interval; "
                                    "boundary not reachable within budget, holding",
                                    self.q, 0.0, 0.0, "static")
        elif T is None:
            why = ("join longer than 10 s at v_max" if 1.875 * abs(q1 - self.q) / self.v_max > 10
                   else "no budget-feasible join to the path")
            return self._reject(why, "dynamic")
        self.seg = dict(kind="join", t=0.0, T=T, q0=self.q, q1=q1, restricted=restricted,
                        tau_av=tau_av)
        self.mode = "join"
        return self._step_join(*ctx)

    def _step_join(self, ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple):
        ctx = (ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        seg = self.seg
        lim = self._capacity(tau_av) if seg.get("cap") else tau_av
        x_now = min(1.0, seg["t"] / seg["T"])
        if tau_av < seg["tau_av"] - 1e-9 or not (lo - 1e-9 <= seg["q1"] <= hi + 1e-9):
            # The limit fell mid-join: re-validate the rest of the segment.
            if (not (lo - 1e-9 <= seg["q1"] <= hi + 1e-9)
                    or self._seg_excess(seg, x_now, lim, tau_extra, extra_fn) > 0):
                return self._begin_stop(ctx, "restricted", "join no longer feasible after "
                                        "limit change, braking", self._room_label())
            seg["tau_av"] = tau_av
        seg["t"] += ts
        x = min(1.0, seg["t"] / seg["T"])
        q, v, _ = self._seg_state(seg, x)
        if x >= 1.0:
            q, v = seg["q1"], 0.0
        a = (v - self.v) / ts             # step average, as on the path
        over = self._demand(q, v, a, tau_extra, extra_fn, tau_couple) > lim
        if x >= 1.0:
            self.seg = None
            if seg["restricted"]:
                self._enter_hold(lo, hi)
                return self._finish("over_budget" if over else "restricted",
                                    seg.get("done", "path point outside holdable interval"),
                                    q, 0.0, a, seg.get("label", "static"))
            self.mode, self.s, self.s_free = "path", 0.0, True
        if over:
            return self._finish("over_budget", "predicted torque exceeds budget", q, v, a,
                                seg.get("label", "") if seg["restricted"] else "")
        if seg["restricted"]:
            return self._finish("restricted", seg.get("why", "joining holdable boundary "
                                "short of the path"), q, v, a, seg.get("label", "static"))
        return self._finish("joining", "joining path", q, v, a)

    def _capacity(self, tau_av):
        """Actuator capacity less the disturbance margin: the budget without the
        reserve. Braking and recovery from a lost braking room may spend it."""
        return (tau_av + self.d_margin) / (1 - self.reserve) - self.d_margin

    def _brake_decel(self, tau_lim, tau_extra, extra_fn, tau_couple):
        """Largest constant deceleration a_b from (q, v) to rest that tau_lim supports
        along the travel it produces. Signed balance, with m = sign(v):
            m tau_motor = -J a_b + m static(q) + b|v'| + tau_c m sign(v') + m tau_couple,
        v' in [0, |v|]; loads that oppose the motion help the stop. Worst case over
        the travel (sampled), friction and coupling sign gives
            a_b = (tau_lim + min_q m static(q) - |tau_couple| - tau_c) / J.
        Iterated on the travel v^2 / (2 a_b); None if nothing positive fits."""
        v, m = abs(self.v), math.copysign(1.0, self.v)
        a, dist = None, 0.0
        for _ in range(4):
            lows = [m * static_load(self.q + m * dist * k / 20, self.tau_g, tau_extra, extra_fn)
                    for k in range(21)]
            a_new = (tau_lim + min(lows) - abs(tau_couple) - self.tau_c) / self.J
            if a_new <= 0:
                return None
            a = a_new if a is None else min(a, a_new)
            dist = v * v / (2 * a)
        return a

    def _begin_stop(self, ctx, status, reason, infeasibility, then="hold"):
        """Brake from the current (q, v) to rest at constant deceleration (clock
        frozen), re-planned every step against the current limit. A stop stays within
        the budget unless that would carry it out of the holdable interval; it then
        uses the reserve (capacity). Velocity is integrated trapezoidally, so the
        emitted (q, v, a) are consistent by construction. then: 'hold' (stay
        restricted where it stops) or 'sync' (re-join the path, e.g. after a jump)."""
        ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple = ctx
        self.seg = self.blend = None
        if abs(self.v) < 1e-9:
            if then == "sync":
                self.mode = "sync"
                return self._sync(*ctx)
            self._enter_hold(lo, hi)
            return self._finish(status, reason, self.q, 0.0, 0.0, infeasibility)
        self.seg = dict(kind="stop", emergency=infeasibility == "braking", status=status,
                        reason=reason, infeasibility=infeasibility, then=then, noted=set())
        self.mode = "stop"
        return self._step_stop(*ctx)

    def _step_stop(self, ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple):
        ctx = (ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        seg = self.seg

        def note(msg):
            if msg not in seg["noted"]:
                seg["noted"].add(msg)
                seg["reason"] += "; " + msg

        def plan(emergency):
            lim = self._capacity(tau_av) if emergency else tau_av
            return self._brake_decel(lim, tau_extra, extra_fn, tau_couple), lim

        a_b, lim = plan(seg["emergency"])
        if not seg["emergency"]:
            m = math.copysign(1.0, self.v)
            q_end = None if a_b is None else self.q + m * self.v ** 2 / (2 * a_b)
            if q_end is None or not (lo <= q_end <= hi):
                seg["emergency"] = True
                note("using the reserve to stop inside the holdable interval")
                a_b, lim = plan(True)
        if a_b is None:
            a_b = max(1.0, (lim - self._hold(self.q, tau_extra, extra_fn)) / self.J)
            note("braking exceeds capacity")
        dv = a_b * ts
        v = 0.0 if abs(self.v) <= dv else self.v - math.copysign(dv, self.v)
        q, a = self.q + 0.5 * (self.v + v) * ts, (v - self.v) / ts
        status = seg["status"]
        if self._demand(q, v, a, tau_extra, extra_fn, tau_couple) > lim + 1e-12:
            status = "over_budget"          # e.g. coupling pushing along the motion
            note("braking exceeds %s" % ("capacity" if seg["emergency"] else "budget"))
        if v != 0.0:
            return self._finish(status, seg["reason"], q, v, a, seg["infeasibility"])
        self.seg = None
        if not (lo - 1e-9 <= q <= hi + 1e-9):
            # Emit this final stop sample; the next step (stationary, outside) returns
            # to the boundary through _return_to_boundary.
            self._enter_hold(lo, hi)
            return self._finish(status, seg["reason"] + "; stopped outside the "
                                "holdable interval", q, v, a, seg["infeasibility"])
        if seg["then"] == "sync":
            self.mode = "sync"          # next step re-joins the path
        else:
            self._enter_hold(lo, hi)
        return self._finish(status, seg["reason"], q, v, a, seg["infeasibility"])

    def _step_hold(self, ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple):
        """Stationary at a restricted point; re-join once the path point is holdable."""
        qp, vp, ap = ref_at(self.sigma)
        if not _finite(qp, vp, ap):
            return self._reject("non-finite plan sample")
        ahead = ref_at(self.sigma + 0.01)[0]
        if lo <= qp <= hi and _finite(ahead) and lo <= ahead <= hi:
            # The path point and the path just ahead are holdable (e.g. the limit was
            # restored): resume -- by a join if the reference is off the path.
            self.hold_iv = None
            if abs(qp - self.q) > 1e-12:
                self.mode = "sync"
                return self._sync(ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
            self.mode, self.s, self.s_free, self.pq, self.pv = "path", 0.0, True, qp, 0.0
            return self._step_path(ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        self.hold_iv = (lo, hi)
        status = ("over_budget" if self._demand(self.q, 0.0, 0.0, tau_extra, extra_fn,
                                                tau_couple) > tau_av else "restricted")
        reason = self.reason if self.reason.startswith("path point outside") else \
            "path point outside holdable interval"
        return self._finish(status, reason, self.q, 0.0, 0.0, "static")

    def _step_path(self, ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple):
        ctx = (ts, ref_at, lo, hi, tau_av, tau_extra, extra_fn, tau_couple)
        tau_full = tau_av
        blending = self.blend is not None
        if self.blend is not None:
            tau_av = 0.9 * tau_av             # 10 % reserved for the start-offset blend
        # -- choose path speed s with look-ahead braking -----------------------
        s_allow, blocked_ahead = 1.0, False
        prev, at_clock, s_kink = None, False, 1.0
        for j in range(self.n_look + 1):
            # quadratic spacing: dense near the present, where the binding
            # point of a speed/torque constraint usually lies after a reversal
            dsig = self.horizon * (j / self.n_look) ** 2
            qp, vp, ap = ref_at(self.sigma + dsig)
            if not _finite(qp, vp, ap):
                return self._reject("non-finite plan sample")
            # A velocity kink between samples (v_p jumps, a_p does not explain it)
            # can only be crossed with the clock at rest: brake to it like a boundary.
            # Likewise where the plan's position and velocity disagree (a jump or an
            # invalid segment): the clock must be at rest there.
            if prev is not None:
                d = dsig - prev[0]
                # trapezoid with Euler-Maclaurin end correction: O(d^5) on smooth plans
                dq_exp = 0.5 * (vp + prev[2]) * d - (ap - prev[3]) * d * d / 12
                dv_kink = abs(vp - prev[2] - 0.5 * (ap + prev[3]) * d)
                if abs(qp - prev[1] - dq_exp) > self.jump_tol + 0.05 * abs(dq_exp):
                    s_cross = 0.01                    # position jump: arrive nearly at rest
                    at_clock = at_clock or j == 1
                elif dv_kink > 0.05:
                    # Velocity kink: cross at the speed whose one-step velocity change
                    # fits half the budget and stays below the discontinuity tolerance.
                    s_cross = min(1.0, 1.5e-4 / (dv_kink * ts),
                                  0.5 * tau_av * ts / (self.J * dv_kink))
                else:
                    s_cross = None
                if s_cross is not None:
                    # Braking target at the sample BEFORE it, planned at half the s rate
                    # so the discrete clock can follow (the grid under-estimates range).
                    # Kept apart from s_allow so the saturation back-off does not scale
                    # the crossing speed itself.
                    s_kink = min(s_kink, math.sqrt(s_cross ** 2 + self.s_rate * prev[0]))
            prev = (dsig, qp, vp, ap)
            kink = False
            if not (lo <= qp <= hi):
                s_j, blocked_ahead = 0.0, True        # must be stationary before here
            elif kink:
                s_j = 0.0
            else:
                s_j = self._s_max(qp, vp, ap, tau_av, tau_extra, extra_fn,
                                  tau_couple if j == 0 else 0.0)
            # kinematic braking: s_now^2 <= s_j^2 + 2 * s_rate * dsigma
            s_allow = min(s_allow, math.sqrt(s_j * s_j + 2 * self.s_rate * dsig))
        # A real jump occupies no path time; a q/v disagreement that persists over
        # path time actually traversed (5 ms of plan) means the plan is invalid.
        self.incons = self.incons + max(0.0, self.sigma - self.sig_seen) if at_clock else 0.0
        self.sig_seen = self.sigma
        if self.incons > 0.005:
            return self._reject("plan inconsistent: position and velocity disagree "
                                "over its segment ahead")
        s_allow = min(s_allow * self.kappa, s_kink)
        q0, v0, a0 = ref_at(self.sigma)
        snap = self.s_free
        if self.s_free:
            # Entering the path from rest (start-up, realignment, end of a join):
            # start at the largest speed the budget absorbs in one step instead of
            # ramping from 0, which would lose path time on feasible requests.
            self.s_free = False
            self.s = min(s_allow, self._s_start(ts, q0, v0, tau_av, tau_extra, extra_fn,
                                                tau_couple))
        if abs(v0) < 1e-9:
            # Path momentarily stationary: v = v_p*s = 0 whatever s is, so s may be
            # lowered at once to the allowance without breaking consistency.
            self.s = min(self.s, s_allow)
        # Raising s costs J v_p s_dot: limit the rise to the torque left at this point.
        rise = self.s_rate
        if abs(v0) > 1e-9:
            left = tau_av - (self.J * abs(a0) * self.s ** 2 + self.b * abs(v0) * self.s
                             + self._hold(q0, tau_extra, extra_fn) + abs(tau_couple))
            rise = min(10 * self.s_rate, max(left, 0.0) / (self.J * abs(v0)))
        if snap:
            rise = 0.0     # the start already spent this step's budget on J v_p s / ts
        s_new = max(self.s - self.s_rate * ts, min(s_allow, self.s + rise * ts, 1.0))
        self.braking_short = s_new > s_allow + 1e-6
        # Clock advance uses the speed consistent with the emitted velocity (after a
        # snap the reference was at rest: v = 0 although s was just chosen afresh).
        s_prev = self.v / v0 if (snap and abs(v0) > 1e-9) else self.s
        for _ in range(4):
            sigma_new = self.sigma + 0.5 * (s_prev + s_new) * ts
            qn, vn, an = ref_at(sigma_new)
            if not (_finite(vn) and abs(vn) * s_new > self.v_max * (1 + 1e-12)):
                break
            # Hard speed cap (safety limit). Only reached when the sampled
            # look-ahead missed the binding point; flagged, never silent.
            s_new = self.v_max / abs(vn)
            self.speed_capped = True
        if _finite(vn) and abs(vn) * s_new > self.v_max:
            s_new = self.v_max / abs(vn)    # residual after iteration: ~1e-12 rad in q
        if not _finite(qn, vn, an):
            return self._reject("non-finite plan sample")
        dq_exp = 0.5 * (self.pv + vn * s_new) * ts
        if not snap and abs(qn - self.pq - dq_exp) > self.jump_tol + 0.05 * abs(dq_exp):
            # Path discontinuity: do not pass the jump through. Move the clock past
            # it, brake to rest, then JOIN the path afterwards. Repeated discontinuities
            # at the same place mean the plan itself is inconsistent: reject it.
            self.jumps = [x for x in self.jumps if sigma_new - x < 0.05] + [sigma_new]
            if len(self.jumps) >= 3:
                return self._reject("plan inconsistent: position and velocity disagree "
                                    "over its segment ahead")
            self.sigma = sigma_new
            return self._begin_stop(ctx, "joining", "path discontinuity: braking, then "
                                    "re-joining", "", then="sync")
        off = offv = 0.0
        if self.blend is not None:
            b = self.blend
            x = min(1.0, (b["t"] + ts) / b["T"])
            p, pd, _ = self._mj(x)
            off, offv = b["D"] * (1 - p), -b["D"] * pd / b["T"]
        if not (lo - 1e-9 <= qn + off <= hi + 1e-9):
            label = self._room_label()
            if label == "static":
                # The clock crept into a static boundary: stop here in one step if
                # the budget allows, then hold restricted.
                q1, a1 = self.q + 0.5 * self.v * ts, -self.v / ts
                if lo <= q1 <= hi and self._demand(q1, 0.0, a1, tau_extra, extra_fn,
                                                   tau_couple) <= tau_full:
                    self._enter_hold(lo, hi)
                    return self._finish("restricted", "path point outside holdable "
                                        "interval", q1, 0.0, a1, "static")
                return self._begin_stop(ctx, "restricted", "reached the holdable boundary, "
                                        "braking", "static")
            # Braking room ran out (the limit dropped while moving): brake along a
            # consistent STOP, using the reserve if needed.
            return self._begin_stop(ctx, "restricted", "insufficient braking room "
                                    "(limit changed while moving), braking", "braking")
        self.sigma, self.s = sigma_new, s_new
        # Step-average acceleration: v_new = v_old + a*ts holds exactly, even where
        # s_dot switches (the instantaneous a_p s^2 + v_p s_dot would jump).
        v_path = vn * s_new
        self.pq, self.pv = qn, v_path
        if self.blend is not None:
            self.blend["t"] += ts
            if self.blend["t"] >= self.blend["T"]:
                self.blend = None
        v = v_path + offv
        q, a = qn + off, (v - self.v) / ts
        moving = abs(vn) > 1e-6 and not blocked_ahead
        self.stall = self.stall + ts if (moving and self.s < 0.02) else 0.0
        if self._demand(q, v, a, tau_extra, extra_fn, tau_couple) > tau_full:
            return self._finish("over_budget", "predicted torque exceeds budget", q, v, a)
        if blocked_ahead and self.s < 1e-6:
            return self._finish("restricted", "path point outside holdable interval",
                                q, v, a, "static")
        if self.stall > self.stall_time:
            return self._finish("restricted", "path cannot be traversed within budget "
                                "at any speed (clock stalled)", q, v, a, "dynamic")
        if blending:
            return self._finish("joining", "blending start offset into the path", q, v, a)
        if self.s < 0.999 or self.kappa < 0.999:
            reason = ("path slowed; braking allowance exceeded" if self.braking_short
                      else "path held at speed limit" if self.speed_capped
                      else "path slowed to fit torque budget")
            return self._finish("reshaped", reason, q, v, a)
        return self._finish("accepted", "", q, v, a)


def yaw_envelope(f, i_limit, reserve=0.20, coupling_scale=1.0, k_t=P.K_T):
    """Largest yaw sine amplitude (rad) at f whose coupling roll can reject at 0 deg."""
    w = 2 * math.pi * f
    avail = k_t * i_limit * (1 - reserve) - P.D_MAX - P.TAU_C
    per_rad = coupling_scale * w * math.hypot(P.K_YV, P.K_YA * w)
    return max(0.0, avail / per_rad)


def admit_yaw_sine(A, f, i_limit, **kw):
    """-> (decision, A_allowed). 'reject' when less than 10 % would remain."""
    A_max = yaw_envelope(f, i_limit, **kw)
    if A <= A_max:
        return "accept", A
    if A_max >= 0.1 * A:
        return "reshape", A_max
    return "reject", 0.0


class YawMonitor:
    """Online yaw derate: if roll spends > sat_frac of a window at the current
    limit while yaw coupling is significant, ask the yaw planner for a smaller
    amplitude (x shrink per trigger). Covers coupling larger than modelled
    (Phase 1 finding).

    Occupancy is (samples saturated with yaw active) / (all valid samples) over a
    complete window: at least `complete` of the window's expected samples at the
    host period dt. The host clears the history whenever it idles (feedback
    loss, drive fallback): a window spanning an outage mixes two regimes and is
    not evidence about either. The hold-off (t_last) survives clear()."""

    def __init__(self, window=0.5, sat_frac=0.05, shrink=0.8, hold_off=0.6, complete=0.9):
        self.window, self.sat_frac, self.shrink, self.hold_off = window, sat_frac, shrink, hold_off
        self.complete = complete

    def reset(self, dt=None):
        self.hist = deque()
        self.n_sat = 0
        self.t_last = -1e9
        self.dt = dt

    def clear(self):
        self.hist.clear()
        self.n_sat = 0

    def update(self, t, flagged):
        self.hist.append((t, bool(flagged)))
        self.n_sat += bool(flagged)
        while self.hist and self.hist[0][0] < t - self.window:
            self.n_sat -= self.hist.popleft()[1]
        need = self.complete * self.window / self.dt if self.dt else 0
        if len(self.hist) < need or self.hist[-1][0] - self.hist[0][0] <= 0.8 * self.window:
            return None
        if self.n_sat / len(self.hist) > self.sat_frac and t - self.t_last > self.hold_off:
            self.t_last = t
            self.clear()
            return self.shrink
        return None
