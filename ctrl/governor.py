"""Request handling: what to do when a request exceeds what the axis can do.

RollGovernor   online reference governor for roll (500 Hz, host side)
admit_yaw_sine offline admission of a yaw sine request against the roll envelope
YawMonitor     online yaw derate trigger (roll saturating => ask yaw to shrink)

Decision rule, used throughout:
  accept   predicted demand <= available torque x (1 - reserve)
  reshape  the request is feasible in position but not at the requested
           speed/acceleration -> slow it (acceleration limited to what torque allows)
  derate   the drive reports a lower current limit -> recompute the envelope
  reject   static load at the requested position exceeds capacity -> do not
           follow; hold at the feasible boundary and flag it
"""
import math

from sim import params as P


def static_load(q, tau_g, tau_extra, extra_fn=None):
    """Torque needed to hold q: nominal gravity + optional extra load, either a
    constant or a position-dependent model extra_fn(q) (Phase 3 estimator)."""
    return tau_g * math.sin(q) + tau_extra + (extra_fn(q) if extra_fn else 0.0)


class RollGovernor:
    """Reference governor: path time-scaling + static feasibility projection.

    The request is a planned path q_p(sigma). The governor advances the path's
    own clock at rate s = dsigma/dt in [0, 1]:
      * s is the largest value for which the torque needed along the path over a
        look-ahead horizon fits the available torque (reshape = slow down, never
        change the path geometry, so the reference cannot overshoot or leave the
        requested range); deceleration of s is planned so it always has room to brake
      * positions whose static load cannot be held are projected onto the
        feasible interval F (reject = hold at the boundary, flagged)
      * a short critically damped follower smooths the projection edges.
    Only the host's own plan (ref_at) and the drive-reported current limit are used.
    """

    def __init__(self, J=P.J_R, b=P.B_VISC, tau_c=P.TAU_C, tau_g=P.TAU_G, k_t=P.K_T,
                 reserve=0.20, d_margin=P.D_MAX, w_follow=60.0, v_max=20.0,
                 q_range=math.radians(90), s_rate=2.0, horizon=0.4, n_look=12,
                 kappa_drop=3.0, kappa_rise=0.25, kappa_min=0.1):
        self.J, self.b, self.tau_c, self.tau_g, self.k_t = J, b, tau_c, tau_g, k_t
        self.reserve, self.d_margin = reserve, d_margin
        self.w, self.v_max, self.q_range = w_follow, v_max, q_range
        self.s_rate, self.horizon, self.n_look = s_rate, horizon, n_look
        # Saturation back-off (model-free): sustained saturation means the model
        # is missing load, so the allowed path speed shrinks; it recovers slowly.
        self.kappa_drop, self.kappa_rise, self.kappa_min = kappa_drop, kappa_rise, kappa_min
        self._grid = [math.radians(x) for x in range(-90, 91)]
        self._cache = None

    def reset(self, q, v=0.0, t=0.0):
        self.q, self.v, self.a = q, v, 0.0
        self.sigma, self.s = t, 1.0          # plans start at rest; look-ahead slows if needed
        self.kappa = 1.0
        self.limited = False     # reshaped this tick (s < 1)
        self.rejected = False    # request outside the feasible set

    def available(self, i_limit):
        return self.k_t * i_limit * (1 - self.reserve) - self.d_margin

    def feasible_interval(self, tau_av, tau_extra, extra_fn=None):
        """Largest interval of 1-deg grid points around 0 that the axis can hold."""
        probe = tuple(round(extra_fn(q), 3) for q in (-1.2, 0.0, 1.2)) if extra_fn else ()
        key = (round(tau_av, 3), round(tau_extra, 3), probe)
        if self._cache and self._cache[0] == key:
            return self._cache[1]
        ok = [abs(static_load(q, self.tau_g, tau_extra, extra_fn)) + self.tau_c <= tau_av
              and abs(q) <= self.q_range for q in self._grid]
        i0 = min(range(len(ok)), key=lambda i: (not ok[i], abs(self._grid[i])))
        if not ok[i0]:
            iv = (0.0, 0.0)                      # nothing holdable: park at 0 (passive side)
        else:
            lo = hi = i0
            while lo > 0 and ok[lo - 1]:
                lo -= 1
            while hi < len(ok) - 1 and ok[hi + 1]:
                hi += 1
            iv = (self._grid[lo], self._grid[hi])
        self._cache = (key, iv)
        return iv

    def _s_max(self, q, v, a, tau_av, tau_extra, extra_fn, tau_couple):
        """Largest path speed s in [0, 1] at which this plan point is feasible:
        J|a| s^2 + b|v| s <= tau_av - hold - |tau_couple| - J|v| s_rate/2
        (the last term reserves torque for changing s at up to half its max rate)."""
        hold = abs(static_load(q, self.tau_g, tau_extra, extra_fn)) + self.tau_c
        m = tau_av - hold - abs(tau_couple) - 0.5 * self.J * abs(v) * self.s_rate
        if m <= 0:
            return 0.0
        A, B = self.J * abs(a), self.b * abs(v)
        s = 1.0 if A < 1e-12 else (-B + math.sqrt(B * B + 4 * A * m)) / (2 * A)
        if A < 1e-12 and B > 1e-12:
            s = m / B
        if abs(v) > 1e-9:
            s = min(s, self.v_max / abs(v))
        return max(0.0, min(1.0, s))

    def step(self, ts, ref_at, i_limit, tau_extra=0.0, tau_couple=0.0, extra_fn=None,
             saturated=False):
        tau_av = self.available(i_limit)
        if saturated:
            self.kappa = max(self.kappa_min, self.kappa - self.kappa_drop * ts)
        else:
            self.kappa = min(1.0, self.kappa + self.kappa_rise * ts)
        lo, hi = self.feasible_interval(tau_av, tau_extra, extra_fn)

        # -- choose path speed s with look-ahead braking -----------------------
        s_allow = 1.0
        for j in range(self.n_look + 1):
            dsig = self.horizon * j / self.n_look
            qp, vp, ap = ref_at(self.sigma + dsig)
            if not (lo <= qp <= hi):
                continue                         # handled by projection, not by speed
            s_j = self._s_max(qp, vp, ap, tau_av, tau_extra, extra_fn, tau_couple if j == 0 else 0.0)
            # kinematic braking: s_now^2 <= s_j^2 + 2 * s_rate * dsigma
            s_allow = min(s_allow, math.sqrt(s_j * s_j + 2 * self.s_rate * dsig))
        s_allow *= self.kappa
        s_new = max(self.s - self.s_rate * ts, min(s_allow, self.s + self.s_rate * ts, 1.0))
        s_dot = (s_new - self.s) / ts
        self.s = s_new
        self.sigma += self.s * ts
        self.limited = self.s < 0.999

        # -- planned state at the scaled clock, projected onto F ---------------
        qp, vp, ap = ref_at(self.sigma)
        q_t = min(hi, max(lo, qp))
        self.rejected = q_t != qp
        v_t = 0.0 if self.rejected else vp * self.s
        a_t = 0.0 if self.rejected else ap * self.s ** 2 + vp * s_dot

        # -- follower: exact when unprojected, smooths projection edges ---------
        a = a_t + self.w ** 2 * (q_t - self.q) + 2 * self.w * (v_t - self.v)
        v = max(-self.v_max, min(self.v_max, self.v + a * ts))
        self.q += 0.5 * (self.v + v) * ts
        self.v, self.a = v, a
        return self.q, self.v, self.a


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
    limit, ask the yaw planner for a smaller amplitude (x shrink per trigger).
    Covers coupling that is larger than modelled (Phase 1 finding)."""

    def __init__(self, window=0.5, sat_frac=0.05, shrink=0.8, hold_off=0.6):
        self.window, self.sat_frac, self.shrink, self.hold_off = window, sat_frac, shrink, hold_off

    def reset(self):
        self.hist = []
        self.t_last = -1e9

    def update(self, t, saturated):
        self.hist.append((t, saturated))
        while self.hist and self.hist[0][0] < t - self.window:
            self.hist.pop(0)
        frac = sum(s for _, s in self.hist) / len(self.hist)
        if frac > self.sat_frac and t - self.t_last > self.hold_off and \
                self.hist[-1][0] - self.hist[0][0] > 0.8 * self.window:
            self.t_last = t
            self.hist.clear()
            return self.shrink
        return None
