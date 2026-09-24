"""Reference and yaw trajectories. Each exposes eval(t) -> (q, qd, qdd), analytic.

Yaw is treated as a prescribed kinematic input (the yaw axis is assumed to track
its own command perfectly); only its effect on roll is simulated.
"""
import math


class Hold:
    def __init__(self, q=0.0):
        self.q = q

    def eval(self, t):
        return self.q, 0.0, 0.0


class RampedSine:
    """A*sin(2*pi*f*t) with a raised-cosine amplitude ramp over t_ramp, so the
    start has no velocity/acceleration step (ASSUMPTION about the logged runs)."""

    def __init__(self, amp, f, t_ramp=1.0, offset=0.0, t0=0.0):
        self.A, self.w, self.Tr, self.off, self.t0 = amp, 2 * math.pi * f, t_ramp, offset, t0

    def eval(self, t):
        t = t - self.t0
        if t <= 0:
            return self.off, 0.0, 0.0
        w, A = self.w, self.A
        s, c = math.sin(w * t), math.cos(w * t)
        S, Sd, Sdd = A * s, A * w * c, -A * w * w * s
        if self.Tr > 0 and t < self.Tr:
            k = math.pi / self.Tr
            e = 0.5 * (1 - math.cos(k * t))
            ed = 0.5 * k * math.sin(k * t)
            edd = 0.5 * k * k * math.cos(k * t)
        else:
            e, ed, edd = 1.0, 0.0, 0.0
        return (self.off + e * S, ed * S + e * Sd, edd * S + 2 * ed * Sd + e * Sdd)


class MinJerkSequence:
    """Waypoint list [(q_target, move_time, dwell_after), ...] from q_start,
    each move a quintic minimum-jerk profile."""

    def __init__(self, q_start, segments):
        self.segs = []
        t, q = 0.0, q_start
        for q1, T, dwell in segments:
            self.segs.append((t, T, q, q1))
            t += T + dwell
            q = q1
        self.q_end, self.t_end = q, t

    def eval(self, t):
        q_hold = self.segs[0][2] if self.segs else self.q_end
        for t0, T, q0, q1 in self.segs:
            if t < t0:
                return q_hold, 0.0, 0.0
            if t < t0 + T:
                x = (t - t0) / T
                D = q1 - q0
                p = 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5
                pd = (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T
                pdd = (60 * x - 180 * x ** 2 + 120 * x ** 3) / T ** 2
                return q0 + D * p, D * pd, D * pdd
            q_hold = q1
        return q_hold, 0.0, 0.0


class GovernedYaw:
    """Yaw plan owned by the host: a base trajectory times an amplitude scale
    s(t) that the host may lower at run time (request_scale). Scale changes are
    quintic blends over t_blend, so qy stays C2 and the coupling torque it
    creates stays bounded. A request only affects the plan from its time onward.
    """

    def __init__(self, base, t_blend=0.5):
        self.base, self.t_blend = base, t_blend
        self.segs = [(0.0, 1.0, 1.0)]     # (t_start, s_from, s_to)
        self.scale_log = [(0.0, 1.0)]

    def _s(self, t):
        t0, s0, s1 = self.segs[0]
        for seg in self.segs:
            if seg[0] <= t:
                t0, s0, s1 = seg
        x = (t - t0) / self.t_blend
        if x >= 1 or s0 == s1:
            return s1, 0.0, 0.0
        T, D = self.t_blend, s1 - s0          # quintic blend: zero s' and s'' at both ends
        return (s0 + D * (10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5),
                D * (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4) / T,
                D * (60 * x - 180 * x ** 2 + 120 * x ** 3) / T ** 2)

    def scale(self, t):
        return self._s(t)[0]

    def request_scale(self, t, s_new):
        s_now = self._s(t)[0]
        s_new = max(0.0, min(s_now, s_new))      # the host only ever shrinks yaw here
        if s_now - s_new > 1e-6:
            self.segs.append((t, s_now, s_new))
            self.scale_log.append((t, s_new))

    def eval(self, t):
        b, bd, bdd = self.base.eval(t)
        s, sd, sdd = self._s(t)
        return s * b, sd * b + s * bd, sdd * b + 2 * sd * bd + s * bdd


class FollowingYaw:
    """Actual yaw motion that follows a host plan imperfectly: delayed by `lag`
    and scaled by `gain` (plan-mismatch tests of the look-ahead information mode).
    Scale requests go to the plan, as they would on the robot."""

    def __init__(self, plan, lag=0.010, gain=1.1):
        self.plan, self.lag, self.gain = plan, lag, gain

    def eval(self, t):
        q, qd, qdd = self.plan.eval(t - self.lag)
        return self.gain * q, self.gain * qd, self.gain * qdd
