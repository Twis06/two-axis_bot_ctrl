"""A plausible stand-in for the controller that produced Runs A-E.

Feedback-only PID on the delayed encoder, no feed-forward, naive integrator
(no anti-windup), unaware of drive derating. It is NOT a design proposal; it
exists so the simulator can be checked against the *shape* of the logged
evidence. One gain set is used for every run.
"""
import math

from ctrl.interfaces import Command, Controller


class LegacyPID(Controller):
    name = "legacy_pid"

    def __init__(self, kp=1.5, kd=0.05, ki=4.0, f_vel=40.0, k_t=0.14, i_max=3.2, g_ff=0.0):
        self.kp, self.kd, self.ki, self.f_vel = kp, kd, ki, f_vel
        self.g_ff = g_ff     # nominal gravity feed-forward amplitude (0 = none)
        self.k_t, self.i_max = k_t, i_max

    def reset(self, cfg):
        self.telemetry = {}
        self.ts = 1.0 / cfg.timing.f_ctrl
        self.a = math.exp(-2 * math.pi * self.f_vel * self.ts)
        self.q_prev = None
        self.v = 0.0
        self.integ = 0.0
        self.last_seq = -1

    def update(self, ctx, fb):
        q_ref, qd_ref, _ = ctx.ref
        if fb is None:
            return Command(ctx.t, 0.0, q_ref)
        if fb.seq != self.last_seq:          # new sample: update velocity estimate
            if self.q_prev is not None:
                raw = (fb.q - self.q_prev[0]) / max(fb.t_meas - self.q_prev[1], 1e-4)
                self.v = self.a * self.v + (1 - self.a) * raw
            self.q_prev = (fb.q, fb.t_meas)
            self.last_seq = fb.seq
        e = q_ref - fb.q
        self.integ += e * self.ts
        tau = (self.kp * e + self.kd * (qd_ref - self.v) + self.ki * self.integ
               + self.g_ff * math.sin(fb.q))
        i = max(-self.i_max, min(self.i_max, tau / self.k_t))
        self.telemetry = {"v_hat": self.v, "integ": self.integ}
        return Command(ctx.t, i, q_ref)
