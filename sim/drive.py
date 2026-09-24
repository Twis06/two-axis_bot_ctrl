"""Drive-side firmware model, ticking at 1 kHz next to the motor.

Fixed hardware behaviour (not a design choice):
  * newest received command wins; older overtaken messages are ignored
  * current target clamped to the active limit (nominal or derated)
  * 1 ms current-command delay before the current loop

Pluggable `DriveSafety` (the design choice, extended in Phase 2) decides what
target to use: the host command, or a local fallback.
"""
import math
from collections import deque


class DriveSafety:
    """Minimal drive safety: command timeout + optional tracking watchdog.

    Fallback = local velocity damping (i = -c*v/Kt) using the drive's own
    1 kHz encoder. With gravity restoring toward q=0 (J qdd = -tau_g sin q),
    zero/damped torque is a passive, safe state for this axis.
    """

    def __init__(self, cmd_timeout=0.010, damping=0.05, k_t_nom=0.14,
                 wd_threshold=None, wd_persist=0.020, wd_action="log"):
        self.cmd_timeout, self.damping, self.k_t = cmd_timeout, damping, k_t_nom
        self.wd_threshold, self.wd_persist, self.wd_action = wd_threshold, wd_persist, wd_action

    def reset(self):
        self.q_prev = None
        self.v = 0.0
        self.wd_since = None
        self.wd_trips = 0
        self.wd_latched = False
        self.events = []

    def local_velocity(self, q_enc, dt):
        if self.q_prev is not None:
            self.v += 0.2 * ((q_enc - self.q_prev) / dt - self.v)  # ~36 Hz LPF at 1 kHz
        self.q_prev = q_enc
        return self.v

    def current_limit(self):
        return float("inf")

    def decide(self, t, q_enc, cmd, cmd_age, dt, i_meas=0.0):
        """Returns (i_target_unclamped, mode)."""
        v = self.local_velocity(q_enc, dt)
        fallback_i = -self.damping * v / self.k_t
        if cmd is None or cmd_age > self.cmd_timeout or not cmd.valid:
            if cmd is not None and cmd_age > self.cmd_timeout:
                self._event(t, "cmd_timeout")
            return fallback_i, 1
        if self.wd_threshold is not None:
            if abs(q_enc - cmd.q_ref) > self.wd_threshold:
                if self.wd_since is None:
                    self.wd_since = t
                elif t - self.wd_since >= self.wd_persist and not self.wd_latched:
                    self.wd_trips += 1
                    self.wd_latched = True
                    self.events.append((t, "watchdog_trip"))
            else:
                self.wd_since = None
                self.wd_latched = False
            if self.wd_latched and self.wd_action == "fallback":
                return fallback_i, 1
        return cmd.i_cmd, 0

    def _event(self, t, name):
        if not self.events or self.events[-1][1] != name or t - self.events[-1][0] > 0.05:
            self.events.append((t, name))


class Drive:
    def __init__(self, dc, safety, t_cmd_delay, f_drive):
        self.dc, self.safety = dc, safety
        n_delay = int(round(t_cmd_delay * f_drive))
        self.fifo = deque([0.0] * n_delay) if n_delay > 0 else None
        self.dt = 1.0 / f_drive
        self.safety.reset()
        self.active_limit = dc.i_limit

    def i_limit(self, t):
        lim = self.dc.i_limit
        for t0, new in self.dc.derate_schedule:
            if t >= t0:
                lim = new
        return lim

    def tick(self, t, q_enc, latest_cmd, i_meas=0.0):
        """latest_cmd: (seq, t_sent, t_arrive, Command) or None.
        Returns dict with the current target for the next 1 ms."""
        cmd = latest_cmd[3] if latest_cmd else None
        cmd_age = t - cmd.t_cmd if cmd else float("inf")
        raw, mode = self.safety.decide(t, q_enc, cmd, cmd_age, self.dt, i_meas)
        if not math.isfinite(raw):
            # Clamping NaN with min/max would return the limit itself: refuse it.
            raw, mode = 0.0, 1
            self.safety._event(t, "nonfinite_target")
        lim = min(self.i_limit(t), self.safety.current_limit())
        self.active_limit = lim
        clipped = abs(raw) >= lim * (1 - 1e-9)   # at or beyond the limit
        tgt = max(-lim, min(lim, raw))
        if self.fifo is not None:
            self.fifo.append(tgt)
            applied = self.fifo.popleft()
        else:
            applied = tgt
        # A newly reduced limit also applies to commands already in the delay line.
        applied = max(-lim, min(lim, applied))
        return dict(raw=raw, tgt=tgt, applied=applied, clipped=clipped, lim=lim,
                    mode=mode, cmd_age=cmd_age, fault=getattr(self.safety, "fault", None) or "",
                    fault_id=getattr(self.safety, "fault_id", 0),
                    locked=getattr(self.safety, "locked", False))
