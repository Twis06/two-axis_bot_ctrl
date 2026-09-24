"""Drive-side supervisor (1 kHz, next to the motor). Owns every hard limit and
fault; the host controller cannot override it.

Faults -> FALLBACK (local velocity damping; passive because gravity restores
toward q = 0):
  * command timeout      no fresh host command for cmd_timeout
  * tracking watchdog    |q - q_ref| > wd_threshold for wd_persist   (latched)
  * over-speed           |v| > v_max                                 (latched)
  * over-temperature     winding model > T_trip                      (latched)
Re-arm is by fault class (ctrl.interfaces.FAULT_CLASS). Every class needs
fresh, valid commands whose reference is aligned with the axis (|q - q_ref| <
rearm_err) for rearm_time, and the condition that tripped it to have cleared.
A tracking fault additionally needs the host's acknowledgement of this fault id:
moving the reference onto the measured position is not, by itself, evidence
that the request is feasible. The host acknowledges only a replacement request
it predicts feasible (a brake-to-rest and hold from the measured state). A first
draft also required the axis at rest; under continuing yaw coupling the damped
axis never came to rest and stayed in passive fallback (Packet 2B). Repeated
tracking/motion faults (lockout_trips within lockout_window) lock the drive in
fallback for the rest of the run.

Thermal derating: first-order winding model, R rising with temperature; the
current limit falls linearly from i_nom at T_derate to i_derated at T_full.
Thermal constants are ASSUMPTIONS (not given in the brief).
"""
import math

from ctrl.interfaces import fault_class
from sim import params as P
from sim.drive import DriveSafety


class DriveSupervisor(DriveSafety):
    def __init__(self, cmd_timeout=0.010, damping=0.05, k_t_nom=P.K_T,
                 wd_threshold=math.radians(12), wd_persist=0.040,
                 v_max=25.0, rearm_err=math.radians(2), rearm_time=0.05,
                 thermal=True, R0=P.R_NOM, R_th=4.0, tau_th=30.0, T_amb=25.0,
                 T_derate=90.0, T_full=110.0, T_trip=130.0,
                 i_nom=P.I_MAX, i_derated=P.I_DERATED,
                 lockout_trips=3, lockout_window=30.0):
        super().__init__(cmd_timeout, damping, k_t_nom, wd_threshold, wd_persist, "fallback")
        self.v_max, self.rearm_err, self.rearm_time = v_max, rearm_err, rearm_time
        self.thermal, self.R0, self.R_th, self.C_th = thermal, R0, R_th, tau_th / R_th
        self.T_amb, self.T_derate, self.T_full, self.T_trip = T_amb, T_derate, T_full, T_trip
        self.i_nom, self.i_der = i_nom, i_derated
        self.lockout_trips, self.lockout_window = lockout_trips, lockout_window

    def reset(self):
        super().reset()
        self.fault = None
        self.fault_id = 0
        self.locked = False
        self.trip_times = []      # tracking/motion latches (lockout count)
        self.seen_command = False
        self.ok_since = None
        self.T = self.T_amb
        self.limit = self.i_nom

    # -- thermal ------------------------------------------------------------
    def thermal_step(self, i, dt):
        R = self.R0 * (1 + 0.0039 * (self.T - 25.0))
        self.T += dt * (i * i * R - (self.T - self.T_amb) / self.R_th) / self.C_th
        x = min(1.0, max(0.0, (self.T - self.T_derate) / (self.T_full - self.T_derate)))
        self.limit = self.i_nom - x * (self.i_nom - self.i_der)
        return self.T

    def current_limit(self):
        return self.limit if self.thermal else float("inf")

    @property
    def fault_class(self):
        return fault_class(self.fault)

    # -- decision -----------------------------------------------------------
    def decide(self, t, q_enc, cmd, cmd_age, dt, i_meas=0.0):
        v = self.local_velocity(q_enc, dt)
        if self.thermal:
            self.thermal_step(i_meas, dt)
        fallback_i = -self.damping * v / self.k_t
        fresh = (cmd is not None and 0 <= cmd_age <= self.cmd_timeout and cmd.valid
                 and all(math.isfinite(x) for x in (cmd.t_cmd, cmd.i_cmd, cmd.q_ref)))
        if fresh:
            self.seen_command = True
        elif self.seen_command and self.fault is None:
            self._latch(t, "cmd_timeout" if cmd_age > self.cmd_timeout else "invalid_command")

        if self.fault is None:
            err = abs(q_enc - cmd.q_ref) if fresh else 0.0
            if fresh and err > self.wd_threshold:
                self.wd_since = t if self.wd_since is None else self.wd_since
                if t - self.wd_since >= self.wd_persist:
                    self._latch(t, "watchdog_trip")
                    self.wd_trips += 1
            else:
                self.wd_since = None
            # One latch per tick: the first cause is the one reported and re-armed on.
            if self.fault is None and abs(v) > self.v_max:
                self._latch(t, "overspeed")
            if self.fault is None and self.thermal and self.T > self.T_trip:
                self._latch(t, "overtemp")
        elif self._rearm_ok(fresh, q_enc, v, cmd):
            self.ok_since = t if self.ok_since is None else self.ok_since
            if t - self.ok_since >= self.rearm_time:
                self.events.append((t, f"rearm_after_{self.fault}"))
                self.fault, self.ok_since, self.wd_since = None, None, None
        else:
            self.ok_since = None

        if self.fault is not None or not fresh:
            return fallback_i, 1
        return cmd.i_cmd, 0

    def _rearm_ok(self, fresh, q_enc, v, cmd):
        """Re-arm conditions for the latched fault's class (module docstring)."""
        if self.locked or not fresh or abs(q_enc - cmd.q_ref) >= self.rearm_err:
            return False
        cls = self.fault_class
        if abs(v) >= 0.8 * self.v_max:
            return False
        if self.thermal and self.T >= self.T_trip - 10:
            return False
        if cls == "tracking":
            return cmd.ack == self.fault_id
        return True

    def _latch(self, t, name):
        self.fault = name
        self.fault_id += 1
        self.ok_since = None
        self.events.append((t, name))
        if fault_class(name) in ("tracking", "motion"):
            self.trip_times = [x for x in self.trip_times if t - x < self.lockout_window] + [t]
            if len(self.trip_times) >= self.lockout_trips and not self.locked:
                self.locked = True
                self.events.append((t, "lockout"))
