"""Drive-side supervisor (1 kHz, next to the motor). Owns every hard limit and
fault; the host controller cannot override it.

Faults -> FALLBACK (local velocity damping; passive because gravity restores
toward q = 0):
  * command timeout      no fresh host command for cmd_timeout
  * tracking watchdog    |q - q_ref| > wd_threshold for wd_persist   (latched)
  * over-speed           |v| > v_max                                 (latched)
  * over-temperature     winding model > T_trip                      (latched)
Latched faults re-arm only when the host has re-aligned its reference
(|q - q_ref| < rearm_err for rearm_time) and commands are fresh.

Thermal derating: first-order winding model, R rising with temperature; the
current limit falls linearly from i_nom at T_derate to i_derated at T_full.
Thermal constants are ASSUMPTIONS (not given in the brief).
"""
import math

from sim import params as P
from sim.drive import DriveSafety


class DriveSupervisor(DriveSafety):
    def __init__(self, cmd_timeout=0.010, damping=0.05, k_t_nom=P.K_T,
                 wd_threshold=math.radians(12), wd_persist=0.040,
                 v_max=25.0, rearm_err=math.radians(2), rearm_time=0.05,
                 thermal=True, R0=P.R_NOM, R_th=4.0, tau_th=30.0, T_amb=25.0,
                 T_derate=90.0, T_full=110.0, T_trip=130.0,
                 i_nom=P.I_MAX, i_derated=P.I_DERATED):
        super().__init__(cmd_timeout, damping, k_t_nom, wd_threshold, wd_persist, "fallback")
        self.v_max, self.rearm_err, self.rearm_time = v_max, rearm_err, rearm_time
        self.thermal, self.R0, self.R_th, self.C_th = thermal, R0, R_th, tau_th / R_th
        self.T_amb, self.T_derate, self.T_full, self.T_trip = T_amb, T_derate, T_full, T_trip
        self.i_nom, self.i_der = i_nom, i_derated

    def reset(self):
        super().reset()
        self.fault = None
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

    # -- decision -----------------------------------------------------------
    def decide(self, t, q_enc, cmd, cmd_age, dt, i_meas=0.0):
        v = self.local_velocity(q_enc, dt)
        if self.thermal:
            self.thermal_step(i_meas, dt)
        fallback_i = -self.damping * v / self.k_t
        fresh = cmd is not None and cmd_age <= self.cmd_timeout and cmd.valid
        if cmd is not None and cmd_age > self.cmd_timeout:
            self._event(t, "cmd_timeout")

        if self.fault is None:
            err = abs(q_enc - cmd.q_ref) if fresh else 0.0
            if fresh and err > self.wd_threshold:
                self.wd_since = t if self.wd_since is None else self.wd_since
                if t - self.wd_since >= self.wd_persist:
                    self._latch(t, "watchdog_trip")
                    self.wd_trips += 1
            else:
                self.wd_since = None
            if abs(v) > self.v_max:
                self._latch(t, "overspeed")
            if self.thermal and self.T > self.T_trip:
                self._latch(t, "overtemp")
        else:
            # Re-arm once the host reference has been re-aligned with the axis
            if fresh and abs(q_enc - cmd.q_ref) < self.rearm_err and \
                    (not self.thermal or self.T < self.T_trip - 10):
                self.ok_since = t if self.ok_since is None else self.ok_since
                if t - self.ok_since >= self.rearm_time:
                    self.events.append((t, f"rearm_after_{self.fault}"))
                    self.fault, self.ok_since, self.wd_since = None, None, None
            else:
                self.ok_since = None

        if self.fault is not None or not fresh:
            return fallback_i, 1
        return cmd.i_cmd, 0

    def _latch(self, t, name):
        self.fault = name
        self.ok_since = None
        self.events.append((t, name))
