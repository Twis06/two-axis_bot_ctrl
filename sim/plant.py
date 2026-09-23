"""Continuous-time roll-axis plant with the specified current lag and a
voltage-headroom limit on the current slew.

State x = (q, qd, i).

    J qdd = Kt i - b qd - tau_c tanh(qd/v_f) - tau_g sin q - tau_lat cos q
            - (k_yv qy_d + k_ya qy_dd) + d

Current: the drive's current loop follows its target with the specified
first-order lag, di/dt = (i_tgt - i)/tau_i, *unless* that needs more voltage
than is available: V = R i + L di/dt + Ke qd must satisfy |V| <= V_avail.
When it does not, di/dt is clipped to the value the saturated voltage gives.
This keeps the brief's 1.2 ms lag as the nominal behaviour and turns R, L,
Ke and V_bus into a binding constraint only when they should be.
"""
import math


class RollPlant:
    def __init__(self, pc):
        self.pc = pc
        self.J = pc.J_total
        self.tau_lat = pc.tau_lat
        self.v_avail = pc.v_avail

    # -- torque decomposition (used for logging and tests) ----------------
    def load_torques(self, q, qd, qy_d, qy_dd):
        pc = self.pc
        return dict(
            visc=pc.b * qd,
            fric=pc.tau_c * math.tanh(qd / pc.v_fric),
            grav=pc.tau_g * math.sin(q) + self.tau_lat * math.cos(q),
            couple=pc.k_yv * qy_d + pc.k_ya * qy_dd,
        )

    def di_dt(self, i, i_tgt, qd):
        pc = self.pc
        di = (i_tgt - i) / pc.tau_i
        if not pc.voltage_limit:
            return di, False
        v = pc.R * i + pc.L * di + pc.k_e * qd
        if v > self.v_avail:
            return (self.v_avail - pc.R * i - pc.k_e * qd) / pc.L, True
        if v < -self.v_avail:
            return (-self.v_avail - pc.R * i - pc.k_e * qd) / pc.L, True
        return di, False

    def deriv(self, q, qd, i, i_tgt, qy_d, qy_dd, d):
        pc = self.pc
        tau = (pc.k_t * i - pc.b * qd - pc.tau_c * math.tanh(qd / pc.v_fric)
               - pc.tau_g * math.sin(q) - self.tau_lat * math.cos(q)
               - (pc.k_yv * qy_d + pc.k_ya * qy_dd) + d)
        di, _ = self.di_dt(i, i_tgt, qd)
        return qd, tau / self.J, di

    def step(self, x, i_tgt, yaw, t, h, d):
        """One RK4 step of size h. `yaw` is a trajectory with eval(t)."""
        q, qd, i = x
        _, y1d, y1dd = yaw.eval(t)
        _, y2d, y2dd = yaw.eval(t + 0.5 * h)
        _, y3d, y3dd = yaw.eval(t + h)
        k1 = self.deriv(q, qd, i, i_tgt, y1d, y1dd, d)
        k2 = self.deriv(q + 0.5 * h * k1[0], qd + 0.5 * h * k1[1], i + 0.5 * h * k1[2],
                        i_tgt, y2d, y2dd, d)
        k3 = self.deriv(q + 0.5 * h * k2[0], qd + 0.5 * h * k2[1], i + 0.5 * h * k2[2],
                        i_tgt, y2d, y2dd, d)
        k4 = self.deriv(q + h * k3[0], qd + h * k3[1], i + h * k3[2], i_tgt, y3d, y3dd, d)
        return (q + h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]),
                qd + h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]),
                i + h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]))
