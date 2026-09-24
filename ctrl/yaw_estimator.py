"""Causal axis kinematics from a delayed, quantized encoder (host side, Packet 2B).

Used for yaw (the coupling feed-forward) and for roll (the velocity from which a
tracking-fault catch is planned).

Input: the drive's yaw encoder as it arrives in Feedback -- 14-bit, timestamped at
the drive (t_meas), delayed by CAN latency, only the newest sample per host tick.
Model: constant acceleration with white jerk (spectral density q_jerk), a
three-state Kalman filter at the measurement timestamps. The coupling torque
is then predicted at the time the command takes effect by constant-acceleration
extrapolation from the last measurement: nothing about the future yaw plan is used.

Deterministic state estimation for Task 2, not the Task 3 learning feature.
q_jerk = 1e5 rad^2/s^5 was chosen on the B/C yaw sines (1.5 / 2.2 Hz, 75 deg)
from 1e4-1e8: it leaves ~10-13 % of the coupling RMS (Simulated, scratchpad
prototype); higher values pass more quantization noise, lower ones lag.
"""
import math

from sim import params as P


class KinematicEstimator:
    # warmup: from a cold start the coupling estimate reached its steady residual
    # within ~12 ms on a 45 rad/s ramp and on the C sine (Simulated, Packet 2B).
    def __init__(self, q_jerk=1e5, lsb=P.ENC_LSB, gap_reset=0.05, warmup=0.012,
                 v0_sd=30.0, a0_sd=500.0):
        if not (q_jerk > 0 and lsb > 0 and gap_reset > 0 and warmup >= 0):
            raise ValueError("KinematicEstimator needs positive q_jerk, lsb and gap_reset")
        self.q_jerk, self.r = q_jerk, lsb * lsb / 12.0     # uniform quantization noise
        self.gap_reset, self.warmup = gap_reset, warmup
        self.P0 = (lsb * lsb, v0_sd * v0_sd, a0_sd * a0_sd)
        self.reset()

    def reset(self):
        self.x = None             # [q, v, a] at t_last
        self.t_last = self.t_init = None

    def _init(self, t, z):
        self.x = [z, 0.0, 0.0]
        self.P = [[self.P0[0], 0, 0], [0, self.P0[1], 0], [0, 0, self.P0[2]]]
        self.t_last = self.t_init = t

    @property
    def ready(self):
        return self.x is not None and self.t_last - self.t_init >= self.warmup

    def update(self, t_meas, z):
        """Fuse one encoder sample; older or repeated timestamps are ignored.
        A gap longer than gap_reset restarts the filter (velocity/acceleration
        extrapolated that far are not an estimate)."""
        if not (math.isfinite(t_meas) and math.isfinite(z)):
            return False
        if self.x is None or t_meas - self.t_last > self.gap_reset:
            self._init(t_meas, z)
            return True
        dt = t_meas - self.t_last
        if dt <= 0:
            return False
        x, Pm, q = self.x, self.P, self.q_jerk
        F = ((1, dt, dt * dt / 2), (0, 1, dt), (0, 0, 1))
        Q = ((dt ** 5 / 20, dt ** 4 / 8, dt ** 3 / 6),
             (dt ** 4 / 8, dt ** 3 / 3, dt ** 2 / 2),
             (dt ** 3 / 6, dt ** 2 / 2, dt))
        x = [sum(F[i][j] * x[j] for j in range(3)) for i in range(3)]
        FP = [[sum(F[i][k] * Pm[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        Pm = [[sum(FP[i][k] * F[j][k] for k in range(3)) + q * Q[i][j] for j in range(3)]
              for i in range(3)]
        # Innovation wrapped to (-pi, pi]: an absolute encoder rolls over at 2 pi.
        y = (z - x[0] + math.pi) % (2 * math.pi) - math.pi
        S = Pm[0][0] + self.r
        K = [Pm[i][0] / S for i in range(3)]
        self.x = [x[i] + K[i] * y for i in range(3)]
        self.P = [[Pm[i][j] - K[i] * Pm[0][j] for j in range(3)] for i in range(3)]
        self.t_last = t_meas
        return True

    def predict(self, t):
        """(q, v, a) at time t by constant-acceleration extrapolation, or None."""
        if self.x is None:
            return None
        q, v, a = self.x
        dt = t - self.t_last
        return q + v * dt + 0.5 * a * dt * dt, v + a * dt, a


YawEstimator = KinematicEstimator
