"""Message and controller interfaces shared by the simulator and controllers.

Units: rad, rad/s, A, s. Only information that would exist on the real robot at
that instant is passed to a controller: delayed, quantized feedback plus the
planned reference. The true plant state is never visible to controllers.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Feedback:
    """Sent by the drive every 1 kHz tick; arrives at the host after CAN latency."""
    seq: int
    t_meas: float        # drive timestamp of the sample (clocks assumed synced)
    q: float             # roll encoder, quantized
    qy: float            # yaw encoder, quantized
    i_meas: float        # measured motor current
    i_limit: float       # drive's active current limit (reports derating)
    mode: int            # drive mode: 0 normal, 1 fallback


@dataclass(frozen=True)
class Command:
    """Sent by the host at its control rate."""
    t_cmd: float         # host time the command was computed for
    i_cmd: float         # requested current, A
    q_ref: float = 0.0   # carried so the drive can run its own tracking watchdog
    valid: bool = True   # host may explicitly request fallback


@dataclass(frozen=True)
class Context:
    """What the host knows at time t besides feedback."""
    t: float
    ref: tuple           # planned roll reference (q, qd, qdd)
    yaw_plan: tuple      # planned yaw (q, qd, qdd) - host commands yaw, so it knows it
    ref_at: object = None      # callable t -> roll plan (look-ahead of the host's own plan)
    yaw_at: object = None      # callable t -> yaw plan
    yaw_planner: object = None  # handle to request a smaller yaw amplitude, or None


class Controller:
    name = "base"

    def reset(self, cfg):
        """Called once before a run with the SimConfig (nominal params only
        should be used for design; cfg.plant holds the TRUE, possibly perturbed,
        plant and must not be read by controllers)."""
        self.telemetry = {}

    def update(self, ctx, fb):
        """fb is the newest Feedback delivered to the host, or None."""
        raise NotImplementedError
