"""Message and controller interfaces shared by the simulator and controllers.

Units: rad, rad/s, A, s. Only information that would exist on the real robot at
that instant is passed to a controller: delayed, quantized feedback plus the
planned reference. The true plant state is never visible to controllers.
"""
from dataclasses import dataclass


# Recovery by fault class (EXECUTION_PLAN.md Packet 2B). The drive re-arms
#   comm      after fresh, valid, aligned commands persist (the link is back)
#   motion    as comm, once the local speed estimate is back below the limit
#   thermal   as comm, once the winding has cooled below trip - 10 K
#   tracking  as comm, and only after the host acknowledged this fault id on every
#             command of the re-arm dwell: re-aligning the reference is not evidence
#             the request is feasible (the host acknowledges a predicted-feasible catch)
# Unknown fault names are treated as tracking faults (the conservative class).
FAULT_CLASS = {"cmd_timeout": "comm", "invalid_command": "comm", "overspeed": "motion",
               "overtemp": "thermal", "watchdog_trip": "tracking"}


def fault_class(name):
    return FAULT_CLASS.get(name, "tracking") if name else ""


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
    fault: str = ""      # latched drive fault ("" none), see FAULT_CLASS
    fault_id: int = 0    # increments at every latch; a tracking fault re-arms only
                         # when the host acknowledges this id (Command.ack)
    locked: bool = False  # repeated tracking/motion faults: no re-arm in this run


@dataclass(frozen=True)
class Command:
    """Sent by the host at its control rate."""
    t_cmd: float         # host time the command was computed for
    i_cmd: float         # requested current, A
    q_ref: float = 0.0   # carried so the drive can run its own tracking watchdog
    valid: bool = True   # host may explicitly request fallback
    ack: int = 0         # drive fault_id the host has decided on (see FAULT_CLASS)


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
