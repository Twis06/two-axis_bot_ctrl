"""Finite motion sequences for completion-time comparisons (Packet 4A).

The periodic A-E requests never end, so "how much of the request was done"
can only be read from the governor's path clock, which a controller reports
about itself. A finite sequence has a physical end state: the plant either
visits every waypoint and settles at the last one, or it does not. That makes
completion time (sim.metrics.completion) a useful-motion measure that a
rejecting or stationary controller cannot pass.

Every sequence is an ASSUMPTION chosen for comparison, not a logged trajectory.
Duration = request end + SLACK_S so slowed (reshaped) controllers can still
finish inside the run and be timed rather than truncated.
"""
import math
from dataclasses import dataclass, replace

from exp import scenarios as S
from sim.config import SimConfig
from sim.trajectories import Hold, MinJerkSequence, RampedSine

R = math.radians
SLACK_S = 4.0


@dataclass(frozen=True)
class FiniteMotion:
    """Duck-compatible with exp.scenarios.Scenario (name, cfg, roll, yaw, note),
    plus what completion scoring needs."""
    name: str
    cfg: SimConfig
    roll: object
    yaw: object
    note: str
    waypoints: tuple      # ordered targets (rad), last = final rest position
    t_request: float      # s, requested completion time (end of last move + dwell)
    waypoint_windows: tuple = None  # optional [start,end) path-time windows; final end=None permits slack


def finite(name, base, q0, segments, yaw=None, note="", **cfg_groups):
    """Build a FiniteMotion from MinJerkSequence segments [(q, T_move, dwell)]."""
    roll = MinJerkSequence(q0, segments)
    t_req = roll.segs[-1][0] + roll.segs[-1][1]          # last move ends; dwell not required
    cfg = replace(base, duration=roll.t_end + SLACK_S, q0=q0)
    if cfg_groups:
        cfg = cfg.with_(**cfg_groups)
    return FiniteMotion(name, cfg, roll, yaw or Hold(0.0), note,
                        tuple(q for q, _, _ in segments), t_req)


def m1(base):
    return finite("M1", base, 0.0, [(R(45), 0.5, 0.3), (R(-45), 0.5, 0.3), (0.0, 0.5, 0.3)],
                  note="unloaded ±45° point-to-point, yaw still (A-like)")


def m2(base, m=S.M_PAYLOAD):
    return finite("M2", base, 0.0, [(R(60), 0.5, 0.3), (R(-60), 0.5, 0.3), (0.0, 0.5, 0.3)],
                  note=f"payload {m:.1f} kg shifted 35 mm, derated 2.4 A, ±60° in 0.5 s (E-like)",
                  plant=dict(m_payload=m), drive=dict(i_limit=2.4))


def m3(base):
    return finite("M3", base, 0.0, [(R(30), 0.6, 0.3), (R(-30), 0.6, 0.3), (0.0, 0.6, 0.3)],
                  yaw=RampedSine(R(75), 1.5, t_ramp=1.0),
                  note="±30° roll moves during yaw ±75° @1.5 Hz (B-like)")


def all_motions(base=None):
    base = base or SimConfig()
    return [m1(base), m2(base), m3(base)]
