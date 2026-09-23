"""Runs A-E as simulator scenarios, plus the logged summaries they are compared to.

Trajectory details are NOT given in the brief; every choice below is an
ASSUMPTION, chosen from the Phase 0 inferences and kept fixed across designs.
"""
import math
from dataclasses import dataclass

from sim.config import SimConfig
from sim.trajectories import Hold, MinJerkSequence, RampedSine

R = math.radians


@dataclass(frozen=True)
class Scenario:
    name: str
    cfg: SimConfig
    roll: object
    yaw: object
    note: str


# Logged summaries (the only observed evidence). None = not reported.
OBSERVED = {
    "A": dict(rms=2.8, peak=6.1, i_peak=2.1, clip_pct=0.0, wd_trips=0),
    "B": dict(rms=4.2, peak=11.5, i_peak=3.1, clip_pct=8.0, wd_trips=0),
    "C": dict(rms=7.8, peak=20.7, i_peak=3.2, clip_pct=31.0, wd_trips=1),
    "D": dict(rms=9.1, peak=None, mean=4.6, i_peak=3.2, clip_pct=38.0, wd_trips=None),
    "E": dict(rms=12.6, peak=25.4, i_peak=2.4, clip_pct=None, wd_trips=None),
}

# D/E unknowns, chosen in Phase 1 as the values under which ONE legacy
# controller reproduces D and E together (see report/phase1_sim.md). They are
# working values, not identified parameters, and are swept in later tests.
M_PAYLOAD = 0.7       # kg, lateral 35 mm (Phase 0 bracket 0.5-1.8 kg)
SWEEP_HZ = 1.0        # +/-80 deg sweep frequency

# The reconstructed legacy controller used as the "before" reference.
LEGACY = dict(kp=1.5, kd=0.05, ki=0.0, f_vel=40.0, g_ff=0.12)
LEGACY_WATCHDOG = dict(wd_threshold=math.radians(20), wd_persist=0.020, wd_action="log")


def run_a(base):
    # 90 deg strokes in 0.5 s (Phase 0 implied 0.41-0.55 s), 0.5 s dwells.
    seq = [(R(45), 0.35, 0.5), (R(-45), 0.5, 0.5), (R(45), 0.5, 0.5), (R(-45), 0.5, 0.5),
           (0.0, 0.35, 0.5)]
    return Scenario("A", _dur(base, 5.5),
                    MinJerkSequence(0.0, seq), Hold(0.0), "shaped ±45° moves, yaw still")


def run_b(base):
    return Scenario("B", _dur(base, 8.0), Hold(0.0), RampedSine(R(75), 1.5, t_ramp=1.0),
                    "yaw ±75° @1.5 Hz, roll held")


def run_c(base):
    return Scenario("C", _dur(base, 8.0), Hold(0.0), RampedSine(R(75), 2.2, t_ramp=1.0),
                    "yaw ±75° @2.2 Hz, roll held")


def run_d(base, m=M_PAYLOAD):
    cfg = _dur(base, 12.0).with_(plant=dict(m_payload=m))
    return Scenario("D", cfg, RampedSine(R(80), SWEEP_HZ, t_ramp=2.0), Hold(0.0),
                    f"payload {m:.1f} kg shifted 35 mm, sweep ±80°")


def run_e(base, m=M_PAYLOAD):
    d = run_d(base, m)
    return Scenario("E", d.cfg.with_(drive=dict(i_limit=2.4)), d.roll, d.yaw,
                    "as D, derated to 2.4 A")


def _dur(cfg, T):
    from dataclasses import replace
    return replace(cfg, duration=T)


def all_runs(base=None):
    base = base or SimConfig()
    return [run_a(base), run_b(base), run_c(base), run_d(base), run_e(base)]
