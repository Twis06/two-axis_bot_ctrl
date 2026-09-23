"""Shared helpers for experiments: run a controller on a scenario with fresh
stateful objects (governable yaw planner, supervisor) every time."""
from ctrl.supervisor import DriveSupervisor
from sim.drive import DriveSafety
from sim.engine import simulate
from sim.metrics import summarize
from sim.trajectories import GovernedYaw
from exp import scenarios as S


def run(sc, make_ctrl, seed=1, supervisor="design", governed_yaw=True, cfg=None):
    cfg = cfg or sc.cfg
    yaw = GovernedYaw(sc.yaw) if governed_yaw else sc.yaw
    if supervisor == "design":
        safety = DriveSupervisor()
    elif supervisor == "legacy":
        safety = DriveSafety(**S.LEGACY_WATCHDOG)
    else:
        safety = supervisor
    log = simulate(cfg, make_ctrl(), sc.roll, yaw, seed=seed, safety=safety)
    return log, summarize(log)
