"""Multi-rate simulation loop.

Timeline of one 1 kHz drive tick at t = k ms:
  1. drive samples the roll and yaw encoders, sends Feedback(seq=k) over CAN
  2. drive takes the newest delivered Command, runs DriveSafety, clamps to the
     active current limit, pushes it into the 1 ms command-delay line
  3. if this is a host tick (every f_drive/f_ctrl ms): host reads the newest
     delivered Feedback, computes a Command, releases it at t + t_compute
  4. the plant is integrated over [t, t+1 ms) with RK4 at dt_sim, holding the
     applied current target and the ZOH disturbance sample
Everything random draws from one numpy Generator seeded by `seed`.
"""
import math
from dataclasses import replace

import numpy as np

from ctrl.interfaces import Command, Context, Feedback
from sim.drive import Drive, DriveSafety
from sim.plant import RollPlant
from sim.sensing import Encoder, bounded_disturbance
from sim.timing import Channel, LatencyModel

LOG_KEYS = ("t", "q", "qd", "i", "i_raw", "i_tgt", "clipped", "vlim", "i_lim", "mode",
            "q_ref", "qd_ref", "qdd_ref", "err", "q_enc", "qy", "qyd", "qydd", "tau_couple",
            "d", "cmd_age", "fb_age", "burst", "qy_plan", "fault_id", "locked", "T_wind")


class Log(dict):
    """dict of equal-length numpy arrays + metadata (.events, .meta)."""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e


def simulate(cfg, controller, roll_ref, yaw, seed=0, safety=None, yaw_plan=None):
    """yaw drives the plant and the yaw encoder. yaw_plan, if given, is what the
    host believes yaw will do (Context.yaw_at / yaw_planner); default: yaw itself,
    i.e. a yaw axis that follows its plan exactly (plan-mismatch tests pass both)."""
    plan = yaw if yaw_plan is None else yaw_plan
    rng = np.random.default_rng(seed)
    tc, pc = cfg.timing, cfg.plant
    plant = RollPlant(pc)
    enc = Encoder(cfg.sensor.enc_bits, cfg.sensor.enc_offset)
    enc_y = Encoder(cfg.sensor.enc_bits)
    safety = safety or DriveSafety(cmd_timeout=cfg.drive.cmd_timeout,
                                   damping=cfg.drive.fallback_damping)
    drive = Drive(cfg.drive, safety, tc.t_cmd_delay, tc.f_drive)

    # Independent latency processes per direction (bursts hit the shared bus
    # in reality; independent draws are the harsher assumption for ordering).
    lat_cmd = LatencyModel(replace(tc, blackout=tc.blackout + tc.command_blackout), rng)
    lat_fb = LatencyModel(replace(tc, blackout=tc.blackout + tc.feedback_blackout), rng)
    ch_cmd = Channel(lat_cmd)
    ch_fb = Channel(lat_fb, enabled=tc.feedback_over_can)
    d_seq = bounded_disturbance(rng, cfg.duration, pc.d_amp, pc.d_bw_hz, fs=tc.f_drive)

    dt_tick = 1.0 / tc.f_drive
    n_sub = int(round(dt_tick / tc.dt_sim))
    h = dt_tick / n_sub
    div = int(round(tc.f_drive / tc.f_ctrl))
    assert abs(tc.f_drive / div - tc.f_ctrl) < 1e-9, "f_ctrl must divide f_drive"
    N = int(round(cfg.duration * tc.f_drive))

    controller.reset(cfg)
    log = Log({k: np.zeros(N) for k in LOG_KEYS})
    tele = {}
    x = (cfg.q0, 0.0, 0.0)
    host_seq = 0
    drive_mode, drive_fault = 0, ("", 0, False)
    fb_age = float("nan")

    for k in range(N):
        t = k * dt_tick
        q, qd, i = x
        qy, qyd, qydd = yaw.eval(t)
        q_enc = enc.read(q)

        # 1-2: drive
        ch_fb.send(t, k, Feedback(k, t, q_enc, enc_y.read(qy), i, drive.active_limit, drive_mode,
                                  *drive_fault))
        dr = drive.tick(t, q_enc, ch_cmd.poll(t), i)
        drive_mode, drive_fault = dr["mode"], (dr["fault"], dr["fault_id"], dr["locked"])

        # 3: host
        if k % div == 0:
            latest = ch_fb.poll(t)
            fb = latest[3] if latest else None
            if fb is not None:
                fb_age = t - fb.t_meas
            ref = roll_ref.eval(t)
            cmd = controller.update(Context(t, ref, plan.eval(t), roll_ref.eval, plan.eval,
                                            plan if hasattr(plan, "request_scale") else None), fb)
            ch_cmd.send(t + tc.t_compute, host_seq, cmd)
            host_seq += 1
            for key, val in getattr(controller, "telemetry", {}).items():
                arr = tele.get(key)
                if arr is None:
                    arr = tele[key] = np.full(N, np.nan)
                arr[k:] = val       # forward-fill until the next host tick
        else:
            ref = roll_ref.eval(t)

        # 4: plant
        d = d_seq[k]
        vl = 0
        for j in range(n_sub):
            ts = t + j * h
            if pc.voltage_limit:
                vl += plant.di_dt(x[2], dr["applied"], x[1])[1]
            x = plant.step(x, dr["applied"], yaw, ts, h, d)

        row = log
        row["t"][k] = t
        row["q"][k], row["qd"][k], row["i"][k] = q, qd, i
        row["i_raw"][k], row["i_tgt"][k] = dr["raw"], dr["applied"]
        row["clipped"][k], row["vlim"][k] = dr["clipped"], vl / n_sub
        row["i_lim"][k], row["mode"][k] = dr["lim"], dr["mode"]
        row["q_ref"][k], row["qd_ref"][k], row["qdd_ref"][k] = ref
        row["err"][k] = q - ref[0]
        row["q_enc"][k] = q_enc
        row["qy"][k], row["qyd"][k], row["qydd"][k] = qy, qyd, qydd
        row["tau_couple"][k] = pc.k_yv * qyd + pc.k_ya * qydd
        row["d"][k] = d
        row["cmd_age"][k] = dr["cmd_age"]
        row["fb_age"][k] = fb_age
        row["burst"][k] = lat_cmd.burst_until > t
        row["qy_plan"][k] = plan.eval(t)[0] if plan is not yaw else qy
        row["fault_id"][k], row["locked"][k] = dr["fault_id"], dr["locked"]
        row["T_wind"][k] = getattr(safety, "T", float("nan"))

        if not all(math.isfinite(v) for v in x):
            raise FloatingPointError(f"plant state diverged at t={t:.4f}: {x}")

    log.update({f"c_{k}": v for k, v in tele.items()})
    log.events = list(safety.events)
    log.meta = dict(seed=seed, controller=controller.name, wd_trips=getattr(safety, "wd_trips", 0),
                    cfg=cfg, T_winding=getattr(safety, "T", None),
                    yaw_scale=getattr(plan, "scale_log", None),
                    notices=list(getattr(controller, "notices", []) or []))
    return log
