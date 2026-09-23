"""Validation of the simulator against closed-form results.

    python -m unittest discover -s tests -v
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ctrl.interfaces import Command, Controller  # noqa: E402
from ctrl.legacy import LegacyPID  # noqa: E402
from sim import params as P  # noqa: E402
from sim.config import PlantConfig, SimConfig, TimingConfig  # noqa: E402
from sim.engine import simulate  # noqa: E402
from sim.plant import RollPlant  # noqa: E402
from sim.sensing import Encoder, bounded_disturbance  # noqa: E402
from sim.timing import Channel, LatencyModel  # noqa: E402
from sim.trajectories import Hold, MinJerkSequence, RampedSine  # noqa: E402

NO_YAW = Hold(0.0)


def run_plant(pc, x0, i_tgt, T, h=1e-4, yaw=NO_YAW):
    plant, x, out = RollPlant(pc), x0, []
    for k in range(int(round(T / h))):
        x = plant.step(x, i_tgt, yaw, k * h, h, 0.0)
        out.append(x)
    return np.array(out)


class Step(Controller):
    """Open-loop current step at t_step (for timing tests)."""
    name = "step"

    def __init__(self, t_step, amp):
        self.t_step, self.amp = t_step, amp

    def reset(self, cfg):
        self.telemetry = {}

    def update(self, ctx, fb):
        return Command(ctx.t, self.amp if ctx.t >= self.t_step else 0.0)


class TestPlant(unittest.TestCase):
    def test_small_oscillation_frequency(self):
        pc = PlantConfig(b=0.0, tau_c=0.0, voltage_limit=False)
        x = run_plant(pc, (math.radians(2), 0.0, 0.0), 0.0, 3.0)
        q = x[:, 0]
        zc = np.where(np.diff(np.sign(q)) != 0)[0]
        period = 2 * np.mean(np.diff(zc)) * 1e-4
        self.assertAlmostEqual(period, 2 * math.pi / math.sqrt(P.TAU_G / P.J_R), delta=2e-3)

    def test_energy_conserved_frictionless(self):
        pc = PlantConfig(b=0.0, tau_c=0.0, voltage_limit=False)
        x = run_plant(pc, (math.radians(80), 0.0, 0.0), 0.0, 5.0)
        E = 0.5 * P.J_R * x[:, 1] ** 2 + P.TAU_G * (1 - np.cos(x[:, 0]))
        self.assertLess(np.ptp(E) / E[0], 1e-6)

    def test_current_lag_time_constant(self):
        pc = PlantConfig(voltage_limit=False, J=1e3)  # lock the rotor
        x = run_plant(pc, (0.0, 0.0, 0.0), 1.0, 0.01, h=1e-5)
        k = int(round(P.TAU_I / 1e-5)) - 1
        self.assertAlmostEqual(x[k, 2], 1 - math.exp(-1), delta=1e-3)

    def test_voltage_limits_stall_current(self):
        pc = PlantConfig(J=1e3, v_bus=5.0, v_util=1.0)      # 5 V / 1.8 ohm = 2.78 A
        x = run_plant(pc, (0.0, 0.0, 0.0), 3.2, 0.05)
        self.assertAlmostEqual(x[-1, 2], 5.0 / P.R_NOM, delta=1e-3)

    def test_back_emf_limits_speed(self):
        # Free spinning, no load: speed settles where Ke*w = V_avail - R*i
        pc = PlantConfig(tau_g=0.0, tau_c=0.0, b=0.0, v_bus=2.0, v_util=1.0)
        x = run_plant(pc, (0.0, 0.0, 0.0), 3.2, 3.0)
        self.assertAlmostEqual(x[-1, 1], 2.0 / P.K_E, delta=0.05)

    def test_coupling_torque_amplitude(self):
        yaw = RampedSine(math.radians(75), 2.2, t_ramp=0.5)
        ts = np.arange(1.0, 3.0, 1e-4)
        tau = np.array([P.K_YV * yaw.eval(t)[1] + P.K_YA * yaw.eval(t)[2] for t in ts])
        w = 2 * math.pi * 2.2
        expect = math.radians(75) * w * math.hypot(P.K_YV, P.K_YA * w)
        self.assertAlmostEqual(np.max(np.abs(tau)), expect, delta=1e-3 * expect)

    def test_payload_adds_lateral_torque_and_inertia(self):
        pc = PlantConfig(m_payload=1.0)
        self.assertAlmostEqual(pc.tau_lat, 1.0 * 9.81 * 0.035)
        self.assertAlmostEqual(pc.J_total, P.J_R + 0.035 ** 2)

    def test_minjerk_endpoints(self):
        tr = MinJerkSequence(0.0, [(1.0, 0.5, 0.2)])
        self.assertEqual(tr.eval(0.0), (0.0, 0.0, 0.0))
        q, qd, qdd = tr.eval(0.25)
        self.assertAlmostEqual(q, 0.5)
        self.assertAlmostEqual(qd, 1.875 / 0.5)
        self.assertEqual(tr.eval(0.6), (1.0, 0.0, 0.0))


class TestSensingTiming(unittest.TestCase):
    def test_encoder_quantization(self):
        e = Encoder(14)
        self.assertAlmostEqual(e.lsb, 2 * math.pi / 16384)
        self.assertEqual(e.read(0.9999 * e.lsb), 0.0)
        self.assertAlmostEqual(e.read(1.0001 * e.lsb), e.lsb)

    def test_disturbance_bound(self):
        d = bounded_disturbance(np.random.default_rng(0), 10.0, 0.05, 3.0)
        self.assertAlmostEqual(np.max(np.abs(d)), 0.05)

    def test_channel_keeps_newest_sequence(self):
        tc = TimingConfig(burst_rate_hz=0.0)
        ch = Channel(LatencyModel(tc, np.random.default_rng(0)))
        ch.q = [(0.003, 1, 0.0, "old"), (0.002, 2, 0.0, "new")]
        import heapq
        heapq.heapify(ch.q)
        self.assertEqual(ch.poll(0.0025)[3], "new")
        self.assertEqual(ch.poll(0.004)[3], "new")   # overtaken seq 1 discarded

    def test_burst_duty(self):
        tc = TimingConfig(burst_rate_hz=0.5, burst_len_s=(0.02, 0.10))
        lat = LatencyModel(tc, np.random.default_rng(3))
        ts = np.arange(0, 2000.0, 0.01)
        frac = np.mean([lat.in_burst(t) for t in ts])
        self.assertAlmostEqual(frac, 0.5 * 0.06, delta=0.008)

    def test_command_pipeline_delay(self):
        # Fixed 1.0 ms CAN, 0.5 ms compute: command computed at 0.100 s is
        # released 0.1005, arrives 0.1015, picked up at drive tick 0.102,
        # passes the 1 ms delay line and is applied from 0.103 s.
        cfg = SimConfig(duration=0.2).with_(
            timing=dict(can_min=1e-3, can_max=1e-3, burst_rate_hz=0.0),
            plant=dict(d_amp=0.0))
        log = simulate(cfg, Step(0.1, 1.0), Hold(0.0), NO_YAW)
        k_first = int(np.argmax(log.i_tgt > 0))
        self.assertAlmostEqual(log.t[k_first], 0.103, places=6)

    def test_cmd_timeout_fallback(self):
        cfg = SimConfig(duration=0.5).with_(timing=dict(blackout=((0.2, 0.3),)))
        log = simulate(cfg, LegacyPID(), Hold(0.0), NO_YAW)
        m = (log.mode > 0) & (log.t > 0.05)     # ignore start-up before 1st command
        self.assertTrue(m.any())
        t_fb = log.t[m]
        self.assertGreater(t_fb[0], 0.2)
        self.assertLess(t_fb[0], 0.2 + 0.015)       # timeout 10 ms + pipeline
        self.assertLess(t_fb[-1], 0.3 + 0.01)       # recovers once CAN returns


class TestEngine(unittest.TestCase):
    def _run(self, seed, **kw):
        cfg = SimConfig(duration=2.0, **kw)
        return simulate(cfg, LegacyPID(), Hold(0.0), RampedSine(math.radians(75), 1.5, 0.5),
                        seed=seed)

    def test_deterministic(self):
        a, b, c = self._run(7), self._run(7), self._run(8)
        np.testing.assert_array_equal(a.q, b.q)
        self.assertGreater(np.max(np.abs(a.q - c.q)), 0)

    def test_integration_converged(self):
        # 24-bit encoder: with 14 bits a 1e-9 rad difference can flip a count
        # and the closed loop then diverges by quantization, not integration.
        base = SimConfig(duration=2.0).with_(sensor=dict(enc_bits=24))
        fine = base.with_(timing=dict(dt_sim=2.5e-5))
        yaw = RampedSine(math.radians(75), 2.2, 0.5)
        a = simulate(base, LegacyPID(), Hold(0.0), yaw, seed=1)
        b = simulate(fine, LegacyPID(), Hold(0.0), yaw, seed=1)
        self.assertLess(np.max(np.abs(a.q - b.q)) * 180 / math.pi, 1e-3)

    def test_linear_response_matches_frequency_domain(self):
        """End-to-end check of delays/gains: closed-loop error amplitude under a
        yaw sine vs. an analytic model of the same loop with the measured mean
        delay. Nonlinear/quantization effects removed for this test."""
        f = 1.5
        cfg = SimConfig(duration=6.0).with_(
            plant=dict(tau_c=0.0, d_amp=0.0, voltage_limit=False),
            timing=dict(burst_rate_hz=0.0, can_min=1.2e-3, can_max=1.2e-3),
            sensor=dict(enc_bits=24))
        ctl = LegacyPID()
        log = simulate(cfg, ctl, Hold(0.0), RampedSine(math.radians(75), f, 0.5), seed=0)
        m = log.t > 3.0
        amp_sim = 0.5 * np.ptp(log.err[m])
        T = (np.nanmean(log.fb_age[m]) + np.nanmean(log.cmd_age[m])
             + P.T_CMD_DELAY + 0.5e-3)   # + half a drive tick (ZOH)
        w = 2 * math.pi * f
        s = 1j * w
        a = math.exp(-2 * math.pi * ctl.f_vel / cfg.timing.f_ctrl)
        z = np.exp(s / cfg.timing.f_ctrl)
        vel = (1 - a) * (1 - 1 / z) * cfg.timing.f_ctrl / (1 - a / z)   # discrete filtered diff
        C = ctl.kp + ctl.ki / s + ctl.kd * vel
        Pl = 1 / (P.J_R * s ** 2 + P.B_VISC * s + P.TAU_G)   # gravity linearised
        L = C * np.exp(-s * T) / (P.TAU_I * s + 1) * Pl
        tau = math.radians(75) * w * math.hypot(P.K_YV, P.K_YA * w)
        amp_model = abs(Pl / (1 + L)) * tau
        self.assertAlmostEqual(amp_sim / amp_model, 1.0, delta=0.05)

    def test_stability_boundary_matches_delay_model(self):
        """Delay-sensitive check: predict the critical gain of a stiff PD loop
        from the frequency-domain model (delay = measured pipeline), then verify
        the simulator is stable at 0.8x and unstable at 1.25x that gain."""
        # Drive limit lifted: at 3.2 A an unstable loop becomes a bounded limit
        # cycle (itself a real behaviour) and growth could not be observed.
        cfg = SimConfig(duration=1.5).with_(
            plant=dict(tau_c=0.0, d_amp=0.0, voltage_limit=False),
            timing=dict(burst_rate_hz=0.0, can_min=1.2e-3, can_max=1.2e-3),
            sensor=dict(enc_bits=24), drive=dict(i_limit=1e6))
        kp0, kd0, fv = 4.0, 0.12, 80.0
        probe = simulate(cfg, LegacyPID(kp=kp0, kd=kd0, ki=0.0, f_vel=fv, i_max=1e9),
                         Hold(0.0), NO_YAW)
        T = (np.nanmean(probe.fb_age[200:]) + np.nanmean(probe.cmd_age[200:])
             + P.T_CMD_DELAY + 0.5e-3)
        a = math.exp(-2 * math.pi * fv / cfg.timing.f_ctrl)

        def L(w, g):
            s = 1j * w
            z = np.exp(s / cfg.timing.f_ctrl)
            vel = (1 - a) * (1 - 1 / z) * cfg.timing.f_ctrl / (1 - a / z)
            C = g * (kp0 + kd0 * vel)
            return C * np.exp(-s * T) / (P.TAU_I * s + 1) / (P.J_R * s ** 2 + P.B_VISC * s
                                                               + P.TAU_G)
        w = np.linspace(5, 600, 200000)
        ph = np.unwrap(np.angle(L(w, 1.0)))
        i180 = np.where(np.diff(np.sign(ph + np.pi)))[0][0]
        g_crit = 1 / abs(L(w[i180], 1.0))

        def growth(g):
            ref = MinJerkSequence(0.0, [(math.radians(2), 0.05, 5.0)])
            lg = simulate(cfg, LegacyPID(kp=g * kp0, kd=g * kd0, ki=0.0, f_vel=fv, i_max=1e9),
                          ref, NO_YAW)
            e = lg.err
            return np.ptp(e[1300:1500]) / max(np.ptp(e[300:500]), 1e-12)
        self.g_crit = g_crit
        self.assertLess(growth(0.8 * g_crit), 0.5)      # decaying
        self.assertGreater(growth(1.25 * g_crit), 2.0)  # growing


if __name__ == "__main__":
    unittest.main()
