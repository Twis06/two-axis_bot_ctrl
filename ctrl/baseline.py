"""Baseline roll controller (host, 500 Hz). Deterministic, model-based.

    governed ref (q_c, v_c, a_c)  <- RollGovernor(request, active limit, load est.)
    tau_ff = J a_c + b v_c + 0.8 tau_c tanh(v_c/0.05) + tau_g sin q_c
             + k_yv qy_d(t+T_act) + k_ya qy_dd(t+T_act)          [yaw plan look-ahead]
    tau_fb = K (1 + wi/s) (1 + s/wz)/(1 + s/wp) [q_c - q_meas]  (Tustin, 500 Hz)
    i_cmd  = clip((tau_ff + tau_fb)/Kt, +/- active limit reported by the drive)
    back-calculation anti-windup on the integrator against that same limit

Design choices, each traceable to Phase 0/1:
  * gains come from ctrl.loopshape.design at the simulator's worst-normal
    delay (7.0 ms) with a check at 11 ms (every message in a burst)
  * the lead filter on the quantized error is the velocity estimate; its HF
    gain sets ~50 mA of current per encoder count
  * friction FF uses the *reference* velocity (measured velocity LSB at 500 Hz
    is 10x the friction speed scale) and is under-compensated (0.8) to avoid
    limit cycles
  * yaw FF uses the host's own yaw plan evaluated T_act ahead, cancelling the
    command-path delay for this predictable disturbance
  * feasibility (governor) is judged with the NOMINAL static model: a constant
    load estimate from the integrator mis-models a lateral payload (its torque
    varies as cos q) and was found to reject holdable positions. A structured
    load estimate is the Phase 3 candidate (ctrl/adaptive.py).
"""
import math

from ctrl import loopshape
from ctrl.governor import RollGovernor, YawMonitor
from ctrl.interfaces import Command, Controller
from sim import params as P


class BaselineController(Controller):
    name = "baseline"

    def __init__(self, T_design=7.0e-3, T_check=11.0e-3, alpha=16, wi_ratio=0.1,
                 T_act=4.0e-3, use_yaw_ff=True, use_governor=True, use_yaw_monitor=True,
                 fb_stale=0.015, i_max=P.I_MAX, load_model=None):
        self.alpha, self.wi_ratio = alpha, wi_ratio
        self.wc, self.K = loopshape.design(T_design, alpha, T_check=T_check)
        self.T_act, self.use_yaw_ff = T_act, use_yaw_ff
        self.use_governor, self.use_yaw_monitor = use_governor, use_yaw_monitor
        self.fb_stale, self.i_max = fb_stale, i_max
        # nominal model (never the true, perturbed plant)
        self.J, self.b, self.tau_c, self.tau_g, self.k_t = P.J_R, P.B_VISC, P.TAU_C, P.TAU_G, P.K_T
        self.k_yv, self.k_ya = P.K_YV, P.K_YA
        self.gov = RollGovernor()
        self.load_model = load_model   # optional: object with .reset(), .update(...), .torque(q)
        self.mon = YawMonitor()

    # ------------------------------------------------------------------
    def reset(self, cfg):
        self.telemetry = {}
        self.ts = ts = 1.0 / cfg.timing.f_ctrl
        wz, wp = self.wc / math.sqrt(self.alpha), self.wc * math.sqrt(self.alpha)
        cz, cp = 2 / (ts * wz), 2 / (ts * wp)
        self.lead = ((1 + cz) / (1 + cp), (1 - cz) / (1 + cp), (1 - cp) / (1 + cp))
        self.wi = self.wi_ratio * self.wc
        self.k_aw = self.wc
        self.initialised = False
        self.was_saturated = False
        self.mon.reset()
        if self.load_model is not None:
            self.load_model.reset()

    def _realign(self, q, t=0.0):
        self.gov.reset(q, t=t)
        self.e_prev = self.x_prev = 0.0
        self.integ = 0.0
        self.initialised = True

    # ------------------------------------------------------------------
    def update(self, ctx, fb):
        t = ctx.t
        if fb is None:
            return Command(t, 0.0, ctx.ref[0], valid=False)
        if not self.initialised:
            self._realign(fb.q, t)
        stale = (t - fb.t_meas) > self.fb_stale
        i_lim = min(fb.i_limit, self.i_max)

        # Drive in fallback (fault latched or timeout): re-align so it can re-arm.
        if fb.mode != 0:
            self._realign(fb.q, self.gov.sigma if self.initialised else t)

        # --- yaw coupling feed-forward (host's own plan, looked ahead) -----
        tau_cpl = 0.0
        if self.use_yaw_ff and ctx.yaw_at is not None:
            _, yd, ydd = ctx.yaw_at(t + self.T_act)
            tau_cpl = self.k_yv * yd + self.k_ya * ydd

        # --- governed reference --------------------------------------------
        extra = self.load_model.torque if self.load_model is not None else None
        if self.use_governor:
            q_c, v_c, a_c = self.gov.step(self.ts, ctx.ref_at, i_lim, 0.0, tau_cpl, extra,
                                          saturated=self.was_saturated)
        else:
            q_c, v_c, a_c = ctx.ref

        # --- feedback: PI x lead on the quantized error ----------------------
        e = q_c - fb.q
        b0, b1, a1 = self.lead
        x = b0 * e + b1 * self.e_prev - a1 * self.x_prev
        self.e_prev, self.x_prev = e, x
        tau_fb = self.K * x + self.integ

        tau_ff = (self.J * a_c + self.b * v_c + 0.8 * self.tau_c * math.tanh(v_c / 0.05)
                  + self.tau_g * math.sin(q_c) + tau_cpl)
        if extra is not None:
            tau_ff += extra(q_c)
        tau = tau_ff + tau_fb
        i_unsat = tau / self.k_t
        i_cmd = max(-i_lim, min(i_lim, i_unsat))

        # --- integrator with back-calculation anti-windup ---------------------
        if not stale and fb.mode == 0:
            self.integ += self.ts * (self.K * self.wi * x + self.k_aw * (i_cmd - i_unsat) * self.k_t)
        if self.load_model is not None:
            self.load_model.update(ctx, fb, q_c, v_c, a_c, i_cmd, saturated=abs(i_unsat) >= i_lim,
                                   stale=stale, integ=self.integ)

        # --- yaw monitor: roll saturating -> ask yaw to shrink ------------------
        saturated = abs(i_unsat) >= i_lim
        self.was_saturated = saturated
        # Only blame yaw when its coupling is a significant share of the demand.
        yaw_active = abs(tau_cpl) > 0.25 * self.k_t * i_lim
        if self.use_yaw_monitor and ctx.yaw_planner is not None and (yaw_active or not saturated):
            shrink = self.mon.update(t, saturated)
            if shrink is not None:
                ctx.yaw_planner.request_scale(t, ctx.yaw_planner.scale(t) * shrink)

        self.telemetry = dict(q_c=q_c, tau_ff=tau_ff, tau_fb=tau_fb, integ=self.integ,
                              gov_limited=float(self.gov.limited), gov_rejected=float(self.gov.rejected),
                              gov_s=self.gov.s, gov_lag=t - self.gov.sigma, gov_kappa=self.gov.kappa,
                              sat=float(saturated), stale=float(stale))
        return Command(t, i_cmd, q_c)
