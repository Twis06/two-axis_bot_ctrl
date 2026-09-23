"""Baseline roll controller (host, 500 Hz). Deterministic, model-based.

    governed ref (q_c, v_c, a_c)  <- RollGovernor(request, active limit, load est.)
    tau_ff = J a_c + b v_c + 0.8 tau_c tanh(v_c/0.05) + tau_g sin q_c
             + k_yv qy_d(t+T_act) + k_ya qy_dd(t+T_act)          [yaw plan look-ahead]
    tau_fb = K (1 + wi/s) (1 + s/wz)/(1 + s/wp) [q_c - q_meas]  (Tustin, 500 Hz)
    i_cmd  = clip((tau_ff + tau_fb)/Kt, +/- active limit reported by the drive)
    back-calculation anti-windup on the integrator against that same limit

Design choices, each traceable to Phase 0/1:
  * gains come from ctrl.loopshape.design at the simulator's worst-normal
    delay (7.0 ms), checked at 11 ms (every message in a burst) and at the
    J +/-30 %, Kt +/-15 % corners
  * the lead filter on the quantized error is the velocity estimate; its HF
    gain sets ~33 mA of current per encoder count
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
                 fb_stale=0.015, i_max=P.I_MAX, load_model=None, anti_windup=True,
                 over_budget_escalate=0.3):
        self.alpha, self.wi_ratio = alpha, wi_ratio
        self.wc, self.K = loopshape.design(
            T_design, alpha, T_check=T_check,
            corners=tuple((T_check or T_design, P.J_R * j, kt)
                          for j in (0.7, 1.3) for kt in (0.85, 1.15)))
        self.T_act, self.use_yaw_ff = T_act, use_yaw_ff
        self.use_governor, self.use_yaw_monitor = use_governor, use_yaw_monitor
        self.fb_stale, self.i_max = fb_stale, i_max
        self.anti_windup = anti_windup   # False only for ablation tests
        # Predicted over-budget torque (e.g. yaw coupling) is never absorbed by
        # moving the reference (governor contract). With a yaw planner the host asks
        # at once for the yaw scale that fits; without one, over-budget persisting
        # for over_budget_escalate is reported as an incompatible request while the
        # axis stays under control (passive fallback cannot resist coupling).
        self.over_budget_escalate = over_budget_escalate
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
        self.k_aw = self.wc if self.anti_windup else 0.0
        self.initialised = False
        self.request_blocked = False
        self.reject_reason = ""
        self.incompatible = ""        # sustained over-budget without yaw coordination
        self.notices = []             # (t, status, reason) whenever the disposition changes
        self.over_since = None
        self.last_shrink = -1e9
        self.was_saturated = False
        self.mon.reset()
        self.gov.cpl_peak, self.gov.kappa = 0.0, 1.0   # new run: no coupling/load history
        if self.load_model is not None:
            self.load_model.reset()

    def _realign(self, q, t=0.0):
        self.gov.reset(q, t=t)
        self.e_prev = self.x_prev = 0.0
        self.integ = 0.0
        self.initialised = True

    INFEASIBILITY = ("", "static", "dynamic", "braking")

    def _note(self, t, status, reason):
        # Routine accepted/reshaped detail changes every tick; log those by status only.
        key = lambda st, rs: (st, "" if st in ("accepted", "reshaped") else rs)
        if not self.notices or key(*self.notices[-1][1:]) != key(status, reason):
            self.notices.append((t, status, reason))
            del self.notices[:-200]

    def _gov_telemetry(self, status):
        return dict(gov_status=float(self.gov.STATUS.index(status)),
                    gov_over_budget=float(status == "over_budget"),
                    gov_braking_short=float(self.gov.braking_short),
                    gov_infeasibility=float(self.INFEASIBILITY.index(self.gov.infeasibility)),
                    gov_incompatible=float(bool(self.incompatible)))

    def _incompatible(self, why, t=None):
        """Report (sticky until replan()) a request the axis cannot honour and that
        only yaw coordination could fix. Control is kept: handing over to passive
        fallback would let the coupling drive the axis."""
        if not self.incompatible:
            self.incompatible = why
            self.notices.append((t, "incompatible", why))

    def _shrink_yaw(self, ctx, t, factor):
        """One shared hold-off for both yaw-reduction paths (governor, monitor)."""
        if t - max(self.last_shrink, self.mon.t_last) < self.mon.hold_off:
            return
        ctx.yaw_planner.request_scale(t, ctx.yaw_planner.scale(t) * factor)
        self.last_shrink = self.mon.t_last = t

    def _coupling_is_cause(self, tau_cpl, room, i_lim):
        """Coupling contributes to breaking the budget: it exceeds the room the
        reference leaves and is a significant share (5 %) of the budget. Negligible
        coupling on an over-budget reference is not a yaw problem."""
        return abs(tau_cpl) > max(room, 0.05 * self.gov.available(i_lim))

    def _cpl_factor(self, room):
        """Yaw scale factor that brings the recent peak coupling within room; halve
        it when the reference alone already exceeds the budget (room <= 0)."""
        if room <= 0:
            return 0.5
        peak = max(self.gov.cpl_peak, 1e-9)
        return max(0.0, min(0.9, 0.9 * room / peak))

    def replan(self):
        """Host acknowledgement after a rejection or incompatibility: clear the
        latched decision; the next update realigns to the measured state."""
        self.request_blocked, self.reject_reason, self.incompatible = False, "", ""
        self.over_since = None
        self.initialised = False

    def _idle(self, ctx, q, reason, valid=False):
        """Freeze the path and reset feedback memory; drive controls fallback/re-arm."""
        if self.initialised:
            self._realign(q, self.gov.sigma)
        self.was_saturated = False
        self.over_since = None
        # Idle is never "accepted": rejected if blocked, else restricted with the reason.
        status = "rejected" if self.request_blocked else "restricted"
        self._note(ctx.t, status, self.reject_reason if self.request_blocked else reason)
        self.telemetry = dict(q_c=q, tau_ff=0.0, tau_fb=0.0, integ=0.0,
                              gov_limited=1.0, gov_rejected=float(self.request_blocked),
                              gov_s=0.0, gov_lag=ctx.t - self.gov.sigma if self.initialised else 0.0,
                              gov_kappa=1.0, sat=0.0, stale=float(reason == "feedback_stale"),
                              request_rejected=float(self.request_blocked), host_fallback=1.0,
                              **self._gov_telemetry(status))
        return Command(ctx.t, 0.0, q, valid=valid)

    # ------------------------------------------------------------------
    def update(self, ctx, fb):
        t = ctx.t
        if fb is None or not all(math.isfinite(x) for x in
                                  (fb.t_meas, fb.q, fb.qy, fb.i_meas, fb.i_limit)):
            q = self.gov.q if self.initialised else 0.0
            return self._idle(ctx, q, "feedback_invalid")
        if fb.i_limit <= 0 or t < fb.t_meas or t - fb.t_meas > self.fb_stale:
            q = self.gov.q if self.initialised else fb.q
            return self._idle(ctx, q, "feedback_stale")
        if self.request_blocked:
            return self._idle(ctx, fb.q, "request_rejected")
        if self.use_yaw_ff and ctx.yaw_at is None:
            return self._idle(ctx, fb.q, "missing_yaw_plan")
        if not self.initialised:
            self._realign(fb.q, t)
        stale = (t - fb.t_meas) > self.fb_stale
        i_lim = min(fb.i_limit, self.i_max)

        # --- yaw coupling feed-forward (host's own plan, looked ahead) -----
        tau_cpl = 0.0
        if self.use_yaw_ff and ctx.yaw_at is not None:
            _, yd, ydd = ctx.yaw_at(t + self.T_act)
            tau_cpl = self.k_yv * yd + self.k_ya * ydd

        # Drive in fallback (fault latched or timeout): re-align so it can re-arm.
        # Local damping cannot resist coupling, so still ask yaw to back off if the
        # predicted coupling does not fit even a static hold here.
        if fb.mode != 0:
            if math.isfinite(tau_cpl):
                room = (self.gov.available(i_lim) - self.tau_c
                        - abs(self.tau_g * math.sin(fb.q)))
                self.gov.cpl_peak = max(self.gov.cpl_peak, abs(tau_cpl))
                if self._coupling_is_cause(tau_cpl, room, i_lim):
                    if ctx.yaw_planner is not None:
                        self._shrink_yaw(ctx, t, self._cpl_factor(room))
                    else:
                        self._incompatible("predicted coupling exceeds the hold budget during "
                                           "drive fallback; no yaw coordination", t)
            return self._idle(ctx, fb.q, "drive_fallback", valid=True)

        # --- governed reference --------------------------------------------
        extra = self.load_model.torque if self.load_model is not None else None
        if self.use_governor:
            q_c, v_c, a_c = self.gov.step(self.ts, ctx.ref_at, i_lim, 0.0, tau_cpl, extra,
                                          saturated=self.was_saturated)
            if self.gov.blocked:
                self.request_blocked = True
                self.reject_reason = self.gov.reason
                return self._idle(ctx, fb.q, "no_holdable_position")
            if self.gov.over_budget:
                self.over_since = t if self.over_since is None else self.over_since
                # Torque left for coupling once the reference itself is paid for.
                room = self.gov.available(i_lim) - self.gov._demand(
                    q_c, v_c, a_c, 0.0, extra, 0.0)
                if not self._coupling_is_cause(tau_cpl, room, i_lim):
                    pass                 # not a coupling problem; the governor reports it
                elif ctx.yaw_planner is not None:
                    self._shrink_yaw(ctx, t, self._cpl_factor(room))
                elif abs(tau_cpl) > room + (self.gov._capacity(self.gov.available(i_lim))
                                            - self.gov.available(i_lim)):
                    self._incompatible("predicted coupling exceeds actuator capacity; "
                                       "no yaw coordination", t)
                elif t - self.over_since >= self.over_budget_escalate:
                    self._incompatible("predicted torque exceeds budget for %.1f s and no "
                                       "yaw coordination can reduce it" % (t - self.over_since), t)
            else:
                self.over_since = None
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
            # Back-calculation also reacts to feed-forward clipping; bound the
            # integral torque to what the actuator can produce so it cannot wind
            # against an infeasible feed-forward (abrupt, ungoverned requests).
            if self.anti_windup:
                tau_cap = self.k_t * i_lim
                self.integ = max(-tau_cap, min(tau_cap, self.integ))
        if self.load_model is not None:
            self.load_model.update(ctx, fb, q_c, v_c, a_c, i_cmd, saturated=abs(i_unsat) >= i_lim,
                                   stale=stale, integ=self.integ)

        # --- yaw monitor: roll saturating -> ask yaw to shrink ------------------
        saturated = abs(i_unsat) >= i_lim
        self.was_saturated = saturated
        # Only blame yaw when its coupling is a significant share of the demand.
        yaw_active = abs(tau_cpl) > 0.25 * self.k_t * i_lim
        if self.use_yaw_monitor and (yaw_active or not saturated):
            shrink = self.mon.update(t, saturated)
            if shrink is not None:
                if ctx.yaw_planner is None:
                    # Pre-2A behaviour, kept in 2A: see Packet 2B (no yaw authority).
                    self.request_blocked = True
                    self.reject_reason = "yaw reduction needed but no yaw coordination"
                    return self._idle(ctx, fb.q, "yaw_coordination_unavailable")
                self._shrink_yaw(ctx, t, shrink)

        self._note(t, self.gov.status, self.gov.reason)
        self.telemetry = dict(q_c=q_c, tau_ff=tau_ff, tau_fb=tau_fb, integ=self.integ,
                              gov_limited=float(self.gov.limited), gov_rejected=float(self.gov.rejected),
                              gov_s=self.gov.s, gov_lag=t - self.gov.sigma, gov_kappa=self.gov.kappa,
                              sat=float(saturated), stale=float(stale), request_rejected=0.0, host_fallback=0.0,
                              **self._gov_telemetry(self.gov.status))
        return Command(t, i_cmd, q_c)
