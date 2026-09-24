"""Baseline roll controller (host, 500 Hz). Deterministic, model-based.

    governed ref (q_c, v_c, a_c)  <- RollGovernor(request, active limit, load est.)
    tau_ff = J a_c + b v_c + 0.8 tau_c tanh(v_c/0.05) + tau_g sin q_c
             + k_yv qy_d(t+T_act) + k_ya qy_dd(t+T_act)          [yaw coupling]
    tau_fb = K (1 + wi/s) (1 + s/wz)/(1 + s/wp) [q_c - q_meas]  (Tustin, 500 Hz)
    i_cmd  = (clip(tau_ff) + clip(tau_fb, headroom left by tau_ff))/Kt, within the
             active limit reported by the drive (feed-forward has priority)
    conditional-integration anti-windup: no integration deeper into a clip

Yaw information (yaw_info):
  "estimate" (default, the claimed baseline) qy_d, qy_dd are predicted at t+T_act
             from the yaw encoder in Feedback (ctrl.yaw_estimator): causal, no plan
  "plan"     the host's own yaw plan evaluated at t+T_act (look-ahead). Stronger
             assumption -- yaw follows its plan -- declared and tested separately
             with plan-following error. A non-finite plan sample falls back to the
             estimate for that tick and is reported.

Fault recovery by class (ctrl.interfaces.FAULT_CLASS): after a communication
fault the host re-aligns and the drive re-arms on its own; after a tracking
fault the request is suspended, the host acknowledges the fault only when a
brake-to-rest and hold from the measured state is predicted feasible, and the
original request resumes only after replan().

Design choices, each traceable to Phase 0/1:
  * gains come from ctrl.loopshape.design at the simulator's worst-normal
    delay (7.0 ms), checked at 11 ms (every message in a burst) and at the
    J +/-30 %, Kt +/-15 % corners
  * the lead filter on the quantized error is the velocity estimate; its HF
    gain sets ~33 mA of current per encoder count
  * friction FF uses the *reference* velocity (measured velocity LSB at 500 Hz
    is 10x the friction speed scale) and is under-compensated (0.8) to avoid
    limit cycles
  * yaw FF is predicted T_act ahead (estimate, or plan in the look-ahead mode),
    cancelling the command-path delay for this predictable disturbance
  * feasibility (governor) is judged with the NOMINAL static model: a constant
    load estimate from the integrator mis-models a lateral payload (its torque
    varies as cos q) and was found to reject holdable positions. A structured
    load estimate is the Phase 3 candidate (ctrl/adaptive.py).
"""
import math

from ctrl import loopshape
from ctrl.governor import RollGovernor, YawMonitor
from ctrl.interfaces import Command, Controller, fault_class
from ctrl.yaw_estimator import KinematicEstimator
from sim import params as P


class BaselineController(Controller):
    name = "baseline"
    YAW_INFO = ("estimate", "plan")

    def __init__(self, T_design=7.0e-3, T_check=11.0e-3, alpha=16, wi_ratio=0.1,
                 T_act=4.0e-3, use_yaw_ff=True, use_governor=True, use_yaw_monitor=True,
                 fb_stale=0.015, i_max=P.I_MAX, load_model=None, anti_windup=True,
                 over_budget_escalate=0.3, yaw_info="estimate"):
        if yaw_info not in self.YAW_INFO:
            raise ValueError("yaw_info must be one of %s" % (self.YAW_INFO,))
        self.yaw_info = yaw_info
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
        self.yest = KinematicEstimator()     # yaw: coupling feed-forward (estimate mode)
        self.rest = KinematicEstimator()     # roll: velocity for a tracking-fault catch

    # ------------------------------------------------------------------
    def reset(self, cfg):
        self.telemetry = {}
        self.ts = ts = 1.0 / cfg.timing.f_ctrl
        wz, wp = self.wc / math.sqrt(self.alpha), self.wc * math.sqrt(self.alpha)
        cz, cp = 2 / (ts * wz), 2 / (ts * wp)
        self.lead = ((1 + cz) / (1 + cp), (1 - cz) / (1 + cp), (1 - cp) / (1 + cp))
        self.wi = self.wi_ratio * self.wc
        self.initialised = False
        self.request_blocked = False
        self.reject_reason = ""
        self.incompatible = ""        # sustained over-budget without yaw coordination
        self.notices = []             # (t, status, reason) whenever the disposition changes
        self.replans = 0
        self.over_since = None
        self.last_shrink = -1e9
        self.was_saturated = False
        self.suspended = ""           # request held after a tracking fault / incompatibility
        self.coord_stop = False       # coordinated stop requested (no yaw authority)
        self.fault_ack = 0            # drive fault id acknowledged in every Command
        self.yaw_info_ok = True
        self.mon.reset(dt=ts)
        self.yest.reset()
        self.rest.reset()
        self.v_meas = 0.0             # estimated roll velocity (0 until the estimate is ready)
        self.gov.cpl_peak, self.gov.kappa = 0.0, 1.0   # new run: no coupling/load history
        self.gov.resume()
        if self.load_model is not None:
            self.load_model.reset()

    def _realign(self, q, t=0.0, v=0.0):
        self.gov.reset(q, v=v, t=t)
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

    MODES = ("sync", "path", "join", "stop", "hold")

    def _gov_telemetry(self, status):
        return dict(gov_status=float(self.gov.STATUS.index(status)),
                    gov_sigma=self.gov.sigma if self.initialised else float("nan"),
                    gov_mode=float(self.MODES.index(self.gov.mode)) if self.initialised else float("nan"),
                    replans=float(self.replans),
                    gov_over_budget=float(status == "over_budget"),
                    gov_braking_short=float(self.gov.braking_short),
                    gov_infeasibility=float(self.INFEASIBILITY.index(self.gov.infeasibility)),
                    gov_incompatible=float(bool(self.incompatible)))

    def _incompatible(self, why, t=None):
        """Report (sticky until replan()) a request the axis cannot honour and that
        only yaw coordination could fix. Control is kept: handing over to passive
        fallback would let the coupling drive the axis. The roll request is brought
        to rest and held, and a coordinated stop is requested; the roll axis alone
        cannot contain a continuing external yaw disturbance beyond its capacity."""
        if not self.incompatible:
            self.incompatible = why
            self.coord_stop = True
            self.notices.append((t, "incompatible", why + "; coordinated stop requested"))
            self._suspend(t, "incompatible yaw coupling: " + why)

    def _suspend(self, t, why):
        if not self.suspended:
            self.suspended = why
            self.gov.suspend(why)
            self.notices.append((t, "suspended", why))

    def _drive_fault(self, t, fb, i_lim, tau_cpl):
        """Host side of recovery by fault class. Returns the fault class."""
        cls = fault_class(fb.fault)
        if fb.locked:
            if not self.request_blocked:
                self.request_blocked = True
                self.reject_reason = "drive lockout after repeated %s faults" % (cls or "drive")
            return cls
        if cls == "tracking":
            self._suspend(t, "tracking fault (%s): request suspended, holding at the re-arm "
                          "position until replan()" % fb.fault)
            # Acknowledge only a replacement request predicted feasible from the
            # measured state: brake to rest within the budget (reserve kept for the
            # model error the fault revealed) and hold inside the holdable interval.
            # The reference then starts at the measured position and velocity.
            extra = self.load_model.torque if self.load_model is not None else None
            # Re-evaluated every tick and withdrawn when it stops fitting: the drive
            # needs the acknowledgement on every command of its re-arm dwell.
            cpl_bound = max(abs(tau_cpl), self.gov.cpl_peak)
            ok = self.rest.ready and self.gov.catch_plan(
                fb.q, self.v_meas, i_lim, cpl_bound, extra) is not None
            self.fault_ack = fb.fault_id if ok else 0
        elif cls:
            self.fault_ack = fb.fault_id    # the cause has cleared or the drive checks it
        return cls

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

    def _reserve(self, i_lim):
        """Torque between the budget and actuator capacity (the governor reserve)."""
        return self.gov._capacity(self.gov.available(i_lim)) - self.gov.available(i_lim)

    def _cpl_factor(self, room):
        """Yaw scale factor that brings the recent peak coupling within room; halve
        it when the reference alone already exceeds the budget (room <= 0)."""
        if room <= 0:
            return 0.5
        peak = max(self.gov.cpl_peak, 1e-9)
        return max(0.0, min(0.9, 0.9 * room / peak))

    def replan(self):
        """Planner acknowledgement after a rejection, incompatibility or tracking
        fault: clear the latched decision and the suspension; the next update
        realigns to the measured state and re-joins the (new) request. A drive
        lockout is not cleared: the drive refuses to re-arm in this run."""
        self.request_blocked, self.reject_reason, self.incompatible = False, "", ""
        self.suspended, self.coord_stop = "", False
        self.replans += 1
        self.gov.resume()
        self.over_since = None
        self.initialised = False

    def _idle(self, ctx, q, reason, valid=False):
        """Freeze the path and reset feedback memory; drive controls fallback/re-arm."""
        if self.initialised:
            # A suspended reference starts from the measured velocity (catch).
            self._realign(q, self.gov.sigma, self.v_meas if self.suspended else 0.0)
        self.was_saturated = False
        self.over_since = None
        self.mon.clear()
        # Idle is never "accepted": rejected if blocked, else restricted with the reason.
        status = "rejected" if self.request_blocked else "restricted"
        self._note(ctx.t, status, self.reject_reason if self.request_blocked else reason)
        self.telemetry = dict(q_c=q, tau_ff=0.0, tau_fb=0.0, integ=0.0,
                              gov_limited=1.0, gov_rejected=float(self.request_blocked),
                              gov_s=0.0, gov_lag=ctx.t - self.gov.sigma if self.initialised else 0.0,
                              gov_kappa=1.0, sat=0.0, stale=float(reason == "feedback_stale"),
                              request_rejected=float(self.request_blocked), host_fallback=1.0,
                              tau_cpl=0.0, i_unsat=0.0, **self._flags(), **self._gov_telemetry(status))
        return Command(ctx.t, 0.0, q, valid=valid, ack=self.fault_ack)

    def _path_speed(self):
        """Path-clock rate: 0 whenever the clock is frozen (join, stop, hold, suspended)."""
        if not self.use_governor:
            return 1.0
        return self.gov.s if self.gov.mode == "path" and not self.suspended else 0.0

    def _flags(self):
        return dict(gov_active=float(self.use_governor),
                    suspended=float(bool(self.suspended)), coord_stop=float(self.coord_stop),
                    fault_ack=float(self.fault_ack), yaw_info_ok=float(self.yaw_info_ok))

    def _coupling(self, ctx, fb):
        """Predicted yaw coupling torque at t + T_act from the selected information."""
        t_eff = ctx.t + self.T_act
        if self.yaw_info == "plan":
            _, yd, ydd = ctx.yaw_at(t_eff)
            tau = self.k_yv * yd + self.k_ya * ydd
            if math.isfinite(tau):
                self.yaw_info_ok = True
                return tau
            if self.yaw_info_ok:
                self._note(ctx.t, "yaw_info", "non-finite yaw plan sample: coupling from "
                           "the measured yaw estimate")
            self.yaw_info_ok = False
        # Until the filter has warmed up (and after a gap reset) its velocity and
        # acceleration are not yet estimates: no coupling feed-forward or decisions.
        est = self.yest.predict(t_eff) if self.yest.ready else None
        return 0.0 if est is None else self.k_yv * est[1] + self.k_ya * est[2]

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
        i_lim = min(fb.i_limit, self.i_max)
        self.yest.update(fb.t_meas, fb.qy)
        self.rest.update(fb.t_meas, fb.q)
        est = self.rest.predict(fb.t_meas)
        self.v_meas = est[1] if est is not None and self.rest.ready else 0.0
        if self.use_yaw_ff and self.yaw_info == "plan" and ctx.yaw_at is None:
            return self._idle(ctx, fb.q, "missing_yaw_plan")
        # --- yaw coupling feed-forward (predicted at t + T_act) -------------
        tau_cpl = self._coupling(ctx, fb) if self.use_yaw_ff else 0.0
        if fb.mode != 0 or fb.locked:
            self._drive_fault(t, fb, i_lim, tau_cpl)
        if self.request_blocked:
            return self._idle(ctx, fb.q, "request_rejected")
        if not self.initialised:
            self._realign(fb.q, t)
        stale = (t - fb.t_meas) > self.fb_stale

        # Drive in fallback (fault latched or timeout): re-align so it can re-arm
        # (tracking faults: only once acknowledged, see _drive_fault). Local damping
        # cannot resist coupling, so still ask yaw to back off if the predicted
        # coupling does not fit even a static hold here.
        if fb.mode != 0:
            if math.isfinite(tau_cpl):
                extra = self.load_model.torque if self.load_model is not None else None
                room = self.gov.available(i_lim) - self.gov._hold(fb.q, 0.0, extra)
                self.gov.cpl_peak = max(self.gov.cpl_peak, abs(tau_cpl))
                if self._coupling_is_cause(tau_cpl, room, i_lim):
                    if ctx.yaw_planner is not None:
                        self._shrink_yaw(ctx, t, self._cpl_factor(room))
                    elif abs(tau_cpl) > room + self._reserve(i_lim):
                        # As in normal operation: immediate only beyond capacity. Within
                        # it, active control can hold once re-armed (the fallback is
                        # transient; the escalation timer runs in normal operation).
                        self._incompatible("predicted coupling exceeds actuator capacity "
                                           "during drive fallback; no yaw coordination", t)
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
            # The governor's own transient motions (join after a realign, stop, start
            # blend) are not evidence against the yaw request (2B review I3): during
            # them coupling is judged against a static hold at the reference, and the
            # escalation timer does not run.
            transient = self.gov.mode in ("join", "stop") or self.gov.blend is not None
            if self.gov.over_budget:
                if transient:
                    self.over_since = None
                    room = self.gov.available(i_lim) - self.gov._hold(q_c, 0.0, extra)
                else:
                    self.over_since = t if self.over_since is None else self.over_since
                    # Torque left for coupling once the reference itself is paid for.
                    room = self.gov.available(i_lim) - self.gov._demand(
                        q_c, v_c, a_c, 0.0, extra, 0.0)
                if not self._coupling_is_cause(tau_cpl, room, i_lim):
                    pass                 # not a coupling problem; the governor reports it
                elif ctx.yaw_planner is not None:
                    self._shrink_yaw(ctx, t, self._cpl_factor(room))
                elif abs(tau_cpl) > (self.gov.available(i_lim) - self.gov._hold(q_c, 0.0, extra)
                                     + self._reserve(i_lim)):
                    # Judged on a static hold at the reference, not on the reference's
                    # own (transient) motion: incompatible means even holding cannot
                    # carry the coupling within actuator capacity.
                    self._incompatible("predicted coupling exceeds actuator capacity; "
                                       "no yaw coordination", t)
                elif self.over_since is not None and t - self.over_since >= self.over_budget_escalate:
                    self._incompatible("predicted torque exceeds budget for %.1f s and no "
                                       "yaw coordination can reduce it" % (t - self.over_since), t)
            else:
                self.over_since = None
        else:
            q_c, v_c, a_c = ctx.ref
            transient = False

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
        if not (math.isfinite(tau) and math.isfinite(q_c)):
            # Last line of defence (inputs are validated upstream): min/max clamping
            # would turn NaN into the limit. Refuse, latch with the cause, fall back.
            self.request_blocked = True
            self.reject_reason = "non-finite command computed"
            return self._idle(ctx, fb.q, "invalid_command_computed")
        # Feed-forward has priority in the clamp; feedback gets the headroom left.
        tau_max = self.k_t * i_lim
        ff_c = max(-tau_max, min(tau_max, tau_ff))
        fb_c = max(-tau_max - ff_c, min(tau_max - ff_c, tau_fb))
        i_cmd = (ff_c + fb_c) / self.k_t

        # --- integrator with conditional-integration anti-windup ----------------
        # While the feedback share is clipped, integration that would push further
        # into the clip is skipped (Packet 2B). Pre-2B (total clamp, back-calculation
        # on the total) the integrator wound against feed-forward clipping: 30 deg in
        # 0.1 s took 1.12 s to settle within 1 deg. With the feed-forward-priority
        # clamp, back-calculation on the feedback headroom still drove it to cancel
        # the proportional term (1.37 s); this rule: 0.60 s, |integ| <= 0.04 N m.
        if not stale and fb.mode == 0:
            clipped_fb = fb_c != tau_fb and (x > 0) == (tau_fb > fb_c)
            if not (self.anti_windup and clipped_fb):
                self.integ += self.ts * self.K * self.wi * x
            # Bound the integral torque to what the actuator can produce.
            if self.anti_windup:
                tau_cap = self.k_t * i_lim
                self.integ = max(-tau_cap, min(tau_cap, self.integ))
        # The limit is active on this command (either share clipped).
        saturated = ff_c != tau_ff or fb_c != tau_fb
        if self.load_model is not None:
            self.load_model.update(ctx, fb, q_c, v_c, a_c, i_cmd, saturated=saturated,
                                   stale=stale, integ=self.integ)

        # --- yaw monitor: roll saturating -> ask yaw to shrink ------------------
        self.was_saturated = saturated
        # Only blame yaw when its coupling is a significant share of the demand.
        yaw_active = abs(tau_cpl) > 0.25 * self.k_t * i_lim
        if self.use_yaw_monitor and transient:
            self.mon.clear()          # a window must not span the governor's own transient
        elif self.use_yaw_monitor:
            # Every valid sample counts in the denominator; only saturation while
            # yaw coupling is significant counts against yaw.
            shrink = self.mon.update(t, saturated and yaw_active)
            if shrink is not None:
                if ctx.yaw_planner is None:
                    # No yaw authority: keep control, report, request a coordinated
                    # stop (passive fallback cannot resist the coupling).
                    self._incompatible("roll saturating under yaw coupling; no yaw "
                                       "coordination", t)
                else:
                    self._shrink_yaw(ctx, t, shrink)

        self._note(t, self.gov.status, self.gov.reason)
        self.telemetry = dict(q_c=q_c, tau_ff=tau_ff, tau_fb=tau_fb, integ=self.integ,
                              gov_limited=float(self.gov.limited), gov_rejected=float(self.gov.rejected),
                              gov_s=self._path_speed(), gov_lag=t - self.gov.sigma, gov_kappa=self.gov.kappa,
                              sat=float(saturated), stale=float(stale), request_rejected=0.0, host_fallback=0.0,
                              tau_cpl=tau_cpl, i_unsat=tau / self.k_t, **self._flags(),
                              **self._gov_telemetry(self.gov.status))
        return Command(t, i_cmd, q_c, ack=self.fault_ack)
