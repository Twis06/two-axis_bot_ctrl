"""Phase 0: paper analysis of Runs A-E from the nominal model.

No simulation. Every quantity is a closed-form or frequency-response calculation
from sim/params.py plus explicitly stated assumptions. Prints markdown tables and
writes figures to report/figs/.

    python analysis/phase0.py
"""
import math
import os
import sys

import numpy as np
from scipy.special import j0
from scipy.optimize import brentq

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from sim.params import *  # noqa: E402,F403

FIGS = os.path.join(ROOT, "report", "figs")
DEG = 180 / math.pi
TAU_MAX = K_T * I_MAX        # 0.448 N m
TAU_DER = K_T * I_DERATED    # 0.336 N m


def h(title):
    print(f"\n## {title}\n")


def table(headers, rows):
    print("| " + " | ".join(headers) + " |")
    print("|" + "---|" * len(headers))
    for r in rows:
        print("| " + " | ".join(str(c) for c in r) + " |")


# ---------------------------------------------------------------------------
# 1. Yaw coupling demand for Runs B, C (roll held at 0 => q_r_ddot ~ 0,
#    gravity ~ 0). Yaw q_y = A sin(wt): velocity and acceleration terms are in
#    quadrature, so the peak coupling torque is A*w*sqrt(K_YV^2 + (K_YA*w)^2).
# ---------------------------------------------------------------------------
def coupling_peak(A, f):
    w = 2 * math.pi * f
    return A * w * math.hypot(K_YV, K_YA * w), A * w * K_YV, A * w * w * K_YA


YAW_A = math.radians(75)
RUNS_BC = {"B": dict(f=1.5, rms=4.2, peak=11.5, ipk=3.1, clip=0.08),
           "C": dict(f=2.2, rms=7.8, peak=20.7, ipk=3.2, clip=0.31)}


def plant_mag(w):
    """|P(jw)| for the rigid roll inertia, P = 1/(J s^2 + b s). Friction ignored."""
    return 1 / abs(J_R * (1j * w) ** 2 + B_VISC * 1j * w)


def section_bc():
    h("Runs B/C - yaw coupling demand vs. capacity")
    rows = []
    out = {}
    for k, r in RUNS_BC.items():
        pk, tv, ta = coupling_peak(YAW_A, r["f"])
        worst = pk + D_MAX + TAU_C
        e_rms = math.radians(r["rms"])
        # Implied |e/tau| using RMS error vs RMS coupling torque (sine RMS = pk/sqrt2).
        k_dyn = (pk / math.sqrt(2)) / e_rms
        w = 2 * math.pi * r["f"]
        s_implied = (1 / k_dyn) / plant_mag(w)  # |S| if e = P S tau_couple
        out[k] = dict(pk=pk, worst=worst, k_dyn=k_dyn, s=s_implied)
        rows.append([k, r["f"], f"{tv:.3f}", f"{ta:.3f}", f"{pk:.3f} ({pk / K_T:.2f} A)",
                     f"{worst:.3f} ({worst / K_T:.2f} A)",
                     f"{worst / TAU_MAX:.0%}", f"{worst / TAU_DER:.0%}",
                     f"{r['ipk']} A", f"{r['ipk'] * K_T / worst:.1f}x",
                     f"{r['peak'] / r['rms']:.2f}", f"{k_dyn:.2f}", f"{s_implied:.2f}"])
    table(["Run", "f Hz", "0.008·q̇y pk", "0.0008·q̈y pk", "coupling pk N·m",
           "+|d|+τc worst", "% of 3.2 A", "% of 2.4 A", "obs. i pk",
           "obs/model", "err pk/RMS", "|τ/e| N·m/rad", "implied |S|"], rows)
    b, c = out["B"], out["C"]
    print(f"\nDisturbance ratio C/B = {c['pk'] / b['pk']:.2f}; "
          f"RMS error ratio = {7.8 / 4.2:.2f}; peak error ratio = {20.7 / 11.5:.2f}.")
    print("Pure sine has peak/RMS = 1.41; observed ~2.7 => error is not a steady sinusoid "
          "(transients, bursts, or saturation episodes).")
    return out


# ---------------------------------------------------------------------------
# 2. Run A - shaped moves to +/-45 deg, yaw still, no saturation.
#    ASSUMPTION: minimum-jerk shaping; peak current coincides with peak accel.
# ---------------------------------------------------------------------------
def section_a():
    h("Run A - what 2.1 A implies")
    tau_pk = 2.1 * K_T
    static = TAU_G * math.sin(math.radians(45)) + TAU_C
    dyn_hi = tau_pk - static + D_MAX   # d may help
    dyn_lo = tau_pk - static - D_MAX   # d may hurt
    a_lo, a_hi = dyn_lo / J_R, dyn_hi / J_R
    D = math.radians(90)               # -45 -> +45 stroke
    # min-jerk: a_pk = 5.774 D / T^2
    T_lo, T_hi = math.sqrt(5.774 * D / a_hi), math.sqrt(5.774 * D / a_lo)
    table(["Quantity", "Value"], [
        ["Peak motor torque (2.1 A)", f"{tau_pk:.3f} N·m ({tau_pk / TAU_MAX:.0%} of 3.2 A)"],
        ["Static at 45°: τg sin45 + τc", f"{static:.3f} N·m"],
        ["Implied inertial torque", f"{dyn_lo:.3f}–{dyn_hi:.3f} N·m"],
        ["Implied peak accel", f"{a_lo:.0f}–{a_hi:.0f} rad/s²"],
        ["Implied 90° min-jerk move time", f"{T_lo:.2f}–{T_hi:.2f} s"],
        ["Peak error 6.1° vs. J·a_pk (no accel FF)",
         f"J·a/e = {J_R * a_lo / math.radians(6.1):.2f}–{J_R * a_hi / math.radians(6.1):.2f} N·m/rad"],
    ])
    print("\nReading: 2.1 A leaves ~34 % headroom, so A's 2.8° RMS is not a torque limit. "
          "An error that scales with J·a at an effective stiffness of ~1–3 N·m/rad is what "
          "feedback-only tracking (no inertia/gravity feed-forward) would give.")


# ---------------------------------------------------------------------------
# 3. Runs D/E - payload CoM shifted 35 mm, sweep +/-80 deg.
#    A symmetric sweep with an odd load (tau_g' sin q) gives ~zero mean error.
#    A nonzero signed mean needs an even/one-signed load: a lateral CoM shift
#    adds m g s cos q, which is > 0 for |q| < 90 deg.
#    ASSUMPTION: sinusoidal sweep, quasi-static (dynamics ignored => the mass
#    estimate is an UPPER bound if the sweep was fast).
# ---------------------------------------------------------------------------
SWEEP_A = math.radians(80)
S_SHIFT = 0.035


def sat_fraction(m_p, tau_lim, lateral=True, n=20001):
    phi = np.linspace(0, 2 * np.pi, n, endpoint=False)
    q = SWEEP_A * np.sin(phi)
    qd_sign = np.sign(np.cos(phi))
    extra = m_p * G * S_SHIFT
    if lateral:
        tau = TAU_G * np.sin(q) + extra * np.cos(q)
    else:
        tau = (TAU_G + extra) * np.sin(q)
    tau = tau + TAU_C * qd_sign
    return float(np.mean(np.abs(tau) > tau_lim))


def section_de():
    h("Runs D/E - shifted payload")
    m38 = brentq(lambda m: sat_fraction(m, TAU_MAX) - 0.38, 0.05, 5.0)
    m38_arm = brentq(lambda m: sat_fraction(m, TAU_MAX, lateral=False) - 0.38, 0.05, 5.0)
    frac_e = sat_fraction(m38, TAU_DER)
    std = math.sqrt(9.1 ** 2 - 4.6 ** 2)
    e_mean = math.radians(4.6)
    cos_mean = float(j0(SWEEP_A))   # time-mean of cos(A sin wt)
    rows = []
    for k in (1.3, 2.8):  # effective stiffness bracket from Runs A-C
        tau_mean = k * e_mean
        m = tau_mean / cos_mean / (G * S_SHIFT)
        rows.append([f"{k}", f"{tau_mean:.3f}", f"{m:.2f}"])
    print(f"D: RMS 9.1° = mean 4.6° ⊕ fluctuation {std:.1f}° -> bias is "
          f"{4.6 ** 2 / 9.1 ** 2:.0%} of mean-square error.\n")
    print("Mass implied by the mean error, if the loop behaves like a spring of "
          f"stiffness K (time-mean cos q over sweep = {cos_mean:.2f}):\n")
    table(["K N·m/rad", "mean unmodelled τ N·m", "lateral mass m_p kg"], rows)
    print()
    table(["Model (quasi-static)", "m_p for 38 % limited @3.2 A",
           "peak static demand N·m", "predicted limited fraction @2.4 A"], [
        ["lateral shift  (+m g s cos q)", f"{m38:.2f} kg",
         f"{np.hypot(TAU_G, m38 * G * S_SHIFT) + TAU_C:.3f}", f"{frac_e:.0%}"],
        ["along-arm shift ((τg+m g s) sin q)", f"{m38_arm:.2f} kg",
         f"{(TAU_G + m38_arm * G * S_SHIFT) * math.sin(SWEEP_A) + TAU_C:.3f}",
         f"{sat_fraction(m38_arm, TAU_DER, lateral=False):.0%}"],
    ])
    dJ = m38 * S_SHIFT ** 2
    phi_eq = math.degrees(math.atan2(m38 * G * S_SHIFT, TAU_G))
    print(f"\nLateral case: gravity equilibrium rotates by {phi_eq:.0f}°, inertia rises by "
          f"≥{dJ * 1e3:.2f}e-3 kg·m² (+{dJ / J_R:.0%}, point-mass lower bound).")
    print("The along-arm model cannot produce a signed mean on a symmetric sweep, so the "
          "evidence favours a lateral (or otherwise asymmetric) load.")
    print("The two mass estimates disagree (mean-error route 0.5–1.2 kg; quasi-static saturation "
          f"route {m38:.1f} kg). A quasi-static fit ignores sweep inertia and |d|, so the gap says "
          "sweep dynamics carry part of the 38 %. Mass is NOT identifiable from the summaries; "
          "treat it as a swept parameter.")
    return m38


# ---------------------------------------------------------------------------
# 4. Delay budget and the feedback bandwidth it permits.
#    Loop: C(s) = K (1+wi/s) (1+s/wz)/(1+s/wp), actuator e^{-sT}/(tau_i s+1),
#    plant 1/(J s^2 + b s). Lead centred on wc: wz = wc/sqrt(a), wp = wc*sqrt(a).
# ---------------------------------------------------------------------------
def delay_budget(f_ctrl=500.0, can=CAN_MAX, can_both_ways=False, can_fb=None):
    ts = 1 / f_ctrl
    items = {
        "current-command delay": T_CMD_DELAY,
        "CAN command": can,
        "CAN feedback": (can if can_fb is None else can_fb) if can_both_ways else 0.0,
        "encoder sample age (1 kHz, mean)": 0.5e-3,
        "controller ZOH (Ts/2)": ts / 2,
        "compute/release (ASSUMPTION ≤ Ts/4)": ts / 4,
    }
    return items, sum(items.values())


def loop(w, K, wc, T, alpha, wi_ratio=0.1, J=J_R):
    s = 1j * w
    wz, wp, wi = wc / math.sqrt(alpha), wc * math.sqrt(alpha), wi_ratio * wc
    C = K * (1 + wi / s) * (1 + s / wz) / (1 + s / wp)
    G_act = np.exp(-s * T) / (TAU_I * s + 1)
    P = 1 / (J * s ** 2 + B_VISC * s)
    return C, G_act, P


W_GRID = np.logspace(0, 4, 6000)


def loop_phase(w, wc, T, alpha, wi_ratio=0.1, J=J_R):
    """Continuous (unwrapped) phase of L(jw) in rad, summed per factor."""
    wz, wp, wi = wc / math.sqrt(alpha), wc * math.sqrt(alpha), wi_ratio * wc
    return (-np.arctan2(wi, w)                        # PI zero/integrator
            + np.arctan(w / wz) - np.arctan(w / wp)   # lead
            - w * T - np.arctan(w * TAU_I)            # delay + current lag
            - np.pi / 2 - np.arctan(w * J / B_VISC))  # 1/(s (J s + b))


def margins(K, wc, T, alpha, J=J_R, lower=False):
    """PM (deg), upper GM (dB); with lower=True also the lower GM (dB, negative):
    how far the gain may DROP (e.g. under saturation) before the loop destabilises."""
    w = W_GRID
    C, Ga, P = loop(w, K, wc, T, alpha, J=J)
    mag = np.abs(C * Ga * P)
    ph = loop_phase(w, wc, T, alpha, J=J)
    i_c = np.where(np.diff(np.sign(mag - 1)))[0]
    pm = min(180 + ph[i] * DEG for i in i_c) if len(i_c) else float("nan")
    # Upper gain margin only: the PI + double integrator also crosses -180 deg
    # at low frequency with |L| >> 1 (conditional stability, not a GM limit).
    x180 = np.where(np.diff(np.sign(ph + np.pi)))[0]
    i_180 = [i for i in x180 if mag[i] < 1]
    gm = min(-20 * np.log10(mag[i]) for i in i_180) if i_180 else float("inf")
    if lower:
        lo = [i for i in x180 if mag[i] > 1]
        return pm, gm, (max(-20 * np.log10(mag[i]) for i in lo) if lo else float("-inf"))
    return pm, gm


def gain_for(wc, T, alpha):
    C, Ga, P = loop(np.array([wc]), 1.0, wc, T, alpha)
    return 1 / abs(C * Ga * P)[0]


def max_wc(T, alpha, pm_req=45.0, gm_req=6.0):
    best = None
    for wc in np.linspace(5, 250, 246):
        K = gain_for(wc, T, alpha)
        pm, gm = margins(K, wc, T, alpha)
        if pm >= pm_req and gm >= gm_req:
            best = wc
    return best if best is not None else float("nan")


def section_delay():
    h("Delay budget")
    rows = []
    scen = {
        "nominal (CAN 1.2 ms, one-way)": delay_budget(can=1.2e-3),
        "worst normal (CAN 1.8 ms)": delay_budget(can=CAN_MAX),
        "burst (CAN 4 ms)": delay_budget(can=CAN_BURST),
        "conservative: fb 1.8 + cmd 4 ms burst": delay_budget(can=CAN_BURST, can_both_ways=True,
                                                             can_fb=CAN_MAX),
    }
    names = list(scen["worst normal (CAN 1.8 ms)"][0].keys())
    for n in names:
        rows.append([n] + [f"{scen[s][0][n] * 1e3:.2f}" for s in scen])
    rows.append(["**pure delay total (ms)**"] + [f"**{scen[s][1] * 1e3:.2f}**" for s in scen])
    rows.append(["+ current lag τi (rational)"] + ["1.20"] * len(scen))
    table(["component (ms)"] + list(scen), rows)
    _, T250 = delay_budget(f_ctrl=250.0)
    print(f"\nAt 250 Hz the same worst-normal budget is {T250 * 1e3:.2f} ms "
          f"(+{(T250 - scen['worst normal (CAN 1.8 ms)'][1]) * 1e3:.1f} ms) -> use 500 Hz.")
    return {k: v[1] for k, v in scen.items()}


def section_bandwidth(Ts):
    h("Feedback bandwidth permitted by the delay (PM ≥ 45°, GM ≥ 6 dB)")
    T_design = Ts["worst normal (CAN 1.8 ms)"]
    T_burst = Ts["burst (CAN 4 ms)"]
    T_cons = Ts["conservative: fb 1.8 + cmd 4 ms burst"]
    rows, designs = [], {}
    for alpha in (9, 16, 25):
        wc = max_wc(T_design, alpha)
        K = gain_for(wc, T_design, alpha)
        pmb, gmb = margins(K, wc, T_burst, alpha)
        pmc, gmc = margins(K, wc, T_cons, alpha)
        pmJ, _ = margins(K, wc, T_design, alpha, J=J_R * 1.55)
        _, _, gm_lo = margins(K, wc, T_design, alpha, lower=True)
        noise_A = K * alpha * ENC_LSB / K_T      # HF gain x 1 count
        designs[alpha] = (wc, K)
        rows.append([alpha, f"{wc:.0f} ({wc / 2 / math.pi:.1f} Hz)", f"{K:.2f}",
                     f"{pmb:.0f}° / {gmb:.1f} dB", f"{pmc:.0f}° / {gmc:.1f} dB",
                     f"{pmJ:.0f}°", f"{gm_lo:.1f} dB", f"{noise_A * 1e3:.0f} mA"])
    table(["lead ratio α", "max ωc rad/s", "Kp N·m/rad", "PM/GM @ burst",
           "PM/GM @ conservative", "PM @ J+55 % (payload)", "lower GM (gain drop)",
           "i per encoder count"], rows)
    print("\nLower GM: PI zero + double integrator is conditionally stable, but the gain would "
          "have to fall by >30 dB (saturation describing-function gain < 3 %) to destabilise "
          "the linear loop. So Run E's in/out-of-saturation cycling is better explained by "
          "integrator windup against an infeasible demand than by a linear instability.")
    return designs, T_design


def section_rejection(designs, T_design, bc):
    h("What feedback alone can do against the yaw disturbance")
    rows = []
    alpha = 16
    wc, K = designs[alpha]
    for k, r in RUNS_BC.items():
        w = 2 * math.pi * r["f"]
        C, Ga, P = loop(np.array([w]), K, wc, T_design, alpha)
        L = (C * Ga * P)[0]
        S, Tt = 1 / (1 + L), L / (1 + L)
        e_fb = abs(P[0] * S) * bc[k]["pk"]
        u_fb = abs(Tt) * bc[k]["pk"] / K_T
        # Feed-forward residual: 20 % coefficient error + phase error from a
        # 4 ms-old yaw measurement (|1 - e^{-jwT}| = 2 sin(wT/2)).
        resid = 0.20 + 2 * math.sin(w * 4e-3 / 2)
        e_ff = e_fb * resid
        rows.append([k, f"{abs(S):.2f}", f"{e_fb * DEG:.1f}°", f"{u_fb:.2f} A",
                     f"{resid:.0%}", f"{e_ff * DEG:.2f}°", f"{bc[k]['s']:.2f}"])
    table(["Run", "|S| designed", "fb-only err pk", "fb-only |T|·τ/Kt",
           "FF residual (20 % coef + 4 ms)", "err pk with FF", "|S| implied by log"], rows)
    print(f"\n(design α = {alpha}, ωc = {wc:.0f} rad/s, T = {T_design * 1e3:.1f} ms; "
          "linear, unsaturated, friction ignored)")


# ---------------------------------------------------------------------------
# 5. Sensing / quantization.
# ---------------------------------------------------------------------------
def section_sensing():
    h("Encoder and velocity estimate")
    table(["Quantity", "Value"], [
        ["Position LSB", f"{ENC_LSB:.2e} rad = {ENC_LSB * DEG:.4f}°"],
        ["Finite-difference velocity LSB @1 kHz", f"{ENC_LSB * 1000:.3f} rad/s"],
        ["Finite-difference velocity LSB @500 Hz", f"{ENC_LSB * 500:.3f} rad/s"],
        ["Friction speed scale", f"{V_FRIC} rad/s -> {ENC_LSB * 500 / V_FRIC:.0f}× below one LSB"],
        ["tanh friction comp on raw velocity", f"≈ sign(v): ±{TAU_C / K_T:.2f} A chatter"],
        ["Time to move 1 count at v_fric", f"{ENC_LSB / V_FRIC * 1e3:.0f} ms"],
    ])


# ---------------------------------------------------------------------------
# 6. Voltage / back-EMF headroom (worst: R +25 %, L +20 %, Vbus 20 V).
#    Usable phase voltage is uncertain: full bus (DC-equivalent model) vs
#    Vbus/sqrt(3) (sinusoidal commutation, SVPWM). Both reported.
# ---------------------------------------------------------------------------
def section_voltage():
    h("Voltage and back-EMF headroom")
    R, L = R_NOM * (1 + R_TOL), L_NOM * (1 + L_TOL)
    didt = 2 * I_MAX / TAU_I   # full-scale reversal at the current-loop rate
    rows = []
    for label, Vb in (("24 V", V_BUS_NOM), ("20 V", V_BUS_MIN)):
        for mode, V in (("DC-equiv (Vbus)", Vb), ("SVPWM (Vbus/√3)", Vb / math.sqrt(3))):
            head = V - R * I_MAX
            w_hold = head / K_E                     # max speed holding 3.2 A
            w_slew = (head - L * didt) / K_E        # ... while also reversing current
            i_stall = V / R
            rows.append([label, mode, f"{V:.1f}", f"{R * I_MAX:.1f}", f"{L * didt:.1f}",
                         f"{w_hold:.0f}", f"{max(w_slew, 0):.0f}", f"{i_stall:.1f}"])
    table(["Bus", "Usable V model", "V avail", "R·Imax", "L·di/dt (reversal)",
           "q̇ max @3.2 A rad/s", "q̇ max while reversing", "stall i A"], rows)
    print(f"\nElectrical time constant L/R = {L_NOM * (1 - L_TOL) / (R_NOM * (1 + R_TOL)) * 1e3:.2f}–"
          f"{L / (R_NOM * (1 - R_TOL)) * 1e3:.2f} ms vs. specified current lag {TAU_I * 1e3:.1f} ms.")
    # Speeds the runs actually need
    v_a = 1.875 * math.radians(90) / 1.3                         # min-jerk peak speed
    v_c = math.radians(20.7) * 2 * math.pi * 2.2                 # error oscillation
    print(f"Speeds needed: Run A ≈ {v_a:.1f} rad/s; Run C roll error motion ≤ {v_c:.1f} rad/s; "
          "Run D sweep speed unknown (voltage binds only if the sweep > ~2 Hz at ±80°).")


# ---------------------------------------------------------------------------
# 7. Thermal (I^2 R) - the link between poor rejection and derating.
# ---------------------------------------------------------------------------
def section_thermal(bc):
    h("Thermal")
    irms_c_min = math.sqrt(0.31) * I_MAX
    irms_c_ff = (bc["C"]["pk"] / math.sqrt(2)) / K_T
    table(["Quantity", "R = 1.8 Ω", "R = 2.25 Ω"], [
        ["P @ 3.2 A continuous", f"{I_MAX ** 2 * 1.8:.1f} W", f"{I_MAX ** 2 * 2.25:.1f} W"],
        ["P @ 2.4 A continuous", f"{I_DERATED ** 2 * 1.8:.1f} W", f"{I_DERATED ** 2 * 2.25:.1f} W"],
        [f"Run C RMS lower bound (31 % at 3.2 A → ≥{irms_c_min:.2f} A)",
         f"≥{irms_c_min ** 2 * 1.8:.1f} W", f"≥{irms_c_min ** 2 * 2.25:.1f} W"],
        [f"Run C ideal-FF coupling RMS ({irms_c_ff:.2f} A)",
         f"{irms_c_ff ** 2 * 1.8:.1f} W", f"{irms_c_ff ** 2 * 2.25:.1f} W"],
    ])
    print("\nSaturated time alone costs ≥ 2x the heat the disturbance itself requires; "
          "if derating is RMS-driven, poor rejection is also what triggers Run E's condition.")


# ---------------------------------------------------------------------------
# 8. Feasibility envelopes (inputs to the Phase 2 request policy).
# ---------------------------------------------------------------------------
def yaw_amax(f, tau_lim, reserve=0.2):
    """Largest yaw amplitude (rad) that roll can reject at 0 deg while keeping
    |d|max, Coulomb friction and a `reserve` fraction of the limit for feedback."""
    w = 2 * np.pi * f
    avail = tau_lim * (1 - reserve) - D_MAX - TAU_C
    return avail / (w * np.hypot(K_YV, K_YA * w))


def section_envelope():
    h("Request envelope: yaw ±A at f with roll held (20 % feedback reserve)")
    rows = []
    for f in (0.5, 1.0, 1.5, 2.2, 3.0):
        rows.append([f, f"±{yaw_amax(f, TAU_MAX) * DEG:.0f}°", f"±{yaw_amax(f, TAU_DER) * DEG:.0f}°"])
    table(["yaw f Hz", "A max @3.2 A", "A max @2.4 A"], rows)
    print("\nRun C (±75° @ 2.2 Hz) is outside the 2.4 A envelope -> must be reshaped/rejected "
          "when derated.")


# ---------------------------------------------------------------------------
# Figures
# ---------------------------------------------------------------------------
C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def style(ax):
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    for sp in ("left", "bottom"):
        ax.spines[sp].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)
    ax.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def figures(bc, m38, designs, T_design):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(FIGS, exist_ok=True)

    # Fig 1 - torque budget per run
    runs = ["A", "B", "C", "D (quasi-static fit)", "E (same load)"]
    lat = m38 * G * S_SHIFT
    comp = {  # peak-ish magnitudes of each contributor, N m
        "yaw coupling": [0, bc["B"]["pk"], bc["C"]["pk"], 0, 0],
        "gravity (+payload)": [TAU_G * math.sin(math.pi / 4), TAU_G * math.sin(math.radians(11.5)),
                               TAU_G * math.sin(math.radians(20.7)),
                               math.hypot(TAU_G, lat), math.hypot(TAU_G, lat)],
        "Coulomb friction": [TAU_C] * 5,
        "|d| bound": [D_MAX] * 5,
    }
    obs = [2.1, 3.1, 3.2, 3.2, 2.4]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    left = np.zeros(len(runs))
    for (name, vals), col in zip(comp.items(), (C1, C2, C3, C4)):
        ax.barh(runs, vals, left=left, color=col, height=0.55, label=name,
                edgecolor="white", linewidth=2)
        left += np.array(vals)
    ax.scatter([o * K_T for o in obs], runs, marker="D", s=40, color=INK, zorder=5,
               label="observed peak current × Kt")
    for x, lab in ((TAU_MAX, "3.2 A"), (TAU_DER, "2.4 A")):
        ax.axvline(x, color=INK2, ls="--", lw=1.2)
        ax.text(x, -0.55, lab, color=INK2, fontsize=8, ha="center", va="bottom")
    ax.set_ylim(4.5, -0.9)
    ax.set_xlabel("torque, N·m (model demand, stacked worst case)", color=INK2)
    ax.set_title("Model torque demand vs. capacity (inertial terms excluded)", color=INK,
                 fontsize=10, loc="left")
    style(ax)
    ax.legend(fontsize=8, frameon=False, loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p0_torque_budget.png"), dpi=160)
    plt.close(fig)

    # Fig 2 - max crossover vs pure delay
    Tgrid = np.linspace(2e-3, 12e-3, 21)
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    for alpha, col in ((9, C1), (16, C2), (25, C3)):
        ax.plot(Tgrid * 1e3, [max_wc(T, alpha) / (2 * math.pi) for T in Tgrid], color=col,
                lw=2, label=f"lead ratio α = {alpha}")
    for T, lab in ((T_design, "worst normal"),):
        ax.axvline(T * 1e3, color=INK2, ls="--", lw=1.2)
        ax.text(T * 1e3 + 0.1, ax.get_ylim()[1] * 0.92, lab, color=INK2, fontsize=8)
    ax.axhline(2.2, color=INK2, ls=":", lw=1.2)
    ax.text(2.1, 2.35, "Run C yaw frequency 2.2 Hz", color=INK2, fontsize=8, ha="left")
    ax.set_xlabel("pure loop delay, ms (plus 1.2 ms current lag)", color=INK2)
    ax.set_ylabel("max crossover, Hz", color=INK2)
    ax.set_title("Feedback bandwidth allowed at PM ≥ 45°, GM ≥ 6 dB", color=INK, fontsize=10,
                 loc="left")
    style(ax)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p0_bandwidth_vs_delay.png"), dpi=160)
    plt.close(fig)

    # Fig 3 - yaw request envelope
    f = np.linspace(0.2, 3.5, 200)
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.plot(f, yaw_amax(f, TAU_MAX) * DEG, color=C1, lw=2, label="3.2 A nominal")
    ax.plot(f, yaw_amax(f, TAU_DER) * DEG, color=C2, lw=2, label="2.4 A derated")
    for k, r in RUNS_BC.items():
        ax.scatter([r["f"]], [75], s=60, color=INK, zorder=5)
        ax.annotate(f"Run {k}", (r["f"], 75), textcoords="offset points", xytext=(6, 6),
                    fontsize=8, color=INK)
    ax.set_ylim(0, 180)
    ax.set_xlabel("yaw frequency, Hz", color=INK2)
    ax.set_ylabel("max yaw amplitude, ±deg", color=INK2)
    ax.set_title("Yaw motion roll can reject at 0° (20 % feedback reserve)", color=INK,
                 fontsize=10, loc="left")
    style(ax)
    ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p0_yaw_envelope.png"), dpi=160)
    plt.close(fig)

    # Fig 4 - D/E saturation fraction vs lateral payload mass
    ms = np.linspace(0.05, 2.0, 120)
    fig, ax = plt.subplots(figsize=(6.4, 3.4))
    ax.plot(ms, [100 * sat_fraction(m, TAU_MAX, n=4001) for m in ms], color=C1, lw=2,
            label="3.2 A (Run D)")
    ax.plot(ms, [100 * sat_fraction(m, TAU_DER, n=4001) for m in ms], color=C2, lw=2,
            label="2.4 A (Run E)")
    ax.axvspan(0.53, 1.15, color=GRID, alpha=0.7, lw=0)
    ax.text(0.84, 50, "range implied by\n+4.6° mean error", color=INK2, fontsize=8, ha="center")
    ax.axhline(38, color=INK2, ls=":", lw=1.2)
    ax.axvline(m38, color=INK2, ls="--", lw=1.2)
    ax.text(m38 - 0.03, 5, f"quasi-static fit {m38:.2f} kg", color=INK2, fontsize=8, ha="right")
    ax.text(1.70, 40, "Run D observed 38 %", color=INK2, fontsize=8, ha="right")
    ax.set_xlabel("payload mass shifted laterally 35 mm, kg (quasi-static sweep ±80°)",
                  color=INK2)
    ax.set_ylabel("time current-limited, %", color=INK2)
    ax.set_title("Lateral payload needed to be current-limited 38 % (static upper bound)",
                 color=INK, fontsize=10, loc="left")
    style(ax)
    ax.legend(fontsize=8, frameon=False, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "p0_payload_fit.png"), dpi=160)
    plt.close(fig)


def main():
    print("# Phase 0 numbers (generated by analysis/phase0.py)")
    h("Capacity")
    table(["Limit", "Torque N·m"], [["3.2 A", f"{TAU_MAX:.3f}"], ["2.4 A", f"{TAU_DER:.3f}"]])
    section_a()
    bc = section_bc()
    m38 = section_de()
    Ts = section_delay()
    designs, T_design = section_bandwidth(Ts)
    section_rejection(designs, T_design, bc)
    section_sensing()
    section_voltage()
    section_thermal(bc)
    section_envelope()
    figures(bc, m38, designs, T_design)
    print(f"\nFigures written to {os.path.relpath(FIGS, ROOT)}/")


if __name__ == "__main__":
    main()
