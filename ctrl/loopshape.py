"""Continuous-time loop shaping used to *derive* the baseline gains.

Loop: C(s) = K (1 + wi/s) (1 + s/wz)/(1 + s/wp)
      G(s) = e^{-sT} / (tau_i s + 1)
      P(s) = 1 / (J s^2 + b s + tau_g)          (gravity linearised at q = 0)
Lead centred on wc: wz = wc/sqrt(alpha), wp = wc*sqrt(alpha); wi = wi_ratio*wc.
"""
import math

import numpy as np

from sim import params as P

DEG = 180 / math.pi
W_GRID = np.logspace(0, 4, 8000)


def freq_response(w, K, wc, T, alpha, wi_ratio=0.1, J=P.J_R, b=P.B_VISC, k_g=P.TAU_G):
    s = 1j * w
    wz, wp, wi = wc / math.sqrt(alpha), wc * math.sqrt(alpha), wi_ratio * wc
    C = K * (1 + wi / s) * (1 + s / wz) / (1 + s / wp)
    G = np.exp(-s * T) / (P.TAU_I * s + 1)
    Pl = 1 / (J * s ** 2 + b * s + k_g)
    return C, G, Pl


def phase(w, wc, T, alpha, wi_ratio=0.1, J=P.J_R, b=P.B_VISC, k_g=P.TAU_G):
    """Unwrapped phase of L = C G P, summed per factor (rad)."""
    wz, wp, wi = wc / math.sqrt(alpha), wc * math.sqrt(alpha), wi_ratio * wc
    plant = -np.arctan2(b * w, k_g - J * w ** 2)          # continuous in w, in (-pi, 0]
    return (-np.arctan2(wi, w) + np.arctan(w / wz) - np.arctan(w / wp)
            - w * T - np.arctan(w * P.TAU_I) + plant)


def margins(K, wc, T, alpha, **kw):
    """(phase margin deg, upper gain margin dB, crossover rad/s)."""
    w = W_GRID
    C, G, Pl = freq_response(w, K, wc, T, alpha, **kw)
    mag = np.abs(C * G * Pl)
    ph = phase(w, wc, T, alpha, **kw)
    ic = np.where(np.diff(np.sign(mag - 1)))[0]
    if not len(ic):
        return float("nan"), float("nan"), float("nan")
    i = ic[-1]
    pm = 180 + ph[i] * DEG
    x = [j for j in np.where(np.diff(np.sign(ph + np.pi)))[0] if mag[j] < 1]
    gm = min(-20 * np.log10(mag[j]) for j in x) if x else float("inf")
    return float(pm), float(gm), float(w[i])


def gain_for(wc, T, alpha, **kw):
    C, G, Pl = freq_response(np.array([wc]), 1.0, wc, T, alpha, **kw)
    return float(1 / abs(C * G * Pl)[0])


def design(T_design, alpha=16, pm_req=45.0, gm_req=6.0, T_check=None, pm_check=30.0,
           wc_grid=np.linspace(10, 120, 221)):
    """Largest crossover meeting PM/GM at T_design and PM >= pm_check at T_check."""
    best = None
    for wc in wc_grid:
        K = gain_for(wc, T_design, alpha)
        pm, gm, _ = margins(K, wc, T_design, alpha)
        ok = pm >= pm_req and gm >= gm_req
        if ok and T_check is not None:
            ok = margins(K, wc, T_check, alpha)[0] >= pm_check
        if ok:
            best = (float(wc), K)
    if best is None:
        raise ValueError("no feasible design")
    return best
