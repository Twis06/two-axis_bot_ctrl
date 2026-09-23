"""Nominal plant, actuator, electrical and timing parameters (SI units).

Every value here is copied from the assessment brief; anything that is an
assumption rather than a given is marked ASSUMPTION.
"""
import math

# --- Roll-axis mechanics -----------------------------------------------------
J_R = 0.004          # kg m^2, roll inertia
B_VISC = 0.012       # N m s/rad, viscous damping
TAU_C = 0.040        # N m, Coulomb friction (nominal)
V_FRIC = 0.02        # rad/s, tanh friction speed scale
TAU_G = 0.120        # N m, gravity moment amplitude (nominal payload)
D_MAX = 0.05         # N m, bound on unknown disturbance |d(t)|

# --- Cross-axis coupling: tau_couple = K_YV * qy_dot + K_YA * qy_ddot ---------
K_YV = 0.008         # N m s/rad
K_YA = 0.0008        # N m s^2/rad

# --- Actuator ------------------------------------------------------------------
K_T = 0.140          # N m/A
TAU_I = 0.0012       # s, current-loop first-order lag
I_MAX = 3.2          # A, nominal current limit
I_DERATED = 2.4      # A, thermally derated limit
T_CMD_DELAY = 0.001  # s, current-command delay

# --- Electrical (for voltage/back-EMF checks) --------------------------------
R_NOM, R_TOL = 1.8, 0.25       # ohm, +/-25 %
L_NOM, L_TOL = 0.45e-3, 0.20   # H, +/-20 %
K_E = K_T                      # V s/rad (SI: Ke == Kt)
V_BUS_NOM, V_BUS_MIN = 24.0, 20.0

# --- Timing and sensing ------------------------------------------------------
F_CURRENT = 1000.0             # Hz, current loop + safety checks
F_CTRL_MAX = 500.0             # Hz, position/velocity loop upper bound
F_POLICY_MAX = 50.0            # Hz, learned residual upper bound
CAN_MIN, CAN_MAX, CAN_BURST = 0.6e-3, 1.8e-3, 4.0e-3   # s
POLICY_INFER = (2e-3, 8e-3)    # s
ENC_BITS = 14
ENC_LSB = 2 * math.pi / 2**ENC_BITS   # rad per count

G = 9.81
