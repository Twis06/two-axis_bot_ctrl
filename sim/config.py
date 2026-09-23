"""Scenario configuration: nominal values from sim/params.py plus the knobs the
experiments perturb. Everything a run depends on lives in a SimConfig, so a
(config, seed) pair fully determines a result.
"""
from dataclasses import dataclass, field, replace
import math

from sim import params as P


@dataclass(frozen=True)
class PlantConfig:
    J: float = P.J_R
    b: float = P.B_VISC
    tau_c: float = P.TAU_C
    v_fric: float = P.V_FRIC
    tau_g: float = P.TAU_G
    # Payload: point mass m_p whose CoM is displaced by s_lat perpendicular to
    # the nominal gravity lever arm. Adds tau_lat*cos(q) and m_p*s_lat^2 inertia.
    m_payload: float = 0.0
    s_lat: float = 0.035
    k_yv: float = P.K_YV
    k_ya: float = P.K_YA
    k_t: float = P.K_T              # true motor constant (motor-strength uncertainty)
    tau_i: float = P.TAU_I
    # Electrical (voltage-limit model)
    R: float = P.R_NOM
    L: float = P.L_NOM
    k_e: float = P.K_E
    v_bus: float = P.V_BUS_NOM
    # Usable fraction of bus voltage: 1/sqrt(3) for sinusoidal commutation
    # (SVPWM). ASSUMPTION - the conservative choice from Phase 0.
    v_util: float = 1 / math.sqrt(3)
    voltage_limit: bool = True
    # Unknown disturbance d(t): band-limited, |d| <= d_amp.
    d_amp: float = P.D_MAX
    d_bw_hz: float = 3.0

    @property
    def tau_lat(self):
        return self.m_payload * P.G * self.s_lat

    @property
    def J_total(self):
        return self.J + self.m_payload * self.s_lat ** 2

    @property
    def v_avail(self):
        return self.v_bus * self.v_util


@dataclass(frozen=True)
class TimingConfig:
    f_drive: float = P.F_CURRENT     # drive tick: current-loop command + safety
    f_ctrl: float = 500.0            # host position/velocity loop
    dt_sim: float = 1e-4             # plant integration step (RK4)
    t_cmd_delay: float = P.T_CMD_DELAY
    t_compute: float = 0.5e-3        # host compute -> command release (ASSUMPTION)
    can_min: float = P.CAN_MIN
    can_max: float = P.CAN_MAX
    can_burst: float = P.CAN_BURST
    burst_rate_hz: float = 0.5       # burst episodes per second (ASSUMPTION)
    burst_len_s: tuple = (0.02, 0.10)  # episode duration range (ASSUMPTION)
    feedback_over_can: bool = True   # encoder feedback also crosses CAN
    drop_prob: float = 0.0           # message loss (fault injection)
    # Fault injection: CAN silent (all messages lost) in [t0, t1)
    blackout: tuple = ()


@dataclass(frozen=True)
class DriveConfig:
    i_limit: float = P.I_MAX
    # Optional derating schedule: list of (t_start, new_limit)
    derate_schedule: tuple = ()
    cmd_timeout: float = 0.010       # s without a fresh command -> fallback
    fallback_damping: float = 0.05   # N m s/rad, drive-local damping in fallback


@dataclass(frozen=True)
class SensorConfig:
    enc_bits: int = P.ENC_BITS
    enc_offset: float = 0.0          # rad, calibration offset (uncertainty)


@dataclass(frozen=True)
class SimConfig:
    plant: PlantConfig = field(default_factory=PlantConfig)
    timing: TimingConfig = field(default_factory=TimingConfig)
    drive: DriveConfig = field(default_factory=DriveConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    duration: float = 10.0
    q0: float = 0.0

    def with_(self, **groups):
        """cfg.with_(plant=dict(m_payload=1.0), timing=dict(f_ctrl=250))"""
        kw = {}
        for name, changes in groups.items():
            if isinstance(changes, dict):
                kw[name] = replace(getattr(self, name), **changes)
            else:
                kw[name] = changes
        return replace(self, **kw)
