"""Build a calculated static-holdability diagram for the submission report.

This uses the supplied nominal gravity model and the explicitly assumed INF-P
load; it runs no control simulation and makes no inference about D/E payloads.
"""
import inspect
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ctrl.governor import RollGovernor
from exp.task4b_eval import infeasible_cases
from sim import params as P
from sim.config import SimConfig


def _inf_p_and_budget():
    """INF-P payload and the governor budget taken from the sources that define them,
    so the figure cannot drift from the published experiment or the frozen governor."""
    plant = infeasible_cases(SimConfig())[0].cfg.plant
    reserve = inspect.signature(RollGovernor.__init__).parameters["reserve"].default
    return plant.m_payload * P.G * plant.s_lat, 1.0 - reserve


def render(path=None):
    path = Path(path) if path is not None else ROOT / "report/figs/static_holdability.png"
    angle_deg = np.linspace(-90.0, 90.0, 1801)
    q = np.deg2rad(angle_deg)
    lateral_moment, budget_frac = _inf_p_and_budget()
    nominal = np.abs(P.TAU_G * np.sin(q)) + P.TAU_C
    inf_p = np.abs(P.TAU_G * np.sin(q) + lateral_moment * np.cos(q)) + P.TAU_C
    derated_capacity = P.K_T * P.I_DERATED
    nominal_capacity = P.K_T * P.I_MAX
    admission_budget = budget_frac * derated_capacity - P.D_MAX

    assert math.isclose(derated_capacity, 0.336, abs_tol=1e-12)
    assert lateral_moment > derated_capacity
    assert nominal[len(q) // 2] < admission_budget

    plt.rcParams.update({"font.size": 11.5, "axes.spines.top": False,
                         "axes.spines.right": False, "savefig.facecolor": "white",
                         "svg.fonttype": "none", "svg.hashsalt": "static-holdability-v1"})
    fig, (ax, band) = plt.subplots(2, 1, figsize=(9.4, 6.4), dpi=170,
                                   gridspec_kw={"height_ratios": [3.2, 1.15], "hspace": 0.16},
                                   sharex=True)
    fig.suptitle("Static holdability depends on load and current limit", x=0.10,
                 y=0.98, ha="left", fontsize=16, fontweight="bold")

    ax.plot(angle_deg, nominal, color="#24699a", linewidth=2.4,
            label="Nominal model + friction")
    ax.plot(angle_deg, inf_p, color="#b54b2b", linewidth=2.4,
            label="INF-P assumption + friction")
    ax.axhline(nominal_capacity, color="#777777", linewidth=1.5, linestyle="--",
               label="3.2 A capacity · 0.448 N·m")
    ax.axhline(derated_capacity, color="#222222", linewidth=1.7, linestyle="--",
               label="2.4 A capacity · 0.336 N·m")
    ax.axhline(admission_budget, color="#41836b", linewidth=1.7, linestyle=":",
               label="2.4 A governor budget · 0.219 N·m")
    ax.scatter([0], [lateral_moment + P.TAU_C], color="#b54b2b", zorder=5, s=36)
    ax.annotate("At 0°: gravity = 0.412 N·m; with friction\nallowance = 0.452 N·m > 0.336 N·m limit",
                xy=(0, lateral_moment + P.TAU_C), xytext=(-84, 0.58),
                arrowprops={"arrowstyle": "->", "color": "#8a3d26", "lw": 1.1},
                color="#713421", fontsize=10.5, va="top")
    ax.set_ylabel("Holding torque (N·m)")
    ax.set_ylim(0, 0.67)
    ax.set_xlim(-90, 90)
    ax.set_xticks(np.arange(-90, 91, 30))
    ax.grid(alpha=0.18)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.64, 0.94),
               frameon=False, fontsize=9.5, ncol=2, columnspacing=1.2)

    bands = [
        ("Nominal model: governor admits", nominal <= admission_budget, "#24699a"),
        ("INF-P: derated torque-capable", inf_p <= derated_capacity, "#b54b2b"),
        ("INF-P: if load known to governor", inf_p <= admission_budget, "#41836b"),
    ]
    for index, (_, mask, color) in enumerate(bands):
        y = 2 - index
        band.fill_between(angle_deg, y - 0.28, y + 0.28, color="#eee5e3", linewidth=0)
        band.fill_between(angle_deg, y - 0.28, y + 0.28, where=mask,
                          color=color, alpha=0.88, linewidth=0)
    band.set_yticks([2, 1, 0], [b[0] for b in bands])
    band.set_ylim(-0.52, 2.52)
    band.set_xlabel("Roll angle (degrees)")
    band.axvline(0, color="#555555", linestyle=":", linewidth=1)
    band.grid(axis="x", alpha=0.18)
    band.tick_params(axis="y", length=0, labelsize=10.5)
    fig.text(0.30, 0.035,
             "Colored bands pass the static torque test; pale regions fail. Motion also needs\n"
             "acceleration, braking and voltage headroom. INF-P is assumed, not D/E data.",
             fontsize=9.4, color="#454545", va="bottom")
    fig.subplots_adjust(left=0.30, right=0.98, top=0.79, bottom=0.18)
    path.parent.mkdir(parents=True, exist_ok=True)
    metadata = ({"Date": None} if path.suffix == ".svg" else
                {"CreationDate": None, "ModDate": None} if path.suffix == ".pdf" else None)
    fig.savefig(path, dpi=170, metadata=metadata)
    if path.suffix == ".svg":
        path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    plt.close(fig)
    return path


if __name__ == "__main__":
    for suffix in ("png", "svg", "pdf"):
        print(render(ROOT / f"report/figs/static_holdability.{suffix}"))
