"""Generate report/task5.md: the Task 5 answer.

It leads with the prospective closed-loop test (exp/task5_prospective.py, prediction
registered before the runs) and keeps the historical motor-strength study
(exp/task5_prediction.py, retrospectively corrected in R2) as a secondary section rendered
by its own generator from its stored results. Runs after both packets in run_all.py.
"""
import json
import re
from pathlib import Path

from exp.task5_prediction import _markdown as historical_markdown

ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "report"

# Git chronology of the prospective test (commit IDs after the note.md history rewrite).
CHRONOLOGY = [("c0b065f", "registration v1: prediction, scorer, tests; no results"),
              ("7452ea8", "registration v2 after the independent pre-run review (scorer start-up fallback fix); "
                          "central prediction unchanged"),
              ("62a9ac6", "declare exp/evidence.py for manifests (the first attempt stopped before any simulation)"),
              ("1101240", "results of the ten registered runs")]


def _c(z):
    return f"{z['re']:+.4f} {z['im']:+.4f}j"


def prospective_section(res):
    reg, out = res["registration"]["prediction"], res["outcome"]
    # The prose below describes the published outcome. A reproduction with a different outcome
    # must not publish it: stop instead of writing contradictory text.
    if out["label"] != "supported" or out["problems"]:
        raise SystemExit(f"prospective outcome is {out['label']} ({out['problems']}): the Task 5 overview text "
                         "is written for the published 'supported' outcome and must be revised first")
    rows = res["rows"]
    mean = lambda d, k: sum(r[k] for r in rows if r["cmd_delay_s"] == d) / sum(1 for r in rows if r["cmd_delay_s"] == d)
    g0, g1 = mean(1e-3, "governed_rms_deg"), mean(5e-3, "governed_rms_deg")
    q0, q1 = mean(1e-3, "request_rms_deg"), mean(5e-3, "request_rms_deg")
    h0 = sum(r["H"]["gain"] for r in rows if r["cmd_delay_s"] == 1e-3) / 5
    h1 = sum(r["H"]["gain"] for r in rows if r["cmd_delay_s"] == 5e-3) / 5
    L = ["# Task 5 — Test your explanation", "",
         "## Prospective test: command delay and closed-loop tracking (prediction registered before the runs)", "",
         "**Explanation under test** (Task 1–2):",
         "",
         "- Tracking at these frequencies is governed by the loop's delay budget: the command path, CAN transport "
         "and feedback age against a controller designed for 7 ms.",
         "- The prediction: adding current-command delay should change the closed-loop response in a specific, "
         "calculable way.",
         "",
         "**Change:** current-command delay from 1 ms (nominal) to 5 ms, with everything else frozen:",
         "",
         "- a 5° roll sine at 3 Hz, yaw still, d(t) = 0;",
         f"- the frozen baseline `{res['registration']['baseline_fingerprint'][:16]}`;",
         "- seeds 301–305, paired.",
         "",
         "**Registered prediction** ([protocol](../docs/plans/task5-prospective-protocol.md), "
         f"[registration](task5_prospective_registration.json), sha256 `{res['registration_sha256'][:16]}`): "
         f"ΔH = H(5 ms) − H(1 ms) = **{_c(reg['dH'])}**. That is {reg['gain_change_db']:+.2f} dB of gain and "
         f"{reg['phase_change_deg']:+.1f}° of phase, with an acceptance disc of radius "
         f"{reg['acceptance']['radius']:.4f} that excludes zero.",
         "",
         "- **What H is:** actual roll divided by the governed reference, as the fundamental over [4, 12) s.",
         "- **Where the prediction comes from:** the linearized loop, the frozen discrete controller and "
         "feed-forward, the friction describing function, the current lag and configuration-derived timing. "
         "No data from these runs was used.",
         "",
         "**Chronology (git):**", ""]
    L += [f"1. `{c}`: {d}" for c, d in CHRONOLOGY]
    L += ["",
          f"**Result (Simulated):** mean measured ΔH = **{_c(out['mean_dH'])}**, a distance of "
          f"{out['distance']:.4f} from the prediction, inside the radius {out['radius']:.4f}. "
          f"**Outcome: {out['label']}.**",
          "",
          "- **Run quality:** all ten runs are valid: no fault events, suspension or rejection, and no fallback, "
          "clipping or governor limiting inside the scoring window.",
          "- **Consistency:** the five paired changes agree within ±0.0003.",
          f"- **Tracking error:** governed RMS error rises from {g0:.2f}° to {g1:.2f}°, and original-request "
          f"RMS error from {q0:.2f}° to {q1:.2f}°. Full rows are in "
          "[task5_prospective_numbers.md](task5_prospective_numbers.md) and the plot is "
          "[figs/task5_prospective.png](figs/task5_prospective.png).",
          "",
          "**What matched and what did not:**",
          "",
          "- **Matched:** the direction and size of the change. The extra delay mainly raises closed-loop "
          "peaking (gain up) rather than adding phase lag, as the phase-margin argument predicted.",
          f"- **Did not match:** the model's absolute gain is low. Measured |H(1 ms)| is {h0:.3f} against the "
          f"model's {reg['H_nominal']['mag']:.3f}, and |H(5 ms)| is {h1:.3f} against "
          f"{reg['H_delayed']['mag']:.3f}. The pre-run review expected this: about 0.5–1 ms of effective delay "
          "is not modelled. It shifts both arms similarly, and it moves ΔH only by about 0.002 per ms, well "
          "below the registered radius.",
          "",
          "**Revised explanation:**",
          "",
          "- The delay-budget mechanism is supported quantitatively at this one condition (3 Hz, +4 ms, "
          "simulation). Other frequencies, amplitudes and delays are untested.",
          "- The frozen loop tolerates +4 ms without faults or reshaping, but tracking error at 3 Hz more than "
          "doubles.",
          "- The absolute-gain offset is consistent with slightly more effective delay than the "
          "configuration-based estimate; it is not separately measured.",
          "",
          "**Smallest justified design change: none to the frozen controller.** At the nominal 1 ms delay it "
          "tracks this request to about 0.2° RMS, and even +4 ms stays well inside the 2° tracking threshold "
          "with no fault. The justified change is in qualification, not control:",
          "",
          "- measure the real command-path delay;",
          "- if it exceeds the 7 ms design assumption, re-derive the margins before widening the operating "
          "envelope.",
          "",
          "**Hardware confirmation:**",
          "",
          "- synchronized command-generation, drive-application and encoder timestamps;",
          "- a guarded low-amplitude 3 Hz roll sine at the nominal and at an added command delay;",
          "- measured gain and phase of roll against the governed reference, compared with this registered "
          "prediction.",
          "",
          "This is proposed, not performed (see the [qualification plan](hardware_qualification_plan.md)).",
          ""]
    return "\n".join(L)


def main():
    pro = json.loads((REPORT / "task5_prospective_results.json").read_text())
    hist = json.loads((REPORT / "task5_results.json").read_text())
    h = historical_markdown(hist)
    h = re.sub(r"^# .*\n", "", h, count=1)                       # demote the historical document
    h = re.sub(r"^## ", "### ", h, flags=re.M)
    text = (prospective_section(pro)
            + "## Historical study: motor strength (registered component prediction; analysis corrected "
              "retrospectively in R2)\n\n"
              "Kept as a secondary, qualified result:\n\n"
              "- **No ordering evidence:** its registration and results were first committed together, so git "
              "does not show the prediction came first.\n"
              "- **Metric chosen afterwards:** its closed-loop comparison was selected after the results.\n"
              "- **Not the primary evidence:** it does not satisfy the brief's prediction-before-test requirement; "
              "the prospective test above does.\n\n"
            + h)
    (REPORT / "task5.md").write_text(text)
    print("wrote report/task5.md")


if __name__ == "__main__":
    main()
