"""Current Task 2 evidence. Run: python -m exp.task2_eval (fixed seeds).

Historical phase reports are preserved. Outputs: report/task2_numbers.md,
report/task2_results.json, report/task2_runs.json (one manifest entry per
run_id, Packet 4A) and report/figs/task2_*.png, published together through
exp.manifest.staged_publish only after the whole evaluation succeeded and no
source file changed meanwhile (Packet 2C).
"""
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np

from ctrl.baseline import BaselineController
from ctrl import loopshape as LS
from exp import manifest as MF
from exp import motions as M
from exp import scenarios as S
from exp.evidence import RunBook, run_set_id
from exp.phase2_eval import legacy, sample_cfg, table
from sim.config import SimConfig
from sim.trajectories import Hold, RampedSine

ROOT = Path(__file__).resolve().parents[1]
DEG = 180/math.pi
PROVENANCE_SOURCES = ("exp/phase2_eval.py", "exp/evidence.py")
BOOK = RunBook("exp.task2_eval")


def evaluate(sc, seed=7, coordinated=True, make_ctrl=BaselineController, supervisor="design"):
    log, stats, rid, ev = BOOK.run(sc, make_ctrl, seed=seed, supervisor=supervisor,
                                   governed_yaw=coordinated)
    stats.update(reject_pct=100*float(np.nanmean(log.c_request_rejected)) if "c_request_rejected" in log else 0.0,
                 suspended_pct=100*float(np.nanmean(log.c_suspended)) if "c_suspended" in log else 0.0,
                 target_over_limit=float(np.max(np.abs(log.i_tgt)-log.i_lim)),
                 current_over_limit=float(np.max(np.abs(log.i)-log.i_lim)),
                 feedback_age_max_ms=1000*float(np.nanmax(log.fb_age)),
                 fault_kinds=sorted({e[1] for e in log.events}),
                 run_id=rid, tracked=bool(ev["tracked"]), net_progress=float(ev["progress"]["progress"]),
                 not_tracked_because=ev["not_tracked_because"])
    comp = ev.get("completion")
    if comp is not None:
        stats.update(completed=bool(comp["completed"]), t_complete=comp.get("t_complete"),
                     t_request=comp.get("t_request"))
    return log, stats


def fault_cases():
    base = SimConfig(); c = S.run_c(base)
    cases = [
        ('Feedback-only loss 60 ms', replace(c, cfg=c.cfg.with_(timing=dict(feedback_blackout=((3, 3.06),)))), True),
        ('Command-only loss 60 ms', replace(c, cfg=c.cfg.with_(timing=dict(command_blackout=((3, 3.06),)))), True),
        ('Both directions lost 60 ms', replace(c, cfg=c.cfg.with_(timing=dict(blackout=((3, 3.06),)))), True),
        ('Burst storm', replace(c, cfg=c.cfg.with_(timing=dict(burst_rate_hz=5, burst_len_s=(.05, .2)))), True),
        ('5% message loss', replace(c, cfg=c.cfg.with_(timing=dict(drop_prob=.05))), True),
        ('Mid-run derate + coupling x1.5', replace(c, cfg=c.cfg.with_(drive=dict(derate_schedule=((3, 2.4),)), plant=dict(k_yv=.012, k_ya=.0012))), True),
        ('Same derate without yaw coordination', replace(c, cfg=c.cfg.with_(drive=dict(derate_schedule=((3, 2.4),)), plant=dict(k_yv=.012, k_ya=.0012))), False),
        ('Combined corner: 4 ms CAN, J -30%, Kt +15%', replace(c, cfg=c.cfg.with_(timing=dict(can_min=.004, can_max=.004, burst_rate_hz=0), plant=dict(J=.0028, k_t=.161, k_e=.161))), True),
    ]
    cfg=replace(base, duration=8).with_(plant=dict(v_bus=20, R=2.25))
    cases.append(('20 V, R +25%, fast roll sweep', S.Scenario('V', cfg, RampedSine(math.radians(80), 1.5), Hold(0), 'electrical'), True))
    e=S.run_e(base)
    cases.append(('Payload E + feedback-only loss', replace(e, cfg=e.cfg.with_(timing=dict(feedback_blackout=((3, 3.1),)))), True))
    cases.append(('Payload E, CoM -35 mm + feedback-only loss', replace(e, cfg=e.cfg.with_(
        plant=dict(s_lat=-0.035), timing=dict(feedback_blackout=((3, 3.1),)))), True))
    cases.append(('C + 0.7 kg +35 mm + feedback loss, no yaw coordination', replace(c, cfg=c.cfg.with_(
        plant=dict(m_payload=0.7), timing=dict(feedback_blackout=((3, 3.1),)))), False))
    return cases


def ids_cell(ids):
    return f"`{run_set_id(ids)}`" if len(ids) > 1 else f"`{ids[0]}`"


def med(samples, key):
    return float(np.median([x[key] for x in samples]))


def main():
    base = SimConfig()
    records = {}; keep = {}
    rows = []
    for sc in S.all_runs():
        samples = []; before = []; ids = []
        for seed in (1, 2, 3, 4, 5):
            log, stats = evaluate(sc, seed); samples.append(stats); ids.append(stats['run_id'])
            _, old = evaluate(sc, seed, coordinated=False, make_ctrl=legacy, supervisor='legacy')
            before.append(old); ids.append(old['run_id'])
            if seed == 1: keep[sc.name] = log
        records[sc.name] = samples; records[sc.name + '_legacy'] = before
        rows.append([sc.name, f"{med(before, 'rms'):.2f}",
                     f"{med(samples, 'rms'):.2f} / {med(samples, 'peak'):.2f}",
                     f"{med(samples, 'rms_gov'):.2f} / {med(samples, 'peak_gov'):.2f}",
                     f"{med(samples, 'net_progress'):.0%}", f"{med(samples, 'i_rms'):.2f}",
                     f"{med(samples, 'clip_pct'):.1f}%", f"{med(samples, 'sat_entries'):.0f}",
                     f"{med(samples, 'events'):.0f}", f"{sum(x['tracked'] for x in samples)}/5",
                     ids_cell(ids)])
        print('Completed A-E case', sc.name, flush=True)
    out = ['## A–E: median of five seeds', '',
           table(['Run', 'Legacy RMS °', 'Original-request RMS / peak °', 'Governed RMS / peak °',
                  'Net path progress', 'RMS current A', 'At command limit', 'Sat. entries', 'Events',
                  'Tracked (4A)', 'Run set'], rows), '',
           'Net path progress is the governor clock sigma reached over the request (Packet 4A metric), not a '
           'measured velocity ratio. Original-request error uses the original wall clock; it intentionally reveals '
           'timing sacrificed by reshaping. "Tracked" applies the 4A rule: progress >= 95 %, request-window path RMS '
           '<= 2 deg, no rejection, suspension or tracking fault (thresholds are project assumptions). '
           'Current-limit occupancy counts clamped commands, not measured-current occupancy.', '']

    rows = []; records['motions'] = {}
    for mo in M.all_motions(base):
        samples = [evaluate(mo, seed)[1] for seed in (1, 2, 3, 4, 5)]
        records['motions'][mo.name] = samples
        done = [x for x in samples if x.get('completed')]
        rows.append([mo.name, mo.note, f"{len(done)}/5",
                     f"{np.median([x['t_complete'] for x in done]):.2f}" if done else '–',
                     f"{samples[0]['t_request']:.2f}", f"{med(samples, 'rms_gov'):.2f}",
                     f"{sum(x['wd_trips'] for x in samples)}", f"{sum(x['suspended_pct'] > 0 for x in samples)}/5",
                     f"{sum(x['tracked'] for x in samples)}/5", ids_cell([x['run_id'] for x in samples])])
        print('Completed motion', mo.name, flush=True)
    out += ['## Finite motions: five seeds (Packet 4A sequences)', '',
            table(['Motion', 'Request', 'Completed', 'Median completion s', 'Requested s', 'Governed RMS °',
                   'WD trips (total)', 'Runs suspended', 'Tracked (4A)', 'Run set'], rows), '',
            'Completion requires visiting every waypoint and settling at the last while the drive and host are '
            'healthy and the request is neither rejected nor suspended. A controller that stops or rejects cannot '
            'complete. M2 is the loaded finite move that tripped 27 times before Packet 2B.', '']

    rows = []; fault_logs = {}; records['faults'] = {}
    for name, sc, coordinated in fault_cases():
        log, s = evaluate(sc, coordinated=coordinated); records['faults'][name] = s; fault_logs[name] = log
        rows.append([name, f"{s['rms']:.2f}", f"{s['rms_gov']:.2f} / {s['peak_gov']:.2f}", f"{s['net_progress']:.0%}",
                     f"{s['clip_pct']:.1f}%", f"{s['fallback_pct']:.1f}%", f"{s['reject_pct']:.1f}%",
                     f"{s['suspended_pct']:.1f}%", f"{s['yaw_scale']:.2f}",
                     ', '.join(s['fault_kinds']) or 'none', 'yes' if s['tracked'] else 'no', f"`{s['run_id']}`"])
    out += ['## Faults and limits: seed 7', '', table(['Case', 'Original RMS °', 'Governed RMS / peak °',
            'Net path progress', 'At command limit', 'Fallback', 'Host rejected', 'Suspended', 'Yaw scale',
            'Events', 'Tracked (4A)', 'run_id'], rows), '',
            'The fallback column reports bounded local damping, not position holding. After a tracking fault the '
            'request is suspended (braked and held) until a replan; without yaw coordination an incompatible '
            'request is suspended with a coordinated-stop request (Packet 2B). Small governed error during '
            'fallback or suspension is not successful tracking.', '']

    rng = np.random.default_rng(2024); records['montecarlo'] = {}; rows = []
    for mk in (S.run_a, S.run_b, S.run_c, S.run_d, S.run_e):
        samples = []
        for k in range(20):
            b, _ = sample_cfg(rng, SimConfig())
            b = b.with_(plant=dict(k_e=b.plant.k_t))
            sc = mk(b)
            # D/E constructors set payload to 0.7 kg; deliberately perturb around it.
            if sc.name in ('D', 'E'):
                sc = replace(sc, cfg=sc.cfg.with_(plant=dict(m_payload=float(rng.uniform(.3, 1.0)))))
            sc = replace(sc, cfg=replace(sc.cfg, duration=6))
            _, s = evaluate(sc, seed=100 + k); samples.append(s)
        records['montecarlo'][sc.name] = samples
        pct = lambda key, p: float(np.percentile([x[key] for x in samples], p))
        rows.append([sc.name, f"{pct('rms', 95):.2f}", f"{pct('rms_gov', 95):.2f}", f"{pct('peak_gov', 95):.2f}",
                     f"{pct('net_progress', 5):.0%}", f"{pct('clip_pct', 95):.1f}%",
                     f"{sum(x['events'] > 0 for x in samples)}/20", f"{sum(x['wd_trips'] for x in samples)}",
                     f"{sum(x['suspended_pct'] > 0 for x in samples)}/20", f"{sum(x['tracked'] for x in samples)}/20",
                     ids_cell([x['run_id'] for x in samples])])
        print('Completed uncertainty case', sc.name, flush=True)
    out += ['## Uncertainty: 20 sampled plants per run', '',
            'Seed 2024 generates J ±30%, damping ±50%, Coulomb friction x0.5–2, gravity ±30%, Kt ±15%, '
            'coupling x0.5–1.5, R ±25%, L ±20%, bus 20–24 V, encoder offset ±0.01 rad and bursts 0–2/s. '
            'Payload is 0–0.3 kg in A–C and 0.3–1.0 kg in D/E, shifted laterally 35 mm. '
            'These are chosen stress ranges, not calibrated probability distributions. K_e tracks K_t in the matching SI convention.', '',
            table(['Run', 'Original RMS p95 °', 'Governed RMS p95 °', 'Governed peak p95 °', 'Net progress p5',
                   'At limit p95', 'Trials with events', 'WD trips (total)', 'Trials suspended', 'Tracked (4A)',
                   'Run set'], rows), '']

    c = BaselineController(); rows = []
    for name, T, J, kt in [('nominal', .006, .004, 1), ('design', .007, .004, 1), ('burst', .011, .004, 1),
                           ('combined light/strong corner', .011, .0028, 1.15), ('heavy/weak corner', .011, .0052, .85)]:
        pm, gm, w = LS.margins(c.K*kt, c.wc, T, c.alpha, J=J)
        rows.append([name, f'{pm:.1f}', f'{gm:.1f}', f'{w/(2*math.pi):.2f}'])
    out += ['## Analytic loop margins (Calculated)', '', f'K = {c.K:.6f} N·m/rad; design omega = {c.wc:.1f} rad/s; '
            f'alpha = {c.alpha}; integral zero = {c.wi_ratio*c.wc:.2f} rad/s. '
            'Continuous-time approximation at nominal roll equilibrium; nonlinear sweeps are evaluated separately.', '',
            table(['Condition', 'PM degrees', 'GM dB', 'Crossover Hz'], rows), '']
    allstats = [s for key in 'ABCDE' for s in records[key]] + list(records['faults'].values())
    allstats += [s for samples in records['montecarlo'].values() for s in samples]
    allstats += [s for samples in records['motions'].values() for s in samples]
    maximum = max(s['target_over_limit'] for s in allstats)
    assert maximum < 1e-9, 'A delayed target exceeded the active current limit'
    out += [f'Maximum applied-target excess above the active limit across all baseline trials: {max(0, maximum):.3g} A. '
            'Measured current may briefly exceed a newly reduced limit while the modeled current loop decays; this is recorded separately in JSON.', '',
            'Figures: [tracking](figs/task2_tracking.png), [feedback-loss timeline](figs/task2_feedback_loss.png).']
    fp = BOOK.fingerprint()
    head = ['# Task 2 — regenerated evidence', '',
            'Generated by `python -m exp.task2_eval`. Simulation only (**Simulated**, except the **Calculated** margins); '
            'whole-run metrics include startup and faults. A–E use the documented reconstruction assumptions, not raw '
            'hardware trajectories.', '',
            f"**Frozen baseline fingerprint:** `{fp['baseline'][:16]}` (sha256 over the controller and simulator "
            f"modules listed in exp/evidence.py BASELINE_SOURCES). Run manifests use the wider declared set "
            f"(code hash `{fp['code_hash'][:16]}`; per-file sha256 in [task2_runs.json](task2_runs.json)); "
            f"git `{MF.git_commit_text(fp['git'])}`, baseline sources {MF.git_sources_text(fp['git'])}"
            f" (uncommitted files in the wider set: "
            f"{MF.git_dirty_text(fp['run_set_git'])}); "
            f"metrics version {fp['metrics_version']}. Controller: `BaselineController()` defaults "
            "(yaw_info='estimate', governor and yaw monitor on) with `DriveSupervisor()` defaults. "
            f"{len(BOOK.runs)} runs; every row cites its run_id or run-set id (the set's run_ids are listed in "
            "[task2_results.json](task2_results.json)).", '']
    with MF.staged_publish(ROOT / 'report', check=BOOK.check_unchanged) as stage:
        plot(keep, fault_logs, stage / 'figs')
        (stage / 'task2_numbers.md').write_text('\n'.join(head + out) + '\n')
        MF.write_json(records, stage / 'task2_results.json')
        MF.write_json(BOOK.record(), stage / 'task2_runs.json')
    print('Published report/task2_numbers.md, task2_results.json, task2_runs.json and figures', flush=True)


def plot(keep, fault_logs, figdir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11,8))
    for row,key in enumerate(('B','C','E')):
        lg=keep[key]; ax=axes[row,0]
        ax.plot(lg.t,lg.q_ref*DEG,label='Original request',alpha=.6)
        ax.plot(lg.t,lg.c_q_c*DEG,label='Governed reference')
        ax.plot(lg.t,lg.q*DEG,label='Actual roll',lw=.9)
        ax.set_ylabel(f'Run {key}: roll (deg)')
        ax=axes[row,1];ax.plot(lg.t,lg.i,label='Measured current',lw=.8)
        ax.plot(lg.t,lg.i_lim,'k--',label='Active limit');ax.plot(lg.t,-lg.i_lim,'k--')
        ax.set_ylabel('Current (A)')
    for ax in axes.flat: ax.grid(alpha=.2);ax.set_xlabel('Time (s)')
    axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8)
    fig.suptitle('Task 2: tracking and delivered motion (seed 1)');fig.tight_layout()
    figdir.mkdir(parents=True, exist_ok=True); fig.savefig(figdir/'task2_tracking.png',dpi=140);plt.close(fig)
    lg=fault_logs['Feedback-only loss 60 ms']; fig,axes=plt.subplots(3,1,figsize=(9,6),sharex=True)
    axes[0].plot(lg.t,lg.err*DEG);axes[0].set_ylabel('Original error (deg)')
    axes[1].plot(lg.t,lg.fb_age*1000,label='Feedback age');axes[1].plot(lg.t,lg.cmd_age*1000,label='Command age')
    axes[1].set_ylabel('Age (ms)');axes[1].legend(fontsize=8)
    axes[2].step(lg.t,lg.mode,where='post',label='Drive fallback');axes[2].step(lg.t,lg.c_host_fallback,where='post',label='Host recovery/fallback')
    axes[2].set_ylabel('Active (0/1)');axes[2].legend(fontsize=8)
    for ax in axes: ax.axvspan(3,3.06,color='orange',alpha=.15);ax.grid(alpha=.2);ax.set_xlim(2.9,3.3)
    axes[2].set_xlabel('Time (s)');fig.suptitle('Feedback-only outage: fresh commands do not conceal lost sensing');fig.tight_layout()
    fig.savefig(figdir/'task2_feedback_loss.png',dpi=140);plt.close(fig)


if __name__=='__main__':
    main()
