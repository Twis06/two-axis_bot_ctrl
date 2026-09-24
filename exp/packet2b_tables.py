"""Markdown tables for report/packets/2B.md from packet2b_eval.py output.
    python3 exp/packet2b_tables.py report/packets/2B_evidence
(the directory holds before.json = pre-2B tree a03bb1b, after.json = 2B tree)."""
import json, sys, numpy as np
S = sys.argv[1]
B=json.load(open(S+'/before.json')); A=json.load(open(S+'/after.json'))
med=lambda xs: float(np.median(xs))
print('## Outage matrix (seeds 1-5, 100 ms outage at 3.0 s; medians, [max])')
print('| Motion | Payload | Outage | Trips pre → post | Re-arms pre → post | Fallback % pre → post | Fallback displacement ° pre → post | Peak q̇ in fallback rad/s pre → post | Peak |i| A pre → post | Governed peak ° pre → post | Original RMS after 4 s ° pre → post | Final q ° pre → post | Progress pre → post | Suspended at end (post) | Lockout (post) |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|')
keys=sorted({(r['motion'],tuple(r['payload']),r['outage']) for r in B['outages']}, key=lambda k:(k[0],k[1],k[2]))
for k in keys:
    b=[r for r in B['outages'] if (r['motion'],tuple(r['payload']),r['outage'])==k]
    a=[r for r in A['outages'] if (r['motion'],tuple(r['payload']),r['outage'])==k]
    f=lambda rs,key: '%g [%g]'%(round(med([r[key] for r in rs]),1), round(max(r[key] for r in rs),1))
    pl='none' if k[1][0]==0 else '%.1f kg %+d mm'%(k[1][0],round(k[1][1]*1000))
    g=lambda rs,key: '%g'%round(med([r[key] for r in rs]),2) if key in rs[0] else '—'
    print('| %s | %s | %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %s → %s | %d/5 | %d/5 |'%(
        k[0],pl,k[2].replace('_blackout','').replace('blackout','both'),
        f(b,'trips'),f(a,'trips'),f(b,'rearms'),f(a,'rearms'),f(b,'fallback_pct'),f(a,'fallback_pct'),
        f(b,'fallback_disp_deg'),f(a,'fallback_disp_deg'),f(b,'qd_peak_fallback'),f(a,'qd_peak_fallback'),
        f(b,'i_peak'),f(a,'i_peak'),f(b,'peak_gov_deg'),f(a,'peak_gov_deg'),
        f(b,'orig_rms_late_deg'),f(a,'orig_rms_late_deg'),f(b,'q_final_deg'),f(a,'q_final_deg'),
        g(b,'progress'),g(a,'progress'),
        sum(r['suspended'] for r in a), sum(r['lockout'] for r in a)))
print()
print('## A-E (seeds 1-5 medians): governed RMS / peak °, path-clock rate, events')
print('| Run | pre (plan look-ahead) | post, estimate (default) | post, plan mode |')
print('|---|---|---|---|')
for n in 'ABCDE':
    row=[]
    for rs in (B['ae'],A['ae'],A.get('ae_plan',[])):
        x=[r for r in rs if r['run']==n]
        if not x: row.append('—'); continue
        row.append('%.2f / %.2f, %.0f%%, ev %g, yaw %.2f'%(med([r['rms_gov'] for r in x]),med([r['peak_gov'] for r in x]),
                   100*med([r['speed'] for r in x]),med([r['events'] for r in x]),med([r['yaw_scale'] for r in x])))
    print('| %s | %s |'%(n,' | '.join(row)))
print()
print('## Monte Carlo (20 sampled plants per run, seed 2024)')
print('| Run | governed peak p95 ° pre → post | trials with events pre → post | watchdog trips total pre → post | suspended (post) |')
print('|---|---|---|---|---|')
for n in 'ABCDE':
    b=[r for r in B['montecarlo'] if r['run']==n]; a=[r for r in A['montecarlo'] if r['run']==n]
    p=lambda rs: np.percentile([r['peak_gov'] for r in rs],95)
    print('| %s | %.1f → %.1f | %d → %d | %d → %d | %d |'%(n,p(b),p(a),sum(r['events']>0 for r in b),sum(r['events']>0 for r in a),
          sum(r['wd_trips'] for r in b),sum(r['wd_trips'] for r in a),sum(r['suspended'] for r in a)))
print()
if 'info' in A:
    print('## Information modes (post, seeds 1-5 medians): governed RMS / peak °, yaw scale')
    print('| Run | estimate, yaw follows plan | estimate, yaw lags 10 ms and +10 % | plan, follows | plan, lags 10 ms and +10 % |')
    print('|---|---|---|---|---|')
    for n in 'BC':
        row=[]
        for mode,mm in (('estimate',False),('estimate',True),('plan',False),('plan',True)):
            x=[r for r in A['info'] if r['run']==n and r['mode']==mode and r['mismatch']==mm]
            row.append('%.2f / %.2f, %.2f, ev %g'%(med([r['rms_gov'] for r in x]),med([r['peak_gov'] for r in x]),med([r['yaw_scale'] for r in x]),med([r['events'] for r in x])))
        print('| %s | %s |'%(n,' | '.join(row)))
print()
print('## Saturation step (30° in 0.1 s, governor off, seed 1)')
for b,a in zip(B['sat'],A['sat']):
    print('- %s: settle %.2f → %.2f s, overshoot %.1f → %.1f °, max |integ| %.3f → %.3f N·m'%(b['case'],b['settle'],a['settle'],b['overshoot_deg'],a['overshoot_deg'],b['integ_max'],a['integ_max']))
print()
print('## No yaw authority (0.4 N·m predicted coupling, no planner)')
pre = B['noauth'] if isinstance(B['noauth'], list) else [B['noauth']]
for b in pre:
    print('- pre:', {k: (round(v, 1) if isinstance(v, float) else v) for k, v in b.items()})
for a in A['noauth']:
    print('- post:', {k: (round(v, 1) if isinstance(v, float) else v) for k, v in a.items()})
