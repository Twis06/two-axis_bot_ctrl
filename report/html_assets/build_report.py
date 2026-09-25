"""Build the offline report from stored evidence; runs no control experiments.

From repository root: uv run --with markdown python report/html_assets/build_report.py
"""
from pathlib import Path
import math
import base64
import hashlib
import html
import json
import re
import markdown

from holdability_figure import render as render_holdability

HERE = Path(__file__).resolve().parent
REPORT = HERE.parent
ROOT = REPORT.parent
FIGURES = {
    'task1': [('p0_torque_budget.png', 'Historical assumed torque budget', 'D/E load bars use an early quasi-static payload fit of about 1.79 kg; neither mass nor sweep inertia is identified from the supplied summaries. Current Task 1 qualifications take precedence.'),
              ('p0_yaw_envelope.png', 'Calculated yaw envelope', 'Model-based yaw feasibility under the plotted reserve assumptions, not a measured hardware boundary.')],
    'task2': [('static_holdability.png', 'Calculated static holdability', 'Nominal versus explicitly assumed INF-P load across roll angle, with 2.4 A capacity and governor budget. At 0°, INF-P gravity alone needs 0.412 N·m against 0.336 N·m available. Static angle tests are not dynamic reachability; D/E payloads are unknown.'),
              ('task2_delay_robustness.png', 'Delay and parameter robustness', 'Frozen-baseline local analysis and simulated instability cross-checks.'),
              ('task2_saturation.png', 'Saturation and anti-windup', 'Distinguish shaped-request behavior from the diagnostic governor-disabled case.')],
    'task4': [('task2_tracking.png', 'Tracking the admitted and original requests', 'B/C request a 0° roll hold while yaw moves at 1.5/2.2 Hz; the original and governed roll lines overlap. Actual roll moves under yaw coupling. E shows sacrificed original-request timing.'),
              ('task2_feedback_loss.png', 'Feedback loss and recovery', 'Local fallback, host state and subsequent delivered motion must be read together.'),
              ('task4b_saturation.png', 'Saturation duration and entries', 'Packet 4B (reviewed round 2, republished in R4).'),
              ('task4b_frequency.png', 'Frequency response and phase', 'Packet 4B frequency evidence; separate local frequency behavior from nonlinear guarantees.'),
              ('task4b_fault_timeline.png', 'Fault and recovery timeline', 'Packet 4B: fallback and catch do not imply completion of the request.'),
              ('task4b_generalization.png', 'Generalization across chosen stress cases', 'Selected stress ranges are not calibrated probability distributions.'),
              ('task4b_infeasible.png', 'Infeasible requests', 'Actual subsequent motion is reported alongside restriction or suspension.')],
    'task5': [('task5_prospective.png', 'Prospective Task 5 test', 'Registered prediction and acceptance region (committed before the runs) against the five measured paired changes and their mean; right, per-run gain and phase. Simulated.')],
    'history': [('p0_payload_fit.png', 'Historical payload fit', 'Illustrative fit with assumed payload geometry; the A–E summaries do not identify mass or COM direction.'),
                ('p0_bandwidth_vs_delay.png', 'Historical bandwidth versus delay', 'Earlier design study; use the current 4.46 Hz design and margins for the final controller.'),
                ('p1_legacy_runs.png', 'Reconstructed legacy A–E runs', 'A fit to incomplete summaries; B/C/D current limiting is not reproduced.'),
                ('p2_runs_compare.png', 'Historical Phase 2 comparison', 'Earlier controller; superseded as final-performance evidence by Task 2 generated results.'),
                ('p2_bode.png', 'Historical Phase 2 loop response', 'Earlier gains and margins; not the frozen-controller margin claim.'),
                ('p2_fault_derate.png', 'Historical fault and derating response', 'Earlier recovery policy; current fault-class behavior is described in Task 2.'),
                ('p2_montecarlo.png', 'Historical Monte Carlo results', 'Earlier sampled generalization; current A–E uncertainty counts are in Task 4.')],
}
used = set()


def render(text):
    text = re.sub(r'([^\n])\n(- |\d+\. )', r'\1\n\n\2', text)
    # Retain derivations as readable TeX when present in the source appendices.
    text = re.sub(r'\$\$(.*?)\$\$', lambda m: '\n<pre class="equation tex">' + html.escape(m[1].strip()) + '</pre>\n', text, flags=re.S)
    text = re.sub(r'(?<!\$)\$([^$\n]+)\$(?!\$)', lambda m: '<code>' + html.escape(m[1]) + '</code>', text)
    out = markdown.markdown(text, extensions=['tables', 'fenced_code', 'attr_list', 'sane_lists'])
    out = re.sub(r'<table>', '<div class="table-wrap" tabindex="0" role="region" aria-label="Scrollable evidence table"><table>', out)
    return out.replace('</table>', '</table></div>')


def figures(group):
    parts = []
    for name, title, caption in FIGURES[group]:
        used.add('figs/' + name)
        data = base64.b64encode((REPORT / 'figs' / name).read_bytes()).decode()
        parts.append(f'<figure><button class="figure-button" type="button" aria-label="Enlarge: {html.escape(title)}"><img loading="lazy" src="data:image/png;base64,{data}" alt="{html.escape(title + ". " + caption)}"></button><figcaption><strong>{html.escape(title)}.</strong> {html.escape(caption)} <span>Click to enlarge.</span></figcaption></figure>')
    return '\n'.join(parts)


def detail(match):
    name, title = match.group(1), match.group(2)
    used.add(name)
    text = (REPORT / name).read_text()
    # Local paths inside nested packet reports resolve relative to their source.
    prefix = str(Path(name).parent)
    out = render(text)
    if prefix != '.':
        out = re.sub(r'href="(?!https?:|#|/)([^"]+)"', lambda m: 'href="' + prefix + '/' + m[1] + '"', out)
    out = out.replace(str(ROOT) + '/', '../').replace(str(ROOT).replace(' ', '%20') + '/', '../')
    return f'<details class="evidence"><summary>{html.escape(title)}</summary><div class="detail-body"><p class="source">Snapshot of <a href="{name}">{name}</a>. Source wording retained; qualifications in the main report take precedence. TeX derivations appear as source notation.</p>{out}</div></details>'


def interactive(kind):
    if kind == 'learning':
        return '''<div class="explorer"><h3>Explore the held-out cases</h3><p>Select a load and current limit. Each point is one seed; bars are medians of the stored primary error. Missing primary metrics are excluded and counted, never plotted as zero.</p><div class="controls"><label>Load (θs, θc), N·m <select id="load"><option value="all">All five loads</option></select></label><label>Current limit <select id="limit"><option value="all">Both limits</option><option value="3.2">3.2 A</option><option value="2.4">2.4 A</option></select></label></div><div id="learning-chart" class="chart"></div><p id="learning-summary" aria-live="polite"></p><div id="learning-table"></div></div>'''
    return '''<div class="explorer"><h3>Motor-strength test: all paired seeds</h3><label>Display metric <select id="prediction-metric"><option value="governed_rms_deg">Governed RMS error (°)</option><option value="measured_peak_current_A">Measured peak current (A)</option><option value="measured_rms_current_A">Measured RMS current (A)</option></select></label><div id="prediction-chart" class="chart"></div><p>Points show seeds 21–23; bars show group medians. Same Run B request, 2–8 s scoring window. These are total closed-loop quantities in the frozen estimate mode; the plan-mode diagnostic rows are excluded.</p></div>'''


def main():
    render_holdability(REPORT / 'figs' / 'static_holdability.png')
    body = (HERE / 'report.md').read_text()
    body = re.sub(r'<!-- FIGURES:(\w+) -->', lambda m: figures(m[1]), body)
    body = re.sub(r'<!-- DETAIL:([^|]+)\|([^>]+) -->', detail, body)
    body = re.sub(r'<!-- INTERACTIVE:(\w+) -->', lambda m: interactive(m[1]), body)
    sources = sorted(p for p in REPORT.glob('*.json'))
    # The assessment brief is an outside source and is not redistributed (gitignored), so it is
    # neither linked nor hashed: the report must build from a clean checkout.
    sources += [ROOT / 'EXECUTION_PLAN.md', ROOT / 'docs/plans/task3-learning.md',
                ROOT / 'docs/plans/task3-execution-log.md',
                ROOT / 'docs/plans/task3-availability-review.md']
    links = ['<ul class="source-list">']
    for p in sources:
        rel = str(p.relative_to(ROOT))
        url = '../' + rel
        digest = hashlib.sha256(p.read_bytes()).hexdigest()
        links.append(f'<li><a href="{html.escape(url)}">{html.escape(rel)}</a> <code title="SHA-256">{digest[:16]}</code></li>')
    links.append('</ul><p>Links open companion files in this repository. Embedded figures, tables and interactive data work without those companion files or a network connection. Short hashes identify this snapshot; full hashes are in <a href="html_assets/report_manifest.json">the report build manifest</a>.</p>')
    body = body.replace('<!-- SOURCES -->', '\n'.join(links))
    content = render(body)
    # Wrap task sections for predictable navigation and print layout.
    content = re.sub(r'<h2 id="([^"]+)"', r'</section><section aria-labelledby="\1"><h2 id="\1"', content)
    content = '<section class="intro">' + content + '</section>'
    def clean(x):   # NaN/inf (e.g. never-usable first-usable time) -> null; the charts treat null as missing
        if isinstance(x, float) and not math.isfinite(x): return None
        if isinstance(x, dict): return {k: clean(v) for k, v in x.items()}
        if isinstance(x, list): return [clean(v) for v in x]
        return x
    data = { 'learning': json.loads((REPORT/'task3_ceiling_results.json').read_text())['rows'],
             'prediction': json.loads((REPORT/'task5_results.json').read_text())['rows'] }
    data = clean(data)
    nav = [('task1','1 · Failure diagnosis'),('task2','2 · Baseline controller'),('task3','3 · Learning decision'),('task4','4 · Evidence'),('task5','5 · Prediction test'),('hardware','Hardware proposal'),('sources','Sources & status'),('history','Historical figures')]
    page = '<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Coupled-axis control — Tasks 1–5</title><style>' + (HERE/'report.css').read_text() + '</style></head><body>'
    page += '<a class="skip" href="#main">Skip to report</a><aside><a class="brand" href="#main">Controls assessment</a><p>Evidence & engineering decisions</p><nav aria-label="Report sections">' + ''.join(f'<a href="#{k}">{v}</a>' for k,v in nav) + '</nav><div class="actions"><button id="expand" type="button">Expand all evidence</button><button id="print" type="button">Print report</button></div><p class="side-note">Snapshot · 24 Sep 2026<br>Baseline 7d857df507c389c9<br>Simulation-backed proposal</p></aside>'
    page += '<main id="main">' + content + '</main><dialog id="figure-dialog"><form method="dialog"><button>Close figure</button></form><img alt=""><p></p></dialog>'
    page += '<script id="data" type="application/json">' + json.dumps(data, allow_nan=False).replace('</','<\/') + '</script><script>' + (HERE/'report.js').read_text() + '</script></body></html>'
    (REPORT/'assessment_report.html').write_text(page)
    inputs = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    for name in sorted(used):
        p = REPORT/name
        inputs[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    for name in ['report.md','report.css','report.js','build_report.py','holdability_figure.py']:
        p=HERE/name
        inputs[str(p.relative_to(ROOT))] = hashlib.sha256(p.read_bytes()).hexdigest()
    params = ROOT / 'sim' / 'params.py'
    inputs[str(params.relative_to(ROOT))] = hashlib.sha256(params.read_bytes()).hexdigest()
    (HERE/'report_manifest.json').write_text(json.dumps({'inputs_sha256':inputs,'output_sha256':hashlib.sha256(page.encode()).hexdigest(),'figure_count':sum(map(len,FIGURES.values())),'learning_rows':len(data['learning']),'prediction_rows':len(data['prediction']),'new_control_experiments':False},indent=2)+'\n')
    print(f'Built report/assessment_report.html: {len(page.encode()):,} bytes; {sum(map(len,FIGURES.values()))} figures; {len(data["learning"])} learning rows; {len(data["prediction"])} prediction rows.')


if __name__ == '__main__':
    main()
