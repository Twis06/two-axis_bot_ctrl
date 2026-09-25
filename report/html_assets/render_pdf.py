"""Render a submission markdown file (memo, qualification plan) to a US-Letter PDF.

Usage: python report/html_assets/render_pdf.py report/memo.md report/memo.pdf
Needs Python-Markdown and Google Chrome (headless print). Not part of run_all.py:
it formats documents and produces no evidence. Prints the page count; for the
memo, the page limit applies to the part before the plot appendix (first '---').
"""
import html, pathlib, re, subprocess, sys
from urllib.parse import unquote, urlsplit

import markdown

src = pathlib.Path(sys.argv[1]).resolve(); out = pathlib.Path(sys.argv[2]).resolve()
md = src.read_text()
md = re.sub(r"\$([^$\n]+)\$", r"<i>\1</i>", md)                       # inline math → italic
body = markdown.markdown(md, extensions=["tables", "sane_lists"])
body = body.replace("<hr />", '<div style="page-break-after:always"></div>')

# Chrome otherwise turns relative Markdown links into absolute file:/// URLs
# pointing to the author's machine. Keep such source references readable in
# the PDF, and preserve real web links and internal PDF anchors.
repo_root = pathlib.Path(__file__).resolve().parents[2]
def printable_link(match):
    target, label = html.unescape(match.group(1)), match.group(2)
    parts = urlsplit(target)
    if parts.scheme in ("http", "https", "mailto") or target.startswith("#"):
        return match.group(0)
    local = (src.parent / unquote(parts.path)).resolve()
    try:
        shown = local.relative_to(repo_root).as_posix()
    except ValueError:
        shown = local.name
    if parts.fragment:
        shown += "#" + parts.fragment
    return f'{label} <span class="source-path">[{html.escape(shown)}]</span>'

body = re.sub(r'<a\s+href="([^"]+)"[^>]*>(.*?)</a>', printable_link, body, flags=re.S)
css = """@page{size:Letter;margin:0.7in 0.75in} body{font:10.5pt/1.35 'Helvetica Neue',Arial,sans-serif;color:#111}
h1{font-size:15pt;margin:0 0 6pt} h2{font-size:12pt;margin:10pt 0 3pt;border-bottom:1px solid #999}
p,li{margin:3pt 0} table{border-collapse:collapse;margin:4pt 0;font-size:9pt;width:100%} td,th{border:1px solid #bbb;padding:1.5pt 4pt;vertical-align:top}
th{background:#eee} img{max-width:100%;max-height:7.5in} code{font-size:9pt} a{color:#114}
.source-path{font-size:0.85em;color:#444}"""
if src.name == "hardware_qualification_plan.md":
    css += """@page{margin:0.55in 0.6in} body{font-size:10pt;line-height:1.25}
    h1{font-size:13.5pt} p{margin:2pt 0} table{font-size:8.8pt;margin:2pt 0}
    td,th{padding:1pt 3pt} """
if src.name == "memo.md":
    css += """@page{margin:0.62in 0.65in} body{font-size:10pt;line-height:1.27}
    h2{margin:8pt 0 3pt} p,li{margin:2.5pt 0} table{font-size:8.7pt;margin:3pt 0}
    td,th{padding:1.3pt 3pt}
    figure.plot-page{margin:0;break-inside:avoid;page-break-inside:avoid}
    figure.plot-page + figure.plot-page{break-before:page;page-break-before:always}
    figure.plot-page img{display:block;width:auto;max-width:100%;max-height:7.55in;margin:10pt auto 8pt}
    figure.plot-page figcaption{font-size:9.5pt;line-height:1.3;margin:6pt 0}"""
html = f"<!doctype html><html><head><meta charset=utf-8><base href='{src.parent.as_uri()}/'><style>{css}</style></head><body>{body}</body></html>"
tmp = out.with_suffix(".print.html"); tmp.write_text(html)
import os, shutil
chrome = os.environ.get("CHROME") or shutil.which("google-chrome") or shutil.which("chromium") or "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", f"--print-to-pdf={out}", tmp.as_uri()],
               check=True, capture_output=True, timeout=120)
tmp.unlink()
data = out.read_bytes(); print(out.name, "pages:", len(re.findall(rb"/Type\s*/Page[^s]", data)))
