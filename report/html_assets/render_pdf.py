"""Render a submission markdown file (memo, qualification plan) to a US-Letter PDF.

Usage: python report/html_assets/render_pdf.py report/memo.md report/memo.pdf
Needs Python-Markdown and Google Chrome (headless print). Not part of run_all.py:
it formats documents and produces no evidence. Prints the page count; for the
memo, the page limit applies to the part before the plot appendix (first '---').
"""
import sys, subprocess, pathlib, re, markdown
src = pathlib.Path(sys.argv[1]).resolve(); out = pathlib.Path(sys.argv[2]).resolve()
md = src.read_text()
md = re.sub(r"\$([^$\n]+)\$", r"<i>\1</i>", md)                       # inline math → italic
body = markdown.markdown(md, extensions=["tables", "sane_lists"])
body = body.replace("<hr />", '<div style="page-break-after:always"></div>')
css = """@page{size:Letter;margin:0.7in 0.75in} body{font:10.5pt/1.35 'Helvetica Neue',Arial,sans-serif;color:#111}
h1{font-size:15pt;margin:0 0 6pt} h2{font-size:12pt;margin:10pt 0 3pt;border-bottom:1px solid #999}
p,li{margin:3pt 0} table{border-collapse:collapse;margin:4pt 0;font-size:9pt;width:100%} td,th{border:1px solid #bbb;padding:1.5pt 4pt;vertical-align:top}
th{background:#eee} img{max-width:100%;max-height:7.5in} code{font-size:9pt} a{color:#114}"""
html = f"<!doctype html><html><head><meta charset=utf-8><base href='{src.parent.as_uri()}/'><style>{css}</style></head><body>{body}</body></html>"
tmp = out.with_suffix(".print.html"); tmp.write_text(html)
import os, shutil
chrome = os.environ.get("CHROME") or shutil.which("google-chrome") or shutil.which("chromium") or "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
subprocess.run([chrome, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", f"--print-to-pdf={out}", tmp.as_uri()],
               check=True, capture_output=True, timeout=120)
tmp.unlink()
data = out.read_bytes(); print(out.name, "pages:", len(re.findall(rb"/Type\s*/Page[^s]", data)))
