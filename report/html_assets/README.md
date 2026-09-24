# HTML assessment report

Open `../assessment_report.html` directly in a browser. Figures, styling, JavaScript and the learning / prediction chart data are embedded. Companion source links require the repository to remain beside the file.

The report contains five task answers, all 18 existing PNG figures, ten expandable evidence sections, interactive held-out learning filters, motor-test metric selection, figure enlargement, a hardware qualification proposal and a source index. It is a comprehensive report, not a page-verified four-page memo.

Build from the repository root:

```sh
uv run --with markdown python report/html_assets/build_report.py
```

This only assembles stored evidence. It does not execute experiments or modify their results. `report.md` owns the synthesis, `report.css` the layout, and `report.js` the interactive charts. `report_manifest.json` records source and output SHA-256 hashes.

The Print button prints the current expanded/collapsed evidence state. Use Expand all evidence first to include source appendices. Historical figures remain explicitly labeled. A full printout exceeds the assessment's separate four-page memo limit.

Verified in headless Chrome on 24 September 2026: desktop 1440×1000 and mobile 390×844; all 18 embedded images decode; no JavaScript errors; learning load/current filters, prediction metric selector, figure dialog and evidence expansion work; no mobile page overflow; local links and section targets resolve. Control tests were not rerun during this documentation build.
