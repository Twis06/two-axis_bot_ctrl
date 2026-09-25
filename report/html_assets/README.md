# HTML assessment report

Open `../assessment_report.html` directly in a browser. Figures, styling, JavaScript and the learning / prediction chart data are embedded. Companion source links require the repository to remain beside the file.

The report contains five task answers, 19 PNG figures, expandable evidence sections, interactive held-out learning filters, motor-test metric selection, figure enlargement, a hardware qualification proposal and a source index. It is a comprehensive report, not a page-verified four-page memo.

Build from the repository root:

```sh
uv run --with markdown python report/html_assets/build_report.py
```

This assembles stored evidence and regenerates one calculated static-holdability diagram from the stated model. It does not execute control experiments or modify their results. `report.md` owns the synthesis, `report.css` the layout, `report.js` the interactive charts, and `holdability_figure.py` the calculated diagram. `report_manifest.json` records source and output SHA-256 hashes.

The Print button prints the current expanded/collapsed evidence state. Use Expand all evidence first to include source appendices. Historical figures remain explicitly labeled. A full printout exceeds the assessment's separate four-page memo limit.

The current 19-image build was checked in headless Chrome at true device-emulated widths 320, 390, 768 and 1440 px: the document fits each viewport, while wide evidence tables scroll within their containers. The learning filter, prediction metric selector, figure enlargement and evidence expansion respond. All 19 embedded PNGs decode, 106 links resolve to existing local paths or internal anchors, and every manifest input hash matches. Control experiments are not rerun by this documentation builder.
