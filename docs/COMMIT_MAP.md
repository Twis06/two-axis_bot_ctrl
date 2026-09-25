# Commit ID map (history rewrites, 2026-09-25)

The history was rewritten twice before and just after the first push to GitHub. File contents, authorship (Twis06), dates and messages are otherwise unchanged; only commit IDs changed.

1. **Removed the author's personal notes file** (`note.md`) from every commit: `git filter-repo --invert-paths --path note.md`.
2. **Replaced machine-specific absolute paths** in recorded provenance (run manifests, reproduction logs) with neutral placeholders: `<repo>`, `<home>`, `<server-workdir>`, `<lab-account>`, `<user>`. This used `git filter-repo --replace-text`. No source file, registration or results file contained such a path, so code hashes, run_ids and registration hashes are unaffected.

- **Markdown references:** commit IDs cited in the current Markdown documents use the current IDs.
- **Recorded provenance:** IDs recorded inside generated evidence (run manifests, reproduction logs) and in historical file versions may be original or intermediate IDs. This table resolves them.
- **Known consequence:** hash manifests of *historical* versions of rewritten files (for example the HTML report manifest and the pre-R4 archive's `SHA256SUMS`) no longer match those rewritten historical files. The current tree's manifests are regenerated.

| Original ID | After note.md removal | Current ID | Commit |
|---|---|---|---|
| `—` | `29d1642d6259` | `fa4c33a759d5` | Add project notes, plan and dependencies |
| `—` | `6df6636fffae` | `ab7f0330b6b7` | Phase 0: closed-form analysis of Runs A-E |
| `—` | `b272ce018d2a` | `ada943b8be36` | Phase 1: roll-axis simulator with delay, quantization and drive model |
| `—` | `d002b3cef3ea` | `b21d6ff5be73` | Phase 1: reproduce Runs A-E with the reconstructed legacy controller |
| `—` | `8035f1344843` | `41af81502faf` | Phase 2: baseline controller, reference governor and drive supervisor |
| `—` | `9d16f9301bdb` | `c3d2e8eb49d4` | Phase 2: evaluate baseline against legacy |
| `—` | `78689a53b09c` | `a10df3295e72` | Add task-organised answers, README and one-command runner |
| `—` | `694ec7fa262f` | `5e1f53cfb7e6` | Checkpoint before EXECUTION_PLAN Packet 2A |
| `—` | `633c05272c16` | `974c1cde63f8` | Task 1: label every causal claim as Obs/Calc/Hyp/Sim |
| `—` | `a03bb1bfcbcd` | `d7c88bc29884` | Packet 2A: governor reference contract (independent review PASS) |
| `—` | `7dc37533de6a` | `86cb22c6dabd` | Packet 4A: metrics, finite motions and run provenance (review pending) |
| `—` | `95d5653c8ceb` | `b7665aa83e1d` | Packet 2B: recovery by fault class, causal yaw information (review PASS WITH ISS |
| `—` | `e6558f94a5e3` | `b9832925d747` | Packet 4A review round 1: suspension, net progress, governed flag, closed-set ru |
| `—` | `78852133ab49` | `3cb25715865f` | Packet 2C: freeze baseline code; evidence scripts publish run manifests |
| `—` | `963448f734d1` | `30b9e193b0bb` | Packet 4A review round 2: completion robust to post-arrival transients, rebuild  |
| `—` | `dbbb75aaa7dc` | `ee0aedafb969` | Packet 2C: baseline fingerprint excludes scoring code; data-driven readings |
| `—` | `adbda45c06f1` | `465f06d5dd3a` | Packet 2C: frozen baseline evidence and task2.md rewrite (review pending) |
| `—` | `a9e12abd6198` | `9851222579c3` | Packet 2C review I-A: windowed coupling bound for the tracking-fault catch |
| `—` | `7e0fd8fc5520` | `854a06858e57` | Packet 2C: re-freeze after review I-A, qualified provenance, review round record |
| `—` | `2132b677f263` | `e507eb6b35bc` | Packet 4B: matched baseline validation (review pending) |
| `—` | `a8d6a956cd1d` | `a31509631d86` | Packet 4B review round 1: controller contracts from logs, corrected readings |
| `—` | `a4bda55417a9` | `7ed3cfc5c12f` | Task 3 decision input: known-load ceiling under the pre-registered L4 protocol |
| `—` | `4a753ed8f12d` | `69b40e0d01f9` | Task 3 L1/L2: bounded payload estimator kernel and standalone audit |
| `—` | `b56626825822` | `02cc32e25e58` | Task 3 L3/L4: feed-forward-only integration and pre-registered comparison |
| `—` | `0a61fff590ea` | `9a2e8c9a14d6` | Packet 4B review round 2: contract-check limits, damping diagnostic, text fixes |
| `—` | `48633c0810c2` | `74d3a3c5b29c` | Task 5: registered motor-strength prediction and test; Task 4 text update |
| `—` | `c57a475fcf0c` | `080b50d9f6c1` | Assessment report: self-contained HTML build of Tasks 1-5 |
| `—` | `24c57d645f95` | `c04cd87b8058` | Execution plan: high-level review findings as repair packets R1-R6 |
| `—` | `c78d8eda71cf` | `9b8b15903784` | Task 3 decision-correction plan (2026-09-24) |
| `—` | `3c11ce1b7a90` | `c4aaed7f66d3` | R1: phase-correct Task 3 completion scoring (review pending) |
| `—` | `84047f06f42d` | `c2184e1aadcb` | R2: Task 5 windows, units and prediction-to-measurement mapping (review pending) |
| `—` | `2598230671dd` | `04b9403eab80` | R3: register supplementary adaptive challenge matrix before execution |
| `—` | `3b4857e10502` | `5cf2237972e5` | R1 review round 1: pin the phase scorer, disclose semantics, rebuildable runs |
| `—` | `b7d225ef28ef` | `315c64d54307` | R2 review round 1: label post-hoc mappings, separate gain from peak, audit regis |
| `—` | `c3e0ff874acd` | `da8838ecf6b7` | R2: status line reflects the completed review round |
| `—` | `5c4880b846ee` | `4e8530f2b6df` | R3: complete machine-audited adoption gate and supplementary challenges (review  |
| `—` | `0f19150b56e4` | `6c3cc6984e68` | R4 prep: archive pre-R4 evidence; full reproduction runner |
| `—` | `7ff822d13cbb` | `b6d9f26ec4d2` | R3 review round 1: gate cannot pass vacuously; meaningful limit check; late-onse |
| `—` | `039f83b4be7d` | `c74aabd6474c` | R4: republish all current evidence from committed sources (b6d9f26) |
| `—` | `9c267c1d2287` | `972e81e1fc85` | R3 packet: review round 1 resolutions and final published numbers |
| `—` | `4cc28e37f164` | `5cc75a3eccc8` | R5: reconcile task answers, READMEs and HTML source with R1-R4 evidence |
| `—` | `dfe451053ce8` | `cf5a7899873a` | R4: report headers work outside a git checkout |
| `—` | `bc5dae330d0e` | `e568c8d8ec94` | R4: clean-snapshot reproduction verified; evidence republished from cf5a789 |
| `—` | `18ac6b2aeabb` | `6ab1f99387c2` | Submission documents: four-page memo, one-page qualification plan, references/to |
| `—` | `06debec26b4d` | `69f3c239df56` | R4: Task 5 results record the current task5_prediction.py hash (republished from |
| `—` | `bd9d35e83c2b` | `5cd4e2e66aa1` | R6 review round 1 (FAIL, 3 Important): fixes |
| `—` | `72c6d56638f9` | `7c884e201c4d` | R6: republish all evidence from 5cd4e2e |
| `—` | `727c7f9819d3` | `ace4a87987e9` | Reproducibility: locked uv environment; parallel Task 2 evaluation |
| `—` | `25559e3a79d0` | `7909d2f527c2` | Republish under the locked uv environment; cross-platform test fix; HTML builds  |
| `—` | `961b6f24e2a3` | `e8c65dbc2644` | R4: cross-platform reproduction (macOS arm64 exact; Linux x86-64 within 3e-11) |
| `—` | `bd8df97a5300` | `b359eb4446fc` | R6 round 2: PASS WITH ISSUES on e8c65db (0 Critical, 0 Important, 9 Minor record |
| `—` | `9642a6cad49f` | `55687c3b2064` | Refine final assessment documents and qualification plan |
| `—` | `0073726b856b` | `2228c56f2035` | Make HTML manifest input order deterministic |
| `—` | `83fb60c1b95f` | `4639be9d7140` | Record clean archive verification and remaining review scope |
| `—` | `da07793a8f13` | `b36f8f4ba965` | Fix static holdability figure layout and embed vector version in memo |
| `—` | `89d80d78a48e` | `0035088c7dfe` | Keep vector figure text searchable and SVG clean |
| `—` | `bf8db462fb35` | `de50a64a2535` | Make vector holdability exports reproducible |
| `—` | `ced6c5bd2092` | `faa03c5a2f0f` | Final pass: gate edge cases, comparison tool, memo figure in run_all, replay cla |
| `—` | `20a0a5ddfc6e` | `60604a0f4852` | Republish from faa03c5 (11 steps, 264 tests) |
| `—` | `dde280b34ad5` | `674f8170b295` | Test counts 264 at the final commit; HTML and memo PDF rebuilt (memo body 4 pp) |
| `—` | `f3e7c4c38016` | `215b2677c967` | Remove personal notes (note.md) from the submission; kept locally and gitignored |
| `—` | `34c6c02131ac` | `eade6beaff18` | R4: final-commit reproductions (Mac fresh install exact; luna1 within 3.1e-11) |
| `—` | `cdbab1cdb2dd` | `9c7a1c80a810` | R6 round 3 (FAIL, 3 Important): fixes |
| `—` | `4bf2f6bc51e8` | `030d26b9e771` | Republish from 9c7a1c8 (11 steps, 266 tests); R6 round 3 recorded |
| `—` | `4805252cfa8d` | `8b13dc881ae4` | Status lines point to R6 for the final verdict; 266 tests; HTML and memo PDF reb |
| `—` | `99fe1f55d8c7` | `6ef3a4435297` | R6 round 4: PASS WITH ISSUES (release gate met); final reproductions of 8b13dc8 |
| `—` | `1838220caf17` | `8375d135a148` | Execution plan: R5 and R6 complete; release gate met |
| `—` | `2253b8fb5049` | `6d529bd8696e` | Execution plan: tick completed items; closed-loop prediction kept open as a decl |
| `—` | `b22cd13ca2d5` | `55dbb23d1e32` | Refresh report hashes after final execution plan updates |
| `—` | `23b1bbd74e81` | `13b281daf959` | Plan one-hour prospective Task 5 completion and submission checks |
| `—` | `43889c3b1418` | `412df89fe6bf` | Update commit references after removing note.md from history; add docs/COMMIT_MA |
| `—` | `1f4d98297283` | `c0b065f0d706` | Task 5 prospective: register numerical prediction BEFORE any run (no results) |
| `—` | `35b9f96dd570` | `7452ea80613a` | Task 5 prospective: registration v2 BEFORE any run (pre-run review fixes) |
| `—` | `dc85377548ad` | `62a9ac678ba0` | Task 5 prospective: declare exp/evidence.py in PROVENANCE_SOURCES |
| `—` | `90350894d6a2` | `11012402b4b9` | Task 5 prospective: results of the registered runs (outcome SUPPORTED) |
| `—` | `29da983b3303` | `82ceda009037` | Task 5: integrate the prospective test (overview, memo, HTML, status, run_all, c |
| `—` | `b8398300523b` | `bd99611b21fa` | Republish all evidence from 82ceda0 (13 steps, 278 tests): 0 of 46,489 floats di |
| `—` | `23395d9fa002` | `a377d89617d9` | Rebuild HTML (20 figures) and PDFs after the Task 5 integration; memo body 4 pag |
| `—` | `be35460fbb32` | `4f1504f5c802` | Task 5 final review (PASS WITH ISSUES): resolve its 13 minors; review packet; re |
| `—` | `f4a12c06b71c` | `f9daecd961e5` | Tidy the repository and lead the README with the deliverables |
