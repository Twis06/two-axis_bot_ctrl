# Commit ID map (history rewrite, 2026-09-25)

Before the first push to GitHub, the author's personal notes file (`note.md`) was removed from the whole history with `git filter-repo --invert-paths --path note.md`. Nothing else changed: file contents, authorship (Twis06) and messages are identical. Only the commit IDs changed.

- **Markdown references:** commit IDs cited in the current Markdown documents were updated to the new IDs.
- **Recorded provenance:** IDs recorded inside generated evidence (run manifests, reproduction logs) and in historical file versions refer to the old IDs. This table resolves them.

| Old ID | New ID | Commit |
|---|---|---|
| `29d1642d6259` | `fa4c33a759d5` | Add project notes, plan and dependencies |
| `6df6636fffae` | `ab7f0330b6b7` | Phase 0: closed-form analysis of Runs A-E |
| `b272ce018d2a` | `ada943b8be36` | Phase 1: roll-axis simulator with delay, quantization and drive model |
| `d002b3cef3ea` | `b21d6ff5be73` | Phase 1: reproduce Runs A-E with the reconstructed legacy controller |
| `8035f1344843` | `41af81502faf` | Phase 2: baseline controller, reference governor and drive supervisor |
| `9d16f9301bdb` | `c3d2e8eb49d4` | Phase 2: evaluate baseline against legacy |
| `78689a53b09c` | `a10df3295e72` | Add task-organised answers, README and one-command runner |
| `694ec7fa262f` | `5e1f53cfb7e6` | Checkpoint before EXECUTION_PLAN Packet 2A |
| `633c05272c16` | `974c1cde63f8` | Task 1: label every causal claim as Obs/Calc/Hyp/Sim |
| `a03bb1bfcbcd` | `d7c88bc29884` | Packet 2A: governor reference contract (independent review PASS) |
| `7dc37533de6a` | `86cb22c6dabd` | Packet 4A: metrics, finite motions and run provenance (review pending) |
| `95d5653c8ceb` | `b7665aa83e1d` | Packet 2B: recovery by fault class, causal yaw information (review PASS WITH ISSUES, round |
| `e6558f94a5e3` | `b9832925d747` | Packet 4A review round 1: suspension, net progress, governed flag, closed-set run_id |
| `78852133ab49` | `3cb25715865f` | Packet 2C: freeze baseline code; evidence scripts publish run manifests |
| `963448f734d1` | `30b9e193b0bb` | Packet 4A review round 2: completion robust to post-arrival transients, rebuild and publis |
| `dbbb75aaa7dc` | `ee0aedafb969` | Packet 2C: baseline fingerprint excludes scoring code; data-driven readings |
| `adbda45c06f1` | `465f06d5dd3a` | Packet 2C: frozen baseline evidence and task2.md rewrite (review pending) |
| `a9e12abd6198` | `9851222579c3` | Packet 2C review I-A: windowed coupling bound for the tracking-fault catch |
| `7e0fd8fc5520` | `854a06858e57` | Packet 2C: re-freeze after review I-A, qualified provenance, review round recorded |
| `2132b677f263` | `e507eb6b35bc` | Packet 4B: matched baseline validation (review pending) |
| `a8d6a956cd1d` | `a31509631d86` | Packet 4B review round 1: controller contracts from logs, corrected readings |
| `a4bda55417a9` | `7ed3cfc5c12f` | Task 3 decision input: known-load ceiling under the pre-registered L4 protocol |
| `4a753ed8f12d` | `69b40e0d01f9` | Task 3 L1/L2: bounded payload estimator kernel and standalone audit |
| `b56626825822` | `7169be837584` | Task 3 L3/L4: feed-forward-only integration and pre-registered comparison |
| `0a61fff590ea` | `a6baf188b7ef` | Packet 4B review round 2: contract-check limits, damping diagnostic, text fixes |
| `48633c0810c2` | `b9cb80a7d9d7` | Task 5: registered motor-strength prediction and test; Task 4 text update |
| `c57a475fcf0c` | `97122a9cd8e1` | Assessment report: self-contained HTML build of Tasks 1-5 |
| `24c57d645f95` | `38fe9e9bf205` | Execution plan: high-level review findings as repair packets R1-R6 |
| `c78d8eda71cf` | `4dce786b800c` | Task 3 decision-correction plan (2026-09-24) |
| `3c11ce1b7a90` | `55947cc3a1da` | R1: phase-correct Task 3 completion scoring (review pending) |
| `84047f06f42d` | `887b28a44db9` | R2: Task 5 windows, units and prediction-to-measurement mapping (review pending) |
| `2598230671dd` | `6abfc2ca8823` | R3: register supplementary adaptive challenge matrix before execution |
| `3b4857e10502` | `bdc46d1d5313` | R1 review round 1: pin the phase scorer, disclose semantics, rebuildable runs |
| `b7d225ef28ef` | `c9c40a997b76` | R2 review round 1: label post-hoc mappings, separate gain from peak, audit registration |
| `c3e0ff874acd` | `29630bcd555a` | R2: status line reflects the completed review round |
| `5c4880b846ee` | `49c026018c02` | R3: complete machine-audited adoption gate and supplementary challenges (review pending) |
| `0f19150b56e4` | `7d431bad235c` | R4 prep: archive pre-R4 evidence; full reproduction runner |
| `7ff822d13cbb` | `73664279a8aa` | R3 review round 1: gate cannot pass vacuously; meaningful limit check; late-onset amendmen |
| `039f83b4be7d` | `dff8efdeaf1d` | R4: republish all current evidence from committed sources (7366427) |
| `9c267c1d2287` | `ff604ffeb39c` | R3 packet: review round 1 resolutions and final published numbers |
| `4cc28e37f164` | `3867cd7a9e96` | R5: reconcile task answers, READMEs and HTML source with R1-R4 evidence |
| `dfe451053ce8` | `c5129292228e` | R4: report headers work outside a git checkout |
| `bc5dae330d0e` | `0a5eac725138` | R4: clean-snapshot reproduction verified; evidence republished from c512929 |
| `18ac6b2aeabb` | `529d63a62cd9` | Submission documents: four-page memo, one-page qualification plan, references/tools note |
| `06debec26b4d` | `36a032b07102` | R4: Task 5 results record the current task5_prediction.py hash (republished from c512929) |
| `bd9d35e83c2b` | `4d6a7ff2190a` | R6 review round 1 (FAIL, 3 Important): fixes |
| `72c6d56638f9` | `01abda0ed581` | R6: republish all evidence from 4d6a7ff |
| `727c7f9819d3` | `b347d483ad98` | Reproducibility: locked uv environment; parallel Task 2 evaluation |
| `25559e3a79d0` | `6db10a36bfd5` | Republish under the locked uv environment; cross-platform test fix; HTML builds without th |
| `961b6f24e2a3` | `156823d4ea15` | R4: cross-platform reproduction (macOS arm64 exact; Linux x86-64 within 3e-11) |
| `bd8df97a5300` | `064550810213` | R6 round 2: PASS WITH ISSUES on 156823d (0 Critical, 0 Important, 9 Minor recorded) |
| `9642a6cad49f` | `623e94043946` | Refine final assessment documents and qualification plan |
| `0073726b856b` | `13fb3f17abfd` | Make HTML manifest input order deterministic |
| `83fb60c1b95f` | `ca83116e113a` | Record clean archive verification and remaining review scope |
| `da07793a8f13` | `81fb57e3b293` | Fix static holdability figure layout and embed vector version in memo |
| `89d80d78a48e` | `c908ffdd1b41` | Keep vector figure text searchable and SVG clean |
| `bf8db462fb35` | `7de1f07e3bbc` | Make vector holdability exports reproducible |
| `ced6c5bd2092` | `33786b9c3fbe` | Final pass: gate edge cases, comparison tool, memo figure in run_all, replay claim scoped |
| `20a0a5ddfc6e` | `55804ecff508` | Republish from 33786b9 (11 steps, 264 tests) |
| `dde280b34ad5` | `f107c0f53908` | Test counts 264 at the final commit; HTML and memo PDF rebuilt (memo body 4 pp) |
| `f3e7c4c38016` | `b7af6b8276ba` | Remove personal notes (note.md) from the submission; kept locally and gitignored |
| `34c6c02131ac` | `e34a2a8c31fc` | R4: final-commit reproductions (Mac fresh install exact; luna1 within 3.1e-11) |
| `cdbab1cdb2dd` | `8c26759c5b82` | R6 round 3 (FAIL, 3 Important): fixes |
| `4bf2f6bc51e8` | `f45613661b54` | Republish from 8c26759 (11 steps, 266 tests); R6 round 3 recorded |
| `4805252cfa8d` | `b9833ecd1333` | Status lines point to R6 for the final verdict; 266 tests; HTML and memo PDF rebuilt |
| `99fe1f55d8c7` | `5c8c9f1f6dc4` | R6 round 4: PASS WITH ISSUES (release gate met); final reproductions of b9833ec |
| `1838220caf17` | `73770d376d99` | Execution plan: R5 and R6 complete; release gate met |
| `2253b8fb5049` | `0604e60929d8` | Execution plan: tick completed items; closed-loop prediction kept open as a declared limit |
| `b22cd13ca2d5` | `5f487025a47f` | Refresh report hashes after final execution plan updates |
| `23b1bbd74e81` | `dd762df71558` | Plan one-hour prospective Task 5 completion and submission checks |
