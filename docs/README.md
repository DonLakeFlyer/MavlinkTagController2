# Documentation index

Documents are filed by the question they answer, not by component.

| Directory | Question | Rules |
| --- | --- | --- |
| [design/](design/) | How does the **current** code work? | Describes shipped behaviour only. Constants are named by symbol, never by line number. Updated when the code changes. |
| [analysis/](analysis/) | What did we **measure**? | Dated (`YYYY-MM_` prefix). Never rewritten after the fact; at most a "superseded by" line is added. |
| [proposals/](proposals/) | What might we **change**? | Every proposal has a `Status:` line and a row in [proposals/README.md](proposals/README.md). When built, the design doc is updated and the proposal moves to `archive/`. |
| [archive/](archive/) | What is **no longer true**? | Banner at the top; otherwise frozen. |

Component READMEs (beside the code) answer only *what is this and how do I run
it* — see the contract below.

## design/

| Doc | Scope |
| --- | --- |
| [SYSTEM_OVERVIEW.md](design/SYSTEM_OVERVIEW.md) | End-to-end data and control flow: SDR → ZMQ → decimator → UDP → detector → controller → GCS; ports, processes, log layout |
| [DETECTOR_PIPELINE.md](design/DETECTOR_PIPELINE.md) | One detection cycle: input stream and gap handling, STFT + W matrix, K-fold, EVT threshold, peak selection, SNR; performance and rationale |
| [COLLECTION_FLOW.md](design/COLLECTION_FLOW.md) | TTDP handshake, acquisition → provisional lock → candidate bank → per-slice re-measurement → PRI refit → `FinishCollection` → revisit |
| [CONFIDENCE_PIPELINE.md](design/CONFIDENCE_PIPELINE.md) | `pf` → EVT threshold → `detection_margin` → `confidence_ratio` → dominant-fold gate; HIGH/LOW classification |
| [RATE_SWITCH_DETECTOR.md](design/RATE_SWITCH_DETECTOR.md) | Dual-rate collars: multi-hypothesis fold bank, `rate_state`, simulator support |
| [FREQUENCY_RANGE.md](design/FREQUENCY_RANGE.md) | DC-spur offset, detection bandwidth, bin resolution |
| [PYTHON_VS_UAVRT.md](design/PYTHON_VS_UAVRT.md) | Stage-by-stage comparison with the MATLAB/C++ `uavrt_detection` |
| [UAVRT_DETECTION_REFERENCE.md](design/UAVRT_DETECTION_REFERENCE.md) | The MATLAB `uavrt_detection` itself (not the Python detector) |

## analysis/

| Doc | What was measured |
| --- | --- |
| [2026-04_FLIGHT_DATA_ANALYSIS.md](analysis/2026-04_FLIGHT_DATA_ANALYSIS.md) | Checks 0–6 against the April 2026 PDC flight logs: SNR floor, offsets, front/back contrast, IQ replay |
| [2026-04_MINI_VS_HF_COMPARISON.md](analysis/2026-04_MINI_VS_HF_COMPARISON.md) | Airspy Mini vs HF+ at 3 km: SNR, noise floor, frequency stability |
| [2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md](analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md) | Why the max-search SNR compresses front/back contrast; the fixed-offset amplitude estimator; the retrospective-lock design that was then implemented |
| [2026-09_MULTIPATH_ANALYSIS_REVIEW.md](analysis/2026-09_MULTIPATH_ANALYSIS_REVIEW.md) | Re-analysis of the Apr-11 failures: noise detections, not multipath; corrected two-ray geometry |
| [2026-09_SIMULATOR_E2E_SESSION.md](analysis/2026-09_SIMULATOR_E2E_SESSION.md) | Simulator end-to-end runs of the 8-heading implementation: results, fixes they prompted, how the rotation logs were read, open items — hand-off note |

## proposals/

See [proposals/README.md](proposals/README.md) for the status table.

| Doc | Topic |
| --- | --- |
| [CONFIDENCE_IMPROVEMENTS.md](proposals/CONFIDENCE_IMPROVEMENTS.md) | On/off contrast, uniformity in the packet, rotation-level frequency consistency |
| [CW_REJECTION.md](proposals/CW_REJECTION.md) | On/off contrast test detail |
| [YAGI_ANTENNA_DESIGN.md](proposals/YAGI_ANTENNA_DESIGN.md) | 3-element Yagi for 146 MHz: dimensions, matching, mounting, LUT requirement |
| [FIELD_TEST_PLAN.md](proposals/FIELD_TEST_PLAN.md) | Capture campaign to close the open checks from the April analysis |

## archive/

| Doc | Why archived |
| --- | --- |
| [MULTIPATH_ANALYSIS.md](archive/MULTIPATH_ANALYSIS.md) | Path-difference model wrong for a ground-level transmitter; superseded by the 2026-09 review |
| [END_TO_END_8x20_SYSTEM.md](archive/END_TO_END_8x20_SYSTEM.md) | 8-heading listening-post design hand-off; implemented 2026-09-12, current behaviour is in `design/` |
| [MIGRATION_FROM_MULTIREPO.md](archive/MIGRATION_FROM_MULTIREPO.md) | Historical note on the repos this monorepo replaced |

---

## README content contract

Target structure for component READMEs: these sections, in this order, and
nothing that belongs in `docs/`. Aim for ≤ 150 lines. `controller/`,
`decimator/`, `detector/`, `shared/` and `airspyhf_zeromq/` follow it; the
others are converged toward it as they are next touched.

| # | Section | Content |
| --- | --- | --- |
| 1 | Title + purpose | What it does and where it sits in the pipeline (one ASCII line). ≤ 10 lines |
| 2 | Build / install | Only what is specific to this component |
| 3 | Usage | CLI table (flag / default / meaning) taken from the argument parser, plus 2–3 invocations |
| 4 | Inputs / outputs | Ports, file layouts, log files — one table |
| 5 | Key files | 5–12 source files with a one-line role |
| 6 | Tests | One command and a link to `tests/README.md` |
| 7 | Further reading | Links into `docs/` |
| 8 | Troubleshooting (optional) | Symptom → cause → fix. No source line numbers |

Not in READMEs: algorithm derivations, protocol semantics, comparisons with
other systems, field results, proposals, history, performance numbers,
authors/licence (root README only).

`*/tests/README.md`: run command, prerequisites / skip conditions, one table of
test files → what each covers. No per-test-function listings.

Design docs — target structure: scope line, mechanism, constants by symbol,
rationale, "Related" footer. The docs written for this layout
(`SYSTEM_OVERVIEW`, `DETECTOR_PIPELINE`, `COLLECTION_FLOW`) follow it; the older
ones converge as they are next touched. Anything not yet built goes to
`proposals/`.
