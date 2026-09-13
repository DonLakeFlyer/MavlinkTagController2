# Simulator end-to-end session (2026-09-12) — hand-off

Manual end-to-end runs of the 8-heading listening-post implementation
(controller `--simulator`, TagTracker auto-detection, one tag at 146 MHz,
dual-rate 1.333 / 2.0 s, K = 20, `--sim-pri-ppm 43`), done on a Linux arm64 VM
the same day the code landed. This note records what was run, what the logs
showed, what was fixed as a result, how the logs were read, and what is still
open — so the work can continue on another machine with a fresh session.

Code state described: uncommitted working tree of MavlinkTagController2 and
TagTracker at the end of this session (see "Where the code is" at the end).

## Runs

| Time | Level | Result | Notes |
| --- | --- | --- | --- |
| 19:20 | `strong`, tag at 0° (old default), spread visit order | 0°, r² 0.978, confirmed | `null_ms` 16 s per dwell (60 % of dwell) → fold speed-up; refined `mu` still ~68 000 on most headings → grid-extension bug |
| 19:38 | `strong`, 0°, clockwise sweep | 359.5°, confirmed | Refined `mu` 68 on 2 headings, 174–54 000 on the rest → footprint range bug |
| 19:53 | `strong`, 135° (new default) | 135.0°, confirmed, sector 3 | `mu` ≈ 68 everywhere; bank held 4 entries: band-edge images ±1920 Hz and a 0 Hz duplicate of candidate 0 |
| 20:19 | `strong`, 135° | 135.5°, confirmed | Band edges masked (105 bins); ±993 Hz sidelobe images and 0 Hz duplicate still present → PRI-fit bias found |
| 20:33 | `strong`, 135° | **135.0°, r² 0.978, 8/8 sightings, candidate 0 selected** | PRI fit converges 1.333011 ± 197 → 1.333056 ± 19 ppm (truth 1.333057). Only remaining artefact: sidelobe images at +1092 / −993 / −1721 Hz (#148) |

Not yet run: `marginal`, `below-marginal`, `competing`, `--sim-antenna ra23k`.

## Fixes made from these runs

All in `detector/pulse_detector.py` unless stated; design docs updated alongside.

1. **Fold search speed** — `_compute_fold_scores` now uses K shifted adds over
   a once-pooled array when the hypothesis grid is `t0 + offsets` with
   consecutive `t0` (always true from `build_hypothesis_indices`). One
   38-hypothesis K=20 fold search 200 → 60 ms; 40-permutation null 16 → 3.9 s.
   `null_ms` per dwell ended at ~7.9 s (two passes) on this VM.
2. **Null refinement removed the whole train** — `_extend_pulse_grid` uses the
   fractional mean of the run of like gaps (a single rounded gap drifted half a
   window per pulse at PRI 176.5 windows), and the drop range around each
   pulse is −3…+4 windows (a pulse spans up to 3 windows at 50 % overlap, the
   fold index may sit one window early, the grid rounds by one). Refined `mu`
   now ≈ 68 on every heading with a strong tag present.
3. **Decimator transition band masked** — `USABLE_BAND_FRACTION` = 0.9: bins
   beyond ±1728 Hz are never searched (last FIR stage cutoff is 0.45 × fs).
   Removed the ±1920 Hz image candidates. Searched bins 116 → 105.
4. **PRI fit (`fit_lock_timing`)** — three changes, each found from a run:
   - footprint is the four windows a pulse touches (half, full, full, half);
     the old two-window sum scored a one-step alias equally;
   - the anchor's sub-step offset is fitted jointly over ±1 step (11 points);
     held fixed, the quantised anchor tilted the PRI by ~250 ppm after one
     dwell — this was the actual cause of the 0 Hz duplicate candidate;
   - `pri_ppm_uncertainty` floor = one step / lever arm (≈ 250 ppm after one
     dwell, ≈ 100 after two, ≈ 25 after eight), so `locks_agree`'s phase
     tolerance is honest. Hypotheses with < 50 % of the best-covered window
     count cannot win.
   Offline check on simulator STFT data at six pulse phases: fit error ≤ 18 ppm
   after two dwells, always inside the reported uncertainty.
5. **Simulator default bearing 135°** for every level (`controller/main.cpp`);
   simulator README level table rewritten in current vocabulary.
6. **TagTracker visit order** is one clockwise sweep in 45° steps (was
   0,2,4,6,1,3,5,7).
7. Mermaid state diagram in `COLLECTION_FLOW.md` updated (semicolons in labels
   break the parser).

## How the logs were read

Rotation directory: newest `~/Logs/Logs-Rotation-<UTC>/`. Checked, in order:

1. **`bearing_result.log`** — one row per tag: `bearing_deg` ≈ the simulated
   bearing (135), `r_squared` > 0.9, `n_valid_slices` = 8, `n_sighted_slices`
   = 8 for `strong`, `confirmed` = 1. `best_snr` ≈ 74 dB means the tag itself
   was selected; ~41–47 dB means a sidelobe image was.
2. **`bearing_candidates.log`** — how many candidates, which was `selected`,
   `n_sighted_slices` per candidate, and the `residuals` column
   (`heading:residual:weight`): weights should be ~0.93–1.07 under uniform
   simulated noise; residuals ~1 % of the fitted power.
3. **`MavlinkTagController.log`** — `Collection slice armed … heading:` lines
   give the visit order (expect 0,45,…,315); the `pulse_detector.py` launch
   line shows `--detection-margin 1.0` and no `--threshold-cache-dir`; the
   `iq_simulator.py` line's `--tx-offset-north-m/-east-m` confirms the
   simulated bearing; `Bearing result:` line for the summary.
4. **Per heading `heading-NNN/detector_2.jsonl`**, record types:
   - `cycle_threshold`: `mu` (Gumbel location of the null), `threshold`,
     `refined`, `dropped_windows`, `n_search_bins`, `blanked_fraction`,
     `null_ms`. With a strong tag `refined` must be true and `mu` ≈ 67–69
     (the pure-noise value for K = 20, 105 bins); anything higher means pulse
     energy leaked into the null. `null_ms` well under the dwell length.
   - `lock_candidate`: `locked` / `admitted` / `seen` / `dropped` / `pri_fit`
     / `pri_fit_clipped` with `candidate_id`, `freq_hz`, `score_ratio`,
     `pri_seconds`, `pri_ppm_uncertainty`. Expect candidate 0 `seen` on every
     heading after the lock, `pri_seconds` converging to 1.333057 with
     shrinking uncertainty, no second candidate at 0 Hz, and `dropped` events
     only when the bank is genuinely full.
   - `timing`: `fold_ms` includes the null; compare with `null_ms`.
   - `detection` records at candidate 0's frequency on every cycle.
5. **Sanity numbers**: true simulated PRI = 1.333 × (1 + 43e-6) = 1.333057 s;
   K = 20 at 1.333 s → 26.7 s dwell; 8 dwells + yaws ≈ 8 min wall time.

Analysis was done with short ad-hoc Python over the `.jsonl` (one `json.loads`
per line, filter on `type`). A reusable `analyzer/rotation_check.py` that
prints the above with pass/fail expectations per simulator level was proposed
but not written.

## Open items

- Issue #148: sidelobe images of a strong tag fill the candidate bank.
  Bench/simulator artefact only (a drone-detected collar is beyond omni range
  and cannot be 20 dB above threshold); proposed rule: reject an alternate
  whose PRI **and** phase agree with an existing candidate.
- Runs still to do: `marginal` (expect one sighting → revisit → confirmed or
  unconfirmed; watch `score_ratio` ≈ 3 against `--lock-score-ratio`),
  `below-marginal` (expect no detections, `refined: false`, "nothing heard"),
  `competing` (interferer takes slot 0, tag must win the fit at 135°),
  `--sim-antenna ra23k`.
- Pi timing of the permutation null (`null_ms`) at dual-rate K = 20; on this
  VM 7.9 s per dwell for two passes.
- Apr-11 replay acceptance (88 % → ≈ 5 %) and the `--lock-score-ratio` review.
- Calibration flight to replace the `kRa2a` / `kRa23k` table values.

## Where the code is

Both repositories had uncommitted changes at the end of the session:
MavlinkTagController2 (detector, controller, analyzer, docs) and TagTracker
(`custom/src/RotationInfo.*`, `CustomPulseRoseMapItem.qml`,
`StateMachine/PythonRotateAndCaptureState.*`, `Settings/Custom.SettingsGroup.json`,
`test/TagTrackerPulseDisplayTest.*`). Test status: controller ctest 9/9,
pytest 202 (detector) + analyzer + simulator green, TagTracker
`TagTrackerPulseDisplayTest` 52/52. Design docs describing the shipped
behaviour: `docs/design/DETECTOR_PIPELINE.md` §5–6,
`docs/design/COLLECTION_FLOW.md`; the archived proposal is
`docs/archive/END_TO_END_8x20_SYSTEM.md`.
