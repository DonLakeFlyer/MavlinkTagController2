# Rate Switch Aware Detector Design

## Problem

A VHF collar transmits pulses at one of two known rates (e.g. 1.5s resting, 2.0s moving). It can switch between them. The detector folds K consecutive pulses to boost SNR. If a rate switch happens mid-segment, a single-rate fold misaligns some pulses and loses sensitivity.

## Key Constraints

1. **Two rates are known exactly.** They come from the collar spec and are configured at startup.
2. **Crystal oscillator timing.** Pulse timing has ppm-level drift — negligible even at K=20. A fold template built from the configured rates will land exactly on each pulse.
3. **Clean transitions.** Every inter-pulse gap is exactly rate A or rate B — there are no intermediate or anomalous gaps during a switch. The collar transitions directly from one rate to the other.
4. **At most one switch per segment.** The collar changes rate infrequently. Within K pulses, there is at most one transition point.
5. **The segment must be long enough for K folds at the slowest rate.** This ensures all hypotheses fit within the available data.

## Hypothesis Enumeration

Given these constraints, the set of possible pulse schedules is small and exact.

Each schedule is a sequence of K−1 gaps. Each gap is either rate A or rate B. With at most one switch point, the possibilities are:

- **Pure A**: all gaps are A → `[A, A, A, A]`
- **Pure B**: all gaps are B → `[B, B, B, B]`
- **A→B at change-point c**: first c gaps are A, rest are B (c = 1..K−2)
- **B→A at change-point c**: first c gaps are B, rest are A (c = 1..K−2)

For K=5 this gives exactly **8 hypotheses**. For K=10 it gives **18**. One of them is guaranteed to match the actual pulse pattern exactly.

## Detection

For each hypothesis, the detector:
1. Computes fold window indices from the gap schedule (cumulative sum of N_A and N_B spacings, with fractional PRI rounded per-offset).
2. Slides the template across all valid start positions within the segment.
3. Sums STFT power at the K window positions → fold score.

The best score across all hypotheses and start positions is compared against an EVT-derived threshold (calibrated over the full hypothesis bank to control false alarm rate).

## Fold Quality Diagnostic

After thresholding, a max-fold-fraction diagnostic measures whether pulse power is concentrated in a single fold (transient/spike). `max_fold_fraction = max(fold_powers) / sum(fold_powers)`. Values > 0.8 indicate a single transient dominates the score. This is used to downgrade confidence, not to discard detections (matching uavrt_detection's `selectpeakindex` heuristic).

For a correctly matched hypothesis, all K folds land on real pulses, so power is spread across folds (low fraction). For a transient, one fold dominates (high fraction).

## Configuration

- `--tip-secondary <seconds>`: enables multi-hypothesis mode with the second rate.
- If `--tip-secondary` is absent, behavior matches the single-rate detector exactly.

## Pulse Reporting Pipeline (Python Detector → Tag Tracker GCS)

### Architecture difference from C++ detector

The C++ detector (`DETECTION_MODE_UAVRT`) launches **two** processes per dual-rate tag — one for each PRI — with tag IDs `id` (primary) and `id + 1` (secondary). Tag Tracker distinguishes rates by tag ID (even = secondary).

The Python detector (`DETECTION_MODE_PYTHON`) launches a **single** process per tag. Both rates and all switch hypotheses are evaluated in one fold pass. Rate-switch information is encoded in the `group_ind` field of each pulse report.

### Detector → Controller (UDP)

`send_pulse_udp()` packs 12 little-endian IEEE-754 doubles (96 bytes) to `127.0.0.1:50000`:

| Index | Field | Type | Description |
|-------|-------|------|-------------|
| 0 | `tag_id` | double→uint32 | Tag ID from `TagInfo_t.id` |
| 1 | `frequency_hz` | double→uint32 | Detection frequency; 0 = heartbeat |
| 2 | `start_time_seconds` | double | Segment collection timestamp (wall clock) |
| 3 | `predict_next_start_seconds` | double | Expected next pulse time (uses last-rate PRI) |
| 4 | `snr` | double | Per-pulse SNR in dB |
| 5 | `stft_score` | double | Score / threshold ratio |
| 6 | `group_seq_counter` | double→uint16 | Cycle counter (same for all pulses in a group) |
| 7 | `group_ind` | double→uint16 | Rate-switch hypothesis encoding (see below) |
| 8 | `group_snr` | double | Incoherently summed K-group SNR |
| 9 | `detection_status` | double→uint8 | 0=sub, 1=super, 2=confirmed, 3=no-detection |
| 10 | `confirmed_status` | double→uint8 | 1=confirmed, 0=unconfirmed |
| 11 | `noise_psd` | double | Estimated noise PSD at pulse frequency |

### `predict_next_start_seconds` rate selection

The predicted next-pulse time uses the PRI from the **last gap** of the winning hypothesis:
- If the hypothesis ends on rate B (e.g. `"B"`, `"A_to_B_c2"`), prediction uses `tip_secondary`.
- Otherwise (e.g. `"A"`, `"B_to_A_c2"`), prediction uses `tip`.

Tag Tracker can compare successive `predict_next_start_seconds` values against both known TIPs to infer which rate is currently active.

### Controller → GCS (MAVLink tunnel)

`PulseHandler::handlePulse()` copies all UDP fields into `PulseInfo_t` and attaches vehicle telemetry (lat/lon/alt/roll/pitch/yaw) from the telemetry cache based on `start_time_seconds`. The struct is sent as raw bytes inside a `TUNNEL` MAVLink message.

Key `PulseInfo_t` fields for rate-switch (defined in `TunnelProtocol.h`):

| Field | Type | Rate-switch usage |
|-------|------|-------------------|
| `tag_id` | uint32_t | Same tag ID for both rates (no +1 split) |
| `group_ind` | uint16_t | Hypothesis encoding (see below) |
| `predict_next_start_seconds` | double | Uses winning hypothesis's last-rate PRI |
| `detection_status` | uint8_t | 3 = no detection (one per cycle, not two) |
| `stft_score` | double | Score/threshold ratio; for no-detection carries best sub-threshold ratio |

### `group_ind` hypothesis encoding

The `group_ind` field encodes the winning rate-switch hypothesis for the K-group:

| `group_ind` value | Meaning | Hypothesis label |
|-------------------|---------|------------------|
| 0 | Pure rate A (primary/resting TIP) | `"A"` |
| 1 | Pure rate B (secondary/moving TIP) | `"B"` |
| 2 .. K−1 | A→B switch at change-point c (c = group_ind − 1) | `"A_to_B_c{c}"` |
| K .. 2K−3 | B→A switch at change-point c (c = group_ind − K + 1) | `"B_to_A_c{c}"` |

For K=5, the values are:

| `group_ind` | Hypothesis |
|-------------|------------|
| 0 | Pure A |
| 1 | Pure B |
| 2 | A→B switch at pulse 1 |
| 3 | A→B switch at pulse 2 |
| 4 | A→B switch at pulse 3 |
| 5 | B→A switch at pulse 1 |
| 6 | B→A switch at pulse 2 |
| 7 | B→A switch at pulse 3 |

For single-rate tags (`intra_pulse2_msecs == 0`), `group_ind` is always 0.

### No-detection behavior

When the detector searches a cycle and finds no pulse above threshold, it sends a single report with `detection_status = 3`. Key field values for no-detection:

- `snr = 0`
- `predict_next_start_seconds = 0`
- `group_ind = 0`
- `stft_score` = best sub-threshold score ratio (useful for diagnostics)
- `noise_psd` = observed noise floor

Because the Python detector is a single process, Tag Tracker receives **one** no-detection message per cycle per tag — unlike the C++ detector which sends two (one per tag ID).

### Tag Tracker GCS implementation requirements

To support Python rate-switch detection, Tag Tracker must:

1. **Decode `group_ind`** using the encoding table above when `detection_mode == DETECTION_MODE_PYTHON` (1). The tag's K value (from `TagInfo_t.k`) is needed to interpret the B→A range.
2. **Display activity state**: Map `group_ind` to a user-visible label (resting/moving/switching). Change-point hypotheses indicate the tag is transitioning.
3. **Use `predict_next_start_seconds` for timing**: This already reflects the detected rate. No need for Tag Tracker to independently compute next-pulse timing.
4. **Handle single no-detection per tag**: Only one `detection_status == 3` message per cycle (not two).
5. **No tag ID splitting**: Both rates report under the same `tag_id`. Do not look for an even-numbered companion tag when `detection_mode == DETECTION_MODE_PYTHON`.

### Simulator support for rate-switch testing

The IQ simulator (`simulator/iq_simulator.py`) supports rate switching:

- **Preset**: `--preset rate-switch` (tip=2.0s → tip_secondary=1.333s, switch at 34s).
- **CLI args**: `--tip-secondary <seconds> --switch-time <seconds>` per tag.
- **Controller plumbing**: When `intra_pulse2_msecs != 0`, `CommandHandler::_simulatorCommand()` passes `--tip-secondary` and computes `--switch-time` including simulator warmup: `switch_time = warmupSeconds + (K + K/2) * tip` (integer division for `K/2`). This places the rate change halfway through the first post-warmup K-grouping.

## Integration Status

Complete:
1. Hypothesis generation (`build_hypothesis_indices`) and multi-hypothesis fold (`fold_multi_hypothesis`) in `pulse_detector.py`.
2. EVT threshold calibrated over full hypothesis bank.
3. Max-fold-fraction diagnostic computed per detection for confidence downgrade.
4. Detection logs include hypothesis label (e.g. `hyp=A_to_B_c2`).
5. UDP reporting includes `group_ind` derived from hypothesis label.
6. Controller launches a single detector process per tag; passes `--tip-secondary` when dual-rate.

## Tests

- Hypothesis index generation for pure and switch schedules.
- Boundary conditions for valid offsets and segment edges.
- Uniformity metric correctness.
- End-to-end fold detection at both rates and during switches.
- EVT false alarm rate validation with full hypothesis bank.
