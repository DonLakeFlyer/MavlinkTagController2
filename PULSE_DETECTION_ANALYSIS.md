# Pulse Detection Pipeline — Detailed Analysis

This application (**UAV-RT Detection**) detects periodic radio tag pulses from streaming IQ data received over UDP. It is authored by Michael W. Shafer and designed for wildlife radio telemetry from UAVs. The system is written in MATLAB with MATLAB Coder annotations for C++ code generation.

---

## 1. System Configuration & Initialization

The entry point is `uavrt_detection.m`. On startup it:

1. Reads a `config/detectorConfig.txt` file via the `DetectorConfig` class, which holds:
   - **`tp`** — pulse duration (seconds)
   - **`tip`** — inter-pulse time (seconds)
   - **`tipu`** — inter-pulse time uncertainty (seconds)
   - **`tipj`** — inter-pulse jitter (seconds)
   - **`K`** — number of pulses to coherently integrate
   - **`Fs`** — sample rate
   - **`tagFreqMHz`** / **`channelCenterFreqMHz`** — expected tag frequency and channel center
   - **`falseAlarmProb`** — desired false alarm probability
   - **`opMode`** — operational mode

2. Constructs an initial **priori** `pulsestats` object (`pulsestats.m`) from the config, describing what we *expect* the pulses to look like.

3. Sets up async data/time buffers (`dsp.AsyncBuffer`) and a UDP receiver to receive channelized IQ samples.

---

## 2. Data Acquisition & Buffering

Inside the main `while true` loop (`uavrt_detection.m`, ~line 185):

- IQ data packets arrive via UDP with embedded timestamps.
- **Timestamp validation** detects gaps:
  - Small gaps (< `tp/2`): accepted as-is.
  - Medium gaps (`tp/2` to `tip`): zero-filled to maintain time continuity.
  - Large gaps (>= `tip`) or backwards-in-time: **stale data** — flush all buffers and reset to Discovery mode.
- Data is appended to `asyncDataBuff` / `asyncTimeBuff`.
- Processing triggers when enough samples accumulate: `sampsForKPulses + overlapSamples`. The number of samples needed is computed from:

```
n_ws * (K * (N + M) + J + 1) + n_ol
```

where N, M, J are the interpulse interval, uncertainty, and jitter in STFT window units.

---

## 3. Waveform Construction & STFT

Once enough data is buffered (~line 340):

1. A **`waveform`** object (`waveform.m`) is created from the buffered IQ data, sample rate, start time, priori pulse statistics, and a 50% STFT overlap fraction.

2. **Priori-dependent properties** are computed from the pulse stats:
   - `n_p = ceil(t_p * Fs)` — samples per pulse
   - `n_w = n_p` — STFT window length = 1 pulse width
   - `n_ol = floor(0.5 * n_w)` — overlap samples
   - `n_ws = n_w - n_ol` — window step
   - `N = floor(n_ip / n_ws)` — interpulse interval in STFT windows
   - `M = ceil(n_ipu / n_ws)` — uncertainty in STFT windows
   - `J = ceil(n_ipj / n_ws)` — jitter in STFT windows

3. **STFT** is computed via `X.spectro()` → `wfmstft(obj)`, producing a time-frequency matrix **S** where rows are frequency bins and columns are time windows.

4. **Spectral Weighting Matrix** is built by `X.setweightingmatrix(zetas)` using `weightingmatrix.m`:
   - A time-domain pulse template (default rectangular `[1 1]`) is DFT-transformed.
   - Sub-bin frequency shifts (`zetas = [0, 0.5]`) are applied to produce shifted DFT coefficient vectors.
   - These are assembled into a matrix **W** with a companion frequency vector **Wf** that has finer frequency resolution (half-bin steps) than the raw STFT.
   - When multiplied as `W' * S`, this acts as a **matched spectral filter** — it weights each STFT column to maximize the response to the expected pulse spectrum, including energy that falls between DFT bins.

---

## 4. Threshold Generation

The `threshold` class (`threshold.m`) generates detection thresholds via **extreme value theory** (~line 108 of `threshold.m`):

1. A **time correlator matrix** Wq is built (see step 5 below) for the current N, M, J, K parameters.
2. **Synthetic white Gaussian noise** is generated (100 trials by default), each trial having the same length as the real data.
3. For each noise trial, the full detection pipeline (spectral weighting → incoherent summation) is run and the **maximum score across all frequencies and time correlations** is recorded.
4. An **extreme value distribution** is fitted to these max scores using `evfit(-scores)`, producing location (μ) and scale (σ) parameters.
5. The detection threshold at the desired false alarm probability Pf is found by solving:

```
1 - exp(-exp((-x - μ) / σ)) = Pf
```

via `evthresh()` (`evthresh.m`).

6. This threshold (computed at 1W reference power) is then **linearly scaled per-frequency-bin** based on each bin's actual noise power spectral density, producing a **frequency-dependent threshold vector**.
7. Threshold parameters (μ, σ) are **cached to disk** and reloaded if N, M, J, K haven't changed, avoiding expensive regeneration.

---

## 5. Incoherent Summation (Core Detection)

The detection core lives in `findpulse()` (`waveform.m`, ~line 427):

### 5a. Search Space Setup

- **Time search range**: For each of the K expected pulses, a range of STFT time windows is computed where that pulse could appear. In *naive* mode, this is the full first interpulse interval. In *informed* mode, prior pulse detections narrow the search window.
- **Frequency search range**: In *naive* mode, all frequencies. In *informed* mode, restricted to `[fstart, fend]` from priori. Excluded frequency bands (other known tags) are masked out.

### 5b. Time Correlator Matrix Wq

Built by `buildtimecorrelatormatrix.m` → `assembleWq.m`:

- All possible pulse arrival patterns are enumerated given N ± M mean inter-pulse intervals and ± J jitter (`generate_pulse_positions_func.m`).
- A **sparse Toeplitz-like matrix** Wq is assembled where each column represents one possible K-pulse arrival pattern. Non-zero entries at rows r1, r2, ..., rK mean "sum STFT windows r1, r2, ..., rK together."

### 5c. Incoherent Summation

Performed by `incohsumtoeplitz.m`:

```
scores = Fb * |W' * S|^2 * Tb * Wq
```

Where:
- `W' * S` applies the spectral weighting (matched filter) to each time window
- `|·|^2` computes power (incoherent — phase is discarded)
- `Tb` is a time blinder vector (masks irrelevant time windows)
- `Wq` sums K time windows matching each candidate pulse pattern
- `Fb` is a frequency blinder (masks excluded/out-of-band frequencies)

The result is a matrix of **scores** — one per frequency bin per candidate timing pattern. For each frequency, the **maximum** across all timing patterns is selected, giving the best K-pulse alignment score at that frequency.

### 5d. Repetition Rejector

`repetitionrejector.m` builds a **comb filter matrix** to suppress signals with known non-target repetition rates (e.g., 2, 3, 5, 10 Hz). Currently set to `[0]` (identity matrix, i.e., disabled).

---

## 6. Peeling Algorithm (Peak Identification)

After incoherent summation (`waveform.m`, ~line 790):

1. **Sum scores** across the K pulses for each frequency bin.
2. Identify **spectral peaks** (local maxima in the score vs. frequency vector).
3. Starting from the **strongest peak**:
   - Identify its **sidebands** — nearby frequencies whose STFT time windows are correlated (within 2 pulse-widths in time) with the peak's time windows.
   - Also look at threshold crossings and slope valleys to define sideband extent.
   - Mask the peak and its sidebands from further consideration.
4. Repeat for the next strongest peak until no more scores exceed the threshold.
5. Output: a list of **peak frequency indices** (`pk_ind`), a **candidate pulse matrix** (`candidatelist`) with amplitude, SNR, timestamps, and a **sideband mask** (`indiv_msk`).

SNR is estimated per frequency bin as:

```
SNR_dB = 10 * log10(PSD_signal / PSD_noise)
```

where noise PSD is computed from the STFT while masking out time-frequency regions near candidate pulses.

---

## 7. Operating Modes (Detection State Machine)

The detector operates as a state machine with **two layers**: an **outer loop** in `uavrt_detection.m` that maps high-level suggestions into concrete modes, and an **inner `process()` method** in `waveform.m` that executes the detection logic for each mode.

### 7a. Configuration-Level Operating Mode (`opMode`)

The `DetectorConfig` class defines a top-level `opMode` string (default: `'freqSearchHardLock'`). The supported values are:

| `opMode` Value | Description |
|---|---|
| `freqSearchHardLock` | Start with a full-bandwidth frequency search. Once confirmed, lock the frequency and never release until reset. (Default) |
| `freqSearchSoftLock` | Start with a full-bandwidth frequency search. Lock frequency on confirmation, but release the lock if tracking fails. |
| `freqKnownHardLock` | The tag frequency is known a priori. Start with an informed (narrowed) frequency search and hard-lock on confirmation. |
| `freqAllNeverLock` | Always search the full frequency bandwidth. Never lock or narrow the frequency search, regardless of detections. |

> **Note:** In the current codebase the `opMode` switch is commented out (~line 323 of `uavrt_detection.m`). The runtime behavior effectively follows a `freqSearchSoftLock` pattern — frequency is locked on confirmation and released if tracking fails.

### 7b. Internal Detection Modes

The `process()` method in `waveform.m` (~line 1370) implements **five internal mode codes**:

| Code | Name | Freq Search | Time Search | Run Mode | Description |
|------|------|-------------|-------------|----------|-------------|
| **D** | Discovery | Naive (all freqs) | Naive (all times) | `search` | Initial detection with no prior. Searches the entire frequency band and all time windows. Computationally the most expensive mode. |
| **I** | Informed Discovery | Informed (narrowed freq) | Naive (all times) | `search` | Like Discovery, but restricts the frequency search to the band around a previously detected frequency (`fstart` to `fend`). Used when frequency is locked (`fLock = true`) but the detector needs to re-search in time. |
| **C** | Confirmation | Naive (all freqs) | Naive (all times) | `confirm` | Re-detects with the same broad search as Discovery, then validates the found pulses against the prior detection using `confirmpulses()`. This is a "trust but verify" step. |
| **T** | Tracking | Informed (narrowed freq) | Informed (narrowed time) | `track` | Both frequency and time searches are narrowed based on prior detections. The fastest mode — only looks where pulses are expected to be. |
| **S** | Search (pseudo-mode) | — | — | — | Not a real processing mode. It's a **suggestion code** from the inner `process()` to the outer loop meaning "go back to searching." The outer loop translates it to D or I depending on `fLock`. |

### 7c. Mode Entry Requirements

Before executing, the `process()` method **validates** that sufficient priori information exists for the requested mode. If not, it silently falls back to Discovery:

- **Tracking (T)** requires both `have_priori_freq_band` (finite `fstart`/`fend`) AND `have_priori_time` (finite last pulse time, `t_p`, `t_ip`, `t_ipu`, `t_ipj`). If either is missing → falls back to **D**.
- **Confirmation (C)** requires `have_priori_time` (needs a previous pulse time to confirm against). If missing → falls back to **D**.
- **Informed Discovery (I)** requires `have_priori_freq_band`. If missing → falls back to **D**.
- **Discovery (D)** has no entry requirements — it's the universal fallback.

### 7d. Detailed Mode Behavior

#### Discovery Mode (D)

1. Calls `findpulse('naive', 'naive', excluded_freq_bands)` — searches all frequencies and all time windows.
2. If a detection is made (score exceeds threshold):
   - Also runs `confirmpulses()` to check against any prior. This allows direct D → T transition if the detected pulse aligns with a previous detection (e.g., after a brief dropout).
   - **If confirmed**: sets `con_dec = true` on pulses, recommends mode **T** (tracking).
   - **If detected but not confirmed**: recommends mode **C** (confirmation).
3. If no detection: recommends mode **S** (search again).

#### Informed Discovery Mode (I)

Identical logic to Discovery, except `findpulse` is called with `freq_search_type = 'informed'`. This restricts the frequency search to the band `[ps_pre.fstart, ps_pre.fend]` (typically ±100 Hz around the last confirmed frequency). The time search remains naive (all windows).

This mode is entered when the outer loop receives an **S** suggestion but `fLock = true` — the detector lost the pulse in time but still trusts the frequency.

#### Confirmation Mode (C)

1. Calls `findpulse('naive', 'naive', excluded_freq_bands)` — full frequency and time search (same as Discovery).
2. If a detection is made:
   - Runs `confirmpulses()` to validate against the prior segment's pulses.
   - **If confirmed**: sets `con_dec = true`, recommends mode **T** (tracking).
   - **If detected but not confirmed**: recommends mode **S** (search) — the detection didn't match expectations, so confidence is not established.
3. If no detection: recommends mode **S** (search).

The key difference from Discovery: in Confirmation mode, a detection that fails the confirmation check sends the detector back to **S** (search), whereas in Discovery mode that same scenario would recommend **C** (try to confirm). This prevents infinite D ↔ C loops.

#### Tracking Mode (T)

1. Calls `findpulse('informed', 'informed', excluded_freq_bands)` — both frequency and time searches are narrowed.
   - **Frequency** is restricted to `[fstart, fend]` (typically ±100 Hz band).
   - **Time** is restricted based on projecting the last known pulse forward by `t_ip ± t_ipu ± t_ipj`.
2. If a detection is made:
   - Runs `confirmpulses()`.
   - **If confirmed**: stays in **T** (tracking). The `trackedCount` counter increments.
   - **If detected but not confirmed**: recommends mode **S** (search) — lost the pulse.
3. If no detection: recommends mode **S** (search) — lost the pulse.

Because the search space is narrowed, Tracking mode is **dramatically faster** than Discovery or Confirmation. This is critical for real-time operation where the processing time must be less than the data segment duration.

### 7e. Outer Loop Mode Mapping

The outer loop in `uavrt_detection.m` (~line 363) translates the `suggestedMode` from the previous segment's `ps_pos.mode` into the actual mode used for processing:

```
suggestedMode = 'S' (Search):
    if fLock == true  → mode = 'I' (Informed Discovery)
    if fLock == false → mode = 'D' (Discovery)

suggestedMode = 'C' → mode = 'C' (Confirmation)
suggestedMode = 'T' → mode = 'T' (Tracking), trackedCount++
```

The `fLock` flag is set to `true` when all pulses in the detected group are confirmed (`con_dec = true`), and set to `false` otherwise.

### 7f. State Transition Diagram

```
                    ┌──────────────────────────────────────────────────────┐
                    │                                                      │
                    ▼                                                      │
    ┌───────────────────────────┐                                          │
    │  D (Discovery)            │──── detection + confirmed ──────────────▶│
    │  or I (Informed Discovery)│                                          │
    │  freq: naive/informed     │──── detection, not confirmed ──▶ C       │
    │  time: naive              │                                  │       │
    │                           │──── no detection ──▶ S ──▶ D/I   │       │
    └───────────────────────────┘                                  │       │
                    ▲                                               │       │
                    │                                               ▼       │
                    │                              ┌─────────────────────┐  │
                    │                              │  C (Confirmation)   │  │
                    │                              │  freq: naive        │  │
                    │◀── det, not confirmed ────── │  time: naive        │──┘
                    │◀── no detection ──────────── │                     │  confirmed
                    │                              └─────────────────────┘  │
                    │                                                      │
                    │                              ┌─────────────────────┐  │
                    │                              │  T (Tracking)       │◀─┘
                    │                              │  freq: informed     │
                    │◀── not confirmed ─────────── │  time: informed     │
                    │◀── no detection ──────────── │                     │──┐
                    │                              └─────────────────────┘  │
                    │                                                      │
                    │                  confirmed ──────────────────────────┘
                    │                  (stays in T)
```

### 7g. Frequency Lock Behavior

The `fLock` variable in the outer loop controls whether the detector trusts a previously detected frequency:

- **Set to `true`** when `all([X.ps_pos.pl.con_dec])` — every pulse in the group was confirmed.
- **Set to `false`** when confirmation fails or no detection is made.

When `fLock = true` and the detector falls back to Search, it enters **Informed Discovery (I)** instead of full **Discovery (D)**. This means it retains the frequency lock but re-searches across all time windows. This is useful for brief signal dropouts where the tag frequency hasn't changed but timing was lost.

### 7h. Processing Time Guard

A safety check (~line 476 of `uavrt_detection.m`) monitors whether the processing time exceeds 90% of the waveform's real-time duration. If it does:

- A warning is printed suggesting the user reduce K by 1.
- All buffers are reset and the detector returns to Search mode.
- This prevents the detector from falling behind the incoming data stream, which would cause stale data accumulation.

---

## 8. Unconfirmed vs. Confirmed Pulses

Every detected pulse carries two boolean flags — `det_dec` (detection decision) and `con_dec` (confirmation decision) — defined on the `pulse` object. Understanding the difference between an **unconfirmed** and a **confirmed** pulse is central to how the detector builds confidence before narrowing its search.

### 8a. Unconfirmed Pulse (Detected Only)

A pulse is **unconfirmed** (`det_dec = true`, `con_dec = false`) when:

- The incoherent summation score at some frequency exceeded the extreme-value threshold, so the detector believes a pulse *may* be present.
- However, **no prior detection exists to validate it against**, or the validation checks failed.

This happens in two main scenarios:

1. **First-ever detection (Discovery mode)**: The detector is in mode D or I and finds a threshold-exceeding signal for the first time. There is no previous pulse to compare against, so the pulse is recorded with `con_dec = false`. The state machine moves to **Confirmation mode (C)** to re-check on the next data segment.

2. **Failed confirmation**: A pulse was detected, but when compared to the prior segment's pulse (see checks below), it did not align in time, frequency, or SNR. The detector does *not* trust it and falls back to **Search mode (S)**.

An unconfirmed pulse is still **reported** over UDP/ROS2 with `confirmed_status = 0`. The ground station can see that the detector found something but hasn't yet verified it. Critically, unconfirmed detections **do not trigger tracking mode** — the detector continues searching the full frequency/time space, which is computationally more expensive but avoids locking onto a false alarm.

### 8b. Confirmed Pulse (`con_dec = true`)

A pulse becomes **confirmed** when it passes all of the following validation checks performed by `confirmpulses.m`:

1. **Time alignment**: The detected pulse's start time must fall within a predicted time window derived from the last known pulse projected forward by the expected inter-pulse interval, accounting for uncertainty and jitter:
   ```
   t_last + (t_ip - t_ipu) * n - t_ipj  ≤  t_detected  ≤  t_last + (t_ip + t_ipu) * n + t_ipj
   ```
   where `n` is the pulse index (0-based if the last pulse overlaps into this segment, 1-based otherwise).

2. **Frequency alignment**: The detected pulse's center frequency must be within **±100 Hz** of the prior detection's center frequency. This prevents the detector from locking onto a different signal at a nearby frequency.

3. **SNR consistency**: All K pulses in the group must have `SNR ≠ -Inf`. This guards against a single loud noise transient that passes the K-pulse summation threshold while the remaining K-1 "pulses" are pure noise (which would have `-Inf` SNR). If K > 1, all pulses must have finite SNR. If K = 1, the single pulse's SNR is checked against `-Inf`.

4. **First-pulse determines group**: Only the **first pulse's** time alignment determines the entire group's confirmation status. If the first pulse is correctly aligned, the remaining K-1 pulses are confirmed by extension (since they are spaced by the expected inter-pulse interval from pulse 1).

Once confirmed, the detector:
- Sets `con_dec = true` on each pulse in the group.
- Sets `fLock = true` in the outer loop — the frequency is now considered **locked**.
- Transitions to **Tracking mode (T)**, which narrows both the frequency and time search ranges, dramatically reducing processing time.
- Updates the posteriori frequency estimate (and every 5th tracking segment, the time estimate).

### 8c. Lifecycle Example

```
Segment 1 (Discovery):
  → Score exceeds threshold at 150.123 MHz
  → No prior pulse exists → det_dec=true, con_dec=false (UNCONFIRMED)
  → State → C (Confirmation)

Segment 2 (Confirmation):
  → Score exceeds threshold at 150.123 MHz
  → Time check: pulse arrived within predicted window ✓
  → Freq check: within ±100 Hz of prior ✓
  → SNR check: all K pulses have finite SNR ✓
  → det_dec=true, con_dec=true (CONFIRMED)
  → State → T (Tracking), frequency locked

Segment 3 (Tracking):
  → Narrowed freq/time search finds pulse at 150.123 MHz
  → Confirmation checks pass again → stays CONFIRMED
  → State → T (continues tracking)

Segment N (Tracking — signal lost):
  → No score exceeds threshold in the narrowed search
  → det_dec=false, con_dec=false
  → State → S (Search) → frequency unlocked
  → Detector returns to full Discovery search
```

### 8d. Impact on Downstream Consumers

The `confirmed_status` field in the UDP/ROS2 pulse message tells the ground control station (GCS) how much to trust the detection:

| `detection_status` | `confirmed_status` | Meaning |
|----|----|----|
| 0 | 0 | No pulse found — heartbeat/no-detection message |
| 1 | 0 | **Unconfirmed** — possible pulse, not yet validated. May be a false alarm. |
| 1 | 1 | **Confirmed** — pulse validated against prior. High confidence detection. |

The GCS can use this to, for example, only update a tag's estimated position when receiving confirmed pulses, while still displaying unconfirmed pulses as tentative markers.

---

## 9. Pulse Confirmation Mechanics (Code Detail)

`confirmpulses.m` validates detected pulses against predictions from the prior segment:

1. **Time check**: Each detected pulse's start time must fall within the predicted window based on the last known pulse time projected forward by `t_ip ± t_ipu ± t_ipj`.
2. **Frequency check**: Detected pulse center frequency must be within ±100 Hz of the prior detection.
3. **SNR check**: All K pulses in the group must have SNR > -∞ (no infinite-noise detections). This rejects cases where a single loud noise spike passes the threshold but the other K-1 "pulses" are pure noise.
4. Only the **first pulse's** time alignment determines the group's confirmation status (subsequent pulses are confirmed by extension).

---

## 10. Posteriori Update & Priori Propagation

After processing, the posteriori pulse statistics (`pulsestats.updateposteriori`) are updated:

- **Frequency update** (`'freq'`): Center frequency set to detected peak; bandwidth fixed to ±100 Hz.
- **Time update** (`'time'`): Inter-pulse time updated as the mean of the observed inter-pulse time and the prior `t_ip`. Uncertainty and jitter are held constant to avoid over-narrowing.
- Frequency is updated every segment; time is updated only every 5 tracking segments to avoid drift.

The posteriori becomes the **priori for the next segment**, propagated via `ps_pre_struc`.

---

## 11. Output & Reporting

Detected pulses are transmitted via:
- **UDP** (`pulseInfoStruct.sendOverUDP()`) to a ground control station
- **ROS2** (`ros2PulseSend`) for integration with the UAV flight stack
- **CSV log** (`wfmcsvwrite`) of spectrograms for post-flight analysis

Each pulse report includes: tag ID, frequency (absolute MHz), start time, predicted next pulse time, SNR, STFT score, group sequence counter, noise PSD estimate, detection and confirmation status.

**Duplicate suppression**: If a detected pulse's time is within 10% of the interpulse interval from the previous detection, it's flagged as a repeat (from segment overlap) and not transmitted.

---

## 12. Summary Data Flow

```
UDP IQ Packets
    → Async Buffer (with gap detection/zero-fill)
    → Waveform Object (with priori pulse stats)
    → STFT (window = pulse width, 50% overlap)
    → Spectral Weighting (matched filter via W matrix, sub-bin shifts)
    → Incoherent Summation (Wq time correlator selects & sums K windows)
    → Per-frequency scores compared to EV-theory threshold
    → Peeling algorithm extracts peaks & sidebands
    → State machine (D → C → T) with confirmation
    → Posteriori update → next segment's priori
    → Pulse report via UDP/ROS2
```
