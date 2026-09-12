# Pulse Detector Tests

pytest suite for `detector/`. `conftest.py` adds `detector/` and `simulator/`
to `sys.path`; the end-to-end tests synthesise IQ with the simulator's
pulse model.

## Run

```bash
./setup_venv.sh                                    # once
.venv/bin/python -m pytest detector/tests -v
.venv/bin/python -m pytest detector/tests/test_end_to_end.py -k K20   # subset
```

No hardware or network needed. Every `fold_detect` call derives its threshold
from the test spectrogram itself, so there is no per-geometry warm-up.

## Files

| File | Covers |
| --- | --- |
| `test_end_to_end.py` | Full STFT·W → fold → threshold → peak pipeline on synthetic IQ. `TestSingleRateK5`, `TestSingleRateK20` (detection, SNR, frequency accuracy, noise-only, margin, reproducibility, K-gain), `TestRateSwitchK5` (A→B / B→A at every change-point, pure-rate labelling, SNR parity with pure rates, noise-only), lock-candidate bank and `fit_lock_timing` behaviour across cycles |
| `test_rate_switch.py` | Multi-hypothesis internals: `build_hypothesis_indices` (counts, spacing, bounds, fractional-PRI rounding), `fold_multi_hypothesis` vs direct numpy, dominant-fold / uniformity gate, `compute_segment_samples`, `hyp_label_to_rate_state` wire values |
| `test_threshold_null.py` | `permutation_null_threshold` (Gumbel fit, false-alarm rate on Gaussian slices, heavy tails raise it, tag never lowers it, masked bins, too few permutations), `resample_windows` refinement removing a detected train from the null, `fold_detect` per-cycle `cycle_threshold` record and `fixed_threshold` test hook, `blank_impulses` (bursts zeroed, pulses untouched, fraction reported, floor lowered) |
| `test_iq_stream.py` | Continuous timeline: contiguous takes, small-gap zero-fill (< 2·tp), large/negative-gap barrier, take across barrier rejected, `head` on ARM, history retirement and counting, gap-event callback |
| `test_udp_receiver.py` | Bounded ring FIFO, overflow drops newest and counts, blocking pop, receive thread delivers all datagrams |
| `test_collection_control.py` | `ARM` lifecycle: arm/complete/re-arm, duplicate ARM idempotent, re-ARM of a completed slice replays, conflicting ARM rejected while collecting, stale completion ignored, cancel, foreign tag id ignored, non-ARM packets rejected |
| `test_detector_protocol.py` | TTDP encode/decode: little-endian header layout, `ArmPayload` round trip, `PulsePayload` round trip incl. integer status fields and `candidate_id` default 0, `NoDetection` message type, `Failed` error codes matching `shared/detector_protocol.h`, malformed packets rejected. Mirror of `controller/tests/test_detector_protocol.cpp` |
| `test_pulse_reporting.py` | `send_pulse_udp` success/failure return and `candidate_id` on the wire |
| `test_structured_logging.py` | `StructuredLogger` `.jsonl` output (types, UTF-8, numpy values, write-failure fallback), `read_jsonl` truncation tolerance, schema of `STARTUP` / `DETECTION` / `NO_DETECTION` / `SESSION_END`, `--dump-spectrogram` sidecar files |

Adding a test: new behaviour in `pulse_detector.py` gets an end-to-end case
that fails before the change; wire-format changes get a case here *and* in
`controller/tests/`.
