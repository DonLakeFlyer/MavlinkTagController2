# Pulse Detector Tests

Unit and end-to-end tests for the Python VHF pulse detector (`detector/pulse_detector.py`), covering single-rate detection, multi-hypothesis rate-switch detection, and the STFT→fold pipeline internals.

## Run

```bash
.venv/bin/python -m pytest detector/tests/ -v
```

## Prerequisites

- Python 3 with numpy, scipy
- `conftest.py` adds `detector/` and `simulator/` to `sys.path`

## Tests

### `test_end_to_end.py`

Full pipeline tests that generate IQ data with known pulses, run detection, and verify results.

#### TestSingleRateK5

| Test | Description |
|------|-------------|
| `test_strong_signal_detected` | 25 dB signal is detected with K=5 folding |
| `test_strong_signal_snr_reasonable` | Detected SNR exceeds 10 dB for a 25 dB input |
| `test_strong_signal_frequency_accuracy` | 200 Hz frequency offset is recovered within 2 FFT bins |
| `test_noise_only_no_detection` | Pure noise produces zero detections and finite noise PSD |
| `test_moderate_signal_detected` | 12 dB signal is detected with K=5 |
| `test_fold_info_present` | Detection result contains `uniformity` and K `fold_snrs` entries |
| `test_negative_freq_offset` | −500 Hz offset is correctly detected |
| `test_detection_margin_tight` | Tight detection margin (0.99) still detects a 25 dB signal |
| `test_reproducibility` | Identical seeds produce identical detection results |

#### TestSingleRateK20

| Test | Description |
|------|-------------|
| `test_strong_signal_detected` | 25 dB signal detected with K=20 |
| `test_weak_signal_detected` | 6 dB signal recovered with K=20 integration gain |
| `test_noise_only_no_detection` | Pure noise with K=20 produces no detections |
| `test_k20_snr_exceeds_k5` | K=20 yields higher SNR than K=5 for the same 15 dB signal |
| `test_fold_offsets_length` | Fold offsets array has K=20 entries |
| `test_frequency_accuracy` | −300 Hz offset preserved with K=20 |
| `test_uniformity_filter` | Strong signal passes uniformity ≥0.25 filter |

#### TestRateSwitchK5

| Test | Description |
|------|-------------|
| `test_a_to_b_c2_detected` | A→B rate switch at change-point 2 is detected and labeled |
| `test_b_to_a_c2_detected` | B→A rate switch at change-point 2 is detected and labeled |
| `test_pure_a_identified_with_dual_hypotheses` | Pure rate-A signal is labeled 'A' under dual-rate search |
| `test_pure_b_identified_with_dual_hypotheses` | Pure rate-B signal is labeled 'B' under dual-rate search |
| `test_a_to_b_snr_matches_pure_rate` | A→B switch fold SNR is within 3 dB of pure rate-A |
| `test_b_to_a_snr_matches_pure_rate` | B→A switch fold SNR is within 3 dB of pure rate-B |
| `test_a_to_b_change_point[1,2,3]` | Each interior A→B change-point is correctly identified (parametrized) |
| `test_b_to_a_change_point[1,2,3]` | Each interior B→A change-point is correctly identified (parametrized) |
| `test_noise_only_dual_rate` | Noise-only with dual-rate hypotheses produces no detection |
| `test_uniformity_passes_correct_switch` | Correct switch hypothesis passes uniformity filter |
| `test_fold_info_has_k_entries` | Rate-switch detection `fold_info` contains K per-fold SNRs |

### `test_rate_switch.py`

Unit tests for the multi-hypothesis rate-switch internals.

#### TestBuildHypothesisIndices

| Test | Description |
|------|-------------|
| `test_single_rate_produces_one_hypothesis` | N_B=None yields exactly one "A" hypothesis |
| `test_two_rates_produce_eight_hypotheses` | Dual-rate yields 8 hypotheses (A, B, + 3 A→B + 3 B→A) |
| `test_pure_A_indices` | Pure-A indices are evenly spaced by N_A |
| `test_pure_B_indices` | Pure-B indices are evenly spaced by N_B |
| `test_switch_A_to_B_indices` | A→B switch indices use N_A before change-point, N_B after |
| `test_switch_B_to_A_indices` | B→A switch indices use N_B before change-point, N_A after |
| `test_all_indices_in_bounds` | All hypothesis indices are within [0, n_time) |
| `test_same_rate_gives_single_hypothesis` | N_B==N_A collapses to single hypothesis |
| `test_small_n_time_reduces_hypotheses` | Hypotheses whose span exceeds n_time are dropped |
| `test_hypothesis_shape` | Each hypothesis index array is (search_range, K) |
| `test_switch_hypotheses_search_full_valid_offset_range` | Switch hypotheses search all valid t0 offsets |
| `test_fractional_pri_rounds_independently` | Fractional PRI offsets are independently rounded per fold |
| `test_fractional_pri_matches_fold_offsets` | Hypothesis indices match the `fold_offsets` rounding strategy |
| `test_fractional_switch_offsets` | Switch hypotheses with fractional PRI use correct cumulative rounding |

#### TestFoldMultiHypothesis

| Test | Description |
|------|-------------|
| `test_known_signal_correct_hypothesis_wins` | Injected A→B signal is found by the matching hypothesis |
| `test_pure_signal_detected_by_pure_hypothesis` | Pure-A signal is found by hypothesis "A" |
| `test_single_rate_fold_matches_manual` | Single-rate multi-hypothesis fold matches manual numpy computation |

#### TestUniformityCheck

| Test | Description |
|------|-------------|
| `test_uniform_signal_passes` | Uniform power across folds yields U=1.0 and passes |
| `test_one_hot_fails` | Power concentrated in one fold fails uniformity |
| `test_boundary_passes` | min/max ratio exactly at threshold passes |
| `test_boundary_just_below_fails` | min/max ratio just below threshold fails |
| `test_zero_power_fails` | All-zero power returns U=0.0 and fails |

#### TestComputeSegmentSamples

| Test | Description |
|------|-------------|
| `test_single_rate` | Single-rate segment length formula is correct |
| `test_dual_rate_uses_max_span` | Dual-rate uses `max(N_A, N_B)` for segment sizing |
| `test_dual_rate_A_larger` | When N_A > N_B, N_A determines segment length |

#### TestEvtCacheNaming

| Test | Description |
|------|-------------|
| `test_new_format_differs_from_legacy` | New cache path format doesn't collide with legacy |
| `test_single_rate_new_differs_from_legacy` | Single-rate new format avoids legacy collision |
| `test_different_N_B_gives_different_path` | Different N_B values produce different cache paths |
| `test_round_trip_save_load` | EVT cache save/load round-trips correctly |
| `test_load_mismatched_N_B_returns_none` | Loading with wrong N_B returns None |

#### TestFoldDetectEndToEnd

| Test | Description |
|------|-------------|
| `test_single_rate_detects_pure_A` | Full STFT→fold pipeline detects injected pure-A pulses |
| `test_dual_rate_detects_switch` | Full pipeline detects A→B rate switch with correct label |
| `test_no_regression_single_rate` | Legacy path matches explicit single-rate hypotheses |
| `test_uniformity_rejects_one_hot` | One-hot spike is rejected by uniformity filter |
| `test_all_detections_filtered_returns_best_cand` | When all detections are filtered, `best_cand` is still returned |

#### TestFoldRegressionSingleRate

| Test | Description |
|------|-------------|
| `test_scores_match_direct` | `fold_multi_hypothesis` scores match direct numpy computation |

#### TestHypLabelToGroupInd

| Test | Description |
|------|-------------|
| `test_pure_a` / `test_pure_b` | Pure labels map to correct group_ind constants |
| `test_a_to_b_c1` / `test_a_to_b_c3` | A→B change-point labels map to correct group_ind (1+c) |
| `test_b_to_a_c1` / `test_b_to_a_c3` | B→A change-point labels map to correct group_ind (K-1+c) |
| `test_unknown_label_defaults_a` | Unknown label defaults to group_ind A |
| `test_single_rate_always_zero` | Single-rate always produces group_ind 0 |
| `test_group_ind_ranges_no_overlap_k5` | A→B and B→A group_ind ranges are disjoint |
| `test_last_rate_determines_predict_tip` | `last_rate` field correctly selects the TIP for prediction |
