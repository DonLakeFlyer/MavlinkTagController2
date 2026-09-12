# Airspy Mini vs HF+: SNR, Noise Floor, and Frequency Stability

Field data from back-to-back runs at 3 km range, 400 ft altitude (Apr 8 2026).

- **Airspy Mini**: `airspy_rx` → `csdr-uavrt` (8× FIR) → `airspy_channelize` → UDP → **uavrt_detection** (C++ / MATLAB-compiled)
- **Airspy HF+**: `airspyhf_zeromq_rx` → ZMQ → `airspyhf_decimator` (200×) → UDP → **pulse_detector.py** (Python)

## SNR and Noise Floor

| Metric | Airspy Mini (C++ detector) | Airspy HF+ (Python detector) |
|---|---|---|
| **Typical SNR** | 10–23 dB (high variance) | 14–28 dB (higher, more stable) |
| **Peak SNR** | ~24 dB | ~28 dB |
| **Noise PSD** | 7e-09 to 3e-08 | 2e-14 to 5e-12 |
| **stft_score** | 0.001 – 0.4 | 1e-12 to 1e-09 |

The HF noise floor is ~4 orders of magnitude lower (picoWatts vs nanoWatts). The HF+ has superior dynamic range and the ZMQ→decimator path preserves 32-bit float precision. The stft_score values are correspondingly much smaller in absolute terms (but detection still works because the threshold is proportional to the per-bin noise power).

## Frequency Stability

- **Mini / C++ detector**: The detected frequency drifts across bins — `146168257` to `146171414`, a spread of ~3 kHz. The C++ detector uses `freqSearchHardLock` mode but still wanders across frequency bins between segments.
- **HF+ / Python detector**: Locks to `146170265` (or `146170232`) Hz and stays there. Two closely-spaced bins with only 33 Hz separation. Much tighter lock.
