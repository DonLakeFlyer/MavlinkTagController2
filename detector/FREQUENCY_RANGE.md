# Frequency Detection Range

## DC Spike Avoidance

The Airspy HF+ has a hardware DC spur at its tuned center frequency. The pipeline avoids this with a two-stage offset:

1. **Radio tuning**: The HF+ is tuned **+10 kHz above** the requested center frequency (`kAirSpyHfFrequencyOffsetHz = 10000` in `CommandHandler.h`).
2. **Decimator shift**: The decimator applies `--shift-khz 10` to digitally shift the signal back, re-centering the target at DC while pushing the hardware DC spike to +10 kHz — well outside the ~3.8 kHz decimated bandwidth.

The Airspy Mini does **not** apply this offset — it tunes to the exact requested frequency.

## Detection Bandwidth

The detector searches all frequency bins across the full decimated bandwidth. The maximum detectable offset from the tuned center frequency is ±Fs/2:

| Radio | Decimated Fs | Detection range |
|-------|-------------|-----------------|
| HF+   | 3840 Hz     | ±1920 Hz        |
| Mini   | 3750 Hz     | ±1875 Hz        |

Signals near the band edges will be attenuated by the decimation FIR filter roll-off, reducing detection sensitivity.

## Frequency Resolution

The detector uses a spectral weighting matrix with sub-bin shifts (zetas = [0.0, 0.5]), doubling the effective frequency resolution compared to a plain FFT. The number of output frequency bins is `nfft = 2 × n_w`, where `n_w = ceil(tp × Fs)`.

| Pulse width (tp) | Fs (Hz) | n_w (samples) | nfft (bins) | Resolution (Hz/bin) |
|-------------------|---------|---------------|-------------|---------------------|
| 15 ms             | 3840    | 58            | 116         | ~33 Hz              |
| 20 ms             | 3840    | 77            | 154         | ~25 Hz              |
| 15 ms             | 3750    | 57            | 114         | ~33 Hz              |
| 20 ms             | 3750    | 75            | 150         | ~25 Hz              |

## Pipeline Paths

Both radio types produce identical UDP packets at the detector input — the ZMQ vs pipe distinction is entirely upstream:

```
HF+:   airspyhf_zeromq_rx → [ZMQ] → airspyhf_decimator → [UDP] → pulse_detector.py
Mini:  airspy_rx → [pipe] → csdr-uavrt → [pipe] → airspy_channelize → [UDP] → pulse_detector.py
```

### UDP Packet Format (both paths)

| Offset | Size | Content |
|--------|------|---------|
| 0      | 8 B  | Timestamp: `complex<float>` — real = uint32 seconds (bit-cast to float32), imag = uint32 nanoseconds (bit-cast to float32) |
| 8      | N×8 B | IQ samples: `complex64` (float32 I + float32 Q per sample) |

### Differences at Detector Input

| Parameter | HF+ | Mini |
|-----------|-----|------|
| `--fs`    | 3840 | 3750 |
| `--port`  | 10000+ | 20000+ |

## Raw Captures

Raw IQ captures with the Airspy HF+ also include the +10 kHz tuning offset (no decimator compensation). The signal of interest appears at −10 kHz in the baseband spectrum. Post-processing must account for this offset.

Airspy Mini raw captures tune to the exact requested frequency — no offset.
