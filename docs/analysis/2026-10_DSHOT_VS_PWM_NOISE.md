# DShot vs PWM: in-flight noise floor — 2026-10-14

Same-day A/B at the 60-acres site to explain the in-flight noise-floor rise
that appeared when the ESCs moved from PWM to DShot on 2026-10-01. Data,
plots and scripts: `/Volumes/Media/PDC Testing/60-acres 10-14-26 - DShot vs PWM/`
(README there has the full tables and the reproduce commands).

**Result: DShot raises the floor 2.7–3.2 dB at the tag channel. The vehicle
now flies 100 Hz PWM** (configuration in
[PDC_VHF_DRONE.md](../hardware/PDC_VHF_DRONE.md)).

## Background

From 10-01 the in-flight mid-band floor was ≈1.5 dB worse than the September
PWM flights after allowing ≈0.8 dB for site (60 acres reads hotter than the
lake front):

| Flight | Site | ESC protocol | Hover altitude | Mid-band median (dBFS/Hz) |
| --- | --- | --- | --- | --- |
| 9-17 | 60 acres | PWM | no ULog | −114.7 |
| 9-21 | Lake front | PWM | ≈4 m | −115.1 |
| 9-22 | Lake front | PWM | ≈1 m | −117.4 |
| 10-02 (`log_79`) | ? | DShot | 114 m | −115.0 |
| 10-13 (`log_83`) | 60 acres | DShot | 123.6 m | −115.0 |

Four things changed on 10-01: DShot, polymer props, arm tilt, FC rotation.
Altitude was also fully confounded with the protocol (every PWM flight with a
ULog was a 1–4 m hover; both DShot flights were at ≈115–125 m).

Hover load was unchanged (ULog, airborne, first and last 10 s trimmed), so
props and tilt did not change the load:

| Flight | Current | Voltage | Power | Motor outputs |
| --- | --- | --- | --- | --- |
| 9-21 (PWM) | 20.3 A | 24.36 V | 495 W | 0.84 / 0.83 / 0.54 / 0.55 (unbalanced) |
| 9-22 (PWM) | 20.7 A | 23.96 V | 497 W | 0.69 / 0.67 / 0.69 / 0.75 |
| 10-02 (DShot) | 22.2 A | 23.51 V | 523 W | 0.58 / 0.62 / 0.63 / 0.66 |
| 10-13 rot 1 (DShot) | 21.4 A | 23.20 V | 497 W | 0.60 / 0.59 / 0.63 / 0.64 |
| 10-13 rot 2 (DShot) | 21.2 A | 23.33 V | 495 W | 0.61 / 0.57 / 0.64 / 0.63 |

## Method

- Four 13 s `airspyhf_rx` captures, one vehicle, minutes apart: low and high
  hover on DShot300 (7.3 m, 124.3 m), then low and high hover on 100 Hz PWM
  (2.5 m, 120.0 m). Heights from the ULogs; each capture is inside a steady
  hover.
- ESCs recalibrated to 1000–2000 µs (under ArduCopter 4.6) before the PWM
  flights. Nothing else on the vehicle changed between the two pairs.
- Tag 147.970 MHz, tuned 147.980 MHz, 768 kS/s, gain 0, LNA on, attenuator
  0 dB, SDR serial `0xC852AA80BE44463D` (the same unit as all earlier
  captures).
- Welch PSD, 20 ms Hann, 50 Hz bins; mid-band = |f| < 300 kHz.

## Results

| Capture | Mid-band median | p10 / p90 | Tag channel | 147.705 MHz hump | 147.995 MHz hump |
| --- | --- | --- | --- | --- | --- |
| DShot, low | −114.9 | −116.1 / −114.4 | −114.7 | +1.8 dB | +1.3 dB |
| DShot, high | −115.0 | −116.2 / −114.3 | −114.8 | +1.9 dB | +1.5 dB |
| PWM, low | −117.6 | −117.9 / −117.1 | −117.7 | +0.3 dB | +0.0 dB |
| PWM, high | −118.2 | −118.5 / −117.8 | −118.2 | +0.3 dB | +0.0 dB |

All levels dBFS/Hz; hump height is above the level 40 kHz either side.

1. **The regression is DShot.** The protocol moves the floor ≈3 dB; low vs
   high hover moves it 0.1 dB (DShot) and 0.6 dB (PWM). Today's DShot matches
   the 10-13 400 ft capture to 0.1 dB; today's PWM matches the 9-22 PWM
   flight. Props, tilt and FC rotation were unchanged between the pairs.
2. **The 147.705 / 147.995 MHz humps in flight come with DShot.** On PWM they
   are gone on a floor 3 dB lower. The same humps in the 10-13 office capture
   with the vehicle off are not explained by this data.
3. **The 147.985 MHz carrier is unchanged** at −102.8 dBFS/Hz in all four
   captures, independent of ESC protocol.
4. **Detector noise agrees.** Two manual 8-heading rotations on PWM (collar
   ≈520 m south, ≈400 ft) had detector `noise_psd` medians of 1.39e-12 and
   1.48e-12, 3.1–3.4 dB below the 10-13 DShot rotations (3.07e-12,
   3.05e-12). Both bearings were within GPS error of truth (+0.7°, −0.1°;
   R² 0.76 / 0.81).

## Consequences

- Without bidirectional DShot there is no ESC RPM telemetry, so the ESC-RPM
  gyro notch has no input.
- Motor output values are not comparable across the 10-01 and 10-14 protocol
  changes (the output-to-throttle mapping differs).
