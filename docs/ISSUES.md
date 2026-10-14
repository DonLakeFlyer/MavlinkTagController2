# Known issues

Problems found in field data and the changes suggested for them. Living
document: items are added as they are found and removed when fixed (the fix
then lands in `design/`). Items 1–7 were fixed in 2026-10 and removed.

Sources, unless stated otherwise: 60-acres flight, 2026-10-13, ~122 m AGL.

| Session | Mode | Geometry |
| --- | --- | --- |
| `Logs-Rotation-2026-10-13_20-35-56` | Manual rotation | Collar ≈260 m south (true bearing 180.6° ± 1.1°; collar ≈10 ft south of the auto-rotation point) |
| `Logs-Rotation-2026-10-13_20-50-12` | Auto rotation | Collar ≈10 ft (3 m) south of the rotation point, i.e. ≈88.6° below the horizon |
| `Logs-RawCapture-2026-10-13_20-34-27` | 13 s raw capture, 400 ft hover | Archived as `flying-400ft.dat` |

All three are archived in `/Volumes/Media/PDC Testing/60-acres 10-13-26 - DShot Flight/`
(rotation folders unchanged). The same day's office bench captures are in
`/Volumes/Media/PDC Testing/Office 10-13-26 - GR01 Noise Testing/`.

Tag 2: 147.970 MHz, 19 ms pulses, dual rate 1.5 s / 2.0 s, K = 20, 8 headings.

---

## Suggested changes

Open suggestions. S1–S8 were implemented 2026-10 (per-heading results with
no cross-heading projection, frequency-only re-search, candidate identity by
frequency, sideband folding, the null-refinement train test, the simulator
rate-switch option and its tests, and TagTracker's spectrogram setting
defaulting on; see `design/COLLECTION_FLOW.md`) and removed. Numbers in
brackets are the issues below that each remaining change addresses.

### Field data collection

| # | Change | Addresses |
| --- | --- | --- |
| S9 | Archive rotation log folders with the raw captures in `/Volumes/Media/PDC Testing/` after every outing (done for 10-13). | — |
| S13 | Put vehicle (GPS) time on detector and controller log lines, or at least log the RPi-clock-to-vehicle-time offset at session start and on every change. | 11 |

### RF noise / hardware tests

| # | Test | Addresses |
| --- | --- | --- |
| S10 | PWM vs DShot A/B: same site and day, low hover (≈3 m) **and** 122 m in each mode, ESCs recalibrated after switching, same capture settings, ULog hover power and per-motor output. | 8 |
| S11 | Discovery on the desktop: 50 Ω terminator at 147.980 MHz, then antenna at 147.900 MHz. | 9 |
| S12 | Office capture with only the GR01 power lead unplugged; if the bumps vanish, go beyond the existing 2-turn clamp-on (stacked cores, antenna-lead ferrites, filtered supply). | 10 |

---

## RF noise / hardware

### 8. In-flight noise floor regressed ≈1.5 dB since 10-01 (DShot and altitude confounded)

| Flight | Site | Hover altitude (ULog) | Mid-band median (dBFS/Hz) |
| --- | --- | --- | --- |
| 9-17 (short legs, PWM) | 60 acres | no ULog | −114.7 |
| 9-21 (short legs, PWM) | Lake front | **≈4 m** | −115.1 |
| 9-22 (long legs, PWM) | Lake front | **≈1 m** | −117.4 |
| 10-02 (DShot), `log_79` | ? | 114 m (loiter, at capture time) | −115.0 |
| **10-13 (long legs, DShot), `log_83`** | 60 acres | **123.6 m** (loiter, at capture time) | **−115.0** |

The 60-acres site reads ≈0.8 dB hotter than the lake front (9-16 vs 9-17), so
≈1.5 dB is left over. Candidates from 10-01 onward:
- DShot,
- polymer props,
- arm tilt,
- FC rotation.

**But altitude is fully confounded with DShot.** Every PWM flight with a ULog
was a low hover (1–4 m, antenna about half a metre above the ground). Both
DShot flights hovered at ≈115–125 m. Height changes what the antenna sees: the
ground, and line of sight to distant noise sources. So the ≈1.5 dB cannot be
assigned to DShot on current data.

**Hover load is unchanged** (ULog, airborne, first and last 10 s trimmed):

| Flight | Current | Voltage | Power | Motor outputs | Yaw torque sp |
| --- | --- | --- | --- | --- | --- |
| 9-21 | 20.3 A | 24.36 V | 495 W | 0.84 / 0.83 / 0.54 / 0.55 (unbalanced) | +0.145 |
| 9-22 | 20.7 A | 23.96 V | 497 W | 0.69 / 0.67 / 0.69 / 0.75 | −0.020 |
| 10-02 | 22.2 A | 23.51 V | 523 W | 0.58 / 0.62 / 0.63 / 0.66 | −0.020 |
| 10-13 rot 1 | 21.4 A | 23.20 V | 497 W | 0.60 / 0.59 / 0.63 / 0.64 | −0.022 |
| 10-13 rot 2 | 21.2 A | 23.33 V | 495 W | 0.61 / 0.57 / 0.64 / 0.63 | −0.022 |

- **Electrical power at hover matches 9-22 within 0.5%.** The ≈3% higher
  current is the lower pack voltage. So the new props and arm tilt did not
  change the load.
- **Motor balance on 10-13 is comparable to 9-22.**
- **ESC RPM (DShot telemetry):** 4116–4486, a ≈9% spread.
- **Motor output values can't be compared across the PWM → DShot change**,
  because the output-to-throttle mapping changed.
- **ESC current telemetry reads 0** on all ESCs.

The lift is tilted: ≈+2.5 dB below 147.95 MHz, ≈+1.3 dB above 148.1 MHz.

The DShot frames and bidirectional replies run continuously when disarmed, and
the powered-at-rest office capture showed no floor lift. So if DShot is the
cause, it acts through ESC switching under load, not the signal wires.

**Planned:** PWM vs DShot A/B on 10-14:
- same site, same day;
- **in each mode, capture at a low hover (≈3 m) and at 122 m.** This separates
  altitude from DShot in one outing;
- ESCs recalibrated after switching to PWM, with motor balance checked;
- same capture settings;
- ULog hover power and per-motor output compared with 10-13 (≈496 W).

### 9. Narrow humps at 147.705 and 147.995 MHz in every October capture

≈15 kHz wide, ≈ −116 dBFS/Hz absolute. They appear at the same level:
- in the office with the vehicle off (Discovery on the desktop over coax),
- in the office with the vehicle powered,
- at 400 ft.

They are absent from every September capture at the same tune (9-13 office;
9-16, 9-17, 9-21 and 9-22 flights), with the same Discovery serial
`0xC852AA80BE44463D` throughout. In flight they add +1.2 / +2.3 dB; the upper
one is 25 kHz from the tag. Suspected to be inside the SDR.

**Test** (desktop, vehicle not involved):
1. 50 Ω terminator on the input at 147.980 MHz.
2. Antenna, tuned to 147.900 MHz.

If they survive the terminator and move with the tune, they are SDR-internal.

### 10. GR01 at rest: small bumps, ferrite had no effect

Office, 147.98 MHz tune. With the vehicle powered (via RPi), the floor rises
≈0.4–0.6 dB over the vehicle-off baseline (−121.5 dBFS/Hz), plus broad ≈1.6 dB
bumps ≈110–120 kHz apart that moved ≈250 kHz between captures. A 2-turn
clamp-on ferrite on the GR01 power lead, right outside its shielded box, left
their height unchanged. It was also fitted for all of the 10-13 flights. The
source (GR01 vs residual from the UBEC DUO) is not identified.

**Test:** capture in the same spot with only the GR01 power lead unplugged.
If the bumps vanish, the GR01 is the source. Since 2 turns already did
nothing, the next steps would be:
- stacked cores / more turns;
- a ferrite on its two antenna leads, which also leave the box;
- a filtered supply instead of raw 6S from the PDB.

The office ambient at 147.98 MHz on 10-13 (−121.5) was ≈7 dB above the
September office baselines at 146.000 MHz, so the office cannot currently
resolve vehicle noise below that level.

---

## Logging / observability

### 11. RPi clock is not vehicle time; logs cannot be lined up with the ULog

Log folder names use vehicle time (`LogFileManager`: `vehicleTimeNow()`).
Everything inside uses the RPi system clock, which in the field has no network
time source:
- controller log lines,
- detector `timestamp_ns` and cycle times,
- raw-capture `capture_utc`.

| | RPi clock | Vehicle / GPS time (folder name or ULog) | RPi behind by |
| --- | --- | --- | --- |
| Office raw captures (on the network) | 19:51:15 | 19:51:29 | 14 s |
| 60-acres raw capture | 20:12:07 | 20:34:27 | 22 min 20 s |
| Rotation 1, slice 2 armed vs 045° dwell start | 20:14:32 | 20:36:53 | 22 min 21 s |
| Rotation 2, slice 1 armed vs 000° dwell start | 20:26:45 | 20:50:23 | 23 min 38 s |

The offset also **changed by ≈77 s between the two rotations**, so even a
single correction per outing is not enough. Lining detector events up with
the ULog (attitude, altitude, motors), or with what people at the collar did,
currently needs dwell-by-dwell manual mapping.

Example: the rotation-1 rate switch happened in the 045° dwell, which is
20:14:32–20:15:12 by the RPi clock but **20:36:53–20:37:32 UTC (13:37 local)**
by GPS.
