# USB / airframe noise testing — lake front, 2026-09-16

Three 13 s `airspyhf_rx` noise captures from the vehicle at the lake front,
plus the discussion of where the in-flight noise floor is coming from and what
to try next. Raw data, PSD plots and a per-capture README are in
`/Volumes/Media/PDC Testing/Lake Front 9-16-26 - ESC Ferrites/`.

Hardware under test: Airspy HF+ Discovery **direct-connected to the antenna**,
USB cable up to the avionics box (3D-printed, wrapped in Titan RF fabric,
containing the Pi). Vehicle changes: ferrites at the exit of the ESC power
lines from the motor arms (all captures); avionics box grounded (capture 3).

## Summary

- Flight raises the noise floor **~13–14 dB uniformly** across the 768 kHz
  band. The noise is continuous, not bursty.
- The powered-but-not-flying floor has structure (broad humps plus fixed
  narrowband carriers); in flight the humps are buried ~10 dB under a flat
  floor, while the carriers (147.88, 147.985, 148.14 MHz) persist at the same
  level and are unrelated to flight. **Ambient/site noise is irrelevant in
  flight** — the airframe dominates.
- **Grounding the avionics box had no measurable effect** (≤ 0.4 dB in a
  single A/B pair; flight-to-flight variation was not measured, so this is one
  observation, not a bound on the effect). Expected in hindsight: the SDR is
  outside the box, so the wrap can only contain the box's own emissions, and
  those are not what sets the floor.
- Compared with the April 2026 flights (different SDR unit, site, frequency),
  today's in-flight floor is ~1.5–4 dB lower. That cannot be attributed to the
  ferrites: the April same-day run-to-run spread was 6–10 dB.
- The remaining ~13 dB is arriving either **through the antenna proper**
  (ESC/motor radiation) or as **common-mode on the USB cable** to the SDR. A
  USB galvanic isolator at the SDR end is the cleanest way to split those two.

---

## Captures

| File | Condition | UTC |
|------|-----------|-----|
| `airspy-hf.1.dat` | Vehicle powered, on the ground, not flying | 2026-09-16 21:13:45 |
| `airspy-hf.2.dat` | Flying, ferrites, box ungrounded | 2026-09-16 21:14:57 |
| `airspy-hf.3.dat` | Flying, ferrites, **box grounded** | 2026-09-17 00:45:11 |

Common settings (from the `.json` sidecars): tag 147.970 MHz, tuned
147.980 MHz (+10 kHz DC-spike offset, tag at −10 kHz), 768 kS/s, 13 s,
complex_float32, gain 0, HF AGC off, HF LNA on, attenuator 0 dB, SDR serial
0xC852AA80BE44463D.

## Measurements

`analyzer/psd_spectrum.py` (Welch, 50 Hz bins, |f| ≤ 200 Hz excluded), the
mean per-bin power of a 100 ms FFT (10 Hz bins) over the ±2 kHz tag channel,
and the detector's own `noise_psd` (mix → 8×5×5 decimate → STFT →
`estimate_noise`, divided by `Fs·n_w`, median over bins).

"Average noise floor" is the log-domain mean over every Welch bin, including
the HF+ anti-alias roll-off at the band edges, so it sits ~3 dB below the
mid-band plateau read from the plots (that gap is also what the flatness
figure reports). The per-bin tag-channel power is not integrated channel
power; summing the 400 bins would add +26 dB. Only differences between
captures are used below, and those are unaffected by either choice.

The ~29 dB spikiness is not an in-band feature: it is a fixed spur at
147.601 MHz (−379 kHz offset, −126 dBFS/Hz in all three captures) sitting in
the band-edge roll-off, where the median baseline is ≈ −155 dBFS/Hz. The only
real in-band carrier is a fixed −103 dBFS/Hz spur at 147.985 MHz (+5 kHz from
tune, 13 dB above the flying plateau), also identical in all three captures.
Mid-band medians (|f| < 300 kHz): −131.4 / −115.5 / −115.1 dBFS/Hz.

| Metric | 1 Powered | 2 Flying, ungrounded | 3 Flying, grounded | 2→3 |
|--------|-----------|----------------------|--------------------|-----|
| Average noise floor (dBFS/Hz) | −132.6 | −118.5 | −118.3 | +0.2 dB |
| Spectral flatness (dB, 0 = flat) | −4.16 | −3.12 | −3.08 | — |
| Spikiness (max peak above median) | 29.4 | 29.5 | 29.4 | — |
| DC spike above baseline | 0.20 dB | 0.13 dB | 0.14 dB | — |
| Tag-channel power per 10 Hz bin, ±2 kHz mean, median over blocks (dBFS) | −118.3 | −105.6 | −105.2 | +0.4 dB |
| Tag-channel power per bin, block-to-block std | 0.63 dB | 0.27 dB | 0.21 dB | — |
| Full-band RMS (dBFS) | −69.4 | −56.5 | −56.3 | +0.2 dB |
| Peak \|x\| | 0.005 | 0.016 | 0.019 | no clipping |
| Detector `noise_psd` (median over bins) | 1.47e-13 | 2.71e-12 | 2.92e-12 | +0.3 dB |

**Powered, not flying.** Lumpy floor: broad hump 147.98–148.10 MHz peaking at
−121.5 dBFS/Hz, narrow humps at 147.88 and 148.14 MHz (−121), smaller ones at
147.68 / 147.73 / 148.26 MHz, and the fixed −103 dBFS/Hz spur at 147.985 MHz.
The tag frequency sits on the shoulder of the big hump at ≈ −128 dBFS/Hz.

**Flying.** Flat mid-band plateau ≈ −115.5 dBFS/Hz (the table's −118.5 is the
log-mean including the band-edge roll-off). Only the fixed carriers still
stand out: 147.88 and 148.14 MHz (~6 dB) and 147.985 MHz (~13 dB); they are
present at the same level in all three captures and unrelated to flight.
Block-to-block std 0.2–0.3 dB: steady broadband noise, not impulsive.

## Comparison with April 2026 (Apr-9 Afternoon 2 - No Wind)

The April sessions have no raw captures, but each `py_detector_*.log`
`DETECTED` line reports `noise` (= `noise_psd`; the `Fs·n_w` scaling and
`compute_stft_power` are unchanged since then, verified against commit
`e7852a2`).

| Condition | `noise_psd` | vs today flying |
|-----------|-------------|-----------------|
| Today, powered on ground | 1.47e-13 | −12.7 dB |
| Today, flying | 2.71e-12 / 2.92e-12 | 0 |
| Apr-9 flying, true tag locks (runs 2–3, +330…+460 Hz) | 4.0–6.7e-12, typ. 4.5e-12 | +1.4 to +3.9 dB (typ. +2 dB) |
| Apr-9 flying, run 1 off-peak bins | 1.1–2.8e-11 | +6 to +10 dB |

Confounders that each could be 2–4 dB on their own: different SDR unit
(0x3B52… in April), different flight state / motor current, single-bin value at
the lock vs today's across-bin median. Site and frequency are **not**
confounders: today's flying spectrum shows ambient buried ~10 dB under the
airframe noise, and is flat across 768 kHz. The decisive point is that the April
same-day spread (run 1 vs runs 2–3) is larger than the effect being looked for,
so "maybe 1.5–4 dB from ferrites" is a hint, not a result.

---

## Where the noise is coming from

### The avionics box wrap

Titan RF fabric is good material for 150 MHz (nickel/copper on polyester,
60+ dB in a lab clamp), but a wrapped 3D-printed box is only as good as its
seams, apertures and bonding:

- Unbonded, it is a floating capacitor plate — ESC common-mode couples onto it
  and re-radiates inward. Typically no help, occasionally a dB or two worse.
- The lid seam and every cable slit is an aperture; the fabric needs to overlap
  and be under compression.
- USB and power leads pass straight through; the fabric does nothing for
  noise riding on them.

None of this matters for the receiver, though: **the SDR is outside the box.**
The wrap can only contain the box's own emissions (Pi, regulators). Putting the
Discovery *inside* the box was previously a disaster, so the box is a strong
emitter — but capture 3 shows those emissions are not what sets the in-flight
floor.

### The USB cable

With the Discovery direct-connected to the antenna, its only ground reference
is its case, tied to Pi ground through the USB shield. Every bit of common-mode
noise on the Pi/airframe ground appears directly at the receiver's reference.
The cable also runs past the arms/ESC lines and picks up radiated common-mode
along its length.

If the antenna is a whip/monopole, the situation is worse: with no ground plane
of its own, **the SDR case + USB cable is the counterpoise** — the other half of
the antenna — and it runs into the noisiest box on the aircraft. A dipole or
Yagi with a balanced feed is much less sensitive to this.

The flat, steady, broadband in-flight floor is consistent with conducted /
common-mode noise, but it is not diagnostic: broadband ESC/motor radiation
picked up by the antenna would look the same. The A/B below is what separates
the two paths.

### Through the antenna proper

ESC/motor switching radiated and picked up by the antenna. Nothing on the USB
or grounding side touches this; the fix is at the source (ESC lines, motor
wires, ferrites there). Cannot be separated from the USB path without an A/B.

---

## Mitigations considered

### USB galvanic isolator (Topping HS02) — first thing to try

- **iFi iDefender+** breaks the USB ground and VBUS from the host when fed a
  separate 5 V (iFi's FAQ: a bus-powered device forces it into bypass unless
  external 5 V is supplied), while the data pair stays DC-coupled. So it does
  address the ground path and needs the same floating 5 V as the HS02; it is a
  cheaper alternative, but the HS02 also isolates the data pair and is
  preferred.
- **Topping HS02** is a true USB 2.0 High Speed (480 Mbps) galvanic isolator:
  data, ground and power all broken by a digital isolator. No DC path between
  Pi ground and SDR ground. Mount it at the **SDR end** with a short cable to
  the Discovery; the long cable back to the Pi then no longer conducts
  Pi/airframe common-mode into the SDR. It can still radiate, so keep it routed
  away from the antenna — this is a conducted-path fix, not a shield.
- Downsides to weigh: audio-desk product (not vibration rated, connectors are
  the weak point); ~45 g device (the 130 g seen on retail listings is the
  packaged weight); occasional HS-USB enumeration quirks — bench-test
  with the Pi first; isolating shrinks the counterpoise of a monopole and may
  detune it.

**Power.** The Discovery is bus-powered (~200–300 mA at 5 V). Topping's manual:
*"For USB devices powered via the USB port, the HS02 requires a 5V power supply
… Connect a 5V DC power supply to the auxiliary power input jack."* Neither
the manual nor retailer spec tables state the bus-powered output current, so
plan on the aux supply; verify the bus-powered limit on the bench before
relying on it. And: *"Connecting the auxiliary power input to the same PC
being isolated will defeat the isolation."* On the aircraft that means the aux
jack must **not** be fed from the Pi's 5 V or from any non-isolated regulator
whose output negative is battery negative — that reconnects SDR ground to the
airframe. An isolated converter's input negative may be battery negative; its
floating output is what matters.

Options for a floating 5 V:

- **USB power bank** (~100 g for 5,000 mAh, ~10 h runtime) into the aux input.
  Floating by definition. Zero build. Use this for the A/B test.
- **Isolated DC-DC** from the 6S pack (19.8–25.2 V): 18–36 V in, regulated 5 V
  out, 3 W. Recom RS3-2405S / Traco TEN 3-2411 (SIP/DIP, need four wires + two
  caps soldered), or pre-built modules with leads/terminals (Mean Well
  DDR-15G-5, ~68 g; CUI VYB15W-Q24-S5 style). 500 mA–1 A fuse on the input,
  10 µF-class ceramics across input and output, output negative left floating,
  ferrite or a few turns through a small toroid on the output pair (the
  converter is a switcher and the ~1 nF barrier capacitance lets some HF
  common-mode through). Tap the pack where ESC power enters, not the Pi's BEC.
- Check which physical jack the aux input is (USB-C on recent units); make a
  short pigtail with only VBUS and GND connected.

### Common-mode choke on the USB cable

4–6 turns through a #31 or #43 mix toroid (FT140 size) at the SDR end — a few
kΩ at 148 MHz vs a few hundred Ω for a single clamp bead. Cheaper than the
HS02, does the same job less completely. A second choke where the USB exits
the box, with the wrap bonded to the USB shell there, makes the fabric the
boundary for the box.

### Others

- Shielded USB cable with shield continuous end to end (many are not).
- If the antenna is a monopole, give it a real counterpoise (radials / ground
  plane at the SDR) so the USB cable stops being the other half.
- Bonding the wrap properly (SMA-bulkhead style clamp) only matters if the
  SDR goes back inside the box, which is not the plan.
- USB isolation via fiber extender or ADuM416x-class isolator: overkill until
  the above is tried.

---

## Test protocol

Same 13 s hover capture, same SDR, same spot, same day, one change at a time:

1. **HS02 + power bank at the SDR end**, nothing else changed. First identify
   the antenna type: with a monopole the SDR case + USB cable is part of the
   counterpoise, so isolating also changes the antenna and neither outcome
   uniquely identifies the path. Fit a fixed counterpoise (or use the
   dipole/Yagi) before this step. Then: if the floor drops several dB, the USB
   path is the main one; build the isolated DC-DC and keep it. If it does not
   move, the noise is arriving through the antenna and the work is at the
   ESC/motor source.
2. **Ferrites off** (or on) with the same setup, to get the ferrite number
   that the April comparison cannot give.
3. Optionally **wrap off**: if the floor changes, the wrap is affecting
   coupling somehow (shielding, seams, parasitic counterpoise for the
   coax/USB) — this A/B does not identify which.

## Reproduce

```bash
cd ~/repos/MavlinkTagController2
D="/Volumes/Media/PDC Testing/Lake Front 9-16-26 - ESC Ferrites"
.venv/bin/python analyzer/psd_spectrum.py "$D/airspy-hf.1.dat" --png "$D/psd-powered.png"
.venv/bin/python analyzer/psd_spectrum.py "$D/airspy-hf.2.dat" --png "$D/psd-flying.png"
.venv/bin/python analyzer/psd_spectrum.py "$D/airspy-hf.3.dat" --png "$D/psd-flying-grounded.png"

# Detector-unit noise (for the April comparison)
.venv/bin/python - "$D" <<'EOF'
import sys, numpy as np
sys.path.insert(0, "analyzer"); sys.path.insert(0, "detector")
from iq_replay import FirDecimStage, estimate_noise
from pulse_detector import build_weighting_matrix, compute_stft_power
d = sys.argv[1]; fs_raw = 768000.0; fs = fs_raw / 200; shift = 10000.0
for n, name in ((1, "powered"), (2, "flying"), (3, "flying-grounded")):
    x = np.fromfile(f"{d}/airspy-hf.{n}.dat", dtype=np.complex64)
    y = x * np.exp(2j * np.pi * (shift / fs_raw) * np.arange(x.size))
    for st in (FirDecimStage(q) for q in (8, 5, 5)):
        y = st.process(y)
    n_w = int(np.ceil(0.015 * fs)); n_ol = n_w // 2
    W, _ = build_weighting_matrix(n_w, fs)
    power, _ = compute_stft_power(y.astype(np.complex64), n_w, n_ol, n_w, W=W)
    print(name, f"noise_psd median {np.median(estimate_noise(power)) / (fs * n_w):.3e}")
EOF
```

April detector noise values were pulled with:

```bash
grep -h "DETECTED" "/Volumes/Media/PDC Testing/April 2026/Apr-9 Afternoon 2 - No Wind"/*/py_detector_*.log
```
