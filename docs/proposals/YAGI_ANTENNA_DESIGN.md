# 3-Element Yagi Antenna Design — 146–147 MHz Drone Telemetry

**Status: open (hardware).** Not built. The RHCP variant this document
originally proposed for multipath rejection was removed on 2026-09-12: at the
1.4° grazing angle of a 5 km / 400 ft geometry the ground reflection keeps the
same circular-polarisation sense, so an RHCP feed rejects nothing and costs 3 dB
(see [2026-09_MULTIPATH_ANALYSIS_REVIEW.md](../analysis/2026-09_MULTIPATH_ANALYSIS_REVIEW.md)).
The case for a Yagi rests on gain and beamwidth only. Adopting it requires a
new pattern LUT in TagTracker (see below).

## Why a 3-Element Yagi

The RA-2AK is a broad-pattern antenna well-suited to handheld surveys where the operator sweeps slowly. For a drone platform doing an 8-point rotation sweep with LM pattern fitting, the antenna beamwidth is the primary limit on bearing resolution — the LM fit has more angular gradient to work with as the beam narrows. A 3-element Yagi offers a practical balance:

| Antenna | Gain (dBd) | 3dB Beamwidth | Boom Length | Verdict |
|---|---|---|---|---|
| RA-2AK (current) | 4 (Telonics published) | ~120° | — | Broad, limits bearing resolution |
| 2-element Yagi | ~5 | ~90° | ~35 cm | Modest improvement, short boom |
| **3-element Yagi** | **~7.5** | **~65°** | **~82 cm** | **Best balance for 680 class** |
| 4-element Yagi | ~9 | ~55° | ~130 cm | Diminishing return, too long |
| 5-element Yagi | ~10 | ~50° | ~200 cm | Not viable on 680 class |

The 3-element is the inflection point: the jump from RA-2AK to 3-element Yagi is ~3 dB of gain and roughly half the beamwidth. The jump from 3-element to 4-element is much smaller and comes at significant cost in boom length and drag.

At 5+ km the two-ray ground loss is a fixed ~20 dB penalty on top of free-space
loss, so both the ~3 dB of extra gain over the RA-2AK's published 4 dBd
(≈ +19 % range on the two-ray slope; see
[2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md](../analysis/2026-09_DETECTOR_AMPLITUDE_ANALYSIS.md))
and the sharper lobe matter: the gain buys link margin, and the Yagi's steeper
gradient near boresight gives the LM fit more to work with for bearing accuracy.

---

## Off-the-Shelf Option: Arrow Antenna 146-3

The **Arrow Antenna 146-3** is a direct match to the design spec and does not need to be built from scratch.

**Purchase links:**
- [Arrow Antenna direct (146-3ii)](http://www.arrowantennas.com/arrowii/146-3ii.html) — ~USD $114
- [GigaParts](https://www.gigaparts.com/arrow-antennas-146-3.html) — ~USD $114, in stock, free shipping
- [Amazon (146-3)](https://www.amazon.com/Arrow-Handheld-Portable-Antenna-146-3/dp/B00TGE64KG) — ~USD $114

**What you get:**
- 3-element Yagi tuned specifically to 146 MHz (not just band coverage)
- 3/4" T6061 aluminium boom
- Easton aluminium arrow shafts for elements (lightweight, stiff)
- Pre-tuned gamma match at the feedpoint — no tuning required
- SWR 1.2:1 at 146 MHz
- Foam handle grip that removes cleanly for drone mounting
- Compatible with the Arrow II Mounting Bracket (sold separately) for mast/boom attachment

**Adapter required:** The Arrow 146-3 has a **BNC connector only**. The Airspy HF+ uses SMA. A BNC-female to SMA-male adapter is needed — use a quality silver-plated one, not a cheap brass adapter, to avoid additional loss at this frequency.

**Drone mount:** Remove the foam grip and fabricate a simple bracket from aluminium flat stock to clamp the 3/4" boom to your drone frame standoff. The Arrow II Mounting Bracket can also be used if it suits your standoff geometry.

---

## Design Dimensions — 146.5 MHz Centre Frequency

Element diameter: **9.5 mm (3/8") 6061-T6 aluminium tube**

At this diameter, the element length correction for end-effect is small but included below.

### Element Lengths

| Element | Length (mm) | Half-length each side (mm) |
|---|---|---|
| Reflector | 1026 | 513 |
| Driven element | 965 | 483 |
| Director | 910 | 455 |

### Boom Spacings (measured from reflector)

| Element | Position on boom |
|---|---|
| Reflector | 0 mm |
| Driven element | 410 mm |
| Director | 820 mm |

**Total boom length: 820 mm**

### Feedpoint Impedance

A split-dipole driven element at this spacing presents approximately **28–35 ohms**. Two matching options are practical for field use:

**Gamma match (recommended for bush conditions):** Directly matches to 50-ohm coax. No balun required. Mechanically simple — a short aluminium rod and a small series capacitor. Robust and adjustable. Set the gamma rod to approximately 120 mm length and 12 mm spacing from the driven element; tune the series capacitor (start at 7–10 pF) for minimum SWR.

**Folded dipole driven element:** Raises feedpoint impedance to ~120–140 ohms. With a λ/4 coaxial impedance transformer (75-ohm coax cut to 340 mm electrical length), presents ~50 ohms. More complex construction than the gamma match but no moving parts and no capacitor to drift.

Avoid direct-connection 50-ohm matching without a balun — common-mode currents on the coax will distort the pattern and corrupt the bearing measurement.

---

## Materials and Construction for African Bush

**Boom:** 20 mm square or round 6061-T6 aluminium extrusion. Square section prevents element rotation without requiring clamps. Minimum wall thickness 2 mm.

**Elements:** 9.5 mm OD 6061-T6 round tube. Each element passes through a hole in the boom and is secured with a nylon set-screw on each side. **Electrically isolate the driven element from the boom** (nylon or HDPE sleeve through the boom hole). Reflector and director can be electrically bonded to boom or isolated — electrically bonded simplifies construction and has negligible pattern effect.

**Corrosion:** Aluminium performs well in the bush environment. Anodising is useful but not essential. All fasteners should be stainless steel or aluminium. Avoid dissimilar metal contact (e.g. steel bolts into aluminium threads) without an isolating washer.

**Feedline:** LMR-195 or Belden 8240 from feedpoint to drone frame (short, 200–300 mm). Then LMR-240 from frame to Airspy HF+. Keep the feedpoint coax transition away from the driven element to avoid pattern distortion. A small PL-259/SMA adapter mount potted in epoxy makes a durable feedpoint junction.

**Weight estimate:**
- Boom (820 mm, 20mm square, 2mm wall): ~95 g
- Three elements (9.5mm tube): ~75 g
- Feedpoint hardware, clamps, coax stub: ~40 g
- **Total: ~210 g**

---

## Mounting on the 680 Frame

**Position:** Below the frame on the centreline, boom aligned fore-and-aft. This puts the main lobe pointing forward in the drone's reference frame and keeps elements clear of props.

**Standoff:** Minimum 80 mm below the frame. Carbon fibre frame members detune the antenna if they pass through the near field. Test with VNA before field deployment — a carbon frame at 80 mm will shift resonance measurably.

**Fore-aft alignment:** The driven element should be forward of the drone's centre of mass so that the main lobe direction (forward) corresponds to the drone's nose heading as reported by the flight controller. Verify the bearing offset between antenna boresight and FC compass heading during initial calibration and apply as a fixed offset in `RotateAndCaptureStateBase`.

**Downward tilt:** A 1–2° nose-down tilt on the boom angles the main lobe toward ground-level collars at distance. At 400 ft AGL / 5 km, the geometric depression angle is 1.4°, so this is a reasonable match. Tilt is easily achieved with a wedge-shaped mount.

**Drag:** At typical survey airspeeds the 820 mm boom presents significant drag asymmetry. Fly in hover or slow creep during detection cycles. If ferrying between hover points at speed, orient the antenna axis fore-and-aft (which it already is) to minimise frontal area.

---

## Pattern Characteristics and TagTracker Integration

### Expected pattern at 146.5 MHz (linear polarisation)

| Parameter | Value |
|---|---|
| Forward gain | ~7.5 dBd |
| 3 dB beamwidth (E-plane) | ~65° |
| Front-to-back ratio | ~10–12 dB |
| First sidelobe level | ~–13 dB |

### Pattern LUT update required

`RotationInfo.cc` in TagTracker contains a hardcoded empirical pattern LUT for the RA-2AK antenna sampled at 10° intervals. **This must be replaced before flying the Yagi.** Using the wrong pattern in the LM fit will produce systematically biased bearings.

**Repository note:** `RotationInfo.cc` and related classes such as `RotateAndCaptureStateBase` are part of the separate TagTracker application codebase (QGroundControl fork); they are not in this monorepo.

The Yagi pattern is well-approximated theoretically but the mounted pattern on the 680 frame will differ due to frame and prop reflections. The recommended process:

1. Use the theoretical pattern (tabulated below) as a starting point for initial testing
2. Mount the antenna on the 680 and measure the actual pattern against a fixed transmitter at known bearing while rotating the drone in a large open area
3. Sample at 10° intervals (or 5° for better LUT resolution), record SNR at each heading
4. Fit a smooth curve (spline) to the measurements, re-sample at 10° steps, replace the LUT

### Theoretical 3-element Yagi pattern (linear, E-plane, 10° steps, normalised to 0 dB at boresight)

| Angle (°) | Relative level (dB) |
|---|---|
| 0 | 0.0 |
| 10 | −0.3 |
| 20 | −1.2 |
| 30 | −2.9 |
| 40 | −5.2 |
| 50 | −8.1 |
| 60 | −11.6 |
| 70 | −15.3 |
| 80 | −18.5 |
| 90 | −20.4 |
| 100 | −19.8 |
| 110 | −17.5 |
| 120 | −14.9 |
| 130 | −12.8 |
| 140 | −11.6 |
| 150 | −11.2 |
| 160 | −11.5 |
| 170 | −12.1 |
| 180 | −12.4 |

Pattern is symmetric: angle 190° = value at 170°, 200° = 160°, etc.

Note these are free-space values. The mounted pattern will differ, particularly in the rear quadrant where frame reflections have the most effect. Empirical characterisation is essential before operational use.

---

## Verification Before Field Deployment

1. **VNA check:** Measure SWR at 146–147 MHz after mounting on drone (motors off). Target SWR < 1.5:1 across the band. Retune gamma match if needed.
2. **Pattern check:** Rotate drone against fixed transmitter at 50+ m, confirm peak and null positions match expectations.
3. **Bearing offset calibration:** Confirm boresight heading matches FC compass heading; apply fixed correction in TagTracker if not.
4. **LUT update in TagTracker:** Do not fly with the RA-2AK LUT still active.
