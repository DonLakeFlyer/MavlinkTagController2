# Known issues

Problems found in field data and the changes suggested for them. Living
document: items are added as they are found and removed when fixed (the fix
then lands in `design/`). Items 1–7 were fixed in 2026-10 and removed. Item 8
(in-flight floor regression since 10-01) was traced to DShot on 2026-10-14 and
the vehicle went back to PWM; see
[analysis/2026-10_DSHOT_VS_PWM_NOISE.md](analysis/2026-10_DSHOT_VS_PWM_NOISE.md).
The 10-14 pulse-pose problems (telemetry cache pruned before K=20 results
arrived; rotation pose taken at ARM, mid-turn) were fixed the same day; see
`design/RATE_SWITCH_DETECTOR.md`. Item 17 (sticky under-voltage bits not
read) was fixed 2026-11-07; see `design/SYSTEM_OVERVIEW.md`. Item 16
(post-flight report called an Airspy dropout "Healthy") was fixed 2026-11-07
in `analyzer/post_flight_analysis.py`.

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
defaulting on; see `design/COLLECTION_FLOW.md`) and removed. S16–S19 (receiver
exit on a lost Airspy, IQ-stall warnings, post-flight IQ-stop checks, sticky
throttle bits) were implemented 2026-11-07 and removed. S13 (vehicle time
in logs) was superseded by `ClockSync` and removed; issue 11 tracks its field
check. S15 (TagTracker unit test for an error after takeoff) was done
2026-11-07; see issue 15. S20 (settle the heading before arming a slice) was
implemented 2026-11-07; see issue 12. Numbers in
brackets are the issues below that each remaining change addresses.

### Field data collection

| # | Change | Addresses |
| --- | --- | --- |
| S9 | Archive rotation log folders with the raw captures in `/Volumes/Media/PDC Testing/` after every outing (done for 10-13 and 10-14). | — |

### RF noise / hardware tests

| # | Test | Addresses |
| --- | --- | --- |
| S11 | Discovery on the desktop: 50 Ω terminator at 147.980 MHz, then antenna at 147.900 MHz. | 9 |
| S12 | Office capture with only the GR01 power lead unplugged; if the bumps vanish, go beyond the existing 2-turn clamp-on (stacked cores, antenna-lead ferrites, filtered supply). | 10 |
| S14 | Replace the Airspy USB cable with a short, strain-relieved one; check both sockets; try another hub port if it continues. Bench test: `dmesg -w` while streaming, wiggle the cable, run motors (props off). | 13 |

### Controller / TagTracker / analyzer

None open.

---

## RF noise / hardware

### 9. Narrow humps at 147.705 and 147.995 MHz in the 10-13 office captures

≈15 kHz wide, ≈ −116 dBFS/Hz absolute, in the office on 10-13 both with the
vehicle off (Discovery on the desktop over coax) and with it powered. Absent
from every September capture at the same tune (9-13 office; 9-16, 9-17, 9-21
and 9-22 flights), with the same Discovery serial `0xC852AA80BE44463D`
throughout.

In flight they come with DShot: ≈ +1.5 dB on DShot (10-13 and 10-14), gone on
PWM (10-14, +0.0 to +0.3 dB on a −118 floor). If they were SDR-internal at the
office level they would still show ≈ +2 dB on PWM. The vehicle-off office
capture is still unexplained.

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

### 13. Airspy drops off USB in flight

Source: 2026-11-06 afternoon, Whizzy (tag 4, 146.788 MHz), 400 ft. Archived
in `PDC Testing Nov/Nov-6 Afternoon 1 - Whizzy Rotation` and
`Nov-6 Afternoon 2 - Whizzy Rotation` (with the 14:31 raw capture), each with
a `Whizzy_analysis.md`.

Both rotations that afternoon died mid-slice. The Pi kernel log (boot of
07:09–14:33 UTC) has `usb 1-1.2: USB disconnect` (Airspy HF+, on port 2 of the
baseboard's SMSC 2514 hub) at 14:28:40.603 and 14:30:58.843, within 1 s of the
IQ stopping in each session log. It reconnected each time within ≈0.3 s. From
14:32:37 to power-off at 14:33:34 there were ≈25 more disconnects, 12 ×
`error -71`, and 7 × `Cannot enable. Maybe the USB cable is bad?`. No kernel
under-voltage message in any boot, and `throttled=0x0` afterwards, so the Pi
supply was fine. The next boot had no disconnects. Points to the USB cable or
connector (vibration), not the Airspy or the Pi supply.

The raw capture between the failures (14:31:50, pointing at the dog, 3.18 km)
is clean and contains the collar at −289 Hz on its 2.0 s rate, but ≈9 dB
weaker than the Nov-4 rotation's facing headings at ≈2.5–2.8 km.

---

## Pipeline / TagTracker

### 14. The controller does not notice the Airspy stopping

When the Airspy drops off, `airspyhf_zeromq_rx` leaves its main loop
(`Exiting...`, not `User cancel`), `airspyhf_stop() failed` follows, and the
process then hangs in teardown without printing `done`. It is still running,
so `MonitoredProcess` reports nothing; the decimator just stops logging. The
only thing that ended the rotation was TagTracker's 10 s stall timeout, with
the generic message "Rotation stalled at slice N".

**Fixed 2026-11-07, not yet field-verified (S16, S17):** the receiver now
exits with status 1 when streaming stops without a user stop. Whether or not
it exits, a slice with no `SLICE_PROGRESS` for more than 5 s makes the
controller log `IQ_STALL`. Either one sends the operator
`#VHF stopped streaming. Check USB connection` (once per session), and the
decimator logs `IQ_STALL` after 5 s without packets. Remove this item once a
dropout in the field shows these.

### 15. Auto rotation failure after takeoff did not RTL

2026-11-06 14:24 UTC: an Auto rotation (Tracker takeoff) stalled at slice 6
(issue 13) and Tracker sent the cancel, but the vehicle held at ≈400 ft
instead of returning. `CustomStateMachine::setError` switches to RTL when the
vehicle is airborne and `RTLOnError` is set, which `autoDetection` does after
`takeoffComplete`; by reading the code this case should have returned.

2026-11-07: TagTracker `CustomStateMachineTest::_stallAfterTakeoffReturns`
runs that path (PX4 MockLink airborne, `RTLOnError` as after takeoff, a
`START_COLLECTION` operation stalling in `PythonWaitForDetectionResultState`)
and the vehicle does switch to Return, so the cause is outside it. Candidates:
- `Vehicle::airborne()` was false. It comes only from `EXTENDED_SYS_STATE`
  `landed_state`; if that message was not reaching Tracker, `setError` skips
  the RTL silently (voice: "Auto Detection failed", with neither "Returning"
  nor "User is in control of vehicle").
- The machine had already been cancelled (pilot mode change or Pause), so the
  stall error was ignored (voice: "cancelled. User is in control of vehicle").
- PX4 rejected the mode change.

Tracker's log or the voice announcement would tell these apart.

---

## Logging / observability

### 11. RPi clock is not vehicle time; logs cannot be lined up with the ULog

**Fix implemented 2026-10-15, revised 2026-11-05, not yet field-verified:** the
controller steps the Pi clock to GPS time (`ClockSync`, see
[design/SYSTEM_OVERVIEW.md](design/SYSTEM_OVERVIEW.md#time-base)). The
growing offset below is the powered-off time of each battery swap.

2026-11-04 (Whizzy rotation): the one-step-per-run version stepped −4188 s,
and the session log then showed the Pi 4 h 46 m behind (`CLOCK_SYNC session
start: GPS offset +17180.845 s; stepped -4187.685 s earlier this run`). The
likely cause, inferred from the PX4 1.17 GPS driver and not yet confirmed, is
that PX4 reported a 3D fix while `SYSTEM_TIME` was still a stale Pixhawk clock
and set that clock from GPS only later. The Pi's main controller log (the
`CLOCK_SYNC stepped … (A -> B UTC)` line) or the flight's ULog would confirm
it; neither was in the session folder. `ClockSync` now steps again whenever
the clock is off by > 1 s outside a session. Remove this item after a flight
whose session logs show `GPS offset` within ±1 s and line up with the ULog.

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
single correction per outing is not enough. On 10-14 it grew from 21 min at
the first capture to ≈27 min by the last rotation. Lining detector events up with
the ULog (attitude, altitude, motors), or with what people at the collar did,
currently needs dwell-by-dwell manual mapping.

Example: the rotation-1 rate switch happened in the 045° dwell, which is
20:14:32–20:15:12 by the RPi clock but **20:36:53–20:37:32 UTC (13:37 local)**
by GPS.

### 12. TagTracker arms each slice before the turn has settled

Source: 60-acres, 2026-10-14, both rotations, ULog yaw at each ARM. The GCS
sends `START_COLLECTION_SLICE` while the vehicle is still turning; yaw then
overshoots by up to ≈8° and settles within ≈1.5–2 s. The segment starts
≈1 s after ARM, so the first ≈1 s of each dwell is up to ≈4° off heading.
Effect here ≈0.1 dB per heading (mean dwell yaw within 0.1° of commanded);
it grows with faster yaw rates or fewer pulses per dwell. Fix belongs in
TagTracker: arm once yaw error and yaw rate have both been small for ≈1 s.

**Fixed 2026-11-07 in TagTracker, not yet field-verified (S20):** after the
heading first comes within 1°, `HeadingSettleState` waits for it to stay within
±3° for 1 s before `START_COLLECTION_SLICE`. It gives up after 4 s and arms
anyway (warning `Heading did not settle`), so gusts cost a less steady slice,
never the rotation. Remove this item once a flight's ULog shows yaw within ±3°
at each ARM.
