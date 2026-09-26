# PDC VHF tag-tracking drone — hardware

Reference for the airframe and avionics the pipeline flies on, built for the
Painted Dog Conservation (PDC) team to track VHF-collared African wild dogs.
Read this before any noise / RF / flight-test discussion instead of
re-deriving the setup. Kept current; not dated.

## Airframe

| Item | Part | Notes |
| --- | --- | --- |
| Frame | ReadyToSky ZD680 ([product page](https://readytosky.com/e_productshow/?1252-ZD680-680mm-Carbon-fiber-Quadcopter-Frame-FPV-Quad-with-Carbon-Fiber-Landing-Skid-1252.html)) | 680 mm wheelbase quad, 3K carbon fiber plates (3 mm bottom plate), carbon landing skid |
| Motors | T-Motor 4014 330KV | ×4 |
| ESCs | Holybro Tekko32 F4 45A | ×4, one per arm; BLHeli_32, 32-bit F4; driven by 100 Hz PWM from the FC (`PWM_MAIN_TIM0 = 100`). Each fitted with the Holybro-supplied 330 µF 35 V electrolytic across the battery input (recommended for 6S). Hover draw ≈19 A total (≈4.7 A/motor) from the 2026-09-21/22 flight logs. Bench 2026-09-21: the in-flight noise floor is the ESCs' PWM chopping at hover duty; a single-pass mix-43 clip-on around the phase wires at the ESC made no difference (0 dB at 50 % throttle). |
| Props | T-Motor CF FA15.2x5 | 15.2 in carbon fiber folding |
| Battery | Tattu 6S 4500 mAh LiPo | ×2 in parallel (9000 mAh, 22.2 V nominal) |

## Flight controller

| Item | Part | Notes |
| --- | --- | --- |
| FC | Holybro Pixhawk 6X | Mounted on the Holybro Pixhawk RPi CM4 Baseboard together with the companion CM4 |
| Firmware | PX4 v1.17 (stable) | |
| GPS / compass | Holybro M9N GPS ([product page](https://holybro.com/collections/standard-gps-module/products/m9n-m10-gps-v2)) | u-blox M9N; module includes compass |
| RC link | | |
| Telemetry | | |

## Power distribution

```
2× Tattu 6S (parallel) → Matek CAN-L4-BM → PDB → 4× Tekko32 ESCs
                                             └→ Matek UBEC DUO ─┬→ CM4 (RPi USB power port)
                                                                └→ Pixhawk 6X (baseboard POWER 1)
```

| Rail | Source | Feeds |
| --- | --- | --- |
| Battery (6S) | Matek CAN-L4-BM battery monitor | Reports voltage/current to the autopilot over DroneCAN; passes battery power to the PDB |
| Battery (6S) | PDB | 4× Tekko32 ESCs; Matek UBEC DUO input |
| 5 V (UBEC out A) | Matek Systems UBEC DUO | CM4 / RPi via the baseboard's USB power port |
| 5 V (UBEC out B) | Matek Systems UBEC DUO | Pixhawk 6X via the baseboard POWER 1 connector |

## Shielding

| What | How | Notes |
| --- | --- | --- |
| Avionics box | 3D-printed box wrapped in Titan RF fabric | Contains the Pixhawk 6X + CM4 baseboard and the UBEC DUO. Outside the box: CAN-L4-BM, PDB, ESCs, SDR. Bench A/B 2026-09-19: 7.4 dB. |
| UBEC DUO | Individually wrapped in Titan RF tape over an insulating layer, inside the avionics box | Bench 2026-09-19: the DUO was the only avionics component raising the SDR floor (+11 dB bare, +3 dB and two ~6 dB humps in the box). Bench 2026-09-20 (SDR captured through the RPi): with the wrap, input and output ferrites below and the USB cable cores, its harmonics no longer appear above the floor. |

## Ferrites

Ferrites on the vehicle are Fair-Rite round-cable snap-on (clip-on) cores in
**mix 43** (NiZn, suppression band ≈25–300 MHz, covers 146 MHz), in two
sizes:

- **0443178181** ([Digi-Key](https://www.digikey.com/en/products/detail/fair-rite-products-corp/0443178181/8594076))
  on the power leads. Single pass only — the bore does not take a second turn.
- **0443164251** ([Digi-Key](https://www.digikey.com/en/products/detail/fair-rite-products-corp/0443164251/8594062))
  on the USB cable. 6.6 mm bore, 32 mm long; takes two turns of the cable.

| Location | Notes |
| --- | --- |
| Matek CAN-L4-BM CAN output lead | Right at the module's CAN connector, on the lead to the Pixhawk 6X. Bench 2026-09-19: leaving this lead disconnected and un-ferrited raised the floor 12 dB. |
| Matek UBEC DUO input pair | Clip-on, single pass, at the exit of the DUO's Titan-tape wrap. |
| Matek UBEC DUO output pairs (both) | Clip-on, single pass, at the wrap exit. Bench 2026-09-20: took 4 dB off the flat floor when the SDR is captured through the RPi. |
| USB cable, Discovery end | Two stacked cores, two turns of the cable through each. Bench 2026-09-20: the DUO's residual humps reach the SDR as common-mode on the USB cable; one 2-turn core cut them 9 → 3 dB, the second to ~2 dB. |
| USB cable, RPi end | One core, two turns. Bench 2026-09-20: removed the last ~2 dB of hump. |
| ESC power lines | At the exit of each motor arm (added for the 2026-09-16 lake-front flights). |

## Companion computer

| Item | Part | Notes |
| --- | --- | --- |
| Computer | Raspberry Pi Compute Module 4 | On the Holybro Pixhawk RPi CM4 Baseboard (shared carrier with the Pixhawk 6X) |
| Enclosure | Avionics box | See [Shielding](#shielding). |
| FC link | Baseboard internal UART (Pixhawk TELEM2 ↔ CM4) | MAVLink |

## SDR

| Item | Part | Notes |
| --- | --- | --- |
| Receiver | Airspy HF+ Discovery ([product page](https://airspy.com/airspy-hf-discovery/)) | 0.5 kHz–31 MHz HF, 60–260 MHz VHF; covers 146 MHz |
| Mounting | Directly on the antenna BNC connector | No coax between antenna and SDR; the SDR sits at the antenna on the landing legs. The SDR body and USB cable are therefore the unbalanced side of the feed — the USB cable cores at the SDR end act as the choke balun. |
| USB cable | SDR → CM4 (baseboard USB) | Runs from the landing legs up to the avionics box. Mix-43 cores at both ends, see [Ferrites](#ferrites). Bench 2026-09-20: the cable is the path by which the UBEC DUO's switching harmonics reached the SDR. |

## Antenna

| Item | Part | Notes |
| --- | --- | --- |
| Antenna | Telonics RA-2AHS or Telonics RA-23K | Both 2-element "H" antennas (driven dipole + reflector); both under test, one flown at a time |
| Feed | None | Airspy HF+ Discovery connects directly to the antenna BNC |
| Mounting | Landing legs, with 26 cm wooden leg extensions | Boom horizontal, element centre ≈22 in (56 cm) below the motor plane, so ≈65 cm from each motor. The frame sits at ≈90° to boresight in the H-plane (little pattern rejection) and the horizontal phase wires are co-polarised with the elements. Both the Zimbabwe vehicle and the test vehicle here have the extensions; without them the antenna was ≈30 cm below the motor plane (≈45 cm from each motor, ~3 dB more motor pickup). |

## Ground station

| Item | Part | Notes |
| --- | --- | --- |
| GCS | TagTracker (QGroundControl custom build) | |
| Link | | |

## Tags

| Tag | Frequency | PRI / pulse | Notes |
| --- | --- | --- | --- |
| | | | |
