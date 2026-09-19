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
| ESCs | Holybro Tekko32 F4 45A | ×4, one per arm; BLHeli_32, 32-bit F4 |
| Props | T-Motor CF FA15.2x5 | 15.2 in carbon fiber folding |
| Battery | Tattu 6S 4500 mAh LiPo | ×2 in parallel (9000 mAh, 22.2 V nominal) |

## Flight controller

| Item | Part | Notes |
| --- | --- | --- |
| FC | Holybro Pixhawk 6X | Mounted on the Holybro Pixhawk RPi CM4 Baseboard together with the companion CM4 |
| Firmware | PX4 v1.17 (stable) | |
| GPS / compass | | |
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
| UBEC DUO | Individually wrapped in Titan RF tape over an insulating layer, inside the avionics box | Bench 2026-09-19: the DUO was the only avionics component raising the SDR floor (+11 dB bare, +3 dB and two ~6 dB humps in the box). Wrap + input-pair ferrite (below) brings it within 0.5 dB of a battery-powered reference. |

## Ferrites

All ferrites on the vehicle are **Fair-Rite 0443178181**
([Digi-Key](https://www.digikey.com/en/products/detail/fair-rite-products-corp/0443178181/8594076)):
round-cable snap-on (clip-on) cores in **mix 43** (NiZn, suppression band
≈25 MHz–1 GHz, covers 146 MHz). Single pass only — the bore does not take a
second turn of the leads used here.

| Location | Notes |
| --- | --- |
| Matek CAN-L4-BM CAN output lead | Right at the module's CAN connector, on the lead to the Pixhawk 6X. Bench 2026-09-19: leaving this lead disconnected and un-ferrited raised the floor 12 dB. |
| Matek UBEC DUO input pair | Clip-on, single pass, at the exit of the DUO's Titan-tape wrap. |
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
| Receiver | Airspy HF+ Discovery | |
| Mounting | | |
| USB cable | | |

## Antenna

| Item | Part | Notes |
| --- | --- | --- |
| Antenna | | |
| Feed | | |
| Mounting | | |

## Ground station

| Item | Part | Notes |
| --- | --- | --- |
| GCS | TagTracker (QGroundControl custom build) | |
| Link | | |

## Tags

| Tag | Frequency | PRI / pulse | Notes |
| --- | --- | --- | --- |
| | | | |
