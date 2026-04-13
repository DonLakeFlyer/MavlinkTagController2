# Controller Tests

Unit tests for the controller's `BearingCalculator` — the Levenberg-Marquardt curve fitter that estimates transmitter bearing from per-heading SNR slices using the measured RA-2AK antenna pattern.

## Run

```bash
cd build && ctest -R bearing
```

## Tests

### `test_bearing_calculator.cpp`

Uses synthetic SNR data generated from the real RA-2AK antenna pattern LUT with the model `SNR(θ) = A · pattern_linear(θ − φ) + B`.

| Test | Description |
|------|-------------|
| `testEmpty` | Empty calculator returns no results |
| `testSingleSlice` | Single SNR slice returns that heading with R²=0 |
| `testTooFewSlices` | Fewer than 3 slices falls back to best-heading instead of curve fit |
| `testReset` | `reset()` clears all accumulated state |
| `testBestSnr` | `best_snr` field reports the highest SNR across slices |
| `testPatternSymmetry` | RA-2AK pattern is symmetric, boresight=1.0, back-lobe≈−10 dB, null near 90° |
| `testBearingAtZero_8slices` | Recovers 0° bearing from 8 synthetic slices within 5° |
| `testBearingAt90_8slices` | Recovers 90° bearing from 8 slices |
| `testBearingAt225_8slices` | Recovers 225° bearing from 8 slices |
| `testBearingWraparound_8slices` | Recovers 350° bearing (360° wraparound) within 10° |
| `testBearingOffGrid_8slices` | Recovers 22° bearing (between 45° grid points) within 10° |
| `testMultipleTags` | Solves bearings for two tags (45° and 270°) simultaneously |
| `testNoisyData` | Bearing at 135° with ±2 dB noise converges within 15° |
| `testBearing_16slices` | Recovers 160° bearing from 16 slices with R²>0.95 |
