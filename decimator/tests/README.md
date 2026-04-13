# Decimator Tests

Unit and integration tests for the `airspyhf_decimator` — the ZMQ→UDP IQ decimation pipeline (8×5×5 = 200× decimation from 768 kHz to 3840 Hz).

## Run

```bash
cd build && ctest -R decimator
```

## Tests

### `test_main.cpp`

Includes the decimator `main.cpp` via `#define main` redirection so all internal functions are testable without modification.

#### CLI Argument Parsing

| Test | Description |
|------|-------------|
| `testParseArgsDefaults` | Default CLI args produce expected option values |
| `testParseArgsCustom` | Custom CLI args (rate, frame, ports, etc.) are parsed correctly |
| `testParseArgsValidation` | Negative input rate, port >65535, and port 0 are rejected |

#### DSP Components

| Test | Description |
|------|-------------|
| `testDesignLowpassNormalization` | FIR lowpass coefficients have odd length and sum to 1.0 |
| `testConvertToComplexLittleEndian` | Little-endian byte buffer is correctly decoded to complex samples |
| `testFrequencyShifterZeroShiftNoop` | Zero-Hz frequency shift leaves samples unchanged |
| `testFrequencyShifterSignConvention` | Positive shift moves tone up, negative shift moves tone down |

#### ZMQ Frame Handling

| Test | Description |
|------|-------------|
| `testParseZmqFrameValid` | A well-formed ZMQ frame parses with correct field values |
| `testParseZmqFrameMalformed` | Bad magic, bad version, short header, and payload-size mismatch are rejected |
| `testZmqReceiverMalformedFrameAccounting` | Malformed frames increment the malformed-packet counter |

#### Decimation Pipeline

| Test | Description |
|------|-------------|
| `testFirDecimatorOutputCount` | Decimate-by-4 on 20 samples produces exactly 5 outputs |
| `testTimestampEncoderMonotonicStep` | Timestamp delta for 1000 samples at 1 kHz equals ~1 second |
| `testTimestampMatchesUavrtDetectionFormat` | Encoded timestamps have nanoseconds <1e9 and correct step sizes |
| `testPulseSurvivesShiftAndDecimation` | Clean pulsed tone at 146 MHz survives 3-stage 8×5×5 decimation with correct pulse spacing |
| `testNoisyPulseSurvivesShiftAndDecimation` | Same as above with additive Gaussian noise; pulse contrast remains above background |
