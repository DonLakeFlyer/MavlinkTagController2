# IQ Simulator Tests

Unit tests for the IQ simulator (`simulator/iq_simulator.py`), covering pulse generation, rate-switch timing, ZMQ header encoding, and distance-based SNR scaling.

## Run

```bash
.venv/bin/python -m pytest simulator/tests/ -v
```

## Prerequisites

- Python 3 with numpy
- `conftest.py` adds `simulator/` to `sys.path`

## Tests

### `test_simulator.py`

#### TestPulseOnFixedRate

| Test | Description |
|------|-------------|
| `test_basic_pulse_count` | 10 s at tip=2.0 produces exactly 5 pulses |
| `test_pulse_width` | Pulse width matches tp (15 ms) within one sample |
| `test_phase_offset` | First pulse starts at the configured `phase_offset` |

#### TestPulseOnRateSwitch

| Test | Description |
|------|-------------|
| `test_pre_switch_matches_fixed` | Before `switch_time`, switched tag matches fixed-rate tag |
| `test_post_switch_uses_secondary_rate` | After switch, IPI matches `tip_secondary` |
| `test_anchor_continuity` | First post-switch pulse is `tip_secondary` after last pre-switch pulse |
| `test_anchor_on_boundary` | Switch exactly on a pulse boundary fires the boundary pulse and spaces the next by `tip_secondary` |
| `test_no_switch_fields_unchanged` | `tip_secondary=0` behaves identically to fixed rate |
| `test_switch_time_zero_is_fixed_rate` | `switch_time=0` with `tip_secondary` behaves as fixed rate |

#### TestEncodeHeader

| Test | Description |
|------|-------------|
| `test_size` | Encoded header is exactly `TTWF_ZMQ_IQ_HEADER_SIZE` bytes |
| `test_magic_and_version` | Header starts with correct magic, version, and header_size |
| `test_field_values` | All header fields (seq, timestamp, sample_rate, etc.) encode correctly |
| `test_round_trip` | Encode then struct-unpack recovers all fields |

#### TestSnrAtDistance

| Test | Description |
|------|-------------|
| `test_same_distance_no_loss` | Same distance → 0 dB path loss |
| `test_double_distance_6db_loss` | Doubling distance → ~6 dB loss (inverse-square) |
| `test_half_distance_6db_gain` | Halving distance → ~6 dB gain |
| `test_ten_x_distance_20db_loss` | 10× distance → 20 dB loss |
| `test_zero_distance_returns_ref` | Zero distance returns reference SNR |
| `test_negative_distance_returns_ref` | Negative distance returns reference SNR |
