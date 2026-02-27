#!/usr/bin/env python3
"""
IQ Signal Simulator — drop-in replacement for airspyhf_zeromq_rx.

Publishes synthetic IQ data over ZeroMQ PUB using the tagtracker wire format,
feeding the decimator → pulse_detector pipeline with configurable:

  - Noise floor (thermal noise power)
  - Pulsed CW tag signals at configurable frequency offset, SNR, tp (pulse
    width), and tip (inter-pulse interval)
  - Distance-based path loss (free-space model at VHF)
  - Multiple simultaneous tags
  - Packet gaps / dropouts for robustness testing

Usage:
  source ../.venv/bin/activate
  python iq_simulator.py --preset strong
  python iq_simulator.py --snr 6 --freq-offset-hz -200 --tp 0.02 --tip 1.5

Wire format: shared/tagtracker_wireformat/zmq_iq_packet.h
  40-byte LE header + interleaved float32 IQ payload.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List

import numpy as np
import zmq

# ---------------------------------------------------------------------------
# Wire-format constants (must match zmq_iq_packet.h)
# ---------------------------------------------------------------------------
TTWF_ZMQ_IQ_MAGIC = 0x5a514941
TTWF_ZMQ_IQ_VERSION = 1
TTWF_ZMQ_IQ_HEADER_SIZE = 40
TTWF_ZMQ_IQ_BYTES_PER_COMPLEX_SAMPLE = 8

# Header struct: < = little-endian
#   magic        uint32  (I)
#   version      uint16  (H)
#   header_size  uint16  (H)
#   sequence     uint64  (Q)
#   timestamp_us uint64  (Q)
#   sample_rate  uint32  (I)
#   sample_count uint32  (I)
#   payload_bytes uint32 (I)
#   flags        uint32  (I)
HEADER_FMT = "<IHHQQIIII"
assert struct.calcsize(HEADER_FMT) == TTWF_ZMQ_IQ_HEADER_SIZE


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------
@dataclass
class TagSignal:
    """A pulsed CW tag transmitter."""
    freq_offset_hz: float = 0.0       # Hz offset from tuned center frequency
    snr_db: float = 20.0              # SNR in dB (signal peak power / noise power)
    tp: float = 0.015                 # Pulse width in seconds
    tip: float = 2.0                  # Inter-pulse interval in seconds
    phase_offset: float = 0.0         # Initial pulse phase offset in seconds

    def pulse_on(self, t: np.ndarray) -> np.ndarray:
        """Return a boolean mask: True where the pulse is active."""
        # Time within the current pulse period
        t_mod = (t - self.phase_offset) % self.tip
        return t_mod < self.tp


@dataclass
class SimConfig:
    """Complete simulation configuration."""
    sample_rate: int = 768000          # Hz — must match real hardware
    zmq_host: str = "127.0.0.1"
    zmq_port: int = 5555
    samples_per_packet: int = 4096    # Complex samples per ZMQ packet
    noise_power_dbfs: float = -40.0   # Noise floor in dBFS
    tags: List[TagSignal] = field(default_factory=list)
    duration: float = 0.0             # Seconds to run (0 = indefinite)
    realtime: bool = True             # Pace packets to match sample rate
    drop_probability: float = 0.0     # Probability of dropping a packet (0..1)
    gap_seconds: float = 0.0          # Inject a gap of this duration every gap_interval
    gap_interval: float = 0.0         # Seconds between gap injections (0 = disabled)
    seed: int | None = None           # RNG seed for reproducibility
    telemetry_sub_endpoint: str = "tcp://127.0.0.1:6001"  # ZMQ SUB endpoint for controller telemetry
    telemetry_topic: str = "vehicle_pose"
    tx_offset_north_m: float = 4000.0
    ra2ahs_hpbw_deg: float = 100.0
    ra2ahs_front_to_back_db: float = 12.0


@dataclass
class DirectionalTelemetryState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    has_pose: bool = False
    vehicle_lat_deg: float = 0.0
    vehicle_lon_deg: float = 0.0
    vehicle_yaw_deg: float = 0.0
    tx_lat_deg: float = 0.0
    tx_lon_deg: float = 0.0
    tx_initialized: bool = False


# ---------------------------------------------------------------------------
# Presets
# ---------------------------------------------------------------------------
PRESETS = {
    "strong": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[TagSignal(freq_offset_hz=0.0, snr_db=25.0, tp=0.015, tip=2.0)],
    ),
    "weak": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[TagSignal(freq_offset_hz=0.0, snr_db=6.0, tp=0.015, tip=2.0)],
    ),
    "noise-only": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[],
    ),
    "two-tags": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[
            TagSignal(freq_offset_hz=0.0, snr_db=20.0, tp=0.015, tip=2.0, phase_offset=0.0),
            TagSignal(freq_offset_hz=800.0, snr_db=15.0, tp=0.020, tip=3.0, phase_offset=0.5),
        ],
    ),
    "distant": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[TagSignal(freq_offset_hz=0.0, snr_db=3.0, tp=0.015, tip=2.0)],
    ),
    "dropout": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[TagSignal(freq_offset_hz=0.0, snr_db=20.0, tp=0.015, tip=2.0)],
        drop_probability=0.05,
    ),
    "gap": SimConfig(
        noise_power_dbfs=-40.0,
        tags=[TagSignal(freq_offset_hz=0.0, snr_db=20.0, tp=0.015, tip=2.0)],
        gap_seconds=0.1,
        gap_interval=10.0,
    ),
}


def _normalize_angle_deg(angle: float) -> float:
    return ((angle + 180.0) % 360.0) - 180.0


def _offset_latlon_north_east(lat_deg: float, lon_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    meters_per_deg_lat = 111320.0
    lat_rad = math.radians(lat_deg)
    meters_per_deg_lon = max(1e-6, 111320.0 * math.cos(lat_rad))
    dlat = north_m / meters_per_deg_lat
    dlon = east_m / meters_per_deg_lon
    return lat_deg + dlat, lon_deg + dlon


def _haversine_distance_m(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    r = 6371000.0
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    return r * c


def _initial_bearing_deg(lat1_deg: float, lon1_deg: float, lat2_deg: float, lon2_deg: float) -> float:
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360.0) % 360.0


def _ra2ahs_relative_gain_db(off_boresight_deg: float, hpbw_deg: float, front_to_back_db: float) -> float:
    off = min(180.0, abs(_normalize_angle_deg(off_boresight_deg)))
    half_power_angle = max(1.0, hpbw_deg / 2.0)
    cos_at_half = max(1e-6, math.cos(math.radians(half_power_angle)))
    exponent = math.log(0.5) / math.log(cos_at_half)
    front_lobe = max(0.0, math.cos(math.radians(off))) ** exponent
    back_floor = 10.0 ** (-abs(front_to_back_db) / 10.0)
    gain_linear = back_floor + (1.0 - back_floor) * front_lobe
    return 10.0 * math.log10(max(1e-9, gain_linear))


def _start_telemetry_subscriber(cfg: SimConfig, telem_state: DirectionalTelemetryState) -> tuple[threading.Event, threading.Thread] | tuple[None, None]:
    if not cfg.telemetry_sub_endpoint:
        return None, None

    stop_event = threading.Event()

    def _run() -> None:
        ctx = zmq.Context.instance()
        sub = ctx.socket(zmq.SUB)
        sub.setsockopt(zmq.RCVHWM, 16)
        sub.connect(cfg.telemetry_sub_endpoint)
        sub.setsockopt_string(zmq.SUBSCRIBE, cfg.telemetry_topic)

        print(
            f"iq_simulator: subscribed to telemetry at {cfg.telemetry_sub_endpoint} topic={cfg.telemetry_topic}",
            file=sys.stderr,
        )

        try:
            while not stop_event.is_set():
                try:
                    msg = sub.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.02)
                    continue

                if not msg.startswith(cfg.telemetry_topic):
                    continue
                payload = msg[len(cfg.telemetry_topic):].strip()
                if not payload:
                    continue

                try:
                    obj = json.loads(payload)
                    lat = float(obj["lat_deg"])
                    lon = float(obj["lon_deg"])
                    yaw = float(obj["yaw_deg"])
                except Exception:
                    continue

                with telem_state.lock:
                    telem_state.vehicle_lat_deg = lat
                    telem_state.vehicle_lon_deg = lon
                    telem_state.vehicle_yaw_deg = yaw
                    telem_state.has_pose = True

                    if not telem_state.tx_initialized:
                        tx_lat, tx_lon = _offset_latlon_north_east(lat, lon, cfg.tx_offset_north_m, 0.0)
                        telem_state.tx_lat_deg = tx_lat
                        telem_state.tx_lon_deg = tx_lon
                        telem_state.tx_initialized = True
                        print(
                            "iq_simulator: initialized transmitter location "
                            f"{cfg.tx_offset_north_m:.0f}m north of first vehicle pose",
                            file=sys.stderr,
                        )
        finally:
            sub.close(linger=0)

    th = threading.Thread(target=_run, daemon=True)
    th.start()
    return stop_event, th


# ---------------------------------------------------------------------------
# Path-loss helper
# ---------------------------------------------------------------------------
def snr_at_distance(
    snr_ref_db: float,
    distance_m: float,
    ref_distance_m: float = 100.0,
) -> float:
    """
    Free-space path loss model.

    Given a reference SNR at ref_distance_m, compute the SNR at distance_m.
    Uses the inverse-square law: SNR drops by 20·log10(d/d_ref).
    """
    if distance_m <= 0 or ref_distance_m <= 0:
        return snr_ref_db
    loss_db = 20.0 * math.log10(distance_m / ref_distance_m)
    return snr_ref_db - loss_db


# ---------------------------------------------------------------------------
# Packet encoder
# ---------------------------------------------------------------------------
def encode_header(
    sequence: int,
    timestamp_us: int,
    sample_rate: int,
    sample_count: int,
    payload_bytes: int,
    flags: int = 0,
) -> bytes:
    """Encode a ttwf_zmq_iq_packet_header_t into 40 bytes (little-endian)."""
    return struct.pack(
        HEADER_FMT,
        TTWF_ZMQ_IQ_MAGIC,
        TTWF_ZMQ_IQ_VERSION,
        TTWF_ZMQ_IQ_HEADER_SIZE,
        sequence,
        timestamp_us,
        sample_rate,
        sample_count,
        payload_bytes,
        flags,
    )


# ---------------------------------------------------------------------------
# Signal generation
# ---------------------------------------------------------------------------
def generate_packet(
    cfg: SimConfig,
    sample_offset: int,
    rng: np.random.Generator,
    telem_state: DirectionalTelemetryState | None,
) -> np.ndarray:
    """
    Generate one packet of complex IQ samples.

    Returns a numpy array of complex64 (interleaved float32 I/Q).
    """
    n = cfg.samples_per_packet
    fs = cfg.sample_rate

    # Time vector for this packet
    t = (np.arange(n, dtype=np.float64) + sample_offset) / fs

    # Noise: complex Gaussian, scaled by noise power
    noise_sigma = 10.0 ** (cfg.noise_power_dbfs / 20.0)
    noise = noise_sigma * (
        rng.standard_normal(n) + 1j * rng.standard_normal(n)
    ) / math.sqrt(2.0)

    # Accumulate tag signals
    signal = np.zeros(n, dtype=np.complex128)
    for i, tag in enumerate(cfg.tags):
        effective_snr_db = tag.snr_db
        if i == 0 and telem_state is not None:
            pose_snapshot = None
            with telem_state.lock:
                if telem_state.has_pose and telem_state.tx_initialized:
                    pose_snapshot = (
                        telem_state.vehicle_lat_deg,
                        telem_state.vehicle_lon_deg,
                        telem_state.vehicle_yaw_deg,
                        telem_state.tx_lat_deg,
                        telem_state.tx_lon_deg,
                    )

            if pose_snapshot is not None:
                vehicle_lat_deg, vehicle_lon_deg, vehicle_yaw_deg, tx_lat_deg, tx_lon_deg = pose_snapshot
                distance_m = _haversine_distance_m(
                    vehicle_lat_deg,
                    vehicle_lon_deg,
                    tx_lat_deg,
                    tx_lon_deg,
                )
                bearing_to_tx = _initial_bearing_deg(
                    vehicle_lat_deg,
                    vehicle_lon_deg,
                    tx_lat_deg,
                    tx_lon_deg,
                )
                off_boresight = _normalize_angle_deg(bearing_to_tx - vehicle_yaw_deg)
                antenna_gain_db = _ra2ahs_relative_gain_db(
                    off_boresight,
                    cfg.ra2ahs_hpbw_deg,
                    cfg.ra2ahs_front_to_back_db,
                )
                effective_snr_db = snr_at_distance(
                    tag.snr_db,
                    max(1.0, distance_m),
                    ref_distance_m=max(1.0, cfg.tx_offset_north_m),
                ) + antenna_gain_db

        # Amplitude from SNR: snr_linear = (amp^2) / (noise_sigma^2)
        snr_linear = 10.0 ** (effective_snr_db / 10.0)
        amp = noise_sigma * math.sqrt(snr_linear)

        # Pulsed CW: tone at freq_offset_hz, keyed on/off
        tone = amp * np.exp(2j * np.pi * tag.freq_offset_hz * t)
        mask = tag.pulse_on(t)
        signal += tone * mask

    samples = (signal + noise).astype(np.complex64)
    return samples


# ---------------------------------------------------------------------------
# Main publisher loop
# ---------------------------------------------------------------------------
def get_time_us() -> int:
    """Monotonic clock in microseconds (matches airspyhf_rx.c)."""
    return int(time.monotonic() * 1_000_000)


_running = True


def _signal_handler(signum, frame):
    global _running
    _running = False


def run(cfg: SimConfig) -> None:
    global _running
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rng = np.random.default_rng(cfg.seed)
    telemetry_state = DirectionalTelemetryState() if cfg.telemetry_sub_endpoint else None
    telem_stop_event, telem_thread = _start_telemetry_subscriber(cfg, telemetry_state) if telemetry_state is not None else (None, None)

    endpoint = f"tcp://{cfg.zmq_host}:{cfg.zmq_port}"
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 32)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(endpoint)
    print(f"iq_simulator: ZMQ PUB bound at {endpoint}", file=sys.stderr)
    print(f"iq_simulator: sample_rate={cfg.sample_rate} "
          f"samples_per_packet={cfg.samples_per_packet} "
          f"noise_floor={cfg.noise_power_dbfs} dBFS "
          f"tags={len(cfg.tags)} "
          f"realtime={'yes' if cfg.realtime else 'no'} "
          f"seed={cfg.seed}",
          file=sys.stderr)
    for i, tag in enumerate(cfg.tags):
        print(f"iq_simulator: tag[{i}] freq_offset={tag.freq_offset_hz:+.1f} Hz "
              f"snr={tag.snr_db:.1f} dB  tp={tag.tp*1000:.1f} ms  "
              f"tip={tag.tip:.3f} s  phase_offset={tag.phase_offset:.3f} s",
              file=sys.stderr)

    # Let subscribers connect
    time.sleep(0.5)

    sequence: int = 0
    sample_offset: int = 0
    start_wall = time.monotonic()
    packets_sent = 0
    packets_dropped = 0
    bytes_sent = 0
    next_gap_time = cfg.gap_interval if cfg.gap_interval > 0 else float("inf")
    in_gap_until = 0.0

    try:
        while _running:
            sim_time = sample_offset / cfg.sample_rate

            # Duration limit
            if cfg.duration > 0 and sim_time >= cfg.duration:
                print(f"iq_simulator: duration {cfg.duration}s reached, stopping",
                      file=sys.stderr)
                break

            # Gap injection
            if cfg.gap_interval > 0 and sim_time >= next_gap_time:
                in_gap_until = sim_time + cfg.gap_seconds
                next_gap_time = sim_time + cfg.gap_interval
                print(f"iq_simulator: injecting {cfg.gap_seconds*1000:.0f}ms gap "
                      f"at t={sim_time:.3f}s", file=sys.stderr)

            if sim_time < in_gap_until:
                # During gap: advance time but don't send
                sample_offset += cfg.samples_per_packet
                sequence += 1  # Sequence still increments (gap = dropped packets)
                if cfg.realtime:
                    _sleep_until(start_wall, sample_offset / cfg.sample_rate)
                continue

            # Random packet drop
            if cfg.drop_probability > 0 and rng.random() < cfg.drop_probability:
                sample_offset += cfg.samples_per_packet
                sequence += 1
                packets_dropped += 1
                if cfg.realtime:
                    _sleep_until(start_wall, sample_offset / cfg.sample_rate)
                continue

            # Generate IQ samples
            samples = generate_packet(cfg, sample_offset, rng, telemetry_state)

            # Encode wire-format packet
            payload = samples.view(np.float32).tobytes()
            payload_bytes = len(payload)
            header = encode_header(
                sequence=sequence,
                timestamp_us=get_time_us(),
                sample_rate=cfg.sample_rate,
                sample_count=cfg.samples_per_packet,
                payload_bytes=payload_bytes,
            )
            msg = header + payload
            pub.send(msg, copy=False)

            packets_sent += 1
            bytes_sent += len(msg)
            sequence += 1
            sample_offset += cfg.samples_per_packet

            # Progress logging
            if packets_sent == 1 or packets_sent % 500 == 0:
                elapsed = time.monotonic() - start_wall
                rate = sample_offset / elapsed if elapsed > 0 else 0
                print(f"iq_simulator: packets={packets_sent} "
                      f"dropped={packets_dropped} "
                      f"sim_time={sim_time:.2f}s "
                      f"effective_rate={rate:.0f} sps "
                      f"bytes={bytes_sent}",
                      file=sys.stderr)

            # Real-time pacing
            if cfg.realtime:
                _sleep_until(start_wall, sample_offset / cfg.sample_rate)

    finally:
        if telem_stop_event is not None:
            telem_stop_event.set()
        if telem_thread is not None:
            telem_thread.join(timeout=1.0)

        elapsed = time.monotonic() - start_wall
        print(f"\niq_simulator: stopped after {elapsed:.1f}s wall time, "
              f"{sample_offset / cfg.sample_rate:.1f}s sim time, "
              f"{packets_sent} packets sent, {packets_dropped} dropped",
              file=sys.stderr)
        pub.close()
        ctx.term()


def _sleep_until(start_wall: float, target_sim_time: float) -> None:
    """Sleep until wall clock catches up to simulated time."""
    now = time.monotonic()
    target_wall = start_wall + target_sim_time
    dt = target_wall - now
    if dt > 0.0001:
        time.sleep(dt)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def parse_args() -> SimConfig:
    p = argparse.ArgumentParser(
        description="IQ signal simulator — drop-in replacement for airspyhf_zeromq_rx",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Presets:
  strong      Single tag, 25 dB SNR (clear detection)
  weak        Single tag, 6 dB SNR (marginal detection)
  noise-only  No tags — pure noise (false-alarm test)
  two-tags    Two tags at different freqs / rates
  distant     Single tag, 3 dB SNR (below typical threshold)
  dropout     Strong tag with 5%% random packet drops
  gap         Strong tag with periodic 100ms gaps every 10s

Distance model:
  Use --distance-m to place a tag at a specific range.
  Reference: --snr at --ref-distance-m (default 100m).
  Free-space path loss: SNR drops 6 dB per distance doubling.

Examples:
  # Strong tag, default pipeline
  %(prog)s --preset strong

  # Tag at 500m from receiver (ref 30 dB at 100m)
  %(prog)s --freq-offset-hz 0 --snr 30 --distance-m 500

  # Two custom tags
  %(prog)s --freq-offset-hz 0 --snr 20 --tp 0.015 --tip 2.0 \\
           --freq-offset-hz 800 --snr 12 --tp 0.020 --tip 3.0

  # Noise only, 30 seconds, fast (no real-time pacing)
  %(prog)s --preset noise-only --duration 30 --no-realtime
""",
    )

    p.add_argument("--preset", choices=list(PRESETS.keys()),
                    help="Load a named scenario preset")
    p.add_argument("--sample-rate", type=int, default=768000,
                    help="IQ sample rate in Hz (default: 768000)")
    p.add_argument("--zmq-port", "-P", type=int, default=5555,
                    help="ZMQ PUB port (default: 5555)")
    p.add_argument("--zmq-host", type=str, default="127.0.0.1",
                    help="ZMQ bind address (default: 127.0.0.1)")
    p.add_argument("--samples-per-packet", type=int, default=4096,
                    help="Complex samples per ZMQ packet (default: 4096)")
    p.add_argument("--noise-power-dbfs", type=float, default=-40.0,
                    help="Noise floor in dBFS (default: -40)")

    # Tag parameters — can be repeated for multiple tags
    p.add_argument("--freq-offset-hz", type=float, action="append", default=None,
                    help="Tag frequency offset from center (Hz). Repeat for multiple tags.")
    p.add_argument("--snr", type=float, action="append", default=None,
                    help="Tag SNR in dB. Repeat for multiple tags.")
    p.add_argument("--tp", type=float, action="append", default=None,
                    help="Pulse width in seconds. Repeat for multiple tags.")
    p.add_argument("--tip", type=float, action="append", default=None,
                    help="Inter-pulse interval in seconds. Repeat for multiple tags.")
    p.add_argument("--phase-offset", type=float, action="append", default=None,
                    help="Pulse phase offset in seconds. Repeat for multiple tags.")

    # Distance model
    p.add_argument("--distance-m", type=float, default=0.0,
                    help="Transmitter distance in meters (applies path loss to first tag)")
    p.add_argument("--ref-distance-m", type=float, default=100.0,
                    help="Reference distance for SNR (default: 100m)")

    # Simulation control
    p.add_argument("--duration", type=float, default=0.0,
                    help="Run duration in seconds (0 = indefinite)")
    p.add_argument("--no-realtime", action="store_true",
                    help="Send packets as fast as possible (no pacing)")
    p.add_argument("--drop-probability", type=float, default=0.0,
                    help="Random packet drop probability 0..1 (default: 0)")
    # (validation for drop-probability range is below, after parse_args)
    p.add_argument("--gap-seconds", type=float, default=0.0,
                    help="Duration of injected gap in seconds")
    p.add_argument("--gap-interval", type=float, default=0.0,
                    help="Inject a gap every N seconds (0 = disabled)")
    p.add_argument("--seed", type=int, default=None,
                    help="RNG seed for reproducible runs")
    p.add_argument("--telemetry-sub-endpoint", type=str, default="tcp://127.0.0.1:6001",
                    help="ZMQ SUB endpoint for vehicle telemetry, e.g. tcp://127.0.0.1:6001")
    p.add_argument("--telemetry-topic", type=str, default="vehicle_pose",
                    help="Telemetry topic prefix (default: vehicle_pose)")
    p.add_argument("--tx-offset-north-m", type=float, default=4000.0,
                    help="Fixed transmitter offset north of first vehicle pose (default: 4000m)")
    p.add_argument("--ra2ahs-hpbw-deg", type=float, default=100.0,
                    help="RA-2AHS horizontal half-power beamwidth in degrees (default: 100)")
    p.add_argument("--ra2ahs-front-to-back-db", type=float, default=12.0,
                    help="RA-2AHS front-to-back ratio in dB (default: 12)")

    args = p.parse_args()

    if not (0.0 <= args.drop_probability <= 1.0):
        p.error("--drop-probability must be between 0.0 and 1.0")

    # Start from preset or defaults
    if args.preset:
        cfg = PRESETS[args.preset]
        # Deep-copy tags so preset isn't mutated
        cfg = SimConfig(
            sample_rate=cfg.sample_rate,
            zmq_host=cfg.zmq_host,
            zmq_port=cfg.zmq_port,
            samples_per_packet=cfg.samples_per_packet,
            noise_power_dbfs=cfg.noise_power_dbfs,
            tags=[TagSignal(**t.__dict__) for t in cfg.tags],
            duration=cfg.duration,
            realtime=cfg.realtime,
            drop_probability=cfg.drop_probability,
            gap_seconds=cfg.gap_seconds,
            gap_interval=cfg.gap_interval,
            seed=cfg.seed,
        )
    else:
        cfg = SimConfig()

    # Override global settings from CLI
    cfg.sample_rate = args.sample_rate
    cfg.zmq_host = args.zmq_host
    cfg.zmq_port = args.zmq_port
    cfg.samples_per_packet = args.samples_per_packet
    if args.noise_power_dbfs != -40.0 or not args.preset:
        cfg.noise_power_dbfs = args.noise_power_dbfs
    if args.duration > 0:
        cfg.duration = args.duration
    if args.no_realtime:
        cfg.realtime = False
    if args.drop_probability > 0:
        cfg.drop_probability = args.drop_probability
    if args.gap_seconds > 0:
        cfg.gap_seconds = args.gap_seconds
    if args.gap_interval > 0:
        cfg.gap_interval = args.gap_interval
    if args.seed is not None:
        cfg.seed = args.seed
    cfg.telemetry_sub_endpoint = args.telemetry_sub_endpoint
    cfg.telemetry_topic = args.telemetry_topic
    cfg.tx_offset_north_m = args.tx_offset_north_m
    cfg.ra2ahs_hpbw_deg = args.ra2ahs_hpbw_deg
    cfg.ra2ahs_front_to_back_db = args.ra2ahs_front_to_back_db

    # Build tags from CLI if --freq-offset-hz was given
    if args.freq_offset_hz is not None:
        n_tags = len(args.freq_offset_hz)
        snrs = _pad_list(args.snr, n_tags, 20.0)
        tps = _pad_list(args.tp, n_tags, 0.015)
        tips = _pad_list(args.tip, n_tags, 2.0)
        phases = _pad_list(args.phase_offset, n_tags, 0.0)

        cfg.tags = []
        for i in range(n_tags):
            cfg.tags.append(TagSignal(
                freq_offset_hz=args.freq_offset_hz[i],
                snr_db=snrs[i],
                tp=tps[i],
                tip=tips[i],
                phase_offset=phases[i],
            ))

    # Apply distance model to first tag
    if args.distance_m > 0 and cfg.tags:
        original_snr = cfg.tags[0].snr_db
        cfg.tags[0].snr_db = snr_at_distance(
            original_snr, args.distance_m, args.ref_distance_m
        )
        print(f"iq_simulator: distance model: {args.distance_m:.0f}m → "
              f"SNR {original_snr:.1f} dB → {cfg.tags[0].snr_db:.1f} dB",
              file=sys.stderr)

    return cfg


def _pad_list(lst: list | None, n: int, default: float) -> list:
    """Extend a list to length n, padding with the last value or default."""
    result = list(lst) if lst is not None else []
    while len(result) < n:
        result.append(result[-1] if result else default)
    return result[:n]


if __name__ == "__main__":
    cfg = parse_args()
    if cfg.telemetry_sub_endpoint:
        print(
            "iq_simulator: directional antenna mode enabled "
            f"(endpoint={cfg.telemetry_sub_endpoint}, tx_offset_north={cfg.tx_offset_north_m:.0f}m)",
            file=sys.stderr,
        )
    run(cfg)
