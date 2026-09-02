"""Fixture-based tests for the flight_checks.py log parsers.

The parsers' extracted values drive every reported flight-data conclusion,
so representative log lines (ANSI codes, Python vs legacy controller fields,
no-detection records, 12-hour clock rollover, heading paths) are pinned here.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from flight_checks import (  # noqa: E402
    collect, parse_config, parse_controller_log, parse_detector_log,
)

PY_PULSE = ("[10:00:01|I] Conf: 1 Id:  2 snr:  14.0 heading: 45.0 "
            "score_ratio: 3.500 noise_psd: 1.0e-05 freq: 146170000 seq: 7 "
            "group_ind: 1 lat/lon/yaw/alt: 38.100000 -122.200000   45  90")
LEGACY_PULSE = ("[10:00:02|I] Conf: 0 Id:  3 snr:  11.5 heading: 45.0 "
                "stft_score: 3.2e+01 noise_psd: 2.0e-05 freq: 146170150 "
                "seq: 8 lat/lon/yaw/alt: 38.100000 -122.200000   45  90")
NODET = ("[10:00:03|I] NO DETECTION Id:  2 score_ratio: 1.100 "
         "noise_psd: 4.0e-05 freq: 146170000 "
         "lat/lon/yaw/alt: 38.100000 -122.200000   45  90")
DETECTED = ("[   5 10:00:04]  DETECTED 146.170012 MHz (+12.0 Hz)  "
            "SNR 13.2 dB  score_ratio 3.100  noise 1.5e-05")
DET_NODET = ("[   6 10:00:05]  no detection  35 ms  best=+12.0 Hz "
             "SNR 3.1 dB ratio 1.200 noise 3.000e-05")


def write_session(d, extra_ctrl=""):
    d.mkdir(parents=True)
    ctrl = "\n".join([PY_PULSE, LEGACY_PULSE, NODET]) + extra_ctrl + "\n"
    (d / "MavlinkTagController.log").write_text(ctrl)
    (d / "py_detector_2.log").write_text(DETECTED + "\n" + DET_NODET + "\n")
    (d / "detector_2.config").write_text(
        "K:\t5\ntip:\t1.333\ntp:\t0.015\ntagFreqMHz:\t146.170000\n")


def test_parse_controller_log(tmp_path):
    # ANSI escapes around a pulse line must be stripped before matching
    log = tmp_path / "MavlinkTagController.log"
    log.write_text("\x1b[32m" + PY_PULSE + "\x1b[0m\n" + LEGACY_PULSE + "\n"
                   + NODET + "\n")
    pulses, nodet = parse_controller_log(log)
    assert len(pulses) == 2
    py, legacy = pulses
    assert py["conf"] == 1 and py["id"] == 2 and py["snr"] == 14.0
    assert py["score_ratio"] == 3.5 and py["stft_score"] is None
    assert py["freq"] == 146170000 and py["seq"] == 7
    assert py["heading"] == 45.0 and py["noise_psd"] == 1.0e-05
    assert legacy["conf"] == 0 and legacy["id"] == 3
    assert legacy["stft_score"] == 32.0 and legacy["score_ratio"] is None
    assert legacy["freq"] == 146170150
    assert nodet == [4.0e-05]


def test_controller_12h_rollover(tmp_path):
    # 11:59 -> 12:00 (noon) must unwrap forward, not jump back 12 h
    log = tmp_path / "MavlinkTagController.log"
    lines = []
    for hms in ("11:59:59", "12:00:01", "01:00:01"):
        lines.append(PY_PULSE.replace("10:00:01", hms))
    log.write_text("\n".join(lines) + "\n")
    pulses, _ = parse_controller_log(log)
    ts = [p["t"] for p in pulses]
    assert ts == sorted(ts)
    assert ts[1] - ts[0] == 2
    assert ts[2] - ts[1] == 3600


def test_parse_detector_log(tmp_path):
    log = tmp_path / "py_detector_2.log"
    log.write_text(DETECTED + "\n" + DET_NODET + "\n")
    dets, nodet = parse_detector_log(log)
    assert len(dets) == 1
    d = dets[0]
    assert d["cycle"] == 5 and d["freq_mhz"] == 146.170012
    assert d["offset_hz"] == 12.0 and d["snr"] == 13.2
    assert d["score_ratio"] == 3.1 and d["noise"] == 1.5e-05
    assert nodet == [3.0e-05]


def test_parse_config(tmp_path):
    cfg = tmp_path / "detector_2.config"
    cfg.write_text("K:\t5\ntip:\t1.333\ntagFreqMHz:\t146.170000\nunknown:\tx\n")
    parsed = parse_config(cfg)
    assert parsed == {"K": "5", "tip": "1.333", "tagFreqMHz": "146.170000"}


def test_collect_heading_paths(tmp_path):
    # both heading-045 (current) and heading_090 (legacy) must be recognized
    write_session(tmp_path / "flight1" / "heading-045")
    write_session(tmp_path / "flight1" / "heading_090")
    write_session(tmp_path / "nohdg")
    sessions = collect(tmp_path)
    by_hdg = {s["heading_cmd"]: s for s in sessions}
    assert set(by_hdg) == {45, 90, None}
    s45 = by_hdg[45]
    assert s45["flight"] == "flight1"
    assert len(s45["pulses"]) == 2
    assert s45["nodet_noise"] == [4.0e-05]
    assert [d["snr"] for d in s45["detections"][2]] == [13.2]
    assert s45["det_nodet_noise"] == [3.0e-05]
    assert s45["configs"][2]["tagFreqMHz"] == "146.170000"
    assert by_hdg[None]["heading_cmd"] is None


if __name__ == "__main__":
    import tempfile
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            with tempfile.TemporaryDirectory() as td:
                fn(Path(td))
            print(f"PASS {name}")
