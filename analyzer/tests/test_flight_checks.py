"""Fixture-based tests for the flight_checks.py log parsers.

The parsers' extracted values drive every reported flight-data conclusion,
so representative log lines (ANSI codes, Python vs legacy controller fields,
no-detection records, 12-hour clock rollover, heading paths) are pinned here.
"""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from flight_checks import (  # noqa: E402
    collect, parse_config, parse_controller_log, parse_detector_jsonl,
    parse_detector_log, parse_session_json,
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


def test_parse_session_json_matches_config_shape(tmp_path):
    path = tmp_path / "session.json"
    path.write_text("""{
      "detection_mode": "python", "detector_sample_rate_sps": 3840,
      "tags": [{"id": 6, "frequency_hz": 146170000, "pulse_width_msecs": 15,
                "intra_pulse1_msecs": 1333, "intra_pulse2_msecs": 2000,
                "k_requested": 0, "k": 5, "false_alarm_probability": 0.05,
                "ip1_mu": null}]
    }""")
    cfgs = parse_session_json(path)
    assert set(cfgs) == {6}
    cfg = cfgs[6]
    assert cfg["K"] == "5" and cfg["Fs"] == "3840"
    assert float(cfg["tip"]) == 1.333 and float(cfg["tip2"]) == 2.0
    assert float(cfg["tp"]) == 0.015
    assert float(cfg["tagFreqMHz"]) == 146.17
    # session.json must feed collect() the same way detector_*.config does
    write_session(tmp_path / "s")
    (tmp_path / "s" / "detector_2.config").unlink()
    (tmp_path / "s" / "session.json").write_text(path.read_text())
    (session,) = collect(tmp_path / "s")
    assert session["configs"][6]["tagFreqMHz"] == cfg["tagFreqMHz"]
    assert session["issues"] == []


def test_parse_session_json_degrades_and_reports(tmp_path):
    path = tmp_path / "session.json"
    path.write_text("""{
      "detector_sample_rate_sps": 3840,
      "tags": [
        {"id": 6, "frequency_hz": null, "pulse_width_msecs": "15",
         "intra_pulse1_msecs": 1333, "k": 5, "false_alarm_probability": 0.05},
        {"frequency_hz": 146170000},
        {"id": 7, "frequency_hz": 146200000, "pulse_width_msecs": 20,
         "intra_pulse1_msecs": 2000, "k": 5, "false_alarm_probability": 0.05}
      ]
    }""")
    issues = []
    cfgs = parse_session_json(path, issues)
    # usable fields survive, unusable ones are omitted rather than zeroed
    assert set(cfgs) == {6, 7}
    assert "tagFreqMHz" not in cfgs[6] and "tp" not in cfgs[6]
    assert float(cfgs[6]["tip"]) == 1.333 and cfgs[6]["K"] == "5"
    assert float(cfgs[7]["tagFreqMHz"]) == 146.2
    # every workaround is reported with the field and tag it affected
    assert any("tag 6" in i and "frequency_hz" in i for i in issues)
    assert any("tag 6" in i and "pulse_width_msecs" in i for i in issues)
    assert any("tags[1]" in i and "id" in i for i in issues)
    assert len(issues) == 3


def test_parse_session_json_rejects_non_finite_values(tmp_path):
    # json.loads accepts NaN/Infinity; they must degrade like null, not abort,
    # including the two fields with their own validation paths (tip2, Fs)
    path = tmp_path / "session.json"
    path.write_text('{"detector_sample_rate_sps": Infinity, '
                    '"tags": [{"id": 6, "k": NaN, "frequency_hz": Infinity, '
                    '"intra_pulse1_msecs": 1333, "intra_pulse2_msecs": NaN}]}')
    issues = []
    cfgs = parse_session_json(path, issues)
    assert "K" not in cfgs[6] and "tagFreqMHz" not in cfgs[6]
    assert "tip2" not in cfgs[6] and "Fs" not in cfgs[6]
    assert float(cfgs[6]["tip"]) == 1.333
    assert sum("not a finite number" in i for i in issues) == 4


def test_parse_detector_jsonl_degrades_bad_numeric_fields(tmp_path):
    path = tmp_path / "detector_2.jsonl"
    good = {"snr_db": 9.0, "score_ratio": 3.1, "noise_psd": 1.5e-05}
    path.write_text("\n".join([
        json.dumps({"type": "startup", "tag_id": 2, "center_freq": "146.17"}),
        json.dumps({"type": "detection", "cycle": 1, "freq_hz": 0.0, **good}),
        json.dumps({"type": "startup", "tag_id": 2, "center_freq": 146.17}),
        json.dumps({"type": "detection", "cycle": 2, "freq_hz": [1], **good}),
        '{"type": "detection", "cycle": 3, "freq_hz": 0.0, "timestamp_ns": 1e40, '
        '"snr_db": 9.0, "score_ratio": 3.1, "noise_psd": 1.5e-05}',
        json.dumps({"type": "detection", "cycle": 4, "freq_hz": 0.0,
                    "snr_db": "bad", "score_ratio": 3.1, "noise_psd": 1.5e-05}),
        '{"type": "detection", "cycle": 5, "freq_hz": 0.0, '
        '"snr_db": 9.0, "score_ratio": NaN, "noise_psd": 1.5e-05}',
        json.dumps({"type": "no_detection", "cycle": 6, "best_candidate": "x"}),
    ]) + "\n")
    issues = []
    dets, nodet = parse_detector_jsonl(path, issues)
    # 1,2: bad center_freq/freq_hz; 4,5: bad metrics; 3 kept with t=nan
    assert [d["cycle"] for d in dets] == [3]
    assert math.isnan(dets[0]["t"]) and dets[0]["snr"] == 9.0
    assert nodet == []
    assert sum("lacks finite center_freq/freq_hz" in i for i in issues) == 2
    assert sum("out-of-range timestamp_ns" in i for i in issues) == 1
    assert sum("non-numeric snr_db/score_ratio/noise_psd" in i for i in issues) == 2


def test_parse_detector_jsonl_skips_non_object_lines(tmp_path):
    path = tmp_path / "detector_2.jsonl"
    path.write_text("42\n[1, 2]\n" + _jsonl(1, 0.0, 10.0) + "not json\n")
    issues = []
    (d,), nodet = parse_detector_jsonl(path, issues)
    assert d["snr"] == 10.0 and nodet == [2.5e-05]
    assert issues == ["detector_2.jsonl: 3 malformed JSON line(s) skipped"]


def test_parse_session_json_unreadable_is_reported_not_raised(tmp_path):
    path = tmp_path / "session.json"
    path.write_text("{ not json")
    issues = []
    assert parse_session_json(path, issues) == {}
    assert len(issues) == 1 and "unreadable" in issues[0]
    # collect() must still produce the session from the other logs
    write_session(tmp_path / "s")
    (tmp_path / "s" / "session.json").write_text("{ not json")
    (session,) = collect(tmp_path / "s")
    assert len(session["pulses"]) == 2
    assert session["configs"][2]["tagFreqMHz"] == "146.170000"  # from .config
    assert any("unreadable" in i for i in session["issues"])


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


def test_detector_jsonl_time_is_utc_regardless_of_host_tz(tmp_path):
    # 1788543167 s = 2026-09-04T17:32:47Z. Analysis hosts are not on UTC, and
    # the rPi/SITL producers differ too; jsonl epochs must not pick up either.
    # (Asserting the absolute UTC value fails on any non-UTC host if the
    # parser regresses to a naive local conversion.)
    path = tmp_path / "detector_2.jsonl"
    path.write_text(_jsonl(1, 0.0, 10.0))
    (d,), _ = parse_detector_jsonl(path)
    assert d["t"] == 17 * 3600 + 32 * 60 + 47


def test_controller_log_accepts_12h_and_24h_stamps(tmp_path):
    log = tmp_path / "MavlinkTagController.log"

    # legacy %I (no AM/PM): noon rollover must unwrap forward
    lines = [PY_PULSE.replace("10:00:01", hms)
             for hms in ("11:59:59", "12:00:01", "01:00:01")]
    log.write_text("\n".join(lines) + "\n")
    ts = [p["t"] for p in parse_controller_log(log)[0]]
    assert ts == sorted(ts)
    assert ts[1] - ts[0] == 2 and ts[2] - ts[1] == 3600

    # current %H UTC: an AM->PM step whose folded value would *increase*
    # (00:10 -> 13:00) must still read as 12 h 50 min, not 50 min
    lines = [PY_PULSE.replace("10:00:01", hms)
             for hms in ("00:10:00", "13:00:00", "23:59:59")]
    log.write_text("\n".join(lines) + "\n")
    ts = [p["t"] for p in parse_controller_log(log)[0]]
    assert ts == sorted(ts)
    assert ts[1] - ts[0] == 12 * 3600 + 50 * 60
    assert ts[2] - ts[1] == 10 * 3600 + 59 * 60 + 59

    # current %H UTC crossing midnight: must unwrap by 24 h, not 12
    lines = [PY_PULSE.replace("10:00:01", hms)
             for hms in ("13:00:00", "23:59:59", "00:00:01")]
    log.write_text("\n".join(lines) + "\n")
    ts = [p["t"] for p in parse_controller_log(log)[0]]
    assert ts == sorted(ts)
    assert ts[2] - ts[1] == 2

    # hour 00 alone identifies %H (%I is 01..12): 00:10 -> 12:30 is 12 h 20 min
    lines = [PY_PULSE.replace("10:00:01", hms) for hms in ("00:10:00", "12:30:00")]
    log.write_text("\n".join(lines) + "\n")
    ts = [p["t"] for p in parse_controller_log(log)[0]]
    assert ts[1] - ts[0] == 12 * 3600 + 20 * 60


STORED_045 = ("[10:00:01|I]  Rotation slice stored: tag_id: 2  heading: 45  "
              "snr: 14.0  confirmed: 1  (CommandHandler.cpp:746)")
PY_PULSE_H90 = PY_PULSE.replace("10:00:01", "10:00:20").replace("heading: 45.0", "heading: 90.0")
STORED_090 = STORED_045.replace("heading: 45", "heading: 90").replace("10:00:01", "10:00:20")


def _jsonl(cycle, offset_hz, snr):
    return "\n".join([
        json.dumps({"type": "startup", "tag_id": 2, "center_freq": 146.17}),
        json.dumps({"type": "detection", "cycle": cycle, "timestamp_ns": 1788543167227431460,
                    "freq_hz": offset_hz, "snr_db": snr, "score_ratio": 3.1,
                    "noise_psd": 1.5e-05}),
        json.dumps({"type": "no_detection", "cycle": cycle + 1,
                    "best_candidate": {"noise_psd": 2.5e-05}}),
    ]) + "\n"


def test_collect_persistent_detector_rotation_layout(tmp_path):
    """One controller log at the rotation root, per-heading detector jsonl in
    heading-NNN/ subdirs: each heading must become its own session, with the
    controller pulses attributed via 'Rotation slice stored' lines."""
    root = tmp_path / "Logs-Rotation-x"
    root.mkdir()
    (root / "MavlinkTagController.log").write_text(
        "\n".join([PY_PULSE, STORED_045, PY_PULSE_H90, STORED_090, LEGACY_PULSE, NODET]) + "\n")
    # root text log spans every heading; it must not be double counted
    (root / "py_detector_2.log").write_text(DETECTED + "\n" + DETECTED + "\n")
    (root / "session.json").write_text(json.dumps({
        "detector_sample_rate_sps": 3840,
        "tags": [{"id": 2, "frequency_hz": 146170000, "pulse_width_msecs": 15,
                  "intra_pulse1_msecs": 1333, "k": 5, "false_alarm_probability": 0.05}]}))
    for hdg, off, snr in ((45, 12.0, 13.2), (90, -30.0, 9.8)):
        hd = root / f"heading-{hdg:03d}"
        hd.mkdir()
        (hd / "detector_2.jsonl").write_text(_jsonl(hdg // 45, off, snr))

    sessions = collect(tmp_path)
    by_hdg = {s["heading_cmd"]: s for s in sessions}
    assert set(by_hdg) == {45, 90, None}

    s45 = by_hdg[45]
    assert s45["flight"] == "Logs-Rotation-x"
    assert [p["id"] for p in s45["pulses"]] == [2]
    assert s45["pulses"][0]["heading_cmd"] == 45
    (d,) = s45["detections"][2]
    assert d["offset_hz"] == 12.0 and abs(d["freq_mhz"] - 146.170012) < 1e-9
    assert d["snr"] == 13.2 and d["noise"] == 1.5e-05
    assert s45["det_nodet_noise"] == [2.5e-05]
    assert s45["configs"][2]["tagFreqMHz"] == "146.170000"
    assert by_hdg[90]["detections"][2][0]["snr"] == 9.8

    # root keeps only what the headings could not claim; the LEGACY pulse has
    # no 'Rotation slice stored' line so it stays there and is reported
    root_s = by_hdg[None]
    assert root_s["detections"] == {}
    assert [p["id"] for p in root_s["pulses"]] == [3]
    assert root_s["nodet_noise"] == [4.0e-05]
    assert any("not attributable" in i for i in root_s["issues"])
    assert by_hdg[None]["heading_cmd"] is None


if __name__ == "__main__":
    import tempfile
    for name, fn in sorted(globals().items()):
        if name.startswith("test_"):
            with tempfile.TemporaryDirectory() as td:
                fn(Path(td))
            print(f"PASS {name}")
