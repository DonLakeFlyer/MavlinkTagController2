#!/usr/bin/env python3
"""Run the FLIGHT_DATA_ANALYSIS.md checks against recorded flight logs.

Walks a directory tree of controller/detector logs (MavlinkTagController.log,
py_detector_*.log or per-heading heading-NNN/detector_*.jsonl, session.json or
legacy detector_*.config) and produces the evidence for:

  Check 0 - detections per heading per rotation (Apr-11 heading_XXX dwells)
  Check 1 - reported SNR distribution vs the predicted noise floor, split by K
  Check 2 - detection frequency vs entered tag frequency (noise vs multipath)
  Check 3 - SNR vs commanded heading
  Check 4 - detection-cycle period from pulse seq/timestamps (true PRI is
            not measurable from these logs; needs raw IQ)
  Check 5 - noise_psd across sessions and headings

Usage:  python3 flight_checks.py "/Users/don/Documents/PDC Testing"
"""

import datetime
import json
import math
import re
import sys
from collections import defaultdict
from pathlib import Path

# ---------------------------------------------------------------- parsing

ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")

CONF_RE = re.compile(
    r"\[(\d\d):(\d\d):(\d\d)\|.\]\s+Conf: (\d+) Id:\s*(\d+) snr:\s*([-\d.]+) "
    r"heading:\s*([-\d.]+) (?:score_ratio: ([\d.]+)|stft_score:\s*([\deE.+-]+)) "
    r"noise_psd:\s*([\deE.+-]+) freq:\s*(\d+) seq: (\d+)"
    r"(?: group_ind: (\d+))? lat/lon/yaw/alt:\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)"
)

DETECTED_RE = re.compile(
    r"\[\s*(\d+)\s+(\d\d):(\d\d):(\d\d)\]\s+DETECTED\s+([\d.]+) MHz\s+"
    r"\(([+-][\d.]+) Hz\)\s+SNR ([-\d.]+) dB\s+score_ratio ([\d.]+)\s+noise ([\deE.+-]+)"
)

# controller "NO DETECTION" cycles and detector "no detection" cycles both
# report the noise estimate; Check 5 must include them or the noise medians
# are conditioned on threshold crossings
CTRL_NODET_RE = re.compile(
    r"NO DETECTION Id:\s*\d+ score_ratio: [\d.]+ noise_psd:\s*([\deE.+-]+)"
)
DET_NODET_RE = re.compile(r"no detection\s.*?\bnoise ([\deE.+-]+)")

# Persistent-detector rotations log every heading into one controller log;
# each stored pulse is followed by this line naming the commanded heading.
SLICE_STORED_RE = re.compile(
    r"Rotation slice stored: tag_id:\s*(\d+)\s+heading:\s*([-\d.]+)"
)

CONFIG_KEYS = ("K", "tip", "tp", "tagFreqMHz", "falseAlarmProb", "opMode", "Fs")


def parse_config(path):
    cfg = {}
    for line in path.read_text(errors="replace").splitlines():
        parts = line.split(":", 1)
        if len(parts) == 2 and parts[0].strip() in CONFIG_KEYS:
            cfg[parts[0].strip()] = parts[1].strip()
    return cfg


def _finite(value):
    """True for a real, finite JSON number (bool excluded)."""
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _session_number(tag, key, issues, tag_label):
    """Return tag[key] as a float, or None (recording why) if unusable."""
    value = tag.get(key)
    if value is None:
        issues.append(f"{tag_label}: '{key}' missing or null")
        return None
    if not _finite(value):
        issues.append(f"{tag_label}: '{key}' is not a finite number ({value!r})")
        return None
    return float(value)


def parse_session_json(path, issues=None):
    """Map session.json (Python-detector sessions) onto the per-tag dict shape
    that parse_config produces, so the checks are source-agnostic.

    Bad or missing values never abort: the affected key is left out (so the
    checks skip it the same way they skip an absent .config) and a
    human-readable reason is appended to *issues*."""
    if issues is None:
        issues = []
    try:
        session = json.loads(path.read_text(errors="replace"))
    except (OSError, json.JSONDecodeError) as exc:
        issues.append(f"{path.name}: unreadable ({exc}); tag parameters unavailable")
        return {}
    if not isinstance(session, dict):
        issues.append(f"{path.name}: top level is not a JSON object")
        return {}

    fs = session.get("detector_sample_rate_sps")
    if fs is not None and not _finite(fs):
        issues.append(f"{path.name}: 'detector_sample_rate_sps' is not a finite number ({fs!r})")
        fs = None
    tags = session.get("tags")
    if not isinstance(tags, list):
        issues.append(f"{path.name}: 'tags' missing or not a list")
        tags = []

    cfgs = {}
    for index, tag in enumerate(tags):
        if not isinstance(tag, dict) or not isinstance(tag.get("id"), int):
            issues.append(f"{path.name}: tags[{index}] has no integer 'id'; skipped")
            continue
        label = f"{path.name} tag {tag['id']}"
        cfg = {"opMode": "python"}
        k = _session_number(tag, "k", issues, label)
        if k is not None:
            cfg["K"] = str(int(k))
        tip = _session_number(tag, "intra_pulse1_msecs", issues, label)
        if tip is not None:
            cfg["tip"] = f"{tip / 1000.0:f}"
        tp = _session_number(tag, "pulse_width_msecs", issues, label)
        if tp is not None:
            cfg["tp"] = f"{tp / 1000.0:f}"
        freq = _session_number(tag, "frequency_hz", issues, label)
        if freq is not None:
            cfg["tagFreqMHz"] = f"{freq / 1e6:f}"
        pf = _session_number(tag, "false_alarm_probability", issues, label)
        if pf is not None:
            cfg["falseAlarmProb"] = f"{pf:f}"
        # intra_pulse2_msecs == 0 legitimately means single-rate; only
        # complain if it is present but unusable
        tip2 = tag.get("intra_pulse2_msecs")
        if tip2 is not None:
            if not _finite(tip2):
                issues.append(f"{label}: 'intra_pulse2_msecs' is not a finite number ({tip2!r})")
            elif tip2:
                cfg["tip2"] = f"{tip2 / 1000.0:f}"
        if fs is not None:
            cfg["Fs"] = str(fs)
        cfgs[tag["id"]] = cfg
    return cfgs


def parse_controller_log(path):
    pulses = []
    text = ANSI_RE.sub("", path.read_text(errors="replace"))
    matches = list(CONF_RE.finditer(text))
    # Older controller logs used %I (12-hour, no AM/PM, local time); newer ones
    # are %H UTC. %I only yields 01..12, so an hour of 00 or 13..23 proves
    # 24-hour and the fold below must be skipped (00:10 -> 13:00 would
    # otherwise read as +50 min). A 24-hour log confined to 01..12 takes the
    # legacy path, where the fold+unwrap still yields correct deltas.
    is_24h = any(int(m.group(1)) == 0 or int(m.group(1)) > 12 for m in matches)
    period = 24 * 3600 if is_24h else 12 * 3600
    wrap = 0
    prev_t = None
    for m in matches:
        h, mn, s = int(m.group(1)), int(m.group(2)), int(m.group(3))
        t = (h if is_24h else h % 12) * 3600 + mn * 60 + s
        # A decrease means the clock rolled over (noon/midnight for %I,
        # midnight for %H); keep within-session deltas monotonic.
        if prev_t is not None and t < prev_t:
            wrap += period
        prev_t = t
        pulses.append({
            "t": t + wrap,
            "conf": int(m.group(4)),
            "id": int(m.group(5)),
            "snr": float(m.group(6)),
            "heading": float(m.group(7)),
            "score_ratio": float(m.group(8)) if m.group(8) else None,
            "stft_score": float(m.group(9)) if m.group(9) else None,
            "noise_psd": float(m.group(10)),
            "freq": int(m.group(11)),
            "seq": int(m.group(12)),
            "lat": float(m.group(14)),
            "lon": float(m.group(15)),
            "yaw": float(m.group(16)),
            "alt": float(m.group(17)),
            "heading_cmd": None,
            "_pos": m.start(),
        })
    # Attribute each pulse to the commanded heading named by the next
    # "Rotation slice stored" line for the same tag (before the next pulse).
    stored = [(m.start(), int(m.group(1)), int(round(float(m.group(2)))))
              for m in SLICE_STORED_RE.finditer(text)]
    for i, p in enumerate(pulses):
        end = pulses[i + 1]["_pos"] if i + 1 < len(pulses) else len(text)
        for pos, tid, hdg in stored:
            if p["_pos"] < pos < end and tid == p["id"]:
                p["heading_cmd"] = hdg
                break
    for p in pulses:
        del p["_pos"]
    nodet_noise = [float(v) for v in CTRL_NODET_RE.findall(text)]
    return pulses, nodet_noise


def parse_detector_log(path):
    dets = []
    text = path.read_text(errors="replace")
    for m in DETECTED_RE.finditer(text):
        h, mn, s = int(m.group(2)), int(m.group(3)), int(m.group(4))
        dets.append({
            "cycle": int(m.group(1)),
            "t": h * 3600 + mn * 60 + s,
            "freq_mhz": float(m.group(5)),
            "offset_hz": float(m.group(6)),
            "snr": float(m.group(7)),
            "score_ratio": float(m.group(8)),
            "noise": float(m.group(9)),
        })
    nodet_noise = [float(v) for v in DET_NODET_RE.findall(text)]
    return dets, nodet_noise


def parse_detector_jsonl(path, issues=None):
    """Per-heading detector_<tag>.jsonl -> same (dets, nodet_noise) shape as
    parse_detector_log so the checks don't care which source they got."""
    if issues is None:
        issues = []
    dets = []
    nodet_noise = []
    center_mhz = None
    bad = 0
    with path.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                bad += 1
                continue
            if not isinstance(e, dict):
                bad += 1  # a bare scalar/list parses but is not a record
                continue
            kind = e.get("type")
            if kind == "startup":
                center_mhz = e.get("center_freq")
            elif kind == "detection":
                offset_hz = e.get("freq_hz")
                if not (_finite(center_mhz) and _finite(offset_hz)):
                    issues.append(f"{path.name}: detection cycle {e.get('cycle')} "
                                  "lacks finite center_freq/freq_hz; skipped")
                    continue
                ts_ns = e.get("timestamp_ns")
                t = float("nan")
                if _finite(ts_ns):
                    # Epoch -> UTC seconds-of-day, matching the detector text
                    # log and independent of the analysis host's timezone.
                    try:
                        lt = datetime.datetime.fromtimestamp(
                            ts_ns / 1e9, tz=datetime.timezone.utc)
                        t = lt.hour * 3600 + lt.minute * 60 + lt.second
                    except (ValueError, OverflowError, OSError):
                        issues.append(f"{path.name}: detection cycle {e.get('cycle')} "
                                      f"has out-of-range timestamp_ns {ts_ns!r}")
                snr = e.get("snr_db")
                score = e.get("score_ratio")
                noise = e.get("noise_psd")
                if not (_finite(snr) and _finite(score) and _finite(noise)):
                    issues.append(f"{path.name}: detection cycle {e.get('cycle')} "
                                  "has non-numeric snr_db/score_ratio/noise_psd; skipped")
                    continue
                dets.append({
                    "cycle": e.get("cycle", 0),
                    "t": t,
                    "freq_mhz": center_mhz + offset_hz / 1e6,
                    "offset_hz": offset_hz,
                    "snr": float(snr),
                    "score_ratio": float(score),
                    "noise": float(noise),
                })
            elif kind == "no_detection":
                cand = e.get("best_candidate") or {}
                if isinstance(cand, dict) and _finite(cand.get("noise_psd")):
                    nodet_noise.append(float(cand["noise_psd"]))
    if bad:
        issues.append(f"{path.name}: {bad} malformed JSON line(s) skipped")
    return dets, nodet_noise


def collect(root):
    """Return list of session dicts: one per directory containing a controller log."""
    sessions = []
    for log in sorted(root.rglob("MavlinkTagController.log")):
        d = log.parent
        cfgs = {}
        issues = []
        session_json = d / "session.json"
        if session_json.exists():
            cfgs = parse_session_json(session_json, issues)
        for c in d.glob("detector_*.config"):
            m = re.search(r"detector_(\d+)", c.name)
            if not m:
                continue
            cfgs[int(m.group(1))] = parse_config(c)
        dets = {}
        det_nodet = []
        for p in d.glob("py_detector_*.log"):
            m = re.search(r"py_detector_(\d+)", p.name)
            if not m:
                continue
            dets[int(m.group(1))], nn = parse_detector_log(p)
            det_nodet.extend(nn)
        pulses, ctrl_nodet = parse_controller_log(log)
        rel = d.relative_to(root)
        m = re.search(r"heading[-_](\d+)", str(rel))
        session = {
            "dir": str(rel),
            "heading_cmd": int(m.group(1)) if m else None,
            "flight": str(rel.parent) if m else None,
            "configs": cfgs,
            "pulses": pulses,
            "nodet_noise": ctrl_nodet,
            "detections": dets,
            "det_nodet_noise": det_nodet,
            "issues": issues,
        }

        # Persistent-detector rotation: one controller log at the root, per-
        # heading detector jsonl in heading-NNN/ (no controller log of their
        # own). Split into per-heading sessions so Check 0/3 can see them.
        heading_dirs = sorted(
            hd for hd in d.glob("heading-*")
            if hd.is_dir() and not (hd / "MavlinkTagController.log").exists()
            and any(hd.glob("detector_*.jsonl")))
        if heading_dirs:
            for hd in heading_dirs:
                hm = re.search(r"heading-(\d+)", hd.name)
                if not hm:
                    continue
                hdg = int(hm.group(1))
                h_issues = []
                h_dets = {}
                h_nodet = []
                for jp in sorted(hd.glob("detector_*.jsonl")):
                    jm = re.search(r"detector_(\d+)", jp.name)
                    if not jm:
                        continue
                    h_dets[int(jm.group(1))], nn = parse_detector_jsonl(jp, h_issues)
                    h_nodet.extend(nn)
                sessions.append({
                    "dir": str(hd.relative_to(root)),
                    "heading_cmd": hdg,
                    "flight": str(rel),
                    "configs": cfgs,
                    "pulses": [p for p in pulses if p["heading_cmd"] == hdg],
                    "nodet_noise": [],
                    "detections": h_dets,
                    "det_nodet_noise": h_nodet,
                    "issues": h_issues,
                })
            # The root's py_detector_*.log spans every heading and would
            # double count against the per-heading jsonl; keep only what the
            # heading sessions could not claim.
            session["detections"] = {}
            session["det_nodet_noise"] = []
            session["pulses"] = [p for p in pulses if p["heading_cmd"] is None]
            if session["pulses"]:
                issues.append(f"{len(session['pulses'])} controller pulse(s) not "
                              "attributable to a heading (no 'Rotation slice stored' line)")

        sessions.append(session)
    return sessions


# ---------------------------------------------------------------- helpers

def pctile(vals, q):
    if not vals:
        return float("nan")
    v = sorted(vals)
    i = (len(v) - 1) * q
    lo, hi = int(math.floor(i)), int(math.ceil(i))
    return v[lo] + (v[hi] - v[lo]) * (i - lo)


def hist_line(vals, lo, hi, nbins=30, width=60):
    if hi <= lo:
        hi = lo + 1.0
    counts = [0] * nbins
    for v in vals:
        b = int((v - lo) / (hi - lo) * nbins)
        counts[min(max(b, 0), nbins - 1)] += 1
    peak = max(counts) or 1
    lines = []
    for i, c in enumerate(counts):
        left = lo + (hi - lo) * i / nbins
        bar = "#" * round(c / peak * width)
        lines.append(f"  {left:7.1f} | {bar} {c if c else ''}")
    return "\n".join(lines)


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


# ---------------------------------------------------------------- checks

def check0_and_3(sessions):
    print("=" * 78)
    print("CHECK 0 / CHECK 3 — detections and SNR per commanded heading (Apr-11 dwells)")
    print("=" * 78)
    flights = defaultdict(dict)
    for s in sessions:
        if s["heading_cmd"] is None:
            continue
        flights[s["flight"]][s["heading_cmd"]] = s
    for flight in sorted(flights):
        byhdg = flights[flight]
        ids = sorted({tid for s in byhdg.values() for tid in s["detections"]} |
                     {p["id"] for s in byhdg.values()
                      for p in s["pulses"] if p["conf"] == 1})
        print(f"\n{flight}   (detector ids: {ids})")
        hdr = "  hdg  " + "".join(f"| id {tid}:  n med-snr  dHz   scr " for tid in ids)
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))
        for hdg in sorted(byhdg):
            s = byhdg[hdg]
            row = f"  {hdg:3d}  "
            for tid in ids:
                dets = s["detections"].get(tid, [])
                if dets:
                    # aggregate the dwell: median snr/offset, best score_ratio
                    snr = pctile([d["snr"] for d in dets], 0.5)
                    off = pctile([d["offset_hz"] for d in dets], 0.5)
                    scr = max(d["score_ratio"] for d in dets)
                    row += f"| {len(dets):3d} {snr:7.1f} {off:+6.0f} {scr:5.2f} "
                    continue
                # controller fallback ('c' rows): confirmed pulses; offset only
                # for legacy stft_score records (py records report entered freq)
                ps = [p for p in s["pulses"]
                      if p["id"] == tid and p["conf"] == 1]
                if ps:
                    snr = pctile([p["snr"] for p in ps], 0.5)
                    tag_mhz = s["configs"].get(tid, {}).get("tagFreqMHz")
                    legacy = [p for p in ps if p["stft_score"] is not None]
                    if legacy and tag_mhz:
                        off = pctile([p["freq"] - float(tag_mhz) * 1e6
                                      for p in legacy], 0.5)
                        off_str = f"{off:+6.0f}"
                    else:
                        off_str = "     -"
                    scr = max(p["score_ratio"] if p["score_ratio"] is not None
                              else p["stft_score"] for p in ps)
                    row += f"| {len(ps):3d} {snr:7.1f} {off_str} {scr:5.2f}c"
                else:
                    row += "| " + " " * 4 + " " * 4 + "---- " + " " * 12
            print(row)
        for tid in ids:
            n_det = sum(1 for s in byhdg.values()
                        if s["detections"].get(tid)
                        or any(p["id"] == tid and p["conf"] == 1
                               for p in s["pulses"]))
            print(f"  id {tid}: detected on {n_det}/{len(byhdg)} headings")


def effective_k(cfg):
    """Controller normalizes K < 2 to 5 before launching the detector
    (CommandHandler.cpp); mirror that so logs group under the K actually run."""
    k = cfg.get("K", "?")
    if k.isdigit() and int(k) < 2:
        return "5"
    return k


def check1(sessions):
    print("\n" + "=" * 78)
    print("CHECK 1 — reported SNR distribution (predicted floor: K=20 ~17.2 dB, K=5 ~12.5)")
    print("=" * 78)
    by_k = defaultdict(list)
    for s in sessions:
        for tid, dets in s["detections"].items():
            if not dets:
                continue
            k = effective_k(s["configs"].get(tid, {}))
            by_k[k].extend(d["snr"] for d in dets)
        # sessions with no detector detections (e.g. uavrt format has no
        # py_detector logs): use confirmed controller pulses
        if not any(s["detections"].values()):
            for p in s["pulses"]:
                if p["conf"] == 1:
                    k = effective_k(s["configs"].get(p["id"], {}))
                    by_k[k].append(p["snr"])
    for k in sorted(by_k, key=lambda k: (0, int(k)) if k.isdigit() else (1, k)):
        v = by_k[k]
        if not v:
            continue
        print(f"\nK={k}  n={len(v)}  min={min(v):.1f}  p5={pctile(v,0.05):.1f}  "
              f"median={pctile(v,0.5):.1f}  p95={pctile(v,0.95):.1f}  max={max(v):.1f}")
        print(hist_line(v, min(v) - 0.5, max(v) + 0.5))


def check2(sessions):
    print("\n" + "=" * 78)
    print("CHECK 2 — detection frequency offset from entered tag frequency")
    print("      (all-headings sanity aggregate — classifying rear-heading energy")
    print("       requires filtering by commanded heading vs a known bearing)")
    print("=" * 78)
    offs_all = []
    print(f"\n{'session':52s} {'id':>3s} {'n':>4s} {'offsets (Hz)':s}")
    for s in sessions:
        for tid, dets in sorted(s["detections"].items()):
            if not dets:
                continue
            # offset_hz in the log is relative to the channel center, not the
            # entered tag frequency — recompute from the absolute frequency
            tag_mhz = s["configs"].get(tid, {}).get("tagFreqMHz")
            if tag_mhz is None:
                print(f"{s['dir'][:52]:52s} {tid:3d} {len(dets):4d} "
                      f"(skipped: no tagFreqMHz config; offsets are channel-relative)")
                continue
            tag_mhz = float(tag_mhz)
            offs = [(d["freq_mhz"] - tag_mhz) * 1e6 for d in dets]
            offs_all.extend(offs)
            show = ", ".join(f"{o:+.0f}" for o in offs[:8])
            if len(offs) > 8:
                show += ", ..."
            print(f"{s['dir'][:52]:52s} {tid:3d} {len(offs):4d} {show}")
        # controller-only fallback, legacy uavrt (stft_score) records only:
        # the Python detector reports the configured frequency, not the
        # detected bin, so its controller records carry no offset information
        if not any(s["detections"].values()):
            by_id = defaultdict(list)
            for p in s["pulses"]:
                if p["conf"] == 1 and p["stft_score"] is not None:
                    by_id[p["id"]].append(p)
            for tid, ps in sorted(by_id.items()):
                tag_mhz = s["configs"].get(tid, {}).get("tagFreqMHz")
                if tag_mhz is None:
                    print(f"{s['dir'][:52]:52s} {tid:3d} {len(ps):4d} "
                          f"(skipped: no tagFreqMHz config)")
                    continue
                offs = [p["freq"] - float(tag_mhz) * 1e6 for p in ps]
                offs_all.extend(offs)
                show = ", ".join(f"{o:+.0f}" for o in offs[:8])
                if len(offs) > 8:
                    show += ", ..."
                print(f"{s['dir'][:52]:52s} {tid:3d} {len(offs):4d} {show} (controller conf=1)")
    if offs_all:
        inside = sum(1 for o in offs_all if abs(o) <= 200)
        print(f"\nAll detections: n={len(offs_all)}, within ±200 Hz of entered freq: "
              f"{inside} ({100*inside/len(offs_all):.0f}%)")
        print("Offset distribution:")
        print(hist_line(offs_all, -2000, 2000))


def check4(sessions):
    print("\n" + "=" * 78)
    print("CHECK 4 — NOT measurable from these logs: timestamps are per K-fold cycle")
    print("at 1 s resolution, not per pulse. True PRI needs raw IQ (ipi_analyzer).")
    print("Below is the detection-CYCLE period only, for reference.")
    print("=" * 78)
    for s in sessions:
        groups = defaultdict(list)
        for p in s["pulses"]:
            groups[p["id"]].append(p)
        for tid, ps in sorted(groups.items()):
            # use first pulse of each seq
            seen = {}
            for p in ps:
                if p["seq"] not in seen:
                    seen[p["seq"]] = p["t"]
            if len(seen) < 10:
                continue
            tip = float(s["configs"].get(tid, {}).get("tip", "nan"))
            xs = sorted(seen)
            ts = [seen[x] for x in xs]
            span_seq = xs[-1] - xs[0]
            if span_seq <= 0:
                continue
            n = len(xs)
            mx, mt = sum(xs) / n, sum(ts) / n
            slope = sum((x - mx) * (t - mt) for x, t in zip(xs, ts)) / \
                    sum((x - mx) ** 2 for x in xs)
            print(f"{s['dir'][:52]:52s} id {tid}: n={n:3d} span={span_seq:4d} seq  "
                  f"cycle period={slope:.2f} s  (entered tip={tip:.3f})")


def check5(sessions):
    print("\n" + "=" * 78)
    print("CHECK 5 — noise_psd across sessions (self-interference / heading dependence)")
    print("=" * 78)
    print(f"\n{'session':52s} {'n':>4s} {'median noise_psd':>18s}")
    for s in sessions:
        # controller and detector logs cover the same events (detection and
        # no-detection cycles both carry noise): prefer controller, fall back
        # to detector logs only if the controller parsed nothing
        vals = [p["noise_psd"] for p in s["pulses"]] + s["nodet_noise"]
        if not vals:
            for dets in s["detections"].values():
                vals.extend(d["noise"] for d in dets)
            vals.extend(s["det_nodet_noise"])
        if vals:
            print(f"{s['dir'][:52]:52s} {len(vals):4d} {pctile(vals, 0.5):18.3g}")


def report_data_issues(sessions):
    """Last section so it is the final thing the operator reads: every
    input problem the parsers worked around, and which session it affects."""
    print("\n" + "=" * 78)
    print("DATA ISSUES — inputs that were skipped or partially used above")
    print("=" * 78)
    n = 0
    for s in sessions:
        for issue in s["issues"]:
            print(f"  {s['dir']}: {issue}")
            n += 1
    if n == 0:
        print("  none")
    else:
        print(f"\n  {n} issue(s) in {sum(1 for s in sessions if s['issues'])} session(s); "
              "checks that depended on the missing values reported them as skipped.")


def main():
    root = Path(sys.argv[1] if len(sys.argv) > 1 else ".")
    if not root.is_dir():
        sys.exit(f"Error: data root {root} is not a directory")
    sessions = collect(root)
    if not sessions:
        sys.exit(f"Error: no MavlinkTagController.log found anywhere under {root}")
    n_p = sum(len(s["pulses"]) for s in sessions)
    n_d = sum(len(d) for s in sessions for d in s["detections"].values())
    print(f"Parsed {len(sessions)} sessions, {n_p} controller pulses, "
          f"{n_d} detector DETECTED lines\n")
    check0_and_3(sessions)
    check1(sessions)
    check2(sessions)
    check4(sessions)
    check5(sessions)
    report_data_issues(sessions)


if __name__ == "__main__":
    main()
