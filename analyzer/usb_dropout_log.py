#!/usr/bin/env python3
"""Bench repro for the in-flight Airspy HF+ USB dropouts (docs/ISSUES.md issue 13, S14).

Streams the Airspy with airspyhf_rx and logs:
  - sample rate once a second (a stall shows before the kernel notices),
  - Airspy presence / devnum on USB (a re-enumeration bumps devnum),
  - hub-port over-current count,
  - Pi under-voltage bits (live and sticky), polled 10x a second,
  - kernel USB messages as they arrive (disconnect, error -71, ...).

Type a note and Enter at any time (e.g. "motors 50%") to put a MARK in the log.
Ctrl-C prints a summary. Stop any session using the Airspy first.
"""

import argparse
import collections
import datetime
import glob
import os
import re
import shutil
import signal
import subprocess
import sys
import threading
import time

AIRSPY_VID, AIRSPY_PID = "03eb", "800c"
SAMPLE_RATE = 768000
BYTES_PER_SAMPLE = 8  # complex float32 from airspyhf_rx
RATE_LOW_FRACTION = 0.5  # pipe delivery is bursty; only flag real stalls
RATE_GRACE_S = 2.0
KERNEL_RE = re.compile(r"usb|xhci|hub|volt|throttl|over-?current|error -\d+", re.I)

if sys.stdout.isatty():
    RED, YEL, GRN, OFF = "\033[1;31m", "\033[33m", "\033[32m", "\033[0m"
else:
    RED = YEL = GRN = OFF = ""


def now_str():
    return datetime.datetime.now().astimezone().isoformat(timespec="milliseconds")


def read_sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def find_airspy():
    """Return (sysfs name, devnum) of the first Airspy HF+, or (None, None)."""
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        if (read_sysfs(f"{dev}/idVendor") == AIRSPY_VID
                and read_sysfs(f"{dev}/idProduct") == AIRSPY_PID):
            return os.path.basename(dev), read_sysfs(f"{dev}/devnum")
    return None, None


def over_current_path(name):
    """sysfs over_current_count of the hub port the device at `name` (e.g. 1-1.2) hangs off."""
    if "." in name:
        parent, port = name.rsplit(".", 1)
        hub_if = f"{parent}:1.0"
    else:
        bus, port = name.split("-", 1)
        parent, hub_if = f"usb{bus}", f"{bus}-0:1.0"
    return f"/sys/bus/usb/devices/{parent}/{hub_if}/{parent}-port{port}/over_current_count"


def read_throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=1).stdout
        return int(out.strip().split("=", 1)[1], 16)
    except (OSError, subprocess.SubprocessError, IndexError, ValueError):
        return None


class Log:
    def __init__(self, path):
        self._f = open(path, "w", buffering=1)
        self._lock = threading.Lock()
        self.counts = collections.Counter()

    def event(self, key, msg="", color=""):
        line = f"{now_str()} {key} {msg}".rstrip()
        with self._lock:
            self.counts[key] += 1
            self._f.write(line + "\n")
            print(f"{color}{line}{OFF if color else ''}", flush=True)


class Stream:
    def __init__(self):
        self.lock = threading.Lock()
        self.bytes = 0
        self.proc = None
        self.started = 0.0

    def take_bytes(self):
        with self.lock:
            n, self.bytes = self.bytes, 0
        return n


def stream_loop(args, stream, log, stop):
    cmd = ["airspyhf_rx", "-r", "stdout", "-f", f"{args.freq:.6f}",
           "-a", str(SAMPLE_RATE), "-g", "off", "-m", "on"]
    while not stop.is_set():
        if find_airspy()[0] is None:
            time.sleep(0.2)
            continue
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # started first: the main thread reads started only once proc is set
        stream.started = time.monotonic()
        stream.proc = proc
        log.event("STREAM_START", f"pid {proc.pid} {args.freq:.6f} MHz")
        err_tail = collections.deque(maxlen=10)

        def read_stderr():
            for raw in proc.stderr:
                text = raw.decode(errors="replace").rstrip()
                if text:
                    err_tail.append(text)
                    log.event("AIRSPYHF_RX", text)

        t = threading.Thread(target=read_stderr, daemon=True)
        t.start()
        while True:
            chunk = proc.stdout.read1(65536)
            if not chunk:
                break
            with stream.lock:
                stream.bytes += len(chunk)
        rc = proc.wait()
        t.join(timeout=1)
        stream.proc = None
        if stop.is_set():
            break
        log.event("STREAM_EXIT", f"rc={rc}", RED)
        time.sleep(1)


def kernel_loop(log, stop, procs):
    proc = subprocess.Popen(
        ["journalctl", "-k", "-f", "-n", "0", "-o", "short-iso-precise", "--no-hostname"],
        stdout=subprocess.PIPE, text=True)
    procs.append(proc)
    for line in proc.stdout:
        if stop.is_set():
            break
        line = line.rstrip()
        if not KERNEL_RE.search(line):
            continue
        if "disconnect" in line.lower():
            log.event("KERNEL_DISCONNECT", line, RED)
        elif re.search(r"error -\d+|bad\?|over-?current|volt", line, re.I):
            log.event("KERNEL_ERROR", line, RED)
        else:
            log.event("KERNEL", line, YEL)
    if not stop.is_set():
        log.event("KERNEL_LOG_ENDED", f"journalctl exited rc={proc.wait()}", RED)


def stdin_loop(log):
    for line in sys.stdin:
        if line.strip():
            log.event("MARK", line.strip(), GRN)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("label", nargs="?", default="bench", help="name used in the output file names")
    p.add_argument("--freq", type=float, default=146.0, help="tune frequency in MHz (default 146.0)")
    p.add_argument("--out-dir", default=os.path.expanduser("~"), help="output directory (default ~)")
    args = p.parse_args()

    missing = [tool for tool in ("airspyhf_rx", "journalctl") if shutil.which(tool) is None]
    if missing:
        sys.exit(f"not found on PATH: {', '.join(missing)}")

    # Without journal access journalctl still runs but shows no kernel lines
    probe = subprocess.run(["journalctl", "-k", "-n", "1", "-q", "--no-pager"],
                           capture_output=True, text=True)
    if not probe.stdout.strip():
        sys.exit("cannot read the kernel journal (add the user to the adm group)")

    # Not -x: Linux truncates process names to 15 characters, so match the command line instead
    if subprocess.run(["pgrep", "-f", "(^|/)airspyhf_zeromq_rx( |$)"], capture_output=True).returncode == 0:
        sys.exit("airspyhf_zeromq_rx is running and holds the Airspy; stop the session first")

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = os.path.join(args.out_dir, f"usbdrop_{args.label}_{stamp}")
    log = Log(base + ".log")
    csv = open(base + ".csv", "w", buffering=1)
    csv.write("time,ksps,airspy_present,devnum,throttled,uv_now_in_second,uv_sticky,over_current_count\n")
    print(f"Logging to {base}.log / .csv  (type a note + Enter to mark, Ctrl-C to stop)")

    name, devnum = find_airspy()
    if name is None:
        log.event("AIRSPY_ABSENT", "not on USB at start", RED)
    else:
        log.event("AIRSPY_PRESENT", f"{name} devnum {devnum}")
    oc_path = over_current_path(name) if name else None
    oc = read_sysfs(oc_path) if oc_path else None

    th = read_throttled()
    if th is None:
        log.event("THROTTLED_UNREADABLE", "vcgencmd get_throttled failed", RED)
    elif th & 0x10000:
        log.event("UV_STICKY_ALREADY_SET",
                  f"0x{th:x}: under-voltage seen since boot; reboot to re-arm the sticky bit", YEL)

    stop = threading.Event()
    stream = Stream()
    procs = []
    threading.Thread(target=stream_loop, args=(args, stream, log, stop), daemon=True).start()
    threading.Thread(target=kernel_loop, args=(log, stop, procs), daemon=True).start()
    if sys.stdin.isatty():
        threading.Thread(target=stdin_loop, args=(log,), daemon=True).start()

    min_ksps = None
    last_th = th
    last_row = time.monotonic()
    next_row = last_row + 1.0
    uv_in_second = 0
    # last_th keeps the last good reading for change detection; the CSV only reports a second with a fresh one
    th_read_in_second = False
    th_failing = th is None
    try:
        while True:
            time.sleep(0.1)
            cur_th = read_throttled()
            if cur_th is None:
                if not th_failing:
                    log.event("THROTTLED_UNREADABLE", "vcgencmd get_throttled failed", RED)
                    th_failing = True
            else:
                if th_failing:
                    log.event("THROTTLED_RECOVERED", f"throttled=0x{cur_th:x}", GRN)
                    th_failing = False
                th_read_in_second = True
                uv_in_second |= cur_th & 0x1
                if last_th is not None:
                    if (cur_th & 0x1) and not (last_th & 0x1):
                        log.event("UV_NOW", f"throttled=0x{cur_th:x}", RED)
                    elif not (cur_th & 0x1) and (last_th & 0x1):
                        log.event("UV_CLEARED", f"throttled=0x{cur_th:x}", GRN)
                    if (cur_th & 0x10000) and not (last_th & 0x10000):
                        log.event("UV_STICKY_SET", f"throttled=0x{cur_th:x}", RED)
                last_th = cur_th

            cur_name, cur_devnum = find_airspy()
            if cur_name != name or cur_devnum != devnum:
                if cur_name is None:
                    log.event("AIRSPY_GONE", f"was {name} devnum {devnum}", RED)
                else:
                    new_oc_path = over_current_path(cur_name)
                    note = ""
                    if new_oc_path != oc_path:
                        # Another port's count is not comparable; keep the baseline on a same-port reconnect
                        oc = read_sysfs(new_oc_path)
                        if oc_path is not None:
                            note = f" (new port, over_current_count {oc})"
                        oc_path = new_oc_path
                    log.event("AIRSPY_BACK", f"{cur_name} devnum {cur_devnum}{note}", YEL)
                name, devnum = cur_name, cur_devnum
            cur_oc = read_sysfs(oc_path) if oc_path else None
            if cur_oc != oc and oc is not None and cur_oc is not None:
                log.event("OVER_CURRENT", f"{oc_path}: {oc} -> {cur_oc}", RED)
            oc = cur_oc if cur_oc is not None else oc

            if time.monotonic() < next_row:
                continue
            next_row += 1.0
            t_now = time.monotonic()
            ksps = stream.take_bytes() / BYTES_PER_SAMPLE / 1000 / (t_now - last_row)
            last_row = t_now
            streaming_s = time.monotonic() - stream.started if stream.proc else 0
            if streaming_s > RATE_GRACE_S:
                min_ksps = ksps if min_ksps is None else min(min_ksps, ksps)
                if ksps < RATE_LOW_FRACTION * SAMPLE_RATE / 1000:
                    log.event("RATE_LOW", f"{ksps:.1f} ksps", RED)
            if th_read_in_second:
                th_cols = f"0x{last_th:x},{uv_in_second},{(last_th >> 16) & 1}"
            else:
                th_cols = ",,"
            csv.write(f"{now_str()},{ksps:.1f},{int(name is not None)},{devnum or ''},"
                      f"{th_cols},{oc or ''}\n")
            uv_in_second = 0
            th_read_in_second = False
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        stop.set()
        # The stream thread clears stream.proc when airspyhf_rx exits on the same Ctrl-C
        proc = stream.proc
        if proc:
            proc.terminate()
        for proc in procs:
            proc.terminate()

    c = log.counts
    print("\n--- Summary ---")
    for key in ("STREAM_EXIT", "AIRSPY_GONE", "KERNEL_DISCONNECT", "KERNEL_ERROR", "KERNEL_LOG_ENDED",
                "RATE_LOW", "UV_NOW", "UV_STICKY_SET", "THROTTLED_UNREADABLE", "OVER_CURRENT", "MARK"):
        print(f"  {key:20s} {c[key]}")
    print(f"  min rate             {min_ksps:.1f} ksps" if min_ksps is not None else "  min rate             n/a")
    print(f"  files                {base}.log, {base}.csv")


if __name__ == "__main__":
    main()
