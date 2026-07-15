#!/usr/bin/env python3
"""sensor_diag_logger.py -- find out WHY a perception stream stalls/drops.

Measurement tool, NOT part of the safety path. Run it for a few hours WHILE the
robot operates normally; it touches nothing the robot uses (read-only subscribers
+ read-only system polling). The watchdog has tripped on both the ZED odom
frequency and the lidars; this logger records everything needed to pin the cause.

It monitors FOUR streams at once so a stall is self-diagnosing (zed_odom was
retired 2026-07-15 with IMU-only ZED -- depth+tracking off, VIO odom never
publishes; zed_imu is the ZED health signal):

  * /zed_mini/zed_node/imu/data  (zed_imu)   -- raw off the sensor, ~200 Hz
  * /lidar_r/scan                (lidar_r)   -- right RPLIDAR A1, ~8 Hz
  * /lidar_l/scan                (lidar_l)   -- left  RPLIDAR A1, ~8 Hz
  * /camera/color/camera_info    (realsense) -- D435i frame heartbeat, ~30 Hz

How to read a stall from WHICH streams stop together:
  * ZED IMU stops                  -> ZED node/USB/camera dropped (hardware/bw).
  * One lidar only                 -> that lidar's USB/serial/cable/motor.
  * Both lidars together           -> shared USB hub/power, or CPU starvation.
  * Lidars + ZED all together      -> system-wide: USB bus, power, or CPU.

For every message it separates ARRIVAL gaps (wall clock, when WE got it) from
HEADER-STAMP gaps (when the driver said it was produced): if header stamps stay
continuous but arrivals gap, the transport/subscriber stalled, not the sensor.

Stalls are correlated against the usual suspects, sampled the whole run:
  * USB kernel events (journalctl -kf / dmesg: disconnect/reset/xhci) -- best effort
  * CPU% + load average, and GPU util/temp/power (nvidia-smi) -- best effort

Outputs (one timestamped dir per run, default under integration/log/):
  * samples.csv      -- ~1 Hz: per-topic Hz, worst arrival gap, CPU, load, GPU
  * gpu_procs.csv    -- per-process GPU (sm%, fb MB) every few s, each PID
                        classified zed/rviz/flair/realsense (catches RViz, which
                        --query-compute-apps omits)
  * usb_topology.txt -- lsusb tree + labeled device->controller map at start
  * events.log       -- dropouts + KERNEL usb lines + USB_TOPO connect/
                        disconnect/re-enum (port + controller labeled) + SKILL
                        transitions, in wall-clock order so you can read the story
  * incidents/*.txt  -- on each ZED dropout: the Stereolabs devices' sysfs
                        power/autosuspend state, lsusb tree, kernel tail --
                        captured within ~100ms of the stall being detected
  * run_meta.json    -- start/end, args, per-topic totals, output index

Usage (let it run, Ctrl+C stops early and still writes the summary):
    python sensor_diag_logger.py --duration 14400      # 4 hours

Reading the kernel log may need privileges (kernel.dmesg_restrict); if so, run
with sudo or pass --no-dmesg. Everything else works without root.
"""

import argparse
import json
import math
import os
import re
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rospy
from sensor_msgs.msg import Imu, LaserScan, CameraInfo
from std_msgs.msg import String


# A dropout is an inter-message arrival gap big enough to mean a real stall
# started (not just scheduling jitter). The threshold is PER SENSOR, derived
# from its nominal rate: DROPOUT_PERIODS missed periods, but never below
# DROPOUT_FLOOR_S. A flat threshold mis-fires -- at 8 Hz (0.125 s period) a
# single jittery scan is 0.25 s, which a flat 0.25 s would log as a "dropout".
DROPOUT_PERIODS = 3.0      # >= this many missed periods => dropout
DROPOUT_FLOOR_S = 0.12     # but never trip below this (guards high-rate IMU)

# Header stamps that jump more than this (seconds) are treated as a clock/epoch
# artifact -- e.g. a node restart re-bases its stamps -- not a real stamp gap.
# Without this guard the first message after a restart logs a ~1.7e9 s gap
# (new stamp minus the pre-restart stamp) and poisons the max-stamp-gap column.
STAMP_SANITY_S = 60.0

# A rospy.Subscriber created before a publisher restarts does not always reconnect
# to the new publisher, so a stream can read ~0 Hz here while it is actually live
# (a fresh subscriber, e.g. the watchdog, sees it fine). If a stream is silent this
# many consecutive sample windows WHILE at least one other stream is alive (so it is
# not a global rosmaster/shutdown outage), recreate its subscriber. Sized in windows
# (~1 s each at the default --interval), so a brief dropout does not needlessly resub.
RESUBSCRIBE_AFTER_DEAD_WINDOWS = 5


# Streams to monitor: key, default topic, ROS msg type, subscriber queue size,
# nominal Hz (for the per-sensor dropout threshold), human description. Add a
# row here to watch another topic -- CSV columns, events, and summary adapt.
STREAMS = [
    # zed_odom row removed [2026-07-15]: IMU-only ZED (depth+tracking off, see
    # sensors.launch) -- VIO odom never publishes, and a subscriber to it makes
    # the wrapper WARN every grab cycle. zed_imu is the ZED health signal now.
    ("zed_imu",  "/zed_mini/zed_node/imu/data", Imu,       4000, 200.0, "ZED raw IMU ~200 Hz"),
    ("lidar_r",  "/lidar_r/scan",               LaserScan,  200,   8.0, "Right RPLIDAR ~8 Hz"),
    ("lidar_l",  "/lidar_l/scan",               LaserScan,  200,   8.0, "Left RPLIDAR ~8 Hz"),
    ("realsense","/camera/color/camera_info",   CameraInfo, 200,  30.0, "RealSense color caminfo ~30 Hz"),
]


# Kernel lines worth keeping. Word-boundary anchored on purpose: a bare "zed"
# substring matches "initiali[zed]", which floods the log with ACPI BIOS-method
# spam ("No Local Variables are initialized for Method [SNTM]") firing ~2x/s.
KERNEL_RE = re.compile(
    r"\b(usb\w*|xhci\w*|uvc\w*|cp210\w*|ftdi\w*|tty\w*|zed-?m\w*|stereolabs)\b",
    re.IGNORECASE,
)


class TopicMonitor:
    """Tracks rate and gaps for one topic. Callback-thread safe."""

    def __init__(self, name, event_log, drop_gap, snapshotter=None):
        self.name = name
        self._event_log = event_log
        self._drop_gap = drop_gap       # per-sensor dropout threshold (seconds)
        self._snapshotter = snapshotter  # incident snapshot on dropout (ZED streams)
        self._lock = threading.Lock()
        self.total = 0
        self._window_count = 0          # messages since last sample()
        self._last_arrival = None       # wall time of previous message
        self._last_stamp = None         # header stamp (s) of previous message
        self._max_gap = 0.0             # worst arrival gap since last sample()
        self._max_stamp_gap = 0.0       # worst header-stamp gap since last sample()
        self.dropouts = 0
        self.worst_gap = 0.0            # worst arrival gap over the whole run

    def on_msg(self, msg):
        now = time.time()
        try:
            stamp = msg.header.stamp.to_sec()
        except AttributeError:
            stamp = None
        with self._lock:
            self.total += 1
            self._window_count += 1
            if self._last_arrival is not None:
                gap = now - self._last_arrival
                if gap > self._max_gap:
                    self._max_gap = gap
                if gap > self.worst_gap:
                    self.worst_gap = gap
                if gap >= self._drop_gap:
                    self.dropouts += 1
                    # Sanitize the stamp gap: NaN if unknown or an epoch jump
                    # (restart). NaN -> we can't call transport-vs-source.
                    if stamp is not None and self._last_stamp is not None:
                        sg = stamp - self._last_stamp
                        stamp_gap = sg if 0.0 <= sg < STAMP_SANITY_S else float("nan")
                    else:
                        stamp_gap = float("nan")
                    if math.isnan(stamp_gap):
                        cls = "resync/restart (stamp reset)"
                    elif stamp_gap < gap / 2:
                        cls = "header continuous->transport stall"
                    else:
                        cls = "header also gapped->source stall"
                    self._event_log.write(
                        f"DROPOUT {self.name:8s} arrival_gap={gap:6.3f}s "
                        f"stamp_gap={stamp_gap:6.3f}s ({cls})"
                    )
                    if self._snapshotter is not None:
                        self._snapshotter.trigger(
                            self.name, f"arrival_gap={gap:.3f}s ({cls})")
            if stamp is not None:
                if self._last_stamp is not None:
                    sg = stamp - self._last_stamp
                    # Ignore epoch jumps so a restart doesn't poison the max.
                    if 0.0 <= sg < STAMP_SANITY_S and sg > self._max_stamp_gap:
                        self._max_stamp_gap = sg
                self._last_stamp = stamp
            self._last_arrival = now

    def sample(self, dt):
        """Return (hz, max_arrival_gap, max_stamp_gap) and reset the window."""
        with self._lock:
            hz = self._window_count / dt if dt > 0 else 0.0
            max_gap = self._max_gap
            max_stamp_gap = self._max_stamp_gap
            self._window_count = 0
            self._max_gap = 0.0
            self._max_stamp_gap = 0.0
        return hz, max_gap, max_stamp_gap


class EventLog:
    """Append-only, timestamped, thread-safe text log."""

    def __init__(self, path):
        self._f = open(path, "a", buffering=1)  # line-buffered
        self._lock = threading.Lock()
        self.lines = 0

    def write(self, line):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with self._lock:
            self._f.write(f"{ts}  {line}\n")
            self.lines += 1

    def close(self):
        self._f.close()


class CpuSampler:
    """Overall CPU% from /proc/stat deltas + load average. No deps."""

    def __init__(self):
        self._prev = self._read()

    @staticmethod
    def _read():
        with open("/proc/stat") as f:
            parts = f.readline().split()[1:]   # user nice system idle iowait ...
        vals = [int(x) for x in parts]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return sum(vals), idle

    def sample(self):
        total, idle = self._read()
        dt_total = total - self._prev[0]
        dt_idle = idle - self._prev[1]
        self._prev = (total, idle)
        cpu = 100.0 * (1.0 - dt_idle / dt_total) if dt_total > 0 else float("nan")
        try:
            load1 = os.getloadavg()[0]
        except OSError:
            load1 = float("nan")
        return cpu, load1


def sample_gpu():
    """(util%, temp C, power W) via nvidia-smi, or NaNs if unavailable."""
    try:
        out = subprocess.check_output(
            ["nvidia-smi",
             "--query-gpu=utilization.gpu,temperature.gpu,power.draw",
             "--format=csv,noheader,nounits"],
            stderr=subprocess.DEVNULL, timeout=2.0,
        ).decode().strip().splitlines()[0]
        util, temp, power = (x.strip() for x in out.split(","))
        return float(util), float(temp), float(power)
    except Exception:
        return float("nan"), float("nan"), float("nan")


def resubscribe_stale_streams(hz_by_key, dead_windows, stream_subs, event_log):
    """Self-heal subscribers that go silent after a mid-run driver relaunch.

    A stream read at ~0 Hz here is not always really dead: a subscriber created
    before the publisher restarted may hold a stale connection while the topic is
    live (this is what made realsense log 0.135 Hz while the watchdog saw 30 Hz).
    Recreate the subscriber once a stream is silent for RESUBSCRIBE_AFTER_DEAD_WINDOWS
    consecutive windows while at least one other stream is alive -- the "others
    alive" guard avoids churning subscribers during a genuine rosmaster/shutdown
    outage (when reconnecting cannot help anyway)."""
    any_alive = any(hz > 0 for hz in hz_by_key.values())
    for key, hz in hz_by_key.items():
        if hz > 0:
            dead_windows[key] = 0
            continue
        dead_windows[key] += 1
        if dead_windows[key] < RESUBSCRIBE_AFTER_DEAD_WINDOWS or not any_alive:
            continue
        info = stream_subs.get(key)
        if info is None:
            continue
        try:
            info["sub"].unregister()
            info["sub"] = rospy.Subscriber(
                info["topic"], info["msg_type"], info["monitor"].on_msg,
                queue_size=info["queue"])
            event_log.write(
                f"RESUB  {key} silent {dead_windows[key]} windows while others live "
                f"-> recreated subscriber to {info['topic']} (stale after relaunch?)")
        except Exception as e:
            event_log.write(f"RESUB  {key} FAILED: {e}")
        dead_windows[key] = 0


def follow_kernel_usb(event_log, stop_evt, use_dmesg):
    """Stream USB/xhci kernel lines into the event log. Best effort.

    Prefers `journalctl -kf -n0`, which follows NEW kernel messages with no
    backlog. `dmesg --follow` is the fallback: it replays the entire ring
    buffer first (util-linux 2.34 here has no --follow-new), but KERNEL_RE
    keeps that replay to genuine USB lines so it's just useful history.
    """
    if not use_dmesg:
        return
    for cmd in (["journalctl", "-kf", "-n", "0", "-o", "short"],
                ["dmesg", "--follow", "--ctime"]):
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, text=True)
        except FileNotFoundError:
            continue
        event_log.write(f"INFO  following kernel log via: {' '.join(cmd)}")
        try:
            for line in proc.stdout:
                if stop_evt.is_set():
                    break
                if KERNEL_RE.search(line):
                    event_log.write("KERNEL " + line.rstrip())
        finally:
            proc.terminate()
        return
    event_log.write("WARN  no kernel log source available (journalctl/dmesg); "
                    "USB events will NOT be captured (try sudo, or --no-dmesg)")


# ---------------------------------------------------------------------------
# Per-process GPU: who is actually on the GPU (answers Run-2's "what saturates
# the GPU when ZED odom stalls"). Streams `nvidia-smi pmon`, which lists BOTH
# compute (C) and GRAPHICS (G) clients -- the latter (e.g. RViz) are invisible
# to `--query-compute-apps`. Each PID is classified from its /proc cmdline.
# ---------------------------------------------------------------------------
class GpuProcSampler:
    # First match wins. Robot ROS nodes first, then desktop apps (so VS Code /
    # Slack / browser GPU-helper processes are caught BEFORE the FLAIR rule),
    # then FLAIR -- whose pattern is robot-specific tokens only. Generic words
    # like "inference"/"preference" are deliberately NOT used: they match
    # VS Code / Slack cmdlines and would inflate the FLAIR GPU attribution.
    LABELS = [
        (re.compile(r"rviz", re.I), "rviz"),
        (re.compile(r"zed_wrapper|zed_node|\bzed\b", re.I), "zed"),
        (re.compile(r"realsense|rs_camera", re.I), "realsense"),
        (re.compile(r"cartographer", re.I), "cartographer"),
        (re.compile(r"move_base|amcl|costmap", re.I), "nav"),
        (re.compile(r"/code|vscode|code-server", re.I), "vscode"),
        (re.compile(r"slack", re.I), "slack"),
        (re.compile(r"firefox|chrome|chromium", re.I), "browser"),
        (re.compile(r"Xorg|gnome-shell|gdm|mutter|plasma", re.I), "desktop"),
        # The single orchestrator (FLAIR/SAM/etc. all run inside it, not
        # standalone), so label it by its entry point rather than a sub-module.
        (re.compile(r"run\.py|--run_on_robot|food_manip|spanet", re.I), "run.py"),
    ]

    def __init__(self, csv_file, t0):
        self._csv = csv_file
        self._t0 = t0
        self._cmd = {}          # pid -> (label, cmdline), cached
        self.rows = 0
        self.labels_seen = set()

    def _classify(self, pid):
        if pid in self._cmd:
            return self._cmd[pid]
        try:
            raw = (open(f"/proc/{pid}/cmdline", "rb").read()
                   .replace(b"\x00", b" ").decode("utf-8", "replace").strip())
        except OSError:
            raw = ""
        label = "other"
        for rx, lab in self.LABELS:
            if rx.search(raw):
                label = lab
                break
        res = (label, raw[:160])
        self._cmd[pid] = res
        return res

    def run(self, stop_evt, interval):
        cmd = ["nvidia-smi", "pmon", "-s", "um", "-d", str(max(1, int(interval)))]
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, text=True)
        except FileNotFoundError:
            return                       # no nvidia-smi -> silently skip
        colidx = {}
        try:
            for line in proc.stdout:
                if stop_evt.is_set():
                    break
                s = line.strip()
                if s.startswith("# gpu"):           # column header
                    colidx = {n: i for i, n in enumerate(s.lstrip("# ").split())}
                    continue
                if not s or s.startswith("#"):
                    continue
                f = s.split()

                def col(name):
                    i = colidx.get(name)
                    return f[i] if (i is not None and i < len(f)) else "-"

                pid = col("pid")
                if not pid.isdigit():               # idle slot ("-")
                    continue
                label, full = self._classify(pid)
                self.labels_seen.add(label)
                self._csv.write(
                    f"{datetime.now().isoformat()},{time.time() - self._t0:.1f},"
                    f"{pid},{col('type')},{col('sm')},{col('mem')},{col('fb')},"
                    f'{label},"{full}"\n')
                self.rows += 1
        finally:
            proc.terminate()


# ---------------------------------------------------------------------------
# USB topology: snapshot the bus at start, then poll sysfs and log every
# CONNECT / DISCONNECT / RE-ENUM (a devnum change at the same port = a USB
# reset). Independent of the kernel log, so silent drops are caught too, and
# every event names the controller -- so we can tell a fault on the isolated
# TB4 port apart from one on the PCH that also carries the lidars.
# ---------------------------------------------------------------------------
SYS_USB = "/sys/bus/usb/devices"

KNOWN_USB = {
    "2b03:f682": "ZED-M video/depth", "2b03:f681": "ZED-M HID/IMU",
    "8086:0b3a": "RealSense D435i", "10c4:ea60": "CP210x lidar",
    "0403:6014": "FTDI ttyUSB0", "045e:0b12": "Xbox controller",
    "303a:80b5": "FeatherS2 LED", "0bda:5411": "Realtek hub (USB2)",
    "0bda:0411": "Realtek hub (USB3)", "1d6b:0002": "root hub USB2",
    "1d6b:0003": "root hub USB3", "8087:0033": "Intel BT",
    "048d:c998": "ITE kbd", "30c9:00ac": "webcam",
}
# Controller PCI address -> label, so an event reads "[PCH (lidars)]" etc.
CTRL_LABEL = {
    "0000:00:14.0": "PCH (lidars)",
    "0000:39:00.0": "TB4 (isolated)",
    "0000:05:00.0": "TB4-domain",
}


def _rd(path):
    try:
        return open(path).read().strip()
    except OSError:
        return ""


def _controller_of(devname):
    """PCI address of the xHCI controller a USB device hangs off."""
    for part in reversed(os.path.realpath(f"{SYS_USB}/{devname}").split("/")):
        if part.count(":") == 2 and "." in part:     # e.g. 0000:39:00.0
            return part
    return "?"


def scan_usb():
    """{port: {vidpid, devnum, speed, label, ctrl}} for real devices (not ifaces)."""
    out = {}
    try:
        names = os.listdir(SYS_USB)
    except OSError:
        return out
    for n in names:
        if ":" in n:                      # interface, e.g. 1-4.4.3:1.0
            continue
        vid, pid = _rd(f"{SYS_USB}/{n}/idVendor"), _rd(f"{SYS_USB}/{n}/idProduct")
        if not vid:
            continue
        vp = f"{vid}:{pid}"
        out[n] = {
            "vidpid": vp, "devnum": _rd(f"{SYS_USB}/{n}/devnum"),
            "speed": _rd(f"{SYS_USB}/{n}/speed"),
            "label": KNOWN_USB.get(vp, _rd(f"{SYS_USB}/{n}/product") or "?"),
            "ctrl": _controller_of(n),
        }
    return out


def write_usb_topology(path):
    """Write lsusb tree/list + a labeled device->controller map at run start."""
    lines = ["# USB topology at run start", ""]
    for title, cmd in (("lsusb -t", ["lsusb", "-t"]), ("lsusb", ["lsusb"]),
                       ("/dev/serial/by-path", ["ls", "-l", "/dev/serial/by-path"])):
        lines.append(f"===== {title} =====")
        try:
            lines.append(subprocess.check_output(
                cmd, text=True, stderr=subprocess.DEVNULL).rstrip())
        except Exception as e:
            lines.append(f"(failed: {e})")
        lines.append("")
    lines.append("===== devices by controller =====")
    devs = scan_usb()
    for n in sorted(devs, key=lambda p: (devs[p]["ctrl"], len(p), p)):
        d = devs[n]
        cl = CTRL_LABEL.get(d["ctrl"], d["ctrl"])
        lines.append(f"  {n:12} {d['vidpid']}  {d['speed'] or '?':>5}M  [{cl}]  {d['label']}")
    Path(path).write_text("\n".join(lines) + "\n")
    return devs


class UsbTopologyWatcher:
    """Diffs successive sysfs snapshots -> CONNECT/DISCONNECT/RE-ENUM events."""

    def __init__(self, event_log, baseline):
        self._log = event_log
        self._prev = {n: d["devnum"] for n, d in baseline.items()}
        self._meta = dict(baseline)
        self.changes = 0

    def _ev(self, msg):
        self.changes += 1
        self._log.write("USB_TOPO " + msg)

    def poll(self):
        cur = scan_usb()
        for n, d in cur.items():
            cl = CTRL_LABEL.get(d["ctrl"], d["ctrl"])
            if n not in self._prev:
                self._ev(f"CONNECT    {d['label']} at {n} [{cl}] dev#{d['devnum']}")
            elif d["devnum"] != self._prev[n]:
                self._ev(f"RE-ENUM    {d['label']} at {n} [{cl}] "
                         f"dev#{self._prev[n]}->#{d['devnum']} (USB reset)")
            self._meta[n] = d
        for n in self._prev:
            if n not in cur:
                m = self._meta.get(n, {})
                cl = CTRL_LABEL.get(m.get("ctrl"), m.get("ctrl", "?"))
                self._ev(f"DISCONNECT {m.get('label', '?')} at {n} [{cl}] "
                         f"(was dev#{self._prev[n]})")
        self._prev = {n: d["devnum"] for n, d in cur.items()}


def watch_usb_topology(watcher, stop_evt, interval):
    while not stop_evt.is_set():
        try:
            watcher.poll()
        except Exception:
            pass
        stop_evt.wait(interval)


# ---------------------------------------------------------------------------
# Incident snapshots: a ZED stall is rare and over in seconds, so capture the
# USB state THE MOMENT a ZED dropout fires -- the sysfs power/autosuspend state
# of the Stereolabs devices (the prime suspect for silent SDK grab hangs; no
# kernel line is logged when a device merely autosuspends), the lsusb tree, and
# the last kernel lines. One file per incident under incidents/, referenced
# from events.log. Rate-limited, and runs in its own thread so the subscriber
# callback that detected the gap is never blocked.
# ---------------------------------------------------------------------------
ZED_VID = "2b03"


class IncidentSnapshotter:
    def __init__(self, outdir, event_log, min_period_s=10.0):
        self._dir = Path(outdir) / "incidents"
        self._log = event_log
        self._min_period = min_period_s
        self._last = 0.0
        self._lock = threading.Lock()
        self.count = 0

    def trigger(self, stream, reason):
        with self._lock:
            now = time.time()
            if now - self._last < self._min_period:
                return
            self._last = now
        threading.Thread(target=self._capture, args=(stream, reason),
                         daemon=True).start()

    @staticmethod
    def _zed_power_states():
        lines = []
        try:
            names = os.listdir(SYS_USB)
        except OSError:
            return ["(sysfs unavailable)"]
        for n in sorted(names):
            if ":" in n or _rd(f"{SYS_USB}/{n}/idVendor") != ZED_VID:
                continue
            base = f"{SYS_USB}/{n}"
            lines.append(
                f"{n}  {_rd(base + '/idVendor')}:{_rd(base + '/idProduct')}"
                f"  ({_rd(base + '/product') or '?'})"
                f"  devnum={_rd(base + '/devnum')}"
                f"  speed={_rd(base + '/speed')}M"
                f"  power/control={_rd(base + '/power/control')}"
                f"  runtime_status={_rd(base + '/power/runtime_status')}"
                f"  autosuspend_delay_ms={_rd(base + '/power/autosuspend_delay_ms')}"
            )
        return lines or ["(no Stereolabs %s device on the bus -- ZED enumerated "
                         "away entirely?)" % ZED_VID]

    def _capture(self, stream, reason):
        ts = datetime.now()
        sections = [f"# incident snapshot: {stream} {reason}",
                    f"# taken {ts.isoformat()} (snapshot lag <~100ms after detection)",
                    "", "===== ZED (Stereolabs) sysfs power state ====="]
        sections += self._zed_power_states()
        for title, cmd in (
            ("lsusb -t", ["lsusb", "-t"]),
            ("kernel tail", ["journalctl", "-k", "-n", "40", "--no-pager",
                             "-o", "short-iso"]),
        ):
            sections += ["", f"===== {title} ====="]
            try:
                sections.append(subprocess.check_output(
                    cmd, text=True, stderr=subprocess.DEVNULL,
                    timeout=5.0).rstrip())
            except Exception as e:
                sections.append(f"(failed: {e})")
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
            path = self._dir / f"{ts.strftime('%H%M%S')}_{stream}.txt"
            path.write_text("\n".join(sections) + "\n")
            self.count += 1
            self._log.write(f"SNAPSHOT {stream} -> incidents/{path.name} ({reason})")
        except Exception as e:
            self._log.write(f"SNAPSHOT {stream} FAILED: {e}")


class SkillTracker:
    """Logs the currently-executing skill whenever it changes.

    Subscribes to /skill_plan -- the same latched topic the webapp reads to show
    the user the active skill. Payload is JSON {"plan": [names...], "current": i}.
    Logging the skill on change drops it into the wall-clock timeline next to
    dropouts / GPU / USB events, so a stall can be tied to whatever skill was
    running (e.g. did odom always die during navigation vs acquisition?).
    """

    def __init__(self, event_log):
        self._log = event_log
        self._last = None
        self.changes = 0
        self.current = "(none)"

    def on_msg(self, msg):
        try:
            d = json.loads(msg.data)
            plan, cur = d.get("plan", []), d.get("current", -1)
            if isinstance(plan, list) and 0 <= cur < len(plan):
                skill = str(plan[cur])
                tag = f"{skill}  [{cur + 1}/{len(plan)}]"
            else:
                skill = tag = "(idle/cleared)"
        except Exception:
            skill = tag = f"(unparsed: {str(msg.data)[:60]})"
        if tag != self._last:
            self._last = tag
            self.current = skill
            self.changes += 1
            self._log.write(f"SKILL  {tag}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=14400.0,
                    help="seconds to run (default 14400 = 4 h)")
    ap.add_argument("--zed-imu-topic", default="/zed_mini/zed_node/imu/data")
    ap.add_argument("--lidar-r-topic", default="/lidar_r/scan")
    ap.add_argument("--lidar-l-topic", default="/lidar_l/scan")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="sample/CSV row period in seconds (default 1.0)")
    ap.add_argument("--outdir", default=None,
                    help="output dir (default: integration/log/sensor_diag_<timestamp>/)")
    ap.add_argument("--no-dmesg", action="store_true",
                    help="don't try to read the kernel log (skip USB capture)")
    ap.add_argument("--realsense-topic", default="/camera/color/camera_info",
                    help="RealSense topic to watch (lightweight camera_info)")
    ap.add_argument("--gpu-interval", type=float, default=2.0,
                    help="per-process GPU (nvidia-smi pmon) sample period s")
    ap.add_argument("--usb-poll", type=float, default=2.0,
                    help="USB topology poll period s")
    ap.add_argument("--no-gpu-procs", action="store_true",
                    help="don't sample per-process GPU usage")
    ap.add_argument("--skill-topic", default="/skill_plan",
                    help="latched topic carrying the active skill (webapp's /skill_plan)")
    args = ap.parse_args()

    # Resolve per-stream topics from CLI overrides (key -> arg attribute).
    topic_overrides = {
        "zed_imu": args.zed_imu_topic,
        "lidar_r": args.lidar_r_topic,
        "lidar_l": args.lidar_l_topic,
        "realsense": args.realsense_topic,
    }

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.outdir:
        outdir = Path(args.outdir)
    else:
        outdir = Path(__file__).parent.parent / "integration" / "log" / f"sensor_diag_{stamp}"
    outdir.mkdir(parents=True, exist_ok=True)
    print(f"[sensor_diag] writing to {outdir}")

    event_log = EventLog(outdir / "events.log")
    samples_path = outdir / "samples.csv"
    samples = open(samples_path, "w", buffering=1)

    # USB topology snapshot at start + a live per-port connect/disconnect watcher.
    usb_devs = write_usb_topology(outdir / "usb_topology.txt")
    n_ctrl = len({d["ctrl"] for d in usb_devs.values()})
    event_log.write(f"INFO  usb topology: {len(usb_devs)} devices on {n_ctrl} "
                    f"controllers (see usb_topology.txt)")
    usb_watcher = UsbTopologyWatcher(event_log, usb_devs)

    # Per-process GPU log (nvidia-smi pmon), unless disabled.
    gpu_csv = None
    gpu_sampler = None
    if not args.no_gpu_procs:
        gpu_csv = open(outdir / "gpu_procs.csv", "w", buffering=1)
        gpu_csv.write("wall_iso,elapsed_s,pid,type,sm_pct,mem_pct,fb_mb,label,cmd\n")

    rospy.init_node("sensor_diag_logger", anonymous=True, disable_signals=True)

    # Incident snapshots for ZED dropouts only: the lidars' failure modes are
    # already understood (hub/serial), and the ZED's are the silent SDK stalls
    # we're hunting. Rate-limited inside the snapshotter.
    snapshotter = IncidentSnapshotter(outdir, event_log)

    # Build a monitor per stream (with its own dropout threshold) and subscribe.
    monitors = []          # list of (key, TopicMonitor, topic, drop_gap)
    stream_subs = {}       # key -> live subscriber + info to recreate it (self-heal)
    for key, default_topic, msg_type, queue, nominal_hz, _desc in STREAMS:
        topic = topic_overrides.get(key, default_topic)
        drop_gap = max(DROPOUT_FLOOR_S, DROPOUT_PERIODS / nominal_hz)
        mon = TopicMonitor(key, event_log, drop_gap,
                           snapshotter=snapshotter if key.startswith("zed") else None)
        sub = rospy.Subscriber(topic, msg_type, mon.on_msg, queue_size=queue)
        monitors.append((key, mon, topic, drop_gap))
        stream_subs[key] = {"sub": sub, "topic": topic, "msg_type": msg_type,
                            "queue": queue, "monitor": mon}
    dead_windows = {key: 0 for key, _m, _t, _d in monitors}  # consecutive silent windows

    # Track the currently-executing skill (logged on change), so a stall can be
    # tied to whatever skill was running at the time.
    skill_tracker = SkillTracker(event_log)
    rospy.Subscriber(args.skill_topic, String, skill_tracker.on_msg, queue_size=10)

    # CSV header: three columns per stream, then system metrics.
    cols = ["wall_iso", "elapsed_s"]
    for key, _mon, _topic, _drop in monitors:
        cols += [f"{key}_hz", f"{key}_max_gap_s", f"{key}_max_stampgap_s"]
    cols += ["cpu_pct", "load1", "gpu_util_pct", "gpu_temp_c", "gpu_power_w"]
    samples.write(",".join(cols) + "\n")

    stop_evt = threading.Event()
    usb_thread = threading.Thread(
        target=follow_kernel_usb, args=(event_log, stop_evt, not args.no_dmesg), daemon=True)
    usb_thread.start()

    cpu = CpuSampler()
    thresholds = " ".join(f"{k}={t}@{d:.2f}s" for k, _m, t, d in monitors)
    event_log.write(f"INFO  run start; {thresholds} duration={args.duration}s "
                    f"(dropout=max({DROPOUT_FLOOR_S}s, {DROPOUT_PERIODS:.0f}/nominal_hz))")
    print(f"[sensor_diag] monitoring {len(monitors)} streams for {args.duration:.0f}s; "
          f"Ctrl+C to stop early.")

    t0 = time.time()
    last = t0
    usb_topo_thread = threading.Thread(
        target=watch_usb_topology, args=(usb_watcher, stop_evt, args.usb_poll), daemon=True)
    usb_topo_thread.start()
    if gpu_csv is not None:
        gpu_sampler = GpuProcSampler(gpu_csv, t0)
        threading.Thread(target=gpu_sampler.run,
                         args=(stop_evt, args.gpu_interval), daemon=True).start()
    try:
        while not rospy.is_shutdown():
            now = time.time()
            elapsed = now - t0
            if elapsed >= args.duration:
                break
            time.sleep(max(0.0, args.interval - (now - last)))
            now = time.time()
            dt = now - last
            last = now

            row = [datetime.now().isoformat(), f"{now - t0:.1f}"]
            hz_by_key = {}
            for _key, mon, _topic, _drop in monitors:
                hz, gap, sgap = mon.sample(dt)
                hz_by_key[_key] = hz
                row += [f"{hz:.1f}", f"{gap:.3f}", f"{sgap:.3f}"]
            resubscribe_stale_streams(hz_by_key, dead_windows, stream_subs, event_log)
            cpu_pct, load1 = cpu.sample()
            g_util, g_temp, g_power = sample_gpu()
            row += [f"{cpu_pct:.1f}", f"{load1:.2f}",
                    f"{g_util:.0f}", f"{g_temp:.0f}", f"{g_power:.1f}"]
            samples.write(",".join(row) + "\n")
    except KeyboardInterrupt:
        print("\n[sensor_diag] Ctrl+C -- stopping.")

    stop_evt.set()
    end = time.time()
    meta = {
        "start_iso": datetime.fromtimestamp(t0).isoformat(),
        "end_iso": datetime.fromtimestamp(end).isoformat(),
        "elapsed_s": round(end - t0, 1),
        "args": vars(args),
        "streams": {
            key: {
                "topic": topic,
                "dropout_gap_s": round(drop_gap, 3),
                "messages": mon.total,
                "dropouts": mon.dropouts,
                "worst_gap_s": round(mon.worst_gap, 3),
            }
            for key, mon, topic, drop_gap in monitors
        },
        "usb_event_lines": event_log.lines,
        "usb_topology_changes": usb_watcher.changes,
        "incident_snapshots": snapshotter.count,
        "gpu_proc_rows": (gpu_sampler.rows if gpu_sampler else 0),
        "gpu_proc_labels": (sorted(gpu_sampler.labels_seen) if gpu_sampler else []),
        "skill_changes": skill_tracker.changes,
        "last_skill": skill_tracker.current,
        "outputs": {
            "samples_csv": "samples.csv",
            "events_log": "events.log",
            "usb_topology": "usb_topology.txt",
            "gpu_procs_csv": ("gpu_procs.csv" if gpu_csv is not None else None),
        },
    }
    (outdir / "run_meta.json").write_text(json.dumps(meta, indent=2))
    event_log.write("INFO  run end; see run_meta.json")
    event_log.close()
    samples.close()
    if gpu_csv is not None:
        gpu_csv.close()

    print("\n[sensor_diag] ===== summary =====")
    print(f"  ran {meta['elapsed_s']:.0f}s -> {outdir}")
    for key, mon, _topic, drop_gap in monitors:
        print(f"  {key:8s}: {mon.total} msgs, {mon.dropouts} dropouts "
              f"(>{drop_gap:.2f}s), worst gap {mon.worst_gap:.3f}s")

    # Targeted hints from WHICH streams dropped together.
    dropped = {key for key, mon, _t, _d in monitors if mon.dropouts}
    if "zed_imu" in dropped:
        print("  hint: ZED IMU stalled -> ZED node/USB/camera drop; "
              "check events.log KERNEL usb lines.")
    if {"lidar_r", "lidar_l"} <= dropped:
        print("  hint: BOTH lidars dropped -> shared USB hub/power or CPU starvation "
              "(check if it lines up with the ZED/CPU spikes).")
    elif "lidar_r" in dropped or "lidar_l" in dropped:
        one = "lidar_r" if "lidar_r" in dropped else "lidar_l"
        print(f"  hint: only {one} dropped -> that unit's USB/serial/cable/motor; "
              "check events.log for its ttyUSB/cp210/ftdi lines.")
    if dropped == {"zed_imu", "lidar_r", "lidar_l"}:
        print("  hint: ALL sensors dropped together -> system-wide: USB bus, power, "
              "or CPU; correlate with cpu_pct/load1 in samples.csv.")
    if "realsense" in dropped and ({"lidar_r", "lidar_l"} & dropped):
        print("  hint: RealSense + lidar dropped together -> a RealSense USB fault hit "
              "the shared PCH controller (the known residual risk of this layout).")
    print(f"  usb topology changes: {usb_watcher.changes} "
          f"(see USB_TOPO lines in events.log + usb_topology.txt)")
    if snapshotter.count:
        print(f"  incident snapshots: {snapshotter.count} (see incidents/ -- ZED "
              f"sysfs power state + lsusb + kernel tail at each stall)")
    if gpu_sampler:
        print(f"  per-process GPU: gpu_procs.csv ({gpu_sampler.rows} rows; "
              f"labels seen: {sorted(gpu_sampler.labels_seen)})")
    print(f"  skill changes: {skill_tracker.changes} (last: {skill_tracker.current})")
    print("  read events.log for the per-stall timeline (incl. SKILL transitions).")


if __name__ == "__main__":
    main()
