#!/usr/bin/env python3
"""sensor_diag_logger.py -- find out WHY a perception stream stalls/drops.

Measurement tool, NOT part of the safety path. Run it for a few hours WHILE the
robot operates normally; it touches nothing the robot uses (read-only subscribers
+ read-only system polling). The watchdog has tripped on both the ZED odom
frequency and the lidars; this logger records everything needed to pin the cause.

It monitors FOUR streams at once so a stall is self-diagnosing:

  * /zed_mini/zed_node/imu/data  (zed_imu)  -- raw off the sensor, ~200-400 Hz
  * /zed_mini/zed_node/odom      (zed_odom) -- VIO-FUSED output, ~30-60 Hz
  * /lidar_r/scan                (lidar_r)  -- right RPLIDAR A1, ~10 Hz
  * /lidar_l/scan                (lidar_l)  -- left  RPLIDAR A1, ~10 Hz

How to read a stall from WHICH streams stop together:
  * ZED odom + IMU stop together   -> ZED node/USB/camera dropped (hardware/bw).
  * ZED odom only (IMU fine)       -> VIO/grab loop stalled (CPU/GPU/tracking).
  * One lidar only                 -> that lidar's USB/serial/cable/motor.
  * Both lidars together           -> shared USB hub/power, or CPU starvation.
  * Lidars + ZED all together      -> system-wide: USB bus, power, or CPU.

For every message it separates ARRIVAL gaps (wall clock, when WE got it) from
HEADER-STAMP gaps (when the driver said it was produced): if header stamps stay
continuous but arrivals gap, the transport/subscriber stalled, not the sensor.

Stalls are correlated against the usual suspects, sampled the whole run:
  * USB kernel events (dmesg/journalctl: disconnect/reset/xhci) -- best effort
  * CPU% + load average, and GPU util/temp/power (nvidia-smi) -- best effort

Outputs (one timestamped dir per run, default under integration/log/):
  * samples.csv  -- ~1 Hz: per-topic Hz, worst arrival gap, CPU, load, GPU
  * events.log   -- every detected dropout (which topic, gap size) + USB lines,
                    interleaved in wall-clock order so you can read the story
  * run_meta.json-- start/end, args, per-topic totals (dropouts, worst gap)

Usage (let it run, Ctrl+C stops early and still writes the summary):
    python sensor_diag_logger.py --duration 14400      # 4 hours

Reading dmesg may need privileges (kernel.dmesg_restrict); if so, run with sudo
or pass --no-dmesg. Everything else works without root.
"""

import argparse
import json
import os
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rospy
from sensor_msgs.msg import Imu, LaserScan
from nav_msgs.msg import Odometry


# A single inter-message gap larger than this (seconds) is logged as a dropout
# event. Normal spacing: zed_odom ~0.02-0.03 s, zed_imu ~0.003-0.005 s, lidar
# ~0.1 s. 0.25 s means a real stall started (several to dozens of msgs missed)
# while still well under the watchdog's full 1 s trip, so we catch precursors.
DROPOUT_GAP_S = 0.25


# Streams to monitor: key, default topic, ROS msg type, subscriber queue size,
# human description. Add a row here to watch another topic -- everything else
# (CSV columns, events, summary) adapts automatically.
STREAMS = [
    ("zed_odom", "/zed_mini/zed_node/odom",     Odometry,  2000, "ZED VIO odom ~30-60 Hz"),
    ("zed_imu",  "/zed_mini/zed_node/imu/data", Imu,       4000, "ZED raw IMU ~200-400 Hz"),
    ("lidar_r",  "/lidar_r/scan",               LaserScan,  200, "Right RPLIDAR ~10 Hz"),
    ("lidar_l",  "/lidar_l/scan",               LaserScan,  200, "Left RPLIDAR ~10 Hz"),
]


class TopicMonitor:
    """Tracks rate and gaps for one topic. Callback-thread safe."""

    def __init__(self, name, event_log):
        self.name = name
        self._event_log = event_log
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
                if gap >= DROPOUT_GAP_S:
                    self.dropouts += 1
                    stamp_gap = (stamp - self._last_stamp) if (stamp and self._last_stamp) else float("nan")
                    self._event_log.write(
                        f"DROPOUT {self.name:8s} arrival_gap={gap:6.3f}s "
                        f"stamp_gap={stamp_gap:6.3f}s "
                        f"(header {'continuous->transport stall' if stamp_gap < gap / 2 else 'also gapped->source stall'})"
                    )
            if stamp is not None:
                if self._last_stamp is not None:
                    sg = stamp - self._last_stamp
                    if sg > self._max_stamp_gap:
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


def follow_kernel_usb(event_log, stop_evt, use_dmesg):
    """Stream USB/xhci kernel lines into the event log. Best effort."""
    if not use_dmesg:
        return
    # Prefer `dmesg --follow` (has the message text); fall back to journalctl -kf.
    for cmd in (["dmesg", "--follow", "--ctime"], ["journalctl", "-kf", "-o", "short"]):
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
                low = line.lower()
                if any(k in low for k in ("usb", "xhci", "zed", "uvc", "cp210", "ftdi", "ttyusb")):
                    event_log.write("KERNEL " + line.rstrip())
        finally:
            proc.terminate()
        return
    event_log.write("WARN  no kernel log source available (dmesg/journalctl); "
                    "USB events will NOT be captured (try sudo, or --no-dmesg)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=14400.0,
                    help="seconds to run (default 14400 = 4 h)")
    ap.add_argument("--zed-odom-topic", default="/zed_mini/zed_node/odom")
    ap.add_argument("--zed-imu-topic", default="/zed_mini/zed_node/imu/data")
    ap.add_argument("--lidar-r-topic", default="/lidar_r/scan")
    ap.add_argument("--lidar-l-topic", default="/lidar_l/scan")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="sample/CSV row period in seconds (default 1.0)")
    ap.add_argument("--outdir", default=None,
                    help="output dir (default: integration/log/sensor_diag_<timestamp>/)")
    ap.add_argument("--no-dmesg", action="store_true",
                    help="don't try to read the kernel log (skip USB capture)")
    args = ap.parse_args()

    # Resolve per-stream topics from CLI overrides (key -> arg attribute).
    topic_overrides = {
        "zed_odom": args.zed_odom_topic,
        "zed_imu": args.zed_imu_topic,
        "lidar_r": args.lidar_r_topic,
        "lidar_l": args.lidar_l_topic,
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

    rospy.init_node("sensor_diag_logger", anonymous=True, disable_signals=True)

    # Build a monitor per stream and subscribe.
    monitors = []          # list of (key, TopicMonitor, topic, description)
    for key, default_topic, msg_type, queue, desc in STREAMS:
        topic = topic_overrides.get(key, default_topic)
        mon = TopicMonitor(key, event_log)
        rospy.Subscriber(topic, msg_type, mon.on_msg, queue_size=queue)
        monitors.append((key, mon, topic, desc))

    # CSV header: three columns per stream, then system metrics.
    cols = ["wall_iso", "elapsed_s"]
    for key, _mon, _topic, _desc in monitors:
        cols += [f"{key}_hz", f"{key}_max_gap_s", f"{key}_max_stampgap_s"]
    cols += ["cpu_pct", "load1", "gpu_util_pct", "gpu_temp_c", "gpu_power_w"]
    samples.write(",".join(cols) + "\n")

    stop_evt = threading.Event()
    usb_thread = threading.Thread(
        target=follow_kernel_usb, args=(event_log, stop_evt, not args.no_dmesg), daemon=True)
    usb_thread.start()

    cpu = CpuSampler()
    topic_summary = " ".join(f"{k}={t}" for k, _m, t, _d in monitors)
    event_log.write(f"INFO  run start; {topic_summary} "
                    f"duration={args.duration}s dropout_gap={DROPOUT_GAP_S}s")
    print(f"[sensor_diag] monitoring {len(monitors)} streams for {args.duration:.0f}s; "
          f"Ctrl+C to stop early.")

    t0 = time.time()
    last = t0
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
            for _key, mon, _topic, _desc in monitors:
                hz, gap, sgap = mon.sample(dt)
                row += [f"{hz:.1f}", f"{gap:.3f}", f"{sgap:.3f}"]
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
        "dropout_gap_s": DROPOUT_GAP_S,
        "streams": {
            key: {
                "topic": topic,
                "messages": mon.total,
                "dropouts": mon.dropouts,
                "worst_gap_s": round(mon.worst_gap, 3),
            }
            for key, mon, topic, _desc in monitors
        },
        "usb_event_lines": event_log.lines,
    }
    (outdir / "run_meta.json").write_text(json.dumps(meta, indent=2))
    event_log.write("INFO  run end; see run_meta.json")
    event_log.close()
    samples.close()

    print("\n[sensor_diag] ===== summary =====")
    print(f"  ran {meta['elapsed_s']:.0f}s -> {outdir}")
    for key, mon, _topic, _desc in monitors:
        print(f"  {key:8s}: {mon.total} msgs, {mon.dropouts} dropouts, "
              f"worst gap {mon.worst_gap:.3f}s")

    # Targeted hints from WHICH streams dropped together.
    dropped = {key for key, mon, _t, _d in monitors if mon.dropouts}
    if {"zed_odom"} <= dropped and "zed_imu" not in dropped:
        print("  hint: ZED odom stalled but IMU did NOT -> VIO/CPU/GPU, not USB.")
    elif {"zed_odom", "zed_imu"} <= dropped:
        print("  hint: ZED odom+IMU stalled together -> ZED node/USB/camera drop; "
              "check events.log KERNEL usb lines.")
    if {"lidar_r", "lidar_l"} <= dropped:
        print("  hint: BOTH lidars dropped -> shared USB hub/power or CPU starvation "
              "(check if it lines up with the ZED/CPU spikes).")
    elif "lidar_r" in dropped or "lidar_l" in dropped:
        one = "lidar_r" if "lidar_r" in dropped else "lidar_l"
        print(f"  hint: only {one} dropped -> that unit's USB/serial/cable/motor; "
              "check events.log for its ttyUSB/cp210/ftdi lines.")
    if dropped == {"zed_odom", "zed_imu", "lidar_r", "lidar_l"}:
        print("  hint: ALL sensors dropped together -> system-wide: USB bus, power, "
              "or CPU; correlate with cpu_pct/load1 in samples.csv.")
    print("  read events.log for the per-stall timeline.")


if __name__ == "__main__":
    main()
