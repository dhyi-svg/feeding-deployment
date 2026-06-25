#!/usr/bin/env python3
"""
compute_health_monitor.py  --  near-hang watchdog for the compute laptop.

This is NOT the ROS robot watchdog (that's safety/watchdog.py). This one watches
*system* resources and warns + (optionally) kills heavy processes when the
machine is about to freeze.

Why this machine hangs: it has 32 cores / 31 GB RAM but only 2 GB of swap. When
RAM fills up, Linux thrashes instead of cleanly OOM-killing, and the whole
desktop locks up for tens of seconds to minutes. The best *early* predictor of
this is the kernel's PSI (Pressure Stall Information): /proc/pressure/memory's
"some avg10" rises sharply a few seconds before a freeze. We watch that, plus
available RAM, swap fill, load, and CPU/GPU temperature.

Design goals:
  - Pure stdlib + a couple of shell-outs (nvidia-smi/notify-send). No ROS import,
    so it keeps running even when ROS is the thing hanging.
  - Cheap: reads /proc each tick; nvidia-smi only every few ticks.
  - Safe by default: warns first; only auto-kills after a CRITICAL state has
    persisted, and only processes on an explicit allow-to-kill list. The robot
    e-stop and controllers live on the NUC, so killing compute-side processes
    (run.py, shared_autonomy.launch, cartographer, ...) does not endanger anyone.

Usage:
  python compute_health_monitor.py                 # warn + auto-kill (default)
  python compute_health_monitor.py --no-kill       # warn only, never kill
  python compute_health_monitor.py --interval 2
  python compute_health_monitor.py --kill-extra 'molmo|some_proc'

Stop with Ctrl+C.
"""

import argparse
import glob
import logging
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from logging.handlers import RotatingFileHandler

# --------------------------------------------------------------------------- #
# Thresholds. Tuned for this laptop (31 GB RAM, 2 GB swap, RTX 4090 mobile).
# A metric is OK / WARN / CRIT. Overall severity = worst single metric.
# --------------------------------------------------------------------------- #
THRESHOLDS = {
    # Available RAM as % of total. Low = danger.
    "mem_avail_pct":  {"warn": 15.0, "crit": 7.0,  "low_is_bad": True},
    # Swap used as % of swap total. With only 2 GB swap, any real fill is bad.
    "swap_used_pct":  {"warn": 40.0, "crit": 75.0, "low_is_bad": False},
    # PSI memory "some" avg10: % of last 10s tasks stalled on memory. THE signal.
    "psi_mem_avg10":  {"warn": 8.0,  "crit": 25.0, "low_is_bad": False},
    # PSI cpu "some" avg10: CPU saturation. Rarely hangs but worth a warn.
    "psi_cpu_avg10":  {"warn": 50.0, "crit": 80.0, "low_is_bad": False},
    # 1-min load average divided by core count.
    "load_norm":      {"warn": 1.5,  "crit": 3.0,  "low_is_bad": False},
    # Hottest CPU package/core, deg C.
    "cpu_temp":       {"warn": 90.0, "crit": 96.0, "low_is_bad": False},
    # GPU temp, deg C.
    "gpu_temp":       {"warn": 83.0, "crit": 88.0, "low_is_bad": False},
    # GPU VRAM used %.
    "gpu_mem_pct":    {"warn": 88.0, "crit": 96.0, "low_is_bad": False},
}

# Metrics whose CRIT state can actually cause a hard freeze -> justify a kill.
HANG_METRICS = ("mem_avail_pct", "swap_used_pct", "psi_mem_avg10",
                "cpu_temp", "gpu_temp", "gpu_mem_pct")

# Processes we are ALLOWED to kill, in priority order (regex over full cmdline).
# First match with the highest RSS gets killed first. Order is a tie-breaker
# hint; actual victim is the biggest memory user among matches (for mem/swap
# pressure) so a leaking process is preferred.
DEFAULT_KILL_PATTERNS = [
    r"integration/run\.py|(^|/)run\.py",
    r"shared_autonomy\.launch|shared_autonomy_manager\.py|shared_autonomy_teleop\.py",
    r"cartographer_node|cartographer_occupancy|cartographer",
    r"roslaunch .*shared_autonomy",
]

# NEVER kill these, even if a pattern would match (safety net).
PROTECTED_PATTERNS = [
    r"compute_health_monitor\.py",          # us
    r"(^|/)sshd?($|\s|:)", r"systemd", r"dbus", r"gnome-shell",
    r"(^|/)Xorg", r"gdm", r"NetworkManager", r"pulseaudio|pipewire",
    r"anthropic.*claude|claude-code|\.vscode-server",  # this assistant / IDE
    r"roscore|rosmaster|rosout",            # killing roscore brings down everything; warn instead
]

KILL_HOLD_SECONDS = 8.0    # CRIT must persist this long before we kill
KILL_COOLDOWN_SECONDS = 20.0  # wait after a kill for memory to free before next
GPU_POLL_EVERY = 3         # poll nvidia-smi every N ticks (it costs ~50ms)

log = logging.getLogger("health")


# --------------------------------------------------------------------------- #
# Metric collection
# --------------------------------------------------------------------------- #
def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def get_mem():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        k, _, v = line.partition(":")
        info[k] = float(v.strip().split()[0]) if v.strip() else 0.0
    total = info.get("MemTotal", 1.0)
    avail = info.get("MemAvailable", 0.0)
    swt = info.get("SwapTotal", 0.0)
    swf = info.get("SwapFree", 0.0)
    mem_avail_pct = 100.0 * avail / total if total else 0.0
    swap_used_pct = 100.0 * (swt - swf) / swt if swt else 0.0
    return mem_avail_pct, swap_used_pct, avail / 1024.0, (swt - swf) / 1024.0


def get_psi(resource):
    """Return 'some' avg10 for /proc/pressure/<resource>, or 0 if unavailable."""
    txt = _read(f"/proc/pressure/{resource}")
    m = re.search(r"some .*?avg10=([\d.]+)", txt)
    return float(m.group(1)) if m else 0.0


def get_load_norm():
    try:
        load1 = float(_read("/proc/loadavg").split()[0])
    except (IndexError, ValueError):
        return 0.0
    return load1 / (os.cpu_count() or 1)


def get_cpu_temp():
    """Hottest CPU temperature in deg C from hwmon/thermal, or 0 if unknown."""
    best = 0.0
    # Prefer coretemp/k10temp hwmon (package + cores).
    for label_path in glob.glob("/sys/class/hwmon/hwmon*/name"):
        name = _read(label_path).strip()
        if name not in ("coretemp", "k10temp", "zenpower"):
            continue
        base = os.path.dirname(label_path)
        for tpath in glob.glob(os.path.join(base, "temp*_input")):
            try:
                best = max(best, int(_read(tpath)) / 1000.0)
            except ValueError:
                pass
    if best:
        return best
    # Fallback: thermal zones.
    for tpath in glob.glob("/sys/class/thermal/thermal_zone*/temp"):
        try:
            best = max(best, int(_read(tpath)) / 1000.0)
        except ValueError:
            pass
    return best


_gpu_cache = {"gpu_temp": 0.0, "gpu_mem_pct": 0.0, "gpu_util": 0.0}


def get_gpu():
    """Return (temp, mem_pct, util). Cached between polls by caller."""
    if not shutil.which("nvidia-smi"):
        return None
    try:
        out = subprocess.run(
            ["nvidia-smi",
             "--query-gpu=temperature.gpu,memory.used,memory.total,utilization.gpu",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=4,
        ).stdout.strip().splitlines()
    except (subprocess.SubprocessError, OSError):
        return None
    temp = mem_pct = util = 0.0
    for row in out:  # take the hottest / fullest GPU if multiple
        try:
            t, used, tot, u = [float(x) for x in row.split(",")]
        except ValueError:
            continue
        temp = max(temp, t)
        mem_pct = max(mem_pct, 100.0 * used / tot if tot else 0.0)
        util = max(util, u)
    return temp, mem_pct, util


def severity(metric, value):
    th = THRESHOLDS[metric]
    if th["low_is_bad"]:
        if value <= th["crit"]:
            return "CRIT"
        if value <= th["warn"]:
            return "WARN"
    else:
        if value >= th["crit"]:
            return "CRIT"
        if value >= th["warn"]:
            return "WARN"
    return "OK"


# --------------------------------------------------------------------------- #
# Process inspection / killing
# --------------------------------------------------------------------------- #
def iter_procs():
    """Yield (pid, cmdline_str, rss_kb, cpu_pct_unused) for all processes."""
    for pid_dir in glob.glob("/proc/[0-9]*"):
        pid = int(os.path.basename(pid_dir))
        cmd = _read(os.path.join(pid_dir, "cmdline")).replace("\x00", " ").strip()
        if not cmd:
            continue
        rss = 0
        for line in _read(os.path.join(pid_dir, "status")).splitlines():
            if line.startswith("VmRSS:"):
                try:
                    rss = int(line.split()[1])
                except (IndexError, ValueError):
                    rss = 0
                break
        yield pid, cmd, rss


def is_protected(cmd):
    return any(re.search(p, cmd) for p in PROTECTED_PATTERNS)


def pick_kill_victim(kill_patterns):
    """Biggest-RSS process matching a kill pattern and not protected, or None."""
    me = os.getpid()
    parent = os.getppid()
    best = None
    for pid, cmd, rss in iter_procs():
        if pid in (me, parent):
            continue
        if is_protected(cmd):
            continue
        if any(re.search(p, cmd) for p in kill_patterns):
            if best is None or rss > best[2]:
                best = (pid, cmd, rss)
    return best


def kill_pid(pid, cmd, notifier, rss_kb=0):
    short = cmd[:80]
    log.warning("AUTO-KILL: sending SIGTERM to pid %d (%.0f MB) -- %s",
                pid, rss_kb / 1024.0, short)
    notifier.alert("CRIT", f"Killing pid {pid} to prevent hang", short)
    try:
        os.kill(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    # Give it 4s to die cleanly, then SIGKILL.
    for _ in range(8):
        time.sleep(0.5)
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            log.info("pid %d exited after SIGTERM", pid)
            return
    try:
        os.kill(pid, signal.SIGKILL)
        log.warning("pid %d ignored SIGTERM; sent SIGKILL", pid)
    except ProcessLookupError:
        pass


# --------------------------------------------------------------------------- #
# Notifications
# --------------------------------------------------------------------------- #
class Notifier:
    """Desktop popup (notify-send) + sound (paplay) + log. Best-effort."""

    SOUNDS = {
        "WARN": "/usr/share/sounds/freedesktop/stereo/dialog-warning.oga",
        "CRIT": "/usr/share/sounds/freedesktop/stereo/dialog-error.oga",
    }

    def __init__(self, enable_popup=True, enable_sound=True):
        self.enable_popup = enable_popup and bool(shutil.which("notify-send"))
        self.enable_sound = enable_sound and bool(shutil.which("paplay"))
        self._env = self._desktop_env()
        self._last = {}  # severity -> last time we alerted (rate limit)

    @staticmethod
    def _desktop_env():
        """notify-send/paplay need DISPLAY + DBUS even when we run from SSH.
        Borrow them from the running gnome-shell session."""
        env = dict(os.environ)
        if env.get("DISPLAY") and env.get("DBUS_SESSION_BUS_ADDRESS"):
            return env
        try:
            pid = subprocess.run(["pgrep", "-u", str(os.getuid()), "-n",
                                  "gnome-shell"], capture_output=True,
                                 text=True).stdout.strip()
            if pid:
                raw = _read(f"/proc/{pid}/environ").split("\x00")
                for item in raw:
                    if item.startswith(("DISPLAY=", "DBUS_SESSION_BUS_ADDRESS=",
                                        "XDG_RUNTIME_DIR=", "XAUTHORITY=")):
                        k, _, v = item.partition("=")
                        env[k] = v
        except (OSError, subprocess.SubprocessError):
            pass
        env.setdefault("DISPLAY", ":0")
        return env

    def alert(self, sev, title, body, rate_limit=10.0):
        now = time.time()
        if now - self._last.get((sev, title), 0) < rate_limit:
            return
        self._last[(sev, title)] = now
        if self.enable_popup:
            urgency = "critical" if sev == "CRIT" else "normal"
            try:
                subprocess.Popen(
                    ["notify-send", "-u", urgency, "-a", "compute-watchdog",
                     f"[{sev}] {title}", body], env=self._env,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                pass
        if self.enable_sound and sev in self.SOUNDS:
            snd = self.SOUNDS[sev]
            if os.path.exists(snd):
                try:
                    subprocess.Popen(["paplay", snd], env=self._env,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                except OSError:
                    pass


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def setup_logging(logfile):
    log.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(message)s",
                            "%H:%M:%S")
    # Console handler only echoes events (WARN/CRIT); the per-tick live status
    # line is printed directly via print() so it shows every second without
    # also flooding the rotating log file.
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    ch.setLevel(logging.WARNING)
    log.addHandler(ch)
    if logfile:
        os.makedirs(os.path.dirname(logfile), exist_ok=True)
        fh = RotatingFileHandler(logfile, maxBytes=2_000_000, backupCount=3)
        fh.setFormatter(fmt)
        fh.setLevel(logging.INFO)
        log.addHandler(fh)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interval", type=float, default=3.0,
                    help="seconds between samples (default 3)")
    ap.add_argument("--no-kill", action="store_true",
                    help="never kill anything; warn only")
    ap.add_argument("--kill-extra", default="",
                    help="extra '|'-separated regexes of cmdlines OK to kill")
    ap.add_argument("--no-popup", action="store_true", help="disable notify-send")
    ap.add_argument("--no-sound", action="store_true", help="disable sound alerts")
    ap.add_argument("--logfile", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "log", "health_monitor.log"))
    args = ap.parse_args()

    setup_logging(args.logfile)
    notifier = Notifier(enable_popup=not args.no_popup,
                        enable_sound=not args.no_sound)

    kill_patterns = list(DEFAULT_KILL_PATTERNS)
    if args.kill_extra.strip():
        kill_patterns.append(args.kill_extra.strip())

    meminfo = _read("/proc/meminfo")
    ram_gb = float(re.search(r"MemTotal:\s+(\d+)", meminfo).group(1)) / 1e6
    swap_gb = float(re.search(r"SwapTotal:\s+(\d+)", meminfo).group(1)) / 1e6
    banner = ("compute health monitor started "
              f"(interval={args.interval:.1f}s, "
              f"auto-kill={'OFF' if args.no_kill else 'ON'})\n"
              f"cores={os.cpu_count() or 0}  ram={ram_gb:.1f}GB  swap={swap_gb:.1f}GB")
    print(banner, flush=True)   # console
    log.info(banner.replace("\n", " | "))   # file

    crit_since = None     # timestamp the current sustained CRIT (hang) began
    last_kill = 0.0
    tick = 0

    while True:
        tick += 1
        mem_avail_pct, swap_used_pct, avail_mb, swap_used_mb = get_mem()
        metrics = {
            "mem_avail_pct": mem_avail_pct,
            "swap_used_pct": swap_used_pct,
            "psi_mem_avg10": get_psi("memory"),
            "psi_cpu_avg10": get_psi("cpu"),
            "load_norm": get_load_norm(),
            "cpu_temp": get_cpu_temp(),
        }
        if tick % GPU_POLL_EVERY == 1:
            gpu = get_gpu()
            if gpu:
                _gpu_cache["gpu_temp"], _gpu_cache["gpu_mem_pct"], _gpu_cache["gpu_util"] = gpu
        if shutil.which("nvidia-smi"):
            metrics["gpu_temp"] = _gpu_cache["gpu_temp"]
            metrics["gpu_mem_pct"] = _gpu_cache["gpu_mem_pct"]

        sev_by_metric = {m: severity(m, v) for m, v in metrics.items()
                         if m in THRESHOLDS}
        worst = "OK"
        for s in sev_by_metric.values():
            if s == "CRIT":
                worst = "CRIT"
                break
            if s == "WARN":
                worst = "WARN"

        # One rich status line, printed live to the console every tick.
        status = (f"[{worst}] memAvail={mem_avail_pct:4.1f}%({avail_mb/1024:4.1f}G) "
                  f"swap={swap_used_pct:4.1f}%({swap_used_mb:4.0f}M) "
                  f"psiMem={metrics['psi_mem_avg10']:5.1f} "
                  f"psiCpu={metrics['psi_cpu_avg10']:5.1f} "
                  f"load={metrics['load_norm']:4.2f} cpuT={metrics['cpu_temp']:4.1f}C")
        if "gpu_temp" in metrics:
            status += (f" gpuT={metrics['gpu_temp']:4.1f}C "
                       f"gpuMem={metrics['gpu_mem_pct']:4.1f}% "
                       f"gpuUtil={_gpu_cache['gpu_util']:3.0f}%")

        # Which metrics are bad right now, and are any of them hang-causing?
        bad = [f"{m}={metrics[m]:.1f}({s})"
               for m, s in sev_by_metric.items() if s != "OK"]
        hang_crit = [m for m in HANG_METRICS
                     if sev_by_metric.get(m) == "CRIT"]

        # Live feed: one rich line every tick. Goes to the console only; the
        # rotating file log stays compact (periodic heartbeat + events below).
        print(f"{time.strftime('%H:%M:%S')} {status}", flush=True)

        if worst == "OK":
            crit_since = None
            if tick % 10 == 0:
                log.info(status)              # heartbeat at idle
        elif worst == "WARN":
            crit_since = None
            log.warning("%s  ->  %s", status, ", ".join(bad))
            notifier.alert("WARN", "Compute load rising", ", ".join(bad),
                           rate_limit=30.0)
        else:  # CRIT
            log.error("%s  ->  %s", status, ", ".join(bad))
            notifier.alert("CRIT", "NEAR-HANG on compute laptop",
                           ", ".join(bad), rate_limit=15.0)
            if hang_crit:
                if crit_since is None:
                    crit_since = time.time()
                held = time.time() - crit_since
                cooled = time.time() - last_kill > KILL_COOLDOWN_SECONDS
                if (not args.no_kill and held >= KILL_HOLD_SECONDS and cooled):
                    victim = pick_kill_victim(kill_patterns)
                    if victim:
                        pid, cmd, rss = victim
                        log.error("Sustained near-hang %.0fs on %s -- killing "
                                  "pid %d (%.0f MB): %s", held,
                                  ",".join(hang_crit), pid, rss / 1024.0, cmd[:90])
                        kill_pid(pid, cmd, notifier, rss_kb=rss)
                        last_kill = time.time()
                        crit_since = None
                    else:
                        log.error("Near-hang but NO killable process matches the "
                                  "allow-list. Free memory manually!")
                        notifier.alert("CRIT", "Near-hang, nothing safe to kill",
                                       "Close apps manually", rate_limit=20.0)
            else:
                crit_since = None  # CRIT was non-hang (e.g. cpu load only)

        time.sleep(args.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nhealth monitor stopped.")
