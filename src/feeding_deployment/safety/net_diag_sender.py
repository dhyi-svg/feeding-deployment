#!/usr/bin/env python3
"""net_diag_sender.py -- Mac side of the Mac->NUC WiFi/UDP link diagnostic.

Measurement tool, NOT part of the safety path. Diagnostic twin of
`estop_sender.py`: it streams the SAME kind of UDP traffic the real e-stop sender
would (so it stresses the 5 GHz link the same way), but on a SEPARATE port
(default 5006, not the live 5005), with no audio/button and no ROS. It records
everything needed to find out WHY the link drops:

  * sent.csv      -- every packet: seq, wall time, monotonic time
  * acks.csv      -- every echo-ack: seq, wall time, RTT (round trip, one clock)
  * wifi_stats.csv-- ~1 Hz RSSI / noise / SNR / tx-rate / channel (via wdutil)
  * wifi_events.log -- macOS WiFi subsystem log for the run window (roam/scan/...)
  * run_meta.json -- start/end wall time, rate, host (so the analyzer can align)

Loss is detected receiver-side from sequence gaps (clock-free); RTT is measured
here on the Mac's single monotonic clock (no cross-machine sync needed).

Run it WHILE the robot operates normally (the physical NUC e-stop is the real
safety device during the test). This tool touches nothing the robot uses.

Run as root so `wdutil info` returns full WiFi stats without a mid-run password
prompt:
    caffeinate -dimsu sudo python net_diag_sender.py --host 192.168.1.3 --duration 3600

Stop early with Ctrl+C (still writes run_meta.json + captures wifi_events.log).
"""

import argparse
import json
import os
import re
import socket
import struct
import subprocess
import threading
import time

# Must match net_diag_receiver.py. The receiver echoes these exact bytes back.
PACKET_FORMAT = "!Qd"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

DEFAULT_PORT = 5006


def default_out_dir():
    run_id = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "integration", "log", "net_diag", run_id)
    return os.path.normpath(base)


# --------------------------------------------------------------------------- #
# WiFi stats sampling (macOS). Try wdutil (best, needs root), then
# system_profiler, then legacy airport. Returns dict with any of:
# rssi, noise, tx_rate, channel (floats/strings; missing -> None).
# --------------------------------------------------------------------------- #
def _run(cmd, timeout=4):
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (subprocess.SubprocessError, OSError):
        return ""


def get_ip(dev):
    """Current IPv4 on the WiFi interface, or None if it has none (= dropped)."""
    return _run(["ipconfig", "getifaddr", dev], timeout=2).strip() or None


def get_inuse_mac(dev):
    """The MAC currently in use on dev. If this ever != the hardware MAC, macOS
    has re-engaged a private (randomized) address."""
    m = re.search(r"\bether\s+([0-9a-f:]{17})", _run(["ifconfig", dev], timeout=2))
    return m.group(1) if m else None


def _first_float(pattern, text):
    m = re.search(pattern, text, re.IGNORECASE)
    return float(m.group(1)) if m else None


def _first_str(pattern, text):
    m = re.search(pattern, text, re.IGNORECASE)
    return m.group(1).strip() if m else None


def sample_wifi_wdutil():
    out = _run(["wdutil", "info"])
    if not out:
        return {}
    return {
        "rssi": _first_float(r"RSSI\s*:\s*(-?\d+)", out),
        "noise": _first_float(r"Noise\s*:\s*(-?\d+)", out),
        "tx_rate": _first_float(r"Tx Rate\s*:\s*([\d.]+)", out),
        "channel": _first_str(r"Channel\s*:\s*(\S+)", out),
        "bssid": _first_str(r"BSSID\s*:\s*([0-9a-fA-F:]{17})", out),
    }


def sample_wifi_system_profiler():
    out = _run(["system_profiler", "SPAirPortDataType"], timeout=8)
    if not out:
        return {}
    sn = re.search(r"Signal\s*/\s*Noise:\s*(-?\d+)\s*dBm\s*/\s*(-?\d+)\s*dBm", out)
    return {
        "rssi": float(sn.group(1)) if sn else None,
        "noise": float(sn.group(2)) if sn else None,
        "tx_rate": _first_float(r"Transmit Rate:\s*([\d.]+)", out),
        "channel": _first_str(r"Channel:\s*(.+)", out),
    }


AIRPORT = ("/System/Library/PrivateFrameworks/Apple80211.framework/Versions/"
           "Current/Resources/airport")


def sample_wifi_airport():
    out = _run([AIRPORT, "-I"])
    if not out:
        return {}
    return {
        "rssi": _first_float(r"agrCtlRSSI:\s*(-?\d+)", out),
        "noise": _first_float(r"agrCtlNoise:\s*(-?\d+)", out),
        "tx_rate": _first_float(r"lastTxRate:\s*([\d.]+)", out),
        "channel": _first_str(r"channel:\s*(\S+)", out),
    }


def sample_wifi():
    """Best-available WiFi stats. Prefer wdutil; fall back if RSSI missing."""
    for fn in (sample_wifi_wdutil, sample_wifi_system_profiler, sample_wifi_airport):
        d = fn()
        if d.get("rssi") is not None:
            return d
    return {"rssi": None, "noise": None, "tx_rate": None, "channel": None}


class WifiSampler(threading.Thread):
    def __init__(self, path, interval, dev="en0"):
        super().__init__(daemon=True)
        self.path = path
        self.interval = interval
        self.dev = dev
        self._stop = threading.Event()
        self.latest_rssi = None

    def run(self):
        with open(self.path, "w", buffering=1) as f:
            f.write("t_wall,rssi,noise,snr,tx_rate,channel,ip,mac,bssid\n")
            while not self._stop.is_set():
                d = sample_wifi()
                rssi, noise = d.get("rssi"), d.get("noise")
                snr = (rssi - noise) if (rssi is not None and noise is not None) else None
                self.latest_rssi = rssi
                # IP / in-use MAC catch the two failure modes directly: IP goes
                # blank on an interface drop; MAC changing = private addr re-engaged.
                ip = get_ip(self.dev)
                mac = get_inuse_mac(self.dev)
                f.write(f"{time.time():.3f},{_fmt(rssi)},{_fmt(noise)},{_fmt(snr)},"
                        f"{_fmt(d.get('tx_rate'))},{_fmt(d.get('channel'))},"
                        f"{_fmt(ip)},{_fmt(mac)},{_fmt(d.get('bssid'))}\n")
                self._stop.wait(self.interval)

    def stop(self):
        self._stop.set()


def _fmt(v):
    return "" if v is None else (v if isinstance(v, str) else f"{v:g}")


def capture_wifi_events(t0_wall, path):
    """Dump the macOS WiFi subsystem log for the run window to `path`."""
    start_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0_wall))
    predicate = ('subsystem == "com.apple.wifi" OR process == "airportd" '
                 'OR process == "wifid"')
    try:
        out = subprocess.run(
            ["log", "show", "--start", start_str, "--predicate", predicate,
             "--info", "--style", "syslog"],
            capture_output=True, text=True, timeout=600).stdout
        with open(path, "w") as f:
            f.write(out)
        print(f"captured WiFi event log -> {path} ({out.count(chr(10))} lines)")
    except (subprocess.SubprocessError, OSError) as exc:
        print(f"could not capture WiFi event log ({exc}); manual fallback:\n"
              f"  log show --start '{start_str}' --predicate '{predicate}' "
              f"--info --style syslog > {path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="192.168.1.3", help="NUC IP running net_diag_receiver.py")
    p.add_argument("--port", type=int, default=DEFAULT_PORT,
                   help=f"UDP port (default {DEFAULT_PORT}; NOT the live 5005)")
    p.add_argument("--rate", type=float, default=100.0, help="send rate in Hz (default 100)")
    p.add_argument("--duration", type=float, default=3600.0,
                   help="run length in seconds (default 3600 = 1 hour)")
    p.add_argument("--wifi-interval", type=float, default=1.0,
                   help="seconds between WiFi stat samples (default 1)")
    p.add_argument("--wifi-dev", default="en0",
                   help="macOS Wi-Fi interface for IP/MAC sampling (default en0)")
    p.add_argument("--out-dir", default=None,
                   help="output dir (default integration/log/net_diag/<timestamp>/)")
    args = p.parse_args()

    out_dir = args.out_dir or default_out_dir()
    os.makedirs(out_dir, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    dest = (args.host, args.port)
    period = 1.0 / args.rate

    sent_f = open(os.path.join(out_dir, "sent.csv"), "w", buffering=1)
    sent_f.write("seq,t_send_wall,t_send_mono\n")
    acks_f = open(os.path.join(out_dir, "acks.csv"), "w", buffering=1)
    acks_f.write("seq,t_ack_wall,rtt_ms\n")
    # Send failures (e.g. EADDRNOTAVAIL when WiFi briefly drops the Mac's IP). We
    # record these and keep going instead of crashing -- a failed send shows up as
    # a missing seq (loss) at the receiver, and this file says why.
    err_f = open(os.path.join(out_dir, "send_errors.csv"), "w", buffering=1)
    err_f.write("t_wall,seq,errno,strerror\n")

    sampler = WifiSampler(os.path.join(out_dir, "wifi_stats.csv"),
                          args.wifi_interval, dev=args.wifi_dev)
    sampler.start()

    t0_wall = time.time()
    print(f"net-diag sender -> {args.host}:{args.port} at {args.rate:.0f} Hz "
          f"for {args.duration:.0f}s")
    print(f"output dir: {out_dir}")
    print("Streaming... (Ctrl+C to stop early). This does NOT affect the robot.")

    seq = 0
    acked = 0
    send_errs = 0
    last_report = time.monotonic()
    last_report_seq = 0
    last_err_print = 0.0
    start_mono = time.monotonic()
    try:
        while time.monotonic() - start_mono < args.duration:
            start = time.monotonic()
            t_send_wall = time.time()
            try:
                sock.sendto(struct.pack(PACKET_FORMAT, seq, start), dest)
                sent_f.write(f"{seq},{t_send_wall:.6f},{start:.6f}\n")
            except OSError as exc:
                # Interface likely lost its IP (WiFi disconnect/reassoc). Record,
                # warn (throttled), and continue -- the gap is captured as loss.
                send_errs += 1
                err_f.write(f"{t_send_wall:.6f},{seq},{exc.errno},{exc.strerror}\n")
                if start - last_err_print >= 1.0:
                    print(f"!! send error (errno {exc.errno}: {exc.strerror}) -- "
                          f"WiFi interface dropped its IP; continuing", flush=True)
                    last_err_print = start
            seq += 1

            # Drain echo-acks (non-blocking) and record RTT.
            while True:
                try:
                    data, _addr = sock.recvfrom(64)
                except (BlockingIOError, socket.error):
                    break
                if len(data) != PACKET_SIZE:
                    continue
                ack_seq, t_send_mono = struct.unpack(PACKET_FORMAT, data)
                rtt_ms = (time.monotonic() - t_send_mono) * 1000.0
                acks_f.write(f"{ack_seq},{time.time():.6f},{rtt_ms:.3f}\n")
                acked += 1

            if start - last_report >= 1.0:
                elapsed = start - last_report
                freq = (seq - last_report_seq) / elapsed
                rssi = sampler.latest_rssi
                print(f"... {freq:.0f} Hz sent, {acked} acked total, "
                      f"RSSI={rssi if rssi is not None else '?'} dBm", flush=True)
                last_report = start
                last_report_seq = seq

            time.sleep(max(0.0, period - (time.monotonic() - start)))
    except KeyboardInterrupt:
        print("\nstopping early (Ctrl+C)")
    finally:
        t1_wall = time.time()
        sampler.stop()
        sent_f.close()
        acks_f.close()
        err_f.close()
        with open(os.path.join(out_dir, "run_meta.json"), "w") as f:
            json.dump({"t0_wall": t0_wall, "t1_wall": t1_wall, "rate": args.rate,
                       "host": args.host, "port": args.port, "sent": seq,
                       "acked": acked, "send_errors": send_errs}, f, indent=2)
        capture_wifi_events(t0_wall, os.path.join(out_dir, "wifi_events.log"))
        sock.close()
        print(f"\ndone. sent={seq} acked={acked} send_errors={send_errs}. "
              f"CSVs in {out_dir}")
        print("Next: scp this dir's files next to the NUC recv.csv, then run "
              "scripts/analyze_net_diag.py <run-dir>")


if __name__ == "__main__":
    main()
