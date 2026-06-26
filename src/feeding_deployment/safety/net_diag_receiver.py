#!/usr/bin/env python3
"""net_diag_receiver.py -- NUC side of the Mac->NUC WiFi/UDP link diagnostic.

This is a measurement tool, NOT part of the safety path. It is the diagnostic
twin of `estop_udp_bridge.py`, but:
  * It binds a SEPARATE UDP port (default 5006, NOT the live e-stop 5005).
  * It imports NO ROS and registers nothing with the ROS master, so it cannot
    interfere with the live robot or the physical-e-stop path running alongside.
  * Instead of republishing, it timestamps every received packet to a CSV and
    detects loss/gaps live.

The NUC is wired to the router, so any gap seen here is over-the-air loss on the
Mac->router hop -- the clean measurement point. Run this on the NUC while the Mac
runs `net_diag_sender.py`; collect the CSVs afterward and feed them, with the
Mac's CSVs, to `scripts/analyze_net_diag.py`.

Packet (Mac -> NUC), must match net_diag_sender.py:
    !Qd  = seq (uint64), t_send_monotonic (double, Mac clock)
We echo the exact bytes straight back as the ack so the Mac can measure RTT on
its own single clock.

Usage (on the NUC, robot stack already running):
    python net_diag_receiver.py
    python net_diag_receiver.py --port 5006 --gap-ms 50

Stop with Ctrl+C (prints a final summary).
"""

import argparse
import os
import re
import socket
import struct
import time

# Mac -> NUC packet; we echo the same bytes back as the ack.
PACKET_FORMAT = "!Qd"
PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

DEFAULT_PORT = 5006
RECV_TIMEOUT_S = 0.5  # wake up periodically even with no traffic (sys log, summary)


# --------------------------------------------------------------------------- #
# Lightweight /proc readers (same idea as compute_health_monitor.py, inlined to
# keep this script standalone). Used to rule out a NUC-side stall as the cause
# of a gap, since the NUC is under full robot load during the run.
# --------------------------------------------------------------------------- #
def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def get_mem_avail_pct():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        k, _, v = line.partition(":")
        if v.strip():
            info[k] = float(v.strip().split()[0])
    total = info.get("MemTotal", 1.0)
    return 100.0 * info.get("MemAvailable", 0.0) / total if total else 0.0


def get_psi(resource):
    m = re.search(r"some .*?avg10=([\d.]+)", _read(f"/proc/pressure/{resource}"))
    return float(m.group(1)) if m else 0.0


def get_load_norm():
    try:
        return float(_read("/proc/loadavg").split()[0]) / (os.cpu_count() or 1)
    except (IndexError, ValueError):
        return 0.0


def default_out_dir():
    run_id = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "..", "integration", "log", "net_diag", run_id)
    return os.path.normpath(base)


class DiagReceiver:
    def __init__(self, port, gap_ms, out_dir, log_sys, sys_interval):
        self.gap_s = gap_ms / 1000.0
        self.log_sys = log_sys
        self.sys_interval = sys_interval
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)

        self.recv_f = open(os.path.join(out_dir, "recv.csv"), "w", buffering=1)
        self.recv_f.write("seq,t_recv_wall,t_recv_mono\n")

        self.gaps_f = open(os.path.join(out_dir, "gaps.csv"), "w", buffering=1)
        self.gaps_f.write("t_wall,kind,gap_ms,lost_pkts,prev_seq,seq\n")

        self.sys_f = None
        if log_sys:
            self.sys_f = open(os.path.join(out_dir, "sys.csv"), "w", buffering=1)
            self.sys_f.write("t_wall,mem_avail_pct,psi_mem_avg10,psi_cpu_avg10,load_norm\n")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", port))
        self.sock.settimeout(RECV_TIMEOUT_S)

        self.sender_addr = None
        self.received = 0
        self.lost = 0
        self.max_gap_ms = 0.0
        self.last_seq = None
        self.last_recv_mono = None

        print(f"net-diag receiver listening on udp/{port} -> {out_dir}", flush=True)
        print("Waiting for packets from the Mac sender (net_diag_sender.py)...", flush=True)

    def _log_gap(self, t_wall, kind, gap_ms, lost, prev_seq, seq):
        self.gaps_f.write(f"{t_wall:.6f},{kind},{gap_ms:.1f},{lost},{prev_seq},{seq}\n")
        print(f"[gap] {time.strftime('%H:%M:%S')} {kind} gap={gap_ms:.0f}ms "
              f"lost={lost} (seq {prev_seq}->{seq})", flush=True)

    def _sample_sys(self):
        if not self.sys_f:
            return
        self.sys_f.write(f"{time.time():.3f},{get_mem_avail_pct():.1f},"
                         f"{get_psi('memory'):.1f},{get_psi('cpu'):.1f},"
                         f"{get_load_norm():.2f}\n")

    def run(self):
        start = time.time()
        report_at = start + 5.0
        sys_at = start
        try:
            while True:
                now = time.time()
                if now >= sys_at:
                    self._sample_sys()
                    sys_at = now + self.sys_interval
                if now >= report_at:
                    loss_pct = 100.0 * self.lost / max(1, self.lost + self.received)
                    print(f"alive: recv={self.received} lost={self.lost} "
                          f"({loss_pct:.2f}%) max_gap={self.max_gap_ms:.0f}ms", flush=True)
                    report_at = now + 5.0

                try:
                    data, addr = self.sock.recvfrom(64)
                except socket.timeout:
                    continue

                t_recv_wall = time.time()
                t_recv_mono = time.monotonic()

                if len(data) != PACKET_SIZE:
                    continue

                # Pin to the first sender, ignore others (mirrors the bridge).
                if self.sender_addr is None:
                    self.sender_addr = addr
                    print(f"locked onto sender at {addr[0]}:{addr[1]}", flush=True)
                elif addr != self.sender_addr:
                    continue

                # Echo straight back as the ack (Mac measures RTT on its clock).
                self.sock.sendto(data, self.sender_addr)

                seq, _t_send_mono = struct.unpack(PACKET_FORMAT, data)
                self.recv_f.write(f"{seq},{t_recv_wall:.6f},{t_recv_mono:.6f}\n")
                self.received += 1

                # Loss: gap in the sequence numbers.
                if self.last_seq is not None and seq > self.last_seq + 1:
                    lost = seq - self.last_seq - 1
                    self.lost += lost
                    self._log_gap(t_recv_wall, "seq_loss", 0.0, lost, self.last_seq, seq)
                # Latency gap: long silence between consecutive arrivals.
                if self.last_recv_mono is not None:
                    gap_ms = (t_recv_mono - self.last_recv_mono) * 1000.0
                    self.max_gap_ms = max(self.max_gap_ms, gap_ms)
                    if gap_ms > self.gap_s * 1000.0:
                        self._log_gap(t_recv_wall, "arrival_gap", gap_ms, 0,
                                      self.last_seq, seq)
                self.last_seq = seq
                self.last_recv_mono = t_recv_mono
        except KeyboardInterrupt:
            pass
        finally:
            self._summary(start)
            for f in (self.recv_f, self.gaps_f, self.sys_f):
                if f:
                    f.close()
            self.sock.close()

    def _summary(self, start):
        dur = time.time() - start
        total = self.received + self.lost
        loss_pct = 100.0 * self.lost / max(1, total)
        print("\n--- net-diag receiver summary ---")
        print(f"duration:   {dur:.0f}s")
        print(f"received:   {self.received}")
        print(f"lost (seq): {self.lost} ({loss_pct:.2f}%)")
        print(f"max gap:    {self.max_gap_ms:.0f} ms")
        print(f"CSVs in:    {self.out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT,
                    help=f"UDP port to bind (default {DEFAULT_PORT}; NOT the live 5005)")
    ap.add_argument("--gap-ms", type=float, default=50.0,
                    help="log an arrival gap when inter-packet spacing exceeds this (ms)")
    ap.add_argument("--out-dir", default=None,
                    help="output dir (default integration/log/net_diag/<timestamp>/)")
    ap.add_argument("--no-log-sys", action="store_true",
                    help="disable periodic NUC load/PSI sampling")
    ap.add_argument("--sys-interval", type=float, default=2.0,
                    help="seconds between NUC sys-load samples (default 2)")
    args = ap.parse_args()

    out_dir = args.out_dir or default_out_dir()
    DiagReceiver(args.port, args.gap_ms, out_dir,
                 log_sys=not args.no_log_sys,
                 sys_interval=args.sys_interval).run()


if __name__ == "__main__":
    main()
