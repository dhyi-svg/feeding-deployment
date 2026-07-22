#!/usr/bin/env python3
"""
bench_test_encoders.py -- firmware v7 encoder bring-up checks (NO ROS).

Run this ON THE NUC with base_server.py STOPPED (the port is exclusive; if
this script can't open it, something else holds it -- check the tmux 'robot'
session). Opening the port DTR-resets the Arduino: you should see the
"Ready v7 enc" banner ~2 s in.

Modes (combine at will):

  watch (default)      Print/parse E lines for --watch seconds. Reports line
                       rate (expect ~10 Hz), ok-flag ratios (0 with motor
                       power off -- the RoboClaws and encoders are powered by
                       the motor battery), and per-motor count rates. Turn a
                       wheel by hand a little during this to see counts move
                       (small excursions only: the RoboClaw actively holds a
                       zero-velocity loop through a 71.2:1 gearbox -- don't
                       fight it hard; use --creep for the real sign test).

  --stream-test N      B1 mangling gate: stream alternating A=0/1 B=1/0 lines
                       at 20 Hz for N seconds while encoder polling runs.
                       1 count/s can't visibly move the base even with motor
                       power on. PASS = every sent value acknowledged by a
                       "Parsed" echo and ~0 unparseable warnings.

  --creep              POWERED sign test -- REQUIRES MOTOR POWER ON, robot on
                       the floor with ~10 cm clearance (moves ~6 cm) or wheels
                       off the ground. Commands A=--speed B=--speed for
                       --duration seconds, then zeros. PASS = all four counts
                       increase (signs +1/+1) at ~= the commanded counts/sec.

Expected drivetrain numbers: 1993.6 counts/wheel-rev, ~6610 counts/m.
"""

import argparse
import sys
import time

import serial

DEFAULT_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"


def wrap_delta_u32(new, old):
    d = (int(new) - int(old)) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


class Bench:
    def __init__(self, port, baud):
        print(f"Opening {port} @ {baud} (this DTR-resets the Arduino)...")
        self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.5)
        self.banner = None
        self.e_lines = 0
        self.parsed = []      # (a, b) echoes seen
        self.unparseable = 0
        self.warns = 0
        self.first_e = None   # first parsed E tuple
        self.last_e = None    # last parsed E tuple
        self.ok_a_count = 0
        self.ok_b_count = 0
        self.t_first_e = None
        self.t_last_e = None

    def close(self):
        try:
            # Guaranteed stop on teardown -- same echo-confirm the NUC host
            # uses, since an encoder poll can eat a single blind stop.
            self.send_ab_confirmed(0, 0)
        finally:
            self.ser.close()

    def send_ab(self, a, b):
        self.ser.write(f"A={int(a)} B={int(b)}\n".encode())
        self.ser.flush()

    def send_ab_confirmed(self, a, b, attempts=5):
        """Re-send (a, b) until the firmware echoes 'Parsed A=a B=b', bounded.
        Returns the attempt count on success, or None if never confirmed."""
        for attempt in range(1, attempts + 1):
            self.send_ab(a, b)
            self.pump(0.15)
            if self.parsed and self.parsed[-1] == (int(a), int(b)):
                return attempt
        return None

    def pump(self, seconds, echo_raw=False):
        """Read lines for `seconds`, updating stats."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = self.ser.readline()
            if not line or not line.endswith(b"\n"):
                continue
            text = line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            if echo_raw:
                print(f"  | {text}")
            self._ingest(text)

    def _ingest(self, text):
        if text.startswith("Ready"):
            self.banner = text
        elif text.startswith("E "):
            parts = text.split()
            if len(parts) != 8:
                return
            try:
                tup = tuple(int(p) for p in parts[1:6]) + (parts[6] == "1", parts[7] == "1")
            except ValueError:
                return
            self.e_lines += 1
            now = time.time()
            if self.first_e is None:
                self.first_e, self.t_first_e = tup, now
            self.last_e, self.t_last_e = tup, now
            self.ok_a_count += 1 if tup[5] else 0
            self.ok_b_count += 1 if tup[6] else 0
        elif text.startswith("Parsed A="):
            try:
                a = int(text.split("A=", 1)[1].split()[0])
                b = int(text.split("B=", 1)[1].split()[0])
                self.parsed.append((a, b))
            except (IndexError, ValueError):
                pass
        elif text.startswith("WARN unparseable"):
            self.unparseable += 1
        elif text.startswith(("WARN", "ERROR")):
            self.warns += 1

    # ---- reports ----
    def report_watch(self, seconds):
        print(f"\n===== WATCH ({seconds:.0f} s) =====")
        print(f"banner: {self.banner or 'NOT SEEN (old firmware? port stolen mid-boot?)'}")
        if self.banner and "v7" in self.banner:
            print("  -> PASS: v7 firmware is running")
        elif self.banner:
            print("  -> FAIL: unexpected firmware version")
        span = (self.t_last_e - self.t_first_e) if self.e_lines >= 2 else 0.0
        rate = (self.e_lines - 1) / span if span > 0 else 0.0
        powered = self.e_lines > 0 and (self.ok_a_count + self.ok_b_count) > 0.5 * self.e_lines
        if powered:
            print(f"E lines: {self.e_lines} (rate {rate:.1f} Hz; expect ~10 powered)")
            print("  -> " + ("PASS" if 8.0 <= rate <= 12.0 else "FAIL (check gating/backoff)"))
        else:
            # Dead controllers: the 1 Hz send/read retry windows (~250 ms/s)
            # defer polls, so ~6-8 Hz is the correct motors-off steady state.
            print(f"E lines: {self.e_lines} (rate {rate:.1f} Hz; expect ~6-8 with motor power OFF)")
            print("  -> " + ("PASS (motors-off rate)" if 5.0 <= rate <= 12.0
                             else "FAIL (check gating/backoff)"))
        if self.e_lines:
            fa = self.ok_a_count / self.e_lines
            fb = self.ok_b_count / self.e_lines
            print(f"ok_a ratio: {fa:.2f}   ok_b ratio: {fb:.2f}")
            print("   (1.00 = RoboClaws answering; ~0 with motor power off is EXPECTED,")
            print("    and confirms the dead-controller 1 Hz backoff path)")
        if self.first_e and self.last_e and span > 0:
            names = ["A1", "A2", "B1", "B2"]
            deltas = [wrap_delta_u32(self.last_e[i + 1], self.first_e[i + 1]) for i in range(4)]
            print("count deltas over window: " +
                  "  ".join(f"{n}={d:+d} ({d / span:+.0f}/s)" for n, d in zip(names, deltas)))
        print(f"firmware warns: send-fail={self.warns} unparseable={self.unparseable}")

    def report_stream(self, sent, seconds, stop_attempts):
        print(f"\n===== STREAM-TEST ({seconds:.0f} s @ 20 Hz, {sent} lines) =====")
        acks = len(self.parsed)
        print(f"sent: {sent}   Parsed echoes: {acks}   unparseable: {self.unparseable}")
        # (echoes < sent is normal: the firmware drains line bursts and echoes
        # only the latest one -- `unparseable` is the real corruption count)
        per_min = self.unparseable * 60.0 / max(seconds, 1e-9)
        frac = self.unparseable / max(sent, 1)
        powered = self.e_lines > 0 and (self.ok_a_count + self.ok_b_count) > 0.5 * self.e_lines
        print(f"mangling: {per_min:.1f} unparseable/min "
              f"({100 * frac:.1f}% of a worst-case all-changing 20 Hz stream)")
        if powered:
            print("   context: motor power ON. This is the meaningful number: healthy")
            print("   controllers ack on the first attempt (~54 ms/send, no retry storm),")
            print("   so mangling here is only the SoftwareSerial-RX interrupt masking")
            print("   during that window. Expect low; echo-confirm repairs the rest.")
        else:
            print("   context: motor power OFF -- NOT representative. Dead controllers")
            print("   burn ~100 ms/send in retries, and because this test changes the")
            print("   command every line, the changed-setpoint backoff bypass re-enables")
            print("   those storms every line -- inflating mangling far above the powered")
            print("   case. Rerun with motor power ON for the real figure.")
        print("   Either way the NUC host repairs every eaten command via echo-confirm")
        print("   within ~0.1 s, and same-value refreshes (steady driving) never mangle.")
        # The hard gate, valid regardless of motor power: a STOP must land, via
        # the same bounded re-send loop the NUC host uses (echo-confirm).
        if stop_attempts is None:
            print("final-delivery gate: FAIL -- stop never echoed after 5 re-sends; report this")
        else:
            print(f"final-delivery gate: PASS (A=0 B=0 echoed after "
                  f"{stop_attempts} send{'s' if stop_attempts > 1 else ''})")

    def report_creep(self, speed, duration, before, after):
        print(f"\n===== CREEP (A={speed} B={speed} for {duration:.1f} s) =====")
        if before is None or after is None:
            print("FAIL: no valid encoder frames around the creep window "
                  "(is motor power on? are ok flags 1?)")
            return
        names = ["A1", "A2", "B1", "B2"]
        deltas = [wrap_delta_u32(after[i + 1], before[i + 1]) for i in range(4)]
        dt_fw = (after[0] - before[0]) / 1000.0
        print(f"firmware dt: {dt_fw:.2f} s")
        ok_signs = all(d > 0 for d in deltas)
        for n, d in zip(names, deltas):
            rate = d / dt_fw if dt_fw > 0 else 0.0
            print(f"  {n}: {d:+6d} counts  ({rate:+7.1f}/s; commanded {speed})")
        print("signs: " + ("PASS: all four positive -> side_a_sign=+1 side_b_sign=+1"
                           if ok_signs else
                           "CHECK: note which are negative and set side_*_sign accordingly"))
        if dt_fw > 0:
            rates = [abs(d / dt_fw) for d in deltas]
            near = all(0.7 * speed <= r <= 1.3 * speed for r in rates)
            print("QPPS check: " + ("PASS: count rates ~= commanded units -> units are counts/sec"
                                    if near else
                                    "CHECK: rates far from commanded value (PID limits? stiction?)"))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--watch", type=float, default=10.0,
                   help="seconds to passively watch E lines (default 10)")
    p.add_argument("--raw", action="store_true", help="echo every raw line")
    p.add_argument("--stream-test", type=float, default=0.0, metavar="SECONDS",
                   help="run the 20 Hz command-mangling gate for SECONDS")
    p.add_argument("--creep", action="store_true",
                   help="POWERED sign test (moves the robot ~6 cm!)")
    p.add_argument("--speed", type=int, default=200, help="creep speed, counts/s")
    p.add_argument("--duration", type=float, default=2.0, help="creep seconds")
    args = p.parse_args()

    bench = Bench(args.port, args.baud)
    try:
        # Boot: DTR reset -> bootloader -> sketch; banner within ~3 s.
        bench.pump(3.0, echo_raw=args.raw)

        if args.watch > 0:
            print(f"\nWatching E lines for {args.watch:.0f} s "
                  "(nudge a wheel gently to see counts move)...")
            bench.pump(args.watch, echo_raw=args.raw)
            bench.report_watch(args.watch)

        if args.stream_test > 0:
            print(f"\nStreaming alternating 1-count commands at 20 Hz for "
                  f"{args.stream_test:.0f} s...")
            bench.parsed = []
            bench.unparseable = 0
            sent = 0
            t_end = time.time() + args.stream_test
            flip = False
            while time.time() < t_end:
                bench.send_ab(1 if flip else 0, 0 if flip else 1)
                flip = not flip
                sent += 1
                bench.pump(0.05, echo_raw=False)
            # Deliver the final stop the way the NUC host does: re-send until
            # the "Parsed" echo confirms it (bounded).
            stop_attempts = bench.send_ab_confirmed(0, 0)
            bench.pump(0.5)
            bench.report_stream(sent, args.stream_test, stop_attempts)

        if args.creep:
            print("\n*** POWERED CREEP TEST ***")
            print(f"The base will drive FORWARD ~{args.speed * args.duration / 6610.0 * 100:.0f} cm "
                  f"at ~{args.speed / 6610.0 * 100:.1f} cm/s.")
            print("Motor power ON, robot on the floor with clearance (or wheels lifted).")
            if input("Type YES to proceed: ").strip() != "YES":
                print("Aborted.")
            else:
                bench.pump(0.5)
                before = bench.last_e
                try:
                    bench.send_ab(args.speed, args.speed)
                    bench.pump(args.duration)
                finally:
                    # Confirmed stop: a single blind stop can be eaten by an
                    # encoder poll, leaving the base creeping until the 5 s
                    # firmware watchdog. Re-send until echoed.
                    if bench.send_ab_confirmed(0, 0) is None:
                        print("WARNING: stop not confirmed after 5 sends -- "
                              "hit the e-stop if the base is still moving!")
                after = bench.last_e
                bench.report_creep(args.speed, args.duration, before, after)
    finally:
        bench.close()
    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
