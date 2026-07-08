#!/usr/bin/env python3
"""
calibrate_wheel_odom.py -- drive the base a controlled amount and report
encoder count deltas, for calibrating wheel_odom_publisher's
counts_per_meter and track_width_m.

Run ON THE NUC, base_server STOPPED (exclusive port), motor power ON, robot
supervised with the physical e-stop in reach. Talks straight to the Arduino
(firmware v7) over serial -- same path as bench_test_encoders.py --creep.

  straight  drive forward, then STOP. Tape-measure the start->stop distance D.
            counts_per_meter = mean4_counts / D
  rotate    spin in place, then STOP. Measure the turned angle A (deg).
            track_width_m = (right_mean - left_mean)_counts / counts_per_meter
                            / radians(A)

The command is DTR-reset-safe and always ends with an echo-confirmed stop.
Counts span accel+cruise+decel (baseline sampled just before motion, final
after settle), matching the tape displacement.
"""

import argparse
import math
import sys
import time

import serial

DEFAULT_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"


def wrap_delta_u32(new, old):
    d = (int(new) - int(old)) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


class Driver:
    def __init__(self, port, baud):
        print(f"Opening {port} (DTR-resets the Arduino)...")
        self.ser = serial.Serial(port, baud, timeout=0.2, write_timeout=0.5)
        self.last_valid = None   # (millis, a1, a2, b1, b2)
        self.last_echo = None

    def close(self):
        self.ser.close()

    def send(self, a, b):
        self.ser.write(f"A={int(a)} B={int(b)}\n".encode())
        self.ser.flush()

    def send_confirmed(self, a, b, attempts=6):
        for k in range(1, attempts + 1):
            self.send(a, b)
            self.pump(0.15)
            if self.last_echo == (int(a), int(b)):
                return k
        return None

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            line = self.ser.readline()
            if not line or not line.endswith(b"\n"):
                continue
            t = line.decode("utf-8", errors="replace").strip()
            if t.startswith("E "):
                p = t.split()
                if len(p) == 8 and p[6] == "1" and p[7] == "1":
                    try:
                        self.last_valid = tuple(int(x) for x in p[1:6])
                    except ValueError:
                        pass
            elif t.startswith("Parsed A="):
                try:
                    a = int(t.split("A=", 1)[1].split()[0])
                    b = int(t.split("B=", 1)[1].split()[0])
                    self.last_echo = (a, b)
                except (IndexError, ValueError):
                    pass

    def wait_valid(self, timeout=5.0):
        self.last_valid = None
        end = time.time() + timeout
        while time.time() < end and self.last_valid is None:
            self.pump(0.2)
        return self.last_valid

    def run(self, a, b, duration):
        base = self.wait_valid()
        if base is None:
            print("ERROR: no valid (ok=1) encoder frame -- is motor power on?")
            return None, None
        started = self.send_confirmed(a, b)
        if started is None:
            print("ERROR: drive command not confirmed; aborting.")
            self.send_confirmed(0, 0)
            return None, None
        # Keep the firmware's CMD_STALE_MS (5 s) watchdog fed: re-send the drive
        # command ~every 1 s for the whole duration. A single send would let the
        # watchdog zero the base ~5 s in. Re-sending the SAME value is free on
        # the controller side (change-only send) but refreshes lastValidCmdMs.
        end = time.time() + duration
        while time.time() < end:
            self.send(a, b)
            self.pump(1.0)
        self.send_confirmed(0, 0)
        self.pump(0.6)  # let the velocity loop settle before final read
        final = self.wait_valid()
        return base, final


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["straight", "rotate"])
    p.add_argument("--port", default=DEFAULT_PORT)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--speed", type=int, default=400, help="counts/s per motor")
    p.add_argument("--duration", type=float, default=8.0, help="drive seconds")
    p.add_argument("--counts-per-meter", type=float, default=6610.0,
                   help="for rotate: the value from a prior straight run")
    p.add_argument("--yes", action="store_true", help="skip the confirm prompt")
    args = p.parse_args()

    a = args.speed
    b = args.speed if args.mode == "straight" else -args.speed
    approx_m = args.speed * args.duration / 6610.0
    print(f"\n*** {args.mode.upper()} CALIBRATION -- THE BASE WILL MOVE ***")
    if args.mode == "straight":
        print(f"Forward ~{approx_m*100:.0f} cm at ~{args.speed/6610.0*100:.1f} cm/s "
              f"(A={a} B={b} for {args.duration:.0f} s).")
    else:
        print(f"Spin in place for {args.duration:.0f} s (A={a} B={b}). "
              f"Watch footprint clearance.")
    print("Motor power ON, area clear, physical e-stop in reach.")
    if not args.yes and input("Type YES to proceed: ").strip() != "YES":
        print("Aborted.")
        return 0

    drv = Driver(args.port, args.baud)
    try:
        drv.pump(3.0)  # boot / banner
        base, final = drv.run(a, b, args.duration)
    finally:
        drv.send_confirmed(0, 0)
        drv.close()
    if base is None or final is None:
        print("No usable data.")
        return 1

    names = ["A1", "A2", "B1", "B2"]
    d = [wrap_delta_u32(final[i + 1], base[i + 1]) for i in range(4)]
    dt = (final[0] - base[0]) / 1000.0
    print(f"\n===== {args.mode.upper()} result (dt={dt:.2f} s) =====")
    for n, di in zip(names, d):
        print(f"  {n}: {di:+7d} counts")
    right_mean = (d[0] + d[1]) / 2.0
    left_mean = (d[2] + d[3]) / 2.0
    mean4 = sum(d) / 4.0

    if args.mode == "straight":
        print(f"\nmean of 4 motors: {mean4:+.1f} counts")
        print(f"per-motor spread: {max(d) - min(d)} counts "
              f"({'OK, <2%' if (max(d)-min(d)) < 0.02*abs(mean4) else 'high -- slip/veer?'})")
        print("\n>>> Tape-measure the straight-line distance the base moved (D, meters).")
        print(">>> counts_per_meter = %.1f / D" % mean4)
        for D in (0.5, 1.0, 1.5, 2.0):
            print(f"      if D = {D:.2f} m  ->  counts_per_meter = {mean4 / D:.0f}")
    else:
        print(f"\nright_mean = {right_mean:+.1f}  left_mean = {left_mean:+.1f} counts")
        diff = right_mean - left_mean
        print(f"(right_mean - left_mean) = {diff:+.1f} counts")
        print(f"\n>>> Measure the angle turned (A, degrees; note direction).")
        print(">>> track_width_m = (right_mean - left_mean)/counts_per_meter / radians(A)")
        print(f">>> using counts_per_meter = {args.counts_per_meter:.0f}:")
        for A in (360.0, 720.0):
            tw = (diff / args.counts_per_meter) / math.radians(A)
            print(f"      if |A| = {A:.0f} deg  ->  track_width_m = {abs(tw):.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
