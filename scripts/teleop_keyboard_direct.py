#!/usr/bin/env python3
"""
teleop_keyboard_direct.py -- DIRECT-SERIAL WASD/arrow-key teleop for the Vention
base, for quick on-robot testing.

Unlike control/base_controller/vention_teleop_keyboard.py (which drives through
BaseInterfaceClient -> base_server RPC -> bulldog gate), this talks STRAIGHT to
the Arduino over serial via VentionBase -- the same path calibrate_wheel_odom.py
and bench_test_encoders.py use. No ROS, no base_server, no bulldog.

  RUN ON THE NUC, with base_server STOPPED (it owns the Arduino port; this script
  can't share it). Motor power on, clear floor / wheels lifted.

    python3 scripts/teleop_keyboard_direct.py
    python3 scripts/teleop_keyboard_direct.py --max_translation 300 --max_rotation 250

Keys:
  w / up      forward           s / down   backward
  a / left    turn left         d / right  turn right
  + / -       faster / slower (scales both speeds)
  space       stop              q / ESC    quit (stops + disconnects)

Hold a key to keep moving (terminal key-repeat feeds a dead-man; release -> stop).
Speeds are motor "counts/s" (same units as the calibration tools).
"""

import argparse
import os
import select
import sys
import termios
import time
import tty

COMMAND_HZ = 20.0

DEFAULT_PORT = ("/dev/serial/by-id/"
                "usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00")
DEFAULT_MAX_TRANSLATION_SPEED = 500   # counts/s
DEFAULT_MAX_ROTATION_SPEED = 400      # counts/s
TURN_IN_PLACE_THRESHOLD = 0.20

# Terminals deliver no key-up, so "held" is inferred from OS key-repeat presses,
# which only START after the keyboard's initial repeat delay (~0.5 s, more over
# VNC). Two timeouts: a generous grace after the FIRST press (bridges the repeat
# delay -- a single short decay caused a spurious (0,0) blip), then a short decay
# once repeats stream (snappy stop on release).
INITIAL_GRACE_SECONDS = 0.65
DECAY_SECONDS = 0.25
REPEAT_DETECT_SECONDS = 0.45

# ANSI arrow escapes ("\x1b[A".."\x1b[D") -> WASD. Parsed BEFORE the ESC=quit
# check, else every arrow (the natural driving keys over ssh/VNC) quits.
ARROW_KEYS = {"A": "w", "B": "s", "D": "a", "C": "d"}


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def compute_wheel_speeds(x_axis, y_axis, max_translation, max_rotation):
    v = -y_axis
    w = x_axis
    if abs(v) < TURN_IN_PLACE_THRESHOLD:
        v = 0.0
    speed_v = v * max_translation
    speed_w = w * max_rotation
    speed_a = int(clamp(speed_v - speed_w, -max_translation, max_translation))
    speed_b = int(clamp(speed_v + speed_w, -max_translation, max_translation))
    return speed_a, speed_b


def drain_stdin():
    """Non-blocking read of all available stdin bytes via the RAW fd (os.read):
    sys.stdin.read(1) buffers internally and splits arrow escapes across ticks."""
    fd = sys.stdin.fileno()
    data = b""
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        data += chunk
    return data.decode("utf-8", errors="ignore")


def parse_pending(s):
    """Consume complete key tokens. Returns (keys, remainder, quit). keys are
    'w'/'a'/'s'/'d'/' '/'+'/'-'; remainder is an incomplete trailing escape."""
    keys = []
    i, n = 0, len(s)
    while i < n:
        ch = s[i]
        if ch == "\x1b":
            if i + 1 >= n or (s[i + 1] == "[" and i + 2 >= n):
                return keys, s[i:], False
            if s[i + 1] == "[":
                if s[i + 2] in ARROW_KEYS:
                    keys.append(ARROW_KEYS[s[i + 2]])
                i += 3
                continue
            return keys, "", True
        lower = ch.lower()
        if lower == "q":
            return keys, "", True
        if ch == " ":
            keys.append(" ")
        elif lower in ("w", "a", "s", "d"):
            keys.append(lower)
        elif ch in "+=":
            keys.append("+")
        elif ch in "-_":
            keys.append("-")
        i += 1
    return keys, "", False


def main():
    parser = argparse.ArgumentParser(
        description="Direct-serial WASD teleop for the Vention base (no base_server).")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Arduino by-id serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--max_translation", type=int, default=DEFAULT_MAX_TRANSLATION_SPEED,
                        help="max forward/back wheel speed (counts/s)")
    parser.add_argument("--max_rotation", type=int, default=DEFAULT_MAX_ROTATION_SPEED,
                        help="max turning contribution (counts/s)")
    parser.add_argument("--decay", type=float, default=DECAY_SECONDS,
                        help="seconds without a repeat press before release (repeat streaming)")
    parser.add_argument("--initial_grace", type=float, default=INITIAL_GRACE_SECONDS,
                        help="hold grace after the FIRST press (bridges OS repeat delay)")
    args = parser.parse_args()

    # Import the serial backend only now, so --help works without pyserial and
    # the script runs standalone from the repo (no ROS/workspace sourcing).
    repo_src = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "src")
    if repo_src not in sys.path:
        sys.path.insert(0, repo_src)
    try:
        from feeding_deployment.control.base_controller.vention_arduino_control import VentionBase
    except Exception as e:  # noqa: BLE001
        print(f"[!] could not import VentionBase ({e}). Run from the repo, pyserial installed.")
        return 2

    base = VentionBase(args.port, args.baud)
    if not base.bridge.connection_status:
        print(f"[!] could not open the Arduino at {args.port}.")
        print("[!] Is base_server still running and holding the port? Stop it first "
              "(NUC 'robot' session -> Ctrl-C the base pane).")
        return 1

    print(__doc__.split("Keys:")[1] if "Keys:" in __doc__ else "")
    print("driving DIRECT to the Arduino (base_server bypassed). "
          "space=stop, q/ESC=quit.")

    period = 1.0 / COMMAND_HZ
    scale = 1.0
    last_sent = None
    last_press = {}   # key -> (last press ts, auto-repeat confirmed)

    def held(key, now):
        t, repeating = last_press.get(key, (0.0, False))
        limit = args.decay if repeating else args.initial_grace
        return now - t < limit

    pending = ""
    pending_since = 0.0
    ESC_TIMEOUT = 0.15

    fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while True:
            now = time.monotonic()
            new = drain_stdin()
            if new:
                if not pending:
                    pending_since = now
                pending += new
            keys, pending, quit_requested = parse_pending(pending)
            if pending and now - pending_since > ESC_TIMEOUT:
                pending = ""
                quit_requested = True
            for key in keys:
                if key == " ":
                    last_press.clear()
                elif key == "+":
                    scale = clamp(scale + 0.1, 0.1, 1.5)
                    print(f"[speed] {int(scale*100)}%\r")
                elif key == "-":
                    scale = clamp(scale - 0.1, 0.1, 1.5)
                    print(f"[speed] {int(scale*100)}%\r")
                else:
                    prev_t, _ = last_press.get(key, (0.0, False))
                    repeating = (now - prev_t) < REPEAT_DETECT_SECONDS
                    last_press[key] = (now, repeating)
            if quit_requested:
                break

            x_axis = y_axis = 0.0
            if held("w", now):
                y_axis -= 1.0
            if held("s", now):
                y_axis += 1.0
            if held("a", now):
                x_axis -= 1.0
            if held("d", now):
                x_axis += 1.0

            speed_a, speed_b = compute_wheel_speeds(
                x_axis, y_axis,
                int(args.max_translation * scale), int(args.max_rotation * scale))

            cmd = (speed_a, speed_b)
            if cmd != last_sent:
                print(f"[->] A={speed_a} B={speed_b}\r")
                last_sent = cmd

            try:
                base.set_speeds(speed_a, speed_b)
            except Exception as e:  # noqa: BLE001
                print(f"\r\n[!] set_speeds failed: {e}\r")
                break

            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[!] Stopping.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
        try:
            base.stop()
            base.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main() or 0)
