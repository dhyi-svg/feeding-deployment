#!/usr/bin/env python3
import argparse
import os
import select
import sys
import termios
import time
import tty

from base_client import BaseInterfaceClient


COMMAND_HZ = 20.0

DEFAULT_MAX_TRANSLATION_SPEED = 500
DEFAULT_MAX_ROTATION_SPEED = 400
TURN_IN_PLACE_THRESHOLD = 0.20

# A direction is considered "held" as long as a press for it has arrived
# within the last DECAY_SECONDS. Terminals don't deliver key-up, so we
# simulate hold via repeated key-repeat presses + this timeout.
DECAY_SECONDS = 0.30

# ANSI arrow-key escape sequences ("\x1b[A".."\x1b[D") mapped to WASD. Arrows
# must be parsed BEFORE the quit-on-ESC check: they start with the ESC byte,
# so naive handling quits the tool on every arrow press (the natural driving
# keys -- this bit ssh/VNC users).
ARROW_KEYS = {"A": "w", "B": "s", "D": "a", "C": "d"}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_wheel_speeds(
    x_axis: float,
    y_axis: float,
    max_translation: int,
    max_rotation: int,
) -> tuple[int, int]:
    v = -y_axis
    w = x_axis

    if abs(v) < TURN_IN_PLACE_THRESHOLD:
        v = 0.0

    speed_v = v * max_translation
    speed_w = w * max_rotation

    speed_a = int(clamp(speed_v - speed_w, -max_translation, max_translation))
    speed_b = int(clamp(speed_v + speed_w, -max_translation, max_translation))

    return speed_a, speed_b


def drain_stdin() -> str:
    """Non-blocking read of all currently-available bytes from stdin.

    Reads the RAW fd with os.read: sys.stdin.read(1) pulls everything into
    Python's internal buffer (making select() on the fd report empty), which
    split arrow-key escape sequences across drain calls -- the bare leading
    ESC then read as a quit.
    """
    fd = sys.stdin.fileno()
    data = b""
    while select.select([fd], [], [], 0)[0]:
        chunk = os.read(fd, 64)
        if not chunk:
            break
        data += chunk
    return data.decode("utf-8", errors="ignore")


def parse_pending(s: str):
    """Consume complete key tokens from s.

    Returns (keys, remainder, quit): keys are logical presses ('w'/'a'/'s'/
    'd'/' '), remainder is an incomplete trailing escape sequence to retry
    next tick (caller times it out into a bare-ESC quit), quit means ESC/q.
    """
    keys: list[str] = []
    i = 0
    n = len(s)
    while i < n:
        ch = s[i]
        if ch == "\x1b":
            # Possibly an arrow sequence "\x1b[X"; keep if incomplete.
            if i + 1 >= n or (s[i + 1] == "[" and i + 2 >= n):
                return keys, s[i:], False
            if s[i + 1] == "[":
                if s[i + 2] in ARROW_KEYS:
                    keys.append(ARROW_KEYS[s[i + 2]])
                i += 3  # consume the sequence either way
                continue
            return keys, "", True  # ESC followed by a non-sequence: quit
        lower = ch.lower()
        if lower == "q":
            return keys, "", True
        if ch == " ":
            keys.append(" ")
        elif lower in ("w", "a", "s", "d"):
            keys.append(lower)
        i += 1
    return keys, "", False


def main() -> None:
    parser = argparse.ArgumentParser(
        description="WASD terminal teleop for Vention base via Arduino bridge"
    )
    parser.add_argument(
        "--max_translation",
        type=int,
        default=DEFAULT_MAX_TRANSLATION_SPEED,
        help="Maximum forward/backward wheel speed",
    )
    parser.add_argument(
        "--max_rotation",
        type=int,
        default=DEFAULT_MAX_ROTATION_SPEED,
        help="Maximum turning contribution",
    )
    parser.add_argument(
        "--decay",
        type=float,
        default=DECAY_SECONDS,
        help="Seconds without a repeat keypress before a direction is released",
    )
    args = parser.parse_args()

    base = BaseInterfaceClient()

    # Fail loudly up front (readable message beats a mid-drive traceback):
    # set_speeds is gated on bulldog being registered on the NUC.
    try:
        base.set_speeds(0, 0)
    except Exception as e:
        print(f"[!] Base refused commands: {e}")
        print("[!] Is the NUC 'robot' stack up (base_server AND bulldog)?")
        return

    print("WASD or arrow keys to drive, space=stop, q or ESC to quit. "
          "Hold a key to keep moving.")

    period = 1.0 / COMMAND_HZ
    last_sent: tuple[int, int] | None = None
    last_press: dict[str, float] = {}  # key -> monotonic timestamp
    pending = ""          # incomplete escape sequence awaiting continuation
    pending_since = 0.0
    ESC_TIMEOUT = 0.15    # bare ESC (no continuation) quits after this

    fd = sys.stdin.fileno()
    old_termios = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        running = True
        while running:
            now = time.monotonic()

            new = drain_stdin()
            if new:
                if not pending:
                    pending_since = now
                pending += new
            keys, pending, quit_requested = parse_pending(pending)
            if pending and now - pending_since > ESC_TIMEOUT:
                # Partial escape never completed: it was a bare ESC press.
                pending = ""
                quit_requested = True
            for key in keys:
                if key == " ":
                    last_press.clear()
                else:
                    last_press[key] = now
            if quit_requested:
                break

            x_axis = 0.0
            y_axis = 0.0
            if now - last_press.get("w", 0.0) < args.decay:
                y_axis -= 1.0
            if now - last_press.get("s", 0.0) < args.decay:
                y_axis += 1.0
            if now - last_press.get("a", 0.0) < args.decay:
                x_axis -= 1.0
            if now - last_press.get("d", 0.0) < args.decay:
                x_axis += 1.0

            speed_a, speed_b = compute_wheel_speeds(
                x_axis,
                y_axis,
                args.max_translation,
                args.max_rotation,
            )

            cmd = (speed_a, speed_b)
            if cmd != last_sent:
                print(f"[->] A={speed_a} B={speed_b}\r")
                last_sent = cmd

            try:
                base.set_speeds(speed_a, speed_b)
            except Exception as e:
                print(f"\r\n[!] set_speeds failed: {e}\r")
                print("[!] Base link lost (base_server/bulldog down?). Exiting.\r")
                break

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n[!] Stopping.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_termios)
        try:
            base.stop()
        except Exception:
            pass


if __name__ == "__main__":
    main()
