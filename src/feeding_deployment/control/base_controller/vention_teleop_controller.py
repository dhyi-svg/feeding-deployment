#!/usr/bin/env python3
import time
import argparse

import pygame

from base_client import BaseInterfaceClient


DEADBAND = 0.12
COMMAND_HZ = 5.0

# Reasonable starting values; tune on robot if needed.
# DEFAULT_MAX_TRANSLATION_SPEED = 500
# DEFAULT_MAX_ROTATION_SPEED = 400
# DEFAULT_MAX_TRANSLATION_SPEED = 1000
# DEFAULT_MAX_ROTATION_SPEED = 800
DEFAULT_MAX_TRANSLATION_SPEED = 487   # counts/s ≈ 0.10 m/s @ 4874 counts/m
DEFAULT_MAX_ROTATION_SPEED = 433      # counts/s ≈ 0.1667 rad/s @ 2600 counts/(rad/s)

TURN_IN_PLACE_THRESHOLD = 0.20


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def compute_wheel_speeds(
    x_axis: float,
    y_axis: float,
    max_translation: int,
    max_rotation: int,
) -> tuple[int, int]:
    """
    Xbox-style left stick mapping:
      - stick up    -> forward
      - stick down  -> backward
      - stick left  -> turn left
      - stick right -> turn right

    pygame usually reports:
      - y_axis < 0 when pushing stick upward
      - x_axis < 0 when pushing stick left

    Differential-drive mix:
      left  = v + w
      right = v - w

    In this codebase:
      Driver A = right side
      Driver B = left side

    So:
      A = right = v - w
      B = left  = v + w
    """
    x = apply_deadband(x_axis, DEADBAND)
    y = apply_deadband(y_axis, DEADBAND)

    # Forward when pushing stick up
    v = -y
    w = x

    # If translation is very small, allow cleaner turn-in-place behavior
    if abs(v) < TURN_IN_PLACE_THRESHOLD:
        v = 0.0

    speed_v = v * max_translation
    speed_w = w * max_rotation

    # A = right wheel, B = left wheel
    speed_a = int(clamp(speed_v - speed_w, -max_translation, max_translation))
    speed_b = int(clamp(speed_v + speed_w, -max_translation, max_translation))

    return speed_a, speed_b


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Joystick teleop for Vention base via Arduino bridge"
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
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No game controller connected.")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Controller: {joystick.get_name()}")

    base = BaseInterfaceClient()

    period = 1.0 / COMMAND_HZ
    last_sent: tuple[int, int] | None = None

    try:
        while True:
            pygame.event.pump()

            # Left stick axes
            x_axis = joystick.get_axis(0)
            y_axis = joystick.get_axis(1)

            # print(f"[raw] x={x_axis:.3f} y={y_axis:.3f}")

            speed_a, speed_b = compute_wheel_speeds(
                x_axis,
                y_axis,
                args.max_translation,
                args.max_rotation,
            )

            cmd = (speed_a, speed_b)
            if cmd != last_sent:
                print(f"[→] A={speed_a} B={speed_b}")
                last_sent = cmd

            base.set_speeds(speed_a, speed_b)

            time.sleep(period)

    except KeyboardInterrupt:
        print("\n[!] Stopping.")
    finally:
        try:
            base.stop()
        except Exception:
            pass
        try:
            joystick.quit()
        except Exception:
            pass
        pygame.quit()


if __name__ == "__main__":
    main()
