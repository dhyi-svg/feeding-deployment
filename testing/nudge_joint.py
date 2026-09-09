"""Nudge ONE joint by a small delta. Dry run by default; --execute to move.

    $PY testing/nudge_joint.py                          # dry run, commands nothing
    $PY testing/nudge_joint.py --execute                # J5 by +5 deg, with a y/n prompt
    $PY testing/nudge_joint.py --joint 4 --deg -5 --execute

Run it as a FILE, not a heredoc -- input() cannot read a prompt when stdin is
already consumed by the heredoc. Ctrl-C does NOT recall a command arm_server
already holds: stop is `pkill -f bulldog_bypass.py` or the physical e-stop.

Needs arm_server.py + bulldog_bypass.py up, and ROS sourced (ArmInterfaceClient
asserts ROS_AVAILABLE). Set the speed first: scripts/session/arm_set_speed.py low
"""
import argparse
import math
import sys

import numpy as np

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand

MAX_DEG = 15.0  # refuse more than this for a single-joint nudge

# Kortex refuses REACH_JOINT_ANGLES while the tool sits in the arm's own base
# protection zone (cylinder r=0.15 m, h=0.16 m centred z=0.07) -- it aborts with
# METHOD_FAILED and no fault, so warn before the command looks mysteriously dead.
ZONE_RADIUS, ZONE_TOP = 0.15, 0.15

p = argparse.ArgumentParser()
p.add_argument("--joint", type=int, default=4, help="0-based index (4 = J5, wrist)")
p.add_argument("--deg", type=float, default=5.0, help="signed delta, degrees")
p.add_argument("--execute", action="store_true", help="actually command the arm")
p.add_argument("--yes", action="store_true", help="skip the interactive confirmation")
args = p.parse_args()

if not 0 <= args.joint <= 6:
    sys.exit(f"joint index must be 0-6, got {args.joint}")
if abs(args.deg) > MAX_DEG:
    sys.exit(f"refusing {args.deg:+.1f} deg: over the {MAX_DEG} deg single-joint limit")


def wrap_deg(a):
    """Wrap degrees into [-180, 180] -- same correction move_angular applies."""
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


print("Connecting to arm_server...")
ai = ArmInterfaceClient()
state = ai.get_state()
current = np.array(state["position"], dtype=float)
ee = state["ee_pos"]
print(f"Connected. gripper_pos = {state['gripper_pos']:.4f}")

radius = math.hypot(ee[0], ee[1])
print(f"tool at [{ee[0]:.4f}, {ee[1]:.4f}, {ee[2]:.4f}]  radius {radius:.4f} m")
if radius < ZONE_RADIUS and ee[2] <= ZONE_TOP:
    print("  WARNING: tool is inside the base protection zone.")
    print("  Kortex will abort with METHOD_FAILED and no fault. Move it clear first.")
print()

target = current.copy()
target[args.joint] += np.radians(args.deg)

print("  joint          current     target      delta")
for i, (c, t) in enumerate(zip(np.degrees(current), np.degrees(target))):
    mark = "   <-- moving" if i == args.joint else ""
    print(f"  J{i + 1} (idx {i})  {c:9.3f}  {t:9.3f}  {wrap_deg(t - c):9.3f}{mark}")

if not args.execute:
    print("\nDRY RUN -- nothing commanded. Re-run with --execute to move.")
    sys.exit(0)

print(f"\nAbout to move J{args.joint + 1} by {args.deg:+.2f} deg.")
print("Ctrl-C will NOT stop an in-flight move.")
print("Stop = `pkill -f bulldog_bypass.py`, or the physical e-stop.")

if not args.yes:
    # The bug this script exists to avoid: no tty means input() would EOFError.
    if not sys.stdin.isatty():
        sys.exit("stdin is not a terminal -- run from a real terminal, or pass --yes.")
    if input("Type 'y' to move: ").strip().lower() != "y":
        sys.exit("Cancelled -- no command sent.")

print("Sending command...")
ai.execute_command(JointCommand(pos=target.tolist()))
print("Command returned.\n")

final = np.array(ai.get_state()["position"], dtype=float)
err = wrap_deg(np.degrees(final - target))

moved = abs(err[args.joint])
print(f"J{args.joint + 1} landed {err[args.joint]:+.3f} deg from target", end="")
print("  <-- DID NOT REACH TARGET" if moved > 0.5 else "")

others = np.abs(err)
others[args.joint] = 0.0
if np.any(others > 0.5):
    print("WARNING: unintended joints moved:", np.round(others, 3))
elif moved <= 0.5:
    print("Confirmed: only the intended joint moved, and it reached the target.")
