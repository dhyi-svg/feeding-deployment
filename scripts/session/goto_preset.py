"""Send the arm to a named joint preset from config/local_arm_presets.yaml.

DRY RUN by default -- prints the target and the per-joint delta, commands nothing.
Pass --execute to move.

    python scripts/session/goto_preset.py microwave_view_pos
    python scripts/session/goto_preset.py microwave_view_pos --execute

This is a JOINT-space move (no IK, no Cartesian), so there is no branch-flip risk:
the target IS a joint vector. The only guard that matters is the size of the move,
so it aborts if any single joint would travel more than MAX_JUMP_DEG. Raise that
deliberately with --max-jump if you know the arm is far from the preset.

Requires arm_server.py + bulldog_bypass.py. Set the speed first
(scripts/session/arm_set_speed.py low).
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

PRESETS = Path(__file__).resolve().parents[2] / "config" / "local_arm_presets.yaml"

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("preset", help="key in config/local_arm_presets.yaml")
parser.add_argument("--execute", action="store_true", help="actually command the arm")
parser.add_argument("--max-jump", type=float, default=90.0, help="abort above this per-joint delta (deg)")
args = parser.parse_args()

presets = yaml.safe_load(PRESETS.read_text())
if args.preset not in presets:
    sys.exit(f"No preset '{args.preset}'. Available: {', '.join(presets)}")
entry = presets[args.preset]

values = np.asarray(entry["values"], dtype=float)
units = entry.get("units", "degrees")
target = values if units == "radians" else np.radians(values)

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
arm = manager.ArmInterface()
state = arm.get_state()
current = np.asarray(state["position"], dtype=float)
gripper = float(state.get("gripper_pos"))

# Wrap each delta into [-180, 180): joint angles are periodic, so a raw subtraction
# can report a ~350 deg "move" for what is physically a few degrees the other way.
delta = np.degrees((target - current + np.pi) % (2 * np.pi) - np.pi)

print(f"preset        : {args.preset}")
print(f"current (deg) : {[round(float(np.degrees(v)), 1) for v in current]}")
print(f"target  (deg) : {[round(float(np.degrees(v)), 1) for v in target]}")
print(f"delta   (deg) : {[round(float(v), 1) for v in delta]}")
print(f"max joint jump: {np.max(np.abs(delta)):.1f} deg  (limit {args.max_jump})")
print(f"gripper       : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")

if gripper > 0.2:
    sys.exit("Gripper is CLOSED -- it may be holding something. Refusing to move to a preset.")
if np.max(np.abs(delta)) > args.max_jump:
    sys.exit(f"Move too large ({np.max(np.abs(delta)):.0f} deg). Re-run with --max-jump if intended.")

if not args.execute:
    sys.exit("\nDRY RUN -- nothing commanded. Re-run with --execute to move.")

print(f"\nspeed: {arm.get_speed()}  -- commanding joint move ...")
# Raw ArmManager proxy exposes set_joint_position, not the client-level
# execute_command; for a JointCommand the client just forwards to this.
arm.set_joint_position(target.tolist())
for _ in range(150):
    time.sleep(0.2)
    if float(np.max(np.abs(np.asarray(arm.get_state()["velocity"], dtype=float)))) < 1e-3:
        break
time.sleep(0.4)

final = np.asarray(arm.get_state()["position"], dtype=float)
residual = np.degrees((target - final + np.pi) % (2 * np.pi) - np.pi)
print(f"final   (deg) : {[round(float(np.degrees(v)), 1) for v in final]}")
print(f"residual(deg) : {[round(float(v), 2) for v in residual]}   max {np.max(np.abs(residual)):.2f}")
print(f"EE            : {[round(float(v), 4) for v in list(arm.get_state()['ee_pos'])[:3]]}")
