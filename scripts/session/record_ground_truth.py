"""Record the arm's current EE pose as a ground-truth handle label.

Teleop the gripper so it is CENTRED on the handle bar (left-right) at the depth where
you would close on it, then run this. The recorded pose is directly comparable to what
the detector outputs: HANDLE_DEPTH_CORR was fitted so the corrected handle equals the EE
pose when touching the handle, which is why GRIP_EXT is 0.

    python scripts/session/record_ground_truth.py --tag pos_A ~/captures/grasp_dry ~/captures/grasp_exec
    python scripts/session/record_ground_truth.py --list

Labels are per APPLIANCE POSITION, not per session -- moving the microwave is expected,
so give each position its own --tag and keep collecting. A corpus spanning several
positions and orientations is the point; it is what shows whether a method generalises
rather than fitting one setup.

Height along a long vertical bar is ambiguous: touch at a repeatable height (mid-bar)
and score depth/lateral error separately from along-bar error.

Appends to ~/captures/ground_truth.jsonl and drops ground_truth.json into each capture
dir given, so a capture carries its own label.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

LEDGER = Path.home() / "captures" / "ground_truth.jsonl"
MOVED_WARN_M = 0.05

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("captures", nargs="*", type=Path, help="capture dirs this label applies to")
parser.add_argument("--tag", help="appliance-position tag, e.g. pos_A. Required unless --list")
parser.add_argument("--note", default="", help="free text, e.g. 'rotated 20 deg, clutter cleared'")
parser.add_argument("--list", action="store_true", help="show recorded labels and exit")
args = parser.parse_args()

if args.list:
    if not LEDGER.exists():
        raise SystemExit("no labels recorded yet")
    for line in LEDGER.read_text().splitlines():
        d = json.loads(line)
        print(f"  {d['stamp']}  {d['tag']:12s}  {np.round(d['ee_pos'], 4)}  {d.get('note','')}")
    raise SystemExit(0)

if not args.tag:
    parser.error("--tag is required (one tag per appliance position)")

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
state = manager.ArmInterface().get_state()
ee = [float(v) for v in list(state["ee_pos"])[:3]]
quat = [float(v) for v in list(state["ee_pos"])[3:7]]
joints = [float(v) for v in state["position"]]
gripper = float(state.get("gripper_pos"))

print(f"EE (label)  : {np.round(ee, 4)}")
print(f"gripper     : {gripper:.4f}")
if gripper > 0.2:
    print("NOTE: gripper is closed. Label the OPEN, centred touch pose for comparability.")

record = {
    "stamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
    "tag": args.tag,
    "ee_pos": ee,
    "ee_quat": quat,
    "joints": joints,
    "gripper": gripper,
    "note": args.note,
    "captures": [str(c) for c in args.captures],
}

# A big jump from the last label under the SAME tag means the appliance moved without
# the tag changing -- that silently corrupts a corpus, so say so loudly.
if LEDGER.exists():
    same_tag = [json.loads(l) for l in LEDGER.read_text().splitlines()
                if json.loads(l)["tag"] == args.tag]
    if same_tag:
        prev = np.asarray(same_tag[-1]["ee_pos"])
        delta = float(np.linalg.norm(np.asarray(ee) - prev))
        print(f"vs last '{args.tag}' label: {delta*100:.1f} cm")
        if delta > MOVED_WARN_M:
            print(f"  WARNING: >{MOVED_WARN_M*100:.0f} cm from the previous label with this tag.")
            print("  Either the appliance moved (use a NEW --tag) or the touch was not centred.")

LEDGER.parent.mkdir(parents=True, exist_ok=True)
with LEDGER.open("a") as f:
    f.write(json.dumps(record) + "\n")
print(f"appended to {LEDGER}")

for c in args.captures:
    if not c.exists():
        print(f"  SKIP {c}: does not exist")
        continue
    (c / "ground_truth.json").write_text(json.dumps(record, indent=2))
    print(f"  labelled {c}")
