"""Append ONE waypoint -- the arm's pose right now -- to a JSON file.

READ-ONLY with respect to the arm: only calls ``get_state()``, which (unlike
every motion method) does not go through ``_require_bulldog()``, so it is safe
to run while a person has their hands on the robot.

This is the operator-triggered counterpart to record_manual_waypoints.py. That
script infers waypoints from pauses, which over-captures when the operator is
still repositioning (2026-09-20: 29 unwanted captures in one run). Here the
operator decides: hand-move the arm, say so, and this runs once.

Re-reads the JSON each call and appends, so the file is the single source of
truth and nothing is held in memory between captures -- any call can fail or be
skipped without corrupting earlier points.

    ARM_RPC_HOST=127.0.0.1 python3 scripts/snapshot_waypoint.py
    ARM_RPC_HOST=127.0.0.1 python3 scripts/snapshot_waypoint.py --label grasp
"""
import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient

ap = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--out", default="~/manual_waypoints.json")
ap.add_argument("--label", default=None, help="optional name for this waypoint")
ap.add_argument("--settle-s", type=float, default=0.4,
                help="sample over this long and report jitter, to catch a capture "
                     "taken while the arm was still being moved (default 0.4 s)")
ap.add_argument("--rate", type=float, default=20.0)
args = ap.parse_args()

out_path = Path(os.path.expanduser(args.out))

ai = ArmInterfaceClient()

# Sample briefly rather than taking a single instant, so the printed jitter tells
# the operator whether the arm was actually settled at capture time.
samples = []
n = max(2, int(args.settle_s * args.rate))
for _ in range(n):
    samples.append(np.asarray(ai.get_state()["position"], dtype=float))
    time.sleep(1.0 / args.rate)
jitter_deg = float(np.rad2deg(np.max(np.ptp(np.asarray(samples), axis=0))))

st = ai.get_state()
q = np.asarray(st["position"], dtype=float)
ee = np.asarray(st["ee_pos"], dtype=float)

doc = {"joint_order": "kinova gen3 7dof, radians, base->wrist",
       "ee_pos_format": "x,y,z,qx,qy,qz,qw in arm_base_link",
       "waypoints": []}
if out_path.exists():
    with open(out_path) as f:
        doc = json.load(f)

wp = {
    "index": len(doc["waypoints"]),
    "label": args.label,
    "t_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    "joints_rad": [float(v) for v in q],
    "joints_deg": [float(v) for v in np.rad2deg(q)],
    "ee_xyz": [float(v) for v in ee[:3]],
    "ee_quat_xyzw": [float(v) for v in ee[3:7]],
    "gripper_pos": float(st["gripper_pos"]),
    "jitter_deg": jitter_deg,
}
doc["waypoints"].append(wp)
doc["updated_utc"] = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

out_path.parent.mkdir(parents=True, exist_ok=True)
with open(out_path, "w") as f:
    json.dump(doc, f, indent=2)

name = wp["label"] or f"wp{wp['index']}"
print(f"\nCAPTURED {name}  (waypoint {wp['index']}, total {len(doc['waypoints'])})")
print(f"  joints_deg : {[round(float(v), 2) for v in np.rad2deg(q)]}")
print(f"  ee_xyz     : [{ee[0]:+.4f}, {ee[1]:+.4f}, {ee[2]:+.4f}]")
print(f"  ee_quat    : [{ee[3]:+.4f}, {ee[4]:+.4f}, {ee[5]:+.4f}, {ee[6]:+.4f}]")
print(f"  gripper    : {wp['gripper_pos']:.4f} ({'open' if wp['gripper_pos'] < 0.2 else 'CLOSED'})")
print(f"  jitter     : {jitter_deg:.3f} deg over {args.settle_s:g}s"
      f"{'  <-- STILL MOVING?' if jitter_deg > 0.5 else ''}")
print(f"  -> {out_path}")
