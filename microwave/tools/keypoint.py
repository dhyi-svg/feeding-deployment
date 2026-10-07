"""Log ONE keypoint of a teleop demo (joints, tool pose, gripper) -- the arm node must be up.
Teleop and the sheppy arm node can't run together, so a demo is: teleop to a pose, bring the
node up, run this, release the node, repeat.

    python3 microwave/tools/keypoint.py --demo microwave/demos/push_open_2026-10-07.json --label "off the handle"
"""
import argparse, json, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # microwave/
import numpy as np

from sheppy_arm import SheppyArm

a = argparse.ArgumentParser()
a.add_argument("--demo", required=True)
a.add_argument("--label", required=True)
args = a.parse_args()
arm = SheppyArm()
try:
    st = arm.get_state()
finally:
    arm.close()
path = Path(args.demo)
demo = json.loads(path.read_text()) if path.exists() else {"keypoints": []}
kp = {"label": args.label, "time": time.strftime("%Y-%m-%d %H:%M:%S"),
      "joints_deg": np.round(np.degrees(st["position"]), 2).tolist(),
      "tool": np.round(st["ee_pos"][:3], 4).tolist(), "quat": np.round(st["ee_pos"][3:], 4).tolist(),
      "gripper": round(float(st["gripper_pos"]), 4)}
demo["keypoints"].append(kp)
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(demo, indent=1))
print(f"#{len(demo['keypoints']) - 1} {args.label}: tool {kp['tool']}  joints {kp['joints_deg']}  gripper {kp['gripper']}")
