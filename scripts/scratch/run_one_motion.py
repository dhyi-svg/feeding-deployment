"""Send a single, already-verified joint target in ONE set_joint_position call
(no intermediate checkpoints) -- for a continuous-looking motion on video.
Safety comes from the target having already been derived via a chain-seeded IK
path (see plan_hover_chain_v2.py) that was checked to stay under the joint-delta
guard at every intermediate point; sending only the endpoint trusts the arm's
own internal joint-space interpolation between here and there, which is the
same continuous branch throughout (not a fresh IK solve landing on a
different branch)."""
import json
import sys
import time

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)

PLAN = sys.argv[1]
ABORT_M = float(sys.argv[2]) if len(sys.argv) > 2 else 0.03

with open(PLAN) as f:
    plan = json.load(f)
assert plan["all_ok"], "plan was not all_ok in sim -- refusing to execute"
final_joints = plan["waypoints"][-1]
final_pos = plan["positions"][-1]

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
ai.set_speed("low")

before = ai.get_state()["ee_pos"]
print(f"before: ee_pos={np.round(before, 4)}")
print(f"commanding single joint target -> {np.round(final_joints, 4)} (expected ee {np.round(final_pos, 4)})")
ai.set_joint_position(final_joints)

time.sleep(0.5)
for _ in range(150):
    stt = ai.get_state()
    if max(abs(x) for x in stt["velocity"]) < 0.01:
        break
    time.sleep(0.1)

after = ai.get_state()
got = np.array(after["ee_pos"][:3])
err = float(np.linalg.norm(got - np.array(final_pos)))
print(f"after:  ee_pos={np.round(after['ee_pos'], 4)}")
print(f"tracking error vs planned target: {err*100:.2f}cm")
if err > ABORT_M:
    print(f"WARNING: err {err*100:.1f}cm > {ABORT_M*100:.0f}cm threshold")
else:
    print("within tolerance.")
