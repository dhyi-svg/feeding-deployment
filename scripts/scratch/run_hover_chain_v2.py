import json
import time

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)

PLAN = "/tmp/hover_chain_plan_v2.json"
ABORT_M = 0.03

with open(PLAN) as f:
    plan = json.load(f)

waypoints = plan["waypoints"]
positions = plan["positions"]
assert plan["all_ok"], "plan was not all_ok in sim -- refusing to execute"

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
ai.set_speed("low")

print(f"Executing {len(waypoints)}-step hover chain, low speed, abort_m={ABORT_M}")
for i, (joints, pos) in enumerate(zip(waypoints, positions), 1):
    print(f"step {i}/{len(waypoints)} -> target ee {np.round(pos, 4)}")
    ai.set_joint_position(joints)
    time.sleep(0.3)
    for _ in range(80):
        stt = ai.get_state()
        if (np.linalg.norm(np.array(stt["ee_pos"][:3]) - np.array(pos)) < 0.02
                and max(abs(x) for x in stt["velocity"]) < 0.01):
            break
        time.sleep(0.15)
    got = np.array(ai.get_state()["ee_pos"][:3])
    err = float(np.linalg.norm(got - np.array(pos)))
    print(f"  reached {np.round(got, 4)} | tracking err {err*100:.1f}cm")
    if err > ABORT_M:
        print(f"  ABORT: err {err*100:.1f}cm > {ABORT_M*100:.0f}cm -- stopping, not advancing further.")
        raise SystemExit(1)

final = ai.get_state()
print("\nHover chain complete.")
print(f"final ee_pos: {np.round(final['ee_pos'], 4)}")
print(f"final joints: {np.round(final['position'], 4)}")
