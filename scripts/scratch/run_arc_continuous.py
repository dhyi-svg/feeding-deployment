"""Execute all planned arc waypoints back-to-back (no per-waypoint user
confirmation) for a continuous-looking swing on video. Each waypoint was
pre-verified in sim (joint-delta guard, ik_err, reach guard) by
plan_arc_continuous.py -- this script re-checks real tracking error after each
step and aborts immediately (not commanding further waypoints) if any step's
real-world error is too large, same safety property as the original
one-step-per-invocation pattern, just run in one process without stopping for
confirmation between steps."""
import json
import sys
import time

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)

PLAN = sys.argv[1]
ABORT_M = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02

with open(PLAN) as f:
    plan = json.load(f)
assert plan["all_ok"], "plan was not all_ok in sim -- refusing to execute"
waypoints = plan["waypoints"]

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
ai.set_speed("low")

print(f"Executing {len(waypoints)}-waypoint arc continuously, target angle {plan['target_angle_deg']}deg")
for i, wp in enumerate(waypoints, 1):
    pos, joints = wp["pos"], wp["joints"]
    print(f"waypoint {i}/{len(waypoints)} -> {np.round(pos, 4)}")
    ai.set_joint_position(joints)
    time.sleep(0.3)
    for _ in range(60):
        stt = ai.get_state()
        if (np.linalg.norm(np.array(stt["ee_pos"][:3]) - np.array(pos)) < 0.02
                and max(abs(x) for x in stt["velocity"]) < 0.01):
            break
        time.sleep(0.15)
    got = np.array(ai.get_state()["ee_pos"][:3])
    err = float(np.linalg.norm(got - np.array(pos)))
    print(f"  reached {np.round(got, 4)} | tracking err {err*100:.1f}cm")
    if err > ABORT_M:
        print(f"  ABORT at waypoint {i}: err {err*100:.1f}cm > {ABORT_M*100:.0f}cm -- "
              "door binding / latch / unexpected contact. Not advancing further.")
        raise SystemExit(1)

final = ai.get_state()
print("\nArc swing complete (all waypoints reached).")
print(f"final ee_pos: {np.round(final['ee_pos'], 4)}")
