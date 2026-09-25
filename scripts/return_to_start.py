"""Drive the arm back to its start pose along the SAME path it came out on.

    ARM_RPC_HOST=127.0.0.1 python3 scripts/return_to_start.py              # dry run
    ARM_RPC_HOST=127.0.0.1 python3 scripts/return_to_start.py --execute
    ARM_RPC_HOST=127.0.0.1 python3 scripts/return_to_start.py --execute --smooth

Why replay rather than just interpolate home: a fresh joint-space move between
two individually-clear endpoints can still sweep a link through the open fridge
door (the repo has a documented incident of exactly that). Every point in
/tmp/fridge_path.json was already traversed by this arm minutes earlier, so
running it backwards is provably clear of whatever the room contained then --
provided nothing has moved and the door is still where the swing left it.

Reads the path the grasp+swing script recorded, reverses it, and replays it as
Cartesian waypoints; then a final joint-space move to START_JOINTS_RAD (the
recorded start pose), which is safe because by then the arm is back at the
pre-grasp standoff, clear of the fridge.

REFUSES to run while the gripper is closed -- retracing the path while still
holding the door would drag it. Release first (scripts/release_fridge.py).
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from real_gen3_ros2_sam3_grasp_fridge import (  # noqa: E402
    ARM, MAX_REACH, MIN_Z, MAX_Z, PATH_LOG, SMOOTH_SINGULARITY_PROBE_M,
    START_JOINTS_RAD, _move_joints_checked)

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient  # noqa: E402
from feeding_deployment.control.robot_controller.command_interface import (  # noqa: E402
    CartesianTrajectoryCommand, JointCommand)
from feeding_deployment.simulation.scene_description import (  # noqa: E402
    create_scene_description_from_config)
from feeding_deployment.simulation.simulator import (  # noqa: E402
    FeedingDeploymentPyBulletSimulator)

MAX_IK_ERR = 0.02
MAX_JUMP_DEG = 45.0        # per replayed point; the path itself was small-stepped
FINAL_STEPS = 6            # sub-steps for the last joint move home

ap = argparse.ArgumentParser(description=__doc__,
                             formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--execute", action="store_true", help="actually move; omit to plan only")
ap.add_argument("--smooth", action="store_true",
                help="replay as one blended Cartesian trajectory instead of point-by-point")
ap.add_argument("--path", default=str(PATH_LOG), help=f"path log to replay (default {PATH_LOG})")
ap.add_argument("--skip-home", action="store_true",
                help="stop after retracing; do not make the final joint move to the start pose")
args = ap.parse_args()

ai = ArmInterfaceClient()
st = ai.get_state()
g = float(st["gripper_pos"])
ee = np.asarray(st["ee_pos"][:3], dtype=float)
q_now = np.asarray(st["position"], dtype=float)
print(f"EE {np.round(ee, 4)}  gripper {g:.4f}")
if g > 0.2:
    sys.exit("Gripper is CLOSED -- release first (scripts/release_fridge.py), "
             "retracing while holding the door would drag it.")

path_file = Path(os.path.expanduser(args.path))
if not path_file.exists():
    sys.exit(f"No path log at {path_file} -- nothing to retrace. (It is written by "
             "real_gen3_ros2_sam3_grasp_fridge.py when it moves.)")
doc = json.loads(path_file.read_text())

pts = []
for ph in doc["phases"]:
    for pos, quat in ph["points"]:
        pts.append((np.asarray(pos, dtype=float), np.asarray(quat, dtype=float)))
if not pts:
    sys.exit("Path log is empty.")
rev = list(reversed(pts))
print(f"path log: {len(pts)} points over {len(doc['phases'])} phase(s) "
      f"({', '.join(ph['phase'] for ph in doc['phases'])}) -> replaying REVERSED")

# The arm should currently be at (or very near) the END of the recorded path.
drift = float(np.linalg.norm(ee - rev[0][0]))
print(f"current EE is {drift*100:.1f} cm from the recorded end of the path")
if drift > 0.15:
    sys.exit(f"Arm is {drift*100:.0f} cm from where the path ended -- it has been moved since. "
             "Retracing from here is not the path it actually took; refusing.")

scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
rb = sim.robot

def solve(pos, quat, seed):
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed[i]), physicsClientId=rb.physics_client_id)
    w = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
                                       list(w.position), list(w.orientation),
                                       maxNumIterations=200,
                                       physicsClientId=rb.physics_client_id)
    q = np.asarray(sol[:7])
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
    err = float(np.linalg.norm(np.asarray(rb.get_end_effector_pose().position) - np.asarray(w.position)))
    return q, err

print("\nchecking the reversed path in sim:")
seed, qs, worst_ik, worst_jump = q_now.copy(), [], 0.0, 0.0
for i, (pos, quat) in enumerate(rev):
    q, err = solve(pos, quat, seed)
    jump = float(np.degrees(np.max(np.abs((q - seed + np.pi) % (2 * np.pi) - np.pi))))
    bad = []
    if err > MAX_IK_ERR: bad.append(f"ik_err {err*100:.1f}cm")
    if np.linalg.norm(pos) > MAX_REACH: bad.append(f"reach {np.linalg.norm(pos):.3f}")
    if not (MIN_Z <= pos[2] <= MAX_Z): bad.append(f"z {pos[2]:.3f}")
    if jump > MAX_JUMP_DEG: bad.append(f"jump {jump:.0f}deg")
    worst_ik, worst_jump = max(worst_ik, err), max(worst_jump, jump)
    if bad:
        sys.exit(f"point {i+1}/{len(rev)} {np.round(pos,3)} FAILS: {'; '.join(bad)} -- refusing.")
    qs.append(q); seed = q
print(f"  all {len(rev)} points OK   worst ik_err {worst_ik*100:.2f} cm   worst jump {worst_jump:.1f} deg")

q_home = np.asarray(START_JOINTS_RAD, dtype=float)
home_jump = float(np.degrees(np.max(np.abs((q_home - seed + np.pi) % (2 * np.pi) - np.pi))))
print(f"final joint move to the start pose: {home_jump:.1f} deg max"
      f"{'  (skipped)' if args.skip_home else f', in {FINAL_STEPS} sub-steps'}")

if not args.execute:
    sys.exit("\nDRY RUN -- nothing commanded.")

if args.smooth:
    ok = ai.execute_command(CartesianTrajectoryCommand([(pos, quat) for pos, quat in rev]))
    print(f"smooth retrace {'DONE' if ok else 'RETURNED FALSE'}")
else:
    for i, q in enumerate(qs):
        _move_joints_checked(ai, q, f"retrace {i+1}/{len(qs)}")
    print("retrace done")

if not args.skip_home:
    cur = np.asarray(ai.get_state()["position"], dtype=float)
    total = (q_home - cur + np.pi) % (2 * np.pi) - np.pi
    for k in range(1, FINAL_STEPS + 1):
        _move_joints_checked(ai, cur + total * (k / FINAL_STEPS), f"home {k}/{FINAL_STEPS}")

fin = ai.get_state()
print(f"\nBACK AT START. EE {np.round(np.asarray(fin['ee_pos'][:3]), 4)}  "
      f"gripper {float(fin['gripper_pos']):.4f}")
