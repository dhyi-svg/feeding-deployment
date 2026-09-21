"""Release the fridge handle: open the gripper, then (optionally) back off.

    ARM_RPC_HOST=127.0.0.1 python3 scripts/release_fridge.py                 # open gripper only
    ARM_RPC_HOST=127.0.0.1 python3 scripts/release_fridge.py --retract 8     # + plan an 8 cm back-off (dry run)
    ARM_RPC_HOST=127.0.0.1 python3 scripts/release_fridge.py --retract 8 --execute

Opening the gripper happens on every call -- that is what the script is for.
The back-off is real arm motion, so it follows the repo's rule: planned and
gated by default, moved only with --execute.

Back-off direction is the gripper's OWN approach axis, reversed (local -z),
taken from the arm's current orientation. That is right at any door angle:
after a 90 deg swing the gripper has yawed with the door, so "straight back
out of the handle" is no longer base -x. Same off() construction as the grasp
script. Gates: IK error, reach, joint jump, tracking abort -- same values.

Does NOT touch /tmp/fridge_door_hinge.json: the door is wherever it is, and a
later re-grasp at the closed door resets that file anyway.
"""
import argparse
import sys
import time

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses
from scipy.spatial.transform import Rotation as R

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    JointCommand, OpenGripperCommand)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

MAX_REACH, MIN_Z, MAX_Z = 0.91, 0.25, 0.75
MAX_IK_ERR, MAX_JUMP_DEG, TRACK_ABORT = 0.02, 45.0, 0.03
ARM = [1, 2, 3, 4, 5, 6, 7]

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--retract", type=float, default=0.0, metavar="CM",
                help="after opening, back the gripper straight out of the handle by this many cm")
ap.add_argument("--execute", action="store_true", help="actually perform the back-off move")
ap.add_argument("--steps", type=int, default=4, help="joint sub-steps for the back-off (default 4)")
args = ap.parse_args()

ai = ArmInterfaceClient()
st = ai.get_state()
g0 = float(st["gripper_pos"])
ee = np.asarray(st["ee_pos"], dtype=float)
print(f"EE {np.round(ee[:3], 4)}  gripper {g0:.4f} ({'CLOSED' if g0 > 0.2 else 'already open'})")

if g0 > 0.2:
    print("opening gripper ...")
    ai.execute_command(OpenGripperCommand())
    time.sleep(2.5)
    g1 = float(ai.get_state()["gripper_pos"])
    print(f"gripper after open: {g1:.4f}")
    if g1 > 0.2:
        sys.exit("Gripper did not open -- not moving the arm with it possibly still on the handle.")
else:
    print("gripper already open; nothing to release.")

if args.retract <= 0:
    sys.exit(0)

# ---- back-off: current pose shifted along its own -z by --retract ------------
def _pose_to_matrix(pose):
    m = np.eye(4)
    m[:3, 3] = pose[0]
    m[:3, :3] = R.from_quat(pose[1]).as_matrix()
    return m

cur = Pose(tuple(ee[:3]), tuple(ee[3:7]))
off = np.eye(4); off[:3, 3] = [0.0, 0.0, -args.retract / 100.0]
m = _pose_to_matrix((cur.position, cur.orientation)) @ off
tgt = Pose(m[:3, 3], R.from_matrix(m[:3, :3]).as_quat())
t = np.asarray(tgt.position)
print(f"\nback-off target {np.round(t, 4)}  ({args.retract:.0f} cm along gripper -z)  range {np.linalg.norm(t):.3f}")

bad = []
if np.linalg.norm(t) > MAX_REACH: bad.append(f"range {np.linalg.norm(t):.3f}>{MAX_REACH}")
if not (MIN_Z <= t[2] <= MAX_Z): bad.append(f"z {t[2]:.3f} out of range")

scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False); rb = sim.robot
q0 = np.asarray(st["position"], dtype=float)
for i, jj in enumerate(ARM):
    p.resetJointState(rb.robot_id, jj, float(q0[i]), physicsClientId=rb.physics_client_id)
w = multiply_poses(scene.robot_base_pose, tgt)
sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, list(w.position), list(w.orientation),
                                   maxNumIterations=400, residualThreshold=1e-5,
                                   physicsClientId=rb.physics_client_id)
q = np.asarray(sol[:7])
for i, jj in enumerate(ARM):
    p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
err = float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))
jump = float(np.degrees(np.max(np.abs((q - q0 + np.pi) % (2 * np.pi) - np.pi))))
if err > MAX_IK_ERR: bad.append(f"IK err {err*100:.1f}cm")
if jump > MAX_JUMP_DEG: bad.append(f"jump {jump:.0f}deg")
print(f"IK {err*100:.2f} cm  jump {jump:.1f} deg  {'FAIL: ' + '; '.join(bad) if bad else 'OK'}")
if bad:
    sys.exit("GATE FAILED -- gripper is open but the arm stays put.")

if not args.execute:
    sys.exit("\nDRY RUN -- gripper opened, back-off NOT commanded. Add --execute to move.")

total = (q - q0 + np.pi) % (2 * np.pi) - np.pi
prev = q0
for s_ in range(1, args.steps + 1):
    qs = q0 + total * (s_ / args.steps)
    ai.execute_command(JointCommand(pos=qs.tolist()))
    # Wait for CONVERGENCE, not just velocity ~ 0: Kortex can drop a sub-step
    # (ROBOT_MOVEMENT_IN_PROGRESS) when the previous one's END arrives early,
    # and a dropped step is smaller than the 5 deg check inside move_angular.
    # See _wait_converged in real_gen3_ros2_sam3_grasp_fridge.py.
    for attempt in range(2):
        derr, deadline = float("inf"), time.time() + 6.0
        while time.time() < deadline:
            time.sleep(0.12)
            st_ = ai.get_state()
            qa = np.asarray(st_["position"], dtype=float)
            derr = float(np.degrees(np.max(np.abs((qa - qs + np.pi) % (2 * np.pi) - np.pi))))
            if derr < 1.0 and float(np.max(np.abs(np.asarray(st_["velocity"], dtype=float)))) < 1e-3:
                break
        if derr < 1.0:
            break
        print(f"  step {s_}: settled {derr:.1f} deg short -- re-sending once")
        time.sleep(0.5)
        ai.execute_command(JointCommand(pos=qs.tolist()))
    if derr >= 1.0:
        sys.exit(f"step {s_}: still {derr:.1f} deg off after retry -- ABORT.")
    prev = qs
fin = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
e = float(np.linalg.norm(fin - t))
print(f"back-off done. EE {np.round(fin, 4)}  tracking {e*100:.1f} cm{'  ABORT-LEVEL' if e > TRACK_ABORT else ''}")
