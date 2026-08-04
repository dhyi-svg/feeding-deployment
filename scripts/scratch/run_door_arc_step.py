"""Door-opening arc, ONE waypoint per call -- same proven pattern as the Jetson's
scripts/real_gen3_open_microwave.py (state persisted, unseeded per-waypoint IK,
per-step abort on tracking error / reach limit), but sized as a SMALL TEST SWING
(~25deg) for this session rather than a full open, since the detected
handle-to-hinge radius here (~12cm) is much smaller than the Jetson rig's assumed
32cm, and reusing the Jetson's arc_length_m=0.55 constant at this small radius
would demand ~262deg of rotation (immediately out of reach). Re-run this script
to advance one waypoint at a time; state lives in /tmp/door_arc_pachirisu.json.
Delete that file to re-plan from the arm's current pose.
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator
from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY,
)

HANDLE_POS = np.array([0.84064532, -0.18060141, 0.47008189])
HINGE_POS = np.array([0.86834637, -0.2974223, 0.46546328])
HINGE_OFFSET = HINGE_POS - HANDLE_POS  # transferred onto the current grasp point below

TARGET_ANGLE_DEG = 25.0
WAYPOINT_SPACING_M = 0.02
ABORT = 0.02
S = Path("/tmp/door_arc_pachirisu.json")

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
ai.set_speed("low")

if not S.exists():
    ee = list(ai.get_state()["ee_pos"])
    grasp = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
    hinge = tuple(np.array(ee[:3]) + HINGE_OFFSET)
    radius = float(np.linalg.norm(np.array(ee[:3]) - np.array(hinge)))
    arc_length_m = radius * np.radians(TARGET_ANGLE_DEG)
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=grasp, hinge_position=hinge,
        arc_length_m=arc_length_m, waypoint_spacing_m=WAYPOINT_SPACING_M,
        direction=1, rotate_orientation=True)  # +1: pulls the handle toward the arm
        # (repo's own "microwave" default is -1, tuned for the lab rig's mirrored
        # layout -- verified in chat that -1 here swings AWAY from the base and
        # immediately exceeds the 90cm reach guard; +1 matches this rig's actual
        # geometry and the physical reality of pulling a door open toward you)
    wps = [list(w.position) + list(w.orientation) for w in wps_pose]
    S.write_text(json.dumps({"wps": wps, "i": 0, "hinge": list(hinge), "radius": radius}))
    print(f"grasp ee_pos: {np.round(ee[:3], 4)}")
    print(f"hinge estimate: {np.round(hinge, 4)} (radius {radius*100:.1f}cm)")
    print(f"{len(wps)} waypoints, target {TARGET_ANGLE_DEG}deg / arc_length {arc_length_m*100:.1f}cm")

st = json.loads(S.read_text())
i, wps = st["i"], st["wps"]
if i >= len(wps):
    print("test swing COMPLETE.")
    sys.exit(0)
wp = wps[i]
pos, quat = wp[:3], wp[3:]

# SEED from the arm's actual current real joints -- NOT a fresh/default sim init.
# A prior run of this script (unseeded, matching the Jetson original) found a
# Cartesian-correct but joint-space-distant solution (elbow/wrist flipped to a
# different branch), causing a ~150deg uncontrolled swing on the real arm even
# though the target was only ~1.7cm further along the arc. Seeding + checking
# the joint-space delta (not just Cartesian ik_err) is the fix.
real_joints = list(ai.get_state()["position"])
MAX_JOINT_DELTA_DEG = 25.0

sd = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
rb = sim.robot
n_finger = len(rb.get_joint_positions()) - 7
for j, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
    p.resetJointState(rb.robot_id, jj, real_joints[j], physicsClientId=rb.physics_client_id)
wpose = multiply_poses(sd.robot_base_pose, Pose(tuple(pos), tuple(quat)))
sol = p.calculateInverseKinematics(
    rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
    physicsClientId=rb.physics_client_id, maxNumIterations=200)
joints = [sol[k] for k in range(7)]
for j, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
    p.resetJointState(rb.robot_id, jj, joints[j], physicsClientId=rb.physics_client_id)
ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
joint_delta_deg = np.degrees(np.abs(np.array(joints) - np.array(real_joints)))
max_delta = float(np.max(joint_delta_deg))
d = float(np.linalg.norm(pos))
print(f"step {i + 1}/{len(wps)} -> target {np.round(pos, 3)} ({d * 100:.0f}cm from base, "
      f"sim ik_err {ikerr * 100:.2f}cm, max joint delta {max_delta:.1f}deg)")
if ikerr > 0.02 or d > 0.90:
    print("  target unreachable / past limit -- stopping.")
    sys.exit(1)
if max_delta > MAX_JOINT_DELTA_DEG:
    print(f"  ABORT: max joint delta {max_delta:.1f}deg > {MAX_JOINT_DELTA_DEG}deg guard -- "
          "looks like another far-branch IK solution, not a continuous step. Not commanding.")
    sys.exit(1)

ai.set_joint_position(joints)
time.sleep(0.5)
for _ in range(60):
    stt = ai.get_state()
    if (np.linalg.norm(np.array(stt["ee_pos"][:3]) - np.array(pos)) < 0.02
            and max(abs(x) for x in stt["velocity"]) < 0.01):
        break
    time.sleep(0.2)
got = np.array(ai.get_state()["ee_pos"][:3])
err = np.linalg.norm(got - np.array(pos))
print(f"  reached {np.round(got, 3)} | tracking err {err * 100:.1f}cm")
if err > ABORT:
    print(f"  ABORT: err {err * 100:.1f}cm > {ABORT * 100:.0f}cm -- door binding / latch / limit. Not advancing.")
    sys.exit(1)

st["i"] = i + 1
S.write_text(json.dumps(st))
print(f"  step {i + 1} done. {len(wps) - st['i']} steps left. Re-run to continue.")
