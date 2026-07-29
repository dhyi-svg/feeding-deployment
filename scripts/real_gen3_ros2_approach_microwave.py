"""Approach-only: live detect -> repo pre-grasp pose -> ONE joint-space move. No grasp.

Deliberately stops at the pre-grasp standoff. It never closes the gripper, never
touches the door, and never runs the opening arc (the perceived hinge is known to
be on the wrong side -- see TESTING_LOG 2026-07-29).

Default is a DRY RUN: it computes and gates everything, prints the target, and
exits without commanding the arm. Pass --execute to actually move.
"""
import argparse
import os
import sys
import time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.perception.grounded_sam import GroundedSAM
from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

TRUTH = np.array([0.6932, -0.1171, 0.5031])  # touched handle, for reporting only

# Safety gates.
MAX_REACH_M = 0.85          # Gen3 usable reach; beyond this the arm strains/aborts
MIN_Z, MAX_Z = 0.25, 0.75   # keep clear of the base and of the rig above
MAX_IK_ERR_M = 0.02         # reject an IK solution that misses the target
MAX_JOINT_JUMP_RAD = 1.4    # ~80 deg on any one joint => likely a wrist flip
MIN_STANDOFF_M = 0.08       # never command closer than this to the handle

ap_arg = argparse.ArgumentParser(description=__doc__)
ap_arg.add_argument("--execute", action="store_true", help="actually command the arm")
args = ap_arg.parse_args()

ai = ArmInterfaceClient()
state = ai.get_state()
start_joints = np.asarray(state["position"], dtype=float)
start_ee = np.asarray(list(state["ee_pos"])[:3], dtype=float)
gripper = float(state.get("gripper_pos"))
print(f"start EE      : [{start_ee[0]:.4f}, {start_ee[1]:.4f}, {start_ee[2]:.4f}]")
print(f"gripper       : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")
if gripper > 0.2:
    sys.exit("Gripper is not open -- refusing to approach while it may be holding something.")

# ---- live detection ---------------------------------------------------------
rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    sys.exit("No RGB-D frames")
gs = GroundedSAM()
apc = AppliancePerception(gs)
print(f"corrections   : depth={apc.handle_depth_corr} lat={apc.handle_lat_corr}")
if apc.handle_depth_corr == 0.0:
    sys.exit("Corrections are OFF -- set HANDLE_DEPTH_CORR before approaching.")

d = rs.get_camera_data()
handle, hinge, placement, top = apc.detect_handle_and_placement(
    "microwave handle", d["rgb_image"], d["camera_info"], d["depth_image"]
)
if handle is None:
    sys.exit("NO DETECTION")
h = np.asarray(handle.position)
print(f"handle        : [{h[0]:.4f}, {h[1]:.4f}, {h[2]:.4f}]  "
      f"(vs touched truth, {np.linalg.norm(h-TRUTH)*100:.1f} cm)")

# ---- pre-grasp standoff -----------------------------------------------------
# In arm_base_link: +x is out/forward (away from the arm), +z is up.
#
# detect_handle_and_placement stamps every pose with a FIXED
# GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5), which on this rig maps local +z to base
# -x -- i.e. the gripper faces BACKWARD, at the robot. With that orientation the
# repo's pre_grasp = handle @ trans(0,0,-0.12) lands BEHIND the microwave
# (range 0.96 m, past the Gen3's ~0.90 m reach and physically unreachable).
#
# The offset is not what is wrong -- the orientation is. Rotating 180 deg about
# local y turns the gripper to face +x (forward, at the appliance); the repo's
# own -0.12 offset then puts the standoff in FRONT of the handle. Verified by
# IK sweep 2026-07-29: this is the only candidate that solves exactly
# (0.00 cm error) and it needs the least joint motion (43 deg vs 169 deg).
PRE_GRASP_STANDOFF_M = 0.12
GRASP_QUAT_FIX = R.from_euler("y", np.pi)  # rig-specific: face the gripper forward

handle_quat = (R.from_quat(np.asarray(handle.orientation)) * GRASP_QUAT_FIX).as_quat()
handle_fixed = Pose(tuple(h), tuple(handle_quat))
print(f"gripper +z -> base {np.round(R.from_quat(handle_quat).as_matrix()[:, 2], 2)} (want +x)")

helper = PerceptionInterface.__new__(PerceptionInterface)
offset = np.eye(4)
offset[:3, 3] = np.array([0.0, 0.0, -PRE_GRASP_STANDOFF_M])
pre_grasp = helper.matrix_to_pose(helper.pose_to_matrix(handle_fixed) @ offset)
tgt = np.asarray(pre_grasp.position)
standoff = float(np.linalg.norm(tgt - h))
print(f"pre-grasp     : [{tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}]")
print(f"standoff from handle : {standoff*100:.1f} cm")
print(f"move distance from current EE : {np.linalg.norm(tgt-start_ee)*100:.1f} cm")

# ---- safety gates -----------------------------------------------------------
fail = []
if standoff < MIN_STANDOFF_M:
    fail.append(f"standoff {standoff:.3f} m < {MIN_STANDOFF_M} m")
if np.linalg.norm(tgt) > MAX_REACH_M:
    fail.append(f"target range {np.linalg.norm(tgt):.3f} m > {MAX_REACH_M} m")
if not (MIN_Z <= tgt[2] <= MAX_Z):
    fail.append(f"target z {tgt[2]:.3f} outside [{MIN_Z}, {MAX_Z}]")
# +x is out/forward, so a standoff must be NEARER the base than the handle.
# A standoff further out is behind the appliance: unreachable and physically
# impossible. This catches an inverted offset sign directly.
if np.linalg.norm(tgt) >= np.linalg.norm(h):
    fail.append(
        f"standoff range {np.linalg.norm(tgt):.3f} m >= handle range "
        f"{np.linalg.norm(h):.3f} m -- standoff is BEHIND the handle (offset sign inverted?)"
    )
if fail:
    sys.exit("SAFETY GATE FAILED: " + "; ".join(fail))
print("safety gates  : PASS (standoff, reach, height)")

# ---- seeded IK --------------------------------------------------------------
# Seed PyBullet from the arm's ACTUAL joints so IK returns a nearby solution --
# unseeded IK caused a wrist flip on large moves (TESTING_LOG 2026-07-14).
scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
rb = sim.robot
for i, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
    p.resetJointState(rb.robot_id, jj, float(start_joints[i]),
                      physicsClientId=rb.physics_client_id)

# The sim scene places the robot at a world offset (it sits on the vention
# base), so an arm_base_link target must be lifted into sim world coordinates
# before IK -- otherwise IK chases a point metres away. Same step the proven
# standalone script does via scene_description.robot_base_pose.
wpose = multiply_poses(scene.robot_base_pose, pre_grasp)
sol = p.calculateInverseKinematics(
    rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
    maxNumIterations=400, residualThreshold=1e-5,
    physicsClientId=rb.physics_client_id)
q = np.asarray(sol[:7], dtype=float)

for i, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
    p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
# Compare in world frame, where the IK target lives.
ik_err = float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(wpose.position)))
jump = float(np.max(np.abs(q - start_joints)))
print(f"IK error      : {ik_err*100:.2f} cm")
print(f"max joint jump: {np.degrees(jump):.1f} deg")
print("joint target  :", [round(float(np.degrees(v)), 1) for v in q])

if ik_err > MAX_IK_ERR_M:
    sys.exit(f"IK MISSED by {ik_err*100:.1f} cm (> {MAX_IK_ERR_M*100:.0f} cm) -- refusing to move.")
if jump > MAX_JOINT_JUMP_RAD:
    sys.exit(f"Joint jump {np.degrees(jump):.0f} deg too large -- likely a wrist flip. Refusing.")
print("IK gates      : PASS")

if not args.execute:
    print("\nDRY RUN -- nothing commanded. Re-run with --execute to move.")
    sys.exit(0)

# ---- execute ----------------------------------------------------------------
print(f"\nspeed: {ai.get_speed()}  -- commanding ONE joint move to pre-grasp ...")
ai.execute_command(JointCommand(pos=q.tolist()))

# A move can return before it settles -- wait for velocity ~0, then re-check.
for _ in range(100):
    time.sleep(0.2)
    st = ai.get_state()
    if float(np.max(np.abs(np.asarray(st["velocity"], dtype=float)))) < 1e-3:
        break
time.sleep(0.5)

st = ai.get_state()
final_ee = np.asarray(list(st["ee_pos"])[:3], dtype=float)
err = final_ee - tgt
print(f"\nfinal EE      : [{final_ee[0]:.4f}, {final_ee[1]:.4f}, {final_ee[2]:.4f}]")
print(f"target        : [{tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}]")
print(f"tracking error: [{err[0]:+.4f}, {err[1]:+.4f}, {err[2]:+.4f}]  |e| = {np.linalg.norm(err)*100:.1f} cm")
print(f"gripper       : {float(st.get('gripper_pos')):.4f}  (untouched -- no grasp)")
print(f"distance to handle now: {np.linalg.norm(final_ee-h)*100:.1f} cm")
