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
MAX_STEP_TRACK_M = 0.03     # with --steps: abort if a sub-step lands >3 cm off plan

ap_arg = argparse.ArgumentParser(description=__doc__)
ap_arg.add_argument("--execute", action="store_true", help="actually command the arm")
ap_arg.add_argument(
    "--steps", type=int, default=1,
    help="split the move into N interpolated sub-steps (default 1 = single jump, "
         "the historical behaviour). 12-16 is the proven range. See --max-step-deg.")
ap_arg.add_argument(
    "--max-step-deg", type=float, default=20.0,
    help="with --steps, abort if any ONE sub-step moves a joint more than this (default 20)")
ap_arg.add_argument(
    "--interp", choices=("joint", "cartesian"), default="joint",
    help="how --steps splits the move. joint (default): interpolate the joint vector "
         "to the gated solution -- per-step motion bounded by construction, ends in "
         "the validated posture. cartesian: straight-line gripper path, but drifts "
         "through the null space and can end in a very different posture.")
args = ap_arg.parse_args()
if args.steps < 1:
    sys.exit("--steps must be >= 1")

ai = ArmInterfaceClient()
state = ai.get_state()
start_joints = np.asarray(state["position"], dtype=float)
start_ee = np.asarray(list(state["ee_pos"])[:3], dtype=float)
start_quat = np.asarray(list(state["ee_pos"])[3:7], dtype=float)  # for Slerp when chaining
gripper = float(state.get("gripper_pos"))
print(f"start EE      : [{start_ee[0]:.4f}, {start_ee[1]:.4f}, {start_ee[2]:.4f}]")
# Record the pose we are LEAVING, not just the one we are going to. Without this the
# only way back to a good viewing pose after a run is to reconstruct it from
# arm_commands_log.txt (doable, but that file is wiped on every arm_server restart).
print(f"start joints  : {[round(float(v), 6) for v in start_joints]}")
print(f"gripper       : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")
if gripper > 0.2:
    sys.exit("Gripper is not open -- refusing to approach while it may be holding something.")

# ---- live detection ---------------------------------------------------------
rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    sys.exit("No RGB-D frames")
gs = GroundedSAM()
# Detection diagnostics. AppliancePerception already builds every overlay (plane fit,
# candidate cluster, final handle/hinge pixels) and already writes a JSON sidecar with
# the intrinsics and the 4x4 base<-camera matrix -- but both hooks no-op when
# data_logger is None, so all of it is discarded at exit. Set DETECTION_LOG_DIR and it
# lands on disk, which is also what makes scripts/session/replay_detection.py possible.
_log_dir = os.environ.get("DETECTION_LOG_DIR")
_data_logger = None
if _log_dir:
    from pathlib import Path

    from feeding_deployment.integration.data_logger import DataLogger

    _data_logger = DataLogger(Path(_log_dir), day=1)
    _data_logger.begin_hla("approach_microwave")
    print(f"detection logging -> {_log_dir}")
apc = AppliancePerception(gs, data_logger=_data_logger)
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

# ---- optional chaining ------------------------------------------------------
# A single command to `q` is one big move: the controller interpolates in JOINT
# space, so the EE traces an unchecked curve and nothing here collision-checks it.
# --steps splits it so each increment is small and abortable. Two ways to split:
#
#   joint (default) -- interpolate the JOINT vector from start to `q`, the exact
#     solution the gates above validated. Per-step joint motion is bounded by
#     construction (total/N), and the arm ENDS in the gated posture. The EE follows
#     the same curve the unchained move would have taken -- no new path, just cut
#     into abortable pieces.
#
#   cartesian -- interpolate the EE along a straight line (Slerp for orientation)
#     and re-solve IK per sub-step. The gripper path is predictable, but each solve
#     drifts through the 7-DOF null space: measured 2026-08-05, a 12-step chain hit
#     the target pose to 0.001 cm while ending 83.8 deg away from `q` in JOINT space
#     (J1 -4.5 -> -88.2). Same gripper pose, very different arm posture, and the
#     downstream grasp/arc scripts seed their IK from wherever this leaves the arm.
#     Use it only when the straight-line gripper path itself matters.
#
# Sub-stepping alone does NOT bound per-step joint motion in cartesian mode -- the
# explicit --max-step-deg check below does, applied to every step in sim BEFORE
# anything is commanded, failing closed on the whole move.
plan = [("pre-grasp", tgt, q)]  # --steps 1: the single jump, unchanged


def _fk(joints):
    """EE position in arm_base_link for a joint vector."""
    for i, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
        p.resetJointState(rb.robot_id, jj, float(joints[i]),
                          physicsClientId=rb.physics_client_id)
    link = p.getLinkState(rb.robot_id, rb.end_effector_id,
                          physicsClientId=rb.physics_client_id)
    return np.asarray(link[4]) - np.asarray(scene.robot_base_pose.position)


if args.steps > 1:
    plan, seed, worst = [], start_joints, 0.0
    print(f"\nchaining into {args.steps} {args.interp} sub-steps "
          f"(guard {args.max_step_deg} deg/step):")

    if args.interp == "cartesian":
        from scipy.spatial.transform import Slerp
        slerp = Slerp([0.0, 1.0],
                      R.from_quat([start_quat, np.asarray(pre_grasp.orientation)]))
    else:
        # Shortest path per joint: a raw difference can read ~350 deg for what is
        # physically a few degrees the other way round.
        total_delta = (q - start_joints + np.pi) % (2 * np.pi) - np.pi

    for step in range(1, args.steps + 1):
        frac = step / args.steps

        if args.interp == "cartesian":
            want_pos = start_ee + (tgt - start_ee) * frac
            way = Pose(tuple(want_pos), tuple(slerp([frac]).as_quat()[0]))
            for i, jj in enumerate([1, 2, 3, 4, 5, 6, 7]):
                p.resetJointState(rb.robot_id, jj, float(seed[i]),
                                  physicsClientId=rb.physics_client_id)
            wp = multiply_poses(scene.robot_base_pose, way)
            sol = p.calculateInverseKinematics(
                rb.robot_id, rb.end_effector_id, list(wp.position), list(wp.orientation),
                maxNumIterations=400, residualThreshold=1e-5,
                physicsClientId=rb.physics_client_id)
            qs = np.asarray(sol[:7], dtype=float)
            err_s = float(np.linalg.norm(_fk(qs) - want_pos))
        else:
            qs = start_joints + total_delta * frac
            want_pos = _fk(qs)   # where the EE actually goes; not a straight line
            err_s = None         # no IK solve here, so there is no residual to report

        delta = float(np.degrees(np.max(np.abs(qs - seed))))
        worst = max(worst, delta)
        bad = []
        if err_s is not None and err_s > MAX_IK_ERR_M:
            bad.append(f"ik {err_s*100:.1f}cm")
        if delta > args.max_step_deg:
            bad.append(f"step {delta:.0f}deg")
        # "n/a" rather than 0.00: in joint mode no IK is solved per step, and a
        # printed 0.00 reads like a suspiciously perfect measurement.
        ik_col = "  n/a  " if err_s is None else f"{err_s*100:5.2f} cm"
        print(f"  step {step:02d}/{args.steps}  EE {np.round(want_pos,3)}  "
              f"ik {ik_col}  step {delta:5.1f} deg  "
              f"{'FAIL: ' + '; '.join(bad) if bad else 'OK'}")
        # Fail closed: a partially-executed chain leaves the arm somewhere unplanned.
        if bad:
            sys.exit(f"Sub-step {step} failed its guard -- refusing the whole move. "
                     f"Try more --steps, or reposition closer first.")
        plan.append((f"step {step:02d}", want_pos, qs))
        seed = qs

    # Whichever mode ran, confirm the chain actually ends at the pose the gates
    # above validated. Catches null-space drift, a wrapped joint, a bad branch.
    end_err = float(np.linalg.norm(_fk(plan[-1][2]) - tgt))
    posture = float(np.degrees(np.max(np.abs(
        (plan[-1][2] - q + np.pi) % (2 * np.pi) - np.pi))))
    print(f"chain OK      : worst single step {worst:.1f} deg "
          f"(vs {np.degrees(jump):.1f} deg unchained)")
    print(f"endpoint      : {end_err*100:.3f} cm from the gated target, "
          f"posture {posture:.1f} deg from the gated joint solution")
    if end_err > MAX_IK_ERR_M:
        sys.exit(f"Chain ends {end_err*100:.1f} cm off target -- refusing.")
elif np.degrees(jump) > 30.0:
    print(f"\nNOTE: {np.degrees(jump):.0f} deg in one move. Consider --steps 12 to split it "
          f"(see scripts/session/check_joint_path.py to inspect the unchained EE path).")

if not args.execute:
    print("\nDRY RUN -- nothing commanded. Re-run with --execute to move.")
    sys.exit(0)

# ---- execute ----------------------------------------------------------------
print(f"\nspeed: {ai.get_speed()}  -- commanding {len(plan)} move(s) to pre-grasp ...")
for name, want, qq in plan:
    ai.execute_command(JointCommand(pos=qq.tolist()))

    # A move can return before it settles -- wait for velocity ~0, then re-check.
    for _ in range(100):
        time.sleep(0.2)
        st = ai.get_state()
        if float(np.max(np.abs(np.asarray(st["velocity"], dtype=float)))) < 1e-3:
            break
    time.sleep(0.5)

    if len(plan) > 1:
        here = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
        track = float(np.linalg.norm(here - np.asarray(want)))
        print(f"  {name}: EE {np.round(here,4)}  tracking {track*100:4.1f} cm")
        if track > MAX_STEP_TRACK_M:
            sys.exit(f"Tracking {track*100:.1f} cm at {name} -- ABORT, holding here.")

st = ai.get_state()
final_ee = np.asarray(list(st["ee_pos"])[:3], dtype=float)
err = final_ee - tgt
print(f"\nfinal EE      : [{final_ee[0]:.4f}, {final_ee[1]:.4f}, {final_ee[2]:.4f}]")
print(f"target        : [{tgt[0]:.4f}, {tgt[1]:.4f}, {tgt[2]:.4f}]")
print(f"tracking error: [{err[0]:+.4f}, {err[1]:+.4f}, {err[2]:+.4f}]  |e| = {np.linalg.norm(err)*100:.1f} cm")
print(f"gripper       : {float(st.get('gripper_pos')):.4f}  (untouched -- no grasp)")
print(f"distance to handle now: {np.linalg.norm(final_ee-h)*100:.1f} cm")
