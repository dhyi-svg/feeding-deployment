"""Full perception-driven grasp: detect ONCE, then pre-grasp -> grasp -> close.

Nothing is re-detected once the arm is close: at ~2 cm the wrist camera cannot
see the microwave and the plane fit returns a handle ~7 cm off (2026-07-29).
Does NOT run the opening arc -- the perceived hinge is wrong-side.
"""
import argparse, sys, time
from pathlib import Path
import numpy as np, pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CloseGripperCommand, JointCommand)
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.perception.grounded_sam import GroundedSAM
from feeding_deployment.perception.appliance_perception.appliance_perception import AppliancePerception
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# Stale reference: measured before the arc, and the microwave has since shifted.
# Kept for REPORTING only -- no longer gated on.
STALE_TRUTH = np.array([0.6932, -0.1171, 0.5031])

# Self-consistency gate, replacing the absolute-truth one. Two independent
# detections of the same handle must agree. This needs no ground truth (which
# goes stale the moment the appliance moves) and directly catches the failure we
# actually see: the centroid sliding along the handle bar, a 7.3 cm spread in y
# across five looks while x and z stay within 1 cm.
DETECT_AGREE = 0.03
# Loose absolute plausibility box for a microwave handle in arm_base_link
# (+x forward, +z up). Catches a wild fit without assuming a position.
PLAUSIBLE_X = (0.45, 0.85)
PLAUSIBLE_Y = (-0.40, 0.20)
PLAUSIBLE_Z = (0.35, 0.65)
PRE_STANDOFF = 0.12
GRIP_EXT = 0.0            # coupled with HANDLE_DEPTH_CORR -- see TESTING_LOG
# No lateral correction. A third detection put the handle within 3 mm of the
# teleoped grasp y (-0.1066 vs -0.1098) while an earlier one was 2.1 cm off
# (-0.1307): that is detection VARIANCE (~2.4 cm spread), not a systematic
# bias, so a fixed offset would overshoot. The 2F-85's 85 mm opening absorbs it.
LATERAL = 0.0
# 0.88: the arm demonstrably reached 0.858 under teleop, and the Gen3 spec is
# ~0.90. 0.85 was over-tight and blocked a feasible grasp (IK solved exactly).
MAX_REACH, MIN_Z, MAX_Z = 0.88, 0.25, 0.75
MAX_IK_ERR, MAX_JUMP_DEG = 0.02, 90.0
TRACK_ABORT = 0.03
ARM = [1,2,3,4,5,6,7]

a = argparse.ArgumentParser(); a.add_argument("--execute", action="store_true")
args = a.parse_args()

ai = ArmInterfaceClient(); st = ai.get_state()
ee0 = np.asarray(list(st["ee_pos"])[:3], dtype=float); g0 = float(st.get("gripper_pos"))
print(f"start EE {np.round(ee0,4)}  gripper {g0:.4f}")
if g0 > 0.2: sys.exit("Gripper not open -- refusing.")

rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0): sys.exit("No RGB-D frames")
apc = AppliancePerception(GroundedSAM())
if apc.handle_depth_corr == 0.0: sys.exit("HANDLE_DEPTH_CORR not set -- refusing.")
def detect_once(tag):
    d = rs.get_camera_data()
    hh, _, _, top = apc.detect_handle_and_placement(
        "microwave handle", d["rgb_image"], d["camera_info"], d["depth_image"])
    if hh is None:
        sys.exit(f"NO DETECTION ({tag})")
    v = np.asarray(hh.position)
    # top_of_appliance drives the repo's post_release_pose (lift to top + 5 cm),
    # which is how open_microwave() exits -- straight up, not backwards.
    tz = float(np.asarray(top.position)[2]) if top is not None else float("nan")
    print(f"  detect {tag}: {np.round(v,4)}   top_of_appliance z {tz:.4f}")
    return hh, v, tz

h1o, h1, tz1 = detect_once("1")
h2o, h2, tz2 = detect_once("2")
spread = float(np.linalg.norm(h1 - h2))
print(f"detections agree to {spread*100:.1f} cm (limit {DETECT_AGREE*100:.0f})")
if spread > DETECT_AGREE:
    sys.exit(f"Detections disagree by {spread*100:.1f} cm -- unstable, refusing.")
h = (h1 + h2) / 2.0
handle = h2o
top_z = float(np.nanmean([tz1, tz2]))
print(f"top_of_appliance z {top_z:.4f} -> post-release lift target {top_z+0.05:.4f}")
Path("/tmp/microwave_top_z.txt").write_text(str(top_z))
for nm, v, lo_hi in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y), ("z", h[2], PLAUSIBLE_Z)):
    if not (lo_hi[0] <= v <= lo_hi[1]):
        sys.exit(f"handle {nm}={v:.3f} outside plausible {lo_hi} -- refusing.")
print(f"handle (mean) {np.round(h,4)}   {np.linalg.norm(h-STALE_TRUTH)*100:.1f} cm from the STALE pre-arc truth (informational)")
print(f"camera->handle {np.linalg.norm(h-ee0)*100:.0f} cm (viewing distance)")

quat = (R.from_quat(np.asarray(handle.orientation)) * R.from_euler("y", np.pi)).as_quat()
hp = Pose(tuple(h), tuple(quat))
helper = PerceptionInterface.__new__(PerceptionInterface)
def off(v):
    m = np.eye(4); m[:3,3] = v
    return helper.matrix_to_pose(helper.pose_to_matrix(hp) @ m)
pre   = off([0.0, 0.0, -PRE_STANDOFF])
grasp = off([LATERAL, 0.0, -GRIP_EXT])
print(f"pre-grasp {np.round(pre.position,4)}  range {np.linalg.norm(pre.position):.3f}")
print(f"grasp     {np.round(grasp.position,4)}  range {np.linalg.norm(grasp.position):.3f}")

scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False); rb = sim.robot

def solve(pose, seed):
    for i,jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed[i]), physicsClientId=rb.physics_client_id)
    w = multiply_poses(scene.robot_base_pose, pose)
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
        list(w.position), list(w.orientation), maxNumIterations=400,
        residualThreshold=1e-5, physicsClientId=rb.physics_client_id)
    q = np.asarray(sol[:7])
    for i,jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    return q, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))

cur = np.asarray(st["position"], dtype=float); plan = []
for name, pose in (("pre-grasp", pre), ("grasp", grasp)):
    t = np.asarray(pose.position)
    bad = []
    if np.linalg.norm(t) > MAX_REACH: bad.append(f"range {np.linalg.norm(t):.3f}>{MAX_REACH}")
    if not (MIN_Z <= t[2] <= MAX_Z):  bad.append(f"z {t[2]:.3f} out of range")
    q, err = solve(pose, cur)
    jump = float(np.degrees(np.max(np.abs(q-cur))))
    if err > MAX_IK_ERR:   bad.append(f"IK err {err*100:.1f}cm")
    if jump > MAX_JUMP_DEG: bad.append(f"jump {jump:.0f}deg")
    print(f"  {name:10s} IK {err*100:5.2f} cm  jump {jump:5.1f} deg  {'FAIL: '+'; '.join(bad) if bad else 'OK'}")
    if bad: sys.exit(f"GATE FAILED at {name}")
    plan.append((name, t, q)); cur = q

if not args.execute:
    sys.exit("\nDRY RUN -- nothing commanded.")

for name, t, q in plan:
    print(f"\n-> {name} ...")
    ai.execute_command(JointCommand(pos=q.tolist()))
    for _ in range(120):
        time.sleep(0.2)
        s = ai.get_state()
        if float(np.max(np.abs(np.asarray(s["velocity"], dtype=float)))) < 1e-3: break
    time.sleep(0.4)
    fin = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
    e = float(np.linalg.norm(fin - t))
    print(f"   EE {np.round(fin,4)}  tracking {e*100:.1f} cm")
    if e > TRACK_ABORT:
        sys.exit(f"Tracking {e*100:.1f} cm at {name} -- ABORT, gripper untouched.")

print("\nclosing gripper ...")
ai.execute_command(CloseGripperCommand()); time.sleep(3.5)
gf = float(ai.get_state().get("gripper_pos"))
print(f"gripper after close: {gf:.4f}")
# Deliberately no automated verdict. gripper_pos saturates near 1.0 whether or
# not the handle is between the fingers (CLAUDE.md records ~0.99 while holding the
# door), so it cannot tell success from a miss -- an earlier version called a
# confirmed-good grasp "closed empty". The proven flow uses a human grip check.
print("Gripper closed. gripper_pos cannot confirm a grasp on this rig --")
print("CHECK VISUALLY before running the arc.")
