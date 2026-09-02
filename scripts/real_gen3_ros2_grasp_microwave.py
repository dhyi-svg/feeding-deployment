"""Full perception-driven grasp: detect ONCE, then pre-grasp -> grasp -> close.

Nothing is re-detected once the arm is close: at ~2 cm the wrist camera cannot
see the microwave and the plane fit returns a handle ~7 cm off (2026-07-29).
Does NOT run the opening arc -- the perceived hinge is wrong-side.
"""
import argparse, os, sys, time
from pathlib import Path
import numpy as np, pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CloseGripperCommand, JointCommand)
from feeding_deployment.perception.detection_service import connect as connect_detector
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
# Floor lowered from 0.35 on 2026-08-22: the microwave's table was lowered, putting
# the handle at z~0.32. Still catches a fit onto the floor or a shelf.
PLAUSIBLE_Z = (0.25, 0.65)
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

a = argparse.ArgumentParser()
a.add_argument("--execute", action="store_true")
a.add_argument(
    "--steps", type=int, default=1,
    help="split EACH of the two moves (pre-grasp, grasp) into N interpolated joint "
         "sub-steps. Default 1 = single command per move, the historical behaviour. "
         "12 is the proven value on the approach script.")
a.add_argument(
    "--max-step-deg", type=float, default=20.0,
    help="with --steps, abort if any ONE sub-step moves a joint more than this")
a.add_argument(
    "--max-detects", type=int, default=5,
    help="keep detecting until two agree within 3 cm, up to this many looks (default 5). "
         "Each look costs ~85 s.")
args = a.parse_args()
if args.steps < 1:
    sys.exit("--steps must be >= 1")

ai = ArmInterfaceClient(); st = ai.get_state()
ee0 = np.asarray(list(st["ee_pos"])[:3], dtype=float); g0 = float(st.get("gripper_pos"))
print(f"start EE {np.round(ee0,4)}  gripper {g0:.4f}")
if g0 > 0.2: sys.exit("Gripper not open -- refusing.")

# Prefer a resident detector: it skips ~6 min of imports, model load and cold-disk
# paging. Falls back to loading in-process when no server is running.
svc = connect_detector()
rs = apc = None
if svc is not None:
    depth_corr, lat_corr = svc.corrections()
    print(f"using detection server (no local model load); corrections "
          f"depth={depth_corr} lat={lat_corr}")
else:
    print("no detection server -- loading the model in-process (~6 min cold).\n"
          "  For ~73 s runs instead, start it once in its own terminal:\n"
          "    $PY -u scripts/session/detection_server.py")
    from feeding_deployment.perception.appliance_perception.appliance_perception import AppliancePerception
    from feeding_deployment.perception.grounded_sam import GroundedSAM
    from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0): sys.exit("No RGB-D frames")
# Detection diagnostics. AppliancePerception already builds every overlay and already
# writes a JSON sidecar with the intrinsics + the 4x4 base<-camera matrix, but both
# hooks no-op when data_logger is None. Set DETECTION_LOG_DIR to keep them, which is
# also what makes scripts/session/replay_detection.py possible. This script detects
# TWICE, so both looks are captured -- that is exactly what you want when the two
# disagree and you need to see which one drifted.
    _log_dir = os.environ.get("DETECTION_LOG_DIR")
    _data_logger = None
    if _log_dir:
        from feeding_deployment.integration.data_logger import DataLogger
        _data_logger = DataLogger(Path(_log_dir), day=1)
        _data_logger.begin_hla("grasp_microwave")
        print(f"detection logging -> {_log_dir}")
    apc = AppliancePerception(GroundedSAM(), data_logger=_data_logger)
    depth_corr, lat_corr = apc.handle_depth_corr, apc.handle_lat_corr
if depth_corr == 0.0: sys.exit("HANDLE_DEPTH_CORR not set -- refusing.")
def detect_once(tag):
    if svc is not None:
        r = svc.detect("microwave handle")
        if r is None:
            sys.exit(f"NO DETECTION ({tag})")
        hh = Pose(tuple(r["position"]), tuple(r["orientation"]))
        top_z_val = r["top_z"]
    else:
        d = rs.get_camera_data()
        hh, _, _, top = apc.detect_handle_and_placement(
            "microwave handle", d["rgb_image"], d["camera_info"], d["depth_image"])
        if hh is None:
            sys.exit(f"NO DETECTION ({tag})")
        top_z_val = float(np.asarray(top.position)[2]) if top is not None else float("nan")
    v = np.asarray(hh.position)
    # top_of_appliance drives the repo's post_release_pose (lift to top + 5 cm),
    # which is how open_microwave() exits -- straight up, not backwards.
    tz = top_z_val
    print(f"  detect {tag}: {np.round(v,4)}   top_of_appliance z {tz:.4f}")
    return hh, v, tz

# Keep looking until two agree, rather than aborting on one bad pair. Background
# clutter can out-vote the handle cluster intermittently, so a lone outlier is common.
dets, pair = [], None
for i in range(1, args.max_detects + 1):
    dets.append(detect_once(str(i)))
    for j in range(len(dets) - 1):
        spread = float(np.linalg.norm(dets[j][1] - dets[-1][1]))
        if spread <= DETECT_AGREE:
            pair = (dets[j], dets[-1], spread)
            break
    if pair:
        break
    if i > 1:
        print(f"  no pair within {DETECT_AGREE*100:.0f} cm yet after {i} looks; re-detecting")
if pair is None:
    sys.exit(f"No two of {len(dets)} detections agreed within {DETECT_AGREE*100:.0f} cm "
             f"-- refusing. Check the overlays in $DETECTION_LOG_DIR.")
(_, ha, tza), (hbo, hb, tzb), spread = pair
print(f"detections agree to {spread*100:.1f} cm (limit {DETECT_AGREE*100:.0f}), "
      f"used {len(dets)} look(s)")
h = (ha + hb) / 2.0
handle = hbo
top_z = float(np.nanmean([tza, tzb]))
print(f"top_of_appliance z {top_z:.4f} -> post-release lift target {top_z+0.05:.4f}")
Path("/tmp/microwave_top_z.txt").write_text(str(top_z))
for nm, v, lo_hi in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y), ("z", h[2], PLAUSIBLE_Z)):
    if not (lo_hi[0] <= v <= lo_hi[1]):
        sys.exit(f"handle {nm}={v:.3f} outside plausible {lo_hi} -- refusing.")
print(f"handle (mean) {np.round(h,4)}   {np.linalg.norm(h-STALE_TRUTH)*100:.1f} cm from the STALE pre-arc truth (informational)")
print(f"camera->handle {np.linalg.norm(h-ee0)*100:.0f} cm (viewing distance)")

quat = (R.from_quat(np.asarray(handle.orientation)) * R.from_euler("y", np.pi)).as_quat()
hp = Pose(tuple(h), tuple(quat))
def _pose_to_matrix(pose):
    m = np.zeros((4,4)); m[:3,3] = pose[0]
    m[:3,:3] = R.from_quat(pose[1]).as_matrix(); m[3,3] = 1
    return m
def off(v):
    m = np.eye(4); m[:3,3] = v
    out = _pose_to_matrix(hp) @ m
    return Pose(out[:3,3], R.from_matrix(out[:3,:3]).as_quat())
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

def fk(joints):
    """EE position in arm_base_link for a joint vector."""
    for i, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(joints[i]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    return np.asarray(ls[4]) - np.asarray(scene.robot_base_pose.position)

# Split each move into small increments. JOINT-space interpolation (same choice as the
# approach script's default): it ends at exactly the IK solution the gates above passed,
# per-step motion is bounded by construction at total/N, and no null-space drift is
# possible. The EE follows the same curve the unchained move would have taken -- just
# cut into pieces that can be checked and aborted between.
if args.steps > 1:
    chained, prev = [], np.asarray(st["position"], dtype=float)
    print(f"\nchaining each move into {args.steps} joint sub-steps "
          f"(guard {args.max_step_deg} deg/step):")
    for name, t, q in plan:
        # Shortest path per joint: a raw difference can read ~350 deg for what is
        # physically a few degrees the other way round.
        total = (q - prev + np.pi) % (2 * np.pi) - np.pi
        base_q, worst = prev.copy(), 0.0
        for s in range(1, args.steps + 1):
            qs = base_q + total * (s / args.steps)
            delta = float(np.degrees(np.max(np.abs(qs - prev))))
            worst = max(worst, delta)
            if delta > args.max_step_deg:
                sys.exit(f"Sub-step {s} of '{name}' moves {delta:.0f} deg "
                         f"(> {args.max_step_deg}) -- refusing the whole move.")
            chained.append((f"{name} {s:02d}/{args.steps}", fk(qs), qs))
            prev = qs
        end_err = float(np.linalg.norm(fk(prev) - t))
        print(f"  {name:10s} {args.steps} steps, worst {worst:5.1f} deg  "
              f"endpoint {end_err*100:.3f} cm from the gated target")
        if end_err > MAX_IK_ERR:
            sys.exit(f"Chain for '{name}' ends {end_err*100:.1f} cm off target -- refusing.")
    plan = chained

if not args.execute:
    sys.exit("\nDRY RUN -- nothing commanded.")

print(f"\nspeed: {ai.get_speed()}  -- commanding {len(plan)} move(s) ...")
for name, t, q in plan:
    ai.execute_command(JointCommand(pos=q.tolist()))
    for _ in range(120):
        time.sleep(0.2)
        s = ai.get_state()
        if float(np.max(np.abs(np.asarray(s["velocity"], dtype=float)))) < 1e-3: break
    time.sleep(0.4)
    fin = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
    e = float(np.linalg.norm(fin - t))
    print(f"  {name:18s} EE {np.round(fin,4)}  tracking {e*100:4.1f} cm")
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
