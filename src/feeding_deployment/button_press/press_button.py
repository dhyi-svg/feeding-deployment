#!/usr/bin/env python3
"""One-command microwave button press: detect the panel, go to the pre-press spot, press, come back.

DRY RUN BY DEFAULT. Nothing moves without ``--execute``.

    python3 -u -m feeding_deployment.button_press.press_button             # plan only (START/+30SEC)
    python3 -u -m feeding_deployment.button_press.press_button --execute   # do it
    ... --target timer_clock                                               # the Timer/Clock button instead

Start with the panel in view (the step before this one is "the arm is in front of the
microwave"). Then, in one call:

  1. preflight      arm ready, speed low, gripper closed, detector locked on --target, tf up
  2. detect         dome-layout detector (dome_pattern.py: the 5 chrome domes' 3+2 layout, any
                    range ~22-50+ cm, no reference images) -> button pixel; depth plane fit ->
                    button xyz + panel normal; tf -> arm base frame. That fixes the PANEL FRAME:
                    origin = the button, z = out of the panel, y = gravity-up, x = y cross z.
     2b. stage      camera further than CLOSE_VIEW_M: go to the pre-press pose + STAGE_OUT_M,
                    and re-detect there with the dome layout again (SIFT as fallback) -- the far
                    3D estimate is biased ~1.5 cm; the ~25 cm one matches how the offset was measured
  3. plan           tool goal = PREPRESS_EE_OFFSET_M / PREPRESS_EE_QUAT_PANEL (constants below)
                    in that frame; the whole sequence (go, press in/out, come back) is planned
                    and gated in the sim BEFORE anything moves
  4. go             straight line to the pre-press spot, wrist turning to face the panel
  5. refine         (--refine only) SIFT re-detect at the spot; correct once if > REFINE_MIN_M off
  6. press          --press-in straight into the panel along its normal, hold, same distance out
  7. return         straight line back to where it started

Because the panel frame's origin is the target button itself, the one stored offset is right
for every button the reference marks (start_30s, timer_clock).

No force sensing (the wrench estimate is unusable with the current fingers): the press depth
is pure geometry, capped at MAX_PRESS_IN_M. Someone stands at the e-stop for every --execute.
Needs the stack from scripts/button_press/bringup.sh (no press_detector).
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from scipy.spatial.transform import Rotation

from feeding_deployment.button_press import Abort, dome_pattern
from feeding_deployment.button_press.autonomous_press import MAX_PANEL_NORMAL_Z, MAX_PRESS_IN_M, Run, build_arg_parser
from feeding_deployment.button_press.geometry import PlaneFitError
from feeding_deployment.button_press.perception import FRESH_S, LOCK_HOLD_S, MIN_INLIERS
from feeding_deployment.button_press.panel_frame import (
    describe,
    from_panel,
    panel_frame,
    rot_angle_deg,
    rot_from_panel,
    to_panel,
)

# ---- the pre-press spot, in the panel frame (x right, y up, z out of the panel; origin = button)
# Kinova tool-frame position and orientation with the LEFT fingertip STANDOFF_M in front of the
# button. MEASURED 2026-10-03 on timer_clock, current fingers: panel frame from the dome layout
# at 30.9 cm, then the fingertip hand-placed 2 cm in front of the button (tool z 10 deg off
# straight-in). Re-measure with scripts/button_press/measure_prepress_offset.py if the fingers or
# camera mount change. (The 09-27 placeholder it replaced had the tool 4.2 cm below the button,
# taught from a biased ~40 cm view -- it would have missed.)
PREPRESS_EE_OFFSET_M = np.array([-0.0049, -0.0114, -0.0131])   # 2nd hand placement (1st: -0.0079, -0.0065, -0.0095)
PREPRESS_EE_QUAT_PANEL = np.array([-0.04947, 0.99522, -0.08381, -0.00846])   # xyzw
STANDOFF_M = 0.02
PRESS_IN_M = 0.010            # set 2026-10-03 by the user (tests from the 2nd hand-placed spot: 1.2, 1.5 cm;
                              # 1.0 cm from the 1st spot, 3.6 mm further back, did not press)
# ---- refusals before anything moves
MAX_GOTO_M = 0.40             # start -> pre-press straight line
MAX_GOTO_ROT_DEG = 45.0       # wrist turn on the way
# ---- far start: stage closer and re-detect before planning the approach
# From ~40 cm the detection is biased: on 2026-10-03 the same, unmoved button came out 1.2 cm
# lower from 39 cm than from 20 cm (normal z 0.154 vs 0.112) -- a button-width miss, and it put
# the goal under the Z floor. 2026-09-27 saw 2-3 cm at 40 cm. 17-22 cm is the validated range.
CLOSE_VIEW_M = 0.25           # camera -> button beyond this: stage first
STAGE_OUT_M = 0.08            # staging pose = pre-press pose this much further out (camera ~25 cm,
                              # inside the dome detector's range: it abstains below MIN_RANGE_M 0.22)
STAGE_SETTLE_S = 0.5          # after the smooth stop, before re-detecting
STAGE_LOCK_HOLD_S = 1.0       # SIFT lock held this long at the staging pose (preflight uses 2 s)
# ---- dome-layout detector (far/coarse; see dome_pattern.py)
DOME_FRAMES = 7               # frames looked at per measurement
DOME_MIN_FITS = 5             # ... of which this many must fit all 5 domes
DOME_MAX_SPREAD_PX = 3.0      # and agree on the target pixel to within this
SIFT_WAIT_S = 6.0             # at the staging pose, wait this long for a SIFT lock before using domes
# ---- close-range refine
REFINE_MIN_M = 0.003          # ignore smaller corrections (detector noise at ~17 cm is ~1-2 mm)
REFINE_MAX_M = 0.01           # bigger than this up close means something is wrong: abort, don't chase it
REFINE_WAIT_S = 3.0


def set_detector_target(node, ns: str, target: str, timeout_s: float = 5.0):
    """Point the running detector at `target` (its target_button parameter)."""
    cli = node.create_client(SetParameters, f"{ns}/set_parameters")
    if not cli.wait_for_service(timeout_sec=timeout_s):
        raise Abort(f"{ns}/set_parameters not available -- is the button detector running?")
    req = SetParameters.Request()
    req.parameters = [Parameter(name="target_button",
                                value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=target))]
    fut = cli.call_async(req)
    t0 = time.monotonic()
    while not fut.done():
        if time.monotonic() - t0 > timeout_s:
            raise Abort(f"setting {ns} target_button timed out")
        time.sleep(0.05)
    res = fut.result().results[0]
    if not res.successful:
        raise Abort(f"detector refused target {target!r}: {res.reason}")


def publish_pose(pub, frame, p, Rm):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, p)
    q = Rotation.from_matrix(Rm).as_quat()
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
    pub.publish(msg)


def prepress_pose(origin, R, out=0.0):
    """Tool (xyz, R) for the stored pre-press spot, `out` metres further out along the normal."""
    p = from_panel(PREPRESS_EE_OFFSET_M + np.array([0.0, 0.0, out]), origin, R)
    return p, rot_from_panel(Rotation.from_quat(PREPRESS_EE_QUAT_PANEL).as_matrix(), R)


def measure_panel_frame_domes(run: Run, target: str):
    """Panel frame from the dome-layout detector: median over DOME_FRAMES frames.

    Same output as Run.measure_panel_frame (origin = button xyz, base frame), but needs no
    reference images and works from ~25-50+ cm, where the SIFT lock fails.
    """
    per = run.per
    if target not in dome_pattern.NAMES:
        raise Abort(f"dome detector knows {dome_pattern.NAMES}, not {target!r}")
    fx, fy, cx, cy = per.intrinsics()
    fits, seen, last, t0 = [], 0, None, time.monotonic()
    while seen < DOME_FRAMES and time.monotonic() - t0 < 6.0:
        color, depth = per.get("color", FRESH_S), per.get("depth", FRESH_S)
        if color is None or depth is None or color is last:
            time.sleep(0.03)
            continue
        last = color
        seen += 1
        bgr = per.bridge.imgmsg_to_cv2(color, "bgr8")
        d = per.bridge.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        if depth.encoding in ("16UC1", "mono16"):
            d = d / 1000.0
        fit = dome_pattern.detect(bgr, d, fx)
        if fit is None:
            continue
        try:
            n, dd, _, _ = dome_pattern.panel_plane(bgr, d, fit, (fx, fy, cx, cy))
        except PlaneFitError:
            continue
        fits.append((np.array(fit.px[target]), n, dd, fit))
    pxs = np.array([f[0] for f in fits]) if fits else np.zeros((0, 2))
    px = np.median(pxs, axis=0) if fits else None
    # Drop single odd frames (a highlight flicker, a frame grabbed mid-settle) instead of failing
    # on them; a camera that is actually moving leaves too few frames agreeing and still aborts.
    keep = [f for f, p in zip(fits, pxs) if np.linalg.norm(p - px) <= DOME_MAX_SPREAD_PX] if fits else []
    print(f"  dome layout: {len(fits)}/{seen} frames fit all 5 domes, {len(keep)} agree within {DOME_MAX_SPREAD_PX:.0f} px")
    run.log({"dome_frames": {"target_px": pxs.tolist(), "seen": seen, "agree": len(keep)}})
    if not fits:
        raise Abort(f"dome detector: no 5-dome layout in {seen} frames -- panel out of view, or the camera is "
                    f"closer than {dome_pattern.MIN_RANGE_M*100:.0f} cm (SIFT's range)")
    if len(keep) < DOME_MIN_FITS:
        raise Abort(f"dome detector: only {len(keep)}/{seen} frames agree on the {target} pixel (need {DOME_MIN_FITS}; "
                    f"pixels {np.round(pxs).astype(int).tolist()}) -- is the arm still moving?")
    fits = keep
    pxs = np.array([f[0] for f in fits])
    px = np.median(pxs, axis=0)
    spread = float(np.max(np.linalg.norm(pxs - px, axis=1)))
    n = np.mean([f[1] for f in fits], axis=0)
    n /= np.linalg.norm(n)
    dd = float(np.median([f[2] for f in fits]))
    x_cam = dome_pattern.button_xyz_cam(px, n, dd, (fx, fy, cx, cy))
    R_bc = per.cam_rotation_in_base()
    button = per.cam_position_in_base() + R_bc @ x_cam
    n_base = R_bc @ n
    if abs(float(n_base[2])) > MAX_PANEL_NORMAL_Z:
        raise Abort(f"dome detector: panel normal {np.round(n_base, 3)} is not near-horizontal -- bad plane fit")
    f0 = fits[0][3]
    print(f"  {target} pixel {np.round(px, 1)} (spread {spread:.1f} px), dome pitch {f0.s_mm:.1f} mm, "
          f"{np.linalg.norm(x_cam)*100:.1f} cm from the camera")
    print(f"  button xyz (base) {np.round(button, 3)}   panel normal (out) {np.round(n_base, 3)}")
    origin, R = panel_frame(button, n_base)
    run.log({"panel_frame_domes": {"origin": origin.tolist(), "R": R.tolist(), "px": px.tolist(),
                                   "fits": len(fits), "seen": seen, "spread_px": spread}})
    return origin, R


def sift_locked(run: Run, target: str, wait_s: float, hold_s: float = LOCK_HOLD_S) -> bool:
    """True once the SIFT detector has held a MIN_INLIERS lock on `target` for hold_s."""
    t0, held = time.monotonic(), 0.0
    while time.monotonic() - t0 < wait_s:
        ok, _ = run.per.locked(MIN_INLIERS)
        held = held + 0.1 if ok and run.per.lock_target() == target else 0.0
        if held >= hold_s:
            return True
        time.sleep(0.1)
    return False


def locate(run: Run, target: str, prefer: str):
    """(origin, R, method). `prefer` = "domes" (far/coarse) or "sift" (close/fine); falls back
    to the other method if the preferred one cannot measure."""
    order = ["domes", "sift"] if prefer == "domes" else ["sift", "domes"]
    errors = []
    for how in order:
        try:
            if how == "sift":
                if not (sift_locked(run, target, SIFT_WAIT_S, STAGE_LOCK_HOLD_S) if prefer == "sift"
                        else sift_locked(run, target, LOCK_HOLD_S + 0.5)):
                    raise Abort(f"SIFT detector not locked on {target}: {run.per.locked(MIN_INLIERS)[1]}")
                print("  [SIFT reference match]")
                origin, R = run.measure_panel_frame()
            else:
                print("  [dome layout]")
                origin, R = measure_panel_frame_domes(run, target)
            return origin, R, how
        except Abort as e:
            print(f"  {how}: {e}")
            errors.append(f"{how}: {e}")
    raise Abort("could not locate the button -- " + " | ".join(errors))


def check_move(p_from, R_from, p_to, R_to, what):
    dist, turn = float(np.linalg.norm(p_to - p_from)), rot_angle_deg(R_from, R_to)
    if dist > MAX_GOTO_M:
        raise Abort(f"{what} is {dist*100:.0f} cm away (> {MAX_GOTO_M*100:.0f}) -- start closer")
    if turn > MAX_GOTO_ROT_DEG:
        raise Abort(f"wrist would turn {turn:.0f} deg to the {what} (> {MAX_GOTO_ROT_DEG:.0f}) -- start facing the panel")


def press_button(run: Run, a, pub) -> int:
    arm = run.arm
    run.preflight(require_lock=False, require_free=False)   # either detector may find it
    p0, R0 = arm.ee_pose()

    print(f"\n== detect: {a.target} ==")
    origin, R, how = locate(run, a.target, prefer="domes")
    cam_d = float(np.linalg.norm(origin - run.per.cam_position_in_base()))
    if cam_d > CLOSE_VIEW_M:
        # Far: go to a staging pose in front of the (rough) button, re-detect from there, and
        # plan everything else from the close measurement.
        p_st, R_st = prepress_pose(origin, R, out=STAGE_OUT_M)
        print(f"  camera is {cam_d*100:.0f} cm from the button (> {CLOSE_VIEW_M*100:.0f}): staging closer first")
        check_move(p0, R0, p_st, R_st, "staging pose")
        stage = run.pose_path(p0, R0, p_st, R_st)
        err, jump, _ = arm.precheck_poses(stage, "to staging", run.log, posture=run.seed_posture)
        print(f"  to staging       {len(stage):3d} pts  IK <= {err*1000:.1f} mm  joint step <= {jump:.1f} deg  ok")
        if not a.execute:
            print("\ndry run -- the staging leg passed the gates. The approach and press are planned only after "
                  "re-detecting at the staging pose, so they need --execute (or start within "
                  f"{CLOSE_VIEW_M*100:.0f} cm to dry-run the whole thing)")
            return 0
        print("\n== go to the staging pose ==")
        try:
            run.execute_path(stage, "to staging")
        except Abort as e:
            run.go_back(p0, R0, f"move aborted ({e})")
            raise
        time.sleep(STAGE_SETTLE_S)
        run.reanchor_seed("at the staging pose")
        print("\n== re-detect from the staging pose ==")
        far = origin
        origin, R, how = locate(run, a.target, prefer="domes")
        print(f"  button moved {np.round((origin - far) * 100, 1)} cm vs the far estimate")
        run.log({"stage": "staging_redetect", "far_button_xyz": far.tolist(), "button_xyz": origin.tolist()})
    n_out = R[:, 2]
    p_goal, R_goal = prepress_pose(origin, R)
    p_now, R_now = arm.ee_pose()
    print(f"  pre-press tool pose: {describe(PREPRESS_EE_OFFSET_M)}")
    print(f"  tool xyz now  {np.round(p_now, 3)}")
    print(f"  tool xyz goal {np.round(p_goal, 3)}   ({np.linalg.norm(p_goal - p_now)*100:.1f} cm straight line, "
          f"wrist turns {rot_angle_deg(R_now, R_goal):.1f} deg)")
    publish_pose(pub, a.arm_frame, p_goal, R_goal)
    run.log({"stage": "press_button_plan", "target": a.target, "button_xyz": origin.tolist(), "panel_R": R.tolist(),
             "ee_start": p0.tolist(), "ee_now": p_now.tolist(), "ee_goal": p_goal.tolist()})
    check_move(p_now, R_now, p_goal, R_goal, "pre-press spot")

    # Plan every leg back to back in the sim before anything moves.
    print("\n== plan (sim pre-check of every leg) ==")
    go = run.pose_path(p_now, R_now, p_goal, R_goal)
    legs = [("to pre-press", go)]
    for i in range(a.presses):
        legs += [(f"press {i + 1} in", run.pose_path(p_goal, R_goal, p_goal - a.press_in * n_out, R_goal)),
                 (f"press {i + 1} out", run.pose_path(p_goal - a.press_in * n_out, R_goal, p_goal, R_goal))]
    back = run.pose_path(p_goal, R_goal, p0, R0)
    if not a.no_return:
        legs.append(("back to start", back))
    state = None
    for name, path in legs:
        err, jump, state = arm.precheck_poses(path, name, run.log, posture=run.seed_posture, start=state)
        print(f"  {name:16s} {len(path):3d} pts  IK <= {err*1000:.1f} mm  joint step <= {jump:.1f} deg  ok")
    if not a.execute:
        print("\ndry run -- every leg passed the gates; add --execute to move")
        return 0

    print("\n== go to the pre-press spot ==")
    try:
        run.execute_path(go, "to pre-press")
    except Abort as e:
        run.go_back(p0, R0, f"move aborted ({e})")
        raise

    if a.refine:
        print("\n== refine: re-detect up close ==")
        time.sleep(REFINE_WAIT_S)   # let the detector see a few still frames
        b = run.button_on_plane(origin, n_out)
        if b is None:
            print("  no fresh lock up close -- keeping the first estimate")
        else:
            d = b - origin
            d_lat = d - np.dot(d, n_out) * n_out
            print(f"  button re-detected {np.round(to_panel(b, origin, R) * 100, 1)} cm from the first estimate "
                  f"(in-plane {np.linalg.norm(d_lat)*1000:.1f} mm)")
            run.log({"stage": "refine", "button_xyz": b.tolist(), "lateral_m": float(np.linalg.norm(d_lat))})
            if np.linalg.norm(d_lat) > REFINE_MAX_M:
                raise Abort(f"button is {np.linalg.norm(d_lat)*100:.1f} cm from where the far view put it "
                            f"(> {REFINE_MAX_M*100:.0f}) -- HOLDING HERE at the standoff (nothing pressed)")
            if np.linalg.norm(d_lat) > REFINE_MIN_M:
                p_new = p_goal + d_lat
                fix = run.pose_path(p_goal, R_goal, p_new, R_goal)
                arm.precheck_poses(fix, "refine", run.log, posture=run.seed_posture)
                run.execute_path(fix, "refine")
                p_goal = p_new

    if a.presses:
        run.press_along_normal(n_out)   # backs itself out on abort

    if a.no_return:
        print("\ndone (--no-return: holding at the pre-press spot).")
        return 0
    print("\n== back to start ==")
    p_now, R_now = arm.ee_pose()
    back = run.pose_path(p_now, R_now, p0, R0)
    arm.precheck_poses(back, "back to start", run.log, posture=run.seed_posture)
    run.execute_path(back, "back to start")
    print("\ndone.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="start_30s",
                    help="which button: start_30s (default, START/+30SEC) or timer_clock. The stored pre-press "
                         "spot is relative to the target button, so it is the same for both.")
    ap.add_argument("--execute", action="store_true", help="actually move the arm (default: dry run)")
    ap.add_argument("--presses", type=int, default=1, help="0 = go to the spot and come back without pressing")
    ap.add_argument("--press-in", type=float, default=PRESS_IN_M,
                    help=f"push this far (m) into the panel from the spot (default {PRESS_IN_M}, max {MAX_PRESS_IN_M})")
    ap.add_argument("--refine", action="store_true",
                    help="re-detect at the pre-press spot (SIFT only -- ~17 cm is too close for the dome "
                         "detector) and correct once; off by default because mixing the two detectors adds "
                         "their 1-2 mm disagreement as a fake correction")
    ap.add_argument("--no-return", action="store_true", help="stay at the pre-press spot at the end")
    ap.add_argument("--steps", action="store_true",
                    help="move in 1 cm / 4 deg stop-and-check joint steps instead of the default: each leg "
                         "as ONE smooth blended Cartesian trajectory (every point pre-checked either way)")
    ap.add_argument("--ns", default="/button_detector")
    ap.add_argument("--arm-frame", default="arm_base_link")
    ap.add_argument("--camera-frame", default="camera_color_optical_frame")
    return ap


def main(argv: list[str] | None = None) -> int:
    a = build_parser().parse_args(argv)
    if a.presses < 0:
        print("--presses must be >= 0")
        return 2
    if not 0 < a.press_in <= MAX_PRESS_IN_M:
        print(f"--press-in {a.press_in} outside (0, {MAX_PRESS_IN_M}]")
        return 2
    # Reuse the driver's Run (perception node, gated arm, press stroke) with force sensing off.
    run_args = build_arg_parser().parse_args(["--no-force"])
    for k in ("execute", "target", "presses", "press_in", "ns", "arm_frame", "camera_frame"):
        setattr(run_args, k, getattr(a, k))
    run_args.smooth = not a.steps

    rclpy.init()
    try:
        run = Run(run_args)
        run.allowed_speeds = ("low", "medium")
        pub = run.per.create_publisher(PoseStamped, "/button_press/target_pose", 1)
        try:
            set_detector_target(run.per, a.ns, a.target)
            return press_button(run, a, pub)
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
