#!/usr/bin/env python3
"""One-command microwave button press: detect the panel, go to the pre-press spot, press, come back.

DRY RUN BY DEFAULT. Nothing moves without ``--execute``.

    python3 -u -m feeding_deployment.button_press.press_button             # plan only (START/+30SEC)
    python3 -u -m feeding_deployment.button_press.press_button --execute   # do it
    ... --target timer_clock                                               # the Timer/Clock button instead

Start with the panel in view, the camera at least 22 cm from it (dome_pattern.MIN_RANGE_M).
Then, in one call:

  1. preflight      arm ready, speed low/medium, gripper closed, camera_info and tf up
  2. detect         dome-layout detector (dome_pattern.py: the 5 chrome domes' 3+2 layout,
                    ~22-50+ cm, no reference images) -> button pixel; depth plane fit ->
                    button xyz + panel normal; tf -> arm base frame. That fixes the PANEL FRAME:
                    origin = the button, z = out of the panel, y = gravity-up, x = y cross z.
     2b. stage      camera further than CLOSE_VIEW_M: go to the pre-press pose + STAGE_OUT_M and
                    re-detect there -- the far 3D estimate is biased ~1.5 cm; the ~25 cm one
                    matches how the offset was measured
  3. plan           tool goal = PREPRESS_EE_OFFSET_M / PREPRESS_EE_QUAT_PANEL (constants below)
                    in that frame; the whole sequence (go, press in/out, come back) is planned
                    and gated in the sim BEFORE anything moves
  4. go             straight line to the pre-press spot, wrist turning to face the panel
  5. press          --press-in straight into the panel along its normal, hold, same distance out
  6. return         straight line back to where it started

Because the panel frame's origin is the target button itself, the one stored offset is right
for every button the dome detector names (start_30s, timer_clock, ...).

The press depth is pure geometry, capped at MAX_PRESS_IN_M. Someone stands at the e-stop for
every --execute. Needs the stack from scripts/button_press/bringup.sh.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from feeding_deployment.button_press import Abort, dome_pattern
from feeding_deployment.button_press.arm import SMOOTH_PRESS_SPACING_M, Arm
from feeding_deployment.button_press.geometry import PlaneFitError
from feeding_deployment.button_press.panel_frame import (
    describe,
    from_panel,
    panel_frame,
    pose_path,
    rot_angle_deg,
    rot_from_panel,
)
from feeding_deployment.button_press.perception import FRESH_S, Perception, to_bgr_depth

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
# ---- dome-layout detector (far/coarse; see dome_pattern.py)
DOME_FRAMES = 7               # frames looked at per measurement
DOME_MIN_FITS = 5             # ... of which this many must fit all 5 domes
DOME_MAX_SPREAD_PX = 3.0      # and agree on the target pixel to within this
# ---- motion
MAX_PRESS_IN_M = 0.03         # refuse deeper presses: nothing stops the arm but the plan
PRESS_HOLD_S = 0.3
# Panel front face is near-vertical; a fitted normal with |z| above this is a bad fit.
MAX_PANEL_NORMAL_Z = 0.5
# Paths are chunked so no point moves more than RAY_STEP_MAX_M or turns more than
# ROT_STEP_MAX_DEG: how many joint degrees a centimetre costs depends on the posture (~5 deg/cm
# at the 2026-09-21 close standoff), and a merely LONG move must not trip the 10 deg joint gate.
# Press strokes use the finer arm.SMOOTH_PRESS_SPACING_M instead.
RAY_STEP_MAX_M = 0.01
ROT_STEP_MAX_DEG = 4.0
ALLOWED_SPEEDS = ("low", "medium")


class Run:
    """Perception node + gated arm + JSONL run log, shared by press_button and
    scripts/button_press/measure_prepress_offset.py."""

    def __init__(self, args):
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_f = open(self.log_dir / f"press_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "a")  # noqa: SIM115
        self.per = Perception(args.arm_frame, args.camera_frame)
        self.arm = Arm(args.execute)
        # The IK seed is anchored to a posture to stop the redundant arm sliding along its
        # self-motion manifold (see Arm.solve_translation). Set by preflight, refreshed by
        # reanchor_seed() after a big move.
        self.seed_posture = None

    def log(self, rec):
        rec = dict(rec, t=time.time())
        self._log_f.write(json.dumps(rec) + "\n")
        self._log_f.flush()

    def reanchor_seed(self, why):
        """Re-anchor the IK seed to the current joints. A posture anchor is only a good IK seed
        while the arm is near it: on 2026-09-21 a stale anchor 10.9 cm away gave 2-4 mm IK
        errors on 1-4 mm corrections, so each move injected more error than it removed."""
        print(f"  IK seed re-anchored to the current posture ({why})")
        if not self.args.execute:
            return   # nothing moved in a dry run, so the preflight anchor is still current
        self.seed_posture = self.arm.joints()

    def preflight(self, require_ready=True):
        print("\n== preflight ==")
        a = self.args
        st = self.arm.state()
        name = self.arm.arm_state_name()
        grip = float(st.get("gripper_pos", -1))
        speed = self.arm.ai.get_speed()
        print(f"  arm state     : {name}")
        print(f"  gripper_pos   : {grip:.3f}  ({'closed' if grip > 0.7 else 'NOT closed'})")
        print(f"  speed preset  : {speed}")
        problems = []
        # Hand-guiding (MANUALLY_CONTROLLED) is fine for measure_prepress_offset, which never moves.
        if require_ready and "SERVOING_READY" not in str(name) and not str(name).startswith("unknown"):
            problems.append(f"arm state {name}")
        if grip <= 0.7:
            problems.append("gripper must be CLOSED (this presses with the closed fingertips)")
        if str(speed).lower() not in ALLOWED_SPEEDS:
            problems.append(f"speed preset is {speed!r}, want {' or '.join(ALLOWED_SPEEDS)} "
                            "(scripts/session/arm_set_speed.py <preset>)")
        print("  waiting for camera_info ...")
        if self.per.wait("info", 5, None) is None:
            problems.append("no camera_info")
        try:
            R_bc = self.per.cam_rotation_in_base()
            print(f"  tf {a.arm_frame} <- {a.camera_frame}: ok (camera +z in base = {np.round(R_bc[:, 2], 3)})")
        except Abort as e:
            problems.append(str(e))

        start_joints = self.arm.joints()
        self.seed_posture = start_joints
        (self.log_dir / "start_joints.json").write_text(json.dumps({
            "joints": start_joints.tolist(), "ee_pos": list(map(float, st["ee_pos"])), "t": time.time()}))
        print(f"  start joints saved -> {self.log_dir / 'start_joints.json'}")
        self.log({"stage": "preflight", "arm_state": name, "gripper": grip, "speed": str(speed),
                  "problems": problems})
        if problems:
            for pr in problems:
                print(f"  PREFLIGHT FAIL: {pr}")
            raise Abort("preflight failed -- nothing commanded")
        print("  preflight OK")

    def pose_path(self, p0, R0, p1, R1, spacing=RAY_STEP_MAX_M):
        """Straight-line (xyz, R) points from one tool pose to another, chunked so that no
        point moves more than `spacing` or turns more than ROT_STEP_MAX_DEG."""
        return pose_path(p0, R0, p1, R1, spacing, ROT_STEP_MAX_DEG)

    def move(self, path, name):
        """Re-check `path` in the sim from the arm's actual pose, then send it as one smooth
        trajectory. Dry run: nothing to do (the pre-check in press_button was the plan)."""
        if not self.args.execute:
            return
        self.arm.precheck_poses(path, name, self.log, posture=self.seed_posture)
        self.arm.smooth_poses(path, name, self.log)

    def go_back(self, p0, R0, why):
        """After an abort mid-move: straight back to (p0, R0), pre-checked. Holds if that fails."""
        p_now, R_now = self.arm.ee_pose()
        if not self.args.execute or (np.linalg.norm(p_now - p0) < 0.005 and rot_angle_deg(R_now, R0) < 2.0):
            return
        print(f"  {why} -- going back to the start first")
        self.move(self.pose_path(p_now, R_now, p0, R0), "bail")

    def press(self, stroke_in, stroke_out, p_spot, R_spot):
        """From the pre-press spot: run the planned stroke in, hold, and the stroke back out,
        --presses times. If a stroke aborts, back out to the spot and hold there."""
        a = self.args
        print(f"\n== press x{a.presses}: {a.press_in*100:.1f} cm straight into the panel and back ==")
        for i in range(1, a.presses + 1):
            try:
                self.move(stroke_in, f"press {i} in")
            except Abort as e:
                self.go_back(p_spot, R_spot, f"press aborted ({e})")
                raise
            if a.execute:
                time.sleep(PRESS_HOLD_S)
            self.log({"stage": "press_normal", "press": i, "depth": a.press_in})
            self.move(stroke_out, f"press {i} out")


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

    Returns (origin = button xyz, R = panel axes), both in the arm base frame.
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
        bgr, d = to_bgr_depth(per.bridge, color, depth)
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
                    f"closer than {dome_pattern.MIN_RANGE_M*100:.0f} cm")
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


def check_move(p_from, R_from, p_to, R_to, what):
    dist, turn = float(np.linalg.norm(p_to - p_from)), rot_angle_deg(R_from, R_to)
    if dist > MAX_GOTO_M:
        raise Abort(f"{what} is {dist*100:.0f} cm away (> {MAX_GOTO_M*100:.0f}) -- start closer")
    if turn > MAX_GOTO_ROT_DEG:
        raise Abort(f"wrist would turn {turn:.0f} deg to the {what} (> {MAX_GOTO_ROT_DEG:.0f}) -- start facing the panel")


def press_button(run: Run, a, pub) -> int:
    arm = run.arm
    run.preflight()
    p0, R0 = arm.ee_pose()

    print(f"\n== detect: {a.target} ==")
    origin, R = measure_panel_frame_domes(run, a.target)
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
            run.move(stage, "to staging")
        except Abort as e:
            run.go_back(p0, R0, f"move aborted ({e})")
            raise
        time.sleep(STAGE_SETTLE_S)
        run.reanchor_seed("at the staging pose")
        print("\n== re-detect from the staging pose ==")
        far = origin
        origin, R = measure_panel_frame_domes(run, a.target)
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
    p_in = p_goal - a.press_in * n_out
    stroke_in = run.pose_path(p_goal, R_goal, p_in, R_goal, spacing=SMOOTH_PRESS_SPACING_M)
    stroke_out = run.pose_path(p_in, R_goal, p_goal, R_goal, spacing=SMOOTH_PRESS_SPACING_M)
    legs = [("to pre-press", go)]
    for i in range(a.presses):
        legs += [(f"press {i + 1} in", stroke_in), (f"press {i + 1} out", stroke_out)]
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
        run.move(go, "to pre-press")
    except Abort as e:
        run.go_back(p0, R0, f"move aborted ({e})")
        raise

    if a.presses:
        run.press(stroke_in, stroke_out, p_goal, R_goal)   # backs itself out on abort

    if a.no_return:
        print("\ndone (--no-return: holding at the pre-press spot).")
        return 0
    print("\n== back to start ==")
    p_now, R_now = arm.ee_pose()
    run.move(run.pose_path(p_now, R_now, p0, R0), "back to start")
    print("\ndone.")
    return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="start_30s", choices=dome_pattern.NAMES,
                    help="which button (default start_30s = START/+30SEC). The stored pre-press spot is "
                         "relative to the target button, so it is the same for every one.")
    ap.add_argument("--execute", action="store_true", help="actually move the arm (default: dry run)")
    ap.add_argument("--presses", type=int, default=1, help="0 = go to the spot and come back without pressing")
    ap.add_argument("--press-in", type=float, default=PRESS_IN_M,
                    help=f"push this far (m) into the panel from the spot (default {PRESS_IN_M}, max {MAX_PRESS_IN_M})")
    ap.add_argument("--no-return", action="store_true", help="stay at the pre-press spot at the end")
    ap.add_argument("--log-dir", default=str(Path.home() / "press_logs"))
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
    rclpy.init()
    try:
        run = Run(a)
        pub = run.per.create_publisher(PoseStamped, "/button_press/target_pose", 1)
        try:
            return press_button(run, a, pub)
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
