#!/usr/bin/env python3
"""One-command microwave button press: detect the panel, go to the pre-press spot, press, come back.

DRY RUN BY DEFAULT. Nothing moves without ``--execute``.

    ros2 run rammp_button_press press_button                  # plan only (START/+30SEC)
    ros2 run rammp_button_press press_button --execute        # do it
    ... --target timer_clock                                  # the Timer/Clock button instead

RAMMP port of feeding_deployment/button_press/press_button.py (button-task): same detection,
panel frame, pre-press constants and gates; the arm is kinova_gen3_node via ``RammpArm``, and
each leg is sent as the exact joint path the sim pre-check solved (execute_joint_trajectory).

Start with the panel in view, the camera at least 22 cm from it (dome_pattern.MIN_RANGE_M).
Then, in one call:

  1. preflight      driver up, not e-stopped, control free, tool frame consistent, gripper
                    closed, camera_info and tf up
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
every --execute. Needs the sheppy profile `arm-camera` (sheppy/README in this package).
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

from rammp_button_press import Abort, dome_pattern
from rammp_button_press.geometry import PlaneFitError
from rammp_button_press.panel_frame import (
    describe,
    from_panel,
    panel_frame,
    pose_path,
    rot_angle_deg,
    rot_from_panel,
    to_panel,
)
from rammp_button_press.perception import FRESH_S, Perception, to_bgr_depth, why_no_fit
from rammp_button_press.rammp_arm import TRACK_ABORT_M, RammpArm

# ---- the pre-press spot, in the panel frame (x right, y up, z out of the panel; origin = button)
# Kinova tool-frame position and orientation with the LEFT fingertip STANDOFF_M in front of the
# button. MEASURED 2026-10-03 on timer_clock, current fingers: panel frame from the dome layout
# at 30.9 cm, then the fingertip hand-placed 2 cm in front of the button (tool z 10 deg off
# straight-in). Re-measure with scripts/button_press/measure_prepress_offset.py if the fingers or
# camera mount change. (The 09-27 placeholder it replaced had the tool 4.2 cm below the button,
# taught from a biased ~40 cm view -- it would have missed.)
# RAMMP rig (Jetson, 2026-10-12): re-measured here. The rchi-cpu-5 values (-0.0049, -0.0114,
# -0.0131 / -0.04947, 0.99522, -0.08381, -0.00846) absorb that rig's hand-eye bias and put the
# fingertip ~7 cm INTO the panel on this one. Reference: the fingertip hand-placed at start_30s's
# pre-press spot, tool (0.7128, 0.0474, 0.1554) in base_link, tool z 23 deg off straight-in; an
# attended 1.0 cm push along the normal from there clicked start_30s. Expressed in the panel frame
# from the CLOSE re-detect at the staging pose (23 cm) -- that is the frame press_button plans the
# approach in. A first version expressed in the far (33.5 cm) frame put the run's spot 1.4 cm too
# low and 0.5 cm too far out (missed); the close view reads start_30s ~0.6 cm lower than the far
# one. Fingertip reads ~3 cm out of the detected plane here (detection short in depth); the offset
# absorbs it. Verified on start_30s only.
PREPRESS_EE_OFFSET_M = np.array([-0.0050, 0.0339, 0.0849])
PREPRESS_EE_QUAT_PANEL = np.array([0.12271, 0.97220, -0.19812, -0.02295])   # xyzw
STANDOFF_M = 0.02
# Fingertip = tool frame + FINGERTIP_PAST_TOOL_M along the tool z (robot.urdf finger_tip_joint
# 0.17955 m from the flange - tool_frame 0.120; the microwave's touch check used 0.062).
FINGERTIP_PAST_TOOL_M = 0.05955
MIN_PREPRESS_FINGERTIP_CLEAR_M = 0.01   # pre-press fingertip must be at least this far out of the fitted plane
PRESS_IN_M = 0.010            # set 2026-10-03 by the user (tests from the 2nd hand-placed spot: 1.2, 1.5 cm;
                              # 1.0 cm from the 1st spot, 3.6 mm further back, did not press)
# ---- refusals before anything moves
# No cap on the start -> staging / pre-press distance (removed 2026-10-12 at the user's request; was
# 0.40 m): every point of the straight line is still gated (reach, z band, joint limits, IK, joint
# step). Nothing checks collisions along it -- watch long moves.
MAX_GOTO_ROT_DEG = 45.0       # wrist turn on the way
# Where a leg may end relative to its target before the run holds. The approach and the press keep
# TRACK_ABORT_M (1 cm). Staging, bail and back-to-start only need to end near their target: the
# re-detect works from wherever staging actually stops. Long moves (50 cm, 2026-10-12) ended
# 0.5-1.05 cm off with the driver reporting success.
LOOSE_TRACK_ABORT_M = 0.05
# ---- far start: stage closer and re-detect before planning the approach
# From ~40 cm the detection is biased: on 2026-10-03 the same, unmoved button came out 1.2 cm
# lower from 39 cm than from 20 cm (normal z 0.154 vs 0.112) -- a button-width miss, and it put
# the goal under the Z floor. 2026-09-27 saw 2-3 cm at 40 cm. 17-22 cm is the validated range.
CLOSE_VIEW_M = 0.25           # camera -> button beyond this: stage first
# (No "jump check" comparing the staging re-detect with the far estimate: removed 2026-10-12. A
# wrong dome assignment shifts the button ~2 cm, but correct far estimates were off by 0.7-4.2 cm
# (front 34 cm .. side 50 cm), so the shift cannot tell them apart; it stopped a correct run on a
# rotated microwave. dome_pattern's blob size check and the frame agreement are the guards.)
STAGE_OUT_M = 0.10            # staging pose = pre-press pose this much further out: camera ~25.5 cm, in
                              # the 23-26 cm range every good re-detect came from. Was 0.08 (~23.5 cm):
                              # a far estimate that read the panel deep put the camera at 21.8 cm, under
                              # dome_pattern's MIN_RANGE_M 0.22, and the re-detect found nothing (2026-10-12)
RETRY_BACKOFF_M = 0.03        # re-detect at staging failed: back straight out this far and try once more
# (no settle sleep: the re-detect only takes frames captured after the arm stopped -- see `after`)
# ---- dome-layout detector (far/coarse; see dome_pattern.py)
DOME_FRAMES = 7               # frames looked at per measurement
DOME_MIN_FITS = 5             # ... of which this many must fit all 5 domes
DOME_MAX_SPREAD_PX = 3.0      # and agree on the target pixel to within this, or within
DOME_SPREAD_FRAC = 0.3        # this x the dome spacing if larger. A dome's highlight can split and
                              # flicker between two spots ~0.25 spacing apart (rotated microwave,
                              # 2026-10-12: 12-13 px of a 47 px spacing, still arm); a wrong dome
                              # assignment is >= 1 spacing away, so it is still refused
# ---- motion
MAX_PRESS_IN_M = 0.03         # refuse deeper presses: nothing stops the arm but the plan
# (no hold sleep at the bottom of a press: smooth_poses already waits there until the tool is still)
# Panel front face is near-vertical; a fitted normal with |z| above this is a bad fit.
MAX_PANEL_NORMAL_Z = 0.5
# Paths are chunked so no point moves more than RAY_STEP_MAX_M or turns more than
# ROT_STEP_MAX_DEG: how many joint degrees a centimetre costs depends on the posture (~5 deg/cm
# at the 2026-09-21 close standoff), and a merely LONG move must not trip the 10 deg joint gate.
# Press strokes use the finer SMOOTH_PRESS_SPACING_M below instead.
RAY_STEP_MAX_M = 0.01
ROT_STEP_MAX_DEG = 4.0
# Press strokes use finer spacing (was arm.SMOOTH_PRESS_SPACING_M, for Kortex's final-waypoint taper).
# Under RAMMP the driver interpolates linearly in JOINT space between the pre-checked points, so
# denser points keep the 1 cm stroke closer to the straight line along the normal.
SMOOTH_PRESS_SPACING_M = 0.003


class Run:
    """Perception node + gated arm + JSONL run log, shared by press_button and
    scripts/button_press/measure_prepress_offset.py."""

    def __init__(self, args):
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_f = None
        self.start_log()
        self.per = Perception(args.arm_frame, args.camera_frame, args.camera_ns)
        self.arm = RammpArm(args.execute, acquire_control=args.acquire_control)
        # The IK seed is anchored to a posture to stop the redundant arm sliding along its
        # self-motion manifold (see Arm.solve_translation). Set by preflight, refreshed by
        # reanchor_seed() after a big move.
        self.seed_posture = None

    def start_log(self):
        """A new JSONL run log (press_server: one per goal)."""
        if self._log_f is not None:
            self._log_f.close()
        self.log_path = str(self.log_dir / f"press_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
        self._log_f = open(self.log_path, "a")  # noqa: SIM115

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
        grip = float(st.get("gripper_pos", -1))
        arm_problems, info = self.arm.status()
        for line in info:
            print(f"  {line}")
        print(f"  gripper_pos    : {grip:.3f}  ({'closed' if grip > 0.7 else 'NOT closed'})"
              f"{'' if self.arm.gripper_present() else '  -- gripper NOT present'}")
        print(f"  tool xyz       : {np.round(st['ee_pos'][:3], 3)}  joints (deg) {np.round(np.degrees(st['position']), 1)}")
        # Speed: no Kortex preset under RAMMP -- RammpArm times every leg (CART_SPEED / JOINT_SPEED).
        # measure_prepress_offset never moves, so it only needs the read-side checks.
        problems = list(arm_problems) if require_ready else [
            pr for pr in arm_problems if pr.startswith(("no ", "tool frame", "/joint_states", "/ee_state"))]
        if grip <= 0.7:
            problems.append("gripper must be CLOSED (this presses with the closed fingertips)")
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
        self.log({"stage": "preflight", "status": info, "gripper": grip, "problems": problems})
        if problems:
            for pr in problems:
                print(f"  PREFLIGHT FAIL: {pr}")
            raise Abort("preflight failed -- nothing commanded")
        if self.args.execute and require_ready:
            self.arm.take_control()   # no-op unless arbitration is enforced
        print("  preflight OK")

    def pose_path(self, p0, R0, p1, R1, spacing=RAY_STEP_MAX_M):
        """Straight-line (xyz, R) points from one tool pose to another, chunked so that no
        point moves more than `spacing` or turns more than ROT_STEP_MAX_DEG."""
        return pose_path(p0, R0, p1, R1, spacing, ROT_STEP_MAX_DEG)

    def move(self, path, name, track_abort_m=TRACK_ABORT_M):
        """Re-check `path` in the sim from the arm's actual pose, then send it as one smooth
        trajectory. Dry run: nothing to do (the pre-check in press_button was the plan)."""
        if not self.args.execute:
            return
        p_now, R_now = self.arm.ee_pose()
        start = (self.arm.joints(), p_now, R_now)
        _, _, _, knots = self.arm.precheck_poses(path, name, self.log, posture=self.seed_posture, start=start)
        self.arm.smooth_poses(path, knots, start, name, self.log, track_abort_m=track_abort_m)

    def go_back(self, p0, R0, why):
        """After an abort mid-move: straight back to (p0, R0), pre-checked. Holds if that fails."""
        p_now, R_now = self.arm.ee_pose()
        if not self.args.execute or (np.linalg.norm(p_now - p0) < 0.005 and rot_angle_deg(R_now, R0) < 2.0):
            return
        print(f"  {why} -- going back to the start first")
        self.long_move(p0, R0, "bail")

    def long_move(self, p, Rm, name):
        """A move that need not be straight (staging, back to start, bail): RAMMP's cuRobo plans it
        and RammpArm gates/executes that exact plan (--long-moves curobo, the default), or the
        straight sim-gated line (--long-moves straight). Ends within LOOSE_TRACK_ABORT_M."""
        if self.args.long_moves == "curobo":
            return self.arm.curobo_move(p, Rm, name, self.log, track_abort_m=LOOSE_TRACK_ABORT_M)
        p_now, R_now = self.arm.ee_pose()
        self.move(self.pose_path(p_now, R_now, p, Rm), name, track_abort_m=LOOSE_TRACK_ABORT_M)

    def press(self, stroke_in, stroke_out, p_spot, R_spot, n_out):
        """From the pre-press spot: run the planned stroke in, hold, and the stroke back out,
        --presses times. If a stroke aborts, back out to the spot and hold there.
        --press-mode impedance streams the stroke to the driver's compliant controller instead
        (RammpArm.press_impedance); everything before and after stays in position mode."""
        a = self.args
        print(f"\n== press x{a.presses}: {a.press_in*100:.1f} cm straight into the panel and back "
              f"({a.press_mode}) ==")
        for i in range(1, a.presses + 1):
            if a.press_mode == "impedance":
                try:
                    self.arm.press_impedance(p_spot, n_out, a.press_in, f"press {i} (impedance)", self.log)
                except Abort as e:
                    self.go_back(p_spot, R_spot, f"press aborted ({e})")
                    raise
                self.log({"stage": "press_normal", "press": i, "depth": a.press_in, "mode": "impedance"})
                continue
            try:
                self.move(stroke_in, f"press {i} in")
            except Abort as e:
                self.go_back(p_spot, R_spot, f"press aborted ({e})")
                raise
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


def stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec


def measure_panel_frame_domes(run: Run, target: str, after: int | None = None):
    """Panel frame from the dome-layout detector: median over DOME_FRAMES frames.

    `after` (ns, ROS clock): only frames captured after it count -- pass the moment the arm
    stopped, so no frame taken while it moved gets in (camera stamps are this machine's clock,
    ~85 ms behind arrival). Returns (origin = button xyz, R = panel axes), arm base frame.
    """
    per = run.per
    if target not in dome_pattern.NAMES:
        raise Abort(f"dome detector knows {dome_pattern.NAMES}, not {target!r}")
    fx, fy, cx, cy = per.intrinsics()
    fits, seen, last, t0, reasons, prev = [], 0, None, time.monotonic(), [], None
    while seen < DOME_FRAMES and time.monotonic() - t0 < 6.0:
        color, depth = per.get("color", FRESH_S), per.get("depth", FRESH_S)
        if color is None or depth is None or color is last or (
                after is not None and min(stamp_ns(color), stamp_ns(depth)) < after):
            time.sleep(0.03)    # wait for the next frame
            continue
        last = color
        seen += 1
        bgr, d = to_bgr_depth(per.bridge, color, depth)
        fit = dome_pattern.detect(bgr, d, fx, hint=prev)
        if fit is None and prev is not None:
            fit = dome_pattern.detect(bgr, d, fx)       # the hint lost it: search the whole frame
        prev = fit or prev
        if fit is None:
            reasons.append(why_no_fit(bgr, d, fx))
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
    tol = max(DOME_MAX_SPREAD_PX, DOME_SPREAD_FRAC * float(np.median([f[3].s_px for f in fits]))) if fits else 0.0
    keep = [f for f, p in zip(fits, pxs) if np.linalg.norm(p - px) <= tol] if fits else []
    print(f"  dome layout: {len(fits)}/{seen} frames fit all 5 domes, {len(keep)} agree within {tol:.0f} px")
    run.log({"dome_frames": {"target_px": pxs.tolist(), "seen": seen, "agree": len(keep)}})
    if not fits:
        why = max(set(reasons), key=reasons.count) if reasons else "no frames"
        raise Abort(f"dome detector: nothing in {seen} frames ({why}) -- "
                    + ("move the camera back" if why.startswith("TOO CLOSE") else
                       "move the camera closer" if why.startswith("TOO FAR") else "is the panel in view?"))
    if len(keep) < DOME_MIN_FITS:
        raise Abort(f"dome detector: only {len(keep)}/{seen} frames agree on the {target} pixel (need {DOME_MIN_FITS}; "
                    f"within {tol:.0f} px; pixels {np.round(pxs).astype(int).tolist()}) -- the camera or arm moving, "
                    "or the detection is unstable from this view")
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


def fingertip_out_of_panel(p_tool, R_tool, origin, R) -> float:
    """Signed distance (m) of the fingertip in front of the fitted panel plane (negative = inside)."""
    tip = np.asarray(p_tool) + np.asarray(R_tool)[:, 2] * FINGERTIP_PAST_TOOL_M
    return float(to_panel(tip, origin, R)[2])


def check_fingertip(p_tool, R_tool, origin, R, what):
    out = fingertip_out_of_panel(p_tool, R_tool, origin, R)
    print(f"  fingertip at the {what}: {out*100:+.1f} cm out of the fitted panel plane")
    if out < MIN_PREPRESS_FINGERTIP_CLEAR_M:
        raise Abort(f"the stored pre-press offset puts the fingertip {-out*100:.1f} cm INTO the panel at the {what} "
                    f"(need >= {MIN_PREPRESS_FINGERTIP_CLEAR_M*100:.0f} cm in front) -- PREPRESS_EE_OFFSET_M is from "
                    "another rig/calibration; re-measure it here with measure_prepress_offset")


def check_move(p_from, R_from, p_to, R_to, what):
    turn = rot_angle_deg(R_from, R_to)
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
        check_fingertip(*prepress_pose(origin, R), origin, R, "pre-press spot (far estimate)")
        p_st, R_st = prepress_pose(origin, R, out=STAGE_OUT_M)
        print(f"  camera is {cam_d*100:.0f} cm from the button (> {CLOSE_VIEW_M*100:.0f}): staging closer first")
        check_move(p0, R0, p_st, R_st, "staging pose")
        if a.long_moves == "curobo":
            if not a.execute:          # dry run: plan + gates only (executing re-plans from the real joints)
                arm.curobo_move(p_st, R_st, "to staging", run.log)
        else:
            stage = run.pose_path(p0, R0, p_st, R_st)
            err, jump, _, _ = arm.precheck_poses(stage, "to staging", run.log, posture=run.seed_posture)
            print(f"  to staging       {len(stage):3d} pts  IK <= {err*1000:.1f} mm  joint step <= {jump:.1f} deg  ok")
        if not a.execute:
            print("\ndry run -- the staging leg passed the gates. The approach and press are planned only after "
                  "re-detecting at the staging pose, so they need --execute (or start within "
                  f"{CLOSE_VIEW_M*100:.0f} cm to dry-run the whole thing)")
            return 0
        print("\n== go to the staging pose ==")
        try:
            run.long_move(p_st, R_st, "to staging")
        except Abort as e:
            run.go_back(p0, R0, f"move aborted ({e})")
            raise
        stopped = run.per.get_clock().now().nanoseconds     # smooth_poses returned: the arm is still
        run.reanchor_seed("at the staging pose")
        print("\n== re-detect from the staging pose ==")
        far = origin
        try:
            origin, R = measure_panel_frame_domes(run, a.target, after=stopped)
        except Abort as e:
            # Once: back straight out along the (far) panel normal and look again -- the far
            # estimate can put staging closer than planned (21.8 cm, 2026-10-12).
            print(f"  re-detect failed ({e}); backing out {RETRY_BACKOFF_M*100:.0f} cm and trying once more")
            p_s, R_s = arm.ee_pose()
            run.move(run.pose_path(p_s, R_s, p_s + RETRY_BACKOFF_M * R[:, 2], R_s), "back out to re-detect",
                     track_abort_m=LOOSE_TRACK_ABORT_M)
            stopped = run.per.get_clock().now().nanoseconds
            origin, R = measure_panel_frame_domes(run, a.target, after=stopped)
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
    check_fingertip(p_goal, R_goal, origin, R, "pre-press spot")
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
    if not a.no_return and a.long_moves == "straight":
        legs.append(("back to start", run.pose_path(p_goal, R_goal, p0, R0)))
    state = None
    for name, path in legs:
        err, jump, state, _ = arm.precheck_poses(path, name, run.log, posture=run.seed_posture, start=state)
        print(f"  {name:16s} {len(path):3d} pts  IK <= {err*1000:.1f} mm  joint step <= {jump:.1f} deg  ok")
    if not a.no_return and a.long_moves == "curobo":
        # Plan-only check that cuRobo can bring the arm back from the pre-press spot; the move
        # itself is re-planned from the real joints when it runs.
        t_b, q_b = arm.plan_curobo(p0, R0, state[0], "back to start (check)")
        e_b, s_b = arm.gate_joint_path(q_b, p0, R0, "back to start (check)", run.log)
        print(f"  back to start    cuRobo {len(q_b):3d} pts  end {e_b*1000:.1f} mm off  joint step <= {s_b:.1f} deg  ok")
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
        run.press(stroke_in, stroke_out, p_goal, R_goal, n_out)   # backs itself out on abort

    if a.no_return:
        print("\ndone (--no-return: holding at the pre-press spot).")
        return 0
    print("\n== back to start ==")
    run.long_move(p0, R0, "back to start")
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
    ap.add_argument("--long-moves", choices=("curobo", "straight"), default="curobo",
                    help="staging / back to start / bail: planned by RAMMP's cuRobo (needs the sheppy `planner` "
                         "node; its world has no microwave or wheelchair) or the old straight sim-gated line")
    ap.add_argument("--press-mode", choices=("position", "impedance"), default="position",
                    help="the press stroke only: position (stiff, sim-gated joint path) or impedance (the "
                         "driver's compliant ee_pose_impedance; holds first, refuses if the arm sags)")
    ap.add_argument("--log-dir", default=str(Path.home() / "press_logs"))
    ap.add_argument("--arm-frame", default="base_link", help="kinova_gen3_node's base frame (= /ee_state's)")
    ap.add_argument("--camera-frame", default="camera_color_optical_frame")
    ap.add_argument("--camera-ns", default="/wrist_camera", help="RealSense topic namespace")
    ap.add_argument("--acquire-control", action="store_true",
                    help="under enforced arbitration, acquire a token (only if nobody holds the arm)")
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
    run = None
    try:
        try:
            run = Run(a)
            pub = run.per.create_publisher(PoseStamped, "/button_press/target_pose", 1)
            return press_button(run, a, pub)
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
    finally:
        if run is not None:
            run.arm.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
