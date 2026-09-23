#!/usr/bin/env python3
"""Autonomous microwave button press: two-dots visual servo + force stop.

DRY RUN BY DEFAULT. Nothing moves without ``--execute``. With ``--execute`` the arm
moves in small, individually gated joint steps and stops on the first surprise.

How it works (pure translation -- the wrist orientation is never changed):

  The wrist camera is rigid to the gripper, so the LEFT fingertip is always at the same
  pixel (``/button_detector/claw_pixel``). If the button's pixel
  (``/button_detector/button_pixel``) sits on that pixel, the button lies on the
  fingertip's line of sight. Translating the wrist along that ray keeps it there until
  the fingertip touches the button. So:

    stage 1  lateral servo    move in the camera's image plane until |button - claw| < PX_TOL
    stage 2  approach         step along the fingertip ray; after EVERY step, with the arm at
                              rest, read the tool force from the press detector; contact when
                              |dF| > CONTACT_N; abort if > FORCE_ABORT_N or the travel cap
    stage 3  press            one more PRESS_TRAVEL step, hold, retract PRESS_RETRACT
    stage 4  retract          back to the stage-1 standoff (``--home``: joint-move to start)

  Lateral error closes on pixels; depth error closes on force. The only calibration used
  is the ROTATION arm_base_link <- camera (tf2), to turn camera-frame directions into
  base-frame directions -- a few degrees of error there just costs a servo iteration.

What has to be running (docs/button_press_runbook.md has the exact commands):
  arm_server.py + stub_base_server.py + bulldog_bypass.py, speed set LOW
  robot_state_publisher + joint_state_bridge + calibration_tf (tf chain; verify with
      ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame)
  realsense2_camera with align_depth
  feeding_deployment.button_press.detector_node  (-p target_button:=<name>)
  feeding_deployment.button_press.press_detector --publish   (baselined with the arm PARKED)
  (launch/ros2/button_press_bringup.launch.py starts the last two)

The travel cap (stage 2) needs ``--tip-dist``: the straight-line distance in metres from
the camera lens to the LEFT fingertip. Measure it with a ruler. Over-estimating it makes
the cap conservative (stops short); under-estimating relies on the force stop. The cap is
    L_max = (distance along the fingertip ray from the camera to the panel plane)
            - tip_dist + PLANE_OVERSHOOT_M
and ``--cap-override`` lowers it further for the first approach tests. On this rig
tip_dist = 0.154 (measured 2026-09-21 from the first contact).

Stopping it: the physical e-stop is the only stop proven on this rig. Killing
bulldog_bypass.py e-stops the arm within ~1 s. Ctrl-C here stops the NEXT step from being
sent -- the in-flight step (<= 1 cm, <= MAX_JOINT_STEP_DEG on any joint) completes. On any
abort the arm is left where it is and the way back is printed; nothing auto-retracts.

Usage ladder (one invocation per rung, a human at the e-stop for every --execute):
    P="python3 -u -m feeding_deployment.button_press.autonomous_press --target timer_clock"
    $P                                                   # dry run, all stages planned
    $P --execute --stage 1                               # lateral servo only
    $P --execute --stage 2 --tip-dist 0.154 --cap-override 0.05
    $P --execute --stage 2 --tip-dist 0.154              # until contact
    $P --execute --tip-dist 0.154 --presses 1            # the real thing
The command that pressed timer_clock unaided on 2026-09-21/22 (from a ~17 cm standoff):
    $P --execute --tip-dist 0.154 --presses 1 --fine-step 0.002 --press-travel 0.001 --cap-override 0.03
Run with the *prepend* form PYTHONPATH=$PWD/src:$PYTHONPATH (PYTHONPATH=src drops rclpy).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import numpy as np
import rclpy

from feeding_deployment.button_press import Abort
from feeding_deployment.button_press.arm import CONVERGE_TOL_DEG, Arm, wait_converged
from feeding_deployment.button_press.contact import (
    COARSE_ABORT_N,
    CONFIRM_RISE_N,
    CONTACT_ARM_MARGIN_M,
    CONTACT_N,
    CONTACT_WINDOW,
    FORCE_ABORT_N,
    JUMP_N,
    ApproachContactMonitor,
)
from feeding_deployment.button_press.geometry import (
    lateral_correction_cam,
    max_joint_delta_deg,
    ray_plane_distance,
)
from feeding_deployment.button_press.perception import (
    FORCE_SETTLE_S,
    FORCE_STALE_S,
    FRESH_S,
    LOCK_HOLD_S,
    MIN_INLIERS,
    MIN_INLIERS_TRACK,
    Perception,
)
from feeding_deployment.button_press.press_detector import find_running_press_detector
from feeding_deployment.control.robot_controller.command_interface import JointCommand

# ---- servo / approach ------------------------------------------------------------------
PX_TOL = 4.0                 # button pixel within this of the claw pixel counts as aligned
SERVO_MAX_ITERS = 6          # from 32 cm the first correction is capped, so allow more rounds
SERVO_MAX_STEP_M = 0.03      # a single lateral correction is capped here
APPROACH_STEP_M = 0.010
FINE_STEP_M = 0.003
FINE_ZONE_M = 0.04           # switch to FINE_STEP_M this close to the travel cap
# Travel cap = (camera->plane along the ray) - tip_dist + PLANE_OVERSHOOT_M. The force stop
# is the primary terminator; the cap only guards against a gross depth/tip error, so it is
# allowed PAST the nominal plane (the dome sits ~5 mm proud of it and the depth has a few
# mm of bias this close). 2026-09-21: tip_dist measured 0.154 m on this rig (plane 17.0 cm
# along the ray, contact at 1.6 cm), so the cap from a 30 cm standoff is ~16 cm.
PLANE_OVERSHOOT_M = 0.015
RESERVO_EVERY_M = 0.03       # re-check lateral alignment this often during the approach
RESERVO_MAX_STEP_M = 0.01
# ---- far phase -------------------------------------------------------------------------
# The claw pixel is near the bottom of the frame, so putting the button ON it drags the
# panel behind the finger. From far away the panel is small and disappears entirely
# (2026-09-21, 32 cm standoff: locked -> "quad not convex"). So beyond CLOSE_STANDOFF_M the
# button is held HOLD_DY_PX above the claw pixel and the wrist advances along THAT ray; the
# proven servo-onto-claw only starts once the panel is close enough to stay visible.
HOLD_DY_PX = 110.0
CLOSE_STANDOFF_M = 0.20      # camera->panel along the hold ray at which the close phase starts
# 1.5 cm, not 2 cm: at the 2026-09-21 far-standoff posture the arm needs ~4.6 deg of joint
# motion per cm, so a 2 cm step plans 9.3 deg against the 10 deg MAX_JOINT_STEP_DEG gate --
# 0.7 deg of headroom, and one noisy solve aborts the run. 1.5 cm plans ~7 deg for the cost
# of ~2 extra steps. Shrink the step rather than open the gate.
FAR_STEP_M = 0.015
FAR_RESERVO_EVERY_M = 0.04
# ---- force -----------------------------------------------------------------------------
# The contact-rule constants (CONTACT_*, JUMP_N, CONFIRM_RISE_N, COARSE_ABORT_N,
# FORCE_ABORT_N) and the measurements behind them live in contact.py, next to the rule.
FORCE_FREE_N = 1.0           # preflight: the tool must be this free
# Settle before re-taking the baseline. 2.8 s was not enough after a multi-cm move:
# on 2026-09-22 the only jog leg baselined straight after a 1.5 cm retract wandered
# 2-7.5 N, while every later leg (same 2 mm steps) stayed inside 0.6-2.8 N.
REBASELINE_SETTLE_S = 5.0
# ---- press -----------------------------------------------------------------------------
PRESS_TRAVEL_M = 0.003
PRESS_HOLD_S = 0.3
PRESS_RETRACT_M = 0.02
# Cap on a single commanded move ALONG A RAY. The joint-jump gate (MAX_JOINT_STEP_DEG)
# is in joint space, but these moves are specified in metres, and how many degrees a
# centimetre costs depends entirely on the posture: at the 2026-09-21 close standoff it
# was ~5 deg/cm, so the 2 cm press retract planned 10.1 deg and the gate refused AFTER
# the button had been pressed. Chunk every ray move to this size and the gate stops
# being reachable by a move that is merely long rather than wrong.
RAY_STEP_MAX_M = 0.01


# =============================================================================================
# The stages
# =============================================================================================
class Run:
    def __init__(self, args):
        self.args = args
        self.log_dir = Path(args.log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self._log_f = open(self.log_dir / f"press_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "a")  # noqa: SIM115
        self.per = Perception(args.ns, args.press_ns, args.arm_frame, args.camera_frame)
        self.arm = Arm(args.execute)
        self.R_bc = None          # arm_base <- camera rotation
        self.ray_cam = None       # fingertip ray, camera frame, unit
        self.ray_base = None
        self.s_panel = None       # distance camera -> panel plane along the ray
        self.z_panel = None       # median camera-z of the panel
        self.travelled = 0.0      # along the fingertip ray, since the close standoff
        self.far_travelled = 0.0  # along the hold ray (far phase)
        self.far_ray_base = None
        self.start_joints = None
        # The IK seed is anchored to a posture to stop the redundant arm sliding along its
        # self-motion manifold (see Arm.solve_translation). It is NOT start_joints: --home and
        # the printed retreat path need those to stay the run's true origin. The anchor is
        # re-set at each phase boundary by reanchor_seed() -- see that method for why.
        self.seed_posture = None

    def step(self, d_base, name):
        """All EE moves go through here so every IK is seeded from the anchored posture."""
        return self.arm.step(d_base, name, self.log, posture=self.seed_posture)

    def move_along(self, unit, dist, name, far=False):
        """Translate `dist` metres along `unit`, in chunks of at most RAY_STEP_MAX_M.

        Keeps the travel bookkeeping in step with what has actually executed: if a chunk
        aborts, the chunks already done are already subtracted, so the printed "way back"
        is right. `unit` points in the direction of travel; `dist` is always positive.
        """
        remaining = float(dist)
        signed = -1.0 if np.dot(unit, self.far_ray_base if far else self.ray_base) < 0 else 1.0
        k = 0
        while remaining > 1e-4:
            stp = min(RAY_STEP_MAX_M, remaining)
            k += 1
            self.step(unit * stp, f"{name} {k} ({signed*stp*100:+.1f})")
            remaining -= stp
            if far:
                self.far_travelled += signed * stp
            else:
                self.travelled += signed * stp

    def reanchor_seed(self, why):
        """Re-anchor the IK seed to the current joints.

        A posture anchor is only a good IK seed while the arm is near it. On 2026-09-21 the
        far phase travelled 10.9 cm from the preflight posture; the stage-1 servo that followed
        then solved every correction from that stale seed and got ik_err 2.2-3.6 mm -- under
        MAX_IK_ERR_M (5 mm) so it was accepted, but the corrections themselves were only 1-4 mm,
        so each move injected more error than it removed. The button pixel oscillated
        (+2, -4, +12, +7, -4, -11 px) and never reached PX_TOL, aborting the run. Re-anchoring
        at each phase boundary keeps the anti-drift property within a phase while keeping the
        seed close enough to stay sub-millimetre.
        """
        print(f"  IK seed re-anchored to the current posture ({why})")
        if not self.args.execute:
            return   # nothing moved in a dry run, so the preflight anchor is still current
        self.seed_posture = self.arm.joints()

    def log(self, rec):
        rec = dict(rec, t=time.time())
        self._log_f.write(json.dumps(rec) + "\n")
        self._log_f.flush()

    def rebaseline_force(self, why):
        """Ask the running press detector (SIGUSR1) to re-take its rest baseline here.

        Kinova's compensated wrench shifts by several N with joint configuration (2026-09-21:
        +3.3 N for one 2 cm / 8.6 deg step, +2.6 N for a 2 cm servo move), and the detector
        stops adapting above 4 N, so after any sizeable move the approach would otherwise
        start from a biased, frozen reading. Arm must be at rest; nothing here moves it.
        """
        if not self.args.execute:
            return
        pids = find_running_press_detector()
        if not pids:
            raise Abort("press detector process not found for re-baseline (no live pidfile, no "
                        "matching process) -- HOLDING HERE")
        print(f"  re-baselining the press detector ({why}); arm still for {REBASELINE_SETTLE_S:.1f} s ...")
        for pid in pids:
            os.kill(pid, signal.SIGUSR1)
        time.sleep(REBASELINE_SETTLE_S)
        f = self.force_now()
        print(f"  |dF| after re-baseline: {f:.2f} N")
        if f > FORCE_FREE_N:
            raise Abort(f"|dF| still {f:.2f} N after re-baseline -- something is loading the tool. HOLDING HERE")
        self.log({"rebaseline": why, "force_after": f})

    def force_now(self):
        f = self.per.force_at_rest()
        if math.isnan(f):
            raise Abort("press detector feed is stale/dead -- HOLDING HERE")
        return f

    # ---- stage 0 ------------------------------------------------------------------------------
    def preflight(self, require_lock=True, require_free=True, min_inliers=MIN_INLIERS):
        print("\n== stage 0: preflight ==")
        a = self.args
        st = self.arm.state()
        name = self.arm.arm_state_name()
        grip = float(st.get("gripper_pos", -1))
        speed = self.arm.ai.get_speed()
        print(f"  arm state     : {name}")
        print(f"  gripper_pos   : {grip:.3f}  ({'closed' if grip > 0.7 else 'NOT closed'})")
        print(f"  speed preset  : {speed}")
        problems = []
        if "SERVOING_READY" not in str(name) and not str(name).startswith("unknown"):
            problems.append(f"arm state {name}")
        if grip <= 0.7:
            problems.append("gripper must be CLOSED (this presses with the closed fingertips)")
        if str(speed).lower() != "low":
            problems.append(f"speed preset is {speed!r}, want 'low' (scripts/session/arm_set_speed.py low)")

        print("  waiting for camera_info / status / pixels ...")
        if self.per.wait("info", 5, None) is None:
            problems.append("no camera_info")
        # The node must hold a lock on the requested target for LOCK_HOLD_S.
        t0 = time.monotonic()
        held = 0.0
        last = ""
        while time.monotonic() - t0 < LOCK_HOLD_S + 6:
            ok, last = self.per.locked(min_inliers)
            if ok and (a.target is None or self.per.lock_target() == a.target):
                held += 0.1
                if held >= LOCK_HOLD_S:
                    break
            else:
                held = 0.0
            time.sleep(0.1)
        print(f"  button node   : {last}")
        if held < LOCK_HOLD_S and require_lock:
            problems.append(f"button node not locked on {a.target or 'any target'} for {LOCK_HOLD_S}s")
        if self.per.get("claw_px", FRESH_S) is None:
            problems.append("no claw_pixel")

        fa = self.per.age("force")
        f = self.per.force_at_rest()
        pressed = self.per.get("pressed", FORCE_STALE_S)
        print(f"  press detector: age {fa if fa is None else round(fa, 2)} s  |dF| {f:.2f} N  pressed={pressed}")
        if fa is None or fa > FORCE_STALE_S:
            problems.append("press detector not publishing (python3 -u -m feeding_deployment.button_press.press_detector --publish)")
        elif require_free and f > FORCE_FREE_N:
            problems.append(f"tool force {f:.2f} N > {FORCE_FREE_N} -- arm touching something / bad baseline")
        elif require_free and pressed:
            problems.append("press detector reports PRESSED")

        try:
            self.R_bc = self.per.cam_rotation_in_base()
            print(f"  tf {a.arm_frame} <- {a.camera_frame}: ok (camera +z in base = {np.round(self.R_bc[:, 2], 3)})")
        except Abort as e:
            problems.append(str(e))

        self.start_joints = self.arm.joints()
        self.seed_posture = self.start_joints
        (self.log_dir / "start_joints.json").write_text(json.dumps({
            "joints": self.start_joints.tolist(), "ee_pos": list(map(float, st["ee_pos"])), "t": time.time()}))
        print(f"  start joints saved -> {self.log_dir / 'start_joints.json'}")
        self.log({"stage": 0, "arm_state": name, "gripper": grip, "speed": str(speed), "status": last,
                  "force": f, "problems": problems})
        if problems:
            for pr in problems:
                print(f"  PREFLIGHT FAIL: {pr}")
            raise Abort("preflight failed -- nothing commanded")
        print("  preflight OK")

    # ---- geometry shared by stages 1/2 --------------------------------------------------------
    def hold_px(self):
        claw = self.per.get("claw_px", FRESH_S)
        if claw is None:
            raise Abort("no claw_pixel")
        return claw + np.array([0.0, -HOLD_DY_PX])

    def measure_panel(self, px=None, label="fingertip ray"):
        """Plane + the ray through `px` (default: the claw pixel). Sets ray_*, s_panel, z_panel."""
        n, d, z_med, npts = self.per.panel_plane()
        if px is None:
            px = self.per.get("claw_px", FRESH_S)
            if px is None:
                raise Abort("no claw_pixel")
        self.ray_cam = self.per.ray(px)
        self.ray_base = self.R_bc @ self.ray_cam
        s_panel, denom = ray_plane_distance(n, d, self.ray_cam)
        if abs(denom) < 0.3:
            raise Abort(f"{label} nearly parallel to the panel (n.r={denom:.2f})")
        self.s_panel = s_panel
        self.z_panel = z_med
        print(f"  panel plane   : normal(cam) {np.round(n, 3)}  median depth {z_med*100:.1f} cm  ({npts} px)")
        print(f"  {label:14s}: cam {np.round(self.ray_cam, 3)}  base {np.round(self.ray_base, 3)}"
              f"  camera->panel along ray {self.s_panel*100:.1f} cm")
        self.log({"panel_normal_cam": n.tolist(), "panel_d": d, "z_panel": z_med, "n_px": npts,
                  "ray_px": np.asarray(px).tolist(), "ray_cam": self.ray_cam.tolist(),
                  "ray_base": self.ray_base.tolist(), "s_panel": self.s_panel})

    def pixel_error(self, after=None, target=None):
        """button_px - target (default target: the claw pixel)."""
        b = self.per.median_after("button_px", after if after is not None else time.monotonic())
        c = self.per.get("claw_px", FRESH_S) if target is None else target
        if b is None or c is None:
            return None
        return b - c

    def lateral_correction(self, e_px, z, cap):
        """Camera-frame in-plane translation that moves the button pixel onto the claw pixel.

        Translating the camera +x makes a static point's u decrease, so the correction
        is +e (button right of claw -> move right). Scaled by the button's depth z.
        """
        fx, fy, _, _ = self.per.intrinsics()
        d_cam = lateral_correction_cam(e_px, z, fx, fy, cap)
        return d_cam, self.R_bc @ d_cam

    # ---- far phase (before stage 1 when the panel is far) ------------------------------------
    def far_approach(self):
        """Hold the button HOLD_DY_PX above the claw pixel and close in along that ray until
        the panel is CLOSE_STANDOFF_M away. No contact is possible here (the fingertip is well
        short of the panel), so any force jump is an abort, not a contact."""
        print(f"\n== far phase: hold the button {HOLD_DY_PX:.0f} px above the claw, close to {CLOSE_STANDOFF_M*100:.0f} cm ==")
        self.far_travelled = 0.0
        since = 0.0
        f_prev = None
        # servo onto the hold pixel first
        moved_at = None
        for it in range(1, SERVO_MAX_ITERS + 1):
            e = self.pixel_error(after=moved_at, target=self.hold_px())
            if e is None:
                raise Abort("button node not locking in the far phase -- HOLDING HERE")
            err = float(np.linalg.norm(e))
            print(f"  hold-servo {it}: button - hold = ({e[0]:+.1f}, {e[1]:+.1f}) px  |e| {err:.1f}")
            if err < PX_TOL:
                break
            d_cam, d_base = self.lateral_correction(e, self.z_panel, SERVO_MAX_STEP_M)
            self.step(d_base, f"hold-servo {it}")
            if not self.args.execute:
                break
            moved_at = time.monotonic() + 0.15
        else:
            raise Abort("far phase: not aligned on the hold pixel -- HOLDING HERE")
        self.measure_panel(self.hold_px(), "hold ray")
        self.far_ray_base = self.ray_base.copy()
        if self.args.execute:
            time.sleep(FORCE_SETTLE_S)
            f_prev = self.force_now()
        step_i = 0
        while self.s_panel > CLOSE_STANDOFF_M:
            step = min(FAR_STEP_M, self.s_panel - CLOSE_STANDOFF_M)
            step_i += 1
            self.step(self.far_ray_base * step, f"far {step_i} (+{step*100:.1f})")
            self.far_travelled += step
            since += step
            self.s_panel -= step   # nominal; re-measured below
            if not self.args.execute:
                if step_i >= 2:
                    print("  (dry run: far steps continue until the panel is at the close standoff)")
                    return
                continue
            time.sleep(FORCE_SETTLE_S)
            f = self.force_now()
            jump = f - f_prev
            f_prev = f
            print(f"  far travelled {self.far_travelled*100:5.1f} cm  |dF| {f:5.2f} N ({jump:+.2f})  nominal panel {self.s_panel*100:.1f} cm")
            # No contact is possible out here, and 2 cm steps move joints several degrees,
            # which shifts the wrench estimate by up to ~3 N. Only a gross collision counts.
            if f > FORCE_ABORT_N:
                raise Abort(f"|dF| {f:.1f} N in the far phase -- collision? HOLDING HERE")
            if since >= FAR_RESERVO_EVERY_M:
                since = 0.0
                ok, st = self.per.locked(MIN_INLIERS_TRACK)
                if not ok:
                    # Between reference views the lock dips; the hold ray is cached and the
                    # panel distance keeps counting down nominally, so just skip this check.
                    print(f"  (weak/no lock in transit: {st} -- continuing on the cached ray)")
                    continue
                e = self.pixel_error(after=time.monotonic() - 0.2, target=self.hold_px())
                if e is not None and np.linalg.norm(e) > PX_TOL:
                    d_cam, d_base = self.lateral_correction(e, self.z_panel, RESERVO_MAX_STEP_M)
                    print(f"  hold re-servo: e=({e[0]:+.1f},{e[1]:+.1f}) px -> base {np.round(d_base*100, 2)} cm")
                    self.step(d_base, "hold re-servo")
                    time.sleep(FORCE_SETTLE_S)
                    f_prev = self.force_now()
                self.measure_panel(self.hold_px(), "hold ray")   # real distance, not nominal
                self.far_ray_base = self.ray_base.copy()
        print(f"  far phase done: panel {self.s_panel*100:.1f} cm along the hold ray after {self.far_travelled*100:.1f} cm")
        self.reanchor_seed("after the far phase")
        self.rebaseline_force("after the far phase")

    # ---- stage 1 ------------------------------------------------------------------------------
    def servo(self):
        print("\n== stage 1: lateral servo at standoff ==")
        self.measure_panel()
        moved_at = None
        for it in range(1, SERVO_MAX_ITERS + 1):
            e = self.pixel_error(after=moved_at)
            if e is None:
                raise Abort("button node stopped locking during the servo -- HOLDING HERE")
            err = float(np.linalg.norm(e))
            print(f"  iter {it}: button - claw = ({e[0]:+.1f}, {e[1]:+.1f}) px  |e| {err:.1f}")
            self.log({"stage": 1, "iter": it, "e_px": e.tolist()})
            if err < PX_TOL:
                print(f"  aligned (|e| < {PX_TOL} px)")
                break
            d_cam, d_base = self.lateral_correction(e, self.z_panel, SERVO_MAX_STEP_M)
            print(f"          correction cam {np.round(d_cam*100, 2)} cm -> base {np.round(d_base*100, 2)} cm")
            self.step(d_base, f"servo {it}")
            if not self.args.execute:
                print("  (dry run: cannot observe the effect of the correction; stopping the loop here)")
                break
            moved_at = time.monotonic() + 0.15   # only trust pixels from frames after the arm settled
        else:
            raise Abort(f"not aligned after {SERVO_MAX_ITERS} iterations -- HOLDING HERE")
        # The ray only depends on the claw pixel, but re-measure the plane now that we moved.
        if self.args.execute:
            self.measure_panel()

    # ---- stage 2 ------------------------------------------------------------------------------
    def approach(self) -> str:
        print("\n== stage 2: approach along the fingertip ray ==")
        a = self.args
        if a.tip_dist is None:
            if a.execute:
                raise Abort("--tip-dist (camera lens -> LEFT fingertip, metres, ruler) is required "
                            "to execute the approach. Over-estimate rather than under-estimate.")
            print("  (dry run without --tip-dist: cap shown for tip_dist = 0)")
            tip = 0.0
        else:
            tip = a.tip_dist
        L_max = self.s_panel - tip + PLANE_OVERSHOOT_M
        if a.cap_override is not None:
            L_max = min(L_max, a.cap_override)
        print(f"  travel cap L_max = s_panel {self.s_panel*100:.1f} - tip {tip*100:.1f} + overshoot {PLANE_OVERSHOOT_M*100:.1f}"
              f"{f' (override {a.cap_override*100:.1f})' if a.cap_override is not None else ''} = {L_max*100:.1f} cm")
        if L_max <= 0:
            raise Abort("travel cap <= 0: fingertip is already at/through the plane per these numbers")
        self.log({"stage": 2, "L_max": L_max, "tip_dist": tip, "cap_override": a.cap_override})

        self.travelled = 0.0
        since_servo = 0.0
        step_i = 0
        # The contact rule (abort / candidate / confirm) lives in contact.py so it can be
        # unit-tested against logged force sequences; this loop only moves and reports.
        mon = ApproachContactMonitor(contact_n=CONTACT_N, jump_n=JUMP_N, window=CONTACT_WINDOW,
                                     confirm_rise_n=CONFIRM_RISE_N, abort_n=FORCE_ABORT_N,
                                     coarse_abort_n=COARSE_ABORT_N)
        if a.execute:
            self.rebaseline_force("before the approach")
            f_ref = self.force_now()
            mon.start(f_ref)
            print(f"  force at rest before the approach: {f_ref:.2f} N (contact needs +{CONTACT_N} N total "
                  f"and +{JUMP_N} N within one step)")
            self.log({"stage": 2, "f_ref": f_ref})
        # Distance the fingertip must travel before it can possibly touch the panel.
        l_contact = self.s_panel - tip
        armed = False
        print(f"  contact arms at {max(0.0, l_contact - CONTACT_ARM_MARGIN_M)*100:.1f} cm travel "
              f"(panel is {l_contact*100:.1f} cm from the fingertip); below that a force jump is "
              f"posture drift, not the button (abort only above {COARSE_ABORT_N} N)")
        while True:
            remaining = L_max - self.travelled
            if remaining <= 1e-4:
                self.log({"stage": 2, "stop": "cap_reached", "travelled": self.travelled})
                print(f"  travel cap reached at {self.travelled*100:.1f} cm with no contact")
                return "cap_reached"
            if a.execute and not armed and self.travelled >= l_contact - CONTACT_ARM_MARGIN_M:
                armed = True
                # Re-zero the detector at the edge of the contact zone: every coarse step so far
                # has shifted the wrench bias, so the pre-approach reference is stale by now.
                self.rebaseline_force("entering the contact zone")
                f_ref = self.force_now()
                mon.rebaseline(f_ref)
                print(f"  contact detection ARMED at {self.travelled*100:.1f} cm; rest force {f_ref:.2f} N")
                self.log({"stage": 2, "armed_at": self.travelled, "f_ref": f_ref})
            step = a.fine_step if (armed or remaining <= FINE_ZONE_M) else APPROACH_STEP_M
            step = min(step, remaining)
            step_i += 1
            self.step(self.ray_base * step, f"approach {step_i} (+{step*100:.1f})")
            self.travelled += step
            since_servo += step
            if not a.execute:
                if step_i >= 3:
                    print(f"  (dry run: {step_i} steps planned; the real loop continues to the cap or contact)")
                    return "dry_run"
                continue
            time.sleep(FORCE_SETTLE_S)
            f = self.force_now()
            v = mon.step(f, armed)
            print(f"  travelled {self.travelled*100:5.1f} cm  |dF| at rest {f:5.2f} N"
                  f"  (+{v.rise:.2f} since start, {v.jump:+.2f} this step)")
            self.log({"stage": 2, "travelled": self.travelled, "force": f, "rise": v.rise, "jump": v.jump})
            if v.kind == "abort":
                raise Abort(f"{v.reason} (at {self.travelled*100:.1f} cm, panel "
                            f"{l_contact*100:.1f} cm from the fingertip at the start). HOLDING HERE")
            if armed:
                print(f"      (rise over the last {len(mon.hist)-1} step(s): {v.window_rise:+.2f} N)")
            if v.kind == "candidate":
                # Candidate only. Push one more fine step: a real contact keeps loading up,
                # a drifting bias does not. See CONTACT_N for the measurements behind this.
                print(f"  candidate contact at {self.travelled*100:.1f} cm ({f:.2f} N, +{v.jump:.2f} N"
                      f" in one step) -- confirming with one more {a.fine_step*1000:.0f} mm step")
                self.log({"stage": 2, "candidate": self.travelled, "force": f, "jump": v.jump})
                stp = min(a.fine_step, L_max - self.travelled)
                if stp <= 1e-4:
                    print("  travel cap reached before the candidate could be confirmed")
                    self.log({"stage": 2, "stop": "cap_before_confirm", "travelled": self.travelled})
                    return "cap_reached"
                self.step(self.ray_base * stp, f"confirm (+{stp*100:.2f})")
                self.travelled += stp
                since_servo += stp
                time.sleep(FORCE_SETTLE_S)
                f2 = self.force_now()
                print(f"    confirm: |dF| {f2:.2f} N ({f2 - f:+.2f} N since the candidate)")
                self.log({"stage": 2, "confirm": self.travelled, "force": f2, "delta": f2 - f})
                verdict = mon.confirm(f, f2)
                if verdict == "abort":
                    raise Abort(f"|dF| {f2:.1f} N while confirming -- HOLDING HERE")
                if verdict == "contact":
                    print(f"  CONTACT CONFIRMED at {self.travelled*100:.1f} cm "
                          f"({f2:.2f} N, rose {v.jump:+.2f} then {f2 - f:+.2f})")
                    self.log({"stage": 2, "stop": "contact", "travelled": self.travelled, "force": f2})
                    return "contact"
                print("  NOT confirmed -- the force did not keep rising, so that was posture "
                      "drift, not the button. Re-baselining and continuing.")
                self.rebaseline_force("rejected a phantom contact")
                mon.rebaseline(self.force_now())
            elif v.reason:
                print(f"  (level +{v.rise:.2f} N but no step jump -- treating as drift, continuing)")
            # Re-check lateral alignment while the node can still see the panel.
            if since_servo >= RESERVO_EVERY_M:
                since_servo = 0.0
                ok, _ = self.per.locked(MIN_INLIERS_TRACK)
                e = self.pixel_error(after=time.monotonic() - 0.2) if ok else None
                if e is not None and np.linalg.norm(e) > PX_TOL:
                    z_now = max(0.05, self.z_panel - self.travelled * self.ray_cam[2])
                    d_cam, d_base = self.lateral_correction(e, z_now, RESERVO_MAX_STEP_M)
                    print(f"  re-servo: e=({e[0]:+.1f},{e[1]:+.1f}) px -> base {np.round(d_base*100, 2)} cm")
                    self.step(d_base, "re-servo")
                    time.sleep(FORCE_SETTLE_S)
                    mon.note_rest(self.force_now())   # a lateral move shifts the bias too; don't count it as a jump
                elif e is None:
                    print("  (node abstaining -- continuing on the cached ray)")

    # ---- stage 3 ------------------------------------------------------------------------------
    def press(self, n_presses: int):
        print(f"\n== stage 3: press x{n_presses} ==")
        for i in range(1, n_presses + 1):
            pt = self.args.press_travel
            self.step(self.ray_base * pt, f"press {i} (+{pt*100:.1f})")
            self.travelled += pt
            if self.args.execute:
                time.sleep(PRESS_HOLD_S)
                f = self.force_now()
                print(f"  press {i}: |dF| {f:.2f} N")
                self.log({"stage": 3, "press": i, "force": f})
                if f > FORCE_ABORT_N:
                    raise Abort(f"|dF| {f:.1f} N during press -- HOLDING HERE")
            self.move_along(-self.ray_base, PRESS_RETRACT_M, f"press {i} retract")
            if i < n_presses:
                if self.args.execute:
                    time.sleep(0.5)
                # Back to contact depth: re-advance what we retracted, minus the press travel.
                self.move_along(self.ray_base, PRESS_RETRACT_M - pt, f"press {i+1} re-approach")

    # ---- stage 4 ------------------------------------------------------------------------------
    def retract(self):
        print("\n== stage 4: retract to standoff ==")
        self.move_along(-self.ray_base, self.travelled, "retract")
        if self.far_ray_base is not None and self.far_travelled > 1e-4:
            print(f"  undoing the far phase ({self.far_travelled*100:.1f} cm along the hold ray)")
            self.move_along(-self.far_ray_base, self.far_travelled, "far retract", far=True)
        if self.args.home:
            q0 = self.start_joints
            q = self.arm.joints()
            jump = max_joint_delta_deg(q0, q)
            print(f"  home: joint move back to start, max joint delta {jump:.1f} deg")
            if jump > 20.0:
                raise Abort(f"home move is {jump:.0f} deg on one joint -- use goto_preset.py deliberately instead")
            if self.args.execute:
                self.arm.ai.execute_command(JointCommand(pos=q0.tolist()))
                derr = wait_converged(self.arm.ai, q0, timeout_s=10.0)
                print(f"  home: converged to {derr:.2f} deg")

    # ---- driver -------------------------------------------------------------------------------
    def run(self):
        a = self.args
        print(f"{'EXECUTE' if a.execute else 'DRY RUN'}  stages 0..{a.stage}  target={a.target or '(node default)'}"
              f"  presses={a.presses}  log={self.log_dir}")
        try:
            self.preflight()
            if a.stage >= 1:
                self.measure_panel(self.hold_px(), "hold ray")
                if self.s_panel > CLOSE_STANDOFF_M + 0.03:
                    self.far_approach()
                self.servo()
            if a.stage >= 2:
                outcome = self.approach()
                if a.stage >= 3 and outcome in ("contact", "dry_run"):
                    self.press(a.presses)
                elif a.stage >= 3:
                    print(f"  no contact ({outcome}) -- skipping the press")
                if a.stage >= 4 and outcome != "dry_run":
                    self.retract()
                elif a.stage >= 4:
                    print("\n== stage 4: (dry run) retract would reverse the approach travel ==")
        except KeyboardInterrupt:
            print("\nCtrl-C: no further steps sent. The in-flight step (if any) completes; arm HOLDS.")
            self._way_back()
            return 130
        except Abort as e:
            print(f"\nABORT: {e}")
            self._way_back()
            return 2
        print("\ndone.")
        return 0

    def _way_back(self):
        if self.start_joints is not None:
            print(f"  start joints are in {self.log_dir / 'start_joints.json'}; travelled along ray: "
                  f"{self.travelled*100:.1f} cm (ray base {np.round(self.ray_base, 3) if self.ray_base is not None else '?'})")
            print("  To retreat: re-run with --execute --stage 4 --resume-travel "
                  f"{self.travelled:.4f} (retract only), or goto_preset.py to a saved pose.")


    # ---- utility modes (one per CLI flag; none runs the press) --------------------------------
    def goto_start(self) -> int:
        """--goto-start: joint-move back to the start joints saved by the previous run."""
        saved = json.loads((self.log_dir / "start_joints.json").read_text())
        q0 = np.asarray(saved["joints"], dtype=float)
        self.preflight(require_lock=False, require_free=False)   # saves the CURRENT joints first
        jump = max_joint_delta_deg(q0, self.arm.joints())
        print(f"\n== goto-start: max joint delta {jump:.1f} deg (saved {time.ctime(saved['t'])}) ==")
        if jump > 20.0:
            print("refusing: > 20 deg on one joint -- use goto_preset.py deliberately")
            return 2
        if not self.args.execute:
            print("dry run -- add --execute to move")
            return 0
        self.arm.ai.execute_command(JointCommand(pos=q0.tolist()))
        derr = wait_converged(self.arm.ai, q0, timeout_s=15.0)
        print(f"converged to {derr:.2f} deg")
        return 0 if derr < CONVERGE_TOL_DEG else 2

    def jog(self, dist: float) -> int:
        """--jog: advance `dist` along the fingertip ray in fine steps, to measure --tip-dist.

        No servo, no contact detection, no press. At a human-confirmed touch the fingertip
        is ON the panel, so tip_dist = (camera->panel along the ray at the start) - travel.
        Jog deliberately accepts a WEAKER lock than a press run: its geometry is the CLAW
        pixel plus the panel quad and never the button position, so a lock too weak to trust
        for "which dome is timer_clock" is still fine here. The press path keeps the full
        MIN_INLIERS gate, which exists because 6-8 inlier locks have picked the wrong dome.
        """
        a = self.args
        self.preflight(require_lock=True, require_free=True, min_inliers=MIN_INLIERS_TRACK)
        self.R_bc = self.per.cam_rotation_in_base()
        self.measure_panel()
        s0 = self.s_panel
        print(f"\n== JOG {dist*100:.1f} cm along the fingertip ray (measurement only) ==")
        print(f"  camera -> panel along the ray right now: {s0*100:.2f} cm")
        print(f"  at a human-confirmed touch:  tip_dist = {s0*100:.2f} cm - travel")
        print(f"  (the assumed tip_dist {(a.tip_dist or 0)*100:.1f} cm predicts a touch at "
              f"{(s0 - (a.tip_dist or 0))*100:.2f} cm of travel)")
        if not a.execute:
            print("  dry run -- add --execute to move")
            return 0
        self.rebaseline_force("before the jog")
        remaining, k = dist, 0
        try:
            while remaining > 1e-4:
                stp = min(a.fine_step, remaining)
                k += 1
                self.step(self.ray_base * stp, f"jog {k} (+{stp*100:.2f})")
                self.travelled += stp
                remaining -= stp
                time.sleep(FORCE_SETTLE_S)
                fo = self.force_now()
                print(f"    travel {self.travelled*100:5.2f} cm | |dF| {fo:5.2f} N | "
                      f"implied tip_dist if touching NOW = {(s0 - self.travelled)*100:5.2f} cm")
                self.log({"stage": "jog", "travelled": self.travelled, "force": fo,
                          "implied_tip_dist": s0 - self.travelled})
                if fo > FORCE_ABORT_N:
                    raise Abort(f"|dF| {fo:.1f} N during the jog -- HOLDING HERE")
        except (Abort, KeyboardInterrupt) as e:
            print(f"\nSTOPPED: {e}")
        print(f"\n  jog done: travelled {self.travelled*100:.2f} cm; "
              f"if the fingertip is touching now, --tip-dist {(s0 - self.travelled):.4f}")
        print(f"  to back out: --execute --stage 4 --resume-travel {self.travelled:.4f}")
        return 0

    def retreat(self, travel: float, far_travel: float | None) -> int:
        """--resume-travel / --resume-far-travel: retract only, after an abort.

        Recomputes the ray(s) from the claw pixel + tf, then reverses the given travel.
        Retreating is exactly what you do while the tool is still loaded, so neither the
        lock nor the free-tool check applies.
        """
        self.preflight(require_lock=False, require_free=False)
        self.R_bc = self.per.cam_rotation_in_base()
        self.ray_cam = self.per.ray(self.per.get("claw_px"))
        self.ray_base = self.R_bc @ self.ray_cam
        self.travelled = travel
        if far_travel:
            self.far_ray_base = self.R_bc @ self.per.ray(self.hold_px())
            self.far_travelled = far_travel
        try:
            self.retract()
        except Abort as e:
            print(f"\nABORT: {e}")
            return 2
        return 0


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--execute", action="store_true", help="actually move the arm (default: dry run)")
    ap.add_argument("--stage", type=int, default=4, choices=range(0, 5),
                    help="stop after this stage: 0 preflight, 1 servo, 2 approach, 3 press, 4 retract")
    ap.add_argument("--presses", type=int, default=1)
    ap.add_argument("--tip-dist", type=float, default=None,
                    help="camera lens -> LEFT fingertip distance in metres (ruler). Required to execute stage 2.")
    ap.add_argument("--cap-override", type=float, default=None,
                    help="limit the approach travel to this many metres (first tests: 0.05)")
    ap.add_argument("--press-travel", type=float, default=PRESS_TRAVEL_M,
                    help=f"extra travel (m) past first contact for the press itself (default {PRESS_TRAVEL_M})")
    ap.add_argument("--fine-step", type=float, default=FINE_STEP_M,
                    help=f"step size (m) for the last {FINE_ZONE_M*100:.0f} cm before the cap (default {FINE_STEP_M})")
    ap.add_argument("--target", default=None, help="require the node to be locked on this button name")
    ap.add_argument("--home", action="store_true", help="after retracting, joint-move back to the start joints")
    ap.add_argument("--goto-start", action="store_true",
                    help="only: joint-move back to the start joints saved by the previous run (<= 20 deg per joint)")
    ap.add_argument("--resume-far-travel", type=float, default=None,
                    help="with --stage 4 only: also undo this many metres of far-phase travel (hold ray)")
    ap.add_argument("--resume-travel", type=float, default=None,
                    help="with --stage 4 only: retract this many metres along the ray without re-approaching")
    ap.add_argument("--jog", type=float, default=None,
                    help="MEASUREMENT mode: advance this many metres along the fingertip ray in "
                         "fine steps and stop. No servo, no contact detection, no press. Used to "
                         "measure --tip-dist against a human-confirmed touch: at real contact the "
                         "fingertip is ON the panel, so tip_dist = (camera->panel along the ray at "
                         "the start) - (travel). Reports both after every step.")
    ap.add_argument("--log-dir", default=str(Path.home() / "press_logs"))
    ap.add_argument("--ns", default="/button_detector")
    ap.add_argument("--press-ns", default="/press_detector")
    ap.add_argument("--arm-frame", default="arm_base_link")
    ap.add_argument("--camera-frame", default="camera_color_optical_frame")
    return ap


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    rclpy.init()
    try:
        run = Run(args)
        if args.goto_start:
            return run.goto_start()
        if args.jog is not None:
            return run.jog(args.jog)
        if args.resume_travel is not None or args.resume_far_travel is not None:
            return run.retreat(args.resume_travel or 0.0, args.resume_far_travel)
        return run.run()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
